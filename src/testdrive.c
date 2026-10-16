//================================================================================
// Copyright (c) 2013 ~ 2026. HyungKi Jeong(clonextop@gmail.com)
// Freely available under the terms of the 3-Clause BSD License
// (https://opensource.org/licenses/BSD-3-Clause)
//
// Redistribution and use in source and binary forms,
// with or without modification, are permitted provided
// that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
// OF SUCH DAMAGE.
//
// Title : QEMU for TestDrive
// Rev.  : 10/14/2026 Wed (clonextop@gmail.com)
//================================================================================
#include <stdbool.h>
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "hw/pci/pci.h"
#include "hw/pci/msi.h"
#include "hw/pci/msix.h"
#include "hw/pci/pcie.h"
#include "hw/pci/pcie_sriov.h"
#include "qapi/error.h"
#include "qemu/range.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "qemu/module.h"
#include "qapi/visitor.h"
#include "ui/console.h"
#include "system/system.h"
#include "testdrive_device.h"

#define TESTDRIVE_DEVICE_NAME "testdrive"

typedef struct {
	PCIDevice pdev;
	struct {
		MemoryRegion region;
	} mem[6];
	TESTDRIVE	   *pTestDrive;
	DisplaySurface *pSurface;
	QemuConsole	   *console;
	Notifier		exit_notifier;
	// Resizable BAR (ReBAR) state
	uint16_t		rebar_ctrl_off; // config offset of the ReBAR Control register (0 = disabled)
	int				rebar_bar_id;	// which BAR is resizable
} TESTDRIVE_DEVICE;

// ReBAR register layout (PCIe Extended Capability ID 0x15)
#ifndef PCI_EXT_CAP_ID_REBAR
#	define PCI_EXT_CAP_ID_REBAR 0x15
#endif
#define TESTDRIVE_REBAR_CAP		   0x04		  // offset of Capability register from cap header
#define TESTDRIVE_REBAR_CTRL	   0x08		  // offset of Control register from cap header
#define TESTDRIVE_REBAR_CTRL_SIZE  0x00003F00 // Control[13:8] : current BAR size index (RW)
#define TESTDRIVE_REBAR_CTRL_SHIFT 8
// Boot-time behavior of the resizable BAR:
//   >= 0 : device forces this size index at realize (2^N MB). The firmware/OS then see
//          the already-resized BAR at boot (no negotiation needed). 8 = 256MB, 5 = 32MB.
//   -1   : "BIOS way" - leave the BAR at its Lua-configured size and let the firmware or
//          guest OS perform the ReBAR negotiation by writing the Control register.
//          (Note: SeaBIOS does NOT negotiate ReBAR; needs UEFI/OVMF or a guest driver.)
#define TESTDRIVE_REBAR_BOOT_INDEX (-1)

// ---------------------------------------------------------------------------
// SR-IOV (Single Root I/O Virtualization) — PCIe Extended Capability ID 0x10
//
// When enabled, the PF (this device) advertises an SR-IOV capability. A guest
// driver that writes NumVFs + "VF Enable" (e.g. `echo N > .../sriov_numvfs`)
// makes QEMU instantiate N "testdrive-vf" PCI functions, which then appear as
// real, separate functions in the guest's `lspci` — i.e. you can see the VFs.
//
// SR-IOV requires a PCIe bus (q35 machine, device behind a pcie-root-port) and
// the ARI capability (VF routing IDs run past function 7).
//
//   TESTDRIVE_SRIOV_TOTAL_VFS : max number of VFs (TotalVFs). 0 disables SR-IOV.
#define TESTDRIVE_SRIOV_TOTAL_VFS  8
#define TESTDRIVE_SRIOV_VF_BAR_SIZE (64 * KiB) // size of each VF's BAR0 (MMIO)
// extended-config offsets of the ARI / SR-IOV capabilities. Chosen to not
// overlap the ReBAR capability at 0x100 (12 bytes). Matches the layout used by
// QEMU's igb device (ARI 0x150, SR-IOV 0x160).
#define TESTDRIVE_SRIOV_ARI_OFFSET  0x150
#define TESTDRIVE_SRIOV_CAP_OFFSET  0x160

#define TESTDRIVE_VF_DEVICE_NAME "testdrive-vf"

DECLARE_INSTANCE_CHECKER(TESTDRIVE_DEVICE, TESTDRIVE_DEV, TESTDRIVE_DEVICE_NAME)

static const MemoryRegionOps testdrive_mem_ops = {
	.read		= (uint64_t (*)(void *, hwaddr, unsigned))testdrive_bar_read,
	.write		= (void (*)(void *, hwaddr, uint64_t, unsigned))testdrive_bar_write,
	.endianness = DEVICE_LITTLE_ENDIAN,
	.valid =
		{
			.min_access_size = 1,
			.max_access_size = 4,
		},
};

void (*testdrive_interrupt_notify)(void *pdev, unsigned int iVector) = (void (*)(void *, unsigned int))msi_notify;

bool testdrive_dma_master(void *pdev, uint64_t addr, void *pBuff, uint64_t byte_size, bool bWrite)
{
	MemTxResult ret;
	if ((ret = pci_dma_rw((PCIDevice *)pdev, addr, pBuff, byte_size, (DMADirection)bWrite, MEMTXATTRS_UNSPECIFIED)) == MEMTX_OK) {
		return true;
	}

	LOGE("testdrive_dma_master : addr(0x%llX) size(%lld) write(%d) - error code(%d)", addr, byte_size, bWrite, ret);
	return false;
}

static void testdrive_display_invalidate(void *opaque)
{
	printf("invalidated display\n");
}
static void testdrive_display_text_update(void *opaque, uint32_t *text)
{
	printf("updated text\n");
}

static bool testdrive_display_update(TESTDRIVE_DEVICE *dev)
{
	TESTDRIVE_DISPLAY *pDisp = &dev->pTestDrive->display;

	if (testdrive_display(dev->pTestDrive)) {
		// qemu_console_resize(dev->console, pDisp->width, pDisp->height);
		// DisplaySurface *surface = qemu_console_surface(dev->con);
		// memcpy(surface_data(surface), dev->pTestDrive->framebuffer, 640 * 480 * 4);
		qemu_console_update(dev->console, 0, 0, pDisp->width, pDisp->height);
	}
	return true;
}

const uint32_t g_TESTDRIVE_DISPLAY_FORMAT2PIXMAN[] = {
	PIXMAN_a8r8g8b8, PIXMAN_x8r8g8b8, PIXMAN_a8b8g8r8, PIXMAN_x8b8g8r8, PIXMAN_b8g8r8a8, PIXMAN_b8g8r8x8, PIXMAN_r8g8b8a8,
	PIXMAN_r8g8b8x8, PIXMAN_r8g8b8,	  PIXMAN_b8g8r8,   PIXMAN_r5g6b5,	PIXMAN_a1r5g5b5, PIXMAN_x1r5g5b5,
};

static const GraphicHwOps ghwops = {
	.invalidate	 = testdrive_display_invalidate,
	.gfx_update	 = (bool (*)(void *))testdrive_display_update,
	.text_update = testdrive_display_text_update,
};

static void __qemu_shutdown_callback(Notifier *notifier, void *data)
{
	TESTDRIVE_DEVICE *dev = container_of(notifier, TESTDRIVE_DEVICE, exit_notifier);
	if (dev->pTestDrive) {
		testdrive_shutdown(dev->pTestDrive);
	}
}

static void testdrive_rebar_resize(TESTDRIVE_DEVICE *dev, uint64_t new_size);

static void device_realize(PCIDevice *pdev, Error **errp)
{
	TESTDRIVE_DEVICE *dev = TESTDRIVE_DEV(pdev);

	if (!(dev->pTestDrive = testdrive_create(pdev))) {
		LOGE("TestDrive device is not ready.");
		exit(1);
	}

	// register callback of qemu exit.
	dev->exit_notifier.notify = &__qemu_shutdown_callback;
	qemu_add_exit_notifier(&dev->exit_notifier);

	// setup BAR#
	for (int i = 0; i < 6; i++) {
		TESTDRIVE_PCI_BAR *bar = testdrive_param.pci.bar[i];
		if (bar) {
			uint8_t attr = bar->option.bar_type ? PCI_BASE_ADDRESS_SPACE_MEMORY : PCI_BASE_ADDRESS_SPACE_IO;
			if (bar->option.b64bit)
				attr |= PCI_BASE_ADDRESS_MEM_TYPE_64;
			if (bar->option.bPrefetchable)
				attr |= PCI_BASE_ADDRESS_MEM_PREFETCH;
			memory_region_init_io(&dev->mem[bar->option.bar_id].region, OBJECT(dev), &testdrive_mem_ops, bar, bar->name, bar->byte_size);
			pci_register_bar(pdev, bar->option.bar_id, attr, &dev->mem[bar->option.bar_id].region);
		}
	}

	// PCI Express capability.
	// Required so the guest OS recognizes this as a PCIe device and enumerates the 4KB
	// extended configuration space where the ReBAR capability lives. Without it, Linux /
	// Windows limit config space to 256 bytes and never see the extended capabilities.
	if (pci_is_express(pdev)) {
		int ret = pcie_endpoint_cap_init(pdev, 0x70);
		if (ret < 0) {
			LOGW("Failed to init PCI Express capability (%d).", ret);
		}
	}

	// setup Resizable BAR (ReBAR) extended capability.
	// Target BAR#2 (the "sample" memory BAR) : it is a memory BAR >= 1MB, which is the
	// minimum size ReBAR can represent. (BAR#0 holds 64KB registers and cannot be used.)
	dev->rebar_ctrl_off = 0;
	{
		int				   rbar = 2;
		TESTDRIVE_PCI_BAR *bar0 = testdrive_param.pci.bar[rbar];
		if (bar0 && bar0->option.bar_type) { // bar_type==1 : memory space (required for ReBAR)
			if (!pci_is_express(pdev)) {
				LOGW("ReBAR requires a PCIe bus (e.g. q35 machine). Skipped.");
			} else {
				uint16_t off	  = PCI_CONFIG_SPACE_SIZE; // 0x100 : first extended capability slot
				uint16_t cap_off  = off + TESTDRIVE_REBAR_CAP;
				uint16_t ctrl_off = off + TESTDRIVE_REBAR_CTRL;

				// one resizable BAR : header(4) + cap(4) + ctrl(4) = 12 bytes
				pcie_add_capability(pdev, PCI_EXT_CAP_ID_REBAR, 1, off, 4 + 8);

				// size index of the Lua-configured BAR size
				unsigned cfg_idx = 0;
				for (uint64_t s = bar0->byte_size; s > (1ULL << 20); s >>= 1)
					cfg_idx++;

				// If forcing a boot size, report & apply it; otherwise ("BIOS way") report
				// the configured size and let the firmware/OS negotiate ReBAR at runtime.
				int		 force_idx	  = TESTDRIVE_REBAR_BOOT_INDEX;
				unsigned reported_idx = (force_idx >= 0) ? (unsigned)force_idx : cfg_idx;

				// advertise supported sizes : 1MB(index 0) .. at least 256MB(index 8),
				// always covering the configured / reported size. Capability register
				// bit (index + 4) set means that size is supported.
				unsigned max_idx = 8;
				if (cfg_idx > max_idx)
					max_idx = cfg_idx;
				if (reported_idx > max_idx)
					max_idx = reported_idx;
				uint32_t sizes = 0;
				for (unsigned n = 0; n <= max_idx; n++)
					sizes |= 1u << (n + 4);
				pci_set_long(pdev->config + cap_off, sizes);

				uint32_t ctrl = (rbar & 0x7)			  // Control[2:0]  : BAR index
							  | ((1u & 0x7) << 5)		  // Control[7:5]  : number of resizable BARs
							  | ((reported_idx & 0x3f) << TESTDRIVE_REBAR_CTRL_SHIFT); // Control[13:8] : current size
				pci_set_long(pdev->config + ctrl_off, ctrl);

				// let the guest write only the size field
				pci_set_long(pdev->wmask + ctrl_off, TESTDRIVE_REBAR_CTRL_SIZE);

				dev->rebar_ctrl_off = ctrl_off;
				dev->rebar_bar_id	= rbar;

				// only force a device-side resize when explicitly requested; in "BIOS way"
				// mode the BAR stays at its configured size until the firmware/OS writes it.
				if (force_idx >= 0)
					testdrive_rebar_resize(dev, 1ULL << (20 + force_idx));

				LOGI("PCIe.ReBAR enabled on BAR%d : %s, size index %d (Lua cfg %d), supported 1MB..%uMB",
					 rbar, (force_idx >= 0) ? "forced boot resize" : "firmware/OS-driven", reported_idx, cfg_idx,
					 1u << max_idx);
			}
		}
	}

	// setup SR-IOV : advertise a Physical Function (PF) that can spawn VFs.
#if TESTDRIVE_SRIOV_TOTAL_VFS > 0
	if (!pci_is_express(pdev)) {
		LOGW("SR-IOV requires a PCIe bus (e.g. q35 + pcie-root-port). Skipped.");
	} else {
		uint16_t total_vfs = TESTDRIVE_SRIOV_TOTAL_VFS;
		uint16_t vf_dev_id = testdrive_param.pci.main.device_id + 1; // VF uses PF id + 1

		// ARI is required : VF routing IDs run beyond function number 7.
		pcie_ari_init(pdev, TESTDRIVE_SRIOV_ARI_OFFSET);

		Error *local_err = NULL;
		if (!pcie_sriov_pf_init(pdev, TESTDRIVE_SRIOV_CAP_OFFSET, TESTDRIVE_VF_DEVICE_NAME, vf_dev_id,
								/*init_vfs*/ total_vfs, /*total_vfs*/ total_vfs,
								/*vf_offset*/ 1, /*vf_stride*/ 1, &local_err)) {
			LOGW("Failed to init SR-IOV capability : %s", local_err ? error_get_pretty(local_err) : "unknown");
			error_free(local_err);
		} else {
			// declare the VF BAR layout the guest will program (VF BAR0, 64-bit MMIO).
			// Each VF's actual MemoryRegion is created in testdrive_vf_realize().
			pcie_sriov_pf_init_vf_bar(pdev, 0, PCI_BASE_ADDRESS_MEM_TYPE_64 | PCI_BASE_ADDRESS_MEM_PREFETCH,
									  TESTDRIVE_SRIOV_VF_BAR_SIZE);
			LOGI("PCIe.SR-IOV enabled : TotalVFs %d, VF device id 0x%04X, VF BAR0 size 0x%llX", total_vfs, vf_dev_id,
				 (unsigned long long)TESTDRIVE_SRIOV_VF_BAR_SIZE);
		}
	}
#endif

	// setup MSI/MSI-X
	if (testdrive_param.pci.interrupt.vector_count) {
		if (testdrive_param.pci.interrupt.extend.bEnable) {
			MemoryRegion *table_bar = &dev->mem[testdrive_param.pci.interrupt.extend.iBar].region;
			//@TODO: MSI-X
			/*
			msix_init(
				pdev, testdrive_param.pci.interrupt.vector_count, table_bar, testdrive_param.pci.interrupt.extend.iBar,
			testdrive_param.pci.interrupt.extend.iOffset, NULL, 0, 0,0x60,NULL);
			*/
			testdrive_interrupt_notify = (void (*)(void *, unsigned int))msix_notify;
		} else {
			msi_init(pdev, 0x50, testdrive_param.pci.interrupt.vector_count, true, testdrive_param.pci.interrupt.per_vector_mask, NULL);
		}
	}

	// enable bus master : but some guest OS(linux) will turn off this.
	// pci_default_write_config(pdev, PCI_COMMAND, pci_get_word(pdev->config + PCI_COMMAND) | PCI_COMMAND_MASTER, 2);

	if (dev->pTestDrive->display.pBuffer) {
		TESTDRIVE_DISPLAY *pDisp = &dev->pTestDrive->display;
		dev->console			 = qemu_graphic_console_create(DEVICE(pdev), 0, &ghwops, dev);
		qemu_console_resize(dev->console, pDisp->width, pDisp->height);
		dev->pSurface = qemu_create_displaysurface_from(
			pDisp->width, pDisp->height, g_TESTDRIVE_DISPLAY_FORMAT2PIXMAN[pDisp->format], pDisp->byte_stride, (uint8_t *)pDisp->pBuffer);
		qemu_console_set_surface(dev->console, dev->pSurface);
	}
}

// Physically resize BAR#rebar_bar_id to new_size and re-map it.
static void testdrive_rebar_resize(TESTDRIVE_DEVICE *dev, uint64_t new_size)
{
	PCIDevice	 *pdev = &dev->pdev;
	int			  bar  = dev->rebar_bar_id;
	MemoryRegion *mr   = &dev->mem[bar].region;

	if (new_size == memory_region_size(mr))
		return;

	// 1. resize the backing memory region
	memory_region_transaction_begin();
	memory_region_set_size(mr, new_size);
	memory_region_transaction_commit();

	// 2. update PCI BAR bookkeeping (used to compute the mapped window length)
	pdev->io_regions[bar].size = new_size;

	// 3. recompute the writable address bits of the BAR register for the new alignment
	int		 off   = PCI_BASE_ADDRESS_0 + bar * 4;
	uint64_t wmask = ~(new_size - 1);
	if (pdev->io_regions[bar].type & PCI_BASE_ADDRESS_MEM_TYPE_64)
		pci_set_quad(pdev->wmask + off, wmask);
	else
		pci_set_long(pdev->wmask + off, (uint32_t)wmask);

	// 4. force a re-mapping with the new size by re-writing the BAR register(s)
	if (pdev->io_regions[bar].type & PCI_BASE_ADDRESS_MEM_TYPE_64) {
		uint64_t barval = pci_get_quad(pdev->config + off);
		pci_default_write_config(pdev, off, (uint32_t)barval, 4);
		pci_default_write_config(pdev, off + 4, (uint32_t)(barval >> 32), 4);
	} else {
		uint32_t barval = pci_get_long(pdev->config + off);
		pci_default_write_config(pdev, off, barval, 4);
	}

	LOGI("PCIe.ReBAR: BAR%d resized to 0x%llX bytes", bar, (unsigned long long)new_size);
}

static void testdrive_config_write(PCIDevice *pdev, uint32_t addr, uint32_t val, int len)
{
	TESTDRIVE_DEVICE *dev = TESTDRIVE_DEV(pdev);

	pci_default_write_config(pdev, addr, val, len);

	// intercept guest writes to the ReBAR Control register's size field
	if (dev->rebar_ctrl_off && ranges_overlap(addr, len, dev->rebar_ctrl_off, 4)) {
		uint32_t ctrl	  = pci_get_long(pdev->config + dev->rebar_ctrl_off);
		unsigned size_idx = (ctrl & TESTDRIVE_REBAR_CTRL_SIZE) >> TESTDRIVE_REBAR_CTRL_SHIFT;
		testdrive_rebar_resize(dev, 1ULL << (20 + size_idx));
	}
}

static void device_exit(PCIDevice *pdev)
{
	TESTDRIVE_DEVICE *dev = TESTDRIVE_DEV(pdev);
#if TESTDRIVE_SRIOV_TOTAL_VFS > 0
	if (pci_is_express(pdev)) {
		pcie_sriov_pf_exit(pdev);
	}
#endif
	if (testdrive_param.pci.interrupt.vector_count) {
		if (testdrive_param.pci.interrupt.extend.bEnable) {
			//@TODO: MSI-X
			// msix_uninit(pdev);
		} else {
			msi_uninit(pdev);
		}
	}
	if (dev->pTestDrive) {
		testdrive_destroy(dev->pTestDrive);
		dev->pTestDrive = NULL;
	}
}

static void device_class_init(ObjectClass *class, const void *data)
{
	PCIDeviceClass *dc		= PCI_DEVICE_CLASS(class);
	dc->realize				= device_realize;
	dc->exit				= device_exit;
	dc->config_write		= testdrive_config_write;
	dc->vendor_id			= testdrive_param.pci.main.vendor_id;
	dc->device_id			= testdrive_param.pci.main.device_id;
	dc->class_id			= testdrive_param.pci.class_id;
	dc->revision			= testdrive_param.pci.revision;
	dc->subsystem_vendor_id = testdrive_param.pci.sub.vendor_id;
	dc->subsystem_id		= testdrive_param.pci.sub.device_id;
	dc->romfile				= testdrive_param.pci.rom_file;
}

// ---------------------------------------------------------------------------
// SR-IOV Virtual Function (VF) device : "testdrive-vf"
//
// This is a minimal PCI function that is instantiated by QEMU's SR-IOV core
// (not by the user) whenever the guest enables VFs on the PF. Each instance
// shows up as its own function in the guest's `lspci`, which is what makes the
// VFs directly visible. It exposes a single dummy MMIO BAR0.
// ---------------------------------------------------------------------------
#if TESTDRIVE_SRIOV_TOTAL_VFS > 0

typedef struct {
	PCIDevice	 pdev;
	MemoryRegion mmio;
} TESTDRIVE_VF;

DECLARE_INSTANCE_CHECKER(TESTDRIVE_VF, TESTDRIVE_VF_DEV, TESTDRIVE_VF_DEVICE_NAME)

static uint64_t testdrive_vf_read(void *opaque, hwaddr addr, unsigned size)
{
	return 0; // dummy : real VF behaviour would be wired here
}

static void testdrive_vf_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
	// dummy : discard writes
}

static const MemoryRegionOps testdrive_vf_mem_ops = {
	.read		= testdrive_vf_read,
	.write		= testdrive_vf_write,
	.endianness = DEVICE_LITTLE_ENDIAN,
	.valid =
		{
			.min_access_size = 1,
			.max_access_size = 4,
		},
};

static void testdrive_vf_realize(PCIDevice *pdev, Error **errp)
{
	TESTDRIVE_VF *vf = TESTDRIVE_VF_DEV(pdev);

	// BAR0 : must match the layout declared on the PF via pcie_sriov_pf_init_vf_bar().
	memory_region_init_io(&vf->mmio, OBJECT(pdev), &testdrive_vf_mem_ops, vf, "testdrive-vf-mmio",
						  TESTDRIVE_SRIOV_VF_BAR_SIZE);
	pci_register_bar(pdev, 0, PCI_BASE_ADDRESS_MEM_TYPE_64 | PCI_BASE_ADDRESS_MEM_PREFETCH, &vf->mmio);

	if (pcie_endpoint_cap_init(pdev, 0x70) < 0) {
		error_setg(errp, "testdrive-vf : failed to init PCIe capability");
		return;
	}
	pcie_ari_init(pdev, 0x100); // VF ARI (first extended-cap slot)
}

static void testdrive_vf_class_init(ObjectClass *class, const void *data)
{
	DeviceClass	   *dc = DEVICE_CLASS(class);
	PCIDeviceClass *c  = PCI_DEVICE_CLASS(class);

	c->realize	 = testdrive_vf_realize;
	c->vendor_id = testdrive_param.pci.main.vendor_id;
	c->device_id = testdrive_param.pci.main.device_id + 1; // PF id + 1
	c->revision	 = testdrive_param.pci.revision;
	c->class_id	 = testdrive_param.pci.class_id;

	// VFs are created by the SR-IOV core, never directly by the user.
	dc->user_creatable = false;
}

#endif // TESTDRIVE_SRIOV_TOTAL_VFS > 0

static void testdrive_device_register(void)
{
	if (testdrive_init()) {
		static InterfaceInfo interfaces[] = {
			{INTERFACE_PCIE_DEVICE},
			{NULL},
		};

		static const TypeInfo device_info = {
			.name		   = TESTDRIVE_DEVICE_NAME,
			.parent		   = TYPE_PCI_DEVICE,
			.instance_size = sizeof(TESTDRIVE_DEVICE),
			.class_init	   = device_class_init,
			.interfaces	   = interfaces,
		};

		type_register_static(&device_info);

#if TESTDRIVE_SRIOV_TOTAL_VFS > 0
		static const TypeInfo vf_info = {
			.name		   = TESTDRIVE_VF_DEVICE_NAME,
			.parent		   = TYPE_PCI_DEVICE,
			.instance_size = sizeof(TESTDRIVE_VF),
			.class_init	   = testdrive_vf_class_init,
			.interfaces	   = interfaces,
		};

		type_register_static(&vf_info);
#endif
	} else {
		LOGW("TestDrive device is need to run from TestDrive Profiling Master environment.");
	}
}
// constructor
type_init(testdrive_device_register)
