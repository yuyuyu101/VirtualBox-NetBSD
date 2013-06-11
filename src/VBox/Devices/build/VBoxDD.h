/* $Id: VBoxDD.h $ */
/** @file
 * Built-in drivers & devices (part 1) header.
 */

/*
 * Copyright (C) 2006-2012 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#ifndef ___build_VBoxDD_h
#define ___build_VBoxDD_h

#include <VBox/vmm/pdm.h>

RT_C_DECLS_BEGIN

/** The default BIOS logo data. */
extern const unsigned char  g_abVgaDefBiosLogo[];
/** The size of the default BIOS logo data. */
extern const unsigned       g_cbVgaDefBiosLogo;
#ifdef VBOX_WITH_EFI
/** The EFI thunk binary. */
extern const unsigned char  g_abEfiThunkBinary[];
/** The size of the EFI thunk binary. */
extern const unsigned       g_cbEfiThunkBinary;
#endif


extern const PDMDEVREG g_DevicePCI;
extern const PDMDEVREG g_DevicePciIch9;
extern const PDMDEVREG g_DevicePcArch;
extern const PDMDEVREG g_DevicePcBios;
extern const PDMDEVREG g_DevicePS2KeyboardMouse;
extern const PDMDEVREG g_DeviceI8254;
extern const PDMDEVREG g_DeviceI8259;
extern const PDMDEVREG g_DeviceHPET;
extern const PDMDEVREG g_DeviceMC146818;
extern const PDMDEVREG g_DevicePIIX3IDE;
extern const PDMDEVREG g_DeviceFloppyController;
extern const PDMDEVREG g_DeviceVga;
extern const PDMDEVREG g_DeviceVMMDev;
extern const PDMDEVREG g_DevicePCNet;
#ifdef VBOX_WITH_E1000
extern const PDMDEVREG g_DeviceE1000;
#endif
#ifdef VBOX_WITH_VIRTIO
extern const PDMDEVREG g_DeviceVirtioNet;
#endif
#ifdef VBOX_WITH_INIP
extern const PDMDEVREG g_DeviceINIP;
#endif
extern const PDMDEVREG g_DeviceICHAC97;
extern const PDMDEVREG g_DeviceSB16;
extern const PDMDEVREG g_DeviceICH6_HDA;
extern const PDMDEVREG g_DeviceAudioSniffer;
extern const PDMDEVREG g_DeviceOHCI;
extern const PDMDEVREG g_DeviceEHCI;
extern const PDMDEVREG g_DeviceACPI;
extern const PDMDEVREG g_DeviceDMA;
extern const PDMDEVREG g_DeviceFloppyController;
extern const PDMDEVREG g_DeviceSerialPort;
extern const PDMDEVREG g_DeviceParallelPort;
#ifdef VBOX_WITH_AHCI
extern const PDMDEVREG g_DeviceAHCI;
#endif
#ifdef VBOX_WITH_BUSLOGIC
extern const PDMDEVREG g_DeviceBusLogic;
#endif
extern const PDMDEVREG g_DevicePCIBridge;
extern const PDMDEVREG g_DevicePciIch9Bridge;
#ifdef VBOX_WITH_LSILOGIC
extern const PDMDEVREG g_DeviceLsiLogicSCSI;
extern const PDMDEVREG g_DeviceLsiLogicSAS;
#endif
#ifdef VBOX_WITH_EFI
extern const PDMDEVREG g_DeviceEFI;
#endif
#ifdef VBOX_WITH_PCI_PASSTHROUGH_IMPL
extern const PDMDEVREG g_DevicePciRaw;
#endif

extern const PDMDRVREG g_DrvMouseQueue;
extern const PDMDRVREG g_DrvKeyboardQueue;
extern const PDMDRVREG g_DrvBlock;
extern const PDMDRVREG g_DrvVBoxHDD;
extern const PDMDRVREG g_DrvVD;
extern const PDMDRVREG g_DrvHostDVD;
extern const PDMDRVREG g_DrvHostFloppy;
extern const PDMDRVREG g_DrvMediaISO;
extern const PDMDRVREG g_DrvRawImage;
extern const PDMDRVREG g_DrvISCSI;
extern const PDMDRVREG g_DrvISCSITransportTcp;
#if defined(RT_OS_LINUX) || defined(RT_OS_FREEBSD)
extern const PDMDRVREG g_DrvHostInterface;
#endif
#ifdef VBOX_WITH_UDPTUNNEL
extern const PDMDRVREG g_DrvUDPTunnel;
#endif
#ifdef VBOX_WITH_VDE
extern const PDMDRVREG g_DrvVDE;
#endif
extern const PDMDRVREG g_DrvIntNet;
extern const PDMDRVREG g_DrvDedicatedNic;
extern const PDMDRVREG g_DrvNAT;
#ifdef VBOX_WITH_NETSHAPER
extern const PDMDRVREG g_DrvNetShaper;
#endif /* VBOX_WITH_NETSHAPER */
extern const PDMDRVREG g_DrvNetSniffer;
extern const PDMDRVREG g_DrvAUDIO;
extern const PDMDRVREG g_DrvACPI;
extern const PDMDRVREG g_DrvAcpiCpu;
extern const PDMDRVREG g_DrvVUSBRootHub;
extern const PDMDRVREG g_DrvChar;
extern const PDMDRVREG g_DrvNamedPipe;
extern const PDMDRVREG g_DrvRawFile;
extern const PDMDRVREG g_DrvHostParallel;
extern const PDMDRVREG g_DrvHostSerial;
#ifdef VBOX_WITH_DRV_DISK_INTEGRITY
extern const PDMDRVREG g_DrvDiskIntegrity;
#endif
#ifdef VBOX_WITH_PCI_PASSTHROUGH_IMPL
extern const PDMDRVREG g_DrvPciRaw;
#endif

#ifdef VBOX_WITH_USB
extern const PDMUSBREG g_UsbDevProxy;
extern const PDMUSBREG g_UsbMsd;
#endif
#ifdef VBOX_WITH_VUSB
extern const PDMUSBREG g_UsbHid;
extern const PDMUSBREG g_UsbHidKbd;
extern const PDMUSBREG g_UsbHidMou;
#endif

#ifdef VBOX_WITH_SCSI
extern const PDMDRVREG g_DrvSCSI;
# if defined(RT_OS_LINUX)
extern const PDMDRVREG g_DrvSCSIHost;
# endif
#endif

RT_C_DECLS_END

#endif
