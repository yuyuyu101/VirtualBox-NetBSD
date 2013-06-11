/*
 * Copyright 2008 Francois Gouget for CodeWeavers
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA
 */

/*
 * Oracle LGPL Disclaimer: For the avoidance of doubt, except that if any license choice
 * other than GPL or LGPL is available it will apply instead, Oracle elects to use only
 * the Lesser General Public License version 2.1 (LGPLv2) at this time for any software where
 * a choice of LGPL license versions is made available with the language indicating
 * that LGPLv2 or any later version may be used, or where a choice of which version
 * of the LGPL is applied is otherwise unspecified.
 */

#ifndef _NTDDK_
#define _NTDDK_

/* Note: We will probably have to duplicate everything ultimately :-( */
#include <ddk/wdm.h>

#include <excpt.h>
/* FIXME: #include <ntdef.h> */
#include <ntstatus.h>
/* FIXME: #include <bugcodes.h> */
/* FIXME: #include <ntiologc.h> */


typedef enum _BUS_DATA_TYPE
{
    ConfigurationSpaceUndefined = -1,
    Cmos,
    EisaConfiguration,
    Pos,
    CbusConfiguration,
    PCIConfiguration,
    VMEConfiguration,
    NuBusConfiguration,
    PCMCIAConfiguration,
    MPIConfiguration,
    MPSAConfiguration,
    PNPISAConfiguration,
    MaximumBusDataType
} BUS_DATA_TYPE, *PBUS_DATA_TYPE;

typedef struct _CONFIGURATION_INFORMATION
{
    ULONG DiskCount;
    ULONG FloppyCount;
    ULONG CdRomCount;
    ULONG TapeCount;
    ULONG ScsiPortCount;
    ULONG SerialCount;
    ULONG ParallelCount;
    BOOLEAN AtDiskPrimaryAddressClaimed;
    BOOLEAN AtDiskSecondaryAddressClaimed;
    ULONG Version;
    ULONG MediumChangerCount;
} CONFIGURATION_INFORMATION, *PCONFIGURATION_INFORMATION;

typedef enum _CONFIGURATION_TYPE
{
    ArcSystem = 0,
    CentralProcessor,
    FloatingPointProcessor,
    PrimaryIcache,
    PrimaryDcache,
    SecondaryIcache,
    SecondaryDcache,
    SecondaryCache,
    EisaAdapter,
    TcAdapter,
    ScsiAdapter,
    DtiAdapter,
    MultiFunctionAdapter,
    DiskController,
    TapeController,
    CdromController,
    WormController,
    SerialController,
    NetworkController,
    DisplayController,
    ParallelController,
    PointerController,
    KeyboardController,
    AudioController,
    OtherController,
    DiskPeripheral,
    FloppyDiskPeripheral,
    TapePeripheral,
    ModemPeripheral,
    MonitorPeripheral,
    PrinterPeripheral,
    PointerPeripheral,
    KeyboardPeripheral,
    TerminalPeripheral,
    OtherPeripheral,
    LinePeripheral,
    NetworkPeripheral,
    SystemMemory,
    DockingInformation,
    RealModeIrqRoutingTable,
    RealModePCIEnumeration,
    MaximunType
} CONFIGURATION_TYPE, *PCONFIGURATION_TYPE;

typedef VOID (WINAPI *PDRIVER_REINITIALIZE)(PDRIVER_OBJECT,PVOID,ULONG);
typedef NTSTATUS (WINAPI *PIO_QUERY_DEVICE_ROUTINE)(PVOID,PUNICODE_STRING,INTERFACE_TYPE,ULONG,
            PKEY_VALUE_FULL_INFORMATION*,CONFIGURATION_TYPE,ULONG,PKEY_VALUE_FULL_INFORMATION*);

NTSTATUS  WINAPI IoQueryDeviceDescription(PINTERFACE_TYPE,PULONG,PCONFIGURATION_TYPE,PULONG,
                                  PCONFIGURATION_TYPE,PULONG,PIO_QUERY_DEVICE_ROUTINE,PVOID);
void      WINAPI IoRegisterDriverReinitialization(PDRIVER_OBJECT,PDRIVER_REINITIALIZE,PVOID);
NTSTATUS  WINAPI IoRegisterShutdownNotification(PDEVICE_OBJECT);

#endif
