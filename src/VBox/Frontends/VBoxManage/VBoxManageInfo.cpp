/* $Id: VBoxManageInfo.cpp $ */
/** @file
 * VBoxManage - The 'showvminfo' command and helper routines.
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

#ifndef VBOX_ONLY_DOCS

/*******************************************************************************
*   Header Files                                                               *
*******************************************************************************/
#include <VBox/com/com.h>
#include <VBox/com/string.h>
#include <VBox/com/Guid.h>
#include <VBox/com/array.h>
#include <VBox/com/ErrorInfo.h>
#include <VBox/com/errorprint.h>

#include <VBox/com/VirtualBox.h>

#ifdef VBOX_WITH_PCI_PASSTHROUGH
#include <VBox/pci.h>
#endif

#include <VBox/log.h>
#include <iprt/stream.h>
#include <iprt/time.h>
#include <iprt/string.h>
#include <iprt/getopt.h>
#include <iprt/ctype.h>

#include "VBoxManage.h"
using namespace com;


// funcs
///////////////////////////////////////////////////////////////////////////////

HRESULT showSnapshots(ComPtr<ISnapshot> &rootSnapshot,
                      ComPtr<ISnapshot> &currentSnapshot,
                      VMINFO_DETAILS details,
                      const Utf8Str &prefix /* = ""*/,
                      int level /*= 0*/)
{
    /* start with the root */
    Bstr name;
    Bstr uuid;
    Bstr description;
    CHECK_ERROR2_RET(rootSnapshot, COMGETTER(Name)(name.asOutParam()), hrcCheck);
    CHECK_ERROR2_RET(rootSnapshot, COMGETTER(Id)(uuid.asOutParam()), hrcCheck);
    CHECK_ERROR2_RET(rootSnapshot, COMGETTER(Description)(description.asOutParam()), hrcCheck);
    bool fCurrent = (rootSnapshot == currentSnapshot);
    if (details == VMINFO_MACHINEREADABLE)
    {
        /* print with hierarchical numbering */
        RTPrintf("SnapshotName%s=\"%ls\"\n", prefix.c_str(), name.raw());
        RTPrintf("SnapshotUUID%s=\"%s\"\n", prefix.c_str(), Utf8Str(uuid).c_str());
        if (!description.isEmpty())
            RTPrintf("SnapshotDescription%s=\"%ls\"\n", prefix.c_str(), description.raw());
        if (fCurrent)
        {
            RTPrintf("CurrentSnapshotName=\"%ls\"\n", name.raw());
            RTPrintf("CurrentSnapshotUUID=\"%s\"\n", Utf8Str(uuid).c_str());
            RTPrintf("CurrentSnapshotNode=\"SnapshotName%s\"\n", prefix.c_str());
        }
    }
    else
    {
        /* print with indentation */
        RTPrintf("   %sName: %ls (UUID: %s)%s\n",
                 prefix.c_str(),
                 name.raw(),
                 Utf8Str(uuid).c_str(),
                 (fCurrent) ? " *" : "");
        if (!description.isEmpty())
            RTPrintf("   %sDescription:\n%ls\n", prefix.c_str(), description.raw());
    }

    /* get the children */
    HRESULT hrc = S_OK;
    SafeIfaceArray <ISnapshot> coll;
    CHECK_ERROR2_RET(rootSnapshot,COMGETTER(Children)(ComSafeArrayAsOutParam(coll)), hrcCheck);
    if (!coll.isNull())
    {
        for (size_t index = 0; index < coll.size(); ++index)
        {
            ComPtr<ISnapshot> snapshot = coll[index];
            if (snapshot)
            {
                Utf8Str newPrefix;
                if (details == VMINFO_MACHINEREADABLE)
                    newPrefix = Utf8StrFmt("%s-%d", prefix.c_str(), index + 1);
                else
                {
                    newPrefix = Utf8StrFmt("%s   ", prefix.c_str());
                }

                /* recursive call */
                HRESULT hrc2 = showSnapshots(snapshot, currentSnapshot, details, newPrefix, level + 1);
                if (FAILED(hrc2))
                    hrc = hrc2;
            }
        }
    }
    return hrc;
}

static void makeTimeStr(char *s, int cb, int64_t millies)
{
    RTTIME t;
    RTTIMESPEC ts;

    RTTimeSpecSetMilli(&ts, millies);

    RTTimeExplode(&t, &ts);

    RTStrPrintf(s, cb, "%04d/%02d/%02d %02d:%02d:%02d UTC",
                        t.i32Year, t.u8Month, t.u8MonthDay,
                        t.u8Hour, t.u8Minute, t.u8Second);
}

const char *machineStateToName(MachineState_T machineState, bool fShort)
{
    switch (machineState)
    {
        case MachineState_PoweredOff:
            return fShort ? "poweroff"             : "powered off";
        case MachineState_Saved:
            return "saved";
        case MachineState_Aborted:
            return "aborted";
        case MachineState_Teleported:
            return "teleported";
        case MachineState_Running:
            return "running";
        case MachineState_Paused:
            return "paused";
        case MachineState_Stuck:
            return fShort ? "gurumeditation"       : "guru meditation";
        case MachineState_LiveSnapshotting:
            return fShort ? "livesnapshotting"     : "live snapshotting";
        case MachineState_Teleporting:
            return "teleporting";
        case MachineState_Starting:
            return "starting";
        case MachineState_Stopping:
            return "stopping";
        case MachineState_Saving:
            return "saving";
        case MachineState_Restoring:
            return "restoring";
        case MachineState_TeleportingPausedVM:
            return fShort ? "teleportingpausedvm"  : "teleporting paused vm";
        case MachineState_TeleportingIn:
            return fShort ? "teleportingin"        : "teleporting (incoming)";
        case MachineState_RestoringSnapshot:
            return fShort ? "restoringsnapshot"    : "restoring snapshot";
        case MachineState_DeletingSnapshot:
            return fShort ? "deletingsnapshot"     : "deleting snapshot";
        case MachineState_DeletingSnapshotOnline:
            return fShort ? "deletingsnapshotlive" : "deleting snapshot live";
        case MachineState_DeletingSnapshotPaused:
            return fShort ? "deletingsnapshotlivepaused" : "deleting snapshot live paused";
        case MachineState_SettingUp:
            return fShort ? "settingup"           : "setting up";
        default:
            break;
    }
    return "unknown";
}

const char *facilityStateToName(AdditionsFacilityStatus_T faStatus, bool fShort)
{
    switch (faStatus)
    {
        case AdditionsFacilityStatus_Inactive:
            return fShort ? "inactive" : "not active";
        case AdditionsFacilityStatus_Paused:
            return "paused";
        case AdditionsFacilityStatus_PreInit:
            return fShort ? "preinit" : "pre-initializing";
        case AdditionsFacilityStatus_Init:
            return fShort ? "init"    : "initializing";
        case AdditionsFacilityStatus_Active:
            return fShort ? "active"  : "active/running";
        case AdditionsFacilityStatus_Terminating:
            return "terminating";
        case AdditionsFacilityStatus_Terminated:
            return "terminated";
        case AdditionsFacilityStatus_Failed:
            return "failed";
        case AdditionsFacilityStatus_Unknown:
        default:
            break;
    }
    return "unknown";
}

/**
 * This takes care of escaping double quotes and slashes that the string might
 * contain.
 *
 * @param   pszName             The variable name.
 * @param   pbstrValue          The value.
 */
static void outputMachineReadableString(const char *pszName, Bstr const *pbstrValue)
{
    Assert(strpbrk(pszName, "\"\\") == NULL);

    com::Utf8Str strValue(*pbstrValue);
    if (    strValue.isEmpty()
        || (   !strValue.count('"')
            && !strValue.count('\\')))
        RTPrintf("%s=\"%s\"\n", pszName, strValue.c_str());
    else
    {
        /* The value needs escaping. */
        RTPrintf("%s=\"", pszName);
        const char *psz = strValue.c_str();
        for (;;)
        {
            const char *pszNext = strpbrk(psz, "\"\\");
            if (!pszNext)
            {
                RTPrintf("%s", psz);
                break;
            }
            RTPrintf("%.*s\\%c", pszNext - psz, psz, *pszNext);
            psz = pszNext + 1;
        }
        RTPrintf("\"\n");
    }
}

/**
 * Converts bandwidth group type to a string.
 * @returns String representation.
 * @param   enmType         Bandwidth control group type.
 */
inline const char * bwGroupTypeToString(BandwidthGroupType_T enmType)
{
    switch (enmType)
    {
        case BandwidthGroupType_Disk:    return "Disk";
        case BandwidthGroupType_Network: return "Network";
    }
    return "unknown";
}

HRESULT showBandwidthGroups(ComPtr<IBandwidthControl> &bwCtrl,
                            VMINFO_DETAILS details)
{
    int rc = S_OK;
    SafeIfaceArray<IBandwidthGroup> bwGroups;

    CHECK_ERROR_RET(bwCtrl, GetAllBandwidthGroups(ComSafeArrayAsOutParam(bwGroups)), rc);

    if (bwGroups.size() && details != VMINFO_MACHINEREADABLE)
        RTPrintf("\n\n");
    for (size_t i = 0; i < bwGroups.size(); i++)
    {
        Bstr strName;
        LONG64 cMaxBytesPerSec;
        BandwidthGroupType_T enmType;

        CHECK_ERROR_RET(bwGroups[i], COMGETTER(Name)(strName.asOutParam()), rc);
        CHECK_ERROR_RET(bwGroups[i], COMGETTER(Type)(&enmType), rc);
        CHECK_ERROR_RET(bwGroups[i], COMGETTER(MaxBytesPerSec)(&cMaxBytesPerSec), rc);

        const char *pszType = bwGroupTypeToString(enmType);
        if (details == VMINFO_MACHINEREADABLE)
            RTPrintf("BandwidthGroup%zu=%ls,%s,%lld\n", i, strName.raw(), pszType, cMaxBytesPerSec);
        else
        {
            const char *pszUnits = "";
            LONG64 cBytes = cMaxBytesPerSec;
            if (cBytes == 0)
            {
                RTPrintf("Name: '%ls', Type: %s, Limit: none (disabled)\n", strName.raw(), pszType);
                continue;
            }
            else if (!(cBytes % _1G))
            {
                pszUnits = "G";
                cBytes /= _1G;
            }
            else if (!(cBytes % _1M))
            {
                pszUnits = "M";
                cBytes /= _1M;
            }
            else if (!(cBytes % _1K))
            {
                pszUnits = "K";
                cBytes /= _1K;
            }
            const char *pszNetUnits = NULL;
            if (enmType == BandwidthGroupType_Network)
            {
                /*
                 * We want to report network rate limit in bits/s, not bytes.
                 * Only if it cannot be express it in kilobits we will fall
                 * back to reporting it in bytes.
                 */
                LONG64 cBits = cMaxBytesPerSec;
                if (!(cBits % 125))
                {
                    cBits /= 125;
                    pszNetUnits = "k";
                    if (!(cBits % 1000000))
                    {
                        cBits /= 1000000;
                        pszNetUnits = "g";
                    }
                    else if (!(cBits % 1000))
                    {
                        cBits /= 1000;
                        pszNetUnits = "m";
                    }
                    RTPrintf("Name: '%ls', Type: %s, Limit: %lld %sbits/sec (%lld %sbytes/sec)\n", strName.raw(), pszType, cBits, pszNetUnits, cBytes, pszUnits);
                }
            }
            if (!pszNetUnits)
                RTPrintf("Name: '%ls', Type: %s, Limit: %lld %sbytes/sec\n", strName.raw(), pszType, cBytes, pszUnits);
        }
    }
    if (details != VMINFO_MACHINEREADABLE)
        RTPrintf(bwGroups.size() != 0 ? "\n" : "<none>\n\n");

    return rc;
}


/* Disable global optimizations for MSC 8.0/64 to make it compile in reasonable
   time. MSC 7.1/32 doesn't have quite as much trouble with it, but still
   sufficient to qualify for this hack as well since this code isn't performance
   critical and probably won't gain much from the extra optimizing in real life. */
#if defined(_MSC_VER)
# pragma optimize("g", off)
#endif

HRESULT showVMInfo(ComPtr<IVirtualBox> virtualBox,
                   ComPtr<IMachine> machine,
                   VMINFO_DETAILS details /*= VMINFO_NONE*/,
                   ComPtr<IConsole> console /*= ComPtr <IConsole> ()*/)
{
    HRESULT rc;

#define SHOW_BOOLEAN_PROP(a_pObj, a_Prop, a_szMachine, a_szHuman) \
    SHOW_BOOLEAN_PROP_EX(a_pObj, a_Prop, a_szMachine, a_szHuman, "on", "off")

#define SHOW_BOOLEAN_PROP_EX(a_pObj, a_Prop, a_szMachine, a_szHuman, a_szTrue, a_szFalse) \
    do \
    { \
        BOOL f; \
        CHECK_ERROR2_RET(a_pObj, COMGETTER(a_Prop)(&f), hrcCheck); \
        if (details == VMINFO_MACHINEREADABLE) \
            RTPrintf( a_szMachine "=\"%s\"\n", f ? "on" : "off"); \
        else \
            RTPrintf("%-16s %s\n", a_szHuman ":", f ? a_szTrue : a_szFalse); \
    } while (0)

#define SHOW_BOOLEAN_METHOD(a_pObj, a_Invocation, a_szMachine, a_szHuman) \
    do \
    { \
        BOOL f; \
        CHECK_ERROR2_RET(a_pObj, a_Invocation, hrcCheck); \
        if (details == VMINFO_MACHINEREADABLE) \
            RTPrintf( a_szMachine "=\"%s\"\n", f ? "on" : "off"); \
        else \
            RTPrintf("%-16s %s\n", a_szHuman ":", f ? "on" : "off"); \
    } while (0)

#define SHOW_STRING_PROP(a_pObj, a_Prop, a_szMachine, a_szHuman) \
    do \
    { \
        Bstr bstr; \
        CHECK_ERROR2_RET(a_pObj, COMGETTER(a_Prop)(bstr.asOutParam()), hrcCheck); \
        if (details == VMINFO_MACHINEREADABLE) \
            outputMachineReadableString(a_szMachine, &bstr); \
        else \
            RTPrintf("%-16s %ls\n", a_szHuman ":", bstr.raw()); \
    } while (0)

#define SHOW_STRINGARRAY_PROP(a_pObj, a_Prop, a_szMachine, a_szHuman) \
    do \
    { \
        SafeArray<BSTR> array; \
        CHECK_ERROR2_RET(a_pObj, COMGETTER(a_Prop)(ComSafeArrayAsOutParam(array)), hrcCheck); \
        Utf8Str str; \
        for (size_t i = 0; i < array.size(); i++) \
        { \
            if (i != 0) \
                str.append(","); \
            str.append(Utf8Str(array[i]).c_str()); \
        } \
        Bstr bstr(str); \
        if (details == VMINFO_MACHINEREADABLE) \
            outputMachineReadableString(a_szMachine, &bstr); \
        else \
            RTPrintf("%-16s %ls\n", a_szHuman ":", bstr.raw()); \
    } while (0)

#define SHOW_UUID_PROP(a_pObj, a_Prop, a_szMachine, a_szHuman) \
    SHOW_STRING_PROP(a_pObj, a_Prop, a_szMachine, a_szHuman)

#define SHOW_ULONG_PROP(a_pObj, a_Prop, a_szMachine, a_szHuman, a_szUnit) \
    do \
    { \
        ULONG u32; \
        CHECK_ERROR2_RET(a_pObj, COMGETTER(a_Prop)(&u32), hrcCheck); \
        if (details == VMINFO_MACHINEREADABLE) \
            RTPrintf(a_szMachine "=%u\n", u32); \
        else \
            RTPrintf("%-16s %u" a_szUnit "\n", a_szHuman ":", u32); \
    } while (0)

#define SHOW_LONG64_PROP(a_pObj, a_Prop, a_szMachine, a_szHuman, a_szUnit) \
    do \
    { \
        LONG64 i64; \
        CHECK_ERROR2_RET(a_pObj, COMGETTER(a_Prop)(&i64), hrcCheck); \
        if (details == VMINFO_MACHINEREADABLE) \
            RTPrintf(a_szHuman "=%lld", i64); \
        else \
            RTPrintf("%-16s %'lld" a_szUnit "\n", a_szHuman ":", i64); \
    } while (0)

    /*
     * The rules for output in -argdump format:
     * 1) the key part (the [0-9a-zA-Z_\-]+ string before the '=' delimiter)
     *    is all lowercase for "VBoxManage modifyvm" parameters. Any
     *    other values printed are in CamelCase.
     * 2) strings (anything non-decimal) are printed surrounded by
     *    double quotes '"'. If the strings themselves contain double
     *    quotes, these characters are escaped by '\'. Any '\' character
     *    in the original string is also escaped by '\'.
     * 3) numbers (containing just [0-9\-]) are written out unchanged.
     */

    BOOL fAccessible;
    CHECK_ERROR2_RET(machine, COMGETTER(Accessible)(&fAccessible), hrcCheck);
    if (!fAccessible)
    {
        Bstr uuid;
        machine->COMGETTER(Id)(uuid.asOutParam());
        if (details == VMINFO_COMPACT)
            RTPrintf("\"<inaccessible>\" {%s}\n", Utf8Str(uuid).c_str());
        else
        {
            if (details == VMINFO_MACHINEREADABLE)
                RTPrintf("name=\"<inaccessible>\"\n");
            else
                RTPrintf("Name:            <inaccessible!>\n");
            if (details == VMINFO_MACHINEREADABLE)
                RTPrintf("UUID=\"%s\"\n", Utf8Str(uuid).c_str());
            else
                RTPrintf("UUID:            %s\n", Utf8Str(uuid).c_str());
            if (details != VMINFO_MACHINEREADABLE)
            {
                Bstr settingsFilePath;
                rc = machine->COMGETTER(SettingsFilePath)(settingsFilePath.asOutParam());
                RTPrintf("Config file:     %ls\n", settingsFilePath.raw());
                ComPtr<IVirtualBoxErrorInfo> accessError;
                rc = machine->COMGETTER(AccessError)(accessError.asOutParam());
                RTPrintf("Access error details:\n");
                ErrorInfo ei(accessError);
                GluePrintErrorInfo(ei);
                RTPrintf("\n");
            }
        }
        return S_OK;
    }

    if (details == VMINFO_COMPACT)
    {
        Bstr machineName;
        machine->COMGETTER(Name)(machineName.asOutParam());
        Bstr uuid;
        machine->COMGETTER(Id)(uuid.asOutParam());

        RTPrintf("\"%ls\" {%s}\n", machineName.raw(), Utf8Str(uuid).c_str());
        return S_OK;
    }

    SHOW_STRING_PROP(      machine, Name,                       "name",                 "Name");

    Bstr osTypeId;
    CHECK_ERROR2_RET(machine, COMGETTER(OSTypeId)(osTypeId.asOutParam()), hrcCheck);
    ComPtr<IGuestOSType> osType;
    CHECK_ERROR2_RET(virtualBox, GetGuestOSType(osTypeId.raw(), osType.asOutParam()), hrcCheck);
    SHOW_STRINGARRAY_PROP( machine, Groups,                     "groups",               "Groups");
    SHOW_STRING_PROP(       osType, Description,                "ostype",               "Guest OS");
    SHOW_UUID_PROP(        machine, Id,                         "UUID",                 "UUID");
    SHOW_STRING_PROP(      machine, SettingsFilePath,           "CfgFile",              "Config file");
    SHOW_STRING_PROP(      machine, SnapshotFolder,             "SnapFldr",             "Snapshot folder");
    SHOW_STRING_PROP(      machine, LogFolder,                  "LogFldr",              "Log folder");
    SHOW_UUID_PROP(        machine, HardwareUUID,               "hardwareuuid",         "Hardware UUID");
    SHOW_ULONG_PROP(       machine, MemorySize,                 "memory",               "Memory size",      "MB");
    SHOW_BOOLEAN_PROP(     machine, PageFusionEnabled,          "pagefusion",           "Page Fusion");
    SHOW_ULONG_PROP(       machine, VRAMSize,                   "vram",                 "VRAM size",        "MB");
    SHOW_ULONG_PROP(       machine, CPUExecutionCap,            "cpuexecutioncap",      "CPU exec cap",     "%%");
    SHOW_BOOLEAN_PROP(     machine, HPETEnabled,                "hpet",                 "HPET");

    ChipsetType_T chipsetType;
    CHECK_ERROR2_RET(machine, COMGETTER(ChipsetType)(&chipsetType), hrcCheck);
    const char *pszChipsetType;
    switch (chipsetType)
    {
        case ChipsetType_Null:  pszChipsetType = "invalid"; break;
        case ChipsetType_PIIX3: pszChipsetType = "piix3"; break;
        case ChipsetType_ICH9:  pszChipsetType = "ich9"; break;
        default:                AssertFailed(); pszChipsetType = "unknown"; break;
    }
    if (details == VMINFO_MACHINEREADABLE)
        RTPrintf("chipset=\"%s\"\n", pszChipsetType);
    else
        RTPrintf("Chipset:         %s\n", pszChipsetType);

    FirmwareType_T firmwareType;
    CHECK_ERROR2_RET(machine, COMGETTER(FirmwareType)(&firmwareType), hrcCheck);
    const char *pszFirmwareType;
    switch (firmwareType)
    {
        case FirmwareType_BIOS:     pszFirmwareType = "BIOS"; break;
        case FirmwareType_EFI:      pszFirmwareType = "EFI"; break;
        case FirmwareType_EFI32:    pszFirmwareType = "EFI32"; break;
        case FirmwareType_EFI64:    pszFirmwareType = "EFI64"; break;
        case FirmwareType_EFIDUAL:  pszFirmwareType = "EFIDUAL"; break;
        default:                    AssertFailed(); pszFirmwareType = "unknown"; break;
    }
    if (details == VMINFO_MACHINEREADABLE)
        RTPrintf("firmware=\"%s\"\n", pszFirmwareType);
    else
        RTPrintf("Firmware:        %s\n", pszFirmwareType);

    SHOW_ULONG_PROP(       machine, CPUCount,                   "cpus",                 "Number of CPUs", "");
    SHOW_BOOLEAN_METHOD(   machine, GetCPUProperty(CPUPropertyType_Synthetic, &f), "synthcpu", "Synthetic Cpu");

    if (details != VMINFO_MACHINEREADABLE)
        RTPrintf("CPUID overrides: ");
    ULONG cFound = 0;
    static uint32_t const s_auCpuIdRanges[] =
    {
        UINT32_C(0x00000000), UINT32_C(0x0000000a),
        UINT32_C(0x80000000), UINT32_C(0x8000000a)
    };
    for (unsigned i = 0; i < RT_ELEMENTS(s_auCpuIdRanges); i += 2)
        for (uint32_t uLeaf = s_auCpuIdRanges[i]; uLeaf < s_auCpuIdRanges[i + 1]; uLeaf++)
        {
            ULONG uEAX, uEBX, uECX, uEDX;
            rc = machine->GetCPUIDLeaf(uLeaf, &uEAX, &uEBX, &uECX, &uEDX);
            if (SUCCEEDED(rc))
            {
                if (details == VMINFO_MACHINEREADABLE)
                    RTPrintf("cpuid=%08x,%08x,%08x,%08x,%08x", uLeaf, uEAX, uEBX, uECX, uEDX);
                else
                {
                    if (!cFound)
                        RTPrintf("Leaf no.  EAX      EBX      ECX      EDX\n");
                    RTPrintf("                 %08x  %08x %08x %08x %08x\n", uLeaf, uEAX, uEBX, uECX, uEDX);
                }
                cFound++;
            }
        }
    if (!cFound && details != VMINFO_MACHINEREADABLE)
        RTPrintf("None\n");

    ComPtr <IBIOSSettings> biosSettings;
    CHECK_ERROR2_RET(machine, COMGETTER(BIOSSettings)(biosSettings.asOutParam()), hrcCheck);

    BIOSBootMenuMode_T bootMenuMode;
    CHECK_ERROR2_RET(biosSettings, COMGETTER(BootMenuMode)(&bootMenuMode), hrcCheck);
    const char *pszBootMenu;
    switch (bootMenuMode)
    {
        case BIOSBootMenuMode_Disabled:
            pszBootMenu = "disabled";
            break;
        case BIOSBootMenuMode_MenuOnly:
            if (details == VMINFO_MACHINEREADABLE)
                pszBootMenu = "menuonly";
            else
                pszBootMenu = "menu only";
            break;
        default:
            if (details == VMINFO_MACHINEREADABLE)
                pszBootMenu = "messageandmenu";
            else
                pszBootMenu = "message and menu";
    }
    if (details == VMINFO_MACHINEREADABLE)
        RTPrintf("bootmenu=\"%s\"\n", pszBootMenu);
    else
        RTPrintf("Boot menu mode:  %s\n", pszBootMenu);

    ComPtr<ISystemProperties> systemProperties;
    CHECK_ERROR2_RET(virtualBox, COMGETTER(SystemProperties)(systemProperties.asOutParam()), hrcCheck);
    ULONG maxBootPosition = 0;
    CHECK_ERROR2_RET(systemProperties, COMGETTER(MaxBootPosition)(&maxBootPosition), hrcCheck);
    for (ULONG i = 1; i <= maxBootPosition; i++)
    {
        DeviceType_T bootOrder;
        CHECK_ERROR2_RET(machine, GetBootOrder(i, &bootOrder), hrcCheck);
        if (bootOrder == DeviceType_Floppy)
        {
            if (details == VMINFO_MACHINEREADABLE)
                RTPrintf("boot%d=\"floppy\"\n", i);
            else
                RTPrintf("Boot Device (%d): Floppy\n", i);
        }
        else if (bootOrder == DeviceType_DVD)
        {
            if (details == VMINFO_MACHINEREADABLE)
                RTPrintf("boot%d=\"dvd\"\n", i);
            else
                RTPrintf("Boot Device (%d): DVD\n", i);
        }
        else if (bootOrder == DeviceType_HardDisk)
        {
            if (details == VMINFO_MACHINEREADABLE)
                RTPrintf("boot%d=\"disk\"\n", i);
            else
                RTPrintf("Boot Device (%d): HardDisk\n", i);
        }
        else if (bootOrder == DeviceType_Network)
        {
            if (details == VMINFO_MACHINEREADABLE)
                RTPrintf("boot%d=\"net\"\n", i);
            else
                RTPrintf("Boot Device (%d): Network\n", i);
        }
        else if (bootOrder == DeviceType_USB)
        {
            if (details == VMINFO_MACHINEREADABLE)
                RTPrintf("boot%d=\"usb\"\n", i);
            else
                RTPrintf("Boot Device (%d): USB\n", i);
        }
        else if (bootOrder == DeviceType_SharedFolder)
        {
            if (details == VMINFO_MACHINEREADABLE)
                RTPrintf("boot%d=\"sharedfolder\"\n", i);
            else
                RTPrintf("Boot Device (%d): Shared Folder\n", i);
        }
        else
        {
            if (details == VMINFO_MACHINEREADABLE)
                RTPrintf("boot%d=\"none\"\n", i);
            else
                RTPrintf("Boot Device (%d): Not Assigned\n", i);
        }
    }

    SHOW_BOOLEAN_PROP(biosSettings, ACPIEnabled,                "acpi",                 "ACPI");
    SHOW_BOOLEAN_PROP(biosSettings, IOAPICEnabled,              "ioapic",               "IOAPIC");
    SHOW_BOOLEAN_METHOD(machine, GetCPUProperty(CPUPropertyType_PAE, &f), "pae",        "PAE");
    SHOW_LONG64_PROP(biosSettings,  TimeOffset,                 "biossystemtimeoffset", "Time offset",  "ms");
    SHOW_BOOLEAN_PROP_EX(machine,   RTCUseUTC,                  "rtcuseutc",            "RTC",          "UTC", "local time");
    SHOW_BOOLEAN_METHOD(machine, GetHWVirtExProperty(HWVirtExPropertyType_Enabled,   &f),   "hwvirtex",     "Hardw. virt.ext");
    SHOW_BOOLEAN_METHOD(machine, GetHWVirtExProperty(HWVirtExPropertyType_Exclusive, &f),   "hwvirtexexcl", "Hardw. virt.ext exclusive");
    SHOW_BOOLEAN_METHOD(machine, GetHWVirtExProperty(HWVirtExPropertyType_NestedPaging, &f),"nestedpaging", "Nested Paging");
    SHOW_BOOLEAN_METHOD(machine, GetHWVirtExProperty(HWVirtExPropertyType_LargePages, &f),  "largepages",   "Large Pages");
    SHOW_BOOLEAN_METHOD(machine, GetHWVirtExProperty(HWVirtExPropertyType_VPID, &f),        "vtxvpid",      "VT-x VPID");

    MachineState_T machineState;
    CHECK_ERROR2_RET(machine, COMGETTER(State)(&machineState), hrcCheck);
    const char *pszState = machineStateToName(machineState, details == VMINFO_MACHINEREADABLE /*=fShort*/);

    LONG64 stateSince;
    machine->COMGETTER(LastStateChange)(&stateSince);
    RTTIMESPEC timeSpec;
    RTTimeSpecSetMilli(&timeSpec, stateSince);
    char pszTime[30] = {0};
    RTTimeSpecToString(&timeSpec, pszTime, sizeof(pszTime));
    if (details == VMINFO_MACHINEREADABLE)
    {
        RTPrintf("VMState=\"%s\"\n", pszState);
        RTPrintf("VMStateChangeTime=\"%s\"\n", pszTime);

        Bstr stateFile;
        machine->COMGETTER(StateFilePath)(stateFile.asOutParam());
        if (!stateFile.isEmpty())
            RTPrintf("VMStateFile=\"%ls\"\n", stateFile.raw());
    }
    else
        RTPrintf("State:           %s (since %s)\n", pszState, pszTime);

    SHOW_ULONG_PROP(      machine,  MonitorCount,               "monitorcount",             "Monitor count", "");
    SHOW_BOOLEAN_PROP(    machine,  Accelerate3DEnabled,        "accelerate3d",             "3D Acceleration");
#ifdef VBOX_WITH_VIDEOHWACCEL
    SHOW_BOOLEAN_PROP(    machine,  Accelerate2DVideoEnabled,   "accelerate2dvideo",        "2D Video Acceleration");
#endif
    SHOW_BOOLEAN_PROP(    machine,  TeleporterEnabled,          "teleporterenabled",        "Teleporter Enabled");
    SHOW_ULONG_PROP(      machine,  TeleporterPort,             "teleporterport",           "Teleporter Port", "");
    SHOW_STRING_PROP(     machine,  TeleporterAddress,          "teleporteraddress",        "Teleporter Address");
    SHOW_STRING_PROP(     machine,  TeleporterPassword,         "teleporterpassword",       "Teleporter Password");
    SHOW_BOOLEAN_PROP(    machine,  TracingEnabled,             "tracing-enabled",          "Tracing Enabled");
    SHOW_BOOLEAN_PROP(    machine,  AllowTracingToAccessVM,     "tracing-allow-vm-access",  "Allow Tracing to Access VM");
    SHOW_STRING_PROP(     machine,  TracingConfig,              "tracing-config",           "Tracing Configuration");
    SHOW_BOOLEAN_PROP(    machine,  AutostartEnabled,           "autostart-enabled",        "Autostart Enabled");
    SHOW_ULONG_PROP(      machine,  AutostartDelay,             "autostart-delay",          "Autostart Delay", "");

/** @todo Convert the remainder of the function to SHOW_XXX macros and add error
 *        checking where missing. */
    /*
     * Storage Controllers and their attached Mediums.
     */
    com::SafeIfaceArray<IStorageController> storageCtls;
    CHECK_ERROR(machine, COMGETTER(StorageControllers)(ComSafeArrayAsOutParam(storageCtls)));
    for (size_t i = 0; i < storageCtls.size(); ++ i)
    {
        ComPtr<IStorageController> storageCtl = storageCtls[i];
        StorageControllerType_T    enmCtlType = StorageControllerType_Null;
        const char *pszCtl = NULL;
        ULONG ulValue = 0;
        BOOL  fBootable = FALSE;
        Bstr storageCtlName;

        storageCtl->COMGETTER(Name)(storageCtlName.asOutParam());
        if (details == VMINFO_MACHINEREADABLE)
            RTPrintf("storagecontrollername%u=\"%ls\"\n", i, storageCtlName.raw());
        else
            RTPrintf("Storage Controller Name (%u):            %ls\n", i, storageCtlName.raw());

        storageCtl->COMGETTER(ControllerType)(&enmCtlType);
        switch (enmCtlType)
        {
            case StorageControllerType_LsiLogic:
                pszCtl = "LsiLogic";
                break;
            case StorageControllerType_BusLogic:
                pszCtl = "BusLogic";
                break;
            case StorageControllerType_IntelAhci:
                pszCtl = "IntelAhci";
                break;
            case StorageControllerType_PIIX3:
                pszCtl = "PIIX3";
                break;
            case StorageControllerType_PIIX4:
                pszCtl = "PIIX4";
                break;
            case StorageControllerType_ICH6:
                pszCtl = "ICH6";
                break;
            case StorageControllerType_I82078:
                pszCtl = "I82078";
                break;

            default:
                pszCtl = "unknown";
        }
        if (details == VMINFO_MACHINEREADABLE)
            RTPrintf("storagecontrollertype%u=\"%s\"\n", i, pszCtl);
        else
            RTPrintf("Storage Controller Type (%u):            %s\n", i, pszCtl);

        storageCtl->COMGETTER(Instance)(&ulValue);
        if (details == VMINFO_MACHINEREADABLE)
            RTPrintf("storagecontrollerinstance%u=\"%lu\"\n", i, ulValue);
        else
            RTPrintf("Storage Controller Instance Number (%u): %lu\n", i, ulValue);

        storageCtl->COMGETTER(MaxPortCount)(&ulValue);
        if (details == VMINFO_MACHINEREADABLE)
            RTPrintf("storagecontrollermaxportcount%u=\"%lu\"\n", i, ulValue);
        else
            RTPrintf("Storage Controller Max Port Count (%u):  %lu\n", i, ulValue);

        storageCtl->COMGETTER(PortCount)(&ulValue);
        if (details == VMINFO_MACHINEREADABLE)
            RTPrintf("storagecontrollerportcount%u=\"%lu\"\n", i, ulValue);
        else
            RTPrintf("Storage Controller Port Count (%u):      %lu\n", i, ulValue);

        storageCtl->COMGETTER(Bootable)(&fBootable);
        if (details == VMINFO_MACHINEREADABLE)
            RTPrintf("storagecontrollerbootable%u=\"%s\"\n", i, fBootable ? "on" : "off");
        else
            RTPrintf("Storage Controller Bootable (%u):        %s\n", i, fBootable ? "on" : "off");
    }

    for (size_t j = 0; j < storageCtls.size(); ++ j)
    {
        ComPtr<IStorageController> storageCtl = storageCtls[j];
        ComPtr<IMedium> medium;
        Bstr storageCtlName;
        Bstr filePath;
        ULONG cDevices;
        ULONG cPorts;

        storageCtl->COMGETTER(Name)(storageCtlName.asOutParam());
        storageCtl->COMGETTER(MaxDevicesPerPortCount)(&cDevices);
        storageCtl->COMGETTER(PortCount)(&cPorts);

        for (ULONG i = 0; i < cPorts; ++ i)
        {
            for (ULONG k = 0; k < cDevices; ++ k)
            {
                ComPtr<IMediumAttachment> mediumAttach;
                machine->GetMediumAttachment(storageCtlName.raw(),
                                             i, k,
                                             mediumAttach.asOutParam());
                BOOL fIsEjected = FALSE;
                BOOL fTempEject = FALSE;
                DeviceType_T devType = DeviceType_Null;
                if (mediumAttach)
                {
                    mediumAttach->COMGETTER(TemporaryEject)(&fTempEject);
                    mediumAttach->COMGETTER(IsEjected)(&fIsEjected);
                    mediumAttach->COMGETTER(Type)(&devType);
                }
                rc = machine->GetMedium(storageCtlName.raw(), i, k,
                                        medium.asOutParam());
                if (SUCCEEDED(rc) && medium)
                {
                    BOOL fPassthrough = FALSE;

                    if (mediumAttach)
                        mediumAttach->COMGETTER(Passthrough)(&fPassthrough);

                    medium->COMGETTER(Location)(filePath.asOutParam());
                    Bstr uuid;
                    medium->COMGETTER(Id)(uuid.asOutParam());

                    if (details == VMINFO_MACHINEREADABLE)
                    {
                        RTPrintf("\"%ls-%d-%d\"=\"%ls\"\n", storageCtlName.raw(),
                                 i, k, filePath.raw());
                        RTPrintf("\"%ls-ImageUUID-%d-%d\"=\"%s\"\n",
                                 storageCtlName.raw(), i, k, Utf8Str(uuid).c_str());
                        if (fPassthrough)
                            RTPrintf("\"%ls-dvdpassthrough\"=\"%s\"\n", storageCtlName.raw(),
                                     fPassthrough ? "on" : "off");
                        if (devType == DeviceType_DVD)
                        {
                            RTPrintf("\"%ls-tempeject\"=\"%s\"\n", storageCtlName.raw(),
                                     fTempEject ? "on" : "off");
                            RTPrintf("\"%ls-IsEjected\"=\"%s\"\n", storageCtlName.raw(),
                                     fIsEjected ? "on" : "off");
                        }
                    }
                    else
                    {
                        RTPrintf("%ls (%d, %d): %ls (UUID: %s)",
                                 storageCtlName.raw(), i, k, filePath.raw(),
                                 Utf8Str(uuid).c_str());
                        if (fPassthrough)
                            RTPrintf(" (passthrough enabled)");
                        if (fTempEject)
                            RTPrintf(" (temp eject)");
                        if (fIsEjected)
                            RTPrintf(" (ejected)");
                        RTPrintf("\n");
                    }
                }
                else if (SUCCEEDED(rc))
                {
                    if (details == VMINFO_MACHINEREADABLE)
                    {
                        RTPrintf("\"%ls-%d-%d\"=\"emptydrive\"\n", storageCtlName.raw(), i, k);
                        if (devType == DeviceType_DVD)
                            RTPrintf("\"%ls-IsEjected\"=\"%s\"\n", storageCtlName.raw(),
                                     fIsEjected ? "on" : "off");
                    }
                    else
                    {
                        RTPrintf("%ls (%d, %d): Empty", storageCtlName.raw(), i, k);
                        if (fTempEject)
                            RTPrintf(" (temp eject)");
                        if (fIsEjected)
                            RTPrintf(" (ejected)");
                        RTPrintf("\n");
                    }
                }
                else
                {
                    if (details == VMINFO_MACHINEREADABLE)
                        RTPrintf("\"%ls-%d-%d\"=\"none\"\n", storageCtlName.raw(), i, k);
                }
            }
        }
    }

    /* get the maximum amount of NICS */
    ULONG maxNICs = getMaxNics(virtualBox, machine);

    for (ULONG currentNIC = 0; currentNIC < maxNICs; currentNIC++)
    {
        ComPtr<INetworkAdapter> nic;
        rc = machine->GetNetworkAdapter(currentNIC, nic.asOutParam());
        if (SUCCEEDED(rc) && nic)
        {
            BOOL fEnabled;
            nic->COMGETTER(Enabled)(&fEnabled);
            if (!fEnabled)
            {
                if (details == VMINFO_MACHINEREADABLE)
                    RTPrintf("nic%d=\"none\"\n", currentNIC + 1);
                else
                    RTPrintf("NIC %d:           disabled\n", currentNIC + 1);
            }
            else
            {
                Bstr strMACAddress;
                nic->COMGETTER(MACAddress)(strMACAddress.asOutParam());
                Utf8Str strAttachment;
                Utf8Str strNatSettings = "";
                Utf8Str strNatForwardings = "";
                NetworkAttachmentType_T attachment;
                nic->COMGETTER(AttachmentType)(&attachment);
                switch (attachment)
                {
                    case NetworkAttachmentType_Null:
                        if (details == VMINFO_MACHINEREADABLE)
                            strAttachment = "null";
                        else
                            strAttachment = "none";
                        break;

                    case NetworkAttachmentType_NAT:
                    {
                        Bstr strNetwork;
                        ComPtr<INATEngine> engine;
                        nic->COMGETTER(NATEngine)(engine.asOutParam());
                        engine->COMGETTER(Network)(strNetwork.asOutParam());
                        com::SafeArray<BSTR> forwardings;
                        engine->COMGETTER(Redirects)(ComSafeArrayAsOutParam(forwardings));
                        strNatForwardings = "";
                        for (size_t i = 0; i < forwardings.size(); ++i)
                        {
                            bool fSkip = false;
                            uint16_t port = 0;
                            BSTR r = forwardings[i];
                            Utf8Str utf = Utf8Str(r);
                            Utf8Str strName;
                            Utf8Str strProto;
                            Utf8Str strHostPort;
                            Utf8Str strHostIP;
                            Utf8Str strGuestPort;
                            Utf8Str strGuestIP;
                            size_t pos, ppos;
                            pos = ppos = 0;
                            #define ITERATE_TO_NEXT_TERM(res, str, pos, ppos)   \
                            do {                                                \
                                pos = str.find(",", ppos);                      \
                                if (pos == Utf8Str::npos)                       \
                                {                                               \
                                    Log(( #res " extracting from %s is failed\n", str.c_str())); \
                                    fSkip = true;                               \
                                }                                               \
                                res = str.substr(ppos, pos - ppos);             \
                                Log2((#res " %s pos:%d, ppos:%d\n", res.c_str(), pos, ppos)); \
                                ppos = pos + 1;                                 \
                            } while (0)
                            ITERATE_TO_NEXT_TERM(strName, utf, pos, ppos);
                            if (fSkip) continue;
                            ITERATE_TO_NEXT_TERM(strProto, utf, pos, ppos);
                            if (fSkip) continue;
                            ITERATE_TO_NEXT_TERM(strHostIP, utf, pos, ppos);
                            if (fSkip) continue;
                            ITERATE_TO_NEXT_TERM(strHostPort, utf, pos, ppos);
                            if (fSkip) continue;
                            ITERATE_TO_NEXT_TERM(strGuestIP, utf, pos, ppos);
                            if (fSkip) continue;
                            strGuestPort = utf.substr(ppos, utf.length() - ppos);
                            #undef ITERATE_TO_NEXT_TERM
                            switch (strProto.toUInt32())
                            {
                                case NATProtocol_TCP:
                                    strProto = "tcp";
                                    break;
                                case NATProtocol_UDP:
                                    strProto = "udp";
                                    break;
                                default:
                                    strProto = "unk";
                                    break;
                            }
                            if (details == VMINFO_MACHINEREADABLE)
                            {
                                strNatForwardings = Utf8StrFmt("%sForwarding(%d)=\"%s,%s,%s,%s,%s,%s\"\n",
                                    strNatForwardings.c_str(), i, strName.c_str(), strProto.c_str(),
                                    strHostIP.c_str(), strHostPort.c_str(),
                                    strGuestIP.c_str(), strGuestPort.c_str());
                            }
                            else
                            {
                                strNatForwardings = Utf8StrFmt("%sNIC %d Rule(%d):   name = %s, protocol = %s,"
                                    " host ip = %s, host port = %s, guest ip = %s, guest port = %s\n",
                                    strNatForwardings.c_str(), currentNIC + 1, i, strName.c_str(), strProto.c_str(),
                                    strHostIP.c_str(), strHostPort.c_str(),
                                    strGuestIP.c_str(), strGuestPort.c_str());
                            }
                        }
                        ULONG mtu = 0;
                        ULONG sockSnd = 0;
                        ULONG sockRcv = 0;
                        ULONG tcpSnd = 0;
                        ULONG tcpRcv = 0;
                        engine->GetNetworkSettings(&mtu, &sockSnd, &sockRcv, &tcpSnd, &tcpRcv);

                        if (details == VMINFO_MACHINEREADABLE)
                        {
                            RTPrintf("natnet%d=\"%ls\"\n", currentNIC + 1, strNetwork.length() ? strNetwork.raw(): Bstr("nat").raw());
                            strAttachment = "nat";
                            strNatSettings = Utf8StrFmt("mtu=\"%d\"\nsockSnd=\"%d\"\nsockRcv=\"%d\"\ntcpWndSnd=\"%d\"\ntcpWndRcv=\"%d\"\n",
                                mtu, sockSnd ? sockSnd : 64, sockRcv ? sockRcv : 64 , tcpSnd ? tcpSnd : 64, tcpRcv ? tcpRcv : 64);
                        }
                        else
                        {
                            strAttachment = "NAT";
                            strNatSettings = Utf8StrFmt("NIC %d Settings:  MTU: %d, Socket (send: %d, receive: %d), TCP Window (send:%d, receive: %d)\n",
                                currentNIC + 1, mtu, sockSnd ? sockSnd : 64, sockRcv ? sockRcv : 64 , tcpSnd ? tcpSnd : 64, tcpRcv ? tcpRcv : 64);
                        }
                        break;
                    }

                    case NetworkAttachmentType_Bridged:
                    {
                        Bstr strBridgeAdp;
                        nic->COMGETTER(BridgedInterface)(strBridgeAdp.asOutParam());
                        if (details == VMINFO_MACHINEREADABLE)
                        {
                            RTPrintf("bridgeadapter%d=\"%ls\"\n", currentNIC + 1, strBridgeAdp.raw());
                            strAttachment = "bridged";
                        }
                        else
                            strAttachment = Utf8StrFmt("Bridged Interface '%ls'", strBridgeAdp.raw());
                        break;
                    }

                    case NetworkAttachmentType_Internal:
                    {
                        Bstr strNetwork;
                        nic->COMGETTER(InternalNetwork)(strNetwork.asOutParam());
                        if (details == VMINFO_MACHINEREADABLE)
                        {
                            RTPrintf("intnet%d=\"%ls\"\n", currentNIC + 1, strNetwork.raw());
                            strAttachment = "intnet";
                        }
                        else
                            strAttachment = Utf8StrFmt("Internal Network '%s'", Utf8Str(strNetwork).c_str());
                        break;
                    }

                    case NetworkAttachmentType_HostOnly:
                    {
                        Bstr strHostonlyAdp;
                        nic->COMGETTER(HostOnlyInterface)(strHostonlyAdp.asOutParam());
                        if (details == VMINFO_MACHINEREADABLE)
                        {
                            RTPrintf("hostonlyadapter%d=\"%ls\"\n", currentNIC + 1, strHostonlyAdp.raw());
                            strAttachment = "hostonly";
                        }
                        else
                            strAttachment = Utf8StrFmt("Host-only Interface '%ls'", strHostonlyAdp.raw());
                        break;
                    }
                    case NetworkAttachmentType_Generic:
                    {
                        Bstr strGenericDriver;
                        nic->COMGETTER(GenericDriver)(strGenericDriver.asOutParam());
                        if (details == VMINFO_MACHINEREADABLE)
                        {
                            RTPrintf("generic%d=\"%ls\"\n", currentNIC + 1, strGenericDriver.raw());
                            strAttachment = "Generic";
                        }
                        else
                        {
                            strAttachment = Utf8StrFmt("Generic '%ls'", strGenericDriver.raw());

                            // show the generic properties
                            com::SafeArray<BSTR> aProperties;
                            com::SafeArray<BSTR> aValues;
                            rc = nic->GetProperties(NULL,
                                                    ComSafeArrayAsOutParam(aProperties),
                                                    ComSafeArrayAsOutParam(aValues));
                            if (SUCCEEDED(rc))
                            {
                                strAttachment += " { ";
                                for (unsigned i = 0; i < aProperties.size(); ++i)
                                    strAttachment += Utf8StrFmt(!i ? "%ls='%ls'" : ", %ls='%ls'",
                                                                aProperties[i], aValues[i]);
                                strAttachment += " }";
                            }
                        }
                        break;
                    }
                    default:
                        strAttachment = "unknown";
                        break;
                }

                /* cable connected */
                BOOL fConnected;
                nic->COMGETTER(CableConnected)(&fConnected);

                /* promisc policy */
                NetworkAdapterPromiscModePolicy_T enmPromiscModePolicy;
                CHECK_ERROR2_RET(nic, COMGETTER(PromiscModePolicy)(&enmPromiscModePolicy), hrcCheck);
                const char *pszPromiscuousGuestPolicy;
                switch (enmPromiscModePolicy)
                {
                    case NetworkAdapterPromiscModePolicy_Deny:          pszPromiscuousGuestPolicy = "deny"; break;
                    case NetworkAdapterPromiscModePolicy_AllowNetwork:  pszPromiscuousGuestPolicy = "allow-vms"; break;
                    case NetworkAdapterPromiscModePolicy_AllowAll:      pszPromiscuousGuestPolicy = "allow-all"; break;
                    default: AssertFailedReturn(VERR_INTERNAL_ERROR_4);
                }

                /* trace stuff */
                BOOL fTraceEnabled;
                nic->COMGETTER(TraceEnabled)(&fTraceEnabled);
                Bstr traceFile;
                nic->COMGETTER(TraceFile)(traceFile.asOutParam());

                /* NIC type */
                NetworkAdapterType_T NICType;
                nic->COMGETTER(AdapterType)(&NICType);
                const char *pszNICType;
                switch (NICType)
                {
                    case NetworkAdapterType_Am79C970A:  pszNICType = "Am79C970A";   break;
                    case NetworkAdapterType_Am79C973:   pszNICType = "Am79C973";    break;
#ifdef VBOX_WITH_E1000
                    case NetworkAdapterType_I82540EM:   pszNICType = "82540EM";     break;
                    case NetworkAdapterType_I82543GC:   pszNICType = "82543GC";     break;
                    case NetworkAdapterType_I82545EM:   pszNICType = "82545EM";     break;
#endif
#ifdef VBOX_WITH_VIRTIO
                    case NetworkAdapterType_Virtio:     pszNICType = "virtio";      break;
#endif
                    default: AssertFailed();            pszNICType = "unknown";     break;
                }

                /* reported line speed */
                ULONG ulLineSpeed;
                nic->COMGETTER(LineSpeed)(&ulLineSpeed);

                /* boot priority of the adapter */
                ULONG ulBootPriority;
                nic->COMGETTER(BootPriority)(&ulBootPriority);

                /* bandwidth group */
                ComObjPtr<IBandwidthGroup> pBwGroup;
                Bstr strBwGroup;
                nic->COMGETTER(BandwidthGroup)(pBwGroup.asOutParam());
                if (!pBwGroup.isNull())
                    pBwGroup->COMGETTER(Name)(strBwGroup.asOutParam());

                if (details == VMINFO_MACHINEREADABLE)
                {
                    RTPrintf("macaddress%d=\"%ls\"\n", currentNIC + 1, strMACAddress.raw());
                    RTPrintf("cableconnected%d=\"%s\"\n", currentNIC + 1, fConnected ? "on" : "off");
                    RTPrintf("nic%d=\"%s\"\n", currentNIC + 1, strAttachment.c_str());
                    RTPrintf("nictype%d=\"%s\"\n", currentNIC + 1, pszNICType);
                    RTPrintf("nicspeed%d=\"%d\"\n", currentNIC + 1, ulLineSpeed);
                }
                else
                    RTPrintf("NIC %u:           MAC: %ls, Attachment: %s, Cable connected: %s, Trace: %s (file: %ls), Type: %s, Reported speed: %d Mbps, Boot priority: %d, Promisc Policy: %s, Bandwidth group: %ls\n",
                             currentNIC + 1, strMACAddress.raw(), strAttachment.c_str(),
                             fConnected ? "on" : "off",
                             fTraceEnabled ? "on" : "off",
                             traceFile.isEmpty() ? Bstr("none").raw() : traceFile.raw(),
                             pszNICType,
                             ulLineSpeed / 1000,
                             (int)ulBootPriority,
                             pszPromiscuousGuestPolicy,
                             strBwGroup.isEmpty() ? Bstr("none").raw() : strBwGroup.raw());
                if (strNatSettings.length())
                    RTPrintf(strNatSettings.c_str());
                if (strNatForwardings.length())
                    RTPrintf(strNatForwardings.c_str());
            }
        }
    }

    /* Pointing device information */
    PointingHIDType_T aPointingHID;
    const char *pszHID = "Unknown";
    const char *pszMrHID = "unknown";
    machine->COMGETTER(PointingHIDType)(&aPointingHID);
    switch (aPointingHID)
    {
        case PointingHIDType_None:
            pszHID = "None";
            pszMrHID = "none";
            break;
        case PointingHIDType_PS2Mouse:
            pszHID = "PS/2 Mouse";
            pszMrHID = "ps2mouse";
            break;
        case PointingHIDType_USBMouse:
            pszHID = "USB Mouse";
            pszMrHID = "usbmouse";
            break;
        case PointingHIDType_USBTablet:
            pszHID = "USB Tablet";
            pszMrHID = "usbtablet";
            break;
        case PointingHIDType_ComboMouse:
            pszHID = "USB Tablet and PS/2 Mouse";
            pszMrHID = "combomouse";
            break;
        default:
            break;
    }
    if (details == VMINFO_MACHINEREADABLE)
        RTPrintf("hidpointing=\"%s\"\n", pszMrHID);
    else
        RTPrintf("Pointing Device: %s\n", pszHID);

    /* Keyboard device information */
    KeyboardHIDType_T aKeyboardHID;
    machine->COMGETTER(KeyboardHIDType)(&aKeyboardHID);
    pszHID = "Unknown";
    pszMrHID = "unknown";
    switch (aKeyboardHID)
    {
        case KeyboardHIDType_None:
            pszHID = "None";
            pszMrHID = "none";
            break;
        case KeyboardHIDType_PS2Keyboard:
            pszHID = "PS/2 Keyboard";
            pszMrHID = "ps2kbd";
            break;
        case KeyboardHIDType_USBKeyboard:
            pszHID = "USB Keyboard";
            pszMrHID = "usbkbd";
            break;
        case KeyboardHIDType_ComboKeyboard:
            pszHID = "USB and PS/2 Keyboard";
            pszMrHID = "combokbd";
            break;
        default:
            break;
    }
    if (details == VMINFO_MACHINEREADABLE)
        RTPrintf("hidkeyboard=\"%s\"\n", pszMrHID);
    else
        RTPrintf("Keyboard Device: %s\n", pszHID);

    ComPtr<ISystemProperties> sysProps;
    virtualBox->COMGETTER(SystemProperties)(sysProps.asOutParam());

    /* get the maximum amount of UARTs */
    ULONG maxUARTs = 0;
    sysProps->COMGETTER(SerialPortCount)(&maxUARTs);
    for (ULONG currentUART = 0; currentUART < maxUARTs; currentUART++)
    {
        ComPtr<ISerialPort> uart;
        rc = machine->GetSerialPort(currentUART, uart.asOutParam());
        if (SUCCEEDED(rc) && uart)
        {
            /* show the config of this UART */
            BOOL fEnabled;
            uart->COMGETTER(Enabled)(&fEnabled);
            if (!fEnabled)
            {
                if (details == VMINFO_MACHINEREADABLE)
                    RTPrintf("uart%d=\"off\"\n", currentUART + 1);
                else
                    RTPrintf("UART %d:          disabled\n", currentUART + 1);
            }
            else
            {
                ULONG ulIRQ, ulIOBase;
                PortMode_T HostMode;
                Bstr path;
                BOOL fServer;
                uart->COMGETTER(IRQ)(&ulIRQ);
                uart->COMGETTER(IOBase)(&ulIOBase);
                uart->COMGETTER(Path)(path.asOutParam());
                uart->COMGETTER(Server)(&fServer);
                uart->COMGETTER(HostMode)(&HostMode);

                if (details == VMINFO_MACHINEREADABLE)
                    RTPrintf("uart%d=\"%#06x,%d\"\n", currentUART + 1,
                             ulIOBase, ulIRQ);
                else
                    RTPrintf("UART %d:          I/O base: %#06x, IRQ: %d",
                             currentUART + 1, ulIOBase, ulIRQ);
                switch (HostMode)
                {
                    default:
                    case PortMode_Disconnected:
                        if (details == VMINFO_MACHINEREADABLE)
                            RTPrintf("uartmode%d=\"disconnected\"\n", currentUART + 1);
                        else
                            RTPrintf(", disconnected\n");
                        break;
                    case PortMode_RawFile:
                        if (details == VMINFO_MACHINEREADABLE)
                            RTPrintf("uartmode%d=\"%ls\"\n", currentUART + 1,
                                     path.raw());
                        else
                            RTPrintf(", attached to raw file '%ls'\n",
                                     path.raw());
                        break;
                    case PortMode_HostPipe:
                        if (details == VMINFO_MACHINEREADABLE)
                            RTPrintf("uartmode%d=\"%s,%ls\"\n", currentUART + 1,
                                     fServer ? "server" : "client", path.raw());
                        else
                            RTPrintf(", attached to pipe (%s) '%ls'\n",
                                     fServer ? "server" : "client", path.raw());
                        break;
                    case PortMode_HostDevice:
                        if (details == VMINFO_MACHINEREADABLE)
                            RTPrintf("uartmode%d=\"%ls\"\n", currentUART + 1,
                                     path.raw());
                        else
                            RTPrintf(", attached to device '%ls'\n", path.raw());
                        break;
                }
            }
        }
    }

    /* get the maximum amount of LPTs */
    ULONG maxLPTs = 0;
    sysProps->COMGETTER(ParallelPortCount)(&maxLPTs);
    for (ULONG currentLPT = 0; currentLPT < maxLPTs; currentLPT++)
    {
        ComPtr<IParallelPort> lpt;
        rc = machine->GetParallelPort(currentLPT, lpt.asOutParam());
        if (SUCCEEDED(rc) && lpt)
        {
            /* show the config of this LPT */
            BOOL fEnabled;
            lpt->COMGETTER(Enabled)(&fEnabled);
            if (!fEnabled)
            {
                if (details == VMINFO_MACHINEREADABLE)
                    RTPrintf("lpt%d=\"off\"\n", currentLPT + 1);
                else
                    RTPrintf("LPT %d:           disabled\n", currentLPT + 1);
            }
            else
            {
                ULONG ulIRQ, ulIOBase;
                Bstr path;
                lpt->COMGETTER(IRQ)(&ulIRQ);
                lpt->COMGETTER(IOBase)(&ulIOBase);
                lpt->COMGETTER(Path)(path.asOutParam());

                if (details == VMINFO_MACHINEREADABLE)
                    RTPrintf("lpt%d=\"%#06x,%d\"\n", currentLPT + 1,
                             ulIOBase, ulIRQ);
                else
                    RTPrintf("LPT %d:           I/O base: %#06x, IRQ: %d",
                             currentLPT + 1, ulIOBase, ulIRQ);
                    if (details == VMINFO_MACHINEREADABLE)
                        RTPrintf("lptmode%d=\"%ls\"\n", currentLPT + 1,
                                 path.raw());
                    else
                        RTPrintf(", attached to device '%ls'\n", path.raw());
            }
        }
    }

    ComPtr<IAudioAdapter> AudioAdapter;
    rc = machine->COMGETTER(AudioAdapter)(AudioAdapter.asOutParam());
    if (SUCCEEDED(rc))
    {
        const char *pszDrv  = "Unknown";
        const char *pszCtrl = "Unknown";
        BOOL fEnabled;
        rc = AudioAdapter->COMGETTER(Enabled)(&fEnabled);
        if (SUCCEEDED(rc) && fEnabled)
        {
            AudioDriverType_T enmDrvType;
            rc = AudioAdapter->COMGETTER(AudioDriver)(&enmDrvType);
            switch (enmDrvType)
            {
                case AudioDriverType_Null:
                    if (details == VMINFO_MACHINEREADABLE)
                        pszDrv = "null";
                    else
                        pszDrv = "Null";
                    break;
                case AudioDriverType_WinMM:
                    if (details == VMINFO_MACHINEREADABLE)
                        pszDrv = "winmm";
                    else
                        pszDrv = "WINMM";
                    break;
                case AudioDriverType_DirectSound:
                    if (details == VMINFO_MACHINEREADABLE)
                        pszDrv = "dsound";
                    else
                        pszDrv = "DSOUND";
                    break;
                case AudioDriverType_OSS:
                    if (details == VMINFO_MACHINEREADABLE)
                        pszDrv = "oss";
                    else
                        pszDrv = "OSS";
                    break;
                case AudioDriverType_ALSA:
                    if (details == VMINFO_MACHINEREADABLE)
                        pszDrv = "alsa";
                    else
                        pszDrv = "ALSA";
                    break;
                case AudioDriverType_Pulse:
                    if (details == VMINFO_MACHINEREADABLE)
                        pszDrv = "pulse";
                    else
                        pszDrv = "PulseAudio";
                    break;
                case AudioDriverType_CoreAudio:
                    if (details == VMINFO_MACHINEREADABLE)
                        pszDrv = "coreaudio";
                    else
                        pszDrv = "CoreAudio";
                    break;
                case AudioDriverType_SolAudio:
                    if (details == VMINFO_MACHINEREADABLE)
                        pszDrv = "solaudio";
                    else
                        pszDrv = "SolAudio";
                    break;
                default:
                    if (details == VMINFO_MACHINEREADABLE)
                        pszDrv = "unknown";
                    break;
            }
            AudioControllerType_T enmCtrlType;
            rc = AudioAdapter->COMGETTER(AudioController)(&enmCtrlType);
            switch (enmCtrlType)
            {
                case AudioControllerType_AC97:
                    if (details == VMINFO_MACHINEREADABLE)
                        pszCtrl = "ac97";
                    else
                        pszCtrl = "AC97";
                    break;
                case AudioControllerType_SB16:
                    if (details == VMINFO_MACHINEREADABLE)
                        pszCtrl = "sb16";
                    else
                        pszCtrl = "SB16";
                    break;
                case AudioControllerType_HDA:
                    if (details == VMINFO_MACHINEREADABLE)
                        pszCtrl = "hda";
                    else
                        pszCtrl = "HDA";
                    break;
            }
        }
        else
            fEnabled = FALSE;
        if (details == VMINFO_MACHINEREADABLE)
        {
            if (fEnabled)
                RTPrintf("audio=\"%s\"\n", pszDrv);
            else
                RTPrintf("audio=\"none\"\n");
        }
        else
        {
            RTPrintf("Audio:           %s",
                    fEnabled ? "enabled" : "disabled");
            if (fEnabled)
                RTPrintf(" (Driver: %s, Controller: %s)",
                    pszDrv, pszCtrl);
            RTPrintf("\n");
        }
    }

    /* Shared clipboard */
    {
        const char *psz = "Unknown";
        ClipboardMode_T enmMode;
        rc = machine->COMGETTER(ClipboardMode)(&enmMode);
        switch (enmMode)
        {
            case ClipboardMode_Disabled:
                if (details == VMINFO_MACHINEREADABLE)
                    psz = "disabled";
                else
                    psz = "disabled";
                break;
            case ClipboardMode_HostToGuest:
                if (details == VMINFO_MACHINEREADABLE)
                    psz = "hosttoguest";
                else
                    psz = "HostToGuest";
                break;
            case ClipboardMode_GuestToHost:
                if (details == VMINFO_MACHINEREADABLE)
                    psz = "guesttohost";
                else
                    psz = "GuestToHost";
                break;
            case ClipboardMode_Bidirectional:
                if (details == VMINFO_MACHINEREADABLE)
                    psz = "bidirectional";
                else
                    psz = "Bidirectional";
                break;
            default:
                if (details == VMINFO_MACHINEREADABLE)
                    psz = "unknown";
                break;
        }
        if (details == VMINFO_MACHINEREADABLE)
            RTPrintf("clipboard=\"%s\"\n", psz);
        else
            RTPrintf("Clipboard Mode:  %s\n", psz);
    }

    /* Drag'n'drop */
    {
        const char *psz = "Unknown";
        DragAndDropMode_T enmMode;
        rc = machine->COMGETTER(DragAndDropMode)(&enmMode);
        switch (enmMode)
        {
            case DragAndDropMode_Disabled:
                if (details == VMINFO_MACHINEREADABLE)
                    psz = "disabled";
                else
                    psz = "disabled";
                break;
            case DragAndDropMode_HostToGuest:
                if (details == VMINFO_MACHINEREADABLE)
                    psz = "hosttoguest";
                else
                    psz = "HostToGuest";
                break;
            case DragAndDropMode_GuestToHost:
                if (details == VMINFO_MACHINEREADABLE)
                    psz = "guesttohost";
                else
                    psz = "GuestToHost";
                break;
            case DragAndDropMode_Bidirectional:
                if (details == VMINFO_MACHINEREADABLE)
                    psz = "bidirectional";
                else
                    psz = "Bidirectional";
                break;
            default:
                if (details == VMINFO_MACHINEREADABLE)
                    psz = "unknown";
                break;
        }
        if (details == VMINFO_MACHINEREADABLE)
            RTPrintf("draganddrop=\"%s\"\n", psz);
        else
            RTPrintf("Drag'n'drop Mode:  %s\n", psz);
    }

    if (console)
    {
        do
        {
            ComPtr<IDisplay> display;
            rc = console->COMGETTER(Display)(display.asOutParam());
            if (rc == E_ACCESSDENIED || display.isNull())
                break; /* VM not powered up */
            if (FAILED(rc))
            {
                com::GlueHandleComError(console, "COMGETTER(Display)(display.asOutParam())", rc, __FILE__, __LINE__);
                return rc;
            }
            ULONG xRes, yRes, bpp;
            rc = display->GetScreenResolution(0, &xRes, &yRes, &bpp);
            if (rc == E_ACCESSDENIED)
                break; /* VM not powered up */
            if (FAILED(rc))
            {
                com::ErrorInfo info(display, COM_IIDOF(IDisplay));
                GluePrintErrorInfo(info);
                return rc;
            }
            if (details == VMINFO_MACHINEREADABLE)
                RTPrintf("VideoMode=\"%d,%d,%d\"\n", xRes, yRes, bpp);
            else
                RTPrintf("Video mode:      %dx%dx%d\n", xRes, yRes, bpp);
        }
        while (0);
    }

    /*
     * Remote Desktop
     */
    ComPtr<IVRDEServer> vrdeServer;
    rc = machine->COMGETTER(VRDEServer)(vrdeServer.asOutParam());
    if (SUCCEEDED(rc) && vrdeServer)
    {
        BOOL fEnabled = false;
        vrdeServer->COMGETTER(Enabled)(&fEnabled);
        if (fEnabled)
        {
            LONG currentPort = -1;
            Bstr ports;
            vrdeServer->GetVRDEProperty(Bstr("TCP/Ports").raw(), ports.asOutParam());
            Bstr address;
            vrdeServer->GetVRDEProperty(Bstr("TCP/Address").raw(), address.asOutParam());
            BOOL fMultiCon;
            vrdeServer->COMGETTER(AllowMultiConnection)(&fMultiCon);
            BOOL fReuseCon;
            vrdeServer->COMGETTER(ReuseSingleConnection)(&fReuseCon);
            Bstr videoChannel;
            vrdeServer->GetVRDEProperty(Bstr("VideoChannel/Enabled").raw(), videoChannel.asOutParam());
            BOOL fVideoChannel =    (videoChannel.compare(Bstr("true"), Bstr::CaseInsensitive)== 0)
                                 || (videoChannel == "1");
            Bstr videoChannelQuality;
            vrdeServer->GetVRDEProperty(Bstr("VideoChannel/Quality").raw(), videoChannelQuality.asOutParam());
            AuthType_T authType;
            const char *strAuthType;
            vrdeServer->COMGETTER(AuthType)(&authType);
            switch (authType)
            {
                case AuthType_Null:
                    strAuthType = "null";
                    break;
                case AuthType_External:
                    strAuthType = "external";
                    break;
                case AuthType_Guest:
                    strAuthType = "guest";
                    break;
                default:
                    strAuthType = "unknown";
                    break;
            }
            if (console)
            {
                ComPtr<IVRDEServerInfo> vrdeServerInfo;
                CHECK_ERROR_RET(console, COMGETTER(VRDEServerInfo)(vrdeServerInfo.asOutParam()), rc);
                if (!vrdeServerInfo.isNull())
                {
                    rc = vrdeServerInfo->COMGETTER(Port)(&currentPort);
                    if (rc == E_ACCESSDENIED)
                    {
                        currentPort = -1; /* VM not powered up */
                    }
                    if (FAILED(rc))
                    {
                        com::ErrorInfo info(vrdeServerInfo, COM_IIDOF(IVRDEServerInfo));
                        GluePrintErrorInfo(info);
                        return rc;
                    }
                }
            }
            if (details == VMINFO_MACHINEREADABLE)
            {
                RTPrintf("vrde=\"on\"\n");
                RTPrintf("vrdeport=%d\n", currentPort);
                RTPrintf("vrdeports=\"%ls\"\n", ports.raw());
                RTPrintf("vrdeaddress=\"%ls\"\n", address.raw());
                RTPrintf("vrdeauthtype=\"%s\"\n", strAuthType);
                RTPrintf("vrdemulticon=\"%s\"\n", fMultiCon ? "on" : "off");
                RTPrintf("vrdereusecon=\"%s\"\n", fReuseCon ? "on" : "off");
                RTPrintf("vrdevideochannel=\"%s\"\n", fVideoChannel ? "on" : "off");
                if (fVideoChannel)
                    RTPrintf("vrdevideochannelquality=\"%ls\"\n", videoChannelQuality.raw());
            }
            else
            {
                if (address.isEmpty())
                    address = "0.0.0.0";
                RTPrintf("VRDE:            enabled (Address %ls, Ports %ls, MultiConn: %s, ReuseSingleConn: %s, Authentication type: %s)\n", address.raw(), ports.raw(), fMultiCon ? "on" : "off", fReuseCon ? "on" : "off", strAuthType);
                if (console && currentPort != -1 && currentPort != 0)
                   RTPrintf("VRDE port:       %d\n", currentPort);
                if (fVideoChannel)
                    RTPrintf("Video redirection: enabled (Quality %ls)\n", videoChannelQuality.raw());
                else
                    RTPrintf("Video redirection: disabled\n");
            }
            com::SafeArray<BSTR> aProperties;
            if (SUCCEEDED(vrdeServer->COMGETTER(VRDEProperties)(ComSafeArrayAsOutParam(aProperties))))
            {
                unsigned i;
                for (i = 0; i < aProperties.size(); ++i)
                {
                    Bstr value;
                    vrdeServer->GetVRDEProperty(aProperties[i], value.asOutParam());
                    if (details == VMINFO_MACHINEREADABLE)
                    {
                        if (value.isEmpty())
                            RTPrintf("vrdeproperty[%ls]=<not set>\n", aProperties[i]);
                        else
                            RTPrintf("vrdeproperty[%ls]=\"%ls\"\n", aProperties[i], value.raw());
                    }
                    else
                    {
                        if (value.isEmpty())
                            RTPrintf("VRDE property: %-10lS = <not set>\n", aProperties[i]);
                        else
                            RTPrintf("VRDE property: %-10lS = \"%ls\"\n", aProperties[i], value.raw());
                    }
                }
            }
        }
        else
        {
            if (details == VMINFO_MACHINEREADABLE)
                RTPrintf("vrde=\"off\"\n");
            else
                RTPrintf("VRDE:            disabled\n");
        }
    }

    /*
     * USB.
     */
    ComPtr<IUSBController> USBCtl;
    rc = machine->COMGETTER(USBController)(USBCtl.asOutParam());
    if (SUCCEEDED(rc))
    {
        BOOL fEnabled;
        BOOL fEHCIEnabled;
        rc = USBCtl->COMGETTER(Enabled)(&fEnabled);
        if (FAILED(rc))
            fEnabled = false;
        if (details == VMINFO_MACHINEREADABLE)
            RTPrintf("usb=\"%s\"\n", fEnabled ? "on" : "off");
        else
            RTPrintf("USB:             %s\n", fEnabled ? "enabled" : "disabled");

        rc = USBCtl->COMGETTER(EnabledEHCI)(&fEHCIEnabled);
        if (FAILED(rc))
            fEHCIEnabled = false;
        if (details == VMINFO_MACHINEREADABLE)
            RTPrintf("ehci=\"%s\"\n", fEHCIEnabled ? "on" : "off");
        else
            RTPrintf("EHCI:            %s\n", fEHCIEnabled ? "enabled" : "disabled");

        SafeIfaceArray <IUSBDeviceFilter> Coll;
        rc = USBCtl->COMGETTER(DeviceFilters)(ComSafeArrayAsOutParam(Coll));
        if (SUCCEEDED(rc))
        {
        if (details != VMINFO_MACHINEREADABLE)
            RTPrintf("\nUSB Device Filters:\n\n");

            if (Coll.size() == 0)
            {
                if (details != VMINFO_MACHINEREADABLE)
                    RTPrintf("<none>\n\n");
            }
            else
            {
                for (size_t index = 0; index < Coll.size(); ++index)
                {
                    ComPtr<IUSBDeviceFilter> DevPtr = Coll[index];

                    /* Query info. */

                    if (details != VMINFO_MACHINEREADABLE)
                        RTPrintf("Index:            %zu\n", index);

                    BOOL bActive = FALSE;
                    CHECK_ERROR_RET(DevPtr, COMGETTER(Active)(&bActive), rc);
                    if (details == VMINFO_MACHINEREADABLE)
                        RTPrintf("USBFilterActive%zu=\"%s\"\n", index + 1, bActive ? "on" : "off");
                    else
                        RTPrintf("Active:           %s\n", bActive ? "yes" : "no");

                    Bstr bstr;
                    CHECK_ERROR_RET(DevPtr, COMGETTER(Name)(bstr.asOutParam()), rc);
                    if (details == VMINFO_MACHINEREADABLE)
                        RTPrintf("USBFilterName%zu=\"%ls\"\n", index + 1, bstr.raw());
                    else
                        RTPrintf("Name:             %ls\n", bstr.raw());
                    CHECK_ERROR_RET(DevPtr, COMGETTER(VendorId)(bstr.asOutParam()), rc);
                    if (details == VMINFO_MACHINEREADABLE)
                        RTPrintf("USBFilterVendorId%zu=\"%ls\"\n", index + 1, bstr.raw());
                    else
                        RTPrintf("VendorId:         %ls\n", bstr.raw());
                    CHECK_ERROR_RET(DevPtr, COMGETTER(ProductId)(bstr.asOutParam()), rc);
                    if (details == VMINFO_MACHINEREADABLE)
                        RTPrintf("USBFilterProductId%zu=\"%ls\"\n", index + 1, bstr.raw());
                    else
                        RTPrintf("ProductId:        %ls\n", bstr.raw());
                    CHECK_ERROR_RET(DevPtr, COMGETTER(Revision)(bstr.asOutParam()), rc);
                    if (details == VMINFO_MACHINEREADABLE)
                        RTPrintf("USBFilterRevision%zu=\"%ls\"\n", index + 1, bstr.raw());
                    else
                        RTPrintf("Revision:         %ls\n", bstr.raw());
                    CHECK_ERROR_RET(DevPtr, COMGETTER(Manufacturer)(bstr.asOutParam()), rc);
                    if (details == VMINFO_MACHINEREADABLE)
                        RTPrintf("USBFilterManufacturer%zu=\"%ls\"\n", index + 1, bstr.raw());
                    else
                        RTPrintf("Manufacturer:     %ls\n", bstr.raw());
                    CHECK_ERROR_RET(DevPtr, COMGETTER(Product)(bstr.asOutParam()), rc);
                    if (details == VMINFO_MACHINEREADABLE)
                        RTPrintf("USBFilterProduct%zu=\"%ls\"\n", index + 1, bstr.raw());
                    else
                        RTPrintf("Product:          %ls\n", bstr.raw());
                    CHECK_ERROR_RET(DevPtr, COMGETTER(Remote)(bstr.asOutParam()), rc);
                    if (details == VMINFO_MACHINEREADABLE)
                        RTPrintf("USBFilterRemote%zu=\"%ls\"\n", index + 1, bstr.raw());
                    else
                        RTPrintf("Remote:           %ls\n", bstr.raw());
                    CHECK_ERROR_RET(DevPtr, COMGETTER(SerialNumber)(bstr.asOutParam()), rc);
                    if (details == VMINFO_MACHINEREADABLE)
                        RTPrintf("USBFilterSerialNumber%zu=\"%ls\"\n", index + 1, bstr.raw());
                    else
                        RTPrintf("Serial Number:    %ls\n", bstr.raw());
                    if (details != VMINFO_MACHINEREADABLE)
                    {
                        ULONG fMaskedIfs;
                        CHECK_ERROR_RET(DevPtr, COMGETTER(MaskedInterfaces)(&fMaskedIfs), rc);
                        if (fMaskedIfs)
                            RTPrintf("Masked Interfaces: %#010x\n", fMaskedIfs);
                        RTPrintf("\n");
                    }
                }
            }
        }

        if (console)
        {
            /* scope */
            {
                if (details != VMINFO_MACHINEREADABLE)
                    RTPrintf("Available remote USB devices:\n\n");

                SafeIfaceArray <IHostUSBDevice> coll;
                CHECK_ERROR_RET(console, COMGETTER(RemoteUSBDevices)(ComSafeArrayAsOutParam(coll)), rc);

                if (coll.size() == 0)
                {
                    if (details != VMINFO_MACHINEREADABLE)
                        RTPrintf("<none>\n\n");
                }
                else
                {
                    for (size_t index = 0; index < coll.size(); ++index)
                    {
                        ComPtr <IHostUSBDevice> dev = coll[index];

                        /* Query info. */
                        Bstr id;
                        CHECK_ERROR_RET(dev, COMGETTER(Id)(id.asOutParam()), rc);
                        USHORT usVendorId;
                        CHECK_ERROR_RET(dev, COMGETTER(VendorId)(&usVendorId), rc);
                        USHORT usProductId;
                        CHECK_ERROR_RET(dev, COMGETTER(ProductId)(&usProductId), rc);
                        USHORT bcdRevision;
                        CHECK_ERROR_RET(dev, COMGETTER(Revision)(&bcdRevision), rc);

                        if (details == VMINFO_MACHINEREADABLE)
                            RTPrintf("USBRemoteUUID%zu=\"%s\"\n"
                                     "USBRemoteVendorId%zu=\"%#06x\"\n"
                                     "USBRemoteProductId%zu=\"%#06x\"\n"
                                     "USBRemoteRevision%zu=\"%#04x%02x\"\n",
                                     index + 1, Utf8Str(id).c_str(),
                                     index + 1, usVendorId,
                                     index + 1, usProductId,
                                     index + 1, bcdRevision >> 8, bcdRevision & 0xff);
                        else
                            RTPrintf("UUID:               %s\n"
                                     "VendorId:           %#06x (%04X)\n"
                                     "ProductId:          %#06x (%04X)\n"
                                     "Revision:           %u.%u (%02u%02u)\n",
                                     Utf8Str(id).c_str(),
                                     usVendorId, usVendorId, usProductId, usProductId,
                                     bcdRevision >> 8, bcdRevision & 0xff,
                                     bcdRevision >> 8, bcdRevision & 0xff);

                        /* optional stuff. */
                        Bstr bstr;
                        CHECK_ERROR_RET(dev, COMGETTER(Manufacturer)(bstr.asOutParam()), rc);
                        if (!bstr.isEmpty())
                        {
                            if (details == VMINFO_MACHINEREADABLE)
                                RTPrintf("USBRemoteManufacturer%zu=\"%ls\"\n", index + 1, bstr.raw());
                            else
                                RTPrintf("Manufacturer:       %ls\n", bstr.raw());
                        }
                        CHECK_ERROR_RET(dev, COMGETTER(Product)(bstr.asOutParam()), rc);
                        if (!bstr.isEmpty())
                        {
                            if (details == VMINFO_MACHINEREADABLE)
                                RTPrintf("USBRemoteProduct%zu=\"%ls\"\n", index + 1, bstr.raw());
                            else
                                RTPrintf("Product:            %ls\n", bstr.raw());
                        }
                        CHECK_ERROR_RET(dev, COMGETTER(SerialNumber)(bstr.asOutParam()), rc);
                        if (!bstr.isEmpty())
                        {
                            if (details == VMINFO_MACHINEREADABLE)
                                RTPrintf("USBRemoteSerialNumber%zu=\"%ls\"\n", index + 1, bstr.raw());
                            else
                                RTPrintf("SerialNumber:       %ls\n", bstr.raw());
                        }
                        CHECK_ERROR_RET(dev, COMGETTER(Address)(bstr.asOutParam()), rc);
                        if (!bstr.isEmpty())
                        {
                            if (details == VMINFO_MACHINEREADABLE)
                                RTPrintf("USBRemoteAddress%zu=\"%ls\"\n", index + 1, bstr.raw());
                            else
                                RTPrintf("Address:            %ls\n", bstr.raw());
                        }

                        if (details != VMINFO_MACHINEREADABLE)
                            RTPrintf("\n");
                    }
                }
            }

            /* scope */
            {
                if (details != VMINFO_MACHINEREADABLE)
                    RTPrintf("Currently Attached USB Devices:\n\n");

                SafeIfaceArray <IUSBDevice> coll;
                CHECK_ERROR_RET(console, COMGETTER(USBDevices)(ComSafeArrayAsOutParam(coll)), rc);

                if (coll.size() == 0)
                {
                    if (details != VMINFO_MACHINEREADABLE)
                        RTPrintf("<none>\n\n");
                }
                else
                {
                    for (size_t index = 0; index < coll.size(); ++index)
                    {
                        ComPtr <IUSBDevice> dev = coll[index];

                        /* Query info. */
                        Bstr id;
                        CHECK_ERROR_RET(dev, COMGETTER(Id)(id.asOutParam()), rc);
                        USHORT usVendorId;
                        CHECK_ERROR_RET(dev, COMGETTER(VendorId)(&usVendorId), rc);
                        USHORT usProductId;
                        CHECK_ERROR_RET(dev, COMGETTER(ProductId)(&usProductId), rc);
                        USHORT bcdRevision;
                        CHECK_ERROR_RET(dev, COMGETTER(Revision)(&bcdRevision), rc);

                        if (details == VMINFO_MACHINEREADABLE)
                            RTPrintf("USBAttachedUUID%zu=\"%s\"\n"
                                     "USBAttachedVendorId%zu=\"%#06x\"\n"
                                     "USBAttachedProductId%zu=\"%#06x\"\n"
                                     "USBAttachedRevision%zu=\"%#04x%02x\"\n",
                                     index + 1, Utf8Str(id).c_str(),
                                     index + 1, usVendorId,
                                     index + 1, usProductId,
                                     index + 1, bcdRevision >> 8, bcdRevision & 0xff);
                        else
                            RTPrintf("UUID:               %s\n"
                                     "VendorId:           %#06x (%04X)\n"
                                     "ProductId:          %#06x (%04X)\n"
                                     "Revision:           %u.%u (%02u%02u)\n",
                                     Utf8Str(id).c_str(),
                                     usVendorId, usVendorId, usProductId, usProductId,
                                     bcdRevision >> 8, bcdRevision & 0xff,
                                     bcdRevision >> 8, bcdRevision & 0xff);

                        /* optional stuff. */
                        Bstr bstr;
                        CHECK_ERROR_RET(dev, COMGETTER(Manufacturer)(bstr.asOutParam()), rc);
                        if (!bstr.isEmpty())
                        {
                            if (details == VMINFO_MACHINEREADABLE)
                                RTPrintf("USBAttachedManufacturer%zu=\"%ls\"\n", index + 1, bstr.raw());
                            else
                                RTPrintf("Manufacturer:       %ls\n", bstr.raw());
                        }
                        CHECK_ERROR_RET(dev, COMGETTER(Product)(bstr.asOutParam()), rc);
                        if (!bstr.isEmpty())
                        {
                            if (details == VMINFO_MACHINEREADABLE)
                                RTPrintf("USBAttachedProduct%zu=\"%ls\"\n", index + 1, bstr.raw());
                            else
                                RTPrintf("Product:            %ls\n", bstr.raw());
                        }
                        CHECK_ERROR_RET(dev, COMGETTER(SerialNumber)(bstr.asOutParam()), rc);
                        if (!bstr.isEmpty())
                        {
                            if (details == VMINFO_MACHINEREADABLE)
                                RTPrintf("USBAttachedSerialNumber%zu=\"%ls\"\n", index + 1, bstr.raw());
                            else
                                RTPrintf("SerialNumber:       %ls\n", bstr.raw());
                        }
                        CHECK_ERROR_RET(dev, COMGETTER(Address)(bstr.asOutParam()), rc);
                        if (!bstr.isEmpty())
                        {
                            if (details == VMINFO_MACHINEREADABLE)
                                RTPrintf("USBAttachedAddress%zu=\"%ls\"\n", index + 1, bstr.raw());
                            else
                                RTPrintf("Address:            %ls\n", bstr.raw());
                        }

                        if (details != VMINFO_MACHINEREADABLE)
                            RTPrintf("\n");
                    }
                }
            }
        }
    } /* USB */

#ifdef VBOX_WITH_PCI_PASSTHROUGH
    /* Host PCI passthrough devices */
    {
         SafeIfaceArray <IPCIDeviceAttachment> assignments;
         rc = machine->COMGETTER(PCIDeviceAssignments)(ComSafeArrayAsOutParam(assignments));
         if (SUCCEEDED(rc))
         {
             if (assignments.size() > 0 && (details != VMINFO_MACHINEREADABLE))
             {
                 RTPrintf("\nAttached physical PCI devices:\n\n");
             }

             for (size_t index = 0; index < assignments.size(); ++index)
             {
                 ComPtr<IPCIDeviceAttachment> Assignment = assignments[index];
                 char szHostPCIAddress[32], szGuestPCIAddress[32];
                 LONG iHostPCIAddress = -1, iGuestPCIAddress = -1;
                 Bstr DevName;

                 Assignment->COMGETTER(Name)(DevName.asOutParam());
                 Assignment->COMGETTER(HostAddress)(&iHostPCIAddress);
                 Assignment->COMGETTER(GuestAddress)(&iGuestPCIAddress);
                 PCIBusAddress().fromLong(iHostPCIAddress).format(szHostPCIAddress, sizeof(szHostPCIAddress));
                 PCIBusAddress().fromLong(iGuestPCIAddress).format(szGuestPCIAddress, sizeof(szGuestPCIAddress));

                 if (details == VMINFO_MACHINEREADABLE)
                     RTPrintf("AttachedHostPCI=%s,%s\n", szHostPCIAddress, szGuestPCIAddress);
                 else
                     RTPrintf("   Host device %ls at %s attached as %s\n", DevName.raw(), szHostPCIAddress, szGuestPCIAddress);
             }

             if (assignments.size() > 0 && (details != VMINFO_MACHINEREADABLE))
             {
                 RTPrintf("\n");
             }
         }
    }
    /* Host PCI passthrough devices */
#endif

    /*
     * Bandwidth groups
     */
    if (details != VMINFO_MACHINEREADABLE)
        RTPrintf("Bandwidth groups:  ");
    {
        ComPtr<IBandwidthControl> bwCtrl;
        CHECK_ERROR_RET(machine, COMGETTER(BandwidthControl)(bwCtrl.asOutParam()), rc);

        rc = showBandwidthGroups(bwCtrl, details);
    }


    /*
     * Shared folders
     */
    if (details != VMINFO_MACHINEREADABLE)
        RTPrintf("Shared folders:  ");
    uint32_t numSharedFolders = 0;
#if 0 // not yet implemented
    /* globally shared folders first */
    {
        SafeIfaceArray <ISharedFolder> sfColl;
        CHECK_ERROR_RET(virtualBox, COMGETTER(SharedFolders)(ComSafeArrayAsOutParam(sfColl)), rc);
        for (size_t i = 0; i < sfColl.size(); ++i)
        {
            ComPtr<ISharedFolder> sf = sfColl[i];
            Bstr name, hostPath;
            sf->COMGETTER(Name)(name.asOutParam());
            sf->COMGETTER(HostPath)(hostPath.asOutParam());
            RTPrintf("Name: '%ls', Host path: '%ls' (global mapping)\n", name.raw(), hostPath.raw());
            ++numSharedFolders;
        }
    }
#endif
    /* now VM mappings */
    {
        com::SafeIfaceArray <ISharedFolder> folders;

        CHECK_ERROR_RET(machine, COMGETTER(SharedFolders)(ComSafeArrayAsOutParam(folders)), rc);

        for (size_t i = 0; i < folders.size(); ++i)
        {
            ComPtr <ISharedFolder> sf = folders[i];

            Bstr name, hostPath;
            BOOL writable;
            sf->COMGETTER(Name)(name.asOutParam());
            sf->COMGETTER(HostPath)(hostPath.asOutParam());
            sf->COMGETTER(Writable)(&writable);
            if (!numSharedFolders && details != VMINFO_MACHINEREADABLE)
                RTPrintf("\n\n");
            if (details == VMINFO_MACHINEREADABLE)
            {
                RTPrintf("SharedFolderNameMachineMapping%zu=\"%ls\"\n", i + 1,
                         name.raw());
                RTPrintf("SharedFolderPathMachineMapping%zu=\"%ls\"\n", i + 1,
                         hostPath.raw());
            }
            else
                RTPrintf("Name: '%ls', Host path: '%ls' (machine mapping), %s\n",
                         name.raw(), hostPath.raw(), writable ? "writable" : "readonly");
            ++numSharedFolders;
        }
    }
    /* transient mappings */
    if (console)
    {
        com::SafeIfaceArray <ISharedFolder> folders;

        CHECK_ERROR_RET(console, COMGETTER(SharedFolders)(ComSafeArrayAsOutParam(folders)), rc);

        for (size_t i = 0; i < folders.size(); ++i)
        {
            ComPtr <ISharedFolder> sf = folders[i];

            Bstr name, hostPath;
            sf->COMGETTER(Name)(name.asOutParam());
            sf->COMGETTER(HostPath)(hostPath.asOutParam());
            if (!numSharedFolders && details != VMINFO_MACHINEREADABLE)
                RTPrintf("\n\n");
            if (details == VMINFO_MACHINEREADABLE)
            {
                RTPrintf("SharedFolderNameTransientMapping%zu=\"%ls\"\n", i + 1,
                         name.raw());
                RTPrintf("SharedFolderPathTransientMapping%zu=\"%ls\"\n", i + 1,
                         hostPath.raw());
            }
            else
                RTPrintf("Name: '%ls', Host path: '%ls' (transient mapping)\n", name.raw(), hostPath.raw());
            ++numSharedFolders;
        }
    }
    if (!numSharedFolders && details != VMINFO_MACHINEREADABLE)
        RTPrintf("<none>\n");
    if (details != VMINFO_MACHINEREADABLE)
        RTPrintf("\n");

    if (console)
    {
        /*
         * Live VRDE info.
         */
        ComPtr<IVRDEServerInfo> vrdeServerInfo;
        CHECK_ERROR_RET(console, COMGETTER(VRDEServerInfo)(vrdeServerInfo.asOutParam()), rc);
        BOOL    Active = FALSE;
        ULONG   NumberOfClients = 0;
        LONG64  BeginTime = 0;
        LONG64  EndTime = 0;
        LONG64  BytesSent = 0;
        LONG64  BytesSentTotal = 0;
        LONG64  BytesReceived = 0;
        LONG64  BytesReceivedTotal = 0;
        Bstr    User;
        Bstr    Domain;
        Bstr    ClientName;
        Bstr    ClientIP;
        ULONG   ClientVersion = 0;
        ULONG   EncryptionStyle = 0;

        if (!vrdeServerInfo.isNull())
        {
            CHECK_ERROR_RET(vrdeServerInfo, COMGETTER(Active)(&Active), rc);
            CHECK_ERROR_RET(vrdeServerInfo, COMGETTER(NumberOfClients)(&NumberOfClients), rc);
            CHECK_ERROR_RET(vrdeServerInfo, COMGETTER(BeginTime)(&BeginTime), rc);
            CHECK_ERROR_RET(vrdeServerInfo, COMGETTER(EndTime)(&EndTime), rc);
            CHECK_ERROR_RET(vrdeServerInfo, COMGETTER(BytesSent)(&BytesSent), rc);
            CHECK_ERROR_RET(vrdeServerInfo, COMGETTER(BytesSentTotal)(&BytesSentTotal), rc);
            CHECK_ERROR_RET(vrdeServerInfo, COMGETTER(BytesReceived)(&BytesReceived), rc);
            CHECK_ERROR_RET(vrdeServerInfo, COMGETTER(BytesReceivedTotal)(&BytesReceivedTotal), rc);
            CHECK_ERROR_RET(vrdeServerInfo, COMGETTER(User)(User.asOutParam()), rc);
            CHECK_ERROR_RET(vrdeServerInfo, COMGETTER(Domain)(Domain.asOutParam()), rc);
            CHECK_ERROR_RET(vrdeServerInfo, COMGETTER(ClientName)(ClientName.asOutParam()), rc);
            CHECK_ERROR_RET(vrdeServerInfo, COMGETTER(ClientIP)(ClientIP.asOutParam()), rc);
            CHECK_ERROR_RET(vrdeServerInfo, COMGETTER(ClientVersion)(&ClientVersion), rc);
            CHECK_ERROR_RET(vrdeServerInfo, COMGETTER(EncryptionStyle)(&EncryptionStyle), rc);
        }

        if (details == VMINFO_MACHINEREADABLE)
            RTPrintf("VRDEActiveConnection=\"%s\"\n", Active ? "on": "off");
        else
            RTPrintf("VRDE Connection:    %s\n", Active? "active": "not active");

        if (details == VMINFO_MACHINEREADABLE)
            RTPrintf("VRDEClients=%d\n", NumberOfClients);
        else
            RTPrintf("Clients so far:     %d\n", NumberOfClients);

        if (NumberOfClients > 0)
        {
            char timestr[128];

            if (Active)
            {
                makeTimeStr(timestr, sizeof(timestr), BeginTime);
                if (details == VMINFO_MACHINEREADABLE)
                    RTPrintf("VRDEStartTime=\"%s\"\n", timestr);
                else
                    RTPrintf("Start time:         %s\n", timestr);
            }
            else
            {
                makeTimeStr(timestr, sizeof(timestr), BeginTime);
                if (details == VMINFO_MACHINEREADABLE)
                    RTPrintf("VRDELastStartTime=\"%s\"\n", timestr);
                else
                    RTPrintf("Last started:       %s\n", timestr);
                makeTimeStr(timestr, sizeof(timestr), EndTime);
                if (details == VMINFO_MACHINEREADABLE)
                    RTPrintf("VRDELastEndTime=\"%s\"\n", timestr);
                else
                    RTPrintf("Last ended:         %s\n", timestr);
            }

            int64_t ThroughputSend = 0;
            int64_t ThroughputReceive = 0;
            if (EndTime != BeginTime)
            {
                ThroughputSend = (BytesSent * 1000) / (EndTime - BeginTime);
                ThroughputReceive = (BytesReceived * 1000) / (EndTime - BeginTime);
            }

            if (details == VMINFO_MACHINEREADABLE)
            {
                RTPrintf("VRDEBytesSent=%lld\n", BytesSent);
                RTPrintf("VRDEThroughputSend=%lld\n", ThroughputSend);
                RTPrintf("VRDEBytesSentTotal=%lld\n", BytesSentTotal);

                RTPrintf("VRDEBytesReceived=%lld\n", BytesReceived);
                RTPrintf("VRDEThroughputReceive=%lld\n", ThroughputReceive);
                RTPrintf("VRDEBytesReceivedTotal=%lld\n", BytesReceivedTotal);
            }
            else
            {
                RTPrintf("Sent:               %lld Bytes\n", BytesSent);
                RTPrintf("Average speed:      %lld B/s\n", ThroughputSend);
                RTPrintf("Sent total:         %lld Bytes\n", BytesSentTotal);

                RTPrintf("Received:           %lld Bytes\n", BytesReceived);
                RTPrintf("Speed:              %lld B/s\n", ThroughputReceive);
                RTPrintf("Received total:     %lld Bytes\n", BytesReceivedTotal);
            }

            if (Active)
            {
                if (details == VMINFO_MACHINEREADABLE)
                {
                    RTPrintf("VRDEUserName=\"%ls\"\n", User.raw());
                    RTPrintf("VRDEDomain=\"%ls\"\n", Domain.raw());
                    RTPrintf("VRDEClientName=\"%ls\"\n", ClientName.raw());
                    RTPrintf("VRDEClientIP=\"%ls\"\n", ClientIP.raw());
                    RTPrintf("VRDEClientVersion=%d\n",  ClientVersion);
                    RTPrintf("VRDEEncryption=\"%s\"\n", EncryptionStyle == 0? "RDP4": "RDP5 (X.509)");
                }
                else
                {
                    RTPrintf("User name:          %ls\n", User.raw());
                    RTPrintf("Domain:             %ls\n", Domain.raw());
                    RTPrintf("Client name:        %ls\n", ClientName.raw());
                    RTPrintf("Client IP:          %ls\n", ClientIP.raw());
                    RTPrintf("Client version:     %d\n",  ClientVersion);
                    RTPrintf("Encryption:         %s\n", EncryptionStyle == 0? "RDP4": "RDP5 (X.509)");
                }
            }
        }

        if (details != VMINFO_MACHINEREADABLE)
            RTPrintf("\n");
    }

    if (    details == VMINFO_STANDARD
        ||  details == VMINFO_FULL
        ||  details == VMINFO_MACHINEREADABLE)
    {
        Bstr description;
        machine->COMGETTER(Description)(description.asOutParam());
        if (!description.isEmpty())
        {
            if (details == VMINFO_MACHINEREADABLE)
                RTPrintf("description=\"%ls\"\n", description.raw());
            else
                RTPrintf("Description:\n%ls\n", description.raw());
        }
    }

    if (details != VMINFO_MACHINEREADABLE)
        RTPrintf("Guest:\n\n");

    ULONG guestVal;
    rc = machine->COMGETTER(MemoryBalloonSize)(&guestVal);
    if (SUCCEEDED(rc))
    {
        if (details == VMINFO_MACHINEREADABLE)
            RTPrintf("GuestMemoryBalloon=%d\n", guestVal);
        else
            RTPrintf("Configured memory balloon size:      %d MB\n", guestVal);
    }

    if (console)
    {
        ComPtr<IGuest> guest;
        rc = console->COMGETTER(Guest)(guest.asOutParam());
        if (SUCCEEDED(rc) && !guest.isNull())
        {
            Bstr guestString;
            rc = guest->COMGETTER(OSTypeId)(guestString.asOutParam());
            if (   SUCCEEDED(rc)
                && !guestString.isEmpty())
            {
                if (details == VMINFO_MACHINEREADABLE)
                    RTPrintf("GuestOSType=\"%ls\"\n", guestString.raw());
                else
                    RTPrintf("OS type:                             %ls\n", guestString.raw());
            }

            AdditionsRunLevelType_T guestRunLevel; /** @todo Add a runlevel-to-string (e.g. 0 = "None") method? */
            rc = guest->COMGETTER(AdditionsRunLevel)(&guestRunLevel);
            if (SUCCEEDED(rc))
            {
                if (details == VMINFO_MACHINEREADABLE)
                    RTPrintf("GuestAdditionsRunLevel=%u\n", guestRunLevel);
                else
                    RTPrintf("Additions run level:                 %u\n", guestRunLevel);
            }

            rc = guest->COMGETTER(AdditionsVersion)(guestString.asOutParam());
            if (   SUCCEEDED(rc)
                && !guestString.isEmpty())
            {
                ULONG uRevision;
                rc = guest->COMGETTER(AdditionsRevision)(&uRevision);
                if (FAILED(rc))
                    uRevision = 0;

                if (details == VMINFO_MACHINEREADABLE)
                    RTPrintf("GuestAdditionsVersion=\"%ls r%u\"\n", guestString.raw(), uRevision);
                else
                    RTPrintf("Additions version:                   %ls r%u\n\n", guestString.raw(), uRevision);
            }

            if (details != VMINFO_MACHINEREADABLE)
                RTPrintf("\nGuest Facilities:\n\n");

            /* Print information about known Guest Additions facilities: */
            SafeIfaceArray <IAdditionsFacility> collFac;
            CHECK_ERROR_RET(guest, COMGETTER(Facilities)(ComSafeArrayAsOutParam(collFac)), rc);
            LONG64 lLastUpdatedMS;
            char szLastUpdated[32];
            AdditionsFacilityStatus_T curStatus;
            for (size_t index = 0; index < collFac.size(); ++index)
            {
                ComPtr<IAdditionsFacility> fac = collFac[index];
                if (fac)
                {
                    CHECK_ERROR_RET(fac, COMGETTER(Name)(guestString.asOutParam()), rc);
                    if (!guestString.isEmpty())
                    {
                        CHECK_ERROR_RET(fac, COMGETTER(Status)(&curStatus), rc);
                        CHECK_ERROR_RET(fac, COMGETTER(LastUpdated)(&lLastUpdatedMS), rc);
                        if (details == VMINFO_MACHINEREADABLE)
                            RTPrintf("GuestAdditionsFacility_%ls=%u,%lld\n",
                                     guestString.raw(), curStatus, lLastUpdatedMS);
                        else
                        {
                            makeTimeStr(szLastUpdated, sizeof(szLastUpdated), lLastUpdatedMS);
                            RTPrintf("Facility \"%ls\": %s (last update: %s)\n",
                                     guestString.raw(), facilityStateToName(curStatus, false /* No short naming */), szLastUpdated);
                        }
                    }
                    else
                        AssertMsgFailed(("Facility with undefined name retrieved!\n"));
                }
                else
                    AssertMsgFailed(("Invalid facility returned!\n"));
            }
            if (!collFac.size() && details != VMINFO_MACHINEREADABLE)
                RTPrintf("No active facilities.\n");
        }
    }

    if (details != VMINFO_MACHINEREADABLE)
        RTPrintf("\n");

    /*
     * snapshots
     */
    ComPtr<ISnapshot> snapshot;
    rc = machine->FindSnapshot(Bstr().raw(), snapshot.asOutParam());
    if (SUCCEEDED(rc) && snapshot)
    {
        ComPtr<ISnapshot> currentSnapshot;
        rc = machine->COMGETTER(CurrentSnapshot)(currentSnapshot.asOutParam());
        if (SUCCEEDED(rc))
        {
            if (details != VMINFO_MACHINEREADABLE)
                RTPrintf("Snapshots:\n\n");
            showSnapshots(snapshot, currentSnapshot, details);
        }
    }

    if (details != VMINFO_MACHINEREADABLE)
        RTPrintf("\n");
    return S_OK;
}

#if defined(_MSC_VER)
# pragma optimize("", on)
#endif

static const RTGETOPTDEF g_aShowVMInfoOptions[] =
{
    { "--details",          'D', RTGETOPT_REQ_NOTHING },
    { "-details",           'D', RTGETOPT_REQ_NOTHING },    // deprecated
    { "--machinereadable",  'M', RTGETOPT_REQ_NOTHING },
    { "-machinereadable",   'M', RTGETOPT_REQ_NOTHING },    // deprecated
    { "--log",              'l', RTGETOPT_REQ_UINT32 },
};

int handleShowVMInfo(HandlerArg *a)
{
    HRESULT rc;
    const char *VMNameOrUuid = NULL;
    bool fLog = false;
    uint32_t uLogIdx = 0;
    bool fDetails = false;
    bool fMachinereadable = false;

    int c;
    RTGETOPTUNION ValueUnion;
    RTGETOPTSTATE GetState;
    // start at 0 because main() has hacked both the argc and argv given to us
    RTGetOptInit(&GetState, a->argc, a->argv, g_aShowVMInfoOptions, RT_ELEMENTS(g_aShowVMInfoOptions),
                 0, RTGETOPTINIT_FLAGS_NO_STD_OPTS);
    while ((c = RTGetOpt(&GetState, &ValueUnion)))
    {
        switch (c)
        {
            case 'D':   // --details
                fDetails = true;
                break;

            case 'M':   // --machinereadable
                fMachinereadable = true;
                break;

            case 'l':   // --log
                fLog = true;
                uLogIdx = ValueUnion.u32;
                break;

            case VINF_GETOPT_NOT_OPTION:
                if (!VMNameOrUuid)
                    VMNameOrUuid = ValueUnion.psz;
                else
                    return errorSyntax(USAGE_SHOWVMINFO, "Invalid parameter '%s'", ValueUnion.psz);
                break;

            default:
                if (c > 0)
                {
                    if (RT_C_IS_PRINT(c))
                        return errorSyntax(USAGE_SHOWVMINFO, "Invalid option -%c", c);
                    else
                        return errorSyntax(USAGE_SHOWVMINFO, "Invalid option case %i", c);
                }
                else if (c == VERR_GETOPT_UNKNOWN_OPTION)
                    return errorSyntax(USAGE_SHOWVMINFO, "unknown option: %s\n", ValueUnion.psz);
                else if (ValueUnion.pDef)
                    return errorSyntax(USAGE_SHOWVMINFO, "%s: %Rrs", ValueUnion.pDef->pszLong, c);
                else
                    return errorSyntax(USAGE_SHOWVMINFO, "error: %Rrs", c);
        }
    }

    /* check for required options */
    if (!VMNameOrUuid)
        return errorSyntax(USAGE_SHOWVMINFO, "VM name or UUID required");

    /* try to find the given machine */
    ComPtr <IMachine> machine;
    CHECK_ERROR(a->virtualBox, FindMachine(Bstr(VMNameOrUuid).raw(),
                                           machine.asOutParam()));
    if (FAILED(rc))
        return 1;

    /* Printing the log is exclusive. */
    if (fLog && (fMachinereadable || fDetails))
        return errorSyntax(USAGE_SHOWVMINFO, "Option --log is exclusive");

    if (fLog)
    {
        ULONG64 uOffset = 0;
        SafeArray<BYTE> aLogData;
        ULONG cbLogData;
        while (true)
        {
            /* Reset the array */
            aLogData.setNull();
            /* Fetch a chunk of the log file */
            CHECK_ERROR_BREAK(machine, ReadLog(uLogIdx, uOffset, _1M,
                                               ComSafeArrayAsOutParam(aLogData)));
            cbLogData = aLogData.size();
            if (cbLogData == 0)
                break;
            /* aLogData has a platform dependent line ending, standardize on
             * Unix style, as RTStrmWrite does the LF -> CR/LF replacement on
             * Windows. Otherwise we end up with CR/CR/LF on Windows. */
            ULONG cbLogDataPrint = cbLogData;
            for (BYTE *s = aLogData.raw(), *d = s;
                 s - aLogData.raw() < (ssize_t)cbLogData;
                 s++, d++)
            {
                if (*s == '\r')
                {
                    /* skip over CR, adjust destination */
                    d--;
                    cbLogDataPrint--;
                }
                else if (s != d)
                    *d = *s;
            }
            RTStrmWrite(g_pStdOut, aLogData.raw(), cbLogDataPrint);
            uOffset += cbLogData;
        }
    }
    else
    {
        /* 2nd option can be -details or -argdump */
        VMINFO_DETAILS details = VMINFO_NONE;
        if (fMachinereadable)
            details = VMINFO_MACHINEREADABLE;
        else if (fDetails)
            details = VMINFO_FULL;
        else
            details = VMINFO_STANDARD;

        ComPtr<IConsole> console;

        /* open an existing session for the VM */
        rc = machine->LockMachine(a->session, LockType_Shared);
        if (SUCCEEDED(rc))
            /* get the session machine */
            rc = a->session->COMGETTER(Machine)(machine.asOutParam());
        if (SUCCEEDED(rc))
            /* get the session console */
            rc = a->session->COMGETTER(Console)(console.asOutParam());

        rc = showVMInfo(a->virtualBox, machine, details, console);

        if (console)
            a->session->UnlockMachine();
    }

    return SUCCEEDED(rc) ? 0 : 1;
}

#endif /* !VBOX_ONLY_DOCS */
/* vi: set tabstop=4 shiftwidth=4 expandtab: */
