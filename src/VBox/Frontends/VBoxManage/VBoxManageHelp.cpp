/* $Id: VBoxManageHelp.cpp $ */
/** @file
 * VBoxManage - help and other message output.
 */

/*
 * Copyright (C) 2006-2013 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */


/*******************************************************************************
*   Header Files                                                               *
*******************************************************************************/
#include <VBox/version.h>

#include <iprt/buildconfig.h>
#include <iprt/ctype.h>
#include <iprt/err.h>
#include <iprt/getopt.h>
#include <iprt/stream.h>

#include "VBoxManage.h"



void showLogo(PRTSTREAM pStrm)
{
    static bool s_fShown; /* show only once */

    if (!s_fShown)
    {
        RTStrmPrintf(pStrm, VBOX_PRODUCT " Command Line Management Interface Version "
                     VBOX_VERSION_STRING "\n"
                     "(C) 2005-" VBOX_C_YEAR " " VBOX_VENDOR "\n"
                     "All rights reserved.\n"
                     "\n");
        s_fShown = true;
    }
}

void printUsage(USAGECATEGORY u64Cmd, PRTSTREAM pStrm)
{
    bool fDumpOpts = false;
#ifdef RT_OS_LINUX
    bool fLinux = true;
#else
    bool fLinux = false;
#endif
#ifdef RT_OS_WINDOWS
    bool fWin = true;
#else
    bool fWin = false;
#endif
#ifdef RT_OS_SOLARIS
    bool fSolaris = true;
#else
    bool fSolaris = false;
#endif
#ifdef RT_OS_FREEBSD
    bool fFreeBSD = true;
#else
    bool fFreeBSD = false;
#endif
#ifdef RT_OS_DARWIN
    bool fDarwin = true;
#else
    bool fDarwin = false;
#endif
#ifdef VBOX_WITH_VBOXSDL
    bool fVBoxSDL = true;
#else
    bool fVBoxSDL = false;
#endif

    if (u64Cmd == USAGE_DUMPOPTS)
    {
        fDumpOpts = true;
        fLinux = true;
        fWin = true;
        fSolaris = true;
        fFreeBSD = true;
        fDarwin = true;
        fVBoxSDL = true;
        u64Cmd = USAGE_ALL;
    }

    RTStrmPrintf(pStrm,
                 "Usage:\n"
                 "\n");

    if (u64Cmd == USAGE_ALL)
        RTStrmPrintf(pStrm,
                     "  VBoxManage [<general option>] <command>\n"
                     " \n \n"
                     "General Options:\n \n"
                     "  [-v|--version]            print version number and exit\n"
                     "  [-q|--nologo]             suppress the logo\n"
                     "  [--settingspw <pw>]       provide the settings password\n"
                     "  [--settingspwfile <file>] provide a file containing the settings password\n"
                     " \n \n"
                     "Commands:\n \n");

    const char *pcszSep1 = " ";
    const char *pcszSep2 = "         ";
    if (u64Cmd != USAGE_ALL)
    {
        pcszSep1 = "VBoxManage";
        pcszSep2 = "";
    }

#define SEP pcszSep1, pcszSep2

    if (u64Cmd & USAGE_LIST)
        RTStrmPrintf(pStrm,
                           "%s list [--long|-l]%s vms|runningvms|ostypes|hostdvds|hostfloppies|\n"
#if defined(VBOX_WITH_NETFLT)
                     "                            bridgedifs|hostonlyifs|dhcpservers|hostinfo|\n"
#else
                     "                            bridgedifs|dhcpservers|hostinfo|\n"
#endif
                     "                            hostcpuids|hddbackends|hdds|dvds|floppies|\n"
                     "                            usbhost|usbfilters|systemproperties|extpacks|\n"
                     "                            groups\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_SHOWVMINFO)
        RTStrmPrintf(pStrm,
                           "%s showvminfo %s      <uuid>|<name> [--details]\n"
                     "                            [--machinereadable]\n"
                           "%s showvminfo %s      <uuid>|<name> --log <idx>\n"
                     "\n", SEP, SEP);

    if (u64Cmd & USAGE_REGISTERVM)
        RTStrmPrintf(pStrm,
                           "%s registervm %s      <filename>\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_UNREGISTERVM)
        RTStrmPrintf(pStrm,
                           "%s unregistervm %s    <uuid>|<name> [--delete]\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_CREATEVM)
        RTStrmPrintf(pStrm,
                           "%s createvm %s        --name <name>\n"
                     "                            [--groups <group>, ...]\n"
                     "                            [--ostype <ostype>]\n"
                     "                            [--register]\n"
                     "                            [--basefolder <path>]\n"
                     "                            [--uuid <uuid>]\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_MODIFYVM)
    {
        RTStrmPrintf(pStrm,
                           "%s modifyvm %s        <uuid|name>\n"
                     "                            [--name <name>]\n"
                     "                            [--groups <group>, ...]\n"
                     "                            [--ostype <ostype>]\n"
                     "                            [--memory <memorysize in MB>]\n"
                     "                            [--pagefusion on|off]\n"
                     "                            [--vram <vramsize in MB>]\n"
                     "                            [--acpi on|off]\n"
#ifdef VBOX_WITH_PCI_PASSTHROUGH
                     "                            [--pciattach 03:04.0]\n"
                     "                            [--pciattach 03:04.0@02:01.0]\n"
                     "                            [--pcidetach 03:04.0]\n"
#endif
                     "                            [--ioapic on|off]\n"
                     "                            [--pae on|off]\n"
                     "                            [--hpet on|off]\n"
                     "                            [--hwvirtex on|off]\n"
                     "                            [--hwvirtexexcl on|off]\n"
                     "                            [--nestedpaging on|off]\n"
                     "                            [--largepages on|off]\n"
                     "                            [--vtxvpid on|off]\n"
                     "                            [--synthcpu on|off]\n"
                     "                            [--cpuidset <leaf> <eax> <ebx> <ecx> <edx>]\n"
                     "                            [--cpuidremove <leaf>]\n"
                     "                            [--cpuidremoveall]\n"
                     "                            [--hardwareuuid <uuid>]\n"
                     "                            [--cpus <number>]\n"
                     "                            [--cpuhotplug on|off]\n"
                     "                            [--plugcpu <id>]\n"
                     "                            [--unplugcpu <id>]\n"
                     "                            [--cpuexecutioncap <1-100>]\n"
                     "                            [--rtcuseutc on|off]\n"
                     "                            [--monitorcount <number>]\n"
                     "                            [--accelerate3d on|off]\n"
#ifdef VBOX_WITH_VIDEOHWACCEL
                     "                            [--accelerate2dvideo on|off]\n"
#endif
                     "                            [--firmware bios|efi|efi32|efi64]\n"
                     "                            [--chipset ich9|piix3]\n"
                     "                            [--bioslogofadein on|off]\n"
                     "                            [--bioslogofadeout on|off]\n"
                     "                            [--bioslogodisplaytime <msec>]\n"
                     "                            [--bioslogoimagepath <imagepath>]\n"
                     "                            [--biosbootmenu disabled|menuonly|messageandmenu]\n"
                     "                            [--biossystemtimeoffset <msec>]\n"
                     "                            [--biospxedebug on|off]\n"
                     "                            [--boot<1-4> none|floppy|dvd|disk|net>]\n"
                     "                            [--nic<1-N> none|null|nat|bridged|intnet"
#if defined(VBOX_WITH_NETFLT)
                     "|hostonly"
#endif
                     "|\n"
                     "                                        generic"
                     "]\n"
                     "                            [--nictype<1-N> Am79C970A|Am79C973"
#ifdef VBOX_WITH_E1000
                  "|\n                                            82540EM|82543GC|82545EM"
#endif
#ifdef VBOX_WITH_VIRTIO
                  "|\n                                            virtio"
#endif /* VBOX_WITH_VIRTIO */
                     "]\n"
                     "                            [--cableconnected<1-N> on|off]\n"
                     "                            [--nictrace<1-N> on|off]\n"
                     "                            [--nictracefile<1-N> <filename>]\n"
                     "                            [--nicproperty<1-N> name=[value]]\n"
                     "                            [--nicspeed<1-N> <kbps>]\n"
                     "                            [--nicbootprio<1-N> <priority>]\n"
                     "                            [--nicpromisc<1-N> deny|allow-vms|allow-all]\n"
                     "                            [--nicbandwidthgroup<1-N> none|<name>]\n"
                     "                            [--bridgeadapter<1-N> none|<devicename>]\n"
#if defined(VBOX_WITH_NETFLT)
                     "                            [--hostonlyadapter<1-N> none|<devicename>]\n"
#endif
                     "                            [--intnet<1-N> <network name>]\n"
                     "                            [--natnet<1-N> <network>|default]\n"
                     "                            [--nicgenericdrv<1-N> <driver>\n"
                     "                            [--natsettings<1-N> [<mtu>],[<socksnd>],\n"
                     "                                                [<sockrcv>],[<tcpsnd>],\n"
                     "                                                [<tcprcv>]]\n"
                     "                            [--natpf<1-N> [<rulename>],tcp|udp,[<hostip>],\n"
                     "                                          <hostport>,[<guestip>],<guestport>]\n"
                     "                            [--natpf<1-N> delete <rulename>]\n"
                     "                            [--nattftpprefix<1-N> <prefix>]\n"
                     "                            [--nattftpfile<1-N> <file>]\n"
                     "                            [--nattftpserver<1-N> <ip>]\n"
                     "                            [--natbindip<1-N> <ip>\n"
                     "                            [--natdnspassdomain<1-N> on|off]\n"
                     "                            [--natdnsproxy<1-N> on|off]\n"
                     "                            [--natdnshostresolver<1-N> on|off]\n"
                     "                            [--nataliasmode<1-N> default|[log],[proxyonly],\n"
                     "                                                         [sameports]]\n"
                     "                            [--macaddress<1-N> auto|<mac>]\n"
                     "                            [--mouse ps2|usb|usbtablet\n"
                     "                            [--keyboard ps2|usb\n"
                     "                            [--uart<1-N> off|<I/O base> <IRQ>]\n"
                     "                            [--uartmode<1-N> disconnected|\n"
                     "                                             server <pipe>|\n"
                     "                                             client <pipe>|\n"
                     "                                             file <file>|\n"
                     "                                             <devicename>]\n"
#if defined(RT_OS_LINUX) || defined(RT_OS_WINDOWS)
                     "                            [--lpt<1-N> off|<I/O base> <IRQ>]\n"
                     "                            [--lptmode<1-N> <devicename>]\n"
#endif
                     "                            [--guestmemoryballoon <balloonsize in MB>]\n"
                     "                            [--audio none|null", SEP);
        if (fWin)
        {
#ifdef VBOX_WITH_WINMM
            RTStrmPrintf(pStrm, "|winmm|dsound");
#else
            RTStrmPrintf(pStrm, "|dsound");
#endif
        }
        if (fSolaris)
        {
            RTStrmPrintf(pStrm, "|solaudio"
#ifdef VBOX_WITH_SOLARIS_OSS
                                    "|oss"
#endif
                        );
        }
        if (fLinux)
        {
            RTStrmPrintf(pStrm, "|oss"
#ifdef VBOX_WITH_ALSA
                                    "|alsa"
#endif
#ifdef VBOX_WITH_PULSE
                                    "|pulse"
#endif
                        );
        }
        if (fFreeBSD)
        {
            /* Get the line break sorted when dumping all option variants. */
            if (fDumpOpts)
            {
                RTStrmPrintf(pStrm, "|\n"
                     "                                     oss");
            }
            else
                RTStrmPrintf(pStrm, "|oss");
#ifdef VBOX_WITH_PULSE
            RTStrmPrintf(pStrm, "|pulse");
#endif
        }
        if (fDarwin)
        {
            RTStrmPrintf(pStrm, "|coreaudio");
        }
        RTStrmPrintf(pStrm, "]\n");
        RTStrmPrintf(pStrm,
                     "                            [--audiocontroller ac97|hda|sb16]\n"
                     "                            [--clipboard disabled|hosttoguest|guesttohost|\n"
                     "                                         bidirectional]\n"
                     "                            [--draganddrop disabled|hosttoguest\n");
        RTStrmPrintf(pStrm,
                     "                            [--vrde on|off]\n"
                     "                            [--vrdeextpack default|<name>\n"
                     "                            [--vrdeproperty <name=[value]>]\n"
                     "                            [--vrdeport <hostport>]\n"
                     "                            [--vrdeaddress <hostip>]\n"
                     "                            [--vrdeauthtype null|external|guest]\n"
                     "                            [--vrdeauthlibrary default|<name>\n"
                     "                            [--vrdemulticon on|off]\n"
                     "                            [--vrdereusecon on|off]\n"
                     "                            [--vrdevideochannel on|off]\n"
                     "                            [--vrdevideochannelquality <percent>]\n");
        RTStrmPrintf(pStrm,
                     "                            [--usb on|off]\n"
                     "                            [--usbehci on|off]\n"
                     "                            [--snapshotfolder default|<path>]\n"
                     "                            [--teleporter on|off]\n"
                     "                            [--teleporterport <port>]\n"
                     "                            [--teleporteraddress <address|empty>\n"
                     "                            [--teleporterpassword <password>]\n"
                     "                            [--teleporterpasswordfile  <file>|stdin]\n"
                     "                            [--tracing-enabled on|off]\n"
                     "                            [--tracing-config <config-string>]\n"
                     "                            [--tracing-allow-vm-access on|off]\n"
#if 0
                     "                            [--iocache on|off]\n"
                     "                            [--iocachesize <I/O cache size in MB>]\n"
#endif
#if 0
                     "                            [--faulttolerance master|standby]\n"
                     "                            [--faulttoleranceaddress <name>]\n"
                     "                            [--faulttoleranceport <port>]\n"
                     "                            [--faulttolerancesyncinterval <msec>]\n"
                     "                            [--faulttolerancepassword <password>]\n"
#endif
#ifdef VBOX_WITH_USB_VIDEO
                     "                            [--usbwebcam on|off]\n"
#endif
#ifdef VBOX_WITH_USB_CARDREADER
                     "                            [--usbcardreader on|off]\n"
#endif
                     "                            [--autostart-enabled on|off]\n"
                     "                            [--autostart-delay <seconds>]\n"
#if 0 /* Disabled until the feature is implemented. */
                     "                            [--autostop-type disabled|savestate|poweroff|\n"
                     "                                             acpishutdown]\n"
#endif
                     "\n");
    }

    if (u64Cmd & USAGE_CLONEVM)
        RTStrmPrintf(pStrm,
                           "%s clonevm %s         <uuid>|<name>\n"
                     "                            [--snapshot <uuid>|<name>]\n"
                     "                            [--mode machine|machineandchildren|all]\n"
                     "                            [--options link|keepallmacs|keepnatmacs|\n"
                     "                                       keepdisknames]\n"
                     "                            [--name <name>]\n"
                     "                            [--groups <group>, ...]\n"
                     "                            [--basefolder <basefolder>]\n"
                     "                            [--uuid <uuid>]\n"
                     "                            [--register]\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_IMPORTAPPLIANCE)
        RTStrmPrintf(pStrm,
                           "%s import %s          <ovf/ova>\n"
                     "                            [--dry-run|-n]\n"
                     "                            [--options keepallmacs|keepnatmacs]\n"
                     "                            [more options]\n"
                     "                            (run with -n to have options displayed\n"
                     "                             for a particular OVF)\n\n", SEP);

    if (u64Cmd & USAGE_EXPORTAPPLIANCE)
        RTStrmPrintf(pStrm,
                           "%s export %s          <machines> --output|-o <name>.<ovf/ova>\n"
                     "                            [--legacy09|--ovf09|--ovf10|--ovf20]\n"
                     "                            [--manifest]\n"
                     "                            [--vsys <number of virtual system>]\n"
                     "                                    [--product <product name>]\n"
                     "                                    [--producturl <product url>]\n"
                     "                                    [--vendor <vendor name>]\n"
                     "                                    [--vendorurl <vendor url>]\n"
                     "                                    [--version <version info>]\n"
                     "                                    [--eula <license text>]\n"
                     "                                    [--eulafile <filename>]\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_STARTVM)
    {
        RTStrmPrintf(pStrm,
                           "%s startvm %s         <uuid>|<name>...\n"
                     "                            [--type gui", SEP);
        if (fVBoxSDL)
            RTStrmPrintf(pStrm, "|sdl");
        RTStrmPrintf(pStrm, "|headless]\n");
        RTStrmPrintf(pStrm,
                     "\n");
    }

    if (u64Cmd & USAGE_CONTROLVM)
    {
        RTStrmPrintf(pStrm,
                           "%s controlvm %s       <uuid>|<name>\n"
                     "                            pause|resume|reset|poweroff|savestate|\n"
                     "                            acpipowerbutton|acpisleepbutton|\n"
                     "                            keyboardputscancode <hex> [<hex> ...]|\n"
                     "                            setlinkstate<1-N> on|off |\n"
#if defined(VBOX_WITH_NETFLT)
                     "                            nic<1-N> null|nat|bridged|intnet|hostonly|generic"
                     "\n"
                     "                                     [<devicename>] |\n"
#else /* !VBOX_WITH_NETFLT */
                     "                            nic<1-N> null|nat|bridged|intnet|generic\n"
                     "                                     [<devicename>] |\n"
#endif /* !VBOX_WITH_NETFLT */
                     "                            nictrace<1-N> on|off |\n"
                     "                            nictracefile<1-N> <filename> |\n"
                     "                            nicproperty<1-N> name=[value] |\n"
                     "                            nicpromisc<1-N> deny|allow-vms|allow-all |\n"
                     "                            natpf<1-N> [<rulename>],tcp|udp,[<hostip>],\n"
                     "                                        <hostport>,[<guestip>],<guestport> |\n"
                     "                            natpf<1-N> delete <rulename> |\n"
                     "                            guestmemoryballoon <balloonsize in MB> |\n"
                     "                            usbattach <uuid>|<address> |\n"
                     "                            usbdetach <uuid>|<address> |\n"
                     "                            clipboard disabled|hosttoguest|guesttohost|\n"
                     "                                      bidirectional |\n"
                     "                            draganddrop disabled|hosttoguest |\n"
                     "                            vrde on|off |\n"
                     "                            vrdeport <port> |\n"
                     "                            vrdeproperty <name=[value]> |\n"
                     "                            vrdevideochannelquality <percent> |\n"
                     "                            setvideomodehint <xres> <yres> <bpp>\n"
                     "                                            [[<display>] [<enabled:yes|no> |\n"
                     "                                              [<xorigin> <yorigin>]]] |\n"
                     "                            screenshotpng <file> [display] |\n"
                     "                            setcredentials <username>\n"
                     "                                           --passwordfile <file> | <password>\n"
                     "                                           <domain>\n"
                     "                                           [--allowlocallogon <yes|no>] |\n"
                     "                            teleport --host <name> --port <port>\n"
                     "                                     [--maxdowntime <msec>]\n"
                     "                                     [--passwordfile <file> |\n"
                     "                                      --password <password>] |\n"
                     "                            plugcpu <id> |\n"
                     "                            unplugcpu <id> |\n"
                     "                            cpuexecutioncap <1-100>\n"
                     "\n", SEP);
    }

    if (u64Cmd & USAGE_DISCARDSTATE)
        RTStrmPrintf(pStrm,
                           "%s discardstate %s    <uuid>|<name>\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_ADOPTSTATE)
        RTStrmPrintf(pStrm,
                           "%s adoptstate %s      <uuid>|<name> <state_file>\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_SNAPSHOT)
        RTStrmPrintf(pStrm,
                           "%s snapshot %s        <uuid>|<name>\n"
                     "                            take <name> [--description <desc>] [--pause] |\n"
                     "                            delete <uuid>|<name> |\n"
                     "                            restore <uuid>|<name> |\n"
                     "                            restorecurrent |\n"
                     "                            edit <uuid>|<name>|--current\n"
                     "                                 [--name <name>]\n"
                     "                                 [--description <desc>] |\n"
                     "                            list [--details|--machinereadable]\n"
                     "                            showvminfo <uuid>|<name>\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_CLOSEMEDIUM)
        RTStrmPrintf(pStrm,
                           "%s closemedium %s     disk|dvd|floppy <uuid>|<filename>\n"
                     "                            [--delete]\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_STORAGEATTACH)
        RTStrmPrintf(pStrm,
                           "%s storageattach %s   <uuid|vmname>\n"
                     "                            --storagectl <name>\n"
                     "                            [--port <number>]\n"
                     "                            [--device <number>]\n"
                     "                            [--type dvddrive|hdd|fdd]\n"
                     "                            [--medium none|emptydrive|additions|\n"
                     "                                      <uuid>|<filename>|host:<drive>|iscsi]\n"
                     "                            [--mtype normal|writethrough|immutable|shareable|\n"
                     "                                     readonly|multiattach]\n"
                     "                            [--comment <text>]\n"
                     "                            [--setuuid <uuid>]\n"
                     "                            [--setparentuuid <uuid>]\n"
                     "                            [--passthrough on|off]\n"
                     "                            [--tempeject on|off]\n"
                     "                            [--nonrotational on|off]\n"
                     "                            [--discard on|off]\n"
                     "                            [--bandwidthgroup <name>]\n"
                     "                            [--forceunmount]\n"
                     "                            [--server <name>|<ip>]\n"
                     "                            [--target <target>]\n"
                     "                            [--tport <port>]\n"
                     "                            [--lun <lun>]\n"
                     "                            [--encodedlun <lun>]\n"
                     "                            [--username <username>]\n"
                     "                            [--password <password>]\n"
                     "                            [--initiator <initiator>]\n"
                     "                            [--intnet]\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_STORAGECONTROLLER)
        RTStrmPrintf(pStrm,
                           "%s storagectl %s      <uuid|vmname>\n"
                     "                            --name <name>\n"
                     "                            [--add ide|sata|scsi|floppy|sas]\n"
                     "                            [--controller LSILogic|LSILogicSAS|BusLogic|\n"
                     "                                          IntelAHCI|PIIX3|PIIX4|ICH6|I82078]\n"
                     "                            [--sataportcount <1-30>]\n"
                     "                            [--hostiocache on|off]\n"
                     "                            [--bootable on|off]\n"
                     "                            [--remove]\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_BANDWIDTHCONTROL)
        RTStrmPrintf(pStrm,
                           "%s bandwidthctl %s    <uuid|vmname>\n"
                     "                            add <name> --type disk|network\n"
                     "                                --limit <megabytes per second>[k|m|g|K|M|G] |\n"
                     "                            set <name>\n"
                     "                                --limit <megabytes per second>[k|m|g|K|M|G] |\n"
                     "                            remove <name> |\n"
                     "                            list [--machinereadable]\n"
                     "                            (limit units: k=kilobit, m=megabit, g=gigabit,\n"
                     "                                          K=kilobyte, M=megabyte, G=gigabyte)\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_SHOWHDINFO)
        RTStrmPrintf(pStrm,
                           "%s showhdinfo %s      <uuid>|<filename>\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_CREATEHD)
        RTStrmPrintf(pStrm,
                           "%s createhd %s        --filename <filename>\n"
                     "                            [--size <megabytes>|--sizebyte <bytes>]\n"
                     "                            [--diffparent <uuid>|<filename>\n"
                     "                            [--format VDI|VMDK|VHD] (default: VDI)\n"
                     "                            [--variant Standard,Fixed,Split2G,Stream,ESX]\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_MODIFYHD)
        RTStrmPrintf(pStrm,
                           "%s modifyhd %s        <uuid>|<filename>\n"
                     "                            [--type normal|writethrough|immutable|shareable|\n"
                     "                                    readonly|multiattach]\n"
                     "                            [--autoreset on|off]\n"
                     "                            [--compact]\n"
                     "                            [--resize <megabytes>|--resizebyte <bytes>]\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_CLONEHD)
        RTStrmPrintf(pStrm,
                           "%s clonehd %s         <uuid>|<filename> <uuid>|<outputfile>\n"
                     "                            [--format VDI|VMDK|VHD|RAW|<other>]\n"
                     "                            [--variant Standard,Fixed,Split2G,Stream,ESX]\n"
                     "                            [--existing]\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_CONVERTFROMRAW)
        RTStrmPrintf(pStrm,
                           "%s convertfromraw %s  <filename> <outputfile>\n"
                     "                            [--format VDI|VMDK|VHD]\n"
                     "                            [--variant Standard,Fixed,Split2G,Stream,ESX]\n"
                     "                            [--uuid <uuid>]\n"
                           "%s convertfromraw %s  stdin <outputfile> <bytes>\n"
                     "                            [--format VDI|VMDK|VHD]\n"
                     "                            [--variant Standard,Fixed,Split2G,Stream,ESX]\n"
                     "                            [--uuid <uuid>]\n"
                     "\n", SEP, SEP);

    if (u64Cmd & USAGE_GETEXTRADATA)
        RTStrmPrintf(pStrm,
                           "%s getextradata %s    global|<uuid>|<name>\n"
                     "                            <key>|enumerate\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_SETEXTRADATA)
        RTStrmPrintf(pStrm,
                           "%s setextradata %s    global|<uuid>|<name>\n"
                     "                            <key>\n"
                     "                            [<value>] (no value deletes key)\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_SETPROPERTY)
        RTStrmPrintf(pStrm,
                           "%s setproperty %s     machinefolder default|<folder> |\n"
                     "                            vrdeauthlibrary default|<library> |\n"
                     "                            websrvauthlibrary default|null|<library> |\n"
                     "                            vrdeextpack null|<library> |\n"
                     "                            autostartdbpath null|<folder> |\n"
                     "                            loghistorycount <value>\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_USBFILTER_ADD)
        RTStrmPrintf(pStrm,
                           "%s usbfilter %s       add <index,0-N>\n"
                     "                            --target <uuid>|<name>|global\n"
                     "                            --name <string>\n"
                     "                            --action ignore|hold (global filters only)\n"
                     "                            [--active yes|no] (yes)\n"
                     "                            [--vendorid <XXXX>] (null)\n"
                     "                            [--productid <XXXX>] (null)\n"
                     "                            [--revision <IIFF>] (null)\n"
                     "                            [--manufacturer <string>] (null)\n"
                     "                            [--product <string>] (null)\n"
                     "                            [--remote yes|no] (null, VM filters only)\n"
                     "                            [--serialnumber <string>] (null)\n"
                     "                            [--maskedinterfaces <XXXXXXXX>]\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_USBFILTER_MODIFY)
        RTStrmPrintf(pStrm,
                           "%s usbfilter %s       modify <index,0-N>\n"
                     "                            --target <uuid>|<name>|global\n"
                     "                            [--name <string>]\n"
                     "                            [--action ignore|hold] (global filters only)\n"
                     "                            [--active yes|no]\n"
                     "                            [--vendorid <XXXX>|\"\"]\n"
                     "                            [--productid <XXXX>|\"\"]\n"
                     "                            [--revision <IIFF>|\"\"]\n"
                     "                            [--manufacturer <string>|\"\"]\n"
                     "                            [--product <string>|\"\"]\n"
                     "                            [--remote yes|no] (null, VM filters only)\n"
                     "                            [--serialnumber <string>|\"\"]\n"
                     "                            [--maskedinterfaces <XXXXXXXX>]\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_USBFILTER_REMOVE)
        RTStrmPrintf(pStrm,
                           "%s usbfilter %s       remove <index,0-N>\n"
                     "                            --target <uuid>|<name>|global\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_SHAREDFOLDER_ADD)
        RTStrmPrintf(pStrm,
                           "%s sharedfolder %s    add <vmname>|<uuid>\n"
                     "                            --name <name> --hostpath <hostpath>\n"
                     "                            [--transient] [--readonly] [--automount]\n"
                     "\n", SEP);

    if (u64Cmd & USAGE_SHAREDFOLDER_REMOVE)
        RTStrmPrintf(pStrm,
                           "%s sharedfolder %s    remove <vmname>|<uuid>\n"
                     "                            --name <name> [--transient]\n"
                     "\n", SEP);

#ifdef VBOX_WITH_GUEST_PROPS
    if (u64Cmd & USAGE_GUESTPROPERTY)
        usageGuestProperty(pStrm, SEP);
#endif /* VBOX_WITH_GUEST_PROPS defined */

#ifdef VBOX_WITH_GUEST_CONTROL
    if (u64Cmd & USAGE_GUESTCONTROL)
        usageGuestControl(pStrm, SEP);
#endif /* VBOX_WITH_GUEST_CONTROL defined */

    if (u64Cmd & USAGE_DEBUGVM)
    {
        RTStrmPrintf(pStrm,
                           "%s debugvm %s         <uuid>|<name>\n"
                     "                            dumpguestcore --filename <name> |\n"
                     "                            info <item> [args] |\n"
                     "                            injectnmi |\n"
                     "                            log [--release|--debug] <settings> ...|\n"
                     "                            logdest [--release|--debug] <settings> ...|\n"
                     "                            logflags [--release|--debug] <settings> ...|\n"
                     "                            osdetect |\n"
                     "                            osinfo |\n"
                     "                            getregisters [--cpu <id>] <reg>|all ... |\n"
                     "                            setregisters [--cpu <id>] <reg>=<value> ... |\n"
                     "                            show [--human-readable|--sh-export|--sh-eval|\n"
                     "                                  --cmd-set] \n"
                     "                                <logdbg-settings|logrel-settings>\n"
                     "                                [[opt] what ...] |\n"
                     "                            statistics [--reset] [--pattern <pattern>]\n"
                     "                            [--descriptions]\n"
                     "\n", SEP);
    }
    if (u64Cmd & USAGE_METRICS)
        RTStrmPrintf(pStrm,
                           "%s metrics %s         list [*|host|<vmname> [<metric_list>]]\n"
                     "                                                 (comma-separated)\n\n"
                           "%s metrics %s         setup\n"
                     "                            [--period <seconds>] (default: 1)\n"
                     "                            [--samples <count>] (default: 1)\n"
                     "                            [--list]\n"
                     "                            [*|host|<vmname> [<metric_list>]]\n\n"
                           "%s metrics %s         query [*|host|<vmname> [<metric_list>]]\n\n"
                           "%s metrics %s         enable\n"
                     "                            [--list]\n"
                     "                            [*|host|<vmname> [<metric_list>]]\n\n"
                           "%s metrics %s         disable\n"
                     "                            [--list]\n"
                     "                            [*|host|<vmname> [<metric_list>]]\n\n"
                           "%s metrics %s         collect\n"
                     "                            [--period <seconds>] (default: 1)\n"
                     "                            [--samples <count>] (default: 1)\n"
                     "                            [--list]\n"
                     "                            [--detach]\n"
                     "                            [*|host|<vmname> [<metric_list>]]\n"
                     "\n", SEP, SEP, SEP, SEP, SEP, SEP);

#if defined(VBOX_WITH_NETFLT)
    if (u64Cmd & USAGE_HOSTONLYIFS)
    {
        RTStrmPrintf(pStrm,
                           "%s hostonlyif %s      ipconfig <name>\n"
                     "                            [--dhcp |\n"
                     "                            --ip<ipv4> [--netmask<ipv4> (def: 255.255.255.0)] |\n"
                     "                            --ipv6<ipv6> [--netmasklengthv6<length> (def: 64)]]\n"
# if !defined(RT_OS_SOLARIS)
                     "                            create |\n"
                     "                            remove <name>\n"
# endif
                     "\n", SEP);
    }
#endif

    if (u64Cmd & USAGE_DHCPSERVER)
    {
        RTStrmPrintf(pStrm,
                           "%s dhcpserver %s      add|modify --netname <network_name> |\n"
#if defined(VBOX_WITH_NETFLT)
                     "                                       --ifname <hostonly_if_name>\n"
#endif
                     "                            [--ip <ip_address>\n"
                     "                            --netmask <network_mask>\n"
                     "                            --lowerip <lower_ip>\n"
                     "                            --upperip <upper_ip>]\n"
                     "                            [--enable | --disable]\n\n"
                           "%s dhcpserver %s      remove --netname <network_name> |\n"
#if defined(VBOX_WITH_NETFLT)
                     "                                   --ifname <hostonly_if_name>\n"
#endif
                     "\n", SEP, SEP);
    }
    if (u64Cmd & USAGE_EXTPACK)
    {
        RTStrmPrintf(pStrm,
                           "%s extpack %s         install [--replace] <tarball> |\n"
                     "                            uninstall [--force] <name> |\n"
                     "                            cleanup\n"
                     "\n", SEP);
    }
}

/**
 * Print a usage synopsis and the syntax error message.
 * @returns RTEXITCODE_SYNTAX.
 */
RTEXITCODE errorSyntax(USAGECATEGORY u64Cmd, const char *pszFormat, ...)
{
    va_list args;
    showLogo(g_pStdErr); // show logo even if suppressed
#ifndef VBOX_ONLY_DOCS
    if (g_fInternalMode)
        printUsageInternal(u64Cmd, g_pStdErr);
    else
        printUsage(u64Cmd, g_pStdErr);
#endif /* !VBOX_ONLY_DOCS */
    va_start(args, pszFormat);
    RTStrmPrintf(g_pStdErr, "\nSyntax error: %N\n", pszFormat, &args);
    va_end(args);
    return RTEXITCODE_SYNTAX;
}

/**
 * errorSyntax for RTGetOpt users.
 *
 * @returns RTEXITCODE_SYNTAX.
 *
 * @param   fUsageCategory  The usage category of the command.
 * @param   rc              The RTGetOpt return code.
 * @param   pValueUnion     The value union.
 */
RTEXITCODE errorGetOpt(USAGECATEGORY fUsageCategory, int rc, union RTGETOPTUNION const *pValueUnion)
{
    /*
     * Check if it is an unhandled standard option.
     */
    if (rc == 'V')
    {
        RTPrintf("%sr%d\n", VBOX_VERSION_STRING, RTBldCfgRevision());
        return RTEXITCODE_SUCCESS;
    }

    if (rc == 'h')
    {
        showLogo(g_pStdErr);
#ifndef VBOX_ONLY_DOCS
        if (g_fInternalMode)
            printUsageInternal(fUsageCategory, g_pStdOut);
        else
            printUsage(fUsageCategory, g_pStdOut);
#endif
        return RTEXITCODE_SUCCESS;
    }

    /*
     * General failure.
     */
    showLogo(g_pStdErr); // show logo even if suppressed
#ifndef VBOX_ONLY_DOCS
    if (g_fInternalMode)
        printUsageInternal(fUsageCategory, g_pStdErr);
    else
        printUsage(fUsageCategory, g_pStdErr);
#endif /* !VBOX_ONLY_DOCS */

    if (rc == VINF_GETOPT_NOT_OPTION)
        return RTMsgErrorExit(RTEXITCODE_SYNTAX, "Invalid parameter '%s'", pValueUnion->psz);
    if (rc > 0)
    {
        if (RT_C_IS_PRINT(rc))
            return RTMsgErrorExit(RTEXITCODE_SYNTAX, "Invalid option -%c", rc);
        return RTMsgErrorExit(RTEXITCODE_SYNTAX, "Invalid option case %i", rc);
    }
    if (rc == VERR_GETOPT_UNKNOWN_OPTION)
        return RTMsgErrorExit(RTEXITCODE_SYNTAX, "Unknown option: %s", pValueUnion->psz);
    if (rc == VERR_GETOPT_INVALID_ARGUMENT_FORMAT)
        return RTMsgErrorExit(RTEXITCODE_SYNTAX, "Invalid argument format: %s", pValueUnion->psz);
    if (pValueUnion->pDef)
        return RTMsgErrorExit(RTEXITCODE_SYNTAX, "%s: %Rrs", pValueUnion->pDef->pszLong, rc);
    return RTMsgErrorExit(RTEXITCODE_SYNTAX, "%Rrs", rc);
}

/**
 * Print an error message without the syntax stuff.
 *
 * @returns RTEXITCODE_SYNTAX.
 */
RTEXITCODE errorArgument(const char *pszFormat, ...)
{
    va_list args;
    va_start(args, pszFormat);
    RTMsgErrorV(pszFormat, args);
    va_end(args);
    return RTEXITCODE_SYNTAX;
}
