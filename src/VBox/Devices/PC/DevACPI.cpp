/* $Id: DevACPI.cpp $ */
/** @file
 * DevACPI - Advanced Configuration and Power Interface (ACPI) Device.
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

/*******************************************************************************
*   Header Files                                                               *
*******************************************************************************/
#define LOG_GROUP LOG_GROUP_DEV_ACPI
#include <VBox/vmm/pdmdev.h>
#include <VBox/vmm/pgm.h>
#include <VBox/vmm/dbgftrace.h>
#include <VBox/vmm/vmcpuset.h>
#include <VBox/log.h>
#include <VBox/param.h>
#include <iprt/assert.h>
#include <iprt/asm.h>
#include <iprt/asm-math.h>
#include <iprt/file.h>
#ifdef IN_RING3
# include <iprt/alloc.h>
# include <iprt/string.h>
# include <iprt/uuid.h>
#endif /* IN_RING3 */

#include "VBoxDD.h"

#ifdef LOG_ENABLED
# define DEBUG_ACPI
#endif

#if defined(IN_RING3) && !defined(VBOX_DEVICE_STRUCT_TESTCASE)
int acpiPrepareDsdt(PPDMDEVINS pDevIns, void* *ppPtr, size_t *puDsdtLen);
int acpiCleanupDsdt(PPDMDEVINS pDevIns, void* pPtr);

int acpiPrepareSsdt(PPDMDEVINS pDevIns, void* *ppPtr, size_t *puSsdtLen);
int acpiCleanupSsdt(PPDMDEVINS pDevIns, void* pPtr);
#endif /* !IN_RING3 */



/*******************************************************************************
*   Defined Constants And Macros                                               *
*******************************************************************************/
#ifdef IN_RING3
/** Locks the device state, ring-3 only.  */
# define DEVACPI_LOCK_R3(a_pThis) \
    do { \
        int rcLock = PDMCritSectEnter(&(a_pThis)->CritSect, VERR_IGNORED); \
        AssertRC(rcLock); \
    } while (0)
#endif
/** Unlocks the device state (all contexts). */
#define DEVACPI_UNLOCK(a_pThis) \
    do { PDMCritSectLeave(&(a_pThis)->CritSect); } while (0)


#define DEBUG_HEX       0x3000
#define DEBUG_CHR       0x3001

#define PM_TMR_FREQ     3579545
/* Default base for PM PIIX4 device */
#define PM_PORT_BASE    0x4000
/* Port offsets in PM device */
enum
{
    PM1a_EVT_OFFSET                     = 0x00,
    PM1b_EVT_OFFSET                     =   -1,   /**<  not supported  */
    PM1a_CTL_OFFSET                     = 0x04,
    PM1b_CTL_OFFSET                     =   -1,   /**<  not supported  */
    PM2_CTL_OFFSET                      =   -1,   /**<  not supported  */
    PM_TMR_OFFSET                       = 0x08,
    GPE0_OFFSET                         = 0x20,
    GPE1_OFFSET                         =   -1    /**<  not supported  */
};

#define BAT_INDEX       0x00004040
#define BAT_DATA        0x00004044
#define SYSI_INDEX      0x00004048
#define SYSI_DATA       0x0000404c
#define ACPI_RESET_BLK  0x00004050

/* PM1x status register bits */
#define TMR_STS         RT_BIT(0)
#define RSR1_STS        (RT_BIT(1) | RT_BIT(2) | RT_BIT(3))
#define BM_STS          RT_BIT(4)
#define GBL_STS         RT_BIT(5)
#define RSR2_STS        (RT_BIT(6) | RT_BIT(7))
#define PWRBTN_STS      RT_BIT(8)
#define SLPBTN_STS      RT_BIT(9)
#define RTC_STS         RT_BIT(10)
#define IGN_STS         RT_BIT(11)
#define RSR3_STS        (RT_BIT(12) | RT_BIT(13) | RT_BIT(14))
#define WAK_STS         RT_BIT(15)
#define RSR_STS         (RSR1_STS | RSR2_STS | RSR3_STS)

/* PM1x enable register bits */
#define TMR_EN          RT_BIT(0)
#define RSR1_EN         (RT_BIT(1) | RT_BIT(2) | RT_BIT(3) | RT_BIT(4))
#define GBL_EN          RT_BIT(5)
#define RSR2_EN         (RT_BIT(6) | RT_BIT(7))
#define PWRBTN_EN       RT_BIT(8)
#define SLPBTN_EN       RT_BIT(9)
#define RTC_EN          RT_BIT(10)
#define RSR3_EN         (RT_BIT(11) | RT_BIT(12) | RT_BIT(13) | RT_BIT(14) | RT_BIT(15))
#define RSR_EN          (RSR1_EN | RSR2_EN | RSR3_EN)
#define IGN_EN          0

/* PM1x control register bits */
#define SCI_EN          RT_BIT(0)
#define BM_RLD          RT_BIT(1)
#define GBL_RLS         RT_BIT(2)
#define RSR1_CNT        (RT_BIT(3) | RT_BIT(4) | RT_BIT(5) | RT_BIT(6) | RT_BIT(7) | RT_BIT(8))
#define IGN_CNT         RT_BIT(9)
#define SLP_TYPx_SHIFT  10
#define SLP_TYPx_MASK    7
#define SLP_EN          RT_BIT(13)
#define RSR2_CNT        (RT_BIT(14) | RT_BIT(15))
#define RSR_CNT         (RSR1_CNT | RSR2_CNT)

#define GPE0_BATTERY_INFO_CHANGED RT_BIT(0)

enum
{
    BAT_STATUS_STATE                    = 0x00, /**< BST battery state */
    BAT_STATUS_PRESENT_RATE             = 0x01, /**< BST battery present rate */
    BAT_STATUS_REMAINING_CAPACITY       = 0x02, /**< BST battery remaining capacity */
    BAT_STATUS_PRESENT_VOLTAGE          = 0x03, /**< BST battery present voltage */
    BAT_INFO_UNITS                      = 0x04, /**< BIF power unit */
    BAT_INFO_DESIGN_CAPACITY            = 0x05, /**< BIF design capacity */
    BAT_INFO_LAST_FULL_CHARGE_CAPACITY  = 0x06, /**< BIF last full charge capacity */
    BAT_INFO_TECHNOLOGY                 = 0x07, /**< BIF battery technology */
    BAT_INFO_DESIGN_VOLTAGE             = 0x08, /**< BIF design voltage */
    BAT_INFO_DESIGN_CAPACITY_OF_WARNING = 0x09, /**< BIF design capacity of warning */
    BAT_INFO_DESIGN_CAPACITY_OF_LOW     = 0x0A, /**< BIF design capacity of low */
    BAT_INFO_CAPACITY_GRANULARITY_1     = 0x0B, /**< BIF battery capacity granularity 1 */
    BAT_INFO_CAPACITY_GRANULARITY_2     = 0x0C, /**< BIF battery capacity granularity 2 */
    BAT_DEVICE_STATUS                   = 0x0D, /**< STA device status */
    BAT_POWER_SOURCE                    = 0x0E, /**< PSR power source */
    BAT_INDEX_LAST
};

enum
{
    CPU_EVENT_TYPE_ADD                  = 0x01, /**< Event type add */
    CPU_EVENT_TYPE_REMOVE               = 0x03  /**< Event type remove */
};

enum
{
    SYSTEM_INFO_INDEX_LOW_MEMORY_LENGTH = 0,
    SYSTEM_INFO_INDEX_USE_IOAPIC        = 1,
    SYSTEM_INFO_INDEX_HPET_STATUS       = 2,
    SYSTEM_INFO_INDEX_SMC_STATUS        = 3,
    SYSTEM_INFO_INDEX_FDC_STATUS        = 4,
    SYSTEM_INFO_INDEX_CPU0_STATUS       = 5,  /**< For compatibility with older saved states. */
    SYSTEM_INFO_INDEX_CPU1_STATUS       = 6,  /**< For compatibility with older saved states. */
    SYSTEM_INFO_INDEX_CPU2_STATUS       = 7,  /**< For compatibility with older saved states. */
    SYSTEM_INFO_INDEX_CPU3_STATUS       = 8,  /**< For compatibility with older saved states. */
    SYSTEM_INFO_INDEX_HIGH_MEMORY_LENGTH= 9,
    SYSTEM_INFO_INDEX_RTC_STATUS        = 10,
    SYSTEM_INFO_INDEX_CPU_LOCKED        = 11, /**< Contains a flag indicating whether the CPU is locked or not */
    SYSTEM_INFO_INDEX_CPU_LOCK_CHECK    = 12, /**< For which CPU the lock status should be checked */
    SYSTEM_INFO_INDEX_CPU_EVENT_TYPE    = 13, /**< Type of the CPU hot-plug event */
    SYSTEM_INFO_INDEX_CPU_EVENT         = 14, /**< The CPU id the event is for */
    SYSTEM_INFO_INDEX_NIC_ADDRESS       = 15, /**< NIC PCI address, or 0 */
    SYSTEM_INFO_INDEX_AUDIO_ADDRESS     = 16, /**< Audio card PCI address, or 0 */
    SYSTEM_INFO_INDEX_POWER_STATES      = 17,
    SYSTEM_INFO_INDEX_IOC_ADDRESS       = 18, /**< IO controller PCI address */
    SYSTEM_INFO_INDEX_HBC_ADDRESS       = 19, /**< host bus controller PCI address */
    SYSTEM_INFO_INDEX_PCI_BASE          = 20, /**< PCI bus MCFG MMIO range base */
    SYSTEM_INFO_INDEX_PCI_LENGTH        = 21, /**< PCI bus MCFG MMIO range length */
    SYSTEM_INFO_INDEX_SERIAL0_IOBASE    = 22,
    SYSTEM_INFO_INDEX_SERIAL0_IRQ       = 23,
    SYSTEM_INFO_INDEX_SERIAL1_IOBASE    = 24,
    SYSTEM_INFO_INDEX_SERIAL1_IRQ       = 25,
    SYSTEM_INFO_INDEX_END               = 26,
    SYSTEM_INFO_INDEX_INVALID           = 0x80,
    SYSTEM_INFO_INDEX_VALID             = 0x200
};

#define AC_OFFLINE                              0
#define AC_ONLINE                               1

#define BAT_TECH_PRIMARY                        1
#define BAT_TECH_SECONDARY                      2

#define STA_DEVICE_PRESENT_MASK                 RT_BIT(0) /**< present */
#define STA_DEVICE_ENABLED_MASK                 RT_BIT(1) /**< enabled and decodes its resources */
#define STA_DEVICE_SHOW_IN_UI_MASK              RT_BIT(2) /**< should be shown in UI */
#define STA_DEVICE_FUNCTIONING_PROPERLY_MASK    RT_BIT(3) /**< functioning properly */
#define STA_BATTERY_PRESENT_MASK                RT_BIT(4) /**< the battery is present */


/*******************************************************************************
*   Structures and Typedefs                                                    *
*******************************************************************************/
/**
 * The ACPI device state.
 */
typedef struct ACPIState
{
    PCIDevice           dev;
    /** Critical section protecting the ACPI state. */
    PDMCRITSECT         CritSect;

    uint16_t            pm1a_en;
    uint16_t            pm1a_sts;
    uint16_t            pm1a_ctl;
    /** Number of logical CPUs in guest */
    uint16_t            cCpus;
    uint64_t            u64PmTimerInitial;
    PTMTIMERR3          pPmTimerR3;
    PTMTIMERR0          pPmTimerR0;
    PTMTIMERRC          pPmTimerRC;

    uint32_t            gpe0_en;
    uint32_t            gpe0_sts;

    uint32_t            uBatteryIndex;
    uint32_t            au8BatteryInfo[13];

    uint32_t            uSystemInfoIndex;
    uint64_t            u64RamSize;
    /** The number of bytes above 4GB. */
    uint64_t            cbRamHigh;
    /** The number of bytes below 4GB. */
    uint32_t            cbRamLow;

    /** Current ACPI S* state. We support S0 and S5. */
    uint32_t            uSleepState;
    uint8_t             au8RSDPPage[0x1000];
    /** This is a workaround for incorrect index field handling by Intels ACPICA.
     *  The system info _INI method writes to offset 0x200. We either observe a
     *  write request to index 0x80 (in that case we don't change the index) or a
     *  write request to offset 0x200 (in that case we divide the index value by
     *  4. Note that the _STA method is sometimes called prior to the _INI method
     *  (ACPI spec 6.3.7, _STA). See the special case for BAT_DEVICE_STATUS in
     *  acpiBatIndexWrite() for handling this. */
    uint8_t             u8IndexShift;
    /** provide an I/O-APIC */
    uint8_t             u8UseIOApic;
    /** provide a floppy controller */
    bool                fUseFdc;
    /** If High Precision Event Timer device should be supported */
    bool                fUseHpet;
    /** If System Management Controller device should be supported */
    bool                fUseSmc;
    /** the guest handled the last power button event */
    bool                fPowerButtonHandled;
    /** If ACPI CPU device should be shown */
    bool                fShowCpu;
    /** If Real Time Clock ACPI object to be shown */
    bool                fShowRtc;
    /** I/O port address of PM device. */
    RTIOPORT            uPmIoPortBase;
    /** Flag whether the GC part of the device is enabled. */
    bool                fGCEnabled;
    /** Flag whether the R0 part of the device is enabled. */
    bool                fR0Enabled;
    /** Array of flags of attached CPUs */
    VMCPUSET            CpuSetAttached;
    /** Which CPU to check for the locked status. */
    uint32_t            idCpuLockCheck;
    /** Mask of locked CPUs (used by the guest). */
    VMCPUSET            CpuSetLocked;
    /** The CPU event type. */
    uint32_t            u32CpuEventType;
    /** The CPU id affected. */
    uint32_t            u32CpuEvent;
    /** Flag whether CPU hot plugging is enabled. */
    bool                fCpuHotPlug;
    /** If MCFG ACPI table shown to the guest */
    bool                fUseMcfg;
    /** Primary NIC PCI address. */
    uint32_t            u32NicPciAddress;
    /** Primary audio card PCI address. */
    uint32_t            u32AudioPciAddress;
    /** Flag whether S1 power state is enabled. */
    bool                fS1Enabled;
    /** Flag whether S4 power state is enabled. */
    bool                fS4Enabled;
    /** Flag whether S1 triggers a state save. */
    bool                fSuspendToSavedState;
    /** Flag whether to set WAK_STS on resume (restore included). */
    bool                fSetWakeupOnResume;
    /** PCI address of the IO controller device. */
    uint32_t            u32IocPciAddress;
    /** PCI address of the host bus controller device. */
    uint32_t            u32HbcPciAddress;
    /* Physical address of PCI config space MMIO region */
    uint64_t            u64PciConfigMMioAddress;
    /* Length of PCI config space MMIO region */
    uint64_t            u64PciConfigMMioLength;
    /** Serial 0 IRQ number */
    uint8_t             uSerial0Irq;
    /** Serial 1 IRQ number */
    uint8_t             uSerial1Irq;
    /** Serial 0 IO port base */
    RTIOPORT            uSerial0IoPortBase;
    /** Serial 1 IO port base */
    RTIOPORT            uSerial1IoPortBase;
    /** ACPI port base interface. */
    PDMIBASE            IBase;
    /** ACPI port interface. */
    PDMIACPIPORT        IACPIPort;
    /** Pointer to the device instance. */
    PPDMDEVINSR3        pDevIns;
    /** Pointer to the driver base interface. */
    R3PTRTYPE(PPDMIBASE) pDrvBase;
    /** Pointer to the driver connector interface. */
    R3PTRTYPE(PPDMIACPICONNECTOR) pDrv;

    /** Pointer to default PCI config read function. */
    R3PTRTYPE(PFNPCICONFIGREAD)   pfnAcpiPciConfigRead;
    /** Pointer to default PCI config write function. */
    R3PTRTYPE(PFNPCICONFIGWRITE)  pfnAcpiPciConfigWrite;

    /** If custom table should be supported */
    bool                fUseCust;
    /** ACPI OEM ID */
    uint8_t             au8OemId[6];
    /** ACPI Crator ID */
    uint8_t             au8CreatorId[4];
    /** ACPI Crator Rev */
    uint32_t            u32CreatorRev;
    /** ACPI custom OEM Tab ID */
    uint8_t             au8OemTabId[8];
    /** ACPI custom OEM Rev */
    uint32_t            u32OemRevision;
    uint32_t            Alignment0;

    /** The custom table binary data. */
    R3PTRTYPE(uint8_t *) pu8CustBin;
    /** The size of the custom table binary. */
    uint64_t            cbCustBin;
} ACPIState;

#pragma pack(1)

/** Generic Address Structure (see ACPIspec 3.0, 5.2.3.1) */
struct ACPIGENADDR
{
    uint8_t             u8AddressSpaceId;       /**< 0=sys, 1=IO, 2=PCICfg, 3=emb, 4=SMBus */
    uint8_t             u8RegisterBitWidth;     /**< size in bits of the given register */
    uint8_t             u8RegisterBitOffset;    /**< bit offset of register */
    uint8_t             u8AccessSize;           /**< 1=byte, 2=word, 3=dword, 4=qword */
    uint64_t            u64Address;             /**< 64-bit address of register */
};
AssertCompileSize(ACPIGENADDR, 12);

/** Root System Description Pointer */
struct ACPITBLRSDP
{
    uint8_t             au8Signature[8];        /**< 'RSD PTR ' */
    uint8_t             u8Checksum;             /**< checksum for the first 20 bytes */
    uint8_t             au8OemId[6];            /**< OEM-supplied identifier */
    uint8_t             u8Revision;             /**< revision number, currently 2 */
#define ACPI_REVISION   2                       /**< ACPI 3.0 */
    uint32_t            u32RSDT;                /**< phys addr of RSDT */
    uint32_t            u32Length;              /**< bytes of this table */
    uint64_t            u64XSDT;                /**< 64-bit phys addr of XSDT */
    uint8_t             u8ExtChecksum;          /**< checksum of entire table */
    uint8_t             u8Reserved[3];          /**< reserved */
};
AssertCompileSize(ACPITBLRSDP, 36);

/** System Description Table Header */
struct ACPITBLHEADER
{
    uint8_t             au8Signature[4];        /**< table identifier */
    uint32_t            u32Length;              /**< length of the table including header */
    uint8_t             u8Revision;             /**< revision number */
    uint8_t             u8Checksum;             /**< all fields inclusive this add to zero */
    uint8_t             au8OemId[6];            /**< OEM-supplied string */
    uint8_t             au8OemTabId[8];         /**< to identify the particular data table */
    uint32_t            u32OemRevision;         /**< OEM-supplied revision number */
    uint8_t             au8CreatorId[4];        /**< ID for the ASL compiler */
    uint32_t            u32CreatorRev;          /**< revision for the ASL compiler */
};
AssertCompileSize(ACPITBLHEADER, 36);

/** Root System Description Table */
struct ACPITBLRSDT
{
    ACPITBLHEADER       header;
    uint32_t            u32Entry[1];            /**< array of phys. addresses to other tables */
};
AssertCompileSize(ACPITBLRSDT, 40);

/** Extended System Description Table */
struct ACPITBLXSDT
{
    ACPITBLHEADER       header;
    uint64_t            u64Entry[1];            /**< array of phys. addresses to other tables */
};
AssertCompileSize(ACPITBLXSDT, 44);

/** Fixed ACPI Description Table */
struct ACPITBLFADT
{
    ACPITBLHEADER       header;
    uint32_t            u32FACS;                /**< phys. address of FACS */
    uint32_t            u32DSDT;                /**< phys. address of DSDT */
    uint8_t             u8IntModel;             /**< was eleminated in ACPI 2.0 */
#define INT_MODEL_DUAL_PIC        1             /**< for ACPI 2+ */
#define INT_MODEL_MULTIPLE_APIC   2
    uint8_t             u8PreferredPMProfile;   /**< preferred power management profile */
    uint16_t            u16SCIInt;              /**< system vector the SCI is wired in 8259 mode */
#define SCI_INT         9
    uint32_t            u32SMICmd;              /**< system port address of SMI command port */
#define SMI_CMD         0x0000442e
    uint8_t             u8AcpiEnable;           /**< SMICmd val to disable ownership of ACPIregs */
#define ACPI_ENABLE     0xa1
    uint8_t             u8AcpiDisable;          /**< SMICmd val to re-enable ownership of ACPIregs */
#define ACPI_DISABLE    0xa0
    uint8_t             u8S4BIOSReq;            /**< SMICmd val to enter S4BIOS state */
    uint8_t             u8PStateCnt;            /**< SMICmd val to assume processor performance
                                                     state control responsibility */
    uint32_t            u32PM1aEVTBLK;          /**< port addr of PM1a event regs block */
    uint32_t            u32PM1bEVTBLK;          /**< port addr of PM1b event regs block */
    uint32_t            u32PM1aCTLBLK;          /**< port addr of PM1a control regs block */
    uint32_t            u32PM1bCTLBLK;          /**< port addr of PM1b control regs block */
    uint32_t            u32PM2CTLBLK;           /**< port addr of PM2 control regs block */
    uint32_t            u32PMTMRBLK;            /**< port addr of PMTMR regs block */
    uint32_t            u32GPE0BLK;             /**< port addr of gen-purp event 0 regs block */
    uint32_t            u32GPE1BLK;             /**< port addr of gen-purp event 1 regs block */
    uint8_t             u8PM1EVTLEN;            /**< bytes decoded by PM1a_EVT_BLK. >= 4 */
    uint8_t             u8PM1CTLLEN;            /**< bytes decoded by PM1b_CNT_BLK. >= 2 */
    uint8_t             u8PM2CTLLEN;            /**< bytes decoded by PM2_CNT_BLK. >= 1 or 0 */
    uint8_t             u8PMTMLEN;              /**< bytes decoded by PM_TMR_BLK. ==4 */
    uint8_t             u8GPE0BLKLEN;           /**< bytes decoded by GPE0_BLK. %2==0 */
#define GPE0_BLK_LEN    2
    uint8_t             u8GPE1BLKLEN;           /**< bytes decoded by GPE1_BLK. %2==0 */
#define GPE1_BLK_LEN    0
    uint8_t             u8GPE1BASE;             /**< offset of GPE1 based events */
#define GPE1_BASE       0
    uint8_t             u8CSTCNT;               /**< SMICmd val to indicate OS supp for C states */
    uint16_t            u16PLVL2LAT;            /**< us to enter/exit C2. >100 => unsupported */
#define P_LVL2_LAT      101                     /**< C2 state not supported */
    uint16_t            u16PLVL3LAT;            /**< us to enter/exit C3. >1000 => unsupported */
#define P_LVL3_LAT      1001                    /**< C3 state not supported */
    uint16_t            u16FlushSize;           /**< # of flush strides to read to flush dirty
                                                     lines from any processors memory caches */
#define FLUSH_SIZE      0                       /**< Ignored if WBVIND set in FADT_FLAGS */
    uint16_t            u16FlushStride;         /**< cache line width */
#define FLUSH_STRIDE    0                       /**< Ignored if WBVIND set in FADT_FLAGS */
    uint8_t             u8DutyOffset;
    uint8_t             u8DutyWidth;
    uint8_t             u8DayAlarm;             /**< RTC CMOS RAM index of day-of-month alarm */
    uint8_t             u8MonAlarm;             /**< RTC CMOS RAM index of month-of-year alarm */
    uint8_t             u8Century;              /**< RTC CMOS RAM index of century */
    uint16_t            u16IAPCBOOTARCH;        /**< IA-PC boot architecture flags */
#define IAPC_BOOT_ARCH_LEGACY_DEV       RT_BIT(0)  /**< legacy devices present such as LPT
                                                     (COM too?) */
#define IAPC_BOOT_ARCH_8042             RT_BIT(1)  /**< legacy keyboard device present */
#define IAPC_BOOT_ARCH_NO_VGA           RT_BIT(2)  /**< VGA not present */
    uint8_t             u8Must0_0;                 /**< must be 0 */
    uint32_t            u32Flags;                  /**< fixed feature flags */
#define FADT_FL_WBINVD                  RT_BIT(0)  /**< emulation of WBINVD available */
#define FADT_FL_WBINVD_FLUSH            RT_BIT(1)
#define FADT_FL_PROC_C1                 RT_BIT(2)  /**< 1=C1 supported on all processors */
#define FADT_FL_P_LVL2_UP               RT_BIT(3)  /**< 1=C2 works on SMP and UNI systems */
#define FADT_FL_PWR_BUTTON              RT_BIT(4)  /**< 1=power button handled as ctrl method dev */
#define FADT_FL_SLP_BUTTON              RT_BIT(5)  /**< 1=sleep button handled as ctrl method dev */
#define FADT_FL_FIX_RTC                 RT_BIT(6)  /**< 0=RTC wake status in fixed register */
#define FADT_FL_RTC_S4                  RT_BIT(7)  /**< 1=RTC can wake system from S4 */
#define FADT_FL_TMR_VAL_EXT             RT_BIT(8)  /**< 1=TMR_VAL implemented as 32 bit */
#define FADT_FL_DCK_CAP                 RT_BIT(9)  /**< 0=system cannot support docking */
#define FADT_FL_RESET_REG_SUP           RT_BIT(10) /**< 1=system supports system resets */
#define FADT_FL_SEALED_CASE             RT_BIT(11) /**< 1=case is sealed */
#define FADT_FL_HEADLESS                RT_BIT(12) /**< 1=system cannot detect moni/keyb/mouse */
#define FADT_FL_CPU_SW_SLP              RT_BIT(13)
#define FADT_FL_PCI_EXT_WAK             RT_BIT(14) /**< 1=system supports PCIEXP_WAKE_STS */
#define FADT_FL_USE_PLATFORM_CLOCK      RT_BIT(15) /**< 1=system has ACPI PM timer */
#define FADT_FL_S4_RTC_STS_VALID        RT_BIT(16) /**< 1=RTC_STS flag is valid when waking from S4 */
#define FADT_FL_REMOVE_POWER_ON_CAPABLE RT_BIT(17) /**< 1=platform can remote power on */
#define FADT_FL_FORCE_APIC_CLUSTER_MODEL  RT_BIT(18)
#define FADT_FL_FORCE_APIC_PHYS_DEST_MODE RT_BIT(19)

    /** Start of the ACPI 2.0 extension. */
    ACPIGENADDR         ResetReg;               /**< ext addr of reset register */
    uint8_t             u8ResetVal;             /**< ResetReg value to reset the system */
#define ACPI_RESET_REG_VAL  0x10
    uint8_t             au8Must0_1[3];          /**< must be 0 */
    uint64_t            u64XFACS;               /**< 64-bit phys address of FACS */
    uint64_t            u64XDSDT;               /**< 64-bit phys address of DSDT */
    ACPIGENADDR         X_PM1aEVTBLK;           /**< ext addr of PM1a event regs block */
    ACPIGENADDR         X_PM1bEVTBLK;           /**< ext addr of PM1b event regs block */
    ACPIGENADDR         X_PM1aCTLBLK;           /**< ext addr of PM1a control regs block */
    ACPIGENADDR         X_PM1bCTLBLK;           /**< ext addr of PM1b control regs block */
    ACPIGENADDR         X_PM2CTLBLK;            /**< ext addr of PM2 control regs block */
    ACPIGENADDR         X_PMTMRBLK;             /**< ext addr of PMTMR control regs block */
    ACPIGENADDR         X_GPE0BLK;              /**< ext addr of GPE1 regs block */
    ACPIGENADDR         X_GPE1BLK;              /**< ext addr of GPE1 regs block */
};
AssertCompileSize(ACPITBLFADT, 244);
#define ACPITBLFADT_VERSION1_SIZE               RT_OFFSETOF(ACPITBLFADT, ResetReg)

/** Firmware ACPI Control Structure */
struct ACPITBLFACS
{
    uint8_t             au8Signature[4];        /**< 'FACS' */
    uint32_t            u32Length;              /**< bytes of entire FACS structure >= 64 */
    uint32_t            u32HWSignature;         /**< systems HW signature at last boot */
    uint32_t            u32FWVector;            /**< address of waking vector */
    uint32_t            u32GlobalLock;          /**< global lock to sync HW/SW */
    uint32_t            u32Flags;               /**< FACS flags */
    uint64_t            u64X_FWVector;          /**< 64-bit waking vector */
    uint8_t             u8Version;              /**< version of this table */
    uint8_t             au8Reserved[31];        /**< zero */
};
AssertCompileSize(ACPITBLFACS, 64);

/** Processor Local APIC Structure */
struct ACPITBLLAPIC
{
    uint8_t             u8Type;                 /**< 0 = LAPIC */
    uint8_t             u8Length;               /**< 8 */
    uint8_t             u8ProcId;               /**< processor ID */
    uint8_t             u8ApicId;               /**< local APIC ID */
    uint32_t            u32Flags;               /**< Flags */
#define LAPIC_ENABLED   0x1
};
AssertCompileSize(ACPITBLLAPIC, 8);

/** I/O APIC Structure */
struct ACPITBLIOAPIC
{
    uint8_t             u8Type;                 /**< 1 == I/O APIC */
    uint8_t             u8Length;               /**< 12 */
    uint8_t             u8IOApicId;             /**< I/O APIC ID */
    uint8_t             u8Reserved;             /**< 0 */
    uint32_t            u32Address;             /**< phys address to access I/O APIC */
    uint32_t            u32GSIB;                /**< global system interrupt number to start */
};
AssertCompileSize(ACPITBLIOAPIC, 12);

/** Interrupt Source Override Structure */
struct ACPITBLISO
{
    uint8_t             u8Type;                 /**< 2 ==  Interrupt Source Override*/
    uint8_t             u8Length;               /**< 10 */
    uint8_t             u8Bus;                  /**< Bus */
    uint8_t             u8Source;               /**< Bus-relative interrupt source (IRQ) */
    uint32_t            u32GSI;                 /**< Global System Interrupt */
    uint16_t            u16Flags;               /**< MPS INTI flags Global */
};
AssertCompileSize(ACPITBLISO, 10);
#define NUMBER_OF_IRQ_SOURCE_OVERRIDES 2

/** HPET Descriptor Structure */
struct ACPITBLHPET
{
    ACPITBLHEADER aHeader;
    uint32_t      u32Id;                        /**< hardware ID of event timer block
                                                     [31:16] PCI vendor ID of first timer block
                                                     [15]    legacy replacement IRQ routing capable
                                                     [14]    reserved
                                                     [13]    COUNT_SIZE_CAP counter size
                                                     [12:8]  number of comparators in first timer block
                                                     [7:0]   hardware rev ID */
    ACPIGENADDR   HpetAddr;                     /**< lower 32-bit base address */
    uint8_t       u32Number;                    /**< sequence number starting at 0 */
    uint16_t      u32MinTick;                   /**< minimum clock ticks which can be set without
                                                     lost interrupts while the counter is programmed
                                                     to operate in periodic mode. Unit: clock tick. */
    uint8_t       u8Attributes;                 /**< page protection and OEM attribute. */
};
AssertCompileSize(ACPITBLHPET, 56);

/** MCFG Descriptor Structure */
typedef struct ACPITBLMCFG
{
    ACPITBLHEADER aHeader;
    uint64_t      u64Reserved;
} ACPITBLMCFG;
AssertCompileSize(ACPITBLMCFG, 44);

/** Number of such entries can be computed from the whole table length in header */
typedef struct ACPITBLMCFGENTRY
{
    uint64_t      u64BaseAddress;
    uint16_t      u16PciSegmentGroup;
    uint8_t       u8StartBus;
    uint8_t       u8EndBus;
    uint32_t      u32Reserved;
} ACPITBLMCFGENTRY;
AssertCompileSize(ACPITBLMCFGENTRY, 16);

# ifdef IN_RING3 /** @todo r=bird: Move this down to where it's used. */

#  define PCAT_COMPAT   0x1                     /**< system has also a dual-8259 setup */

/** Custom Description Table */
struct ACPITBLCUST
{
    ACPITBLHEADER       header;
    uint8_t             au8Data[476];
};
AssertCompileSize(ACPITBLCUST, 512);

/**
 * Multiple APIC Description Table.
 *
 * This structure looks somewhat convoluted due layout of MADT table in MP case.
 * There extpected to be multiple LAPIC records for each CPU, thus we cannot
 * use regular C structure and proxy to raw memory instead.
 */
class AcpiTableMADT
{
    /**
     * All actual data stored in dynamically allocated memory pointed by this field.
     */
    uint8_t            *m_pbData;
    /**
     * Number of CPU entries in this MADT.
     */
    uint32_t            m_cCpus;

    /**
     * Number of interrupt overrides.
     */
     uint32_t            m_cIsos;

public:
    /**
     * Address of ACPI header
     */
    inline ACPITBLHEADER *header_addr(void) const
    {
        return (ACPITBLHEADER *)m_pbData;
    }

    /**
     * Address of local APIC for each CPU. Note that different CPUs address different LAPICs,
     * although address is the same for all of them.
     */
    inline uint32_t *u32LAPIC_addr(void) const
    {
        return (uint32_t *)(header_addr() + 1);
    }

    /**
     * Address of APIC flags
     */
    inline uint32_t *u32Flags_addr(void) const
    {
        return (uint32_t *)(u32LAPIC_addr() + 1);
    }

    /**
     * Address of ISO description
     */
    inline ACPITBLISO *ISO_addr(void) const
    {
        return (ACPITBLISO *)(u32Flags_addr() + 1);
    }

    /**
     * Address of per-CPU LAPIC descriptions
     */
    inline ACPITBLLAPIC *LApics_addr(void) const
    {
        return (ACPITBLLAPIC *)(ISO_addr() + m_cIsos);
    }

    /**
     * Address of IO APIC description
     */
    inline ACPITBLIOAPIC *IOApic_addr(void) const
    {
        return (ACPITBLIOAPIC *)(LApics_addr() + m_cCpus);
    }

    /**
     * Size of MADT.
     * Note that this function assumes IOApic to be the last field in structure.
     */
    inline uint32_t size(void) const
    {
        return (uint8_t *)(IOApic_addr() + 1) - (uint8_t *)header_addr();
    }

    /**
     * Raw data of MADT.
     */
    inline const uint8_t *data(void) const
    {
        return m_pbData;
    }

    /**
     * Size of MADT for given ACPI config, useful to compute layout.
     */
    static uint32_t sizeFor(ACPIState *pThis, uint32_t cIsos)
    {
        return AcpiTableMADT(pThis->cCpus, cIsos).size();
    }

    /*
     * Constructor, only works in Ring 3, doesn't look like a big deal.
     */
    AcpiTableMADT(uint32_t cCpus, uint32_t cIsos)
    {
        m_cCpus  = cCpus;
        m_cIsos  = cIsos;
        m_pbData = NULL;                /* size() uses this and gcc will complain if not initialized. */
        uint32_t cb = size();
        m_pbData = (uint8_t *)RTMemAllocZ(cb);
    }

    ~AcpiTableMADT()
    {
        RTMemFree(m_pbData);
    }
};
# endif /* IN_RING3 */

#pragma pack()


#ifndef VBOX_DEVICE_STRUCT_TESTCASE /* exclude the rest of the file */
/*******************************************************************************
*   Internal Functions                                                         *
*******************************************************************************/
RT_C_DECLS_BEGIN
PDMBOTHCBDECL(int) acpiPMTmrRead(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t *pu32, unsigned cb);
RT_C_DECLS_END
#ifdef IN_RING3
static int acpiPlantTables(ACPIState *pThis);
#endif

#ifdef IN_RING3

/* SCI IRQ */
DECLINLINE(void) acpiSetIrq(ACPIState *pThis, int level)
{
    if (pThis->pm1a_ctl & SCI_EN)
        PDMDevHlpPCISetIrq(pThis->pDevIns, 0, level);
}

DECLINLINE(uint32_t) pm1a_pure_en(uint32_t en)
{
    return en & ~(RSR_EN | IGN_EN);
}

DECLINLINE(uint32_t) pm1a_pure_sts(uint32_t sts)
{
    return sts & ~(RSR_STS | IGN_STS);
}

DECLINLINE(int) pm1a_level(ACPIState *pThis)
{
    return (pm1a_pure_en(pThis->pm1a_en) & pm1a_pure_sts(pThis->pm1a_sts)) != 0;
}

DECLINLINE(int) gpe0_level(ACPIState *pThis)
{
    return (pThis->gpe0_en & pThis->gpe0_sts) != 0;
}

/**
 * Used by acpiPM1aStsWrite, acpiPM1aEnWrite, acpiPmTimer,
 * acpiPort_PowerBuffonPress and acpiPort_SleepButtonPress to
 * update the GPE0.STS and GPE0.EN registers and trigger IRQs.
 *
 * Caller must hold the state lock.
 *
 * @param   pThis       The ACPI instance.
 * @param   sts         The new GPE0.STS value.
 * @param   en          The new GPE0.EN value.
 */
static void update_pm1a(ACPIState *pThis, uint32_t sts, uint32_t en)
{
    Assert(PDMCritSectIsOwner(&pThis->CritSect));

    if (gpe0_level(pThis))
        return;

    int const old_level = pm1a_level(pThis);
    int const new_level = (pm1a_pure_en(en) & pm1a_pure_sts(sts)) != 0;

    Log(("update_pm1a() old=%x new=%x\n", old_level, new_level));

    pThis->pm1a_en = en;
    pThis->pm1a_sts = sts;

    if (new_level != old_level)
        acpiSetIrq(pThis, new_level);
}

/**
 * Used by acpiGpe0StsWrite, acpiGpe0EnWrite, acpiAttach and acpiDetach to
 * update the GPE0.STS and GPE0.EN registers and trigger IRQs.
 *
 * Caller must hold the state lock.
 *
 * @param   pThis       The ACPI instance.
 * @param   sts         The new GPE0.STS value.
 * @param   en          The new GPE0.EN value.
 */
static void update_gpe0(ACPIState *pThis, uint32_t sts, uint32_t en)
{
    Assert(PDMCritSectIsOwner(&pThis->CritSect));

    if (pm1a_level(pThis))
        return;

    int const old_level = (pThis->gpe0_en & pThis->gpe0_sts) != 0;
    int const new_level = (en & sts) != 0;

    pThis->gpe0_en  = en;
    pThis->gpe0_sts = sts;

    if (new_level != old_level)
        acpiSetIrq(pThis, new_level);
}

/**
 * Used by acpiPM1aCtlWrite to power off the VM.
 *
 * @param   pThis   The ACPI instance.
 * @returns Strict VBox status code.
 */
static int acpiPowerOff(ACPIState *pThis)
{
    int rc = PDMDevHlpVMPowerOff(pThis->pDevIns);
    if (RT_FAILURE(rc))
        AssertMsgFailed(("Could not power down the VM. rc = %Rrc\n", rc));
    return rc;
}

/**
 * Used by acpiPM1aCtlWrite to put the VM to sleep.
 *
 * @param   pThis   The ACPI instance.
 * @returns Strict VBox status code.
 */
static int acpiSleep(ACPIState *pThis)
{
    /* We must set WAK_STS on resume (includes restore) so the guest knows that
       we've woken up and can continue executing code.  The guest is probably
       reading the PMSTS register in a loop to check this. */
    int rc;
    pThis->fSetWakeupOnResume = true;
    if (pThis->fSuspendToSavedState)
    {
        rc = PDMDevHlpVMSuspendSaveAndPowerOff(pThis->pDevIns);
        if (rc != VERR_NOT_SUPPORTED)
            AssertRC(rc);
        else
        {
            LogRel(("ACPI: PDMDevHlpVMSuspendSaveAndPowerOff is not supported, falling back to suspend-only\n"));
            rc = PDMDevHlpVMSuspend(pThis->pDevIns);
            AssertRC(rc);
        }
    }
    else
    {
        rc = PDMDevHlpVMSuspend(pThis->pDevIns);
        AssertRC(rc);
    }
    return rc;
}


/**
 * @interface_method_impl{PDMIACPIPORT,pfnPowerButtonPress}
 */
static DECLCALLBACK(int) acpiPort_PowerButtonPress(PPDMIACPIPORT pInterface)
{
    ACPIState *pThis = RT_FROM_MEMBER(pInterface, ACPIState, IACPIPort);
    DEVACPI_LOCK_R3(pThis);

    Log(("acpiPort_PowerButtonPress: handled=%d status=%x\n", pThis->fPowerButtonHandled, pThis->pm1a_sts));
    pThis->fPowerButtonHandled = false;
    update_pm1a(pThis, pThis->pm1a_sts | PWRBTN_STS, pThis->pm1a_en);

    DEVACPI_UNLOCK(pThis);
    return VINF_SUCCESS;
}

/**
 * @interface_method_impl{PDMIACPIPORT,pfnGetPowerButtonHandled}
 */
static DECLCALLBACK(int) acpiPort_GetPowerButtonHandled(PPDMIACPIPORT pInterface, bool *pfHandled)
{
    ACPIState *pThis = RT_FROM_MEMBER(pInterface, ACPIState, IACPIPort);
    DEVACPI_LOCK_R3(pThis);

    *pfHandled = pThis->fPowerButtonHandled;

    DEVACPI_UNLOCK(pThis);
    return VINF_SUCCESS;
}

/**
 * @interface_method_impl{PDMIACPIPORT,pfnGetGuestEnteredACPIMode, Check if the
 *                       Guest entered into G0 (working) or G1 (sleeping)}
 */
static DECLCALLBACK(int) acpiPort_GetGuestEnteredACPIMode(PPDMIACPIPORT pInterface, bool *pfEntered)
{
    ACPIState *pThis = RT_FROM_MEMBER(pInterface, ACPIState, IACPIPort);
    DEVACPI_LOCK_R3(pThis);

    *pfEntered = (pThis->pm1a_ctl & SCI_EN) != 0;

    DEVACPI_UNLOCK(pThis);
    return VINF_SUCCESS;
}

/**
 * @interface_method_impl{PDMIACPIPORT,pfnGetCpuStatus}
 */
static DECLCALLBACK(int) acpiPort_GetCpuStatus(PPDMIACPIPORT pInterface, unsigned uCpu, bool *pfLocked)
{
    ACPIState *pThis = RT_FROM_MEMBER(pInterface, ACPIState, IACPIPort);
    DEVACPI_LOCK_R3(pThis);

    *pfLocked = VMCPUSET_IS_PRESENT(&pThis->CpuSetLocked, uCpu);

    DEVACPI_UNLOCK(pThis);
    return VINF_SUCCESS;
}

/**
 * Send an ACPI sleep button event.
 *
 * @returns VBox status code
 * @param   pInterface      Pointer to the interface structure containing the called function pointer.
 */
static DECLCALLBACK(int) acpiPort_SleepButtonPress(PPDMIACPIPORT pInterface)
{
    ACPIState *pThis = RT_FROM_MEMBER(pInterface, ACPIState, IACPIPort);
    DEVACPI_LOCK_R3(pThis);

    update_pm1a(pThis, pThis->pm1a_sts | SLPBTN_STS, pThis->pm1a_en);

    DEVACPI_UNLOCK(pThis);
    return VINF_SUCCESS;
}

/**
 * Used by acpiPmTimer to re-arm the PM timer.
 *
 * The caller is expected to either hold the clock lock or to have made sure
 * the VM is resetting or loading state.
 *
 * @param   pThis               The ACPI instance.
 * @param   uNow                The current time.
 */
static void acpiPmTimerReset(ACPIState *pThis, uint64_t uNow)
{
    uint64_t uTimerFreq = TMTimerGetFreq(pThis->CTX_SUFF(pPmTimer));
    uint64_t uInterval  = ASMMultU64ByU32DivByU32(0xffffffff, uTimerFreq, PM_TMR_FREQ);
    TMTimerSet(pThis->pPmTimerR3, uNow + uInterval);
    Log(("acpi: uInterval = %RU64\n", uInterval));
}

/**
 * @callback_method_impl{FNTMTIMERDEV, PM Timer callback}
 */
static DECLCALLBACK(void) acpiPmTimer(PPDMDEVINS pDevIns, PTMTIMER pTimer, void *pvUser)
{
    ACPIState *pThis = (ACPIState *)pvUser;
    Assert(TMTimerIsLockOwner(pTimer));
    NOREF(pDevIns);

    DEVACPI_LOCK_R3(pThis);
    Log(("acpi: pm timer sts %#x (%d), en %#x (%d)\n",
         pThis->pm1a_sts, (pThis->pm1a_sts & TMR_STS) != 0,
         pThis->pm1a_en, (pThis->pm1a_en & TMR_EN) != 0));
    update_pm1a(pThis, pThis->pm1a_sts | TMR_STS, pThis->pm1a_en);
    DEVACPI_UNLOCK(pThis);

    acpiPmTimerReset(pThis, TMTimerGet(pTimer));
}

/**
 * _BST method - used by acpiBatDataRead to implement BAT_STATUS_STATE and
 * acpiLoadState.
 *
 * @returns VINF_SUCCESS.
 * @param   pThis           The ACPI instance.
 */
static int acpiFetchBatteryStatus(ACPIState *pThis)
{
    uint32_t           *p = pThis->au8BatteryInfo;
    bool               fPresent;              /* battery present? */
    PDMACPIBATCAPACITY hostRemainingCapacity; /* 0..100 */
    PDMACPIBATSTATE    hostBatteryState;      /* bitfield */
    uint32_t           hostPresentRate;       /* 0..1000 */
    int                rc;

    if (!pThis->pDrv)
        return VINF_SUCCESS;
    rc = pThis->pDrv->pfnQueryBatteryStatus(pThis->pDrv, &fPresent, &hostRemainingCapacity,
                                            &hostBatteryState, &hostPresentRate);
    AssertRC(rc);

    /* default values */
    p[BAT_STATUS_STATE]              = hostBatteryState;
    p[BAT_STATUS_PRESENT_RATE]       = hostPresentRate == ~0U ? 0xFFFFFFFF
                                                              : hostPresentRate * 50;  /* mW */
    p[BAT_STATUS_REMAINING_CAPACITY] = 50000; /* mWh */
    p[BAT_STATUS_PRESENT_VOLTAGE]    = 10000; /* mV */

    /* did we get a valid battery state? */
    if (hostRemainingCapacity != PDM_ACPI_BAT_CAPACITY_UNKNOWN)
        p[BAT_STATUS_REMAINING_CAPACITY] = hostRemainingCapacity * 500; /* mWh */
    if (hostBatteryState == PDM_ACPI_BAT_STATE_CHARGED)
        p[BAT_STATUS_PRESENT_RATE] = 0; /* mV */

    return VINF_SUCCESS;
}

/**
 * _BIF method - used by acpiBatDataRead to implement BAT_INFO_UNITS and
 * acpiLoadState.
 *
 * @returns VINF_SUCCESS.
 * @param   pThis           The ACPI instance.
 */
static int acpiFetchBatteryInfo(ACPIState *pThis)
{
    uint32_t *p = pThis->au8BatteryInfo;

    p[BAT_INFO_UNITS]                      = 0;     /* mWh */
    p[BAT_INFO_DESIGN_CAPACITY]            = 50000; /* mWh */
    p[BAT_INFO_LAST_FULL_CHARGE_CAPACITY]  = 50000; /* mWh */
    p[BAT_INFO_TECHNOLOGY]                 = BAT_TECH_PRIMARY;
    p[BAT_INFO_DESIGN_VOLTAGE]             = 10000; /* mV */
    p[BAT_INFO_DESIGN_CAPACITY_OF_WARNING] = 100;   /* mWh */
    p[BAT_INFO_DESIGN_CAPACITY_OF_LOW]     = 50;    /* mWh */
    p[BAT_INFO_CAPACITY_GRANULARITY_1]     = 1;     /* mWh */
    p[BAT_INFO_CAPACITY_GRANULARITY_2]     = 1;     /* mWh */

    return VINF_SUCCESS;
}

/**
 * The _STA method - used by acpiBatDataRead to implement BAT_DEVICE_STATUS.
 *
 * @returns status mask or 0.
 * @param   pThis           The ACPI instance.
 */
static uint32_t acpiGetBatteryDeviceStatus(ACPIState *pThis)
{
    bool               fPresent;              /* battery present? */
    PDMACPIBATCAPACITY hostRemainingCapacity; /* 0..100 */
    PDMACPIBATSTATE    hostBatteryState;      /* bitfield */
    uint32_t           hostPresentRate;       /* 0..1000 */
    int                rc;

    if (!pThis->pDrv)
        return 0;
    rc = pThis->pDrv->pfnQueryBatteryStatus(pThis->pDrv, &fPresent, &hostRemainingCapacity,
                                            &hostBatteryState, &hostPresentRate);
    AssertRC(rc);

    return fPresent
        ?   STA_DEVICE_PRESENT_MASK                     /* present */
          | STA_DEVICE_ENABLED_MASK                     /* enabled and decodes its resources */
          | STA_DEVICE_SHOW_IN_UI_MASK                  /* should be shown in UI */
          | STA_DEVICE_FUNCTIONING_PROPERLY_MASK        /* functioning properly */
          | STA_BATTERY_PRESENT_MASK                    /* battery is present */
        : 0;                                            /* device not present */
}

/**
 * Used by acpiBatDataRead to implement BAT_POWER_SOURCE.
 *
 * @returns status.
 * @param   pThis           The ACPI instance.
 */
static uint32_t acpiGetPowerSource(ACPIState *pThis)
{
    /* query the current power source from the host driver */
    if (!pThis->pDrv)
        return AC_ONLINE;

    PDMACPIPOWERSOURCE ps;
    int rc = pThis->pDrv->pfnQueryPowerSource(pThis->pDrv, &ps);
    AssertRC(rc);
    return ps == PDM_ACPI_POWER_SOURCE_BATTERY ? AC_OFFLINE : AC_ONLINE;
}

/**
 * @callback_method_impl{FNIOMIOPORTOUT, Battery status index}
 */
PDMBOTHCBDECL(int) acpiBatIndexWrite(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t u32, unsigned cb)
{
    Log(("acpiBatIndexWrite: %#x (%#x)\n", u32, u32 >> 2));
    if (cb != 4)
        return PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u u32=%#x\n", cb, Port, u32);

    ACPIState *pThis = (ACPIState *)pvUser;
    DEVACPI_LOCK_R3(pThis);

    u32 >>= pThis->u8IndexShift;
    /* see comment at the declaration of u8IndexShift */
    if (pThis->u8IndexShift == 0 && u32 == (BAT_DEVICE_STATUS << 2))
    {
        pThis->u8IndexShift = 2;
        u32 >>= 2;
    }
    Assert(u32 < BAT_INDEX_LAST);
    pThis->uBatteryIndex = u32;

    DEVACPI_UNLOCK(pThis);
    return VINF_SUCCESS;
}

/**
 * @callback_method_impl{FNIOMIOPORTIN, Battery status data}
 */
PDMBOTHCBDECL(int) acpiBatDataRead(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t *pu32, unsigned cb)
{
    if (cb != 4)
        return VERR_IOM_IOPORT_UNUSED;

    ACPIState *pThis = (ACPIState *)pvUser;
    DEVACPI_LOCK_R3(pThis);

    int rc = VINF_SUCCESS;
    switch (pThis->uBatteryIndex)
    {
        case BAT_STATUS_STATE:
            acpiFetchBatteryStatus(pThis);
            /* fall thru */
        case BAT_STATUS_PRESENT_RATE:
        case BAT_STATUS_REMAINING_CAPACITY:
        case BAT_STATUS_PRESENT_VOLTAGE:
            *pu32 = pThis->au8BatteryInfo[pThis->uBatteryIndex];
            break;

        case BAT_INFO_UNITS:
            acpiFetchBatteryInfo(pThis);
            /* fall thru */
        case BAT_INFO_DESIGN_CAPACITY:
        case BAT_INFO_LAST_FULL_CHARGE_CAPACITY:
        case BAT_INFO_TECHNOLOGY:
        case BAT_INFO_DESIGN_VOLTAGE:
        case BAT_INFO_DESIGN_CAPACITY_OF_WARNING:
        case BAT_INFO_DESIGN_CAPACITY_OF_LOW:
        case BAT_INFO_CAPACITY_GRANULARITY_1:
        case BAT_INFO_CAPACITY_GRANULARITY_2:
            *pu32 = pThis->au8BatteryInfo[pThis->uBatteryIndex];
            break;

        case BAT_DEVICE_STATUS:
            *pu32 = acpiGetBatteryDeviceStatus(pThis);
            break;

        case BAT_POWER_SOURCE:
            *pu32 = acpiGetPowerSource(pThis);
            break;

        default:
            rc = PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u idx=%u\n", cb, Port, pThis->uBatteryIndex);
            *pu32 = UINT32_MAX;
            break;
    }

    DEVACPI_UNLOCK(pThis);
    return rc;
}

/**
 * @callback_method_impl{FNIOMIOPORTOUT, System info index}
 */
PDMBOTHCBDECL(int) acpiSysInfoIndexWrite(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t u32, unsigned cb)
{
    Log(("acpiSysInfoIndexWrite: %#x (%#x)\n", u32, u32 >> 2));
    if (cb != 4)
        return PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u u32=%#x\n", cb, Port, u32);

    ACPIState *pThis = (ACPIState *)pvUser;
    DEVACPI_LOCK_R3(pThis);

    if (u32 == SYSTEM_INFO_INDEX_VALID || u32 == SYSTEM_INFO_INDEX_INVALID)
        pThis->uSystemInfoIndex = u32;
    else
    {
        /* see comment at the declaration of u8IndexShift */
        if (u32 > SYSTEM_INFO_INDEX_END && pThis->u8IndexShift == 0)
        {
            if ((u32 >> 2) < SYSTEM_INFO_INDEX_END && (u32 & 0x3) == 0)
                pThis->u8IndexShift = 2;
        }

        u32 >>= pThis->u8IndexShift;
        Assert(u32 < SYSTEM_INFO_INDEX_END);
        pThis->uSystemInfoIndex = u32;
    }

    DEVACPI_UNLOCK(pThis);
    return VINF_SUCCESS;
}

/**
 * @callback_method_impl{FNIOMIOPORTIN, System info data}
 */
PDMBOTHCBDECL(int) acpiSysInfoDataRead(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t *pu32, unsigned cb)
{
    if (cb != 4)
        return VERR_IOM_IOPORT_UNUSED;

    ACPIState *pThis = (ACPIState *)pvUser;
    DEVACPI_LOCK_R3(pThis);

    int rc = VINF_SUCCESS;
    uint32_t const uSystemInfoIndex = pThis->uSystemInfoIndex;
    switch (uSystemInfoIndex)
    {
        case SYSTEM_INFO_INDEX_LOW_MEMORY_LENGTH:
            *pu32 = pThis->cbRamLow;
            break;

        case SYSTEM_INFO_INDEX_HIGH_MEMORY_LENGTH:
            *pu32 = pThis->cbRamHigh >> 16; /* 64KB units */
            Assert(((uint64_t)*pu32 << 16) == pThis->cbRamHigh);
            break;

        case SYSTEM_INFO_INDEX_USE_IOAPIC:
            *pu32 = pThis->u8UseIOApic;
            break;

        case SYSTEM_INFO_INDEX_HPET_STATUS:
            *pu32 = pThis->fUseHpet
                  ? (  STA_DEVICE_PRESENT_MASK
                     | STA_DEVICE_ENABLED_MASK
                     | STA_DEVICE_SHOW_IN_UI_MASK
                     | STA_DEVICE_FUNCTIONING_PROPERLY_MASK)
                  : 0;
            break;

        case SYSTEM_INFO_INDEX_SMC_STATUS:
            *pu32 = pThis->fUseSmc
                  ? (  STA_DEVICE_PRESENT_MASK
                     | STA_DEVICE_ENABLED_MASK
                     /* no need to show this device in the UI */
                     | STA_DEVICE_FUNCTIONING_PROPERLY_MASK)
                  : 0;
            break;

        case SYSTEM_INFO_INDEX_FDC_STATUS:
            *pu32 = pThis->fUseFdc
                  ? (  STA_DEVICE_PRESENT_MASK
                     | STA_DEVICE_ENABLED_MASK
                     | STA_DEVICE_SHOW_IN_UI_MASK
                     | STA_DEVICE_FUNCTIONING_PROPERLY_MASK)
                  : 0;
            break;

        case SYSTEM_INFO_INDEX_NIC_ADDRESS:
            *pu32 = pThis->u32NicPciAddress;
            break;

        case SYSTEM_INFO_INDEX_AUDIO_ADDRESS:
            *pu32 = pThis->u32AudioPciAddress;
            break;

        case SYSTEM_INFO_INDEX_POWER_STATES:
            *pu32 = RT_BIT(0) | RT_BIT(5);  /* S1 and S5 always exposed */
            if (pThis->fS1Enabled)          /* Optionally expose S1 and S4 */
                *pu32 |= RT_BIT(1);
            if (pThis->fS4Enabled)
                *pu32 |= RT_BIT(4);
            break;

       case SYSTEM_INFO_INDEX_IOC_ADDRESS:
            *pu32 = pThis->u32IocPciAddress;
            break;

        case SYSTEM_INFO_INDEX_HBC_ADDRESS:
            *pu32 = pThis->u32HbcPciAddress;
            break;

        case SYSTEM_INFO_INDEX_PCI_BASE:
            /** @todo couldn't MCFG be in 64-bit range? */
            Assert(pThis->u64PciConfigMMioAddress < 0xffffffff);
            *pu32 = (uint32_t)pThis->u64PciConfigMMioAddress;
            break;

        case SYSTEM_INFO_INDEX_PCI_LENGTH:
            /** @todo couldn't MCFG be in 64-bit range? */
            Assert(pThis->u64PciConfigMMioLength< 0xffffffff);
            *pu32 = (uint32_t)pThis->u64PciConfigMMioLength;
            break;

        /* This is only for compatibility with older saved states that
           may include ACPI code that read these values.  Legacy is
           a wonderful thing, isn't it? :-) */
        case SYSTEM_INFO_INDEX_CPU0_STATUS:
        case SYSTEM_INFO_INDEX_CPU1_STATUS:
        case SYSTEM_INFO_INDEX_CPU2_STATUS:
        case SYSTEM_INFO_INDEX_CPU3_STATUS:
            *pu32 = (   pThis->fShowCpu
                     && pThis->uSystemInfoIndex - SYSTEM_INFO_INDEX_CPU0_STATUS < pThis->cCpus
                     && VMCPUSET_IS_PRESENT(&pThis->CpuSetAttached,
                                            pThis->uSystemInfoIndex - SYSTEM_INFO_INDEX_CPU0_STATUS) )
                  ? (  STA_DEVICE_PRESENT_MASK
                     | STA_DEVICE_ENABLED_MASK
                     | STA_DEVICE_SHOW_IN_UI_MASK
                     | STA_DEVICE_FUNCTIONING_PROPERLY_MASK)
                  : 0;
            break;

        case SYSTEM_INFO_INDEX_RTC_STATUS:
            *pu32 = pThis->fShowRtc
                  ? (  STA_DEVICE_PRESENT_MASK
                     | STA_DEVICE_ENABLED_MASK
                     | STA_DEVICE_SHOW_IN_UI_MASK
                     | STA_DEVICE_FUNCTIONING_PROPERLY_MASK)
                  : 0;
            break;

        case SYSTEM_INFO_INDEX_CPU_LOCKED:
            if (pThis->idCpuLockCheck < VMM_MAX_CPU_COUNT)
            {
                *pu32 = VMCPUSET_IS_PRESENT(&pThis->CpuSetLocked, pThis->idCpuLockCheck);
                pThis->idCpuLockCheck = UINT32_C(0xffffffff); /* Make the entry invalid */
            }
            else
            {
                rc = PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "CPU lock check protocol violation (idCpuLockCheck=%#x)\n",
                                       pThis->idCpuLockCheck);
                /* Always return locked status just to be safe */
                *pu32 = 1;
            }
            break;

        case SYSTEM_INFO_INDEX_CPU_EVENT_TYPE:
            *pu32 = pThis->u32CpuEventType;
            break;

        case SYSTEM_INFO_INDEX_CPU_EVENT:
            *pu32 = pThis->u32CpuEvent;
            break;

        case SYSTEM_INFO_INDEX_SERIAL0_IOBASE:
            *pu32 = pThis->uSerial0IoPortBase;
            break;

        case SYSTEM_INFO_INDEX_SERIAL0_IRQ:
            *pu32 = pThis->uSerial0Irq;
            break;

        case SYSTEM_INFO_INDEX_SERIAL1_IOBASE:
            *pu32 = pThis->uSerial1IoPortBase;
            break;

        case SYSTEM_INFO_INDEX_SERIAL1_IRQ:
            *pu32 = pThis->uSerial1Irq;
            break;

        case SYSTEM_INFO_INDEX_END:
            /** @todo why isn't this setting any output value?  */
            break;

        /* Solaris 9 tries to read from this index */
        case SYSTEM_INFO_INDEX_INVALID:
            *pu32 = 0;
            break;

        default:
            *pu32 = UINT32_MAX;
            rc = PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u idx=%u\n", cb, Port, pThis->uBatteryIndex);
            break;
    }

    DEVACPI_UNLOCK(pThis);
    Log(("acpiSysInfoDataRead: idx=%d val=%#x (%d) rc=%Rrc\n", uSystemInfoIndex, *pu32, *pu32, rc));
    return rc;
}

/**
 * @callback_method_impl{FNIOMIOPORTOUT, System info data}
 */
PDMBOTHCBDECL(int) acpiSysInfoDataWrite(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t u32, unsigned cb)
{
    ACPIState *pThis = (ACPIState *)pvUser;
    if (cb != 4)
        return PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u u32=%#x idx=%u\n", cb, Port, u32, pThis->uSystemInfoIndex);

    DEVACPI_LOCK_R3(pThis);
    Log(("addr=%#x cb=%d u32=%#x si=%#x\n", Port, cb, u32, pThis->uSystemInfoIndex));

    int rc = VINF_SUCCESS;
    switch (pThis->uSystemInfoIndex)
    {
        case SYSTEM_INFO_INDEX_INVALID:
            AssertMsg(u32 == 0xbadc0de, ("u32=%u\n", u32));
            pThis->u8IndexShift = 0;
            break;

        case SYSTEM_INFO_INDEX_VALID:
            AssertMsg(u32 == 0xbadc0de, ("u32=%u\n", u32));
            pThis->u8IndexShift = 2;
            break;

        case SYSTEM_INFO_INDEX_CPU_LOCK_CHECK:
            pThis->idCpuLockCheck = u32;
            break;

        case SYSTEM_INFO_INDEX_CPU_LOCKED:
            if (u32 < pThis->cCpus)
                VMCPUSET_DEL(&pThis->CpuSetLocked, u32); /* Unlock the CPU */
            else
                LogRel(("ACPI: CPU %u does not exist\n", u32));
            break;

        default:
            rc = PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u u32=%#x idx=%u\n", cb, Port, u32, pThis->uSystemInfoIndex);
            break;
    }

    DEVACPI_UNLOCK(pThis);
    return rc;
}

/**
 * @callback_method_impl{FNIOMIOPORTIN, PM1a Enable}
 */
PDMBOTHCBDECL(int) acpiPm1aEnRead(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t *pu32, unsigned cb)
{
    NOREF(pDevIns); NOREF(Port);
    if (cb != 2)
        return VERR_IOM_IOPORT_UNUSED;

    ACPIState *pThis = (ACPIState *)pvUser;
    DEVACPI_LOCK_R3(pThis);

    *pu32 = pThis->pm1a_en;

    DEVACPI_UNLOCK(pThis);
    Log(("acpiPm1aEnRead -> %#x\n", *pu32));
    return VINF_SUCCESS;
}

/**
 * @callback_method_impl{FNIOMIOPORTOUT, PM1a Enable}
 */
PDMBOTHCBDECL(int) acpiPM1aEnWrite(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t u32, unsigned cb)
{
    if (cb != 2 && cb != 4)
        return PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u u32=%#x\n", cb, Port, u32);

    ACPIState *pThis = (ACPIState *)pvUser;
    DEVACPI_LOCK_R3(pThis);

    Log(("acpiPM1aEnWrite: %#x (%#x)\n", u32, u32 & ~(RSR_EN | IGN_EN) & 0xffff));
    u32 &= ~(RSR_EN | IGN_EN);
    u32 &= 0xffff;
    update_pm1a(pThis, pThis->pm1a_sts, u32);

    DEVACPI_UNLOCK(pThis);
    return VINF_SUCCESS;
}

/**
 * @callback_method_impl{FNIOMIOPORTIN, PM1a Status}
 */
PDMBOTHCBDECL(int) acpiPm1aStsRead(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t *pu32, unsigned cb)
{
    if (cb != 2)
    {
        int rc = PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u\n", cb, Port);
        return rc == VINF_SUCCESS ? VERR_IOM_IOPORT_UNUSED : rc;
    }

    ACPIState *pThis = (ACPIState *)pvUser;
    DEVACPI_LOCK_R3(pThis);

    *pu32 = pThis->pm1a_sts;

    DEVACPI_UNLOCK(pThis);
    Log(("acpiPm1aStsRead: %#x\n", *pu32));
    return VINF_SUCCESS;
}

/**
 * @callback_method_impl{FNIOMIOPORTOUT, PM1a Status}
 */
PDMBOTHCBDECL(int) acpiPM1aStsWrite(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t u32, unsigned cb)
{
    if (cb != 2 && cb != 4)
        return PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u u32=%#x\n", cb, Port, u32);

    ACPIState *pThis = (ACPIState *)pvUser;
    DEVACPI_LOCK_R3(pThis);

    Log(("acpiPM1aStsWrite: %#x (%#x)\n", u32, u32 & ~(RSR_STS | IGN_STS) & 0xffff));
    u32 &= 0xffff;
    if (u32 & PWRBTN_STS)
        pThis->fPowerButtonHandled = true; /* Remember that the guest handled the last power button event */
    u32 = pThis->pm1a_sts & ~(u32 & ~(RSR_STS | IGN_STS));
    update_pm1a(pThis, u32, pThis->pm1a_en);

    DEVACPI_UNLOCK(pThis);
    return VINF_SUCCESS;
}

/**
 * @callback_method_impl{FNIOMIOPORTIN, PM1a Control}
 */
PDMBOTHCBDECL(int) acpiPm1aCtlRead(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t *pu32, unsigned cb)
{
    if (cb != 2)
    {
        int rc = PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u\n", cb, Port);
        return rc == VINF_SUCCESS ? VERR_IOM_IOPORT_UNUSED : rc;
    }

    ACPIState *pThis = (ACPIState *)pvUser;
    DEVACPI_LOCK_R3(pThis);

    *pu32 = pThis->pm1a_ctl;

    DEVACPI_UNLOCK(pThis);
    Log(("acpiPm1aCtlRead: %#x\n", *pu32));
    return VINF_SUCCESS;
}

/**
 * @callback_method_impl{FNIOMIOPORTOUT, PM1a Control}
 */
PDMBOTHCBDECL(int) acpiPM1aCtlWrite(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t u32, unsigned cb)
{
    if (cb != 2 && cb != 4)
        return PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u u32=%#x\n", cb, Port, u32);

    ACPIState *pThis = (ACPIState *)pvUser;
    DEVACPI_LOCK_R3(pThis);

    Log(("acpiPM1aCtlWrite: %#x (%#x)\n", u32, u32 & ~(RSR_CNT | IGN_CNT) & 0xffff));
    u32 &= 0xffff;
    pThis->pm1a_ctl = u32 & ~(RSR_CNT | IGN_CNT);

    int rc = VINF_SUCCESS;
    uint32_t const uSleepState = (pThis->pm1a_ctl >> SLP_TYPx_SHIFT) & SLP_TYPx_MASK;
    if (uSleepState != pThis->uSleepState)
    {
        pThis->uSleepState = uSleepState;
        switch (uSleepState)
        {
            case 0x00:                  /* S0 */
                break;

            case 0x01:                  /* S1 */
                if (pThis->fS1Enabled)
                {
                    LogRel(("Entering S1 power state (powered-on suspend)\n"));
                    rc = acpiSleep(pThis);
                    break;
                }
                LogRel(("Ignoring guest attempt to enter S1 power state (powered-on suspend)!\n"));
                /* fall thru */

            case 0x04:                  /* S4 */
                if (pThis->fS4Enabled)
                {
                    LogRel(("Entering S4 power state (suspend to disk)\n"));
                    rc = acpiPowerOff(pThis);/* Same behavior as S5 */
                    break;
                }
                LogRel(("Ignoring guest attempt to enter S4 power state (suspend to disk)!\n"));
                /* fall thru */

            case 0x05:                  /* S5 */
                LogRel(("Entering S5 power state (power down)\n"));
                rc = acpiPowerOff(pThis);
                break;

            default:
                rc = PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "Unknown sleep state %#x (u32=%#x)\n", uSleepState, u32);
                break;
        }
    }

    DEVACPI_UNLOCK(pThis);
    Log(("acpiPM1aCtlWrite: rc=%Rrc\n", rc));
    return rc;
}

#endif /* IN_RING3 */

/**
 * @callback_method_impl{FNIOMIOPORTIN, PMTMR}
 *
 * @remarks Only I/O port currently implemented in all contexts.
 */
PDMBOTHCBDECL(int) acpiPMTmrRead(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t *pu32, unsigned cb)
{
    if (cb != 4)
        return VERR_IOM_IOPORT_UNUSED;

    ACPIState *pThis = PDMINS_2_DATA(pDevIns, ACPIState *);

    /*
     * We use the clock lock to serialize access to u64PmTimerInitial and to
     * make sure we get a reliable time from the clock.
     */
    int rc = TMTimerLock(pThis->CTX_SUFF(pPmTimer), VINF_IOM_R3_IOPORT_READ);
    if (rc == VINF_SUCCESS)
    {
        uint64_t const u64PmTimerInitial = pThis->u64PmTimerInitial;
        uint64_t u64Now = TMTimerGet(pThis->CTX_SUFF(pPmTimer));
        TMTimerUnlock(pThis->CTX_SUFF(pPmTimer));

        /*
         * Calculate the return value.
         */
        DBGFTRACE_PDM_U64_TAG(pDevIns, u64Now, "acpi");
        uint64_t u64Elapsed = u64Now - u64PmTimerInitial;
        *pu32 = ASMMultU64ByU32DivByU32(u64Elapsed, PM_TMR_FREQ, TMTimerGetFreq(pThis->CTX_SUFF(pPmTimer)));
        Log(("acpi: acpiPMTmrRead -> %#x\n", *pu32));
    }

    NOREF(pvUser); NOREF(Port);
    return rc;
}

#ifdef IN_RING3

/**
 * @callback_method_impl{FNIOMIOPORTIN, GPE0 Status}
 */
PDMBOTHCBDECL(int) acpiGpe0StsRead(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t *pu32, unsigned cb)
{
    if (cb != 1)
    {
        int rc = PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u\n", cb, Port);
        return rc == VINF_SUCCESS ? VERR_IOM_IOPORT_UNUSED : rc;
    }

    ACPIState *pThis = (ACPIState *)pvUser;
    DEVACPI_LOCK_R3(pThis);

    *pu32 = pThis->gpe0_sts & 0xff;

    DEVACPI_UNLOCK(pThis);
    Log(("acpiGpe0StsRead: %#x\n", *pu32));
    return VINF_SUCCESS;
}

/**
 * @callback_method_impl{FNIOMIOPORTOUT, GPE0 Status}
 */
PDMBOTHCBDECL(int) acpiGpe0StsWrite(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t u32, unsigned cb)
{
    if (cb != 1)
        return PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u u32=%#x\n", cb, Port, u32);

    ACPIState *pThis = (ACPIState *)pvUser;
    DEVACPI_LOCK_R3(pThis);

    Log(("acpiGpe0StsWrite: %#x (%#x)\n", u32, pThis->gpe0_sts & ~u32));
    u32 = pThis->gpe0_sts & ~u32;
    update_gpe0(pThis, u32, pThis->gpe0_en);

    DEVACPI_UNLOCK(pThis);
    return VINF_SUCCESS;
}

/**
 * @callback_method_impl{FNIOMIOPORTIN, GPE0 Enable}
 */
PDMBOTHCBDECL(int) acpiGpe0EnRead(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t *pu32, unsigned cb)
{
    if (cb != 1)
    {
        int rc = PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u\n", cb, Port);
        return rc == VINF_SUCCESS ? VERR_IOM_IOPORT_UNUSED : rc;
    }

    ACPIState *pThis = (ACPIState *)pvUser;
    DEVACPI_LOCK_R3(pThis);

    *pu32 = pThis->gpe0_en & 0xff;

    DEVACPI_UNLOCK(pThis);
    Log(("acpiGpe0EnRead: %#x\n", *pu32));
    return VINF_SUCCESS;
}

/**
 * @callback_method_impl{FNIOMIOPORTOUT, GPE0 Enable}
 */
PDMBOTHCBDECL(int) acpiGpe0EnWrite(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t u32, unsigned cb)
{
    if (cb != 1)
        return PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u u32=%#x\n", cb, Port, u32);

    ACPIState *pThis = (ACPIState *)pvUser;
    DEVACPI_LOCK_R3(pThis);

    Log(("acpiGpe0EnWrite: %#x\n", u32));
    update_gpe0(pThis, pThis->gpe0_sts, u32);

    DEVACPI_UNLOCK(pThis);
    return VINF_SUCCESS;
}

/**
 * @callback_method_impl{FNIOMIOPORTOUT, SMI_CMD}
 */
PDMBOTHCBDECL(int) acpiSmiWrite(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t u32, unsigned cb)
{
    Log(("acpiSmiWrite %#x\n", u32));
    if (cb != 1)
        return PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u u32=%#x\n", cb, Port, u32);

    ACPIState *pThis = (ACPIState *)pvUser;
    DEVACPI_LOCK_R3(pThis);

    if (u32 == ACPI_ENABLE)
        pThis->pm1a_ctl |= SCI_EN;
    else if (u32 == ACPI_DISABLE)
        pThis->pm1a_ctl &= ~SCI_EN;
    else
        Log(("acpiSmiWrite: %#x <- unknown value\n", u32));

    DEVACPI_UNLOCK(pThis);
    return VINF_SUCCESS;
}

/**
 * @{FNIOMIOPORTOUT, ACPI_RESET_BLK}
 */
PDMBOTHCBDECL(int) acpiResetWrite(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t u32, unsigned cb)
{
    Log(("acpiResetWrite: %#x\n", u32));
    NOREF(pvUser);
    if (cb != 1)
        return PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u u32=%#x\n", cb, Port, u32);

    /* No state locking required. */
    int rc = VINF_SUCCESS;
    if (u32 == ACPI_RESET_REG_VAL)
    {
        LogRel(("Reset initiated by ACPI\n"));
        rc = PDMDevHlpVMReset(pDevIns);
    }
    else
        Log(("acpiResetWrite: %#x <- unknown value\n", u32));

    return rc;
}

# ifdef DEBUG_ACPI

/**
 * @callback_method_impl{FNIOMIOPORTOUT, Debug hex value logger}
 */
PDMBOTHCBDECL(int) acpiDhexWrite(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t u32, unsigned cb)
{
    NOREF(pvUser);
    switch (cb)
    {
        case 1:
            Log(("%#x\n", u32 & 0xff));
            break;
        case 2:
            Log(("%#6x\n", u32 & 0xffff));
        case 4:
            Log(("%#10x\n", u32));
            break;
        default:
            return PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u u32=%#x\n", cb, Port, u32);
    }
    return VINF_SUCCESS;
}

/**
 * @callback_method_impl{FNIOMIOPORTOUT, Debug char logger}
 */
PDMBOTHCBDECL(int) acpiDchrWrite(PPDMDEVINS pDevIns, void *pvUser, RTIOPORT Port, uint32_t u32, unsigned cb)
{
    NOREF(pvUser);
    switch (cb)
    {
        case 1:
            Log(("%c", u32 & 0xff));
            break;
        default:
            return PDMDevHlpDBGFStop(pDevIns, RT_SRC_POS, "cb=%d Port=%u u32=%#x\n", cb, Port, u32);
    }
    return VINF_SUCCESS;
}

# endif /* DEBUG_ACPI */

/**
 * Used to calculate the value of a PM I/O port.
 *
 * @returns The actual I/O port value.
 * @param   pThis               The ACPI instance.
 * @param   offset              The offset into the I/O space, or -1 if invalid.
 */
static RTIOPORT acpiCalcPmPort(ACPIState *pThis, int32_t offset)
{
    Assert(pThis->uPmIoPortBase != 0);

    if (offset == -1)
        return 0;

    return (RTIOPORT)(pThis->uPmIoPortBase + offset);
}

/**
 * Called by acpiLoadState and acpiUpdatePmHandlers to register the PM1a, PM
 * timer and GPE0 I/O ports.
 *
 * @returns VBox status code.
 * @param   pThis           The ACPI instance.
 */
static int acpiRegisterPmHandlers(ACPIState *pThis)
{
    int   rc = VINF_SUCCESS;

#define R(offset, cnt, writer, reader, description) \
    do { \
        rc = PDMDevHlpIOPortRegister(pThis->pDevIns, acpiCalcPmPort(pThis, offset), cnt, pThis, writer, reader, \
                                      NULL, NULL, description); \
        if (RT_FAILURE(rc)) \
            return rc; \
    } while (0)
#define L       (GPE0_BLK_LEN / 2)

    R(PM1a_EVT_OFFSET+2, 1, acpiPM1aEnWrite,       acpiPm1aEnRead,      "ACPI PM1a Enable");
    R(PM1a_EVT_OFFSET,   1, acpiPM1aStsWrite,      acpiPm1aStsRead,     "ACPI PM1a Status");
    R(PM1a_CTL_OFFSET,   1, acpiPM1aCtlWrite,      acpiPm1aCtlRead,     "ACPI PM1a Control");
    R(PM_TMR_OFFSET,     1, NULL,                  acpiPMTmrRead,       "ACPI PM Timer");
    R(GPE0_OFFSET + L,   L, acpiGpe0EnWrite,       acpiGpe0EnRead,      "ACPI GPE0 Enable");
    R(GPE0_OFFSET,       L, acpiGpe0StsWrite,      acpiGpe0StsRead,     "ACPI GPE0 Status");
#undef L
#undef R

    /* register RC stuff */
    if (pThis->fGCEnabled)
    {
        rc = PDMDevHlpIOPortRegisterRC(pThis->pDevIns, acpiCalcPmPort(pThis, PM_TMR_OFFSET),
                                       1, 0, NULL, "acpiPMTmrRead",
                                       NULL, NULL, "ACPI PM Timer");
        AssertRCReturn(rc, rc);
    }

    /* register R0 stuff */
    if (pThis->fR0Enabled)
    {
        rc = PDMDevHlpIOPortRegisterR0(pThis->pDevIns, acpiCalcPmPort(pThis, PM_TMR_OFFSET),
                                       1, 0, NULL, "acpiPMTmrRead",
                                       NULL, NULL, "ACPI PM Timer");
        AssertRCReturn(rc, rc);
    }

    return rc;
}

/**
 * Called by acpiLoadState and acpiUpdatePmHandlers to unregister the PM1a, PM
 * timer and GPE0 I/O ports.
 *
 * @returns VBox status code.
 * @param   pThis           The ACPI instance.
 */
static int acpiUnregisterPmHandlers(ACPIState *pThis)
{
#define U(offset, cnt) \
    do { \
        int rc = PDMDevHlpIOPortDeregister(pThis->pDevIns, acpiCalcPmPort(pThis, offset), cnt); \
        AssertRCReturn(rc, rc); \
    } while (0)
#define L       (GPE0_BLK_LEN / 2)

    U(PM1a_EVT_OFFSET+2, 1);
    U(PM1a_EVT_OFFSET,   1);
    U(PM1a_CTL_OFFSET,   1);
    U(PM_TMR_OFFSET,     1);
    U(GPE0_OFFSET + L,   L);
    U(GPE0_OFFSET,       L);
#undef L
#undef U

    return VINF_SUCCESS;
}

/**
 * Called by acpiPciConfigWrite and acpiReset to change the location of the
 * PM1a, PM timer and GPE0 ports.
 *
 * @returns VBox status code.
 *
 * @param   pThis           The ACPI instance.
 * @param   NewIoPortBase   The new base address of the I/O ports.
 */
static int acpiUpdatePmHandlers(ACPIState *pThis, RTIOPORT NewIoPortBase)
{
    Log(("acpi: rebasing PM 0x%x -> 0x%x\n", pThis->uPmIoPortBase, NewIoPortBase));
    if (NewIoPortBase != pThis->uPmIoPortBase)
    {
        int rc = acpiUnregisterPmHandlers(pThis);
        if (RT_FAILURE(rc))
            return rc;

        pThis->uPmIoPortBase = NewIoPortBase;

        rc = acpiRegisterPmHandlers(pThis);
        if (RT_FAILURE(rc))
            return rc;

        /* We have to update FADT table acccording to the new base */
        rc = acpiPlantTables(pThis);
        AssertRC(rc);
        if (RT_FAILURE(rc))
            return rc;
    }

    return VINF_SUCCESS;
}


/**
 * Saved state structure description, version 4.
 */
static const SSMFIELD g_AcpiSavedStateFields4[] =
{
    SSMFIELD_ENTRY(ACPIState, pm1a_en),
    SSMFIELD_ENTRY(ACPIState, pm1a_sts),
    SSMFIELD_ENTRY(ACPIState, pm1a_ctl),
    SSMFIELD_ENTRY(ACPIState, u64PmTimerInitial),
    SSMFIELD_ENTRY(ACPIState, gpe0_en),
    SSMFIELD_ENTRY(ACPIState, gpe0_sts),
    SSMFIELD_ENTRY(ACPIState, uBatteryIndex),
    SSMFIELD_ENTRY(ACPIState, uSystemInfoIndex),
    SSMFIELD_ENTRY(ACPIState, u64RamSize),
    SSMFIELD_ENTRY(ACPIState, u8IndexShift),
    SSMFIELD_ENTRY(ACPIState, u8UseIOApic),
    SSMFIELD_ENTRY(ACPIState, uSleepState),
    SSMFIELD_ENTRY_TERM()
};

/**
 * Saved state structure description, version 5.
 */
static const SSMFIELD g_AcpiSavedStateFields5[] =
{
    SSMFIELD_ENTRY(ACPIState, pm1a_en),
    SSMFIELD_ENTRY(ACPIState, pm1a_sts),
    SSMFIELD_ENTRY(ACPIState, pm1a_ctl),
    SSMFIELD_ENTRY(ACPIState, u64PmTimerInitial),
    SSMFIELD_ENTRY(ACPIState, gpe0_en),
    SSMFIELD_ENTRY(ACPIState, gpe0_sts),
    SSMFIELD_ENTRY(ACPIState, uBatteryIndex),
    SSMFIELD_ENTRY(ACPIState, uSystemInfoIndex),
    SSMFIELD_ENTRY(ACPIState, uSleepState),
    SSMFIELD_ENTRY(ACPIState, u8IndexShift),
    SSMFIELD_ENTRY(ACPIState, uPmIoPortBase),
    SSMFIELD_ENTRY_TERM()
};

/**
 * Saved state structure description, version 6.
 */
static const SSMFIELD g_AcpiSavedStateFields6[] =
{
    SSMFIELD_ENTRY(ACPIState, pm1a_en),
    SSMFIELD_ENTRY(ACPIState, pm1a_sts),
    SSMFIELD_ENTRY(ACPIState, pm1a_ctl),
    SSMFIELD_ENTRY(ACPIState, u64PmTimerInitial),
    SSMFIELD_ENTRY(ACPIState, gpe0_en),
    SSMFIELD_ENTRY(ACPIState, gpe0_sts),
    SSMFIELD_ENTRY(ACPIState, uBatteryIndex),
    SSMFIELD_ENTRY(ACPIState, uSystemInfoIndex),
    SSMFIELD_ENTRY(ACPIState, uSleepState),
    SSMFIELD_ENTRY(ACPIState, u8IndexShift),
    SSMFIELD_ENTRY(ACPIState, uPmIoPortBase),
    SSMFIELD_ENTRY(ACPIState, fSuspendToSavedState),
    SSMFIELD_ENTRY_TERM()
};


/**
 * @callback_method_impl{FNSSMDEVSAVEEXEC}
 */
static DECLCALLBACK(int) acpiSaveState(PPDMDEVINS pDevIns, PSSMHANDLE pSSMHandle)
{
    ACPIState *pThis = PDMINS_2_DATA(pDevIns, ACPIState *);
    return SSMR3PutStruct(pSSMHandle, pThis, &g_AcpiSavedStateFields6[0]);
}

/**
 * @callback_method_impl{FNSSMDEVLOADEXEC}
 */
static DECLCALLBACK(int) acpiLoadState(PPDMDEVINS pDevIns, PSSMHANDLE pSSMHandle,
                                       uint32_t uVersion, uint32_t uPass)
{
    ACPIState *pThis = PDMINS_2_DATA(pDevIns, ACPIState *);
    Assert(uPass == SSM_PASS_FINAL); NOREF(uPass);

    /*
     * Unregister PM handlers, will register with actual base after state
     * successfully loaded.
     */
    int rc = acpiUnregisterPmHandlers(pThis);
    if (RT_FAILURE(rc))
        return rc;

    switch (uVersion)
    {
        case 4:
            rc = SSMR3GetStruct(pSSMHandle, pThis, &g_AcpiSavedStateFields4[0]);
            break;
        case 5:
            rc = SSMR3GetStruct(pSSMHandle, pThis, &g_AcpiSavedStateFields5[0]);
            break;
        case 6:
            rc = SSMR3GetStruct(pSSMHandle, pThis, &g_AcpiSavedStateFields6[0]);
            break;
        default:
            rc = VERR_SSM_UNSUPPORTED_DATA_UNIT_VERSION;
            break;
    }
    if (RT_SUCCESS(rc))
    {
        rc = acpiRegisterPmHandlers(pThis);
        if (RT_FAILURE(rc))
            return rc;
        rc = acpiFetchBatteryStatus(pThis);
        if (RT_FAILURE(rc))
            return rc;
        rc = acpiFetchBatteryInfo(pThis);
        if (RT_FAILURE(rc))
            return rc;
        TMTimerLock(pThis->pPmTimerR3, VERR_IGNORED);
        acpiPmTimerReset(pThis, TMTimerGet(pThis->pPmTimerR3));
        TMTimerUnlock(pThis->pPmTimerR3);
    }
    return rc;
}

/**
 * @interface_method_impl{PDMIBASE,pfnQueryInterface}
 */
static DECLCALLBACK(void *) acpiQueryInterface(PPDMIBASE pInterface, const char *pszIID)
{
    ACPIState *pThis = RT_FROM_MEMBER(pInterface, ACPIState, IBase);
    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIBASE, &pThis->IBase);
    PDMIBASE_RETURN_INTERFACE(pszIID, PDMIACPIPORT, &pThis->IACPIPort);
    return NULL;
}

/**
 * Calculate the check sum for some ACPI data before planting it.
 *
 * All the bytes must add up to 0.
 *
 * @returns check sum.
 * @param   pvSrc       What to check sum.
 * @param   cbData      The amount of data to checksum.
 */
static uint8_t acpiChecksum(const void * const pvSrc, size_t cbData)
{
    uint8_t const *pbSrc = (uint8_t const *)pvSrc;
    uint8_t uSum = 0;
    for (size_t i = 0; i < cbData; ++i)
        uSum += pbSrc[i];
    return -uSum;
}

/**
 * Prepare a ACPI table header.
 */
static void acpiPrepareHeader(ACPIState *pThis, ACPITBLHEADER *header,
                              const char au8Signature[4],
                              uint32_t u32Length, uint8_t u8Revision)
{
    memcpy(header->au8Signature, au8Signature, 4);
    header->u32Length             = RT_H2LE_U32(u32Length);
    header->u8Revision            = u8Revision;
    memcpy(header->au8OemId, pThis->au8OemId, 6);
    memcpy(header->au8OemTabId, "VBOX", 4);
    memcpy(header->au8OemTabId+4, au8Signature, 4);
    header->u32OemRevision        = RT_H2LE_U32(1);
    memcpy(header->au8CreatorId, pThis->au8CreatorId, 4);
    header->u32CreatorRev         = pThis->u32CreatorRev;
}

/**
 * Initialize a generic address structure (ACPIGENADDR).
 */
static void acpiWriteGenericAddr(ACPIGENADDR *g, uint8_t u8AddressSpaceId,
                                 uint8_t u8RegisterBitWidth, uint8_t u8RegisterBitOffset,
                                 uint8_t u8AccessSize, uint64_t u64Address)
{
    g->u8AddressSpaceId    = u8AddressSpaceId;
    g->u8RegisterBitWidth  = u8RegisterBitWidth;
    g->u8RegisterBitOffset = u8RegisterBitOffset;
    g->u8AccessSize        = u8AccessSize;
    g->u64Address          = RT_H2LE_U64(u64Address);
}

/**
 * Wrapper around PDMDevHlpPhysWrite used when planting ACPI tables.
 */
static void acpiPhyscpy(ACPIState *pThis, RTGCPHYS32 dst, const void * const src, size_t size)
{
    PDMDevHlpPhysWrite(pThis->pDevIns, dst, src, size);
}

/**
 * Plant the Differentiated System Description Table (DSDT).
 */
static void acpiSetupDSDT(ACPIState *pThis, RTGCPHYS32 addr,
                            void* pPtr, size_t uDsdtLen)
{
    acpiPhyscpy(pThis, addr, pPtr, uDsdtLen);
}

/**
 * Plan the Secondary System Description Table (SSDT).
 */
static void acpiSetupSSDT(ACPIState *pThis, RTGCPHYS32 addr,
                            void* pPtr, size_t uSsdtLen)
{
    acpiPhyscpy(pThis, addr, pPtr, uSsdtLen);
}

/**
 * Plant the Firmware ACPI Control Structure (FACS).
 */
static void acpiSetupFACS(ACPIState *pThis, RTGCPHYS32 addr)
{
    ACPITBLFACS facs;

    memset(&facs, 0, sizeof(facs));
    memcpy(facs.au8Signature, "FACS", 4);
    facs.u32Length            = RT_H2LE_U32(sizeof(ACPITBLFACS));
    facs.u32HWSignature       = RT_H2LE_U32(0);
    facs.u32FWVector          = RT_H2LE_U32(0);
    facs.u32GlobalLock        = RT_H2LE_U32(0);
    facs.u32Flags             = RT_H2LE_U32(0);
    facs.u64X_FWVector        = RT_H2LE_U64(0);
    facs.u8Version            = 1;

    acpiPhyscpy(pThis, addr, (const uint8_t *)&facs, sizeof(facs));
}

/**
 * Plant the Fixed ACPI Description Table (FADT aka FACP).
 */
static void acpiSetupFADT(ACPIState *pThis, RTGCPHYS32 GCPhysAcpi1, RTGCPHYS32 GCPhysAcpi2, RTGCPHYS32 GCPhysFacs, RTGCPHYS GCPhysDsdt)
{
    ACPITBLFADT fadt;

    /* First the ACPI version 2+ version of the structure. */
    memset(&fadt, 0, sizeof(fadt));
    acpiPrepareHeader(pThis, &fadt.header, "FACP", sizeof(fadt), 4);
    fadt.u32FACS              = RT_H2LE_U32(GCPhysFacs);
    fadt.u32DSDT              = RT_H2LE_U32(GCPhysDsdt);
    fadt.u8IntModel           = 0;  /* dropped from the ACPI 2.0 spec. */
    fadt.u8PreferredPMProfile = 0;  /* unspecified */
    fadt.u16SCIInt            = RT_H2LE_U16(SCI_INT);
    fadt.u32SMICmd            = RT_H2LE_U32(SMI_CMD);
    fadt.u8AcpiEnable         = ACPI_ENABLE;
    fadt.u8AcpiDisable        = ACPI_DISABLE;
    fadt.u8S4BIOSReq          = 0;
    fadt.u8PStateCnt          = 0;
    fadt.u32PM1aEVTBLK        = RT_H2LE_U32(acpiCalcPmPort(pThis, PM1a_EVT_OFFSET));
    fadt.u32PM1bEVTBLK        = RT_H2LE_U32(acpiCalcPmPort(pThis, PM1b_EVT_OFFSET));
    fadt.u32PM1aCTLBLK        = RT_H2LE_U32(acpiCalcPmPort(pThis, PM1a_CTL_OFFSET));
    fadt.u32PM1bCTLBLK        = RT_H2LE_U32(acpiCalcPmPort(pThis, PM1b_CTL_OFFSET));
    fadt.u32PM2CTLBLK         = RT_H2LE_U32(acpiCalcPmPort(pThis, PM2_CTL_OFFSET));
    fadt.u32PMTMRBLK          = RT_H2LE_U32(acpiCalcPmPort(pThis, PM_TMR_OFFSET));
    fadt.u32GPE0BLK           = RT_H2LE_U32(acpiCalcPmPort(pThis, GPE0_OFFSET));
    fadt.u32GPE1BLK           = RT_H2LE_U32(acpiCalcPmPort(pThis, GPE1_OFFSET));
    fadt.u8PM1EVTLEN          = 4;
    fadt.u8PM1CTLLEN          = 2;
    fadt.u8PM2CTLLEN          = 0;
    fadt.u8PMTMLEN            = 4;
    fadt.u8GPE0BLKLEN         = GPE0_BLK_LEN;
    fadt.u8GPE1BLKLEN         = GPE1_BLK_LEN;
    fadt.u8GPE1BASE           = GPE1_BASE;
    fadt.u8CSTCNT             = 0;
    fadt.u16PLVL2LAT          = RT_H2LE_U16(P_LVL2_LAT);
    fadt.u16PLVL3LAT          = RT_H2LE_U16(P_LVL3_LAT);
    fadt.u16FlushSize         = RT_H2LE_U16(FLUSH_SIZE);
    fadt.u16FlushStride       = RT_H2LE_U16(FLUSH_STRIDE);
    fadt.u8DutyOffset         = 0;
    fadt.u8DutyWidth          = 0;
    fadt.u8DayAlarm           = 0;
    fadt.u8MonAlarm           = 0;
    fadt.u8Century            = 0;
    fadt.u16IAPCBOOTARCH      = RT_H2LE_U16(IAPC_BOOT_ARCH_LEGACY_DEV | IAPC_BOOT_ARCH_8042);
    /** @note WBINVD is required for ACPI versions newer than 1.0 */
    fadt.u32Flags             = RT_H2LE_U32(  FADT_FL_WBINVD
                                            | FADT_FL_FIX_RTC
                                            | FADT_FL_TMR_VAL_EXT
                                            | FADT_FL_RESET_REG_SUP);

    /* We have to force physical APIC mode or Linux can't use more than 8 CPUs */
    if (pThis->fCpuHotPlug)
        fadt.u32Flags |= RT_H2LE_U32(FADT_FL_FORCE_APIC_PHYS_DEST_MODE);

    acpiWriteGenericAddr(&fadt.ResetReg,     1,  8, 0, 1, ACPI_RESET_BLK);
    fadt.u8ResetVal           = ACPI_RESET_REG_VAL;
    fadt.u64XFACS             = RT_H2LE_U64((uint64_t)GCPhysFacs);
    fadt.u64XDSDT             = RT_H2LE_U64((uint64_t)GCPhysDsdt);
    acpiWriteGenericAddr(&fadt.X_PM1aEVTBLK, 1, 32, 0, 2, acpiCalcPmPort(pThis, PM1a_EVT_OFFSET));
    acpiWriteGenericAddr(&fadt.X_PM1bEVTBLK, 0,  0, 0, 0, acpiCalcPmPort(pThis, PM1b_EVT_OFFSET));
    acpiWriteGenericAddr(&fadt.X_PM1aCTLBLK, 1, 16, 0, 2, acpiCalcPmPort(pThis, PM1a_CTL_OFFSET));
    acpiWriteGenericAddr(&fadt.X_PM1bCTLBLK, 0,  0, 0, 0, acpiCalcPmPort(pThis, PM1b_CTL_OFFSET));
    acpiWriteGenericAddr(&fadt.X_PM2CTLBLK,  0,  0, 0, 0, acpiCalcPmPort(pThis, PM2_CTL_OFFSET));
    acpiWriteGenericAddr(&fadt.X_PMTMRBLK,   1, 32, 0, 3, acpiCalcPmPort(pThis, PM_TMR_OFFSET));
    acpiWriteGenericAddr(&fadt.X_GPE0BLK,    1, 16, 0, 1, acpiCalcPmPort(pThis, GPE0_OFFSET));
    acpiWriteGenericAddr(&fadt.X_GPE1BLK,    0,  0, 0, 0, acpiCalcPmPort(pThis, GPE1_OFFSET));
    fadt.header.u8Checksum    = acpiChecksum(&fadt, sizeof(fadt));
    acpiPhyscpy(pThis, GCPhysAcpi2, &fadt, sizeof(fadt));

    /* Now the ACPI 1.0 version. */
    fadt.header.u32Length     = ACPITBLFADT_VERSION1_SIZE;
    fadt.u8IntModel           = INT_MODEL_DUAL_PIC;
    fadt.header.u8Checksum    = 0;  /* Must be zeroed before recalculating checksum! */
    fadt.header.u8Checksum    = acpiChecksum(&fadt, ACPITBLFADT_VERSION1_SIZE);
    acpiPhyscpy(pThis, GCPhysAcpi1, &fadt, ACPITBLFADT_VERSION1_SIZE);
}

/**
 * Plant the root System Description Table.
 *
 * The RSDT and XSDT tables are basically identical. The only difference is 32
 * vs 64 bits addresses for description headers. RSDT is for ACPI 1.0. XSDT for
 * ACPI 2.0 and up.
 */
static int acpiSetupRSDT(ACPIState *pThis, RTGCPHYS32 addr, unsigned int nb_entries, uint32_t *addrs)
{
    ACPITBLRSDT *rsdt;
    const size_t size = sizeof(ACPITBLHEADER) + nb_entries * sizeof(rsdt->u32Entry[0]);

    rsdt = (ACPITBLRSDT*)RTMemAllocZ(size);
    if (!rsdt)
        return PDMDEV_SET_ERROR(pThis->pDevIns, VERR_NO_TMP_MEMORY, N_("Cannot allocate RSDT"));

    acpiPrepareHeader(pThis, &rsdt->header, "RSDT", (uint32_t)size, 1);
    for (unsigned int i = 0; i < nb_entries; ++i)
    {
        rsdt->u32Entry[i] = RT_H2LE_U32(addrs[i]);
        Log(("Setup RSDT: [%d] = %x\n", i, rsdt->u32Entry[i]));
    }
    rsdt->header.u8Checksum = acpiChecksum(rsdt, size);
    acpiPhyscpy(pThis, addr, rsdt, size);
    RTMemFree(rsdt);
    return VINF_SUCCESS;
}

/**
 * Plant the Extended System Description Table.
 */
static int acpiSetupXSDT(ACPIState *pThis, RTGCPHYS32 addr, unsigned int nb_entries, uint32_t *addrs)
{
    ACPITBLXSDT *xsdt;
    const size_t size = sizeof(ACPITBLHEADER) + nb_entries * sizeof(xsdt->u64Entry[0]);

    xsdt = (ACPITBLXSDT*)RTMemAllocZ(size);
    if (!xsdt)
        return VERR_NO_TMP_MEMORY;

    acpiPrepareHeader(pThis, &xsdt->header, "XSDT", (uint32_t)size, 1 /* according to ACPI 3.0 specs */);

    if (pThis->fUseCust)
        memcpy(xsdt->header.au8OemTabId, pThis->au8OemTabId, 8);

    for (unsigned int i = 0; i < nb_entries; ++i)
    {
        xsdt->u64Entry[i] = RT_H2LE_U64((uint64_t)addrs[i]);
        Log(("Setup XSDT: [%d] = %RX64\n", i, xsdt->u64Entry[i]));
    }
    xsdt->header.u8Checksum = acpiChecksum(xsdt, size);
    acpiPhyscpy(pThis, addr, xsdt, size);
    RTMemFree(xsdt);
    return VINF_SUCCESS;
}

/**
 * Plant the Root System Description Pointer (RSDP).
 */
static void acpiSetupRSDP(ACPIState *pThis, ACPITBLRSDP *rsdp, RTGCPHYS32 GCPhysRsdt, RTGCPHYS GCPhysXsdt)
{
    memset(rsdp, 0, sizeof(*rsdp));

    /* ACPI 1.0 part (RSDT */
    memcpy(rsdp->au8Signature, "RSD PTR ", 8);
    memcpy(rsdp->au8OemId, pThis->au8OemId, 6);
    rsdp->u8Revision    = ACPI_REVISION;
    rsdp->u32RSDT       = RT_H2LE_U32(GCPhysRsdt);
    rsdp->u8Checksum    = acpiChecksum(rsdp, RT_OFFSETOF(ACPITBLRSDP, u32Length));

    /* ACPI 2.0 part (XSDT) */
    rsdp->u32Length     = RT_H2LE_U32(sizeof(ACPITBLRSDP));
    rsdp->u64XSDT       = RT_H2LE_U64(GCPhysXsdt);
    rsdp->u8ExtChecksum = acpiChecksum(rsdp, sizeof(ACPITBLRSDP));
}

/**
 * Plant the Multiple APIC Description Table (MADT).
 *
 * @note    APIC without IO-APIC hangs Windows Vista therefore we setup both.
 *
 * @todo    All hardcoded, should set this up based on the actual VM config!!!!!
 */
static void acpiSetupMADT(ACPIState *pThis, RTGCPHYS32 addr)
{
    uint16_t cpus = pThis->cCpus;
    AcpiTableMADT madt(cpus, NUMBER_OF_IRQ_SOURCE_OVERRIDES);

    acpiPrepareHeader(pThis, madt.header_addr(), "APIC", madt.size(), 2);

    *madt.u32LAPIC_addr()          = RT_H2LE_U32(0xfee00000);
    *madt.u32Flags_addr()          = RT_H2LE_U32(PCAT_COMPAT);

    /* LAPICs records */
    ACPITBLLAPIC* lapic = madt.LApics_addr();
    for (uint16_t i = 0; i < cpus; i++)
    {
        lapic->u8Type      = 0;
        lapic->u8Length    = sizeof(ACPITBLLAPIC);
        lapic->u8ProcId    = i;
        /** Must match numbering convention in MPTABLES */
        lapic->u8ApicId    = i;
        lapic->u32Flags    = VMCPUSET_IS_PRESENT(&pThis->CpuSetAttached, i) ? RT_H2LE_U32(LAPIC_ENABLED) : 0;
        lapic++;
    }

    /* IO-APIC record */
    ACPITBLIOAPIC* ioapic = madt.IOApic_addr();
    ioapic->u8Type     = 1;
    ioapic->u8Length   = sizeof(ACPITBLIOAPIC);
    /** Must match MP tables ID */
    ioapic->u8IOApicId = cpus;
    ioapic->u8Reserved = 0;
    ioapic->u32Address = RT_H2LE_U32(0xfec00000);
    ioapic->u32GSIB    = RT_H2LE_U32(0);

    /* Interrupt Source Overrides */
    /* Flags:
        bits[3:2]:
          00 conforms to the bus
          01 edge-triggered
          10 reserved
          11 level-triggered
        bits[1:0]
          00 conforms to the bus
          01 active-high
          10 reserved
          11 active-low */
    /* If changing, also update PDMIsaSetIrq() and MPS */
    ACPITBLISO* isos = madt.ISO_addr();
    /* Timer interrupt rule IRQ0 to GSI2 */
    isos[0].u8Type     = 2;
    isos[0].u8Length   = sizeof(ACPITBLISO);
    isos[0].u8Bus      = 0; /* Must be 0 */
    isos[0].u8Source   = 0; /* IRQ0 */
    isos[0].u32GSI     = 2; /* connected to pin 2 */
    isos[0].u16Flags   = 0; /* conform to the bus */

    /* ACPI interrupt rule - IRQ9 to GSI9 */
    isos[1].u8Type     = 2;
    isos[1].u8Length   = sizeof(ACPITBLISO);
    isos[1].u8Bus      = 0; /* Must be 0 */
    isos[1].u8Source   = 9; /* IRQ9 */
    isos[1].u32GSI     = 9; /* connected to pin 9 */
    isos[1].u16Flags   = 0xd; /* active high, level triggered */
    Assert(NUMBER_OF_IRQ_SOURCE_OVERRIDES == 2);

    madt.header_addr()->u8Checksum = acpiChecksum(madt.data(), madt.size());
    acpiPhyscpy(pThis, addr, madt.data(), madt.size());
}

/**
 * Plant the High Performance Event Timer (HPET) descriptor.
 */
static void acpiSetupHPET(ACPIState *pThis, RTGCPHYS32 addr)
{
    ACPITBLHPET hpet;

    memset(&hpet, 0, sizeof(hpet));

    acpiPrepareHeader(pThis, &hpet.aHeader, "HPET", sizeof(hpet), 1);
    /* Keep base address consistent with appropriate DSDT entry  (vbox.dsl) */
    acpiWriteGenericAddr(&hpet.HpetAddr,
                         0  /* Memory address space */,
                         64 /* Register bit width */,
                         0  /* Bit offset */,
                         0, /* Register access size, is it correct? */
                         0xfed00000 /* Address */);

    hpet.u32Id        = 0x8086a201; /* must match what HPET ID returns, is it correct ? */
    hpet.u32Number    = 0;
    hpet.u32MinTick   = 4096;
    hpet.u8Attributes = 0;

    hpet.aHeader.u8Checksum = acpiChecksum(&hpet, sizeof(hpet));

    acpiPhyscpy(pThis, addr, (const uint8_t *)&hpet, sizeof(hpet));
}


/** Custom Description Table */
static void acpiSetupCUST(ACPIState *pThis, RTGCPHYS32 addr)
{
    ACPITBLCUST cust;

    /* First the ACPI version 1 version of the structure. */
    memset(&cust, 0, sizeof(cust));
    acpiPrepareHeader(pThis, &cust.header, "CUST", sizeof(cust), 1);

    memcpy(cust.header.au8OemTabId, pThis->au8OemTabId, 8);
    cust.header.u32OemRevision = RT_H2LE_U32(pThis->u32OemRevision);
    cust.header.u8Checksum = acpiChecksum((uint8_t *)&cust, sizeof(cust));

    acpiPhyscpy(pThis, addr, pThis->pu8CustBin, pThis->cbCustBin);
}

/**
 * Used by acpiPlantTables to plant a MMCONFIG PCI config space access (MCFG)
 * descriptor.
 *
 * @param   pThis       The ACPI instance.
 * @param   GCPhysDst   Where to plant it.
 */
static void acpiSetupMCFG(ACPIState *pThis, RTGCPHYS32 GCPhysDst)
{
    struct
    {
        ACPITBLMCFG       hdr;
        ACPITBLMCFGENTRY  entry;
    }       tbl;
    uint8_t u8StartBus = 0;
    uint8_t u8EndBus   = (pThis->u64PciConfigMMioLength >> 20) - 1;

    RT_ZERO(tbl);

    acpiPrepareHeader(pThis, &tbl.hdr.aHeader, "MCFG", sizeof(tbl), 1);
    tbl.entry.u64BaseAddress = pThis->u64PciConfigMMioAddress;
    tbl.entry.u8StartBus     = u8StartBus;
    tbl.entry.u8EndBus       = u8EndBus;
    // u16PciSegmentGroup must match _SEG in ACPI table

    tbl.hdr.aHeader.u8Checksum = acpiChecksum(&tbl, sizeof(tbl));

    acpiPhyscpy(pThis, GCPhysDst, (const uint8_t *)&tbl, sizeof(tbl));
}

/**
 * Used by acpiPlantTables and acpiConstruct.
 *
 * @returns Guest memory address.
 */
static uint32_t find_rsdp_space(void)
{
    return 0xe0000;
}

/**
 * Create the ACPI tables in guest memory.
 */
static int acpiPlantTables(ACPIState *pThis)
{
    int        rc;
    RTGCPHYS32 GCPhysCur, GCPhysRsdt, GCPhysXsdt, GCPhysFadtAcpi1, GCPhysFadtAcpi2, GCPhysFacs, GCPhysDsdt;
    RTGCPHYS32 GCPhysHpet = 0;
    RTGCPHYS32 GCPhysApic = 0;
    RTGCPHYS32 GCPhysSsdt = 0;
    RTGCPHYS32 GCPhysMcfg = 0;
    RTGCPHYS32 GCPhysCust = 0;
    uint32_t   addend = 0;
    RTGCPHYS32 aGCPhysRsdt[8];
    RTGCPHYS32 aGCPhysXsdt[8];
    uint32_t   cAddr;
    uint32_t   iMadt  = 0;
    uint32_t   iHpet  = 0;
    uint32_t   iSsdt  = 0;
    uint32_t   iMcfg  = 0;
    uint32_t   iCust  = 0;
    size_t     cbRsdt = sizeof(ACPITBLHEADER);
    size_t     cbXsdt = sizeof(ACPITBLHEADER);

    cAddr = 1;                  /* FADT */
    if (pThis->u8UseIOApic)
        iMadt = cAddr++;        /* MADT */

    if (pThis->fUseHpet)
        iHpet = cAddr++;        /* HPET */

    if (pThis->fUseMcfg)
        iMcfg = cAddr++;        /* MCFG */

    if (pThis->fUseCust)
        iCust = cAddr++;        /* CUST */

    iSsdt = cAddr++;            /* SSDT */

    Assert(cAddr < RT_ELEMENTS(aGCPhysRsdt));
    Assert(cAddr < RT_ELEMENTS(aGCPhysXsdt));

    cbRsdt += cAddr*sizeof(uint32_t);  /* each entry: 32 bits phys. address. */
    cbXsdt += cAddr*sizeof(uint64_t);  /* each entry: 64 bits phys. address. */

    rc = CFGMR3QueryU64(pThis->pDevIns->pCfg, "RamSize", &pThis->u64RamSize);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pThis->pDevIns, rc,
                                N_("Configuration error: Querying \"RamSize\" as integer failed"));

    uint32_t cbRamHole;
    rc = CFGMR3QueryU32Def(pThis->pDevIns->pCfg, "RamHoleSize", &cbRamHole, MM_RAM_HOLE_SIZE_DEFAULT);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pThis->pDevIns, rc,
                                N_("Configuration error: Querying \"RamHoleSize\" as integer failed"));

    /*
     * Calculate the sizes for the high and low regions.
     */
    const uint64_t offRamHole = _4G - cbRamHole;
    pThis->cbRamHigh = offRamHole < pThis->u64RamSize ? pThis->u64RamSize - offRamHole : 0;
    uint64_t cbRamLow = offRamHole < pThis->u64RamSize ? offRamHole : pThis->u64RamSize;
    if (cbRamLow > UINT32_C(0xffe00000)) /* See MEM3. */
    {
        /* Note: This is also enforced by DevPcBios.cpp. */
        LogRel(("DevACPI: Clipping cbRamLow=%#RX64 down to 0xffe00000.\n", cbRamLow));
        cbRamLow = UINT32_C(0xffe00000);
    }
    pThis->cbRamLow = (uint32_t)cbRamLow;

    GCPhysCur = 0;
    GCPhysRsdt = GCPhysCur;

    GCPhysCur = RT_ALIGN_32(GCPhysCur + cbRsdt, 16);
    GCPhysXsdt = GCPhysCur;

    GCPhysCur = RT_ALIGN_32(GCPhysCur + cbXsdt, 16);
    GCPhysFadtAcpi1 = GCPhysCur;

    GCPhysCur = RT_ALIGN_32(GCPhysCur + ACPITBLFADT_VERSION1_SIZE, 16);
    GCPhysFadtAcpi2 = GCPhysCur;

    GCPhysCur = RT_ALIGN_32(GCPhysCur + sizeof(ACPITBLFADT), 64);
    GCPhysFacs = GCPhysCur;

    GCPhysCur = RT_ALIGN_32(GCPhysCur + sizeof(ACPITBLFACS), 16);
    if (pThis->u8UseIOApic)
    {
        GCPhysApic = GCPhysCur;
        GCPhysCur = RT_ALIGN_32(GCPhysCur + AcpiTableMADT::sizeFor(pThis, NUMBER_OF_IRQ_SOURCE_OVERRIDES), 16);
    }
    if (pThis->fUseHpet)
    {
        GCPhysHpet = GCPhysCur;
        GCPhysCur = RT_ALIGN_32(GCPhysCur + sizeof(ACPITBLHPET), 16);
    }
    if (pThis->fUseMcfg)
    {
        GCPhysMcfg = GCPhysCur;
        /* Assume one entry */
        GCPhysCur = RT_ALIGN_32(GCPhysCur + sizeof(ACPITBLMCFG) + sizeof(ACPITBLMCFGENTRY), 16);
    }
    if (pThis->fUseCust)
    {
        GCPhysCust = GCPhysCur;
        GCPhysCur = RT_ALIGN_32(GCPhysCur + pThis->cbCustBin, 16);
    }

    void  *pvSsdtCode = NULL;
    size_t cbSsdtSize = 0;
    rc = acpiPrepareSsdt(pThis->pDevIns, &pvSsdtCode, &cbSsdtSize);
    if (RT_FAILURE(rc))
        return rc;

    GCPhysSsdt = GCPhysCur;
    GCPhysCur = RT_ALIGN_32(GCPhysCur + cbSsdtSize, 16);

    GCPhysDsdt = GCPhysCur;

    void  *pvDsdtCode = NULL;
    size_t cbDsdtSize = 0;
    rc = acpiPrepareDsdt(pThis->pDevIns, &pvDsdtCode, &cbDsdtSize);
    if (RT_FAILURE(rc))
        return rc;

    GCPhysCur = RT_ALIGN_32(GCPhysCur + cbDsdtSize, 16);

    if (GCPhysCur > 0x10000)
        return PDMDEV_SET_ERROR(pThis->pDevIns, VERR_TOO_MUCH_DATA,
                                N_("Error: ACPI tables bigger than 64KB"));

    Log(("RSDP 0x%08X\n", find_rsdp_space()));
    addend = pThis->cbRamLow - 0x10000;
    Log(("RSDT 0x%08X XSDT 0x%08X\n", GCPhysRsdt + addend, GCPhysXsdt + addend));
    Log(("FACS 0x%08X FADT (1.0) 0x%08X, FADT (2+) 0x%08X\n", GCPhysFacs + addend, GCPhysFadtAcpi1 + addend, GCPhysFadtAcpi2 + addend));
    Log(("DSDT 0x%08X", GCPhysDsdt + addend));
    if (pThis->u8UseIOApic)
        Log((" MADT 0x%08X", GCPhysApic + addend));
    if (pThis->fUseHpet)
        Log((" HPET 0x%08X", GCPhysHpet + addend));
    if (pThis->fUseMcfg)
        Log((" MCFG 0x%08X", GCPhysMcfg + addend));
    if (pThis->fUseCust)
        Log((" CUST 0x%08X", GCPhysCust + addend));
    Log((" SSDT 0x%08X", GCPhysSsdt + addend));
    Log(("\n"));

    acpiSetupRSDP(pThis, (ACPITBLRSDP *)pThis->au8RSDPPage, GCPhysRsdt + addend, GCPhysXsdt + addend);
    acpiSetupDSDT(pThis, GCPhysDsdt + addend, pvDsdtCode, cbDsdtSize);
    acpiCleanupDsdt(pThis->pDevIns, pvDsdtCode);
    acpiSetupFACS(pThis, GCPhysFacs + addend);
    acpiSetupFADT(pThis, GCPhysFadtAcpi1 + addend, GCPhysFadtAcpi2 + addend, GCPhysFacs + addend, GCPhysDsdt + addend);

    aGCPhysRsdt[0] = GCPhysFadtAcpi1 + addend;
    aGCPhysXsdt[0] = GCPhysFadtAcpi2 + addend;
    if (pThis->u8UseIOApic)
    {
        acpiSetupMADT(pThis, GCPhysApic + addend);
        aGCPhysRsdt[iMadt] = GCPhysApic + addend;
        aGCPhysXsdt[iMadt] = GCPhysApic + addend;
    }
    if (pThis->fUseHpet)
    {
        acpiSetupHPET(pThis, GCPhysHpet + addend);
        aGCPhysRsdt[iHpet] = GCPhysHpet + addend;
        aGCPhysXsdt[iHpet] = GCPhysHpet + addend;
    }
    if (pThis->fUseMcfg)
    {
        acpiSetupMCFG(pThis, GCPhysMcfg + addend);
        aGCPhysRsdt[iMcfg] = GCPhysMcfg + addend;
        aGCPhysXsdt[iMcfg] = GCPhysMcfg + addend;
    }
    if (pThis->fUseCust)
    {
        acpiSetupCUST(pThis, GCPhysCust + addend);
        aGCPhysRsdt[iCust] = GCPhysCust + addend;
        aGCPhysXsdt[iCust] = GCPhysCust + addend;
    }

    acpiSetupSSDT(pThis, GCPhysSsdt + addend, pvSsdtCode, cbSsdtSize);
    acpiCleanupSsdt(pThis->pDevIns, pvSsdtCode);
    aGCPhysRsdt[iSsdt] = GCPhysSsdt + addend;
    aGCPhysXsdt[iSsdt] = GCPhysSsdt + addend;

    rc = acpiSetupRSDT(pThis, GCPhysRsdt + addend, cAddr, aGCPhysRsdt);
    if (RT_FAILURE(rc))
        return rc;
    return acpiSetupXSDT(pThis, GCPhysXsdt + addend, cAddr, aGCPhysXsdt);
}

/**
 * @callback_method_impl{FNPCICONFIGREAD}
 */
static DECLCALLBACK(uint32_t) acpiPciConfigRead(PPCIDEVICE pPciDev, uint32_t Address, unsigned cb)
{
    PPDMDEVINS pDevIns = pPciDev->pDevIns;
    ACPIState *pThis   = PDMINS_2_DATA(pDevIns, ACPIState *);

    Log2(("acpi: PCI config read: 0x%x (%d)\n", Address, cb));
    return pThis->pfnAcpiPciConfigRead(pPciDev, Address, cb);
}

/**
 * @callback_method_impl{FNPCICONFIGWRITE}
 */
static DECLCALLBACK(void) acpiPciConfigWrite(PPCIDEVICE pPciDev, uint32_t Address, uint32_t u32Value, unsigned cb)
{
    PPDMDEVINS  pDevIns = pPciDev->pDevIns;
    ACPIState  *pThis   = PDMINS_2_DATA(pDevIns, ACPIState *);

    Log2(("acpi: PCI config write: 0x%x -> 0x%x (%d)\n", u32Value, Address, cb));
    DEVACPI_LOCK_R3(pThis);

    if (Address == VBOX_PCI_INTERRUPT_LINE)
    {
        Log(("acpi: ignore interrupt line settings: %d, we'll use hardcoded value %d\n", u32Value, SCI_INT));
        u32Value = SCI_INT;
    }

    pThis->pfnAcpiPciConfigWrite(pPciDev, Address, u32Value, cb);

    /* PMREGMISC written */
    if (Address == 0x80)
    {
        /* Check Power Management IO Space Enable (PMIOSE) bit */
        if (pPciDev->config[0x80] & 0x1)
        {
            RTIOPORT NewIoPortBase = (RTIOPORT)PCIDevGetDWord(pPciDev, 0x40);
            NewIoPortBase &= 0xffc0;

            int rc = acpiUpdatePmHandlers(pThis, NewIoPortBase);
            AssertRC(rc);
        }
    }

    DEVACPI_UNLOCK(pThis);
}

/**
 * Attach a new CPU.
 *
 * @returns VBox status code.
 * @param   pDevIns     The device instance.
 * @param   iLUN        The logical unit which is being attached.
 * @param   fFlags      Flags, combination of the PDMDEVATT_FLAGS_* \#defines.
 *
 * @remarks This code path is not used during construction.
 */
static DECLCALLBACK(int) acpiAttach(PPDMDEVINS pDevIns, unsigned iLUN, uint32_t fFlags)
{
    ACPIState *pThis = PDMINS_2_DATA(pDevIns, ACPIState *);
    LogFlow(("acpiAttach: pDevIns=%p iLUN=%u fFlags=%#x\n", pDevIns, iLUN, fFlags));

    AssertMsgReturn(!(fFlags & PDM_TACH_FLAGS_NOT_HOT_PLUG),
                    ("Hot-plug flag is not set\n"),
                    VERR_NOT_SUPPORTED);
    AssertReturn(iLUN < VMM_MAX_CPU_COUNT, VERR_PDM_NO_SUCH_LUN);

    /* Check if it was already attached */
    int rc = VINF_SUCCESS;
    DEVACPI_LOCK_R3(pThis);
    if (!VMCPUSET_IS_PRESENT(&pThis->CpuSetAttached, iLUN))
    {
        PPDMIBASE IBaseTmp;
        rc = PDMDevHlpDriverAttach(pDevIns, iLUN, &pThis->IBase, &IBaseTmp, "ACPI CPU");
        if (RT_SUCCESS(rc))
        {
            /* Enable the CPU */
            VMCPUSET_ADD(&pThis->CpuSetAttached, iLUN);

            /*
             * Lock the CPU because we don't know if the guest will use it or not.
             * Prevents ejection while the CPU is still used
             */
            VMCPUSET_ADD(&pThis->CpuSetLocked, iLUN);
            pThis->u32CpuEventType = CPU_EVENT_TYPE_ADD;
            pThis->u32CpuEvent     = iLUN;

            /* Notify the guest */
            update_gpe0(pThis, pThis->gpe0_sts | 0x2, pThis->gpe0_en);
        }
    }
    DEVACPI_UNLOCK(pThis);
    return rc;
}

/**
 * Detach notification.
 *
 * @param   pDevIns     The device instance.
 * @param   iLUN        The logical unit which is being detached.
 * @param   fFlags      Flags, combination of the PDMDEVATT_FLAGS_* \#defines.
 */
static DECLCALLBACK(void) acpiDetach(PPDMDEVINS pDevIns, unsigned iLUN, uint32_t fFlags)
{
    ACPIState *pThis = PDMINS_2_DATA(pDevIns, ACPIState *);

    LogFlow(("acpiDetach: pDevIns=%p iLUN=%u fFlags=%#x\n", pDevIns, iLUN, fFlags));

    AssertMsgReturnVoid(!(fFlags & PDM_TACH_FLAGS_NOT_HOT_PLUG),
                        ("Hot-plug flag is not set\n"));

    /* Check if it was already detached */
    DEVACPI_LOCK_R3(pThis);
    if (VMCPUSET_IS_PRESENT(&pThis->CpuSetAttached, iLUN))
    {
        if (!VMCPUSET_IS_PRESENT(&pThis->CpuSetLocked, iLUN))
        {
            /* Disable the CPU */
            VMCPUSET_DEL(&pThis->CpuSetAttached, iLUN);
            pThis->u32CpuEventType = CPU_EVENT_TYPE_REMOVE;
            pThis->u32CpuEvent     = iLUN;

            /* Notify the guest */
            update_gpe0(pThis, pThis->gpe0_sts | 0x2, pThis->gpe0_en);
        }
        else
            AssertMsgFailed(("CPU is still locked by the guest\n"));
    }
    DEVACPI_UNLOCK(pThis);
}

/**
 * @interface_method_impl{PDMDEVREG,pfnResume}
 */
static DECLCALLBACK(void) acpiResume(PPDMDEVINS pDevIns)
{
    ACPIState *pThis = PDMINS_2_DATA(pDevIns, ACPIState *);
    if (pThis->fSetWakeupOnResume)
    {
        Log(("acpiResume: setting WAK_STS\n"));
        pThis->fSetWakeupOnResume = false;
        pThis->pm1a_sts |= WAK_STS;
    }
}

/**
 * @interface_method_impl{PDMDEVREG,pfnReset}
 */
static DECLCALLBACK(void) acpiReset(PPDMDEVINS pDevIns)
{
    ACPIState *pThis = PDMINS_2_DATA(pDevIns, ACPIState *);

    TMTimerLock(pThis->pPmTimerR3, VERR_IGNORED);
    pThis->pm1a_en           = 0;
    pThis->pm1a_sts          = 0;
    pThis->pm1a_ctl          = 0;
    pThis->u64PmTimerInitial = TMTimerGet(pThis->pPmTimerR3);
    acpiPmTimerReset(pThis, pThis->u64PmTimerInitial);
    pThis->uBatteryIndex     = 0;
    pThis->uSystemInfoIndex  = 0;
    pThis->gpe0_en           = 0;
    pThis->gpe0_sts          = 0;
    pThis->uSleepState       = 0;
    TMTimerUnlock(pThis->pPmTimerR3);

    /** @todo Should we really reset PM base? */
    acpiUpdatePmHandlers(pThis, PM_PORT_BASE);

    acpiPlantTables(pThis);
}

/**
 * @interface_method_impl{PDMDEVREG,pfnRelocate}
 */
static DECLCALLBACK(void) acpiRelocate(PPDMDEVINS pDevIns, RTGCINTPTR offDelta)
{
    ACPIState *pThis = PDMINS_2_DATA(pDevIns, ACPIState *);
    pThis->pPmTimerRC = TMTimerRCPtr(pThis->pPmTimerR3);
    NOREF(offDelta);
}

/**
 * @interface_methid_impl{PDMDEVREG,pfnDestruct}
 */
static DECLCALLBACK(int) acpiDestruct(PPDMDEVINS pDevIns)
{
    ACPIState *pThis = PDMINS_2_DATA(pDevIns, ACPIState *);
    if (pThis->pu8CustBin)
    {
        MMR3HeapFree(pThis->pu8CustBin);
        pThis->pu8CustBin = NULL;
    }
    return VINF_SUCCESS;
}

/**
 * @interface_method_impl{PDMDEVREG,pfnConstruct}
 */
static DECLCALLBACK(int) acpiConstruct(PPDMDEVINS pDevIns, int iInstance, PCFGMNODE pCfg)
{
    ACPIState *pThis = PDMINS_2_DATA(pDevIns, ACPIState *);
    PDMDEV_CHECK_VERSIONS_RETURN(pDevIns);

    /*
     * Init data and set defaults.
     */
    /** @todo move more of the code up! */

    pThis->pDevIns = pDevIns;
    VMCPUSET_EMPTY(&pThis->CpuSetAttached);
    VMCPUSET_EMPTY(&pThis->CpuSetLocked);
    pThis->idCpuLockCheck  = UINT32_C(0xffffffff);
    pThis->u32CpuEventType = 0;
    pThis->u32CpuEvent     = UINT32_C(0xffffffff);

    /* The first CPU can't be attached/detached */
    VMCPUSET_ADD(&pThis->CpuSetAttached, 0);
    VMCPUSET_ADD(&pThis->CpuSetLocked, 0);

    /* IBase */
    pThis->IBase.pfnQueryInterface              = acpiQueryInterface;
    /* IACPIPort */
    pThis->IACPIPort.pfnSleepButtonPress        = acpiPort_SleepButtonPress;
    pThis->IACPIPort.pfnPowerButtonPress        = acpiPort_PowerButtonPress;
    pThis->IACPIPort.pfnGetPowerButtonHandled   = acpiPort_GetPowerButtonHandled;
    pThis->IACPIPort.pfnGetGuestEnteredACPIMode = acpiPort_GetGuestEnteredACPIMode;
    pThis->IACPIPort.pfnGetCpuStatus            = acpiPort_GetCpuStatus;

    /* Set the default critical section to NOP (related to the PM timer). */
    int rc = PDMDevHlpSetDeviceCritSect(pDevIns, PDMDevHlpCritSectGetNop(pDevIns));
    AssertRCReturn(rc, rc);

    rc = PDMDevHlpCritSectInit(pDevIns, &pThis->CritSect, RT_SRC_POS, "acpi%u", iInstance);
    AssertRCReturn(rc, rc);

    /*
     * Validate and read the configuration.
     */
    if (!CFGMR3AreValuesValid(pCfg,
                              "RamSize\0"
                              "RamHoleSize\0"
                              "IOAPIC\0"
                              "NumCPUs\0"
                              "GCEnabled\0"
                              "R0Enabled\0"
                              "HpetEnabled\0"
                              "McfgEnabled\0"
                              "McfgBase\0"
                              "McfgLength\0"
                              "SmcEnabled\0"
                              "FdcEnabled\0"
                              "ShowRtc\0"
                              "ShowCpu\0"
                              "NicPciAddress\0"
                              "AudioPciAddress\0"
                              "IocPciAddress\0"
                              "HostBusPciAddress\0"
                              "EnableSuspendToDisk\0"
                              "PowerS1Enabled\0"
                              "PowerS4Enabled\0"
                              "CpuHotPlug\0"
                              "AmlFilePath\0"
                              "Serial0IoPortBase\0"
                              "Serial1IoPortBase\0"
                              "Serial0Irq\0"
                              "Serial1Irq\0"
                              "AcpiOemId\0"
                              "AcpiCreatorId\0"
                              "AcpiCreatorRev\0"
                              "CustomTable\0"
                              "SLICTable\0"
                              ))
        return PDMDEV_SET_ERROR(pDevIns, VERR_PDM_DEVINS_UNKNOWN_CFG_VALUES,
                                N_("Configuration error: Invalid config key for ACPI device"));

    /* query whether we are supposed to present an IOAPIC */
    rc = CFGMR3QueryU8Def(pCfg, "IOAPIC", &pThis->u8UseIOApic, 1);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"IOAPIC\""));

    rc = CFGMR3QueryU16Def(pCfg, "NumCPUs", &pThis->cCpus, 1);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Querying \"NumCPUs\" as integer failed"));

    /* query whether we are supposed to present an FDC controller */
    rc = CFGMR3QueryBoolDef(pCfg, "FdcEnabled", &pThis->fUseFdc, true);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"FdcEnabled\""));

    /* query whether we are supposed to present HPET */
    rc = CFGMR3QueryBoolDef(pCfg, "HpetEnabled", &pThis->fUseHpet, false);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"HpetEnabled\""));
    /* query MCFG configuration */
    rc = CFGMR3QueryU64Def(pCfg, "McfgBase", &pThis->u64PciConfigMMioAddress, 0);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"McfgBase\""));
    rc = CFGMR3QueryU64Def(pCfg, "McfgLength", &pThis->u64PciConfigMMioLength, 0);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"McfgLength\""));
    pThis->fUseMcfg = (pThis->u64PciConfigMMioAddress != 0) && (pThis->u64PciConfigMMioLength != 0);

    /* query whether we are supposed to present custom table */
    pThis->fUseCust = false;

    /* query whether we are supposed to present SMC */
    rc = CFGMR3QueryBoolDef(pCfg, "SmcEnabled", &pThis->fUseSmc, false);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"SmcEnabled\""));

    /* query whether we are supposed to present RTC object */
    rc = CFGMR3QueryBoolDef(pCfg, "ShowRtc", &pThis->fShowRtc, false);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"ShowRtc\""));

    /* query whether we are supposed to present CPU objects */
    rc = CFGMR3QueryBoolDef(pCfg, "ShowCpu", &pThis->fShowCpu, false);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"ShowCpu\""));

    /* query primary NIC PCI address */
    rc = CFGMR3QueryU32Def(pCfg, "NicPciAddress", &pThis->u32NicPciAddress, 0);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"NicPciAddress\""));

    /* query primary NIC PCI address */
    rc = CFGMR3QueryU32Def(pCfg, "AudioPciAddress", &pThis->u32AudioPciAddress, 0);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"AudioPciAddress\""));

    /* query IO controller (southbridge) PCI address */
    rc = CFGMR3QueryU32Def(pCfg, "IocPciAddress", &pThis->u32IocPciAddress, 0);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"IocPciAddress\""));

    /* query host bus controller PCI address */
    rc = CFGMR3QueryU32Def(pCfg, "HostBusPciAddress", &pThis->u32HbcPciAddress, 0);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"HostBusPciAddress\""));

    /* query whether S1 power state should be exposed */
    rc = CFGMR3QueryBoolDef(pCfg, "PowerS1Enabled", &pThis->fS1Enabled, false);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"PowerS1Enabled\""));

    /* query whether S4 power state should be exposed */
    rc = CFGMR3QueryBoolDef(pCfg, "PowerS4Enabled", &pThis->fS4Enabled, false);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"PowerS4Enabled\""));

    /* query whether S1 power state should save the VM state */
    rc = CFGMR3QueryBoolDef(pCfg, "EnableSuspendToDisk", &pThis->fSuspendToSavedState, false);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"EnableSuspendToDisk\""));

    /* query whether we are allow CPU hot plugging */
    rc = CFGMR3QueryBoolDef(pCfg, "CpuHotPlug", &pThis->fCpuHotPlug, false);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"CpuHotPlug\""));

    rc = CFGMR3QueryBoolDef(pCfg, "GCEnabled", &pThis->fGCEnabled, true);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"GCEnabled\""));

    rc = CFGMR3QueryBoolDef(pCfg, "R0Enabled", &pThis->fR0Enabled, true);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("configuration error: failed to read \"R0Enabled\""));

    /* query serial info */
    rc = CFGMR3QueryU8Def(pCfg, "Serial0Irq", &pThis->uSerial0Irq, 4);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"Serial0Irq\""));

    rc = CFGMR3QueryU16Def(pCfg, "Serial0IoPortBase", &pThis->uSerial0IoPortBase, 0x3f8);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"Serial0IoPortBase\""));

    /* Serial 1 is enabled, get config data */
    rc = CFGMR3QueryU8Def(pCfg, "Serial1Irq", &pThis->uSerial1Irq, 3);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"Serial1Irq\""));

    rc = CFGMR3QueryU16Def(pCfg, "Serial1IoPortBase", &pThis->uSerial1IoPortBase, 0x2f8);
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pDevIns, rc,
                                N_("Configuration error: Failed to read \"Serial1IoPortBase\""));

    /* Try to attach the other CPUs */
    for (unsigned i = 1; i < pThis->cCpus; i++)
    {
        if (pThis->fCpuHotPlug)
        {
            PPDMIBASE IBaseTmp;
            rc = PDMDevHlpDriverAttach(pDevIns, i, &pThis->IBase, &IBaseTmp, "ACPI CPU");

            if (RT_SUCCESS(rc))
            {
                VMCPUSET_ADD(&pThis->CpuSetAttached, i);
                VMCPUSET_ADD(&pThis->CpuSetLocked, i);
                Log(("acpi: Attached CPU %u\n", i));
            }
            else if (rc == VERR_PDM_NO_ATTACHED_DRIVER)
                Log(("acpi: CPU %u not attached yet\n", i));
            else
                return PDMDEV_SET_ERROR(pDevIns, rc, N_("Failed to attach CPU object\n"));
        }
        else
        {
            /* CPU is always attached if hot-plug is not enabled. */
            VMCPUSET_ADD(&pThis->CpuSetAttached, i);
            VMCPUSET_ADD(&pThis->CpuSetLocked, i);
        }
    }

    char *pszOemId = NULL;
    rc = CFGMR3QueryStringAllocDef(pThis->pDevIns->pCfg, "AcpiOemId", &pszOemId, "VBOX  ");
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pThis->pDevIns, rc,
                                N_("Configuration error: Querying \"AcpiOemId\" as string failed"));
    size_t cbOemId = strlen(pszOemId);
    if (cbOemId > 6)
        return PDMDEV_SET_ERROR(pThis->pDevIns, rc,
                                N_("Configuration error: \"AcpiOemId\" must contain not more than 6 characters"));
    memset(pThis->au8OemId, ' ', sizeof(pThis->au8OemId));
    memcpy(pThis->au8OemId, pszOemId, cbOemId);
    MMR3HeapFree(pszOemId);

    char *pszCreatorId = NULL;
    rc = CFGMR3QueryStringAllocDef(pThis->pDevIns->pCfg, "AcpiCreatorId", &pszCreatorId, "ASL ");
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pThis->pDevIns, rc,
                                N_("Configuration error: Querying \"AcpiCreatorId\" as string failed"));
    size_t cbCreatorId = strlen(pszCreatorId);
    if (cbCreatorId > 4)
        return PDMDEV_SET_ERROR(pThis->pDevIns, rc,
                                N_("Configuration error: \"AcpiCreatorId\" must contain not more than 4 characters"));
    memset(pThis->au8CreatorId, ' ', sizeof(pThis->au8CreatorId));
    memcpy(pThis->au8CreatorId, pszCreatorId, cbCreatorId);
    MMR3HeapFree(pszCreatorId);

    rc = CFGMR3QueryU32Def(pThis->pDevIns->pCfg, "AcpiCreatorRev", &pThis->u32CreatorRev, RT_H2LE_U32(0x61));
    if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pThis->pDevIns, rc,
                                N_("Configuration error: Querying \"AcpiCreatorRev\" as integer failed"));
    pThis->u32OemRevision         = RT_H2LE_U32(0x1);

    /*
     * Get the custom table binary file name.
     */
    char *pszCustBinFile;
    rc = CFGMR3QueryStringAlloc(pThis->pDevIns->pCfg, "CustomTable", &pszCustBinFile);
    if (rc == VERR_CFGM_VALUE_NOT_FOUND)
        rc = CFGMR3QueryStringAlloc(pThis->pDevIns->pCfg, "SLICTable", &pszCustBinFile);
    if (rc == VERR_CFGM_VALUE_NOT_FOUND)
    {
        pszCustBinFile = NULL;
        rc = VINF_SUCCESS;
    }
    else if (RT_FAILURE(rc))
        return PDMDEV_SET_ERROR(pThis->pDevIns, rc,
                                N_("Configuration error: Querying \"CustomTable\" as a string failed"));
    else if (!*pszCustBinFile)
    {
        MMR3HeapFree(pszCustBinFile);
        pszCustBinFile = NULL;
    }

    /*
     * Determine the custom table binary size, open specified ROM file in the process.
     */
    if (pszCustBinFile)
    {
        RTFILE FileCUSTBin;
        rc = RTFileOpen(&FileCUSTBin, pszCustBinFile,
                        RTFILE_O_READ | RTFILE_O_OPEN | RTFILE_O_DENY_WRITE);
        if (RT_SUCCESS(rc))
        {
            rc = RTFileGetSize(FileCUSTBin, &pThis->cbCustBin);
            if (RT_SUCCESS(rc))
            {
                /* The following checks should be in sync the AssertReleaseMsg's below. */
                if (    pThis->cbCustBin > 3072
                    ||  pThis->cbCustBin < sizeof(ACPITBLHEADER))
                    rc = VERR_TOO_MUCH_DATA;

                /*
                 * Allocate buffer for the custom table binary data.
                 */
                pThis->pu8CustBin = (uint8_t *)PDMDevHlpMMHeapAlloc(pThis->pDevIns, pThis->cbCustBin);
                if (pThis->pu8CustBin)
                {
                    rc = RTFileRead(FileCUSTBin, pThis->pu8CustBin, pThis->cbCustBin, NULL);
                    if (RT_FAILURE(rc))
                    {
                        AssertMsgFailed(("RTFileRead(,,%d,NULL) -> %Rrc\n", pThis->cbCustBin, rc));
                        MMR3HeapFree(pThis->pu8CustBin);
                        pThis->pu8CustBin = NULL;
                    }
                    else
                    {
                        pThis->fUseCust = true;
                        memcpy(&pThis->au8OemId[0], &pThis->pu8CustBin[10], 6);
                        memcpy(&pThis->au8OemTabId[0], &pThis->pu8CustBin[16], 8);
                        memcpy(&pThis->u32OemRevision, &pThis->pu8CustBin[24], 4);
                        memcpy(&pThis->au8CreatorId[0], &pThis->pu8CustBin[28], 4);
                        memcpy(&pThis->u32CreatorRev, &pThis->pu8CustBin[32], 4);
                        LogRel(("Reading custom ACPI table from file '%s' (%d bytes)\n", pszCustBinFile, pThis->cbCustBin));
                    }
                }
                else
                    rc = VERR_NO_MEMORY;

                RTFileClose(FileCUSTBin);
            }
        }
        MMR3HeapFree(pszCustBinFile);
        if (RT_FAILURE(rc))
            return PDMDEV_SET_ERROR(pDevIns, rc,
                                    N_("Error reading custom ACPI table"));
    }

    /* Set default port base */
    pThis->uPmIoPortBase = PM_PORT_BASE;

    /*
     * FDC and SMC try to use the same non-shareable interrupt (6),
     * enable only one device.
     */
    if (pThis->fUseSmc)
        pThis->fUseFdc = false;

    /*
     * Plant ACPI tables.
     */
    RTGCPHYS32 GCPhysRsdp = find_rsdp_space();
    if (!GCPhysRsdp)
        return PDMDEV_SET_ERROR(pDevIns, VERR_NO_MEMORY,
                                N_("Can not find space for RSDP. ACPI is disabled"));

    rc = acpiPlantTables(pThis);
    if (RT_FAILURE(rc))
        return rc;

    rc = PDMDevHlpROMRegister(pDevIns, GCPhysRsdp, 0x1000, pThis->au8RSDPPage, 0x1000,
                              PGMPHYS_ROM_FLAGS_PERMANENT_BINARY, "ACPI RSDP");
    if (RT_FAILURE(rc))
        return rc;

    /*
     * Register I/O ports.
     */
    rc = acpiRegisterPmHandlers(pThis);
    if (RT_FAILURE(rc))
        return rc;

#define R(addr, cnt, writer, reader, description)       \
    do { \
        rc = PDMDevHlpIOPortRegister(pDevIns, addr, cnt, pThis, writer, reader, \
                                      NULL, NULL, description); \
        if (RT_FAILURE(rc)) \
            return rc; \
    } while (0)
    R(SMI_CMD,        1, acpiSmiWrite,          NULL,                "ACPI SMI");
#ifdef DEBUG_ACPI
    R(DEBUG_HEX,      1, acpiDhexWrite,         NULL,                "ACPI Debug hex");
    R(DEBUG_CHR,      1, acpiDchrWrite,         NULL,                "ACPI Debug char");
#endif
    R(BAT_INDEX,      1, acpiBatIndexWrite,     NULL,                "ACPI Battery status index");
    R(BAT_DATA,       1, NULL,                  acpiBatDataRead,     "ACPI Battery status data");
    R(SYSI_INDEX,     1, acpiSysInfoIndexWrite, NULL,                "ACPI system info index");
    R(SYSI_DATA,      1, acpiSysInfoDataWrite,  acpiSysInfoDataRead, "ACPI system info data");
    R(ACPI_RESET_BLK, 1, acpiResetWrite,        NULL,                "ACPI Reset");
#undef R

    /*
     * Create the PM timer.
     */
    PTMTIMER pTimer;
    rc = PDMDevHlpTMTimerCreate(pDevIns, TMCLOCK_VIRTUAL_SYNC, acpiPmTimer, &pThis->dev,
                                TMTIMER_FLAGS_NO_CRIT_SECT, "ACPI PM Timer", &pTimer);
    AssertRCReturn(rc, rc);
    pThis->pPmTimerR3 = pTimer;
    pThis->pPmTimerR0 = TMTimerR0Ptr(pTimer);
    pThis->pPmTimerRC = TMTimerRCPtr(pTimer);

    rc = TMTimerLock(pTimer, VERR_IGNORED);
    AssertRCReturn(rc, rc);
    pThis->u64PmTimerInitial = TMTimerGet(pTimer);
    acpiPmTimerReset(pThis, pThis->u64PmTimerInitial);
    TMTimerUnlock(pTimer);

    /*
     * Set up the PCI device.
     */
    PCIDevSetVendorId(&pThis->dev, 0x8086); /* Intel */
    PCIDevSetDeviceId(&pThis->dev, 0x7113); /* 82371AB */

    /* See p. 50 of PIIX4 manual */
    PCIDevSetCommand(&pThis->dev, 0x01);
    PCIDevSetStatus(&pThis->dev, 0x0280);

    PCIDevSetRevisionId(&pThis->dev, 0x08);

    PCIDevSetClassProg(&pThis->dev, 0x00);
    PCIDevSetClassSub(&pThis->dev, 0x80);
    PCIDevSetClassBase(&pThis->dev, 0x06);

    PCIDevSetHeaderType(&pThis->dev, 0x80);

    PCIDevSetBIST(&pThis->dev, 0x00);

    PCIDevSetInterruptLine(&pThis->dev, SCI_INT);
    PCIDevSetInterruptPin (&pThis->dev, 0x01);

    pThis->dev.config[0x40] = 0x01; /* PM base address, this bit marks it as IO range, not PA */

#if 0
    int smb_io_base = 0xb100;
    dev->config[0x90] = smb_io_base | 1; /* SMBus base address */
    dev->config[0x90] = smb_io_base >> 8;
#endif

    rc = PDMDevHlpPCIRegister(pDevIns, &pThis->dev);
    if (RT_FAILURE(rc))
        return rc;

    PDMDevHlpPCISetConfigCallbacks(pDevIns, &pThis->dev,
                                   acpiPciConfigRead,  &pThis->pfnAcpiPciConfigRead,
                                   acpiPciConfigWrite, &pThis->pfnAcpiPciConfigWrite);

    /*
     * Register the saved state.
     */
    rc = PDMDevHlpSSMRegister(pDevIns, 6, sizeof(*pThis), acpiSaveState, acpiLoadState);
    if (RT_FAILURE(rc))
        return rc;

   /*
    * Get the corresponding connector interface
    */
   rc = PDMDevHlpDriverAttach(pDevIns, 0, &pThis->IBase, &pThis->pDrvBase, "ACPI Driver Port");
   if (RT_SUCCESS(rc))
   {
       pThis->pDrv = PDMIBASE_QUERY_INTERFACE(pThis->pDrvBase, PDMIACPICONNECTOR);
       if (!pThis->pDrv)
           return PDMDEV_SET_ERROR(pDevIns, VERR_PDM_MISSING_INTERFACE,
                                   N_("LUN #0 doesn't have an ACPI connector interface"));
   }
   else if (rc == VERR_PDM_NO_ATTACHED_DRIVER)
   {
       Log(("acpi: %s/%d: warning: no driver attached to LUN #0!\n",
            pDevIns->pReg->szName, pDevIns->iInstance));
       rc = VINF_SUCCESS;
   }
   else
       return PDMDEV_SET_ERROR(pDevIns, rc, N_("Failed to attach LUN #0"));

    return rc;
}

/**
 * The device registration structure.
 */
const PDMDEVREG g_DeviceACPI =
{
    /* u32Version */
    PDM_DEVREG_VERSION,
    /* szName */
    "acpi",
    /* szRCMod */
    "VBoxDDGC.gc",
    /* szR0Mod */
    "VBoxDDR0.r0",
    /* pszDescription */
    "Advanced Configuration and Power Interface",
    /* fFlags */
    PDM_DEVREG_FLAGS_DEFAULT_BITS | PDM_DEVREG_FLAGS_RC | PDM_DEVREG_FLAGS_R0,
    /* fClass */
    PDM_DEVREG_CLASS_ACPI,
    /* cMaxInstances */
    ~0U,
    /* cbInstance */
    sizeof(ACPIState),
    /* pfnConstruct */
    acpiConstruct,
    /* pfnDestruct */
    acpiDestruct,
    /* pfnRelocate */
    acpiRelocate,
    /* pfnIOCtl */
    NULL,
    /* pfnPowerOn */
    NULL,
    /* pfnReset */
    acpiReset,
    /* pfnSuspend */
    NULL,
    /* pfnResume */
    acpiResume,
    /* pfnAttach */
    acpiAttach,
    /* pfnDetach */
    acpiDetach,
    /* pfnQueryInterface. */
    NULL,
    /* pfnInitComplete */
    NULL,
    /* pfnPowerOff */
    NULL,
    /* pfnSoftReset */
    NULL,
    /* u32VersionEnd */
    PDM_DEVREG_VERSION
};

#endif /* IN_RING3 */
#endif /* !VBOX_DEVICE_STRUCT_TESTCASE */
