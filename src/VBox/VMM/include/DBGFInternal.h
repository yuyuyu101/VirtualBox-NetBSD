/* $Id: DBGFInternal.h $ */
/** @file
 * DBGF - Internal header file.
 */

/*
 * Copyright (C) 2006-2011 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#ifndef ___DBGFInternal_h
#define ___DBGFInternal_h

#include <VBox/cdefs.h>
#include <VBox/types.h>
#include <iprt/semaphore.h>
#include <iprt/critsect.h>
#include <iprt/string.h>
#include <iprt/avl.h>
#include <VBox/vmm/dbgf.h>



/** @defgroup grp_dbgf_int   Internals
 * @ingroup grp_dbgf
 * @internal
 * @{
 */


/** VMM Debugger Command. */
typedef enum DBGFCMD
{
    /** No command.
     * This is assigned to the field by the emulation thread after
     * a command has been completed. */
    DBGFCMD_NO_COMMAND = 0,
    /** Halt the VM. */
    DBGFCMD_HALT,
    /** Resume execution. */
    DBGFCMD_GO,
    /** Single step execution - stepping into calls. */
    DBGFCMD_SINGLE_STEP,
    /** Set a breakpoint. */
    DBGFCMD_BREAKPOINT_SET,
    /** Set a access breakpoint. */
    DBGFCMD_BREAKPOINT_SET_ACCESS,
    /** Set a REM breakpoint. */
    DBGFCMD_BREAKPOINT_SET_REM,
    /** Clear a breakpoint. */
    DBGFCMD_BREAKPOINT_CLEAR,
    /** Enable a breakpoint. */
    DBGFCMD_BREAKPOINT_ENABLE,
    /** Disable a breakpoint. */
    DBGFCMD_BREAKPOINT_DISABLE,
    /** List breakpoints. */
    DBGFCMD_BREAKPOINT_LIST,

    /** Detaches the debugger.
     * Disabling all breakpoints, watch points and the like. */
    DBGFCMD_DETACH_DEBUGGER = 0x7ffffffe,
    /** Detached the debugger.
     * The isn't a command as such, it's just that it's necessary for the
     * detaching protocol to be racefree. */
    DBGFCMD_DETACHED_DEBUGGER = 0x7fffffff
} DBGFCMD;

/**
 * VMM Debugger Command.
 */
typedef union DBGFCMDDATA
{
    uint32_t    uDummy;
} DBGFCMDDATA;
/** Pointer to DBGF Command Data. */
typedef DBGFCMDDATA *PDBGFCMDDATA;

/**
 * Info type.
 */
typedef enum DBGFINFOTYPE
{
    /** Invalid. */
    DBGFINFOTYPE_INVALID = 0,
    /** Device owner. */
    DBGFINFOTYPE_DEV,
    /** Driver owner. */
    DBGFINFOTYPE_DRV,
    /** Internal owner. */
    DBGFINFOTYPE_INT,
    /** External owner. */
    DBGFINFOTYPE_EXT
} DBGFINFOTYPE;


/** Pointer to info structure. */
typedef struct DBGFINFO *PDBGFINFO;

#ifdef IN_RING3
/**
 * Info structure.
 */
typedef struct DBGFINFO
{
    /** The flags. */
    uint32_t        fFlags;
    /** Owner type. */
    DBGFINFOTYPE    enmType;
    /** Per type data. */
    union
    {
        /** DBGFINFOTYPE_DEV */
        struct
        {
            /** Device info handler function. */
            PFNDBGFHANDLERDEV   pfnHandler;
            /** The device instance. */
            PPDMDEVINS          pDevIns;
        } Dev;

        /** DBGFINFOTYPE_DRV */
        struct
        {
            /** Driver info handler function. */
            PFNDBGFHANDLERDRV   pfnHandler;
            /** The driver instance. */
            PPDMDRVINS          pDrvIns;
        } Drv;

        /** DBGFINFOTYPE_INT */
        struct
        {
            /** Internal info handler function. */
            PFNDBGFHANDLERINT   pfnHandler;
        } Int;

        /** DBGFINFOTYPE_EXT */
        struct
        {
            /** External info handler function. */
            PFNDBGFHANDLEREXT   pfnHandler;
            /** The user argument. */
            void               *pvUser;
        } Ext;
    } u;

    /** Pointer to the description. */
    const char     *pszDesc;
    /** Pointer to the next info structure. */
    PDBGFINFO       pNext;
    /** The identifier name length. */
    size_t          cchName;
    /** The identifier name. (Extends 'beyond' the struct as usual.) */
    char            szName[1];
} DBGFINFO;
#endif /* IN_RING3 */


/**
 * Guest OS digger instance.
 */
typedef struct DBGFOS
{
    /** Pointer to the registration record. */
    PCDBGFOSREG pReg;
    /** Pointer to the next OS we've registered. */
    struct DBGFOS *pNext;
    /** The instance data (variable size). */
    uint8_t abData[16];
} DBGFOS;
/** Pointer to guest OS digger instance. */
typedef DBGFOS *PDBGFOS;
/** Pointer to const guest OS digger instance. */
typedef DBGFOS const *PCDBGFOS;


/**
 * Converts a DBGF pointer into a VM pointer.
 * @returns Pointer to the VM structure the CPUM is part of.
 * @param   pDBGF   Pointer to DBGF instance data.
 */
#define DBGF2VM(pDBGF)  ( (PVM)((char*)pDBGF - pDBGF->offVM) )


/**
 * DBGF Data (part of VM)
 */
typedef struct DBGF
{
    /** Offset to the VM structure. */
    int32_t                     offVM;

    /** Debugger Attached flag.
     * Set if a debugger is attached, elsewise it's clear.
     */
    bool volatile               fAttached;

    /** Stopped in the Hypervisor.
     * Set if we're stopped on a trace, breakpoint or assertion inside
     * the hypervisor and have to restrict the available operations.
     */
    bool volatile               fStoppedInHyper;

    /**
     * Ping-Pong construct where the Ping side is the VMM and the Pong side
     * the Debugger.
     */
    RTPINGPONG                  PingPong;

    /** The Event to the debugger.
     * The VMM will ping the debugger when the event is ready. The event is
     * either a response to a command or to a break/watch point issued
     * previously.
     */
    DBGFEVENT                   DbgEvent;

    /** The Command to the VMM.
     * Operated in an atomic fashion since the VMM will poll on this.
     * This means that a the command data must be written before this member
     * is set. The VMM will reset this member to the no-command state
     * when it have processed it.
     */
    DBGFCMD volatile            enmVMMCmd;
    /** The Command data.
     * Not all commands take data. */
    DBGFCMDDATA                 VMMCmdData;

    /** List of registered info handlers. */
    R3PTRTYPE(PDBGFINFO)        pInfoFirst;
    /** Critical section protecting the above list. */
    RTCRITSECT                  InfoCritSect;

    /** Range tree containing the loaded symbols of the a VM.
     * This tree will never have blind spots. */
    R3PTRTYPE(AVLRGCPTRTREE)    SymbolTree;
    /** Symbol name space. */
    R3PTRTYPE(PRTSTRSPACE)      pSymbolSpace;
    /** Indicates whether DBGFSym.cpp is initialized or not.
     * This part is initialized in a lazy manner for performance reasons. */
    bool                        fSymInited;
    /** Alignment padding. */
    uint32_t                    uAlignment0;

    /** The number of hardware breakpoints. */
    uint32_t                    cHwBreakpoints;
    /** The number of active breakpoints. */
    uint32_t                    cBreakpoints;
    /** Array of hardware breakpoints. (0..3)
     * This is shared among all the CPUs because life is much simpler that way. */
    DBGFBP                      aHwBreakpoints[4];
    /** Array of int 3 and REM breakpoints. (4..)
     * @remark This is currently a fixed size array for reasons of simplicity. */
    DBGFBP                      aBreakpoints[32];

    /** The address space database lock. */
    RTSEMRW                     hAsDbLock;
    /** The address space handle database.      (Protected by hAsDbLock.) */
    R3PTRTYPE(AVLPVTREE)        AsHandleTree;
    /** The address space process id database.  (Protected by hAsDbLock.) */
    R3PTRTYPE(AVLU32TREE)       AsPidTree;
    /** The address space name database.        (Protected by hAsDbLock.) */
    R3PTRTYPE(RTSTRSPACE)       AsNameSpace;
    /** Special address space aliases.          (Protected by hAsDbLock.) */
    RTDBGAS volatile            ahAsAliases[DBGF_AS_COUNT];
    /** For lazily populating the aliased address spaces. */
    bool volatile               afAsAliasPopuplated[DBGF_AS_COUNT];
    /** Alignment padding. */
    bool                        afAlignment1[2];

    /** The register database lock. */
    RTSEMRW                     hRegDbLock;
    /** String space for looking up registers.  (Protected by hRegDbLock.) */
    R3PTRTYPE(RTSTRSPACE)       RegSpace;
    /** String space holding the register sets. (Protected by hRegDbLock.)  */
    R3PTRTYPE(RTSTRSPACE)       RegSetSpace;
    /** The number of registers (aliases, sub-fields and the special CPU
     * register aliases (eg AH) are not counted). */
    uint32_t                    cRegs;
    /** For early initialization by . */
    bool volatile               fRegDbInitialized;
    /** Alignment padding. */
    bool                        afAlignment2[3];

    /** The current Guest OS digger. */
    R3PTRTYPE(PDBGFOS)          pCurOS;
    /** The head of the Guest OS digger instances. */
    R3PTRTYPE(PDBGFOS)          pOSHead;
} DBGF;
/** Pointer to DBGF Data. */
typedef DBGF *PDBGF;


/** Converts a DBGFCPU pointer into a VM pointer. */
#define DBGFCPU_2_VM(pDbgfCpu) ((PVM)((uint8_t *)(pDbgfCpu) + (pDbgfCpu)->offVM))

/**
 * The per CPU data for DBGF.
 */
typedef struct DBGFCPU
{
    /** The offset into the VM structure.
     * @see DBGFCPU_2_VM(). */
    uint32_t                offVM;

    /** Current active breakpoint (id).
     * This is ~0U if not active. It is set when a execution engine
     * encounters a breakpoint and returns VINF_EM_DBG_BREAKPOINT. This is
     * currently not used for REM breakpoints because of the lazy coupling
     * between VBox and REM. */
    uint32_t                iActiveBp;
    /** Set if we're singlestepping in raw mode.
     * This is checked and cleared in the \#DB handler. */
    bool                    fSingleSteppingRaw;

    /** Padding the structure to 16 bytes. */
    bool                    afReserved[7];

    /** The guest register set for this CPU.  Can be NULL. */
    R3PTRTYPE(struct DBGFREGSET *) pGuestRegSet;
    /** The hypervisor register set for this CPU.  Can be NULL. */
    R3PTRTYPE(struct DBGFREGSET *) pHyperRegSet;
} DBGFCPU;
/** Pointer to DBGFCPU data. */
typedef DBGFCPU *PDBGFCPU;


int  dbgfR3AsInit(PVM pVM);
void dbgfR3AsTerm(PVM pVM);
void dbgfR3AsRelocate(PVM pVM, RTGCUINTPTR offDelta);
int  dbgfR3BpInit(PVM pVM);
int  dbgfR3InfoInit(PVM pVM);
int  dbgfR3InfoTerm(PVM pVM);
void dbgfR3OSTerm(PVM pVM);
int  dbgfR3RegInit(PVM pVM);
void dbgfR3RegTerm(PVM pVM);
int  dbgfR3SymInit(PVM pVM);
int  dbgfR3SymTerm(PVM pVM);
int  dbgfR3TraceInit(PVM pVM);
void dbgfR3TraceRelocate(PVM pVM);
void dbgfR3TraceTerm(PVM pVM);



#ifdef IN_RING3

#endif

/** @} */

#endif
