/* $Id: KeyboardImpl.h $ */
/** @file
 * VBox frontends: Basic Frontend (BFE):
 * Declaration of Keyboard class and related things
 */

/*
 * Copyright (C) 2006-2010 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#ifndef ____H_KEYBOARDIMPL
#define ____H_KEYBOARDIMPL

#include <VBox/vmm/pdm.h>

/** Simple keyboard event class. */
class KeyboardEvent
{
public:
    KeyboardEvent() : scan(-1) {}
    KeyboardEvent(int _scan) : scan(_scan) {}
    bool isValid()
    {
        return (scan & ~0x80) && !(scan & ~0xFF);
    }
    int scan;
};

class Keyboard
{

public:

    Keyboard();
    virtual ~Keyboard();

    STDMETHOD(PutScancode)(LONG scancode);
    STDMETHOD(PutScancodes)(ComSafeArrayIn (LONG, scancodes),
                            ULONG *codesStored);
    STDMETHOD(PutCAD)();

    static const PDMDRVREG  DrvReg;

private:

    static DECLCALLBACK(void *) drvQueryInterface(PPDMIBASE pInterface, const char *pszIID);
    static DECLCALLBACK(int)    drvConstruct(PPDMDRVINS pDrvIns, PCFGMNODE pCfg, uint32_t fFlags);
    static DECLCALLBACK(void)   drvDestruct(PPDMDRVINS pDrvIns);

    /** Pointer to the associated keyboard driver. */
    struct DRVMAINKEYBOARD *mpDrv;
    /** Pointer to the device instance for the VMM Device. */
    PPDMDEVINS              mpVMMDev;
    /** Set after the first attempt to find the VMM Device. */
    bool                    mfVMMDevInited;
};

extern Keyboard *gKeyboard;

#endif // !____H_KEYBOARDIMPL
