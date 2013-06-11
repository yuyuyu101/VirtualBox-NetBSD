/** @file
 *
 * VBox frontends: Qt GUI ("VirtualBox"):
 * UIKeyboardHandlerFullscreen class declaration
 */

/*
 * Copyright (C) 2010 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#ifndef ___UIKeyboardHandlerFullscreen_h___
#define ___UIKeyboardHandlerFullscreen_h___

/* Local includes */
#include "UIKeyboardHandler.h"

class UIKeyboardHandlerFullscreen : public UIKeyboardHandler
{
    Q_OBJECT;

protected:

    /* Fullscreen keyboard-handler constructor/destructor: */
    UIKeyboardHandlerFullscreen(UIMachineLogic *pMachineLogic);
    virtual ~UIKeyboardHandlerFullscreen();

private:

    /* Event handlers: */
    bool eventFilter(QObject *pWatched, QEvent *pEvent);

    /* Friend classes: */
    friend class UIKeyboardHandler;
};

#endif // !___UIKeyboardHandlerFullscreen_h___

