/* $Id: VBoxTrayMsg.h $ */
/** @file
 * VBoxTrayMsg - Globally registered messages (RPC) to/from VBoxTray.
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

#ifndef ___VBOXTRAY_MSG_H
#define ___VBOXTRAY_MSG_H

#define VBOXTRAY_PIPE_IPC               "\\\\.\\pipe\\VBoxTrayIPC"
#define VBOXTRAY_PIPE_IPC_BUFSIZE       64 * 1024

enum VBOXTRAYIPCMSGTYPE
{
    /** Restarts VBoxTray. */
    VBOXTRAYIPCMSGTYPE_RESTART        = 10,

    /** Asks the IPC thread to quit. */
    VBOXTRAYIPCMSGTYPE_IPC_QUIT       = 50,

    /** Shows a balloon message in the tray area. */
    VBOXTRAYIPCMSGTYPE_SHOWBALLOONMSG = 100
};

/* VBoxTray's IPC header. */
typedef struct _VBOXTRAYIPCHEADER
{
    /** Message type. */
    ULONG ulMsg;
    /** Size of message body
     *  (without this header). */
    ULONG cbBody;
    /** User-supplied wParam. */
    ULONG wParam;
    /** User-supplied lParam. */
    ULONG lParam;
} VBOXTRAYIPCHEADER, *PVBOXTRAYIPCHEADER;

typedef struct _VBOXTRAYIPCMSG_SHOWBALLOONMSG
{
    /** Message content. */
    TCHAR    szContent[256];
    /** Message title. */
    TCHAR    szTitle[64];
    /** Message type. */
    ULONG    ulType;
    /** Flags; not used yet. */
    ULONG    ulFlags;
    /** Time to show the message (in msec). */
    ULONG    ulShowMS;
} VBOXTRAYIPCMSG_SHOWBALLOONMSG, *PVBOXTRAYIPCMSG_SHOWBALLOONMSG;

#endif /* !___VBOXTRAY_MSG_H */

