/*
 * Copyright (C) 2009 Robert Shearman
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

#ifndef _WS2DEF_
#define _WS2DEF_

/* FIXME: #include <inaddr.h> */

#ifdef USE_WS_PREFIX
#define WS(x)    WS_##x
#else
#define WS(x)    x
#endif

#ifndef __CSADDR_DEFINED__
#define __CSADDR_DEFINED__

typedef struct _SOCKET_ADDRESS {
        LPSOCKADDR      lpSockaddr;
        INT             iSockaddrLength;
} SOCKET_ADDRESS, *PSOCKET_ADDRESS, *LPSOCKET_ADDRESS;

typedef struct _CSADDR_INFO {
        SOCKET_ADDRESS  LocalAddr;
        SOCKET_ADDRESS  RemoteAddr;
        INT             iSocketType;
        INT             iProtocol;
} CSADDR_INFO, *PCSADDR_INFO, *LPCSADDR_INFO;
#endif

#ifdef USE_WS_PREFIX
#define WS__SS_MAXSIZE 128
#define WS__SS_ALIGNSIZE (sizeof(__int64))
#define WS__SS_PAD1SIZE (WS__SS_ALIGNSIZE - sizeof(short))
#define WS__SS_PAD2SIZE (WS__SS_MAXSIZE - 2 * WS__SS_ALIGNSIZE)
#else
#define _SS_MAXSIZE 128
#define _SS_ALIGNSIZE (sizeof(__int64))
#define _SS_PAD1SIZE (_SS_ALIGNSIZE - sizeof(short))
#define _SS_PAD2SIZE (_SS_MAXSIZE - 2 * _SS_ALIGNSIZE)
#endif

typedef struct WS(sockaddr_storage) {
        short ss_family;
        char __ss_pad1[WS(_SS_PAD1SIZE)];
        __int64 DECLSPEC_ALIGN(8) __ss_align;
        char __ss_pad2[WS(_SS_PAD2SIZE)];
} SOCKADDR_STORAGE, *PSOCKADDR_STORAGE, *LPSOCKADDR_STORAGE;

/*socket address list */
typedef struct _SOCKET_ADDRESS_LIST {
        INT             iAddressCount;
        SOCKET_ADDRESS  Address[1];
} SOCKET_ADDRESS_LIST, *LPSOCKET_ADDRESS_LIST;

#endif /* _WS2DEF_ */
