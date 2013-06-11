/** @file
 * Main - Network Interfaces.
 */

/*
 * Copyright (C) 2008-2012 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#ifndef ___netif_h
#define ___netif_h

#include <iprt/cdefs.h>
#include <iprt/types.h>
#include <iprt/net.h>
/** @todo r=bird: The inlined code below that drags in asm.h here. I doubt
 *        speed is very important here, so move it into a .cpp file, please. */
#include <iprt/asm.h>

#ifndef RT_OS_WINDOWS
# include <arpa/inet.h>
# include <stdio.h>
#endif /* !RT_OS_WINDOWS */

#define VBOXNET_IPV4ADDR_DEFAULT      0x0138A8C0  /* 192.168.56.1 */
#define VBOXNET_IPV4MASK_DEFAULT      "255.255.255.0"

#define VBOXNET_MAX_SHORT_NAME 50

#if 1
/**
 * Encapsulation type.
 */
typedef enum NETIFTYPE
{
    NETIF_T_UNKNOWN,
    NETIF_T_ETHERNET,
    NETIF_T_PPP,
    NETIF_T_SLIP
} NETIFTYPE;

/**
 * Current state of the interface.
 */
typedef enum NETIFSTATUS
{
    NETIF_S_UNKNOWN,
    NETIF_S_UP,
    NETIF_S_DOWN
} NETIFSTATUS;

/**
 * Host Network Interface Information.
 */
typedef struct NETIFINFO
{
    NETIFINFO     *pNext;
    RTNETADDRIPV4  IPAddress;
    RTNETADDRIPV4  IPNetMask;
    RTNETADDRIPV6  IPv6Address;
    RTNETADDRIPV6  IPv6NetMask;
    BOOL           bDhcpEnabled;
    BOOL           bIsDefault;
    RTMAC          MACAddress;
    NETIFTYPE      enmMediumType;
    NETIFSTATUS    enmStatus;
    uint32_t       uSpeedMbits;
    RTUUID         Uuid;
    char           szShortName[VBOXNET_MAX_SHORT_NAME];
    char           szName[1];
} NETIFINFO;

/** Pointer to a network interface info. */
typedef NETIFINFO *PNETIFINFO;
/** Pointer to a const network interface info. */
typedef NETIFINFO const *PCNETIFINFO;
#endif

int NetIfList(std::list <ComObjPtr<HostNetworkInterface> > &list);
int NetIfEnableStaticIpConfig(VirtualBox *pVbox, HostNetworkInterface * pIf, ULONG aOldIp, ULONG aNewIp, ULONG aMask);
int NetIfEnableStaticIpConfigV6(VirtualBox *pVbox, HostNetworkInterface * pIf, IN_BSTR aOldIPV6Address, IN_BSTR aIPV6Address, ULONG aIPV6MaskPrefixLength);
int NetIfEnableDynamicIpConfig(VirtualBox *pVbox, HostNetworkInterface * pIf);
int NetIfCreateHostOnlyNetworkInterface (VirtualBox *pVbox, IHostNetworkInterface **aHostNetworkInterface, IProgress **aProgress, const char *pcszName = NULL);
int NetIfRemoveHostOnlyNetworkInterface (VirtualBox *pVbox, IN_GUID aId, IProgress **aProgress);
int NetIfGetConfig(HostNetworkInterface * pIf, NETIFINFO *);
int NetIfGetConfigByName(PNETIFINFO pInfo);
int NetIfDhcpRediscover(VirtualBox *pVbox, HostNetworkInterface * pIf);
int NetIfAdpCtlOut(const char * pcszName, const char * pcszCmd, char *pszBuffer, size_t cBufSize);

DECLINLINE(Bstr) composeIPv6Address(PRTNETADDRIPV6 aAddrPtr)
{
    char szTmp[8*5] = "";

    if (aAddrPtr->s.Lo || aAddrPtr->s.Hi)
        RTStrPrintf(szTmp, sizeof(szTmp),
                    "%02x%02x:%02x%02x:%02x%02x:%02x%02x:"
                    "%02x%02x:%02x%02x:%02x%02x:%02x%02x",
                    aAddrPtr->au8[0], aAddrPtr->au8[1],
                    aAddrPtr->au8[2], aAddrPtr->au8[3],
                    aAddrPtr->au8[4], aAddrPtr->au8[5],
                    aAddrPtr->au8[6], aAddrPtr->au8[7],
                    aAddrPtr->au8[8], aAddrPtr->au8[9],
                    aAddrPtr->au8[10], aAddrPtr->au8[11],
                    aAddrPtr->au8[12], aAddrPtr->au8[13],
                    aAddrPtr->au8[14], aAddrPtr->au8[15]);
    return Bstr(szTmp);
}

DECLINLINE(ULONG) composeIPv6PrefixLenghFromAddress(PRTNETADDRIPV6 aAddrPtr)
{
    int res = ASMBitFirstClear(aAddrPtr, sizeof(RTNETADDRIPV6)*8);
    return res != -1 ? res : 128;
}

DECLINLINE(int) prefixLength2IPv6Address(ULONG cPrefix, PRTNETADDRIPV6 aAddrPtr)
{
    if(cPrefix > 128)
        return VERR_INVALID_PARAMETER;
    if(!aAddrPtr)
        return VERR_INVALID_PARAMETER;

    memset(aAddrPtr, 0, sizeof(RTNETADDRIPV6));

    ASMBitSetRange(aAddrPtr, 0, cPrefix);

    return VINF_SUCCESS;
}

DECLINLINE(Bstr) composeHardwareAddress(PRTMAC aMacPtr)
{
    char szTmp[6*3];

    RTStrPrintf(szTmp, sizeof(szTmp),
                "%02x:%02x:%02x:%02x:%02x:%02x",
                aMacPtr->au8[0], aMacPtr->au8[1],
                aMacPtr->au8[2], aMacPtr->au8[3],
                aMacPtr->au8[4], aMacPtr->au8[5]);
    return Bstr(szTmp);
}

DECLINLINE(Bstr) getDefaultIPv4Address(Bstr bstrIfName)
{
    /* Get the index from the name */
    Utf8Str strTmp = bstrIfName;
    const char *pszIfName = strTmp.c_str();
    int iInstance = 0, iPos = strcspn(pszIfName, "0123456789");
    if (pszIfName[iPos])
        iInstance = RTStrToUInt32(pszIfName + iPos);

    in_addr tmp;
#if defined(RT_OS_WINDOWS)
    tmp.S_un.S_addr = VBOXNET_IPV4ADDR_DEFAULT + (iInstance << 16);
#else
    tmp.s_addr = VBOXNET_IPV4ADDR_DEFAULT + (iInstance << 16);
#endif
    char *addr = inet_ntoa(tmp);
    return Bstr(addr);
}

#endif  /* !___netif_h */
/* vi: set tabstop=4 shiftwidth=4 expandtab: */
