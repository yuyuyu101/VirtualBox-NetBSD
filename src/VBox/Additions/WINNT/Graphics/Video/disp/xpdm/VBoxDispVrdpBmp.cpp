/* $Id: VBoxDispVrdpBmp.cpp $ */

/** @file
 * VBox XPDM Display driver
 */

/*
 * Copyright (C) 2011 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#include "VBoxDisp.h"
#include <iprt/crc.h>
#include <VBox/RemoteDesktop/VRDEOrders.h>

/*
 * Cache has a fixed number of preallocated entries. Entries are linked in the MRU
 * list. The list contains both used and free entries. Free entries are at the end.
 * The most recently used entry is in the head.
 *
 * The purpose of the cache is to answer whether the bitmap was already encountered
 * before.
 *
 * No serialization because the code is executed under vboxHwBuffer* semaphore.
 */

static uint64_t surfHash (const SURFOBJ *pso, uint32_t cbLine)
{
    uint64_t u64CRC = RTCrc64Start ();

    uint32_t h   = pso->sizlBitmap.cy;
    uint8_t *pu8 = (uint8_t *)pso->pvScan0;

    while (h > 0)
    {
        u64CRC = RTCrc64Process (u64CRC, pu8, cbLine);
        pu8 += pso->lDelta;
        h--;
    }

    u64CRC = RTCrc64Finish (u64CRC);

    return u64CRC;
} /* Hash function end. */


static BOOL bcComputeHash (const SURFOBJ *pso, VRDPBCHASH *phash)
{
    uint32_t cbLine;

    int bytesPerPixel = format2BytesPerPixel (pso);

    if (bytesPerPixel == 0)
    {
        return FALSE;
    }

    phash->cx            = (uint16_t)pso->sizlBitmap.cx;
    phash->cy            = (uint16_t)pso->sizlBitmap.cy;
    phash->bytesPerPixel = bytesPerPixel;

    cbLine               = pso->sizlBitmap.cx * bytesPerPixel;
    phash->hash64        = surfHash (pso, cbLine);

    memset (phash->padding, 0, sizeof (phash->padding));

    return TRUE;
}

/* Meves an entry to the head of MRU list. */
static void bcMoveToHead (VRDPBC *pCache, VRDPBCENTRY *pEntry)
{
    if (pEntry->prev)
    {
        /* The entry is not yet in the head. Exclude from list. */
        pEntry->prev->next = pEntry->next;

        if (pEntry->next)
        {
            pEntry->next->prev = pEntry->prev;
        }
        else
        {
            pCache->tail = pEntry->prev;
        }

        /* Insert the entry at the head of MRU list. */
        pEntry->prev = NULL;
        pEntry->next = pCache->head;

        Assert(pCache->head);

        pCache->head->prev = pEntry;
        pCache->head = pEntry;
    }
}

/* Returns TRUE if the hash already presents in the cache.
 * Moves the found entry to the head of MRU list.
 */
static BOOL bcFindHash (VRDPBC *pCache, const VRDPBCHASH *phash)
{
    /* Search the MRU list. */
    VRDPBCENTRY *pEntry = pCache->head;

    while (pEntry && pEntry->fUsed)
    {
        if (memcmp (&pEntry->hash, phash, sizeof (VRDPBCHASH)) == 0)
        {
            /* Found the entry. Move it to the head of MRU list. */
            bcMoveToHead (pCache, pEntry);

            return TRUE;
        }

        pEntry = pEntry->next;
    }

    return FALSE;
}

/* Returns TRUE is a entry was also deleted to nake room for new entry. */
static BOOL bcInsertHash (VRDPBC *pCache, const VRDPBCHASH *phash, VRDPBCHASH *phashDeleted)
{
    BOOL bRc = FALSE;
    VRDPBCENTRY *pEntry;

    LOG(("insert hash cache %p, tail %p.", pCache, pCache->tail));

    /* Get the free entry to be used. Try tail, that should be */
    pEntry = pCache->tail;

    if (pEntry == NULL)
    {
        return bRc;
    }

    if (pEntry->fUsed)
    {
        /* The cache is full. Remove the tail. */
        memcpy (phashDeleted, &pEntry->hash, sizeof (VRDPBCHASH));
        bRc = TRUE;
    }

    bcMoveToHead (pCache, pEntry);

    memcpy (&pEntry->hash, phash, sizeof (VRDPBCHASH));
    pEntry->fUsed = TRUE;

    return bRc;
}

/* Find out whether the surface already in the cache.
 * Insert in the cache if not.
 */
int vrdpbmpCacheSurface(VRDPBC *pCache, const SURFOBJ *pso, VRDPBCHASH *phash, VRDPBCHASH *phashDeleted)
{
    int rc;

    VRDPBCHASH hash;

    BOOL bResult = bcComputeHash (pso, &hash);

    LOG(("vrdpbmpCacheSurface: compute hash %d.", bResult));
    if (!bResult)
    {
        WARN(("MEMBLT: vrdpbmpCacheSurface: could not compute hash."));
        return VRDPBMP_RC_NOT_CACHED;
    }

    bResult = bcFindHash (pCache, &hash);

    LOG(("vrdpbmpCacheSurface: find hash %d.", bResult));
    *phash = hash;

    if (bResult)
    {
        return VRDPBMP_RC_ALREADY_CACHED;
    }

    rc = VRDPBMP_RC_CACHED;

    bResult = bcInsertHash (pCache, &hash, phashDeleted);

    LOG(("vrdpbmpCacheSurface: insert hash %d.", bResult));
    if (bResult)
    {
        rc |= VRDPBMP_RC_F_DELETED;
    }

    return rc;
}

/* Setup the initial state of the cache. */
void vrdpbmpReset(VRDPBC *pCache)
{
    int i;

    Assert(sizeof (VRDPBCHASH) == sizeof (VRDEBITMAPHASH));

    /* Reinitialize the cache structure. */
    memset(pCache, 0, sizeof (VRDPBC));

    pCache->head = &pCache->aEntries[0];
    pCache->tail = &pCache->aEntries[RT_ELEMENTS(pCache->aEntries) - 1];

    for (i = 0; i < RT_ELEMENTS(pCache->aEntries); i++)
    {
        VRDPBCENTRY *pEntry = &pCache->aEntries[i];

        if (pEntry != pCache->tail)
        {
            pEntry->next = &pCache->aEntries[i + 1];
        }

        if (pEntry != pCache->head)
        {
            pEntry->prev = &pCache->aEntries[i - 1];
        }
    }
}
