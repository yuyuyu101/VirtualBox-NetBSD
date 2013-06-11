/** @file
 * helpers - Guest Additions Service helper functions
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

#include <malloc.h>
#include <windows.h>

#include <iprt/string.h>
#include <VBox/Log.h>
#include <VBox/VBoxGuestLib.h>

#include <VBoxGuestInternal.h>

#include "VBoxHelpers.h"
#include "resource.h"


int hlpReportStatus(VBoxGuestFacilityStatus statusCurrent)
{
    int rc = VbglR3ReportAdditionsStatus(VBoxGuestFacilityType_VBoxTrayClient,
                                         statusCurrent,
                                         0 /* Flags */);
    if (RT_FAILURE(rc))
        Log(("VBoxTray: Could not report VBoxTray status \"%ld\", rc=%Rrc\n", statusCurrent, rc));
    return rc;
}

/**
 * Attempt to force Windows to reload the cursor image by attaching to the
 * thread of the window currently under the mouse, hiding the cursor and
 * showing it again.  This could fail to work in any number of ways (no
 * window under the cursor, the cursor has moved to a different window while
 * we are processing), but we just accept this, as the cursor will be reloaded
 * at some point anyway.
 */
void hlpReloadCursor(void)
{
    POINT mousePos;
    HWND hWin;
    DWORD hThread, hCurrentThread;

    GetCursorPos(&mousePos);
    hWin = WindowFromPoint(mousePos);
    if (hWin)
    {
        hThread = GetWindowThreadProcessId(hWin, NULL);
        hCurrentThread = GetCurrentThreadId();
        if (hCurrentThread != hThread)
            AttachThreadInput(hCurrentThread, hThread, TRUE);
    }
    ShowCursor(false);
    ShowCursor(true);
    if (hWin && (hCurrentThread != hThread))
        AttachThreadInput(hCurrentThread, hThread, FALSE);
}

static unsigned hlpNextAdjacentRectXP(RECTL *paRects, unsigned nRects, unsigned uRect)
{
    unsigned i;
    for (i = 0; i < nRects; i++)
    {
        if (paRects[uRect].right == paRects[i].left)
            return i;
    }
    return ~0;
}

static unsigned hlpNextAdjacentRectXN(RECTL *paRects, unsigned nRects, unsigned uRect)
{
    unsigned i;
    for (i = 0; i < nRects; i++)
    {
        if (paRects[uRect].left == paRects[i].right)
            return i;
    }
    return ~0;
}

static unsigned hlpNextAdjacentRectYP(RECTL *paRects, unsigned nRects, unsigned uRect)
{
    unsigned i;
    for (i = 0; i < nRects; i++)
    {
        if (paRects[uRect].bottom == paRects[i].top)
            return i;
    }
    return ~0;
}

static unsigned hlpNextAdjacentRectYN(RECTL *paRects, unsigned nRects, unsigned uRect)
{
    unsigned i;
    for (i = 0; i < nRects; i++)
    {
        if (paRects[uRect].top == paRects[i].bottom)
            return i;
    }
    return ~0;
}

void hlpResizeRect(RECTL *paRects, unsigned nRects, unsigned uPrimary,
                   unsigned uResized, int iNewWidth, int iNewHeight)
{
    DDCLOG(("nRects %d, iPrimary %d, iResized %d, NewWidth %d, NewHeight %d\n", nRects, uPrimary, uResized, iNewWidth, iNewHeight));

    RECTL *paNewRects = (RECTL *)alloca (sizeof (RECTL) * nRects);
    memcpy (paNewRects, paRects, sizeof (RECTL) * nRects);
    paNewRects[uResized].right += iNewWidth - (paNewRects[uResized].right - paNewRects[uResized].left);
    paNewRects[uResized].bottom += iNewHeight - (paNewRects[uResized].bottom - paNewRects[uResized].top);

    /* Verify all pairs of originally adjacent rectangles for all 4 directions.
     * If the pair has a "good" delta (that is the first rectangle intersects the second)
     * at a direction and the second rectangle is not primary one (which can not be moved),
     * move the second rectangle to make it adjacent to the first one.
     */

    /* X positive. */
    unsigned iRect;
    for (iRect = 0; iRect < nRects; iRect++)
    {
        /* Find the next adjacent original rect in x positive direction. */
        unsigned iNextRect = hlpNextAdjacentRectXP(paRects, nRects, iRect);
        DDCLOG(("next %d -> %d\n", iRect, iNextRect));

        if (iNextRect == ~0 || iNextRect == uPrimary)
        {
            continue;
        }

        /* Check whether there is an X intersection between these adjacent rects in the new rectangles
         * and fix the intersection if delta is "good".
         */
        int delta = paNewRects[iRect].right - paNewRects[iNextRect].left;

        if (delta != 0)
        {
            DDCLOG(("XP intersection right %d left %d, diff %d\n",
                     paNewRects[iRect].right, paNewRects[iNextRect].left,
                     delta));

            paNewRects[iNextRect].left += delta;
            paNewRects[iNextRect].right += delta;
        }
    }

    /* X negative. */
    for (iRect = 0; iRect < nRects; iRect++)
    {
        /* Find the next adjacent original rect in x negative direction. */
        unsigned iNextRect = hlpNextAdjacentRectXN(paRects, nRects, iRect);
        DDCLOG(("next %d -> %d\n", iRect, iNextRect));

        if (iNextRect == ~0 || iNextRect == uPrimary)
        {
            continue;
        }

        /* Check whether there is an X intersection between these adjacent rects in the new rectangles
         * and fix the intersection if delta is "good".
         */
        int delta = paNewRects[iRect].left - paNewRects[iNextRect].right;

        if (delta != 0)
        {
            DDCLOG(("XN intersection left %d right %d, diff %d\n",
                     paNewRects[iRect].left, paNewRects[iNextRect].right,
                     delta));

            paNewRects[iNextRect].left += delta;
            paNewRects[iNextRect].right += delta;
        }
    }

    /* Y positive (in the computer sense, top->down). */
    for (iRect = 0; iRect < nRects; iRect++)
    {
        /* Find the next adjacent original rect in y positive direction. */
        unsigned iNextRect = hlpNextAdjacentRectYP(paRects, nRects, iRect);
        DDCLOG(("next %d -> %d\n", iRect, iNextRect));

        if (iNextRect == ~0 || iNextRect == uPrimary)
        {
            continue;
        }

        /* Check whether there is an Y intersection between these adjacent rects in the new rectangles
         * and fix the intersection if delta is "good".
         */
        int delta = paNewRects[iRect].bottom - paNewRects[iNextRect].top;

        if (delta != 0)
        {
            DDCLOG(("YP intersection bottom %d top %d, diff %d\n",
                     paNewRects[iRect].bottom, paNewRects[iNextRect].top,
                     delta));

            paNewRects[iNextRect].top += delta;
            paNewRects[iNextRect].bottom += delta;
        }
    }

    /* Y negative (in the computer sense, down->top). */
    for (iRect = 0; iRect < nRects; iRect++)
    {
        /* Find the next adjacent original rect in x negative direction. */
        unsigned iNextRect = hlpNextAdjacentRectYN(paRects, nRects, iRect);
        DDCLOG(("next %d -> %d\n", iRect, iNextRect));

        if (iNextRect == ~0 || iNextRect == uPrimary)
        {
            continue;
        }

        /* Check whether there is an Y intersection between these adjacent rects in the new rectangles
         * and fix the intersection if delta is "good".
         */
        int delta = paNewRects[iRect].top - paNewRects[iNextRect].bottom;

        if (delta != 0)
        {
            DDCLOG(("YN intersection top %d bottom %d, diff %d\n",
                     paNewRects[iRect].top, paNewRects[iNextRect].bottom,
                     delta));

            paNewRects[iNextRect].top += delta;
            paNewRects[iNextRect].bottom += delta;
        }
    }

    memcpy (paRects, paNewRects, sizeof (RECTL) * nRects);
    return;
}

int hlpShowBalloonTip(HINSTANCE hInst, HWND hWnd, UINT uID,
                      const char *pszMsg, const char *pszTitle,
                      UINT uTimeout, DWORD dwInfoFlags)
{
    NOTIFYICONDATA niData;
    ZeroMemory(&niData, sizeof(NOTIFYICONDATA));
    niData.cbSize = sizeof(NOTIFYICONDATA);
    niData.uFlags = NIF_INFO; /* Display a balloon notification. */
    niData.hWnd = hWnd;
    niData.uID = uID;
    /* If not timeout set, set it to 5sec. */
    if (uTimeout == 0)
        uTimeout = 5000;
    niData.uTimeout = uTimeout;
    /* If no info flag (info, warning, error) set,
     * set it to info by default. */
    if (dwInfoFlags == 0)
        dwInfoFlags = NIIF_INFO;
    niData.dwInfoFlags = dwInfoFlags;

    /* Do we want to have */

    /* Get running OS version. */
    OSVERSIONINFO osInfo;
    osInfo.dwOSVersionInfoSize = sizeof(OSVERSIONINFO);
    if (FALSE == GetVersionEx(&osInfo))
        return RTErrConvertFromWin32(GetLastError());

    /* Is the current OS supported (at least WinXP) for displaying
     * our own icon and do we actually *want* to display our own stuff? */
    if (   osInfo.dwMajorVersion >= 5
        && (dwInfoFlags & NIIF_INFO))
    {
        /* Load (or retrieve handle of) the app's icon. */
        HICON hIcon = LoadIcon(hInst, MAKEINTRESOURCE(IDI_VIRTUALBOX));
        if (hIcon)
            niData.dwInfoFlags = NIIF_USER; /* Use an own notification icon. */

        if (   osInfo.dwMajorVersion == 5
            && osInfo.dwMinorVersion == 1) /* WinXP. */
        {
            /* Use an own icon instead of the default one. */
            niData.hIcon = hIcon;
        }
        else if (osInfo.dwMajorVersion == 6) /* Vista and up. */
        {
            /* Use an own icon instead of the default one. */
            niData.dwInfoFlags |= NIIF_LARGE_ICON; /* Use a  large icon if available! */
            niData.hIcon        = hIcon;
            niData.hBalloonIcon = hIcon;
        }
    }
    else
    {
        /* This might be a warning, error message or a to old OS. Use the
         * standard icons provided by Windows (if any). */
    }

    strcpy(niData.szInfo, pszMsg ? pszMsg : "-");
    strcpy(niData.szInfoTitle, pszTitle ? pszTitle : "Information");

    if (!Shell_NotifyIcon(NIM_MODIFY, &niData))
    {
        DWORD dwErr = GetLastError();
        return RTErrConvertFromWin32(dwErr);
    }
    return VINF_SUCCESS;
}

