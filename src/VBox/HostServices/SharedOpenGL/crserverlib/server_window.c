/* Copyright (c) 2001, Stanford University
 * All rights reserved
 *
 * See the file LICENSE.txt for information on redistributing this software.
 */

#include "server.h"
#include "server_dispatch.h"
#include "cr_mem.h"
#include "cr_rand.h"
#include "cr_string.h"

GLint SERVER_DISPATCH_APIENTRY
crServerDispatchWindowCreate(const char *dpyName, GLint visBits)
{
    return crServerDispatchWindowCreateEx(dpyName, visBits, -1);
}


GLint
crServerDispatchWindowCreateEx(const char *dpyName, GLint visBits, GLint preloadWinID)
{
    CRMuralInfo *mural;
    GLint windowID = -1;
    GLint spuWindow;
    GLint dims[2];
    CRCreateInfo_t *pCreateInfo;

    if (cr_server.sharedWindows) {
        int pos, j;

        /* find empty position in my (curclient) windowList */
        for (pos = 0; pos < CR_MAX_WINDOWS; pos++) {
            if (cr_server.curClient->windowList[pos] == 0) {
                break;
            }
        }
        if (pos == CR_MAX_WINDOWS) {
            crWarning("Too many windows in crserver!");
            return -1;
        }

        /* Look if any other client has a window for this slot */
        for (j = 0; j < cr_server.numClients; j++) {
            if (cr_server.clients[j]->windowList[pos] != 0) {
                /* use that client's window */
                windowID = cr_server.clients[j]->windowList[pos];
                cr_server.curClient->windowList[pos] = windowID;
                crServerReturnValue( &windowID, sizeof(windowID) ); /* real return value */
                crDebug("CRServer: client %p sharing window %d",
                                cr_server.curClient, windowID);
                return windowID;
            }
        }
    }

    /*
     * Have first SPU make a new window.
     */
    spuWindow = cr_server.head_spu->dispatch_table.WindowCreate( dpyName, visBits );
    if (spuWindow < 0) {
        crServerReturnValue( &spuWindow, sizeof(spuWindow) );
        return spuWindow;
    }

    /* get initial window size */
    cr_server.head_spu->dispatch_table.GetChromiumParametervCR(GL_WINDOW_SIZE_CR, spuWindow, GL_INT, 2, dims);

    /*
     * Create a new mural for the new window.
     */
    mural = (CRMuralInfo *) crCalloc(sizeof(CRMuralInfo));
    if (mural) {
        CRMuralInfo *defaultMural = (CRMuralInfo *) crHashtableSearch(cr_server.muralTable, 0);
        CRASSERT(defaultMural);

        if (cr_server.fRootVrOn)
        {
        	VBOXVR_TEXTURE Tex = {0};
            int rc = CrVrScrCompositorInit(&mural->RootVrCompositor);
            if (!RT_SUCCESS(rc))
            {
                crWarning("CrVrScrCompositorInit failed, rc %d", rc);
                cr_server.head_spu->dispatch_table.WindowDestroy( spuWindow );
                crFree(mural);
                return -1;
            }

            Tex.width = dims[0];
            Tex.height = dims[1];
            Tex.target = GL_TEXTURE_2D;
            Tex.hwid = 0;

            CrVrScrCompositorEntryInit(&mural->RootVrCEntry, &Tex);
            mural->fRootVrOn = GL_TRUE;
        }

        mural->gX = 0;
        mural->gY = 0;
        mural->width = dims[0];
        mural->height = dims[1];

        mural->spuWindow = spuWindow;
        mural->screenId = 0;
        mural->bVisible = GL_FALSE;
        mural->bUseFBO = GL_FALSE;

        mural->cVisibleRects = 0;
        mural->pVisibleRects = NULL;
        mural->bReceivedRects = GL_FALSE;

        mural->pvOutputRedirectInstance = NULL;

        /* generate ID for this new window/mural (special-case for file conns) */
        if (cr_server.curClient && cr_server.curClient->conn->type == CR_FILE)
            windowID = spuWindow;
        else
            windowID = preloadWinID<0 ? crServerGenerateID(&cr_server.idsPool.freeWindowID) : preloadWinID;
        crHashtableAdd(cr_server.muralTable, windowID, mural);

        pCreateInfo = (CRCreateInfo_t *) crAlloc(sizeof(CRCreateInfo_t));
        pCreateInfo->pszDpyName = dpyName ? crStrdup(dpyName) : NULL;
        pCreateInfo->visualBits = visBits;
        crHashtableAdd(cr_server.pWindowCreateInfoTable, windowID, pCreateInfo);

        crServerSetupOutputRedirect(mural);
    }

    crDebug("CRServer: client %p created new window %d (SPU window %d)",
                    cr_server.curClient, windowID, spuWindow);

    if (windowID != -1 && !cr_server.bIsInLoadingState) {
        int pos;
        for (pos = 0; pos < CR_MAX_WINDOWS; pos++) {
            if (cr_server.curClient->windowList[pos] == 0) {
                cr_server.curClient->windowList[pos] = windowID;
                break;
            }
        }
    }

    crServerReturnValue( &windowID, sizeof(windowID) );
    return windowID;
}

static int crServerRemoveClientWindow(CRClient *pClient, GLint window)
{
    int pos;

    for (pos = 0; pos < CR_MAX_WINDOWS; ++pos)
    {
        if (pClient->windowList[pos] == window)
        {
            pClient->windowList[pos] = 0;
            return true;
        }
    }

    return false;
}

void SERVER_DISPATCH_APIENTRY
crServerDispatchWindowDestroy( GLint window )
{
    CRMuralInfo *mural;
    int32_t client;
    CRClientNode *pNode;
    int found=false;

    if (!window)
    {
        crWarning("Unexpected attempt to delete default mural, ignored!");
        return;
    }

    mural = (CRMuralInfo *) crHashtableSearch(cr_server.muralTable, window);
    if (!mural) {
         crWarning("CRServer: invalid window %d passed to WindowDestroy()", window);
         return;
    }

    if (mural->pvOutputRedirectInstance)
    {
        cr_server.outputRedirect.CROREnd(mural->pvOutputRedirectInstance);
        mural->pvOutputRedirectInstance = NULL;
    }

    if (cr_server.currentWindow == window)
    {
        cr_server.currentWindow = -1;
    }

    crServerRedirMuralFBO(mural, GL_FALSE);
    crServerDeleteMuralFBO(mural);

    crDebug("CRServer: Destroying window %d (spu window %d)", window, mural->spuWindow);
    cr_server.head_spu->dispatch_table.WindowDestroy( mural->spuWindow );

    if (cr_server.curClient)
    {
        if (cr_server.curClient->currentMural == mural)
        {
            cr_server.curClient->currentMural = NULL;
            cr_server.curClient->currentWindow = -1;
        }

        found = crServerRemoveClientWindow(cr_server.curClient, window);

        /*Same as with contexts, some apps destroy it not in a thread where it was created*/
        if (!found)
        {
            for (client=0; client<cr_server.numClients; ++client)
            {
                if (cr_server.clients[client]==cr_server.curClient)
                    continue;

                found = crServerRemoveClientWindow(cr_server.clients[client], window);

                if (found) break;
            }
        }

        if (!found)
        {
            pNode=cr_server.pCleanupClient;

            while (pNode && !found)
            {
                found = crServerRemoveClientWindow(pNode->pClient, window);
                pNode = pNode->next;
            }
        }

        CRASSERT(found);
    }

    /*Make sure this window isn't active in other clients*/
    for (client=0; client<cr_server.numClients; ++client)
    {
        if (cr_server.clients[client]->currentMural == mural)
        {
            cr_server.clients[client]->currentMural = NULL;
            cr_server.clients[client]->currentWindow = -1;
        }
    }

    pNode=cr_server.pCleanupClient;
    while (pNode)
    {
        if (pNode->pClient->currentMural == mural)
        {
            pNode->pClient->currentMural = NULL;
            pNode->pClient->currentWindow = -1;
        }
        pNode = pNode->next;
    }

    crHashtableDelete(cr_server.pWindowCreateInfoTable, window, crServerCreateInfoDeleteCB);

    if (mural->pVisibleRects)
    {
        crFree(mural->pVisibleRects);
    }

    if (mural->fRootVrOn)
        CrVrScrCompositorTerm(&mural->RootVrCompositor);

    crHashtableDelete(cr_server.muralTable, window, crFree);
}

static void crServerVBoxRootVrTranslateForMural(CRMuralInfo *mural)
{
    int32_t dx = cr_server.RootVrCurPoint.x - mural->gX;
    int32_t dy = cr_server.RootVrCurPoint.y - mural->gY;

    VBoxVrListTranslate(&cr_server.RootVr, dx, dy);
}

int crServerMuralSynchRootVr(CRMuralInfo *mural, uint32_t *pcRects, const RTRECT **ppRects)
{
    int rc;
    RTRECT Rect;

    if (mural->bReceivedRects)
    {
    	*pcRects = mural->cVisibleRects;
    	*ppRects = (const RTRECT *)mural->pVisibleRects;
    }
    else
    {
    	*pcRects = 1;
    	Rect.xLeft = 0;
    	Rect.yTop = 0;
    	Rect.xRight = mural->width;
    	Rect.yBottom = mural->height;
    	*ppRects = &Rect;

    }

    rc = CrVrScrCompositorEntryRegionsSet(&mural->RootVrCompositor, &mural->RootVrCEntry, NULL, *pcRects, *ppRects, NULL);
    if (!RT_SUCCESS(rc))
    {
        crWarning("CrVrScrCompositorEntryRegionsSet failed, rc %d", rc);
        return rc;
    }

    crServerVBoxRootVrTranslateForMural(mural);
    rc = CrVrScrCompositorEntryListIntersect(&mural->RootVrCompositor, &mural->RootVrCEntry, &cr_server.RootVr, NULL);
    if (!RT_SUCCESS(rc))
    {
        crWarning("CrVrScrCompositorEntryRegionsSet failed, rc %d", rc);
        return rc;
    }

    rc = CrVrScrCompositorEntryRegionsGet(&mural->RootVrCompositor, &mural->RootVrCEntry, pcRects, NULL, ppRects);
    if (!RT_SUCCESS(rc))
    {
        crWarning("CrVrScrCompositorEntryRegionsGet failed, rc %d", rc);
        return rc;
    }

    return VINF_SUCCESS;
}

void SERVER_DISPATCH_APIENTRY
crServerDispatchWindowSize( GLint window, GLint width, GLint height )
{
    CRMuralInfo *mural;

    /*  crDebug("CRServer: Window %d size %d x %d", window, width, height);*/
    mural = (CRMuralInfo *) crHashtableSearch(cr_server.muralTable, window);
    if (!mural) {
#if EXTRA_WARN
         crWarning("CRServer: invalid window %d passed to WindowSize()", window);
#endif
         return;
    }

    mural->width = width;
    mural->height = height;

    if (cr_server.curClient && cr_server.curClient->currentMural == mural)
    {
        crStateGetCurrent()->buffer.width = mural->width;
        crStateGetCurrent()->buffer.height = mural->height;
    }

    if (mural->fRootVrOn)
    {
    	RTRECT Rect;
        VBOXVR_TEXTURE Tex;
        int rc = VINF_SUCCESS;
        Tex.width = width;
        Tex.height = height;
        Tex.target = GL_TEXTURE_2D;
        Tex.hwid = 0;

        rc = CrVrScrCompositorEntryRemove(&mural->RootVrCompositor, &mural->RootVrCEntry);
        if (!RT_SUCCESS(rc))
        {
            crWarning("CrVrScrCompositorEntryRemove failed, rc %d", rc);
            return;
        }

        CrVrScrCompositorEntryInit(&mural->RootVrCEntry, &Tex);
        /* initially set regions to all visible since this is what some guest assume
         * and will not post any more visible regions command */
        Rect.xLeft = 0;
        Rect.xRight = width;
        Rect.yTop = 0;
        Rect.yBottom = height;
        rc = CrVrScrCompositorEntryRegionsSet(&mural->RootVrCompositor, &mural->RootVrCEntry, NULL, 1, &Rect, NULL);
        if (!RT_SUCCESS(rc))
        {
            crWarning("CrVrScrCompositorEntryRegionsSet failed, rc %d", rc);
            return;
        }

        crServerVBoxRootVrTranslateForMural(mural);
        rc = CrVrScrCompositorEntryListIntersect(&mural->RootVrCompositor, &mural->RootVrCEntry, &cr_server.RootVr, NULL);
        if (!RT_SUCCESS(rc))
        {
            crWarning("CrVrScrCompositorEntryRegionsSet failed, rc %d", rc);
            return;
        }
    }

    crServerCheckMuralGeometry(mural);

    cr_server.head_spu->dispatch_table.WindowSize(mural->spuWindow, width, height);

    /* Work-around Intel driver bug */
    CRASSERT(!cr_server.curClient
            || !cr_server.curClient->currentMural
            || cr_server.curClient->currentMural == mural);
    if (cr_server.curClient && cr_server.curClient->currentMural == mural)
    {
        CRContextInfo * ctxInfo = cr_server.currentCtxInfo;
        CRASSERT(ctxInfo);
        crServerDispatchMakeCurrent(window, 0, ctxInfo->CreateInfo.externalID);
    }
}


void SERVER_DISPATCH_APIENTRY
crServerDispatchWindowPosition( GLint window, GLint x, GLint y )
{
    CRMuralInfo *mural = (CRMuralInfo *) crHashtableSearch(cr_server.muralTable, window);
    /*  crDebug("CRServer: Window %d pos %d, %d", window, x, y);*/
    if (!mural) {
#if EXTRA_WARN
         crWarning("CRServer: invalid window %d passed to WindowPosition()", window);
#endif
         return;
    }
    mural->gX = x;
    mural->gY = y;

    crServerCheckMuralGeometry(mural);
}

void SERVER_DISPATCH_APIENTRY
crServerDispatchWindowVisibleRegion( GLint window, GLint cRects, GLint *pRects )
{
    CRMuralInfo *mural = (CRMuralInfo *) crHashtableSearch(cr_server.muralTable, window);
    if (!mural) {
#if EXTRA_WARN
         crWarning("CRServer: invalid window %d passed to WindowVisibleRegion()", window);
#endif
         return;
    }

    if (mural->pVisibleRects)
    {
        crFree(mural->pVisibleRects);
        mural->pVisibleRects = NULL;
    }

    mural->cVisibleRects = cRects;
    mural->bReceivedRects = GL_TRUE;
    if (cRects)
    {
        mural->pVisibleRects = (GLint*) crAlloc(4*sizeof(GLint)*cRects);
        if (!mural->pVisibleRects)
        {
            crError("Out of memory in crServerDispatchWindowVisibleRegion");
        }
        crMemcpy(mural->pVisibleRects, pRects, 4*sizeof(GLint)*cRects);
    }

    if (mural->fRootVrOn)
    {
        int rc = CrVrScrCompositorEntryRegionsSet(&mural->RootVrCompositor, &mural->RootVrCEntry, NULL, cRects, (const RTRECT *)pRects, NULL);
        if (!RT_SUCCESS(rc))
        {
            crWarning("CrVrScrCompositorEntryRegionsSet failed, rc %d", rc);
            return;
        }

        crServerVBoxRootVrTranslateForMural(mural);
        rc = CrVrScrCompositorEntryListIntersect(&mural->RootVrCompositor, &mural->RootVrCEntry, &cr_server.RootVr, NULL);
        if (!RT_SUCCESS(rc))
        {
            crWarning("CrVrScrCompositorEntryRegionsSet failed, rc %d", rc);
            return;
        }

        rc = CrVrScrCompositorEntryRegionsGet(&mural->RootVrCompositor, &mural->RootVrCEntry, &cRects, NULL, (const RTRECT**)&pRects);
        if (!RT_SUCCESS(rc))
        {
            crWarning("CrVrScrCompositorEntryRegionsGet failed, rc %d", rc);
            return;
        }
    }

    cr_server.head_spu->dispatch_table.WindowVisibleRegion(mural->spuWindow, cRects, pRects);

    if (mural->pvOutputRedirectInstance)
    {
    	cRects = mural->cVisibleRects;
    	pRects = mural->pVisibleRects;
        /* @todo the code assumes that RTRECT == four GLInts. */
        cr_server.outputRedirect.CRORVisibleRegion(mural->pvOutputRedirectInstance,
                                                   cRects, (RTRECT *)pRects);
    }
}



void SERVER_DISPATCH_APIENTRY
crServerDispatchWindowShow( GLint window, GLint state )
{
    CRMuralInfo *mural = (CRMuralInfo *) crHashtableSearch(cr_server.muralTable, window);
    if (!mural) {
#if EXTRA_WARN
         crWarning("CRServer: invalid window %d passed to WindowShow()", window);
#endif
         return;
    }

    if (!mural->bUseFBO)
    {
        cr_server.head_spu->dispatch_table.WindowShow(mural->spuWindow, state);
    }

    mural->bVisible = state;
}


GLint
crServerSPUWindowID(GLint serverWindow)
{
    CRMuralInfo *mural = (CRMuralInfo *) crHashtableSearch(cr_server.muralTable, serverWindow);
    if (!mural) {
#if EXTRA_WARN
         crWarning("CRServer: invalid window %d passed to crServerSPUWindowID()",
                             serverWindow);
#endif
         return -1;
    }
    return mural->spuWindow;
}
