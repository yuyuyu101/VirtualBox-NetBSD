/*
 *IDirect3DSwapChain9 implementation
 *
 *Copyright 2002-2003 Jason Edmeades
 *Copyright 2002-2003 Raphael Junqueira
 *Copyright 2005 Oliver Stieber
 *Copyright 2007-2008 Stefan Dösinger for CodeWeavers
 *
 *This library is free software; you can redistribute it and/or
 *modify it under the terms of the GNU Lesser General Public
 *License as published by the Free Software Foundation; either
 *version 2.1 of the License, or (at your option) any later version.
 *
 *This library is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *Lesser General Public License for more details.
 *
 *You should have received a copy of the GNU Lesser General Public
 *License along with this library; if not, write to the Free Software
 *Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA
 */

/*
 * Oracle LGPL Disclaimer: For the avoidance of doubt, except that if any license choice
 * other than GPL or LGPL is available it will apply instead, Oracle elects to use only
 * the Lesser General Public License version 2.1 (LGPLv2) at this time for any software where
 * a choice of LGPL license versions is made available with the language indicating
 * that LGPLv2 or any later version may be used, or where a choice of which version
 * of the LGPL is applied is otherwise unspecified.
 */

#include "config.h"
#include "wined3d_private.h"

WINE_DEFAULT_DEBUG_CHANNEL(d3d);

/* IDirect3DSwapChain IUnknown parts follow: */
HRESULT WINAPI IWineD3DBaseSwapChainImpl_QueryInterface(IWineD3DSwapChain *iface, REFIID riid, void **object)
{
    TRACE("iface %p, riid %s, object %p.\n", iface, debugstr_guid(riid), object);

    if (IsEqualGUID(riid, &IID_IWineD3DSwapChain)
            || IsEqualGUID(riid, &IID_IWineD3DBase)
            || IsEqualGUID(riid, &IID_IUnknown))
    {
        IUnknown_AddRef(iface);
        *object = iface;
        return S_OK;
    }

    WARN("%s not implemented, returning E_NOINTERFACE.\n", debugstr_guid(riid));

    *object = NULL;
    return E_NOINTERFACE;
}

ULONG WINAPI IWineD3DBaseSwapChainImpl_AddRef(IWineD3DSwapChain *iface) {
    IWineD3DSwapChainImpl *This = (IWineD3DSwapChainImpl *)iface;
    DWORD refCount = InterlockedIncrement(&This->ref);
    TRACE("(%p) : AddRef increasing from %d\n", This, refCount - 1);
    return refCount;
}

ULONG WINAPI IWineD3DBaseSwapChainImpl_Release(IWineD3DSwapChain *iface) {
    IWineD3DSwapChainImpl *This = (IWineD3DSwapChainImpl *)iface;
    DWORD refCount;
    refCount = InterlockedDecrement(&This->ref);
    TRACE("(%p) : ReleaseRef to %d\n", This, refCount);
    if (refCount == 0) {
        IWineD3DSwapChain_Destroy(iface);
    }
    return refCount;
}

HRESULT WINAPI IWineD3DBaseSwapChainImpl_GetParent(IWineD3DSwapChain *iface, IUnknown ** ppParent){
    IWineD3DSwapChainImpl *This = (IWineD3DSwapChainImpl *)iface;
    *ppParent = This->parent;
    IUnknown_AddRef(*ppParent);
    TRACE("(%p) returning %p\n", This , *ppParent);
    return WINED3D_OK;
}

HRESULT WINAPI IWineD3DBaseSwapChainImpl_GetFrontBufferData(IWineD3DSwapChain *iface, IWineD3DSurface *pDestSurface) {
    IWineD3DSwapChainImpl *This = (IWineD3DSwapChainImpl *)iface;
    POINT start;

    TRACE("(%p) : iface(%p) pDestSurface(%p)\n", This, iface, pDestSurface);

    start.x = 0;
    start.y = 0;

#ifndef VBOX_WITH_WDDM
    if (This->presentParms.Windowed) {
        MapWindowPoints(This->win_handle, NULL, &start, 1);
    }
#endif

    IWineD3DSurface_BltFast(pDestSurface, start.x, start.y, This->frontBuffer, NULL, 0);
    return WINED3D_OK;
}

HRESULT WINAPI IWineD3DBaseSwapChainImpl_GetBackBuffer(IWineD3DSwapChain *iface, UINT iBackBuffer, WINED3DBACKBUFFER_TYPE Type, IWineD3DSurface **ppBackBuffer) {

    IWineD3DSwapChainImpl *This = (IWineD3DSwapChainImpl *)iface;

#ifdef VBOX_WITH_WDDM
    if (iBackBuffer == ~0UL)
    {
        *ppBackBuffer = This->frontBuffer;
        if(*ppBackBuffer) IWineD3DSurface_AddRef(*ppBackBuffer);
        return WINED3D_OK;
    }
#endif

    if (iBackBuffer > This->presentParms.BackBufferCount - 1) {
        TRACE("Back buffer count out of range\n");
        /* Native d3d9 doesn't set NULL here, just as wine's d3d9. But set it
         * here in wined3d to avoid problems in other libs
         */
        *ppBackBuffer = NULL;
        return WINED3DERR_INVALIDCALL;
    }

    /* Return invalid if there is no backbuffer array, otherwise it will crash when ddraw is
     * used (there This->backBuffer is always NULL). We need this because this function has
     * to be called from IWineD3DStateBlockImpl_InitStartupStateBlock to get the default
     * scissorrect dimensions. */
    if( !This->backBuffer ) {
        *ppBackBuffer = NULL;
        return WINED3DERR_INVALIDCALL;
    }

    *ppBackBuffer = This->backBuffer[iBackBuffer];
    TRACE("(%p) : BackBuf %d Type %d  returning %p\n", This, iBackBuffer, Type, *ppBackBuffer);

    /* Note inc ref on returned surface */
    if(*ppBackBuffer) IWineD3DSurface_AddRef(*ppBackBuffer);
    return WINED3D_OK;

}

HRESULT WINAPI IWineD3DBaseSwapChainImpl_GetRasterStatus(IWineD3DSwapChain *iface, WINED3DRASTER_STATUS *pRasterStatus) {
    static BOOL warned;
    pRasterStatus->InVBlank = TRUE;
    pRasterStatus->ScanLine = 0;
    /* No openGL equivalent */
    if (!warned)
    {
        FIXME("iface %p, raster_status %p stub!\n", iface, pRasterStatus);
        warned = TRUE;
    }
    return WINED3D_OK;
}

HRESULT WINAPI IWineD3DBaseSwapChainImpl_GetDisplayMode(IWineD3DSwapChain *iface, WINED3DDISPLAYMODE*pMode) {
    IWineD3DSwapChainImpl *This = (IWineD3DSwapChainImpl *)iface;
    HRESULT hr;

    TRACE("(%p)->(%p): Calling GetAdapterDisplayMode\n", This, pMode);
    hr = IWineD3D_GetAdapterDisplayMode(This->device->wined3d, This->device->adapter->ordinal, pMode);

    TRACE("(%p) : returning w(%d) h(%d) rr(%d) fmt(%u,%s)\n", This, pMode->Width, pMode->Height, pMode->RefreshRate,
          pMode->Format, debug_d3dformat(pMode->Format));
    return hr;
}

HRESULT WINAPI IWineD3DBaseSwapChainImpl_GetDevice(IWineD3DSwapChain *iface, IWineD3DDevice**ppDevice) {
    IWineD3DSwapChainImpl *This = (IWineD3DSwapChainImpl *)iface;

    *ppDevice = (IWineD3DDevice *)This->device;

    /* Note  Calling this method will increase the internal reference count
    on the IDirect3DDevice9 interface. */
    IWineD3DDevice_AddRef(*ppDevice);
    TRACE("(%p) : returning %p\n", This, *ppDevice);
    return WINED3D_OK;
}

HRESULT WINAPI IWineD3DBaseSwapChainImpl_GetPresentParameters(IWineD3DSwapChain *iface, WINED3DPRESENT_PARAMETERS *pPresentationParameters) {
    IWineD3DSwapChainImpl *This = (IWineD3DSwapChainImpl *)iface;
    TRACE("(%p)\n", This);

    *pPresentationParameters = This->presentParms;

    return WINED3D_OK;
}

HRESULT WINAPI IWineD3DBaseSwapChainImpl_SetGammaRamp(IWineD3DSwapChain *iface, DWORD Flags, CONST WINED3DGAMMARAMP *pRamp){

    IWineD3DSwapChainImpl *This = (IWineD3DSwapChainImpl *)iface;
    HDC hDC;
    HWND hWnd;
#ifndef VBOX_WITH_WDDM
    hWnd = This->device_window;
#else
    hWnd = This->win_handle;
#endif
    TRACE("(%p) : pRamp@%p flags(%d)\n", This, pRamp, Flags);
    hDC = GetDC(hWnd);
    SetDeviceGammaRamp(hDC, (LPVOID)pRamp);
    ReleaseDC(hWnd, hDC);
    return WINED3D_OK;

}

HRESULT WINAPI IWineD3DBaseSwapChainImpl_GetGammaRamp(IWineD3DSwapChain *iface, WINED3DGAMMARAMP *pRamp){

    IWineD3DSwapChainImpl *This = (IWineD3DSwapChainImpl *)iface;
    HDC hDC;
    HWND hWnd;
#ifndef VBOX_WITH_WDDM
    hWnd = This->device_window;
#else
    hWnd = This->win_handle;
#endif
    TRACE("(%p) : pRamp@%p\n", This, pRamp);
    hDC = GetDC(hWnd);
    GetDeviceGammaRamp(hDC, pRamp);
    ReleaseDC(hWnd, hDC);
    return WINED3D_OK;

}
