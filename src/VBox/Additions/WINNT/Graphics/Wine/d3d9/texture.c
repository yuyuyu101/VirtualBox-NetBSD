/*
 * IDirect3DTexture9 implementation
 *
 * Copyright 2002-2005 Jason Edmeades
 * Copyright 2002-2005 Raphael Junqueira
 * Copyright 2005 Oliver Stieber
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

#include "config.h"
#include "d3d9_private.h"

WINE_DEFAULT_DEBUG_CHANNEL(d3d9);

/* IDirect3DTexture9 IUnknown parts follow: */
static HRESULT WINAPI IDirect3DTexture9Impl_QueryInterface(LPDIRECT3DTEXTURE9 iface, REFIID riid, LPVOID* ppobj) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;

    TRACE("iface %p, riid %s, object %p.\n", iface, debugstr_guid(riid), ppobj);

    if (IsEqualGUID(riid, &IID_IUnknown)
        || IsEqualGUID(riid, &IID_IDirect3DResource9)
        || IsEqualGUID(riid, &IID_IDirect3DBaseTexture9)
        || IsEqualGUID(riid, &IID_IDirect3DTexture9)) {
        IDirect3DTexture9_AddRef(iface);
        *ppobj = This;
        return S_OK;
    }

    WARN("(%p)->(%s,%p) not found\n", This, debugstr_guid(riid), ppobj);
    *ppobj = NULL;
    return E_NOINTERFACE;
}

static ULONG WINAPI IDirect3DTexture9Impl_AddRef(LPDIRECT3DTEXTURE9 iface) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;
    ULONG ref = InterlockedIncrement(&This->ref);

    TRACE("%p increasing refcount to %u.\n", iface, ref);

    if (ref == 1)
    {
        IDirect3DDevice9Ex_AddRef(This->parentDevice);
        wined3d_mutex_lock();
        IWineD3DTexture_AddRef(This->wineD3DTexture);
        wined3d_mutex_unlock();
    }

    return ref;
}

static ULONG WINAPI IDirect3DTexture9Impl_Release(LPDIRECT3DTEXTURE9 iface) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;
    ULONG ref = InterlockedDecrement(&This->ref);

    TRACE("%p decreasing refcount to %u.\n", iface, ref);

    if (ref == 0) {
        IDirect3DDevice9Ex *parentDevice = This->parentDevice;

        wined3d_mutex_lock();
        IWineD3DTexture_Release(This->wineD3DTexture);
        wined3d_mutex_unlock();

        /* Release the device last, as it may cause the device to be destroyed. */
        IDirect3DDevice9Ex_Release(parentDevice);
    }
    return ref;
}

/* IDirect3DTexture9 IDirect3DResource9 Interface follow: */
static HRESULT WINAPI IDirect3DTexture9Impl_GetDevice(IDirect3DTexture9 *iface, IDirect3DDevice9 **device)
{
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;

    TRACE("iface %p, device %p.\n", iface, device);

    *device = (IDirect3DDevice9 *)This->parentDevice;
    IDirect3DDevice9_AddRef(*device);

    TRACE("Returning device %p.\n", *device);

    return D3D_OK;
}

static HRESULT WINAPI IDirect3DTexture9Impl_SetPrivateData(LPDIRECT3DTEXTURE9 iface, REFGUID refguid, CONST void* pData, DWORD SizeOfData, DWORD Flags) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;
    HRESULT hr;

    TRACE("iface %p, guid %s, data %p, data_size %u, flags %#x.\n",
            iface, debugstr_guid(refguid), pData, SizeOfData, Flags);

    wined3d_mutex_lock();
    hr = IWineD3DTexture_SetPrivateData(This->wineD3DTexture, refguid, pData, SizeOfData, Flags);
    wined3d_mutex_unlock();

    return hr;
}

static HRESULT WINAPI IDirect3DTexture9Impl_GetPrivateData(LPDIRECT3DTEXTURE9 iface, REFGUID refguid, void* pData, DWORD* pSizeOfData) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;
    HRESULT hr;

    TRACE("iface %p, guid %s, data %p, data_size %p.\n",
            iface, debugstr_guid(refguid), pData, pSizeOfData);

    wined3d_mutex_lock();
    hr = IWineD3DTexture_GetPrivateData(This->wineD3DTexture, refguid, pData, pSizeOfData);
    wined3d_mutex_unlock();

    return hr;
}

static HRESULT WINAPI IDirect3DTexture9Impl_FreePrivateData(LPDIRECT3DTEXTURE9 iface, REFGUID refguid) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;
    HRESULT hr;

    TRACE("iface %p, guid %s.\n", iface, debugstr_guid(refguid));

    wined3d_mutex_lock();
    hr = IWineD3DTexture_FreePrivateData(This->wineD3DTexture, refguid);
    wined3d_mutex_unlock();

    return hr;
}

static DWORD WINAPI IDirect3DTexture9Impl_SetPriority(LPDIRECT3DTEXTURE9 iface, DWORD PriorityNew) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;
    DWORD ret;

    TRACE("iface %p, priority %u.\n", iface, PriorityNew);

    wined3d_mutex_lock();
    ret = IWineD3DTexture_SetPriority(This->wineD3DTexture, PriorityNew);
    wined3d_mutex_unlock();

    return ret;
}

static DWORD WINAPI IDirect3DTexture9Impl_GetPriority(LPDIRECT3DTEXTURE9 iface) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;
    DWORD ret;

    TRACE("iface %p.\n", iface);

    wined3d_mutex_lock();
    ret = IWineD3DTexture_GetPriority(This->wineD3DTexture);
    wined3d_mutex_unlock();

    return ret;
}

static void WINAPI IDirect3DTexture9Impl_PreLoad(LPDIRECT3DTEXTURE9 iface) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;

    TRACE("iface %p.\n", iface);

    wined3d_mutex_lock();
    IWineD3DTexture_PreLoad(This->wineD3DTexture);
    wined3d_mutex_unlock();
}

static D3DRESOURCETYPE WINAPI IDirect3DTexture9Impl_GetType(LPDIRECT3DTEXTURE9 iface) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;
    HRESULT ret;

    TRACE("iface %p.\n", iface);

    wined3d_mutex_lock();
    ret = IWineD3DTexture_GetType(This->wineD3DTexture);
    wined3d_mutex_unlock();

    return ret;
}

/* IDirect3DTexture9 IDirect3DBaseTexture9 Interface follow: */
static DWORD WINAPI IDirect3DTexture9Impl_SetLOD(LPDIRECT3DTEXTURE9 iface, DWORD LODNew) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;
    DWORD ret;

    TRACE("iface %p, lod %u.\n", iface, LODNew);

    wined3d_mutex_lock();
    ret = IWineD3DTexture_SetLOD(This->wineD3DTexture, LODNew);
    wined3d_mutex_unlock();

    return ret;
}

static DWORD WINAPI IDirect3DTexture9Impl_GetLOD(LPDIRECT3DTEXTURE9 iface) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;
    DWORD ret;

    TRACE("iface %p.\n", iface);

    wined3d_mutex_lock();
    ret = IWineD3DTexture_GetLOD(This->wineD3DTexture);
    wined3d_mutex_unlock();

    return ret;
}

static DWORD WINAPI IDirect3DTexture9Impl_GetLevelCount(LPDIRECT3DTEXTURE9 iface) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;
    DWORD ret;

    TRACE("iface %p.\n", iface);

    wined3d_mutex_lock();
    ret = IWineD3DTexture_GetLevelCount(This->wineD3DTexture);
    wined3d_mutex_unlock();

    return ret;
}

static HRESULT WINAPI IDirect3DTexture9Impl_SetAutoGenFilterType(LPDIRECT3DTEXTURE9 iface, D3DTEXTUREFILTERTYPE FilterType) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;
    HRESULT hr;

    TRACE("iface %p, filter_type %#x.\n", iface, FilterType);

    wined3d_mutex_lock();
    hr = IWineD3DTexture_SetAutoGenFilterType(This->wineD3DTexture, (WINED3DTEXTUREFILTERTYPE) FilterType);
    wined3d_mutex_unlock();

    return hr;
}

static D3DTEXTUREFILTERTYPE WINAPI IDirect3DTexture9Impl_GetAutoGenFilterType(LPDIRECT3DTEXTURE9 iface) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;
    D3DTEXTUREFILTERTYPE ret;

    TRACE("iface %p.\n", iface);

    wined3d_mutex_lock();
    ret = (D3DTEXTUREFILTERTYPE) IWineD3DTexture_GetAutoGenFilterType(This->wineD3DTexture);
    wined3d_mutex_unlock();

    return ret;
}

static void WINAPI IDirect3DTexture9Impl_GenerateMipSubLevels(LPDIRECT3DTEXTURE9 iface) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;

    TRACE("iface %p.\n", iface);

    wined3d_mutex_lock();
    IWineD3DTexture_GenerateMipSubLevels(This->wineD3DTexture);
    wined3d_mutex_unlock();
}

/* IDirect3DTexture9 Interface follow: */
static HRESULT WINAPI IDirect3DTexture9Impl_GetLevelDesc(LPDIRECT3DTEXTURE9 iface, UINT Level, D3DSURFACE_DESC* pDesc) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;
    WINED3DSURFACE_DESC wined3ddesc;
    HRESULT hr;

    TRACE("iface %p, level %u, desc %p.\n", iface, Level, pDesc);

    wined3d_mutex_lock();
    hr = IWineD3DTexture_GetLevelDesc(This->wineD3DTexture, Level, &wined3ddesc);
    wined3d_mutex_unlock();

    if (SUCCEEDED(hr))
    {
        pDesc->Format = d3dformat_from_wined3dformat(wined3ddesc.format);
        pDesc->Type = wined3ddesc.resource_type;
        pDesc->Usage = wined3ddesc.usage;
        pDesc->Pool = wined3ddesc.pool;
        pDesc->MultiSampleType = wined3ddesc.multisample_type;
        pDesc->MultiSampleQuality = wined3ddesc.multisample_quality;
        pDesc->Width = wined3ddesc.width;
        pDesc->Height = wined3ddesc.height;
    }

    return hr;
}

static HRESULT WINAPI IDirect3DTexture9Impl_GetSurfaceLevel(LPDIRECT3DTEXTURE9 iface, UINT Level, IDirect3DSurface9** ppSurfaceLevel) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;
    HRESULT hrc = D3D_OK;
    IWineD3DSurface *mySurface = NULL;

    TRACE("iface %p, level %u, surface %p.\n", iface, Level, ppSurfaceLevel);

    wined3d_mutex_lock();
    hrc = IWineD3DTexture_GetSurfaceLevel(This->wineD3DTexture, Level, &mySurface);
    if (hrc == D3D_OK && NULL != ppSurfaceLevel) {
       IWineD3DSurface_GetParent(mySurface, (IUnknown **)ppSurfaceLevel);
       IWineD3DSurface_Release(mySurface);
    }
    wined3d_mutex_unlock();

    return hrc;
}

static HRESULT WINAPI IDirect3DTexture9Impl_LockRect(LPDIRECT3DTEXTURE9 iface, UINT Level, D3DLOCKED_RECT* pLockedRect, CONST RECT* pRect, DWORD Flags) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;
    HRESULT hr;

    TRACE("iface %p, level %u, locked_rect %p, rect %p, flags %#x.\n",
            iface, Level, pLockedRect, pRect, Flags);

    wined3d_mutex_lock();
    hr = IWineD3DTexture_LockRect(This->wineD3DTexture, Level, (WINED3DLOCKED_RECT *) pLockedRect, pRect, Flags);
    wined3d_mutex_unlock();

    return hr;
}

static HRESULT WINAPI IDirect3DTexture9Impl_UnlockRect(LPDIRECT3DTEXTURE9 iface, UINT Level) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;
    HRESULT hr;

    TRACE("iface %p, level %u.\n", iface, Level);

    wined3d_mutex_lock();
    hr = IWineD3DTexture_UnlockRect(This->wineD3DTexture, Level);
    wined3d_mutex_unlock();

    return hr;
}

static HRESULT WINAPI IDirect3DTexture9Impl_AddDirtyRect(LPDIRECT3DTEXTURE9 iface, CONST RECT* pDirtyRect) {
    IDirect3DTexture9Impl *This = (IDirect3DTexture9Impl *)iface;
    HRESULT hr;

    TRACE("iface %p, dirty_rect %p.\n", iface, pDirtyRect);

    wined3d_mutex_lock();
    hr = IWineD3DTexture_AddDirtyRect(This->wineD3DTexture, pDirtyRect);
    wined3d_mutex_unlock();

    return hr;
}

static const IDirect3DTexture9Vtbl Direct3DTexture9_Vtbl =
{
    /* IUnknown */
    IDirect3DTexture9Impl_QueryInterface,
    IDirect3DTexture9Impl_AddRef,
    IDirect3DTexture9Impl_Release,
     /* IDirect3DResource9 */
    IDirect3DTexture9Impl_GetDevice,
    IDirect3DTexture9Impl_SetPrivateData,
    IDirect3DTexture9Impl_GetPrivateData,
    IDirect3DTexture9Impl_FreePrivateData,
    IDirect3DTexture9Impl_SetPriority,
    IDirect3DTexture9Impl_GetPriority,
    IDirect3DTexture9Impl_PreLoad,
    IDirect3DTexture9Impl_GetType,
    /* IDirect3dBaseTexture9 */
    IDirect3DTexture9Impl_SetLOD,
    IDirect3DTexture9Impl_GetLOD,
    IDirect3DTexture9Impl_GetLevelCount,
    IDirect3DTexture9Impl_SetAutoGenFilterType,
    IDirect3DTexture9Impl_GetAutoGenFilterType,
    IDirect3DTexture9Impl_GenerateMipSubLevels,
    /* IDirect3DTexture9 */
    IDirect3DTexture9Impl_GetLevelDesc,
    IDirect3DTexture9Impl_GetSurfaceLevel,
    IDirect3DTexture9Impl_LockRect,
    IDirect3DTexture9Impl_UnlockRect,
    IDirect3DTexture9Impl_AddDirtyRect
};

static void STDMETHODCALLTYPE d3d9_texture_wined3d_object_destroyed(void *parent)
{
    HeapFree(GetProcessHeap(), 0, parent);
}

static const struct wined3d_parent_ops d3d9_texture_wined3d_parent_ops =
{
    d3d9_texture_wined3d_object_destroyed,
};

HRESULT texture_init(IDirect3DTexture9Impl *texture, IDirect3DDevice9Impl *device,
        UINT width, UINT height, UINT levels, DWORD usage, D3DFORMAT format, D3DPOOL pool
#ifdef VBOX_WITH_WDDM
        , HANDLE *shared_handle
        , void **pavClientMem
#endif
        )
{
    HRESULT hr;

    texture->lpVtbl = &Direct3DTexture9_Vtbl;
    texture->ref = 1;

    wined3d_mutex_lock();
#ifdef VBOX_WITH_WDDM
    hr = IWineD3DDevice_CreateTexture(device->WineD3DDevice, width, height, levels,
            usage & WINED3DUSAGE_MASK, wined3dformat_from_d3dformat(format), pool,
            &texture->wineD3DTexture, (IUnknown *)texture, &d3d9_texture_wined3d_parent_ops
            , shared_handle
            , pavClientMem);
#else
    hr = IWineD3DDevice_CreateTexture(device->WineD3DDevice, width, height, levels,
            usage & WINED3DUSAGE_MASK, wined3dformat_from_d3dformat(format), pool,
            &texture->wineD3DTexture, (IUnknown *)texture, &d3d9_texture_wined3d_parent_ops);
#endif

    wined3d_mutex_unlock();
    if (FAILED(hr))
    {
        WARN("Failed to create wined3d texture, hr %#x.\n", hr);
        return hr;
    }

    texture->parentDevice = (IDirect3DDevice9Ex *)device;
    IDirect3DDevice9Ex_AddRef(texture->parentDevice);

    return D3D_OK;
}
