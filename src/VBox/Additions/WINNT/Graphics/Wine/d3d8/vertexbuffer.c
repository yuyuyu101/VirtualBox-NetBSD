/*
 * IDirect3DVertexBuffer8 implementation
 *
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
#include "d3d8_private.h"

WINE_DEFAULT_DEBUG_CHANNEL(d3d8);

/* IDirect3DVertexBuffer8 IUnknown parts follow: */
static HRESULT WINAPI IDirect3DVertexBuffer8Impl_QueryInterface(LPDIRECT3DVERTEXBUFFER8 iface, REFIID riid, LPVOID *ppobj) {
    IDirect3DVertexBuffer8Impl *This = (IDirect3DVertexBuffer8Impl *)iface;

    TRACE("iface %p, riid %s, object %p.\n", iface, debugstr_guid(riid), ppobj);

    if (IsEqualGUID(riid, &IID_IUnknown)
        || IsEqualGUID(riid, &IID_IDirect3DResource8)
        || IsEqualGUID(riid, &IID_IDirect3DVertexBuffer8)) {
        IUnknown_AddRef(iface);
        *ppobj = This;
        return S_OK;
    }

    WARN("(%p)->(%s,%p),not found\n", This, debugstr_guid(riid), ppobj);

    *ppobj = NULL;
    return E_NOINTERFACE;
}

static ULONG WINAPI IDirect3DVertexBuffer8Impl_AddRef(LPDIRECT3DVERTEXBUFFER8 iface) {
    IDirect3DVertexBuffer8Impl *This = (IDirect3DVertexBuffer8Impl *)iface;
    ULONG ref = InterlockedIncrement(&This->ref);

    TRACE("%p increasing refcount to %u.\n", iface, ref);

    if (ref == 1)
    {
        IDirect3DDevice8_AddRef(This->parentDevice);
        wined3d_mutex_lock();
        IWineD3DBuffer_AddRef(This->wineD3DVertexBuffer);
        wined3d_mutex_unlock();
    }

    return ref;
}

static ULONG WINAPI IDirect3DVertexBuffer8Impl_Release(LPDIRECT3DVERTEXBUFFER8 iface) {
    IDirect3DVertexBuffer8Impl *This = (IDirect3DVertexBuffer8Impl *)iface;
    ULONG ref = InterlockedDecrement(&This->ref);

    TRACE("%p decreasing refcount to %u.\n", iface, ref);

    if (ref == 0) {
        IDirect3DDevice8 *parentDevice = This->parentDevice;

        wined3d_mutex_lock();
        IWineD3DBuffer_Release(This->wineD3DVertexBuffer);
        wined3d_mutex_unlock();

        /* Release the device last, as it may cause the device to be destroyed. */
        IDirect3DDevice8_Release(parentDevice);
    }

    return ref;
}

/* IDirect3DVertexBuffer8 IDirect3DResource8 Interface follow: */
static HRESULT WINAPI IDirect3DVertexBuffer8Impl_GetDevice(IDirect3DVertexBuffer8 *iface, IDirect3DDevice8 **device)
{
    IDirect3DVertexBuffer8Impl *This = (IDirect3DVertexBuffer8Impl *)iface;

    TRACE("iface %p, device %p.\n", iface, device);

    *device = (IDirect3DDevice8 *)This->parentDevice;
    IDirect3DDevice8_AddRef(*device);

    TRACE("Returning device %p.\n", *device);

    return D3D_OK;
}

static HRESULT WINAPI IDirect3DVertexBuffer8Impl_SetPrivateData(LPDIRECT3DVERTEXBUFFER8 iface, REFGUID refguid, CONST void *pData, DWORD SizeOfData, DWORD Flags) {
    IDirect3DVertexBuffer8Impl *This = (IDirect3DVertexBuffer8Impl *)iface;
    HRESULT hr;

    TRACE("iface %p, guid %s, data %p, data_size %u, flags %#x.\n",
            iface, debugstr_guid(refguid), pData, SizeOfData, Flags);

    wined3d_mutex_lock();
    hr = IWineD3DBuffer_SetPrivateData(This->wineD3DVertexBuffer, refguid, pData, SizeOfData, Flags);
    wined3d_mutex_unlock();

    return hr;
}

static HRESULT WINAPI IDirect3DVertexBuffer8Impl_GetPrivateData(LPDIRECT3DVERTEXBUFFER8 iface, REFGUID refguid, void *pData, DWORD *pSizeOfData) {
    IDirect3DVertexBuffer8Impl *This = (IDirect3DVertexBuffer8Impl *)iface;
    HRESULT hr;

    TRACE("iface %p, guid %s, data %p, data_size %p.\n",
            iface, debugstr_guid(refguid), pData, pSizeOfData);

    wined3d_mutex_lock();
    hr = IWineD3DBuffer_GetPrivateData(This->wineD3DVertexBuffer, refguid, pData, pSizeOfData);
    wined3d_mutex_unlock();

    return hr;
}

static HRESULT WINAPI IDirect3DVertexBuffer8Impl_FreePrivateData(LPDIRECT3DVERTEXBUFFER8 iface, REFGUID refguid) {
    IDirect3DVertexBuffer8Impl *This = (IDirect3DVertexBuffer8Impl *)iface;
    HRESULT hr;

    TRACE("iface %p, guid %s.\n", iface, debugstr_guid(refguid));

    wined3d_mutex_lock();
    hr = IWineD3DBuffer_FreePrivateData(This->wineD3DVertexBuffer, refguid);
    wined3d_mutex_unlock();

    return hr;
}

static DWORD WINAPI IDirect3DVertexBuffer8Impl_SetPriority(LPDIRECT3DVERTEXBUFFER8 iface, DWORD PriorityNew) {
    IDirect3DVertexBuffer8Impl *This = (IDirect3DVertexBuffer8Impl *)iface;
    DWORD ret;

    TRACE("iface %p, priority %u.\n", iface, PriorityNew);

    wined3d_mutex_lock();
    ret = IWineD3DBuffer_SetPriority(This->wineD3DVertexBuffer, PriorityNew);
    wined3d_mutex_unlock();

    return ret;
}

static DWORD WINAPI IDirect3DVertexBuffer8Impl_GetPriority(LPDIRECT3DVERTEXBUFFER8 iface) {
    IDirect3DVertexBuffer8Impl *This = (IDirect3DVertexBuffer8Impl *)iface;
    DWORD ret;

    TRACE("iface %p.\n", iface);

    wined3d_mutex_lock();
    ret = IWineD3DBuffer_GetPriority(This->wineD3DVertexBuffer);
    wined3d_mutex_unlock();

    return ret;
}

static void WINAPI IDirect3DVertexBuffer8Impl_PreLoad(LPDIRECT3DVERTEXBUFFER8 iface) {
    IDirect3DVertexBuffer8Impl *This = (IDirect3DVertexBuffer8Impl *)iface;

    TRACE("iface %p.\n", iface);

    wined3d_mutex_lock();
    IWineD3DBuffer_PreLoad(This->wineD3DVertexBuffer);
    wined3d_mutex_unlock();
}

static D3DRESOURCETYPE WINAPI IDirect3DVertexBuffer8Impl_GetType(IDirect3DVertexBuffer8 *iface)
{
    TRACE("iface %p.\n", iface);

    return D3DRTYPE_VERTEXBUFFER;
}

/* IDirect3DVertexBuffer8 Interface follow: */
static HRESULT WINAPI IDirect3DVertexBuffer8Impl_Lock(LPDIRECT3DVERTEXBUFFER8 iface, UINT OffsetToLock, UINT SizeToLock, BYTE **ppbData, DWORD Flags) {
    IDirect3DVertexBuffer8Impl *This = (IDirect3DVertexBuffer8Impl *)iface;
    HRESULT hr;

    TRACE("iface %p, offset %u, size %u, data %p, flags %#x.\n",
            iface, OffsetToLock, SizeToLock, ppbData, Flags);

    wined3d_mutex_lock();
    hr = IWineD3DBuffer_Map(This->wineD3DVertexBuffer, OffsetToLock, SizeToLock, ppbData, Flags);
    wined3d_mutex_unlock();

    return hr;
}

static HRESULT WINAPI IDirect3DVertexBuffer8Impl_Unlock(LPDIRECT3DVERTEXBUFFER8 iface) {
    IDirect3DVertexBuffer8Impl *This = (IDirect3DVertexBuffer8Impl *)iface;
    HRESULT hr;

    TRACE("iface %p.\n", iface);

    wined3d_mutex_lock();
    hr = IWineD3DBuffer_Unmap(This->wineD3DVertexBuffer);
    wined3d_mutex_unlock();

    return hr;
}

static HRESULT WINAPI IDirect3DVertexBuffer8Impl_GetDesc(LPDIRECT3DVERTEXBUFFER8 iface, D3DVERTEXBUFFER_DESC *pDesc) {
    IDirect3DVertexBuffer8Impl *This = (IDirect3DVertexBuffer8Impl *)iface;
    HRESULT hr;
    WINED3DBUFFER_DESC desc;

    TRACE("iface %p, desc %p.\n", iface, pDesc);

    wined3d_mutex_lock();
    hr = IWineD3DBuffer_GetDesc(This->wineD3DVertexBuffer, &desc);
    wined3d_mutex_unlock();

    if (SUCCEEDED(hr)) {
        pDesc->Type = D3DRTYPE_VERTEXBUFFER;
        pDesc->Usage = desc.Usage;
        pDesc->Pool = desc.Pool;
        pDesc->Size = desc.Size;
        pDesc->FVF = This->fvf;
        pDesc->Format = D3DFMT_VERTEXDATA;
    }

    return hr;
}

static const IDirect3DVertexBuffer8Vtbl Direct3DVertexBuffer8_Vtbl =
{
    /* IUnknown */
    IDirect3DVertexBuffer8Impl_QueryInterface,
    IDirect3DVertexBuffer8Impl_AddRef,
    IDirect3DVertexBuffer8Impl_Release,
    /* IDirect3DResource8 */
    IDirect3DVertexBuffer8Impl_GetDevice,
    IDirect3DVertexBuffer8Impl_SetPrivateData,
    IDirect3DVertexBuffer8Impl_GetPrivateData,
    IDirect3DVertexBuffer8Impl_FreePrivateData,
    IDirect3DVertexBuffer8Impl_SetPriority,
    IDirect3DVertexBuffer8Impl_GetPriority,
    IDirect3DVertexBuffer8Impl_PreLoad,
    IDirect3DVertexBuffer8Impl_GetType,
    /* IDirect3DVertexBuffer8 */
    IDirect3DVertexBuffer8Impl_Lock,
    IDirect3DVertexBuffer8Impl_Unlock,
    IDirect3DVertexBuffer8Impl_GetDesc
};

static void STDMETHODCALLTYPE d3d8_vertexbuffer_wined3d_object_destroyed(void *parent)
{
    HeapFree(GetProcessHeap(), 0, parent);
}

static const struct wined3d_parent_ops d3d8_vertexbuffer_wined3d_parent_ops =
{
    d3d8_vertexbuffer_wined3d_object_destroyed,
};

HRESULT vertexbuffer_init(IDirect3DVertexBuffer8Impl *buffer, IDirect3DDevice8Impl *device,
        UINT size, DWORD usage, DWORD fvf, D3DPOOL pool)
{
    HRESULT hr;

    buffer->lpVtbl = &Direct3DVertexBuffer8_Vtbl;
    buffer->ref = 1;
    buffer->fvf = fvf;

    wined3d_mutex_lock();
    hr = IWineD3DDevice_CreateVertexBuffer(device->WineD3DDevice, size,
            usage & WINED3DUSAGE_MASK, (WINED3DPOOL)pool, &buffer->wineD3DVertexBuffer,
            (IUnknown *)buffer, &d3d8_vertexbuffer_wined3d_parent_ops);
    wined3d_mutex_unlock();
    if (FAILED(hr))
    {
        WARN("Failed to create wined3d buffer, hr %#x.\n", hr);
        return hr;
    }

    buffer->parentDevice = (IDirect3DDevice8 *)device;
    IUnknown_AddRef(buffer->parentDevice);

    return D3D_OK;
}
