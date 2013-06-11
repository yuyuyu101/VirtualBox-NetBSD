/*
 * IWineD3DResource Implementation
 *
 * Copyright 2002-2004 Jason Edmeades
 * Copyright 2003-2004 Raphael Junqueira
 * Copyright 2004 Christian Costa
 * Copyright 2005 Oliver Stieber
 * Copyright 2009 Henri Verbeet for CodeWeavers
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
#include "wined3d_private.h"

WINE_DEFAULT_DEBUG_CHANNEL(d3d);

HRESULT resource_init(IWineD3DResource *iface, WINED3DRESOURCETYPE resource_type,
        IWineD3DDeviceImpl *device, UINT size, DWORD usage, const struct wined3d_format_desc *format_desc,
        WINED3DPOOL pool, IUnknown *parent, const struct wined3d_parent_ops *parent_ops
#ifdef VBOX_WITH_WDDM
        , HANDLE *shared_handle
        , void *pvClientMem
#endif
        )
{
    struct IWineD3DResourceClass *resource = &((IWineD3DResourceImpl *)iface)->resource;

    resource->device = device;
    resource->parent = parent;
    resource->resourceType = resource_type;
    resource->ref = 1;
    resource->pool = pool;
    resource->format_desc = format_desc;
    resource->usage = usage;
    resource->size = size;
    resource->priority = 0;
    resource->parent_ops = parent_ops;
    list_init(&resource->privateData);

#ifdef VBOX_WITH_WDDM
    resource->sharerc_handle = 0;
    resource->sharerc_flags = 0;
    resource->sharerc_locks = 0;
    if (pool == WINED3DPOOL_SYSTEMMEM && pvClientMem)
    {
        resource->allocatedMemory = pvClientMem;
        resource->heapMemory = NULL;
    }
    else
#endif
    {
#ifdef VBOX_WITH_WDDM
        if (pool == WINED3DPOOL_DEFAULT && shared_handle)
        {
            resource->sharerc_handle = (DWORD)*shared_handle;
            resource->sharerc_flags = VBOXSHRC_F_SHARED;
            if (*shared_handle)
                resource->sharerc_flags |= VBOXSHRC_F_SHARED_OPENED;
        }
#endif
        if (size)
        {
            resource->heapMemory = HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, size + RESOURCE_ALIGNMENT);
            if (!resource->heapMemory)
            {
                ERR("Out of memory!\n");
                return WINED3DERR_OUTOFVIDEOMEMORY;
            }
        }
        else
        {
            resource->heapMemory = NULL;
        }
        resource->allocatedMemory = (BYTE *)(((ULONG_PTR)resource->heapMemory + (RESOURCE_ALIGNMENT - 1)) & ~(RESOURCE_ALIGNMENT - 1));
    }

#ifndef VBOX_WITH_WDDM
    /* Check that we have enough video ram left */
    if (pool == WINED3DPOOL_DEFAULT)
    {
        if (size > IWineD3DDevice_GetAvailableTextureMem((IWineD3DDevice *)device))
        {
            ERR("Out of adapter memory\n");
            HeapFree(GetProcessHeap(), 0, resource->heapMemory);
            return WINED3DERR_OUTOFVIDEOMEMORY;
        }
        WineD3DAdapterChangeGLRam(device, size);
    }
#endif

    device_resource_add(device, iface);

    return WINED3D_OK;
}

void resource_cleanup(IWineD3DResource *iface)
{
    IWineD3DResourceImpl *This = (IWineD3DResourceImpl *)iface;
    struct list *e1, *e2;
    PrivateData *data;
    HRESULT hr;

    TRACE("(%p) Cleaning up resource\n", This);
#ifndef VBOX_WITH_WDDM
    if (This->resource.pool == WINED3DPOOL_DEFAULT) {
        TRACE("Decrementing device memory pool by %u\n", This->resource.size);
        WineD3DAdapterChangeGLRam(This->resource.device, -This->resource.size);
    }
#endif

    LIST_FOR_EACH_SAFE(e1, e2, &This->resource.privateData) {
        data = LIST_ENTRY(e1, PrivateData, entry);
        hr = resource_free_private_data(iface, &data->tag);
        if(hr != WINED3D_OK) {
            ERR("Failed to free private data when destroying resource %p, hr = %08x\n", This, hr);
        }
    }

    HeapFree(GetProcessHeap(), 0, This->resource.heapMemory);
    This->resource.allocatedMemory = 0;
    This->resource.heapMemory = 0;

    if (This->resource.device) device_resource_released(This->resource.device, iface);
}

static PrivateData* resource_find_private_data(IWineD3DResourceImpl *This, REFGUID tag)
{
    PrivateData *data;
    struct list *entry;

    TRACE("Searching for private data %s\n", debugstr_guid(tag));
    LIST_FOR_EACH(entry, &This->resource.privateData)
    {
        data = LIST_ENTRY(entry, PrivateData, entry);
        if (IsEqualGUID(&data->tag, tag)) {
            TRACE("Found %p\n", data);
            return data;
        }
    }
    TRACE("Not found\n");
    return NULL;
}

HRESULT resource_set_private_data(IWineD3DResource *iface, REFGUID refguid,
        const void *pData, DWORD SizeOfData, DWORD Flags)
{
    IWineD3DResourceImpl *This = (IWineD3DResourceImpl *)iface;
    PrivateData *data;

    TRACE("(%p) : %s %p %d %d\n", This, debugstr_guid(refguid), pData, SizeOfData, Flags);
    resource_free_private_data(iface, refguid);

    data = HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, sizeof(*data));
    if (NULL == data) return E_OUTOFMEMORY;

    data->tag = *refguid;
    data->flags = Flags;

    if (Flags & WINED3DSPD_IUNKNOWN) {
        if(SizeOfData != sizeof(IUnknown *)) {
            WARN("IUnknown data with size %d, returning WINED3DERR_INVALIDCALL\n", SizeOfData);
            HeapFree(GetProcessHeap(), 0, data);
            return WINED3DERR_INVALIDCALL;
        }
        data->ptr.object = (LPUNKNOWN)pData;
        data->size = sizeof(LPUNKNOWN);
        IUnknown_AddRef(data->ptr.object);
    }
    else
    {
        data->ptr.data = HeapAlloc(GetProcessHeap(), 0, SizeOfData);
        if (NULL == data->ptr.data) {
            HeapFree(GetProcessHeap(), 0, data);
            return E_OUTOFMEMORY;
        }
        data->size = SizeOfData;
        memcpy(data->ptr.data, pData, SizeOfData);
    }
    list_add_tail(&This->resource.privateData, &data->entry);

    return WINED3D_OK;
}

HRESULT resource_get_private_data(IWineD3DResource *iface, REFGUID refguid, void *pData, DWORD *pSizeOfData)
{
    IWineD3DResourceImpl *This = (IWineD3DResourceImpl *)iface;
    PrivateData *data;

    TRACE("(%p) : %p %p %p\n", This, refguid, pData, pSizeOfData);
    data = resource_find_private_data(This, refguid);
    if (data == NULL) return WINED3DERR_NOTFOUND;

    if (*pSizeOfData < data->size) {
        *pSizeOfData = data->size;
        return WINED3DERR_MOREDATA;
    }

    if (data->flags & WINED3DSPD_IUNKNOWN) {
        *(LPUNKNOWN *)pData = data->ptr.object;
        if (((IWineD3DImpl *)This->resource.device->wined3d)->dxVersion != 7)
        {
            /* D3D8 and D3D9 addref the private data, DDraw does not. This can't be handled in
             * ddraw because it doesn't know if the pointer returned is an IUnknown * or just a
             * Blob
             */
            IUnknown_AddRef(data->ptr.object);
        }
    }
    else {
        memcpy(pData, data->ptr.data, data->size);
    }

    return WINED3D_OK;
}
HRESULT resource_free_private_data(IWineD3DResource *iface, REFGUID refguid)
{
    IWineD3DResourceImpl *This = (IWineD3DResourceImpl *)iface;
    PrivateData *data;

    TRACE("(%p) : %s\n", This, debugstr_guid(refguid));
    data = resource_find_private_data(This, refguid);
    if (data == NULL) return WINED3DERR_NOTFOUND;

    if (data->flags & WINED3DSPD_IUNKNOWN)
    {
        if (data->ptr.object != NULL)
            IUnknown_Release(data->ptr.object);
    } else {
        HeapFree(GetProcessHeap(), 0, data->ptr.data);
    }
    list_remove(&data->entry);

    HeapFree(GetProcessHeap(), 0, data);

    return WINED3D_OK;
}

DWORD resource_set_priority(IWineD3DResource *iface, DWORD PriorityNew)
{
    IWineD3DResourceImpl *This = (IWineD3DResourceImpl *)iface;
    DWORD PriorityOld = This->resource.priority;
    This->resource.priority = PriorityNew;
    TRACE("(%p) : new priority %d, returning old priority %d\n", This, PriorityNew, PriorityOld );
    return PriorityOld;
}

DWORD resource_get_priority(IWineD3DResource *iface)
{
    IWineD3DResourceImpl *This = (IWineD3DResourceImpl *)iface;
    TRACE("(%p) : returning %d\n", This, This->resource.priority );
    return This->resource.priority;
}

WINED3DRESOURCETYPE resource_get_type(IWineD3DResource *iface)
{
    IWineD3DResourceImpl *This = (IWineD3DResourceImpl *)iface;
    TRACE("(%p) : returning %d\n", This, This->resource.resourceType);
    return This->resource.resourceType;
}

HRESULT resource_get_parent(IWineD3DResource *iface, IUnknown **pParent)
{
    IWineD3DResourceImpl *This = (IWineD3DResourceImpl *)iface;
    IUnknown_AddRef(This->resource.parent);
    *pParent = This->resource.parent;
    return WINED3D_OK;
}

#ifdef VBOX_WITH_WDDM
HRESULT WINAPI IWineD3DResourceImpl_SetShRcState(IWineD3DResource *iface, VBOXWINEEX_SHRC_STATE enmState) {
    IWineD3DResourceImpl *This = (IWineD3DResourceImpl*)iface;
    if (!VBOXSHRC_IS_SHARED(This))
    {
        ERR("invalid arg");
        return E_INVALIDARG;
    }

    return WINED3D_OK;
}
#endif
