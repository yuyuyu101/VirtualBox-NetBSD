/*
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
 *
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
#include "wine/port.h"

#include "wined3d_private.h"

WINE_DEFAULT_DEBUG_CHANNEL(d3d);

/* IUnknown methods */

static HRESULT STDMETHODCALLTYPE rendertarget_view_QueryInterface(IWineD3DRendertargetView *iface,
        REFIID riid, void **object)
{
    TRACE("iface %p, riid %s, object %p\n", iface, debugstr_guid(riid), object);

    if (IsEqualGUID(riid, &IID_IWineD3DRendertargetView)
            || IsEqualGUID(riid, &IID_IWineD3DBase)
            || IsEqualGUID(riid, &IID_IUnknown))
    {
        IUnknown_AddRef(iface);
        *object = iface;
        return S_OK;
    }

    WARN("%s not implemented, returning E_NOINTERFACE\n", debugstr_guid(riid));

    *object = NULL;
    return E_NOINTERFACE;
}

static ULONG STDMETHODCALLTYPE rendertarget_view_AddRef(IWineD3DRendertargetView *iface)
{
    struct wined3d_rendertarget_view *This = (struct wined3d_rendertarget_view *)iface;
    ULONG refcount = InterlockedIncrement(&This->refcount);

    TRACE("%p increasing refcount to %u\n", This, refcount);

    return refcount;
}

static ULONG STDMETHODCALLTYPE rendertarget_view_Release(IWineD3DRendertargetView *iface)
{
    struct wined3d_rendertarget_view *This = (struct wined3d_rendertarget_view *)iface;
    ULONG refcount = InterlockedDecrement(&This->refcount);

    TRACE("%p decreasing refcount to %u\n", This, refcount);

    if (!refcount)
    {
        IWineD3DResource_Release(This->resource);
        HeapFree(GetProcessHeap(), 0, This);
    }

    return refcount;
}

/* IWineD3DBase methods */

static HRESULT STDMETHODCALLTYPE rendertarget_view_GetParent(IWineD3DRendertargetView *iface, IUnknown **parent)
{
    struct wined3d_rendertarget_view *This = (struct wined3d_rendertarget_view *)iface;

    IUnknown_AddRef(This->parent);
    *parent = This->parent;

    return WINED3D_OK;
}

/* IWineD3DRendertargetView methods */

static HRESULT STDMETHODCALLTYPE rendertarget_view_GetResource(IWineD3DRendertargetView *iface,
        IWineD3DResource **resource)
{
    struct wined3d_rendertarget_view *This = (struct wined3d_rendertarget_view *)iface;

    IWineD3DResource_AddRef(This->resource);
    *resource = This->resource;

    return WINED3D_OK;
}

static const struct IWineD3DRendertargetViewVtbl wined3d_rendertarget_view_vtbl =
{
    /* IUnknown methods */
    rendertarget_view_QueryInterface,
    rendertarget_view_AddRef,
    rendertarget_view_Release,
    /* IWineD3DBase methods */
    rendertarget_view_GetParent,
    /* IWineD3DRendertargetView methods */
    rendertarget_view_GetResource,
};

void wined3d_rendertarget_view_init(struct wined3d_rendertarget_view *view,
        IWineD3DResource *resource, IUnknown *parent)
{
    view->vtbl = &wined3d_rendertarget_view_vtbl;
    view->refcount = 1;
    IWineD3DResource_AddRef(resource);
    view->resource = resource;
    view->parent = parent;
}
