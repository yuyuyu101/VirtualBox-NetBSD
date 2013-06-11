/*
 * IWineD3DTexture implementation
 *
 * Copyright 2002-2005 Jason Edmeades
 * Copyright 2002-2005 Raphael Junqueira
 * Copyright 2005 Oliver Stieber
 * Copyright 2007-2008 Stefan Dösinger for CodeWeavers
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

WINE_DEFAULT_DEBUG_CHANNEL(d3d_texture);

static void texture_internal_preload(IWineD3DBaseTexture *iface, enum WINED3DSRGB srgb)
{
    /* Override the IWineD3DResource PreLoad method. */
    IWineD3DTextureImpl *This = (IWineD3DTextureImpl *)iface;
    IWineD3DDeviceImpl *device = This->resource.device;
    struct wined3d_context *context = NULL;
    unsigned int i;
    BOOL srgb_mode;
    BOOL *dirty;

    TRACE("(%p) : About to load texture.\n", This);

    switch (srgb)
    {
        case SRGB_RGB:
            srgb_mode = FALSE;
            break;

        case SRGB_BOTH:
            texture_internal_preload(iface, SRGB_RGB);
            /* Fallthrough */

        case SRGB_SRGB:
            srgb_mode = TRUE;
            break;

        default:
            srgb_mode = This->baseTexture.is_srgb;
            break;
    }
    dirty = srgb_mode ? &This->baseTexture.texture_srgb.dirty : &This->baseTexture.texture_rgb.dirty;

    if (!device->isInDraw)
    {
        /* context_acquire() sets isInDraw to TRUE when loading a pbuffer into a texture,
         * thus no danger of recursive calls. */
        context = context_acquire(device, NULL, CTXUSAGE_RESOURCELOAD);
    }

    if (This->resource.format_desc->format == WINED3DFMT_P8_UINT
            || This->resource.format_desc->format == WINED3DFMT_P8_UINT_A8_UNORM)
    {
        for (i = 0; i < This->baseTexture.levels; ++i)
        {
            if (palette9_changed((IWineD3DSurfaceImpl *)This->surfaces[i]))
            {
                TRACE("Reloading surface because the d3d8/9 palette was changed.\n");
                /* TODO: This is not necessarily needed with hw palettized texture support. */
                IWineD3DSurface_LoadLocation(This->surfaces[i], SFLAG_INSYSMEM, NULL);
                /* Make sure the texture is reloaded because of the palette change, this kills performance though :( */
                IWineD3DSurface_ModifyLocation(This->surfaces[i], SFLAG_INTEXTURE, FALSE);
            }
        }
    }

    /* If the texture is marked dirty or the srgb sampler setting has changed
     * since the last load then reload the surfaces. */
    if (*dirty)
    {
        for (i = 0; i < This->baseTexture.levels; ++i)
        {
            IWineD3DSurface_LoadTexture(This->surfaces[i], srgb_mode);
        }
    }
    else
    {
        TRACE("(%p) Texture not dirty, nothing to do.\n", iface);
    }

    if (context) context_release(context);

    /* No longer dirty. */
    *dirty = FALSE;
}

static void texture_cleanup(IWineD3DTextureImpl *This)
{
    unsigned int i;

    TRACE("(%p) : Cleaning up\n", This);

    for (i = 0; i < This->baseTexture.levels; ++i)
    {
        if (This->surfaces[i])
        {
            /* Clean out the texture name we gave to the surface so that the
             * surface doesn't try and release it */
            surface_set_texture_name(This->surfaces[i], 0, TRUE);
            surface_set_texture_name(This->surfaces[i], 0, FALSE);
            surface_set_texture_target(This->surfaces[i], 0);
            IWineD3DSurface_SetContainer(This->surfaces[i], 0);
            IWineD3DSurface_Release(This->surfaces[i]);
        }
    }

    TRACE("(%p) : Cleaning up base texture\n", This);
    basetexture_cleanup((IWineD3DBaseTexture *)This);
}

/* *******************************************
   IWineD3DTexture IUnknown parts follow
   ******************************************* */

static HRESULT WINAPI IWineD3DTextureImpl_QueryInterface(IWineD3DTexture *iface, REFIID riid, LPVOID *ppobj)
{
    IWineD3DTextureImpl *This = (IWineD3DTextureImpl *)iface;
    TRACE("(%p)->(%s,%p)\n",This,debugstr_guid(riid),ppobj);
    if (IsEqualGUID(riid, &IID_IUnknown)
        || IsEqualGUID(riid, &IID_IWineD3DBase)
        || IsEqualGUID(riid, &IID_IWineD3DResource)
        || IsEqualGUID(riid, &IID_IWineD3DBaseTexture)
        || IsEqualGUID(riid, &IID_IWineD3DTexture)){
        IUnknown_AddRef(iface);
        *ppobj = This;
        return WINED3D_OK;
    }
    *ppobj = NULL;
    return E_NOINTERFACE;
}

static ULONG WINAPI IWineD3DTextureImpl_AddRef(IWineD3DTexture *iface) {
    IWineD3DTextureImpl *This = (IWineD3DTextureImpl *)iface;
    TRACE("(%p) : AddRef increasing from %d\n", This, This->resource.ref);
    return InterlockedIncrement(&This->resource.ref);
}

static ULONG WINAPI IWineD3DTextureImpl_Release(IWineD3DTexture *iface) {
    IWineD3DTextureImpl *This = (IWineD3DTextureImpl *)iface;
    ULONG ref;
    TRACE("(%p) : Releasing from %d\n", This, This->resource.ref);
    ref = InterlockedDecrement(&This->resource.ref);
    if (!ref)
    {
        texture_cleanup(This);
        This->resource.parent_ops->wined3d_object_destroyed(This->resource.parent);
        HeapFree(GetProcessHeap(), 0, This);
    }
    return ref;
}


/* ****************************************************
   IWineD3DTexture IWineD3DResource parts follow
   **************************************************** */
static HRESULT WINAPI IWineD3DTextureImpl_SetPrivateData(IWineD3DTexture *iface, REFGUID refguid, CONST void* pData, DWORD SizeOfData, DWORD Flags) {
    return resource_set_private_data((IWineD3DResource *)iface, refguid, pData, SizeOfData, Flags);
}

static HRESULT WINAPI IWineD3DTextureImpl_GetPrivateData(IWineD3DTexture *iface, REFGUID refguid, void* pData, DWORD* pSizeOfData) {
    return resource_get_private_data((IWineD3DResource *)iface, refguid, pData, pSizeOfData);
}

static HRESULT WINAPI IWineD3DTextureImpl_FreePrivateData(IWineD3DTexture *iface, REFGUID refguid) {
    return resource_free_private_data((IWineD3DResource *)iface, refguid);
}

static DWORD WINAPI IWineD3DTextureImpl_SetPriority(IWineD3DTexture *iface, DWORD PriorityNew) {
    return resource_set_priority((IWineD3DResource *)iface, PriorityNew);
}

static DWORD WINAPI IWineD3DTextureImpl_GetPriority(IWineD3DTexture *iface) {
    return resource_get_priority((IWineD3DResource *)iface);
}

static void WINAPI IWineD3DTextureImpl_PreLoad(IWineD3DTexture *iface) {
    texture_internal_preload((IWineD3DBaseTexture *) iface, SRGB_ANY);
}

static void WINAPI IWineD3DTextureImpl_UnLoad(IWineD3DTexture *iface) {
    unsigned int i;
    IWineD3DTextureImpl *This = (IWineD3DTextureImpl *)iface;
    TRACE("(%p)\n", This);

    /* Unload all the surfaces and reset the texture name. If UnLoad was called on the
     * surface before, this one will be a NOP and vice versa. Unloading an unloaded
     * surface is fine
     */
    for (i = 0; i < This->baseTexture.levels; i++) {
        IWineD3DSurface_UnLoad(This->surfaces[i]);
        surface_set_texture_name(This->surfaces[i], 0, FALSE); /* Delete rgb name */
        surface_set_texture_name(This->surfaces[i], 0, TRUE); /* delete srgb name */
    }

    basetexture_unload((IWineD3DBaseTexture *)iface);
}

static WINED3DRESOURCETYPE WINAPI IWineD3DTextureImpl_GetType(IWineD3DTexture *iface) {
    return resource_get_type((IWineD3DResource *)iface);
}

static HRESULT WINAPI IWineD3DTextureImpl_GetParent(IWineD3DTexture *iface, IUnknown **pParent) {
    return resource_get_parent((IWineD3DResource *)iface, pParent);
}

#ifdef VBOX_WITH_WDDM
static HRESULT WINAPI IWineD3DTextureImpl_SetShRcState(IWineD3DTexture *iface, VBOXWINEEX_SHRC_STATE enmState) {
    IWineD3DTextureImpl *This = (IWineD3DTextureImpl*)iface;
    struct wined3d_context *context = NULL;
    HRESULT hr = IWineD3DResourceImpl_SetShRcState((IWineD3DResource*)iface, enmState);
    unsigned int i;

    if (FAILED(hr))
    {
        ERR("IWineD3DResource_SetShRcState failed");
        return hr;
    }

    for (i = 0; i < This->baseTexture.levels; ++i)
    {
        if (This->surfaces[i])
        {
            HRESULT tmpHr = IWineD3DResource_SetShRcState((IWineD3DResource*)This->surfaces[i], enmState);
            Assert(tmpHr == S_OK);
        }
    }

    if (!This->resource.device->isInDraw)
    {
        context = context_acquire(This->resource.device, NULL, CTXUSAGE_RESOURCELOAD);
        if (!context)
        {
            ERR("zero context!");
            return E_FAIL;
        }

        if (!context->valid)
        {
            ERR("context invalid!");
            context_release(context);
            return E_FAIL;
        }
    }

    device_cleanup_durtify_texture_target(This->resource.device, This->target);

    if (context)
        context_release(context);

    return WINED3D_OK;
}
#endif

/* ******************************************************
   IWineD3DTexture IWineD3DBaseTexture parts follow
   ****************************************************** */
static DWORD WINAPI IWineD3DTextureImpl_SetLOD(IWineD3DTexture *iface, DWORD LODNew) {
    return basetexture_set_lod((IWineD3DBaseTexture *)iface, LODNew);
}

static DWORD WINAPI IWineD3DTextureImpl_GetLOD(IWineD3DTexture *iface) {
    return basetexture_get_lod((IWineD3DBaseTexture *)iface);
}

static DWORD WINAPI IWineD3DTextureImpl_GetLevelCount(IWineD3DTexture *iface) {
    return basetexture_get_level_count((IWineD3DBaseTexture *)iface);
}

static HRESULT WINAPI IWineD3DTextureImpl_SetAutoGenFilterType(IWineD3DTexture *iface, WINED3DTEXTUREFILTERTYPE FilterType) {
  return basetexture_set_autogen_filter_type((IWineD3DBaseTexture *)iface, FilterType);
}

static WINED3DTEXTUREFILTERTYPE WINAPI IWineD3DTextureImpl_GetAutoGenFilterType(IWineD3DTexture *iface) {
  return basetexture_get_autogen_filter_type((IWineD3DBaseTexture *)iface);
}

static void WINAPI IWineD3DTextureImpl_GenerateMipSubLevels(IWineD3DTexture *iface) {
    basetexture_generate_mipmaps((IWineD3DBaseTexture *)iface);
}

/* Internal function, No d3d mapping */
static BOOL WINAPI IWineD3DTextureImpl_SetDirty(IWineD3DTexture *iface, BOOL dirty) {
    return basetexture_set_dirty((IWineD3DBaseTexture *)iface, dirty);
}

static BOOL WINAPI IWineD3DTextureImpl_GetDirty(IWineD3DTexture *iface) {
    return basetexture_get_dirty((IWineD3DBaseTexture *)iface);
}

/* Context activation is done by the caller. */
static HRESULT WINAPI IWineD3DTextureImpl_BindTexture(IWineD3DTexture *iface, BOOL srgb) {
    IWineD3DTextureImpl *This = (IWineD3DTextureImpl *)iface;
    BOOL set_gl_texture_desc;
    HRESULT hr;

    TRACE("(%p) : relay to BaseTexture\n", This);

    hr = basetexture_bind((IWineD3DBaseTexture *)iface, srgb, &set_gl_texture_desc);
    if (set_gl_texture_desc && SUCCEEDED(hr)) {
        UINT i;
        struct gl_texture *gl_tex;

        if(This->baseTexture.is_srgb) {
            gl_tex = &This->baseTexture.texture_srgb;
        } else {
            gl_tex = &This->baseTexture.texture_rgb;
        }

        for (i = 0; i < This->baseTexture.levels; ++i) {
            surface_set_texture_name(This->surfaces[i], gl_tex->name, This->baseTexture.is_srgb);
        }

        /* Conditinal non power of two textures use a different clamping default. If we're using the GL_WINE_normalized_texrect
         * partial driver emulation, we're dealing with a GL_TEXTURE_2D texture which has the address mode set to repeat - something
         * that prevents us from hitting the accelerated codepath. Thus manually set the GL state. The same applies to filtering.
         * Even if the texture has only one mip level, the default LINEAR_MIPMAP_LINEAR filter causes a SW fallback on macos.
         */
        if(IWineD3DBaseTexture_IsCondNP2(iface)) {
#ifdef VBOX_WITH_WDDM
            if (!VBOXSHRC_IS_SHARED_OPENED(This))
#endif
            {
                ENTER_GL();
                glTexParameteri(IWineD3DTexture_GetTextureDimensions(iface), GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
                checkGLcall("glTexParameteri(dimension, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)");
                glTexParameteri(IWineD3DTexture_GetTextureDimensions(iface), GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
                checkGLcall("glTexParameteri(dimension, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)");
                glTexParameteri(IWineD3DTexture_GetTextureDimensions(iface), GL_TEXTURE_MIN_FILTER, GL_NEAREST);
                checkGLcall("glTexParameteri(dimension, GL_TEXTURE_MIN_FILTER, GL_NEAREST)");
                glTexParameteri(IWineD3DTexture_GetTextureDimensions(iface), GL_TEXTURE_MAG_FILTER, GL_NEAREST);
                checkGLcall("glTexParameteri(dimension, GL_TEXTURE_MAG_FILTER, GL_NEAREST)");
                LEAVE_GL();
            }
            gl_tex->states[WINED3DTEXSTA_ADDRESSU]      = WINED3DTADDRESS_CLAMP;
            gl_tex->states[WINED3DTEXSTA_ADDRESSV]      = WINED3DTADDRESS_CLAMP;
            gl_tex->states[WINED3DTEXSTA_MAGFILTER]     = WINED3DTEXF_POINT;
            gl_tex->states[WINED3DTEXSTA_MINFILTER]     = WINED3DTEXF_POINT;
            gl_tex->states[WINED3DTEXSTA_MIPFILTER]     = WINED3DTEXF_NONE;
        }
    }

    return hr;
}

static UINT WINAPI IWineD3DTextureImpl_GetTextureDimensions(IWineD3DTexture *iface) {
    IWineD3DTextureImpl *This = (IWineD3DTextureImpl *)iface;
    TRACE("(%p)\n", This);

    return This->target;
}

static BOOL WINAPI IWineD3DTextureImpl_IsCondNP2(IWineD3DTexture *iface) {
    IWineD3DTextureImpl *This = (IWineD3DTextureImpl *)iface;
    TRACE("(%p)\n", This);

    return This->cond_np2;
}

/* *******************************************
   IWineD3DTexture IWineD3DTexture parts follow
   ******************************************* */
static HRESULT WINAPI IWineD3DTextureImpl_GetLevelDesc(IWineD3DTexture *iface, UINT Level, WINED3DSURFACE_DESC* pDesc) {
    IWineD3DTextureImpl *This = (IWineD3DTextureImpl *)iface;

    if (Level < This->baseTexture.levels) {
        TRACE("(%p) Level (%d)\n", This, Level);
        return IWineD3DSurface_GetDesc(This->surfaces[Level], pDesc);
    }
    WARN("(%p) level(%d) overflow Levels(%d)\n", This, Level, This->baseTexture.levels);
    return WINED3DERR_INVALIDCALL;
}

static HRESULT WINAPI IWineD3DTextureImpl_GetSurfaceLevel(IWineD3DTexture *iface, UINT Level, IWineD3DSurface** ppSurfaceLevel) {
    IWineD3DTextureImpl *This = (IWineD3DTextureImpl *)iface;
    HRESULT hr = WINED3DERR_INVALIDCALL;

    if (Level < This->baseTexture.levels) {
        *ppSurfaceLevel = This->surfaces[Level];
        IWineD3DSurface_AddRef(This->surfaces[Level]);
        hr = WINED3D_OK;
        TRACE("(%p) : returning %p for level %d\n", This, *ppSurfaceLevel, Level);
    }
    if (WINED3D_OK != hr) {
        WARN("(%p) level(%d) overflow Levels(%d)\n", This, Level, This->baseTexture.levels);
        *ppSurfaceLevel = NULL; /* Just to be on the safe side.. */
    }
    return hr;
}

static HRESULT WINAPI IWineD3DTextureImpl_LockRect(IWineD3DTexture *iface, UINT Level, WINED3DLOCKED_RECT *pLockedRect,
                                            CONST RECT *pRect, DWORD Flags) {
    IWineD3DTextureImpl *This = (IWineD3DTextureImpl *)iface;
    HRESULT hr = WINED3DERR_INVALIDCALL;

    if (Level < This->baseTexture.levels) {
        hr = IWineD3DSurface_LockRect(This->surfaces[Level], pLockedRect, pRect, Flags);
    }
    if (WINED3D_OK == hr) {
        TRACE("(%p) Level (%d) success\n", This, Level);
    } else {
        WARN("(%p) level(%d) overflow Levels(%d)\n", This, Level, This->baseTexture.levels);
    }

    return hr;
}

static HRESULT WINAPI IWineD3DTextureImpl_UnlockRect(IWineD3DTexture *iface, UINT Level) {
   IWineD3DTextureImpl *This = (IWineD3DTextureImpl *)iface;
    HRESULT hr = WINED3DERR_INVALIDCALL;

    if (Level < This->baseTexture.levels) {
        hr = IWineD3DSurface_UnlockRect(This->surfaces[Level]);
    }
    if ( WINED3D_OK == hr) {
        TRACE("(%p) Level (%d) success\n", This, Level);
    } else {
        WARN("(%p) level(%d) overflow Levels(%d)\n", This, Level, This->baseTexture.levels);
    }
    return hr;
}

static HRESULT WINAPI IWineD3DTextureImpl_AddDirtyRect(IWineD3DTexture *iface, CONST RECT* pDirtyRect) {
    IWineD3DTextureImpl *This = (IWineD3DTextureImpl *)iface;
    This->baseTexture.texture_rgb.dirty = TRUE;
    This->baseTexture.texture_srgb.dirty = TRUE;
    TRACE("(%p) : dirtyfication of surface Level (0)\n", This);
    surface_add_dirty_rect(This->surfaces[0], pDirtyRect);

    return WINED3D_OK;
}

static const IWineD3DTextureVtbl IWineD3DTexture_Vtbl =
{
    /* IUnknown */
    IWineD3DTextureImpl_QueryInterface,
    IWineD3DTextureImpl_AddRef,
    IWineD3DTextureImpl_Release,
    /* IWineD3DResource */
    IWineD3DTextureImpl_GetParent,
    IWineD3DTextureImpl_SetPrivateData,
    IWineD3DTextureImpl_GetPrivateData,
    IWineD3DTextureImpl_FreePrivateData,
    IWineD3DTextureImpl_SetPriority,
    IWineD3DTextureImpl_GetPriority,
    IWineD3DTextureImpl_PreLoad,
    IWineD3DTextureImpl_UnLoad,
    IWineD3DTextureImpl_GetType,
#ifdef VBOX_WITH_WDDM
    IWineD3DTextureImpl_SetShRcState,
#endif
    /* IWineD3DBaseTexture */
    IWineD3DTextureImpl_SetLOD,
    IWineD3DTextureImpl_GetLOD,
    IWineD3DTextureImpl_GetLevelCount,
    IWineD3DTextureImpl_SetAutoGenFilterType,
    IWineD3DTextureImpl_GetAutoGenFilterType,
    IWineD3DTextureImpl_GenerateMipSubLevels,
    IWineD3DTextureImpl_SetDirty,
    IWineD3DTextureImpl_GetDirty,
    IWineD3DTextureImpl_BindTexture,
    IWineD3DTextureImpl_GetTextureDimensions,
    IWineD3DTextureImpl_IsCondNP2,
    /* IWineD3DTexture */
    IWineD3DTextureImpl_GetLevelDesc,
    IWineD3DTextureImpl_GetSurfaceLevel,
    IWineD3DTextureImpl_LockRect,
    IWineD3DTextureImpl_UnlockRect,
    IWineD3DTextureImpl_AddDirtyRect
};

void texture_state_init(IWineD3DTexture *iface, struct gl_texture *gl_tex)
{
    basetexture_state_init((IWineD3DBaseTexture*)iface, gl_tex);
    if(IWineD3DBaseTexture_IsCondNP2(iface)) {
        gl_tex->states[WINED3DTEXSTA_ADDRESSU]      = WINED3DTADDRESS_CLAMP;
        gl_tex->states[WINED3DTEXSTA_ADDRESSV]      = WINED3DTADDRESS_CLAMP;
        gl_tex->states[WINED3DTEXSTA_MAGFILTER]     = WINED3DTEXF_POINT;
        gl_tex->states[WINED3DTEXSTA_MINFILTER]     = WINED3DTEXF_POINT;
        gl_tex->states[WINED3DTEXSTA_MIPFILTER]     = WINED3DTEXF_NONE;
    }
}

HRESULT texture_init(IWineD3DTextureImpl *texture, UINT width, UINT height, UINT levels,
        IWineD3DDeviceImpl *device, DWORD usage, WINED3DFORMAT format, WINED3DPOOL pool,
        IUnknown *parent, const struct wined3d_parent_ops *parent_ops
#ifdef VBOX_WITH_WDDM
        , HANDLE *shared_handle
        , void **pavClientMem
#endif
        )
{
    const struct wined3d_gl_info *gl_info = &device->adapter->gl_info;
    const struct wined3d_format_desc *format_desc = getFormatDescEntry(format, gl_info);
    UINT pow2_width, pow2_height;
    UINT tmp_w, tmp_h;
    unsigned int i;
    HRESULT hr;

    /* TODO: It should only be possible to create textures for formats
     * that are reported as supported. */
    if (WINED3DFMT_UNKNOWN >= format)
    {
        WARN("(%p) : Texture cannot be created with a format of WINED3DFMT_UNKNOWN.\n", texture);
        return WINED3DERR_INVALIDCALL;
    }

    /* Non-power2 support. */
    if (gl_info->supported[ARB_TEXTURE_NON_POWER_OF_TWO])
    {
        pow2_width = width;
        pow2_height = height;
    }
    else
    {
        /* Find the nearest pow2 match. */
        pow2_width = pow2_height = 1;
        while (pow2_width < width) pow2_width <<= 1;
        while (pow2_height < height) pow2_height <<= 1;

        if (pow2_width != width || pow2_height != height)
        {
            if (levels > 1)
            {
                WARN("Attempted to create a mipmapped np2 texture without unconditional np2 support.\n");
                return WINED3DERR_INVALIDCALL;
            }
            levels = 1;
        }
    }

    /* Calculate levels for mip mapping. */
    if (usage & WINED3DUSAGE_AUTOGENMIPMAP)
    {
        if (!gl_info->supported[SGIS_GENERATE_MIPMAP])
        {
            WARN("No mipmap generation support, returning WINED3DERR_INVALIDCALL.\n");
            return WINED3DERR_INVALIDCALL;
        }

        if (levels > 1)
        {
            WARN("D3DUSAGE_AUTOGENMIPMAP is set, and level count > 1, returning WINED3DERR_INVALIDCALL.\n");
            return WINED3DERR_INVALIDCALL;
        }

        levels = 1;
    }
    else if (!levels)
    {
        levels = wined3d_log2i(max(width, height)) + 1;
        TRACE("Calculated levels = %u.\n", levels);
    }

    texture->lpVtbl = &IWineD3DTexture_Vtbl;

    hr = basetexture_init((IWineD3DBaseTextureImpl *)texture, levels, WINED3DRTYPE_TEXTURE,
            device, 0, usage, format_desc, pool, parent, parent_ops
#ifdef VBOX_WITH_WDDM
            , shared_handle, pavClientMem
#endif
        );
    if (FAILED(hr))
    {
        WARN("Failed to initialize basetexture, returning %#x.\n", hr);
        return hr;
    }

    /* Precalculated scaling for 'faked' non power of two texture coords.
     * Second also don't use ARB_TEXTURE_RECTANGLE in case the surface format is P8 and EXT_PALETTED_TEXTURE
     * is used in combination with texture uploads (RTL_READTEX). The reason is that EXT_PALETTED_TEXTURE
     * doesn't work in combination with ARB_TEXTURE_RECTANGLE. */
    if (gl_info->supported[WINE_NORMALIZED_TEXRECT] && (width != pow2_width || height != pow2_height))
    {
        texture->baseTexture.pow2Matrix[0] = 1.0f;
        texture->baseTexture.pow2Matrix[5] = 1.0f;
        texture->baseTexture.pow2Matrix[10] = 1.0f;
        texture->baseTexture.pow2Matrix[15] = 1.0f;
        texture->target = GL_TEXTURE_2D;
        texture->cond_np2 = TRUE;
        texture->baseTexture.minMipLookup = minMipLookup_noFilter;
    }
    else if (gl_info->supported[ARB_TEXTURE_RECTANGLE] && (width != pow2_width || height != pow2_height)
            && !(format_desc->format == WINED3DFMT_P8_UINT && gl_info->supported[EXT_PALETTED_TEXTURE]
            && wined3d_settings.rendertargetlock_mode == RTL_READTEX))
    {
        if ((width != 1) || (height != 1)) texture->baseTexture.pow2Matrix_identity = FALSE;

        texture->baseTexture.pow2Matrix[0] = (float)width;
        texture->baseTexture.pow2Matrix[5] = (float)height;
        texture->baseTexture.pow2Matrix[10] = 1.0f;
        texture->baseTexture.pow2Matrix[15] = 1.0f;
        texture->target = GL_TEXTURE_RECTANGLE_ARB;
        texture->cond_np2 = TRUE;

        if(texture->resource.format_desc->Flags & WINED3DFMT_FLAG_FILTERING)
        {
            texture->baseTexture.minMipLookup = minMipLookup_noMip;
        }
        else
        {
            texture->baseTexture.minMipLookup = minMipLookup_noFilter;
        }
    }
    else
    {
        if ((width != pow2_width) || (height != pow2_height))
        {
            texture->baseTexture.pow2Matrix_identity = FALSE;
            texture->baseTexture.pow2Matrix[0] = (((float)width) / ((float)pow2_width));
            texture->baseTexture.pow2Matrix[5] = (((float)height) / ((float)pow2_height));
        }
        else
        {
            texture->baseTexture.pow2Matrix[0] = 1.0f;
            texture->baseTexture.pow2Matrix[5] = 1.0f;
        }

        texture->baseTexture.pow2Matrix[10] = 1.0f;
        texture->baseTexture.pow2Matrix[15] = 1.0f;
        texture->target = GL_TEXTURE_2D;
        texture->cond_np2 = FALSE;
    }
    TRACE("xf(%f) yf(%f)\n", texture->baseTexture.pow2Matrix[0], texture->baseTexture.pow2Matrix[5]);

    /* Generate all the surfaces. */
    tmp_w = width;
    tmp_h = height;
    for (i = 0; i < texture->baseTexture.levels; ++i)
    {
#ifdef VBOX_WITH_WDDM
        /* Use the callback to create the texture surface. */
        hr = IWineD3DDeviceParent_CreateSurface(device->device_parent, parent, tmp_w, tmp_h, format_desc->format,
                usage, pool, i, WINED3DCUBEMAP_FACE_POSITIVE_X, &texture->surfaces[i]
                , NULL /* <- we first create a surface in an everage "non-shared" fashion and initialize its share properties later (see below)
                        * this is done this way because the surface does not have its parent (texture) setup properly
                        * thus we can not initialize texture at this stage */
                , pavClientMem ? pavClientMem[i] : NULL);

#else
        /* Use the callback to create the texture surface. */
        hr = IWineD3DDeviceParent_CreateSurface(device->device_parent, parent, tmp_w, tmp_h, format_desc->format,
                usage, pool, i, WINED3DCUBEMAP_FACE_POSITIVE_X, &texture->surfaces[i]);
#endif

        if (FAILED(hr))
        {
            FIXME("Failed to create surface %p, hr %#x\n", texture, hr);
            texture->surfaces[i] = NULL;
            texture_cleanup(texture);
            return hr;
        }

        IWineD3DSurface_SetContainer(texture->surfaces[i], (IWineD3DBase *)texture);
        TRACE("Created surface level %u @ %p.\n", i, texture->surfaces[i]);
        surface_set_texture_target(texture->surfaces[i], texture->target);
        /* Calculate the next mipmap level. */
        tmp_w = max(1, tmp_w >> 1);
        tmp_h = max(1, tmp_h >> 1);
    }
    texture->baseTexture.internal_preload = texture_internal_preload;

#ifdef VBOX_WITH_WDDM
    if (VBOXSHRC_IS_SHARED(texture))
    {
        struct wined3d_context * context;
        Assert(shared_handle);
        for (i = 0; i < texture->baseTexture.levels; ++i)
        {
            VBOXSHRC_COPY_SHAREDATA((IWineD3DSurfaceImpl*)texture->surfaces[i], texture);
        }
#ifdef DEBUG
        for (i = 0; i < texture->baseTexture.levels; ++i)
        {
            Assert(!((IWineD3DSurfaceImpl*)texture->surfaces[i])->texture_name);
        }
#endif
        for (i = 0; i < texture->baseTexture.levels; ++i)
        {
            if (!VBOXSHRC_IS_SHARED_OPENED(texture))
            {
                IWineD3DSurface_LoadLocation(texture->surfaces[i], SFLAG_INTEXTURE, NULL);
                Assert(!(*shared_handle));
                *shared_handle = VBOXSHRC_GET_SHAREHANDLE(texture);
            }
            else
            {
                surface_setup_location_onopen((IWineD3DSurfaceImpl*)texture->surfaces[i]);
                Assert(*shared_handle);
                Assert(*shared_handle == VBOXSHRC_GET_SHAREHANDLE(texture));
            }
        }
#ifdef DEBUG
        for (i = 0; i < texture->baseTexture.levels; ++i)
        {
            Assert((GLuint)(*shared_handle) == ((IWineD3DSurfaceImpl*)texture->surfaces[i])->texture_name);
        }
#endif

        Assert(!device->isInDraw);

        /* flush to ensure the texture is allocated/referenced before it is used/released by another
         * process opening/creating it */
        context = context_acquire(device, NULL, CTXUSAGE_RESOURCELOAD);
        if (context->valid)
        {
            wglFlush();
        }
        else
        {
            ERR("invalid context!");
        }
        context_release(context);
    }
    else
    {
        Assert(!shared_handle);
    }
#endif

    return WINED3D_OK;
}
