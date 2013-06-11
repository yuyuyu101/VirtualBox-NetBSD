/*
 * IWineD3DVolumeTexture implementation
 *
 * Copyright 2002-2005 Jason Edmeades
 * Copyright 2002-2005 Raphael Junqueira
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

WINE_DEFAULT_DEBUG_CHANNEL(d3d_texture);

static void volumetexture_internal_preload(IWineD3DBaseTexture *iface, enum WINED3DSRGB srgb)
{
    /* Override the IWineD3DResource Preload method. */
    IWineD3DVolumeTextureImpl *This = (IWineD3DVolumeTextureImpl *)iface;
    IWineD3DDeviceImpl *device = This->resource.device;
    const struct wined3d_gl_info *gl_info = &device->adapter->gl_info;
    struct wined3d_context *context = NULL;
    BOOL srgb_mode = This->baseTexture.is_srgb;
    BOOL srgb_was_toggled = FALSE;
    unsigned int i;

    TRACE("(%p) : About to load texture.\n", This);

    if (!device->isInDraw) context = context_acquire(device, NULL, CTXUSAGE_RESOURCELOAD);
    else if (gl_info->supported[EXT_TEXTURE_SRGB] && This->baseTexture.bindCount > 0)
    {
        srgb_mode = device->stateBlock->samplerState[This->baseTexture.sampler][WINED3DSAMP_SRGBTEXTURE];
        srgb_was_toggled = This->baseTexture.is_srgb != srgb_mode;
        This->baseTexture.is_srgb = srgb_mode;
    }

    /* If the texture is marked dirty or the srgb sampler setting has changed
     * since the last load then reload the volumes. */
    if (This->baseTexture.texture_rgb.dirty)
    {
        for (i = 0; i < This->baseTexture.levels; ++i)
        {
            IWineD3DVolume_LoadTexture(This->volumes[i], i, srgb_mode);
        }
    }
    else if (srgb_was_toggled)
    {
        for (i = 0; i < This->baseTexture.levels; ++i)
        {
            volume_add_dirty_box(This->volumes[i], NULL);
            IWineD3DVolume_LoadTexture(This->volumes[i], i, srgb_mode);
        }
    }
    else
    {
        TRACE("(%p) Texture not dirty, nothing to do.\n", iface);
    }

    if (context) context_release(context);

    /* No longer dirty */
    This->baseTexture.texture_rgb.dirty = FALSE;
}

static void volumetexture_cleanup(IWineD3DVolumeTextureImpl *This)
{
    unsigned int i;

    TRACE("(%p) : Cleaning up.\n", This);

    for (i = 0; i < This->baseTexture.levels; ++i)
    {
        IWineD3DVolume *volume = This->volumes[i];

        if (volume)
        {
            /* Cleanup the container. */
            IWineD3DVolume_SetContainer(volume, NULL);
            IWineD3DVolume_Release(volume);
        }
    }
    basetexture_cleanup((IWineD3DBaseTexture *)This);
}

/* *******************************************
   IWineD3DTexture IUnknown parts follow
   ******************************************* */

static HRESULT WINAPI IWineD3DVolumeTextureImpl_QueryInterface(IWineD3DVolumeTexture *iface, REFIID riid, LPVOID *ppobj)
{
    IWineD3DVolumeTextureImpl *This = (IWineD3DVolumeTextureImpl *)iface;
    TRACE("(%p)->(%s,%p)\n",This,debugstr_guid(riid),ppobj);
    if (IsEqualGUID(riid, &IID_IUnknown)
        || IsEqualGUID(riid, &IID_IWineD3DBase)
        || IsEqualGUID(riid, &IID_IWineD3DResource)
        || IsEqualGUID(riid, &IID_IWineD3DBaseTexture)
        || IsEqualGUID(riid, &IID_IWineD3DVolumeTexture)) {
        IUnknown_AddRef(iface);
        *ppobj = This;
        return S_OK;
    }
    *ppobj = NULL;
    return E_NOINTERFACE;
}

static ULONG WINAPI IWineD3DVolumeTextureImpl_AddRef(IWineD3DVolumeTexture *iface) {
    IWineD3DVolumeTextureImpl *This = (IWineD3DVolumeTextureImpl *)iface;
    TRACE("(%p) : AddRef increasing from %d\n", This, This->resource.ref);
    return InterlockedIncrement(&This->resource.ref);
}

static ULONG WINAPI IWineD3DVolumeTextureImpl_Release(IWineD3DVolumeTexture *iface) {
    IWineD3DVolumeTextureImpl *This = (IWineD3DVolumeTextureImpl *)iface;
    ULONG ref;
    TRACE("(%p) : Releasing from %d\n", This, This->resource.ref);
    ref = InterlockedDecrement(&This->resource.ref);
    if (!ref)
    {
        volumetexture_cleanup(This);
        This->resource.parent_ops->wined3d_object_destroyed(This->resource.parent);
        HeapFree(GetProcessHeap(), 0, This);
    }
    return ref;
}

/* ****************************************************
   IWineD3DVolumeTexture IWineD3DResource parts follow
   **************************************************** */
static HRESULT WINAPI IWineD3DVolumeTextureImpl_SetPrivateData(IWineD3DVolumeTexture *iface, REFGUID refguid, CONST void* pData, DWORD SizeOfData, DWORD Flags) {
    return resource_set_private_data((IWineD3DResource *)iface, refguid, pData, SizeOfData, Flags);
}

static HRESULT WINAPI IWineD3DVolumeTextureImpl_GetPrivateData(IWineD3DVolumeTexture *iface, REFGUID refguid, void* pData, DWORD* pSizeOfData) {
    return resource_get_private_data((IWineD3DResource *)iface, refguid, pData, pSizeOfData);
}

static HRESULT WINAPI IWineD3DVolumeTextureImpl_FreePrivateData(IWineD3DVolumeTexture *iface, REFGUID refguid) {
    return resource_free_private_data((IWineD3DResource *)iface, refguid);
}

static DWORD WINAPI IWineD3DVolumeTextureImpl_SetPriority(IWineD3DVolumeTexture *iface, DWORD PriorityNew) {
    return resource_set_priority((IWineD3DResource *)iface, PriorityNew);
}

static DWORD WINAPI IWineD3DVolumeTextureImpl_GetPriority(IWineD3DVolumeTexture *iface) {
    return resource_get_priority((IWineD3DResource *)iface);
}

static void WINAPI IWineD3DVolumeTextureImpl_PreLoad(IWineD3DVolumeTexture *iface) {
    volumetexture_internal_preload((IWineD3DBaseTexture *) iface, SRGB_ANY);
}

static void WINAPI IWineD3DVolumeTextureImpl_UnLoad(IWineD3DVolumeTexture *iface) {
    unsigned int i;
    IWineD3DVolumeTextureImpl *This = (IWineD3DVolumeTextureImpl *)iface;
    TRACE("(%p)\n", This);

    /* Unload all the surfaces and reset the texture name. If UnLoad was called on the
     * surface before, this one will be a NOP and vice versa. Unloading an unloaded
     * surface is fine
     */
    for (i = 0; i < This->baseTexture.levels; i++) {
        IWineD3DVolume_UnLoad(This->volumes[i]);
    }

    basetexture_unload((IWineD3DBaseTexture *)iface);
}

static WINED3DRESOURCETYPE WINAPI IWineD3DVolumeTextureImpl_GetType(IWineD3DVolumeTexture *iface) {
    return resource_get_type((IWineD3DResource *)iface);
}

static HRESULT WINAPI IWineD3DVolumeTextureImpl_GetParent(IWineD3DVolumeTexture *iface, IUnknown **pParent) {
    return resource_get_parent((IWineD3DResource *)iface, pParent);
}

/* ******************************************************
   IWineD3DVolumeTexture IWineD3DBaseTexture parts follow
   ****************************************************** */
static DWORD WINAPI IWineD3DVolumeTextureImpl_SetLOD(IWineD3DVolumeTexture *iface, DWORD LODNew) {
    return basetexture_set_lod((IWineD3DBaseTexture *)iface, LODNew);
}

static DWORD WINAPI IWineD3DVolumeTextureImpl_GetLOD(IWineD3DVolumeTexture *iface) {
    return basetexture_get_lod((IWineD3DBaseTexture *)iface);
}

static DWORD WINAPI IWineD3DVolumeTextureImpl_GetLevelCount(IWineD3DVolumeTexture *iface) {
    return basetexture_get_level_count((IWineD3DBaseTexture *)iface);
}

static HRESULT WINAPI IWineD3DVolumeTextureImpl_SetAutoGenFilterType(IWineD3DVolumeTexture *iface, WINED3DTEXTUREFILTERTYPE FilterType) {
  return basetexture_set_autogen_filter_type((IWineD3DBaseTexture *)iface, FilterType);
}

static WINED3DTEXTUREFILTERTYPE WINAPI IWineD3DVolumeTextureImpl_GetAutoGenFilterType(IWineD3DVolumeTexture *iface) {
  return basetexture_get_autogen_filter_type((IWineD3DBaseTexture *)iface);
}

static void WINAPI IWineD3DVolumeTextureImpl_GenerateMipSubLevels(IWineD3DVolumeTexture *iface) {
    basetexture_generate_mipmaps((IWineD3DBaseTexture *)iface);
}

/* Internal function, No d3d mapping */
static BOOL WINAPI IWineD3DVolumeTextureImpl_SetDirty(IWineD3DVolumeTexture *iface, BOOL dirty) {
    return basetexture_set_dirty((IWineD3DBaseTexture *)iface, dirty);
}

static BOOL WINAPI IWineD3DVolumeTextureImpl_GetDirty(IWineD3DVolumeTexture *iface) {
    return basetexture_get_dirty((IWineD3DBaseTexture *)iface);
}

/* Context activation is done by the caller. */
static HRESULT WINAPI IWineD3DVolumeTextureImpl_BindTexture(IWineD3DVolumeTexture *iface, BOOL srgb)
{
    BOOL dummy;

    TRACE("iface %p, srgb %#x.\n", iface, srgb);

    return basetexture_bind((IWineD3DBaseTexture *)iface, srgb, &dummy);
}

static UINT WINAPI IWineD3DVolumeTextureImpl_GetTextureDimensions(IWineD3DVolumeTexture *iface)
{
    TRACE("iface %p.\n", iface);

    return GL_TEXTURE_3D;
}

static BOOL WINAPI IWineD3DVolumeTextureImpl_IsCondNP2(IWineD3DVolumeTexture *iface)
{
    TRACE("iface %p.\n", iface);

    return FALSE;
}

/* *******************************************
   IWineD3DVolumeTexture IWineD3DVolumeTexture parts follow
   ******************************************* */
static HRESULT WINAPI IWineD3DVolumeTextureImpl_GetLevelDesc(IWineD3DVolumeTexture *iface, UINT Level,WINED3DVOLUME_DESC *pDesc) {
    IWineD3DVolumeTextureImpl *This = (IWineD3DVolumeTextureImpl *)iface;
    if (Level < This->baseTexture.levels) {
        TRACE("(%p) Level (%d)\n", This, Level);
        return IWineD3DVolume_GetDesc(This->volumes[Level], pDesc);
    } else {
        WARN("(%p) Level (%d)\n", This, Level);
    }
    return WINED3D_OK;
}
static HRESULT WINAPI IWineD3DVolumeTextureImpl_GetVolumeLevel(IWineD3DVolumeTexture *iface, UINT Level, IWineD3DVolume** ppVolumeLevel) {
    IWineD3DVolumeTextureImpl *This = (IWineD3DVolumeTextureImpl *)iface;
    if (Level < This->baseTexture.levels) {
      *ppVolumeLevel = This->volumes[Level];
      IWineD3DVolume_AddRef(*ppVolumeLevel);
      TRACE("(%p) -> level(%d) returning volume@%p\n", This, Level, *ppVolumeLevel);
    } else {
      WARN("(%p) Level(%d) overflow Levels(%d)\n", This, Level, This->baseTexture.levels);
      return WINED3DERR_INVALIDCALL;
    }
    return WINED3D_OK;

}
static HRESULT WINAPI IWineD3DVolumeTextureImpl_LockBox(IWineD3DVolumeTexture *iface, UINT Level, WINED3DLOCKED_BOX* pLockedVolume, CONST WINED3DBOX* pBox, DWORD Flags) {
    HRESULT hr;
    IWineD3DVolumeTextureImpl *This = (IWineD3DVolumeTextureImpl *)iface;

    if (Level < This->baseTexture.levels) {
      hr = IWineD3DVolume_LockBox(This->volumes[Level], pLockedVolume, pBox, Flags);
      TRACE("(%p) Level (%d) success(%u)\n", This, Level, hr);

    } else {
      FIXME("(%p) level(%d) overflow Levels(%d)\n", This, Level, This->baseTexture.levels);
      return WINED3DERR_INVALIDCALL;
    }
    return hr;
}

static HRESULT WINAPI IWineD3DVolumeTextureImpl_UnlockBox(IWineD3DVolumeTexture *iface, UINT Level) {
    HRESULT hr;
    IWineD3DVolumeTextureImpl *This = (IWineD3DVolumeTextureImpl *)iface;

    if (Level < This->baseTexture.levels) {
      hr = IWineD3DVolume_UnlockBox(This->volumes[Level]);
      TRACE("(%p) -> level(%d) success(%u)\n", This, Level, hr);

    } else {
      FIXME("(%p) level(%d) overflow Levels(%d)\n", This, Level, This->baseTexture.levels);
      return WINED3DERR_INVALIDCALL;
    }
    return hr;
}

static HRESULT WINAPI IWineD3DVolumeTextureImpl_AddDirtyBox(IWineD3DVolumeTexture *iface, CONST WINED3DBOX* pDirtyBox) {
    IWineD3DVolumeTextureImpl *This = (IWineD3DVolumeTextureImpl *)iface;
    This->baseTexture.texture_rgb.dirty = TRUE;
    This->baseTexture.texture_srgb.dirty = TRUE;
    TRACE("(%p) : dirtyfication of volume Level (0)\n", This);
    volume_add_dirty_box(This->volumes[0], pDirtyBox);

    return WINED3D_OK;
}

static const IWineD3DVolumeTextureVtbl IWineD3DVolumeTexture_Vtbl =
{
    /* IUnknown */
    IWineD3DVolumeTextureImpl_QueryInterface,
    IWineD3DVolumeTextureImpl_AddRef,
    IWineD3DVolumeTextureImpl_Release,
    /* resource */
    IWineD3DVolumeTextureImpl_GetParent,
    IWineD3DVolumeTextureImpl_SetPrivateData,
    IWineD3DVolumeTextureImpl_GetPrivateData,
    IWineD3DVolumeTextureImpl_FreePrivateData,
    IWineD3DVolumeTextureImpl_SetPriority,
    IWineD3DVolumeTextureImpl_GetPriority,
    IWineD3DVolumeTextureImpl_PreLoad,
    IWineD3DVolumeTextureImpl_UnLoad,
    IWineD3DVolumeTextureImpl_GetType,
#ifdef VBOX_WITH_WDDM
    IWineD3DResourceImpl_SetShRcState,
#endif
    /* BaseTexture */
    IWineD3DVolumeTextureImpl_SetLOD,
    IWineD3DVolumeTextureImpl_GetLOD,
    IWineD3DVolumeTextureImpl_GetLevelCount,
    IWineD3DVolumeTextureImpl_SetAutoGenFilterType,
    IWineD3DVolumeTextureImpl_GetAutoGenFilterType,
    IWineD3DVolumeTextureImpl_GenerateMipSubLevels,
    IWineD3DVolumeTextureImpl_SetDirty,
    IWineD3DVolumeTextureImpl_GetDirty,
    /* not in d3d */
    IWineD3DVolumeTextureImpl_BindTexture,
    IWineD3DVolumeTextureImpl_GetTextureDimensions,
    IWineD3DVolumeTextureImpl_IsCondNP2,
    /* volume texture */
    IWineD3DVolumeTextureImpl_GetLevelDesc,
    IWineD3DVolumeTextureImpl_GetVolumeLevel,
    IWineD3DVolumeTextureImpl_LockBox,
    IWineD3DVolumeTextureImpl_UnlockBox,
    IWineD3DVolumeTextureImpl_AddDirtyBox
};

HRESULT volumetexture_init(IWineD3DVolumeTextureImpl *texture, UINT width, UINT height,
        UINT depth, UINT levels, IWineD3DDeviceImpl *device, DWORD usage, WINED3DFORMAT format,
        WINED3DPOOL pool, IUnknown *parent, const struct wined3d_parent_ops *parent_ops
#ifdef VBOX_WITH_WDDM
        , HANDLE *shared_handle
        , void **pavClientMem
#endif
        )
{
    const struct wined3d_gl_info *gl_info = &device->adapter->gl_info;
    const struct wined3d_format_desc *format_desc = getFormatDescEntry(format, gl_info);
    UINT tmp_w, tmp_h, tmp_d;
    unsigned int i;
    HRESULT hr;

#ifdef VBOX_WITH_WDDM
    if (shared_handle)
    {
        ERR("shared handle support for volume textures not impemented yet, ignoring!");
    }
#endif

    /* TODO: It should only be possible to create textures for formats
     * that are reported as supported. */
    if (WINED3DFMT_UNKNOWN >= format)
    {
        WARN("(%p) : Texture cannot be created with a format of WINED3DFMT_UNKNOWN.\n", texture);
        return WINED3DERR_INVALIDCALL;
    }

    if (!gl_info->supported[EXT_TEXTURE3D])
    {
        WARN("(%p) : Texture cannot be created - no volume texture support.\n", texture);
        return WINED3DERR_INVALIDCALL;
    }

    /* Calculate levels for mip mapping. */
    if (usage & WINED3DUSAGE_AUTOGENMIPMAP)
    {
        if (!gl_info->supported[SGIS_GENERATE_MIPMAP])
        {
            WARN("No mipmap generation support, returning D3DERR_INVALIDCALL.\n");
            return WINED3DERR_INVALIDCALL;
        }

        if (levels > 1)
        {
            WARN("D3DUSAGE_AUTOGENMIPMAP is set, and level count > 1, returning D3DERR_INVALIDCALL.\n");
            return WINED3DERR_INVALIDCALL;
        }

        levels = 1;
    }
    else if (!levels)
    {
        levels = wined3d_log2i(max(max(width, height), depth)) + 1;
        TRACE("Calculated levels = %u.\n", levels);
    }

    texture->lpVtbl = &IWineD3DVolumeTexture_Vtbl;

    hr = basetexture_init((IWineD3DBaseTextureImpl *)texture, levels, WINED3DRTYPE_VOLUMETEXTURE,
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

    /* Is NP2 support for volumes needed? */
    texture->baseTexture.pow2Matrix[0] = 1.0f;
    texture->baseTexture.pow2Matrix[5] = 1.0f;
    texture->baseTexture.pow2Matrix[10] = 1.0f;
    texture->baseTexture.pow2Matrix[15] = 1.0f;

    /* Generate all the surfaces. */
    tmp_w = width;
    tmp_h = height;
    tmp_d = depth;

    for (i = 0; i < texture->baseTexture.levels; ++i)
    {
        /* Create the volume. */
#ifdef VBOX_WITH_WDDM
        hr = IWineD3DDeviceParent_CreateVolume(device->device_parent, parent,
                tmp_w, tmp_h, tmp_d, format, pool, usage, &texture->volumes[i]
                , shared_handle, pavClientMem ? pavClientMem[i] : NULL);
#else
        hr = IWineD3DDeviceParent_CreateVolume(device->device_parent, parent,
                tmp_w, tmp_h, tmp_d, format, pool, usage, &texture->volumes[i]);
#endif
        if (FAILED(hr))
        {
            ERR("Creating a volume for the volume texture failed, hr %#x.\n", hr);
            texture->volumes[i] = NULL;
            volumetexture_cleanup(texture);
            return hr;
        }

        /* Set its container to this texture. */
        IWineD3DVolume_SetContainer(texture->volumes[i], (IWineD3DBase *)texture);

        /* Calculate the next mipmap level. */
        tmp_w = max(1, tmp_w >> 1);
        tmp_h = max(1, tmp_h >> 1);
        tmp_d = max(1, tmp_d >> 1);
    }
    texture->baseTexture.internal_preload = volumetexture_internal_preload;

    return WINED3D_OK;
}
