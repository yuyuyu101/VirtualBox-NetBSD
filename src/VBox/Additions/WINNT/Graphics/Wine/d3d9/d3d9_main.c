/*
 * Direct3D 9
 *
 * Copyright 2002-2003 Jason Edmeades
 * Copyright 2002-2003 Raphael Junqueira
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

#define VBOX_WINE_DEBUG_DEFINES

#include "config.h"
#include "initguid.h"
#include "d3d9_private.h"

WINE_DEFAULT_DEBUG_CHANNEL(d3d9);

static int D3DPERF_event_level = 0;

void WINAPI DebugSetMute(void) {
    /* nothing to do */
}

IDirect3D9* WINAPI DECLSPEC_HOTPATCH Direct3DCreate9(UINT SDKVersion) {
    IDirect3D9Impl* object = HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, sizeof(IDirect3D9Impl));

    object->lpVtbl = &Direct3D9_Vtbl;
    object->ref = 1;

    wined3d_mutex_lock();
    object->WineD3D = WineDirect3DCreate(9, (IUnknown *)object);
    wined3d_mutex_unlock();

    TRACE("SDKVersion = %x, Created Direct3D object @ %p, WineObj @ %p\n", SDKVersion, object, object->WineD3D);

    if (!object->WineD3D)
    {
        HeapFree( GetProcessHeap(), 0, object );
        object = NULL;
    }
    return (IDirect3D9*) object;
}

HRESULT WINAPI DECLSPEC_HOTPATCH Direct3DCreate9Ex(UINT SDKVersion, IDirect3D9Ex **direct3d9ex) {
#ifndef VBOX_WITH_WDDM
    /* real D3D lib does not allow 9Ex creation if no WDDM driver is installed,
     * the Direct3DCreate9Ex should return D3DERR_NOTAVAILABLE in that case.
     * do it that way for our XPWM case */
    return D3DERR_NOTAVAILABLE;
#else
    IDirect3D9 *ret;
    IDirect3D9Impl* object;
    TRACE("Calling Direct3DCreate9\n");
    ret = Direct3DCreate9(SDKVersion);
    if(!ret) {
        *direct3d9ex = NULL;
        return D3DERR_NOTAVAILABLE;
    }

    object = (IDirect3D9Impl *) ret;
    object->extended = TRUE; /* Enables QI for extended interfaces */
    *direct3d9ex = (IDirect3D9Ex *) object;
    return D3D_OK;
#endif
}

/*******************************************************************
 *       Direct3DShaderValidatorCreate9 (D3D9.@)
 *
 * No documentation available for this function.
 * SDK only says it is internal and shouldn't be used.
 */
void* WINAPI Direct3DShaderValidatorCreate9(void)
{
    static int once;

    if (!once++) FIXME("stub\n");
    return NULL;
}

/*******************************************************************
 *       DllMain
 */
BOOL WINAPI DllMain(HINSTANCE hInstDLL, DWORD fdwReason, LPVOID lpv)
{
    /* At process attach */
    TRACE("fdwReason=%d\n", fdwReason);
    if (fdwReason == DLL_PROCESS_ATTACH)
        DisableThreadLibraryCalls(hInstDLL);

    return TRUE;
}

/***********************************************************************
 *              D3DPERF_BeginEvent (D3D9.@)
 */
int WINAPI D3DPERF_BeginEvent(D3DCOLOR color, LPCWSTR name) {
    TRACE("(color %#x, name %s) : stub\n", color, debugstr_w(name));

    return D3DPERF_event_level++;
}

/***********************************************************************
 *              D3DPERF_EndEvent (D3D9.@)
 */
int WINAPI D3DPERF_EndEvent(void) {
    TRACE("(void) : stub\n");

    return --D3DPERF_event_level;
}

/***********************************************************************
 *              D3DPERF_GetStatus (D3D9.@)
 */
DWORD WINAPI D3DPERF_GetStatus(void) {
    FIXME("(void) : stub\n");

    return 0;
}

/***********************************************************************
 *              D3DPERF_SetOptions (D3D9.@)
 *
 */
void WINAPI D3DPERF_SetOptions(DWORD options)
{
  FIXME("(%#x) : stub\n", options);
}

/***********************************************************************
 *              D3DPERF_QueryRepeatFrame (D3D9.@)
 */
BOOL WINAPI D3DPERF_QueryRepeatFrame(void) {
    FIXME("(void) : stub\n");

    return FALSE;
}

/***********************************************************************
 *              D3DPERF_SetMarker (D3D9.@)
 */
void WINAPI D3DPERF_SetMarker(D3DCOLOR color, LPCWSTR name) {
    FIXME("(color %#x, name %s) : stub\n", color, debugstr_w(name));
}

/***********************************************************************
 *              D3DPERF_SetRegion (D3D9.@)
 */
void WINAPI D3DPERF_SetRegion(D3DCOLOR color, LPCWSTR name) {
    FIXME("(color %#x, name %s) : stub\n", color, debugstr_w(name));
}
