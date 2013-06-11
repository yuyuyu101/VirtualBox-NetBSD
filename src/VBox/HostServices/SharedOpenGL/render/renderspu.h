/* Copyright (c) 2001, Stanford University
 * All rights reserved.
 *
 * See the file LICENSE.txt for information on redistributing this software.
 */

#ifndef CR_RENDERSPU_H
#define CR_RENDERSPU_H

#ifdef WINDOWS
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define RENDER_APIENTRY __stdcall
#define snprintf _snprintf
#elif defined(DARWIN)
# ifndef VBOX_WITH_COCOA_QT
#  include <AGL/AGL.h>
# else
#  include "renderspu_cocoa_helper.h"
# endif
#define RENDER_APIENTRY
#else
#include <GL/glx.h>
#define RENDER_APIENTRY
#endif
#include "cr_threads.h"
#include "cr_spu.h"
#include "cr_hash.h"
#include "cr_server.h"

#include <iprt/cdefs.h>

#define MAX_VISUALS 32

#ifdef RT_OS_DARWIN
# ifndef VBOX_WITH_COCOA_QT
enum
{
    /* Event classes */
    kEventClassVBox         = 'vbox',
    /* Event kinds */
    kEventVBoxShowWindow    = 'swin',
    kEventVBoxHideWindow    = 'hwin',
    kEventVBoxMoveWindow    = 'mwin',
    kEventVBoxResizeWindow  = 'rwin',
    kEventVBoxDisposeWindow = 'dwin',
    kEventVBoxUpdateDock    = 'udck',
    kEventVBoxUpdateContext = 'uctx',
    kEventVBoxBoundsChanged = 'bchg'
};
pascal OSStatus windowEvtHndlr(EventHandlerCallRef myHandler, EventRef event, void* userData);
# endif
#endif /* RT_OS_DARWIN */

/**
 * Visual info
 */
typedef struct {
    GLbitfield visAttribs;
    const char *displayName;
#if defined(WINDOWS)
//    HDC device_context;
#elif defined(DARWIN)
# ifndef VBOX_WITH_COCOA_QT
    WindowRef window;
# endif
#elif defined(GLX)
    Display *dpy;
    XVisualInfo *visual;
#ifdef GLX_VERSION_1_3
    GLXFBConfig fbconfig;
#endif /* GLX_VERSION_1_3 */
#endif
} VisualInfo;

/**
 * Window info
 */
typedef struct {
    int x, y;
    int width, height;
    int id; /**< integer window ID */
    VisualInfo *visual;
    GLboolean mapPending;
    GLboolean visible;
    GLboolean everCurrent; /**< has this window ever been bound? */
    char *title;
#if defined(WINDOWS)
    HDC nativeWindow; /**< for render_to_app_window */
    HWND hWnd;
    HDC device_context;
    HRGN hRgn;
#elif defined(DARWIN)
# ifndef VBOX_WITH_COCOA_QT
    WindowRef window;
    WindowRef nativeWindow; /**< for render_to_app_window */
    WindowRef appWindow;
    EventHandlerUPP event_handler;
    GLint bufferName;
    AGLContext dummyContext;
    RgnHandle hVisibleRegion;
    /* unsigned long context_ptr; */
# else
    NativeNSViewRef window;
    NativeNSViewRef nativeWindow; /**< for render_to_app_window */
    NativeNSOpenGLContextRef *currentCtx;
# endif
#elif defined(GLX)
    Window window;
    Window nativeWindow;  /**< for render_to_app_window */
    Window appWindow;     /**< Same as nativeWindow but for garbage collections purposes */
#endif
    int nvSwapGroup;

#ifdef USE_OSMESA
    GLubyte *buffer;    /**< for rendering to off screen buffer.  */
    int in_buffer_width;
    int in_buffer_height;
#endif

} WindowInfo;

/**
 * Context Info
 */
typedef struct _ContextInfo {
    int id; /**< integer context ID */
    VisualInfo *visual;
    GLboolean everCurrent;
    GLboolean haveWindowPosARB;
    WindowInfo *currentWindow;
#if defined(WINDOWS)
    HGLRC hRC;
#elif defined(DARWIN)
# ifndef VBOX_WITH_COCOA_QT
    AGLContext context;
# else
    NativeNSOpenGLContextRef context;
# endif
#elif defined(GLX)
    GLXContext context;
#endif
    struct _ContextInfo *shared;
    char *extensionString;
} ContextInfo;

/**
 * Barrier info
 */
typedef struct {
    CRbarrier barrier;
    GLuint count;
} Barrier;

/**
 * Renderspu state info
 */
typedef struct {
    SPUDispatchTable self;
    int id;

    unsigned int window_id;
    unsigned int context_id;

    /** config options */
    /*@{*/
    char *window_title;
    int defaultX, defaultY;
    unsigned int defaultWidth, defaultHeight;
    int default_visual;
    int use_L2;
    int fullscreen, ontop;
    char display_string[100];
#if defined(GLX)
    int try_direct;
    int force_direct;
    int sync;
#endif
    int render_to_app_window;
    int render_to_crut_window;
    int crut_drawable;
    int resizable;
    int use_lut8, lut8[3][256];
    int borderless;
    int nvSwapGroup;
    int ignore_papi;
    int ignore_window_moves;
    int pbufferWidth, pbufferHeight;
    int use_glxchoosevisual;
    int draw_bbox;
    /*@}*/

    CRServer *server;
    int gather_port;
    int gather_userbuf_size;
    CRConnection **gather_conns;

    GLint drawCursor;
    GLint cursorX, cursorY;

    int numVisuals;
    VisualInfo visuals[MAX_VISUALS];

    CRHashTable *windowTable;
    CRHashTable *contextTable;

#ifndef CHROMIUM_THREADSAFE
    ContextInfo *currentContext;
#endif

    crOpenGLInterface ws;  /**< Window System interface */

    CRHashTable *barrierHash;

    int is_swap_master, num_swap_clients;
    int swap_mtu;
    char *swap_master_url;
    CRConnection **swap_conns;

#ifdef USE_OSMESA
    /** Off screen rendering hooks.  */
    int use_osmesa;

    OSMesaContext (*OSMesaCreateContext)( GLenum format, OSMesaContext sharelist );
    GLboolean (* OSMesaMakeCurrent)( OSMesaContext ctx,
                     GLubyte *buffer,
                     GLenum type,
                     GLsizei width,
                     GLsizei height );
    void (*OSMesaDestroyContext)( OSMesaContext ctx );
#endif

#ifdef RT_OS_WINDOWS
    DWORD dwWinThreadId;
    HANDLE hWinThreadReadyEvent;
#endif

#ifdef RT_OS_DARWIN
# ifndef VBOX_WITH_COCOA_QT
    RgnHandle hRootVisibleRegion;
    RTSEMFASTMUTEX syncMutex;
    EventHandlerUPP hParentEventHandler;
    WindowGroupRef pParentGroup;
    WindowGroupRef pMasterGroup;
    GLint currentBufferName;
    uint64_t uiDockUpdateTS;
    bool fInit;
# endif
#endif /* RT_OS_DARWIN */

    int force_hidden_wdn_create;
} RenderSPU;

#ifdef RT_OS_WINDOWS

/* Asks window thread to create new window.
   msg.lParam - holds pointer to CREATESTRUCT structure
                note that lpCreateParams is used to specify address to store handle of created window
   msg.wParam - unused, should be NULL
*/
#define WM_VBOX_RENDERSPU_CREATE_WINDOW (WM_APP+1)

typedef struct _VBOX_RENDERSPU_DESTROY_WINDOW {
    HWND hWnd; /* handle to window to destroy */
} VBOX_RENDERSPU_DESTROY_WINDOW;

/* Asks window thread to destroy previously created window.
   msg.lParam - holds pointer to RENDERSPU_VBOX_WINDOW_DESTROY structure
   msg.wParam - unused, should be NULL
*/
#define WM_VBOX_RENDERSPU_DESTROY_WINDOW (WM_APP+2)

#endif

extern RenderSPU render_spu;

/* @todo remove this hack */
extern uint64_t render_spu_parent_window_id;

#ifdef CHROMIUM_THREADSAFE
extern CRtsd _RenderTSD;
#define GET_CONTEXT_VAL() ((ContextInfo *) crGetTSD(&_RenderTSD))
#define SET_CONTEXT_VAL(_v) do { \
        crSetTSD(&_RenderTSD, (_v)); \
    } while (0)
#else
#define GET_CONTEXT_VAL() (render_spu.currentContext)
#define SET_CONTEXT_VAL(_v) do { \
        render_spu.currentContext = (_v); \
    } while (0)

#endif

#define GET_CONTEXT(T)  ContextInfo *T = GET_CONTEXT_VAL()

extern void renderspuSetVBoxConfiguration( RenderSPU *spu );
extern void renderspuMakeVisString( GLbitfield visAttribs, char *s );
extern VisualInfo *renderspuFindVisual(const char *displayName, GLbitfield visAttribs );
extern GLboolean renderspu_SystemInitVisual( VisualInfo *visual );
extern GLboolean renderspu_SystemCreateContext( VisualInfo *visual, ContextInfo *context, ContextInfo *sharedContext );
extern void renderspu_SystemDestroyContext( ContextInfo *context );
extern GLboolean renderspu_SystemCreateWindow( VisualInfo *visual, GLboolean showIt, WindowInfo *window );
extern GLboolean renderspu_SystemVBoxCreateWindow( VisualInfo *visual, GLboolean showIt, WindowInfo *window );
extern void renderspu_SystemDestroyWindow( WindowInfo *window );
extern void renderspu_SystemWindowSize( WindowInfo *window, GLint w, GLint h );
extern void renderspu_SystemGetWindowGeometry( WindowInfo *window, GLint *x, GLint *y, GLint *w, GLint *h );
extern void renderspu_SystemGetMaxWindowSize( WindowInfo *window, GLint *w, GLint *h );
extern void renderspu_SystemWindowPosition( WindowInfo *window, GLint x, GLint y );
extern void renderspu_SystemWindowVisibleRegion(WindowInfo *window, GLint cRects, GLint* pRects);
extern void renderspu_SystemWindowApplyVisibleRegion(WindowInfo *window);
#ifdef RT_OS_DARWIN
extern void renderspu_SystemSetRootVisibleRegion(GLint cRects, GLint *pRects);
# ifdef VBOX_WITH_COCOA_QT
extern void renderspu_SystemFlush();
extern void renderspu_SystemFinish();
extern void renderspu_SystemBindFramebufferEXT(GLenum target, GLuint framebuffer);
extern void renderspu_SystemCopyPixels(GLint x, GLint y, GLsizei width, GLsizei height, GLenum type);
extern void renderspu_SystemGetIntegerv(GLenum pname, GLint *params);
extern void renderspu_SystemReadBuffer(GLenum mode);
extern void renderspu_SystemDrawBuffer(GLenum mode);
# endif
#endif
extern void renderspu_SystemShowWindow( WindowInfo *window, GLboolean showIt );
extern void renderspu_SystemMakeCurrent( WindowInfo *window, GLint windowInfor, ContextInfo *context );
extern void renderspu_SystemSwapBuffers( WindowInfo *window, GLint flags );
extern void renderspu_SystemReparentWindow(WindowInfo *window);
extern void renderspu_GCWindow(void);
extern int renderspuCreateFunctions( SPUNamedFunctionTable table[] );

extern GLint RENDER_APIENTRY renderspuWindowCreate( const char *dpyName, GLint visBits );
void RENDER_APIENTRY renderspuWindowDestroy( GLint win );
extern GLint RENDER_APIENTRY renderspuCreateContext( const char *dpyname, GLint visBits, GLint shareCtx );
extern void RENDER_APIENTRY renderspuMakeCurrent(GLint crWindow, GLint nativeWindow, GLint ctx);
extern void RENDER_APIENTRY renderspuSwapBuffers( GLint window, GLint flags );

#ifdef __cplusplus
extern "C" {
#endif
DECLEXPORT(void) renderspuSetWindowId(uint64_t winId);
DECLEXPORT(void) renderspuReparentWindow(GLint window);
#ifdef __cplusplus
}
#endif

#endif /* CR_RENDERSPU_H */
