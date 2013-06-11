<?xml version="1.0"?>
<!-- $Id: xpcidl.xsl $ -->

<!--
 *  A template to generate a XPCOM IDL compatible interface definition file
 *  from the generic interface definition expressed in XML.

    Copyright (C) 2008-2012 Oracle Corporation

    This file is part of VirtualBox Open Source Edition (OSE), as
    available from http://www.virtualbox.org. This file is free software;
    you can redistribute it and/or modify it under the terms of the GNU
    General Public License (GPL) as published by the Free Software
    Foundation, in version 2 as it comes in the "COPYING" file of the
    VirtualBox OSE distribution. VirtualBox OSE is distributed in the
    hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
-->

<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
<xsl:output method="text"/>

<xsl:strip-space elements="*"/>


<!--
//  helper definitions
/////////////////////////////////////////////////////////////////////////////
-->

<!--
 *  capitalizes the first letter
-->
<xsl:template name="capitalize">
  <xsl:param name="str" select="."/>
  <xsl:value-of select="
    concat(
      translate(substring($str,1,1),'abcdefghijklmnopqrstuvwxyz','ABCDEFGHIJKLMNOPQRSTUVWXYZ'),
      substring($str,2)
    )
  "/>
</xsl:template>

<!--
 *  uncapitalizes the first letter only if the second one is not capital
 *  otherwise leaves the string unchanged
-->
<xsl:template name="uncapitalize">
  <xsl:param name="str" select="."/>
  <xsl:choose>
    <xsl:when test="not(contains('ABCDEFGHIJKLMNOPQRSTUVWXYZ', substring($str,2,1)))">
      <xsl:value-of select="
        concat(
          translate(substring($str,1,1),'ABCDEFGHIJKLMNOPQRSTUVWXYZ','abcdefghijklmnopqrstuvwxyz'),
          substring($str,2)
        )
      "/>
    </xsl:when>
    <xsl:otherwise>
      <xsl:value-of select="string($str)"/>
    </xsl:otherwise>
  </xsl:choose>
</xsl:template>

<!--
 *  translates the string to uppercase
-->
<xsl:template name="uppercase">
  <xsl:param name="str" select="."/>
  <xsl:value-of select="
    translate($str,'abcdefghijklmnopqrstuvwxyz','ABCDEFGHIJKLMNOPQRSTUVWXYZ')
  "/>
</xsl:template>


<!--
 *  translates the string to lowercase
-->
<xsl:template name="lowercase">
  <xsl:param name="str" select="."/>
  <xsl:value-of select="
    translate($str,'ABCDEFGHIJKLMNOPQRSTUVWXYZ','abcdefghijklmnopqrstuvwxyz')
  "/>
</xsl:template>


<!--
//  templates
/////////////////////////////////////////////////////////////////////////////
-->


<!--
 *  not explicitly matched elements and attributes
-->
<xsl:template match="*"/>


<!--
 *  header
-->
<xsl:template match="/idl">
  <xsl:text>
/*
 *  DO NOT EDIT! This is a generated file.
 *
 *  XPCOM IDL (XPIDL) definition for VirtualBox Main API (COM interfaces)
 *  generated from XIDL (XML interface definition).
 *
 *  Source    : src/VBox/Main/idl/VirtualBox.xidl
 *  Generator : src/VBox/Main/idl/xpcidl.xsl
 *
 *  This file contains portions from the following Mozilla XPCOM files:
 *      xpcom/include/xpcom/nsID.h
 *      xpcom/include/nsIException.h
 *      xpcom/include/nsprpub/prtypes.h
 *      xpcom/include/xpcom/nsISupportsBase.h
 *
 * These files were originally triple-licensed (MPL/GPL2/LGPL2.1). Oracle
 * elects to distribute this derived work under the LGPL2.1 only.
 */

/*
 * Copyright (C) 2008-2012 Oracle Corporation
 *
 * This file is part of a free software library; you can redistribute
 * it and/or modify it under the terms of the GNU Lesser General
 * Public License version 2.1 as published by the Free Software
 * Foundation and shipped in the "COPYING" file with this library.
 * The library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY of any kind.
 *
 * Oracle LGPL Disclaimer: For the avoidance of doubt, except that if
 * any license choice other than GPL or LGPL is available it will
 * apply instead, Oracle elects to use only the Lesser General Public
 * License version 2.1 (LGPLv2) at this time for any software where
 * a choice of LGPL license versions is made available with the
 * language indicating that LGPLv2 or any later version may be used,
 * or where a choice of which version of the LGPL is applied is
 * otherwise unspecified.
 */

#ifndef ___VirtualBox_CXPCOM_h
#define ___VirtualBox_CXPCOM_h

#ifdef __cplusplus
# include "VirtualBox_XPCOM.h"
#else /* !__cplusplus */

#include &lt;stddef.h&gt;
#include "wchar.h"

#if defined(WIN32)

#define PR_EXPORT(__type) extern __declspec(dllexport) __type
#define PR_EXPORT_DATA(__type) extern __declspec(dllexport) __type
#define PR_IMPORT(__type) __declspec(dllimport) __type
#define PR_IMPORT_DATA(__type) __declspec(dllimport) __type

#define PR_EXTERN(__type) extern __declspec(dllexport) __type
#define PR_IMPLEMENT(__type) __declspec(dllexport) __type
#define PR_EXTERN_DATA(__type) extern __declspec(dllexport) __type
#define PR_IMPLEMENT_DATA(__type) __declspec(dllexport) __type

#define PR_CALLBACK
#define PR_CALLBACK_DECL
#define PR_STATIC_CALLBACK(__x) static __x

#elif defined(XP_BEOS)

#define PR_EXPORT(__type) extern __declspec(dllexport) __type
#define PR_EXPORT_DATA(__type) extern __declspec(dllexport) __type
#define PR_IMPORT(__type) extern __declspec(dllexport) __type
#define PR_IMPORT_DATA(__type) extern __declspec(dllexport) __type

#define PR_EXTERN(__type) extern __declspec(dllexport) __type
#define PR_IMPLEMENT(__type) __declspec(dllexport) __type
#define PR_EXTERN_DATA(__type) extern __declspec(dllexport) __type
#define PR_IMPLEMENT_DATA(__type) __declspec(dllexport) __type

#define PR_CALLBACK
#define PR_CALLBACK_DECL
#define PR_STATIC_CALLBACK(__x) static __x

#elif defined(WIN16)

#define PR_CALLBACK_DECL        __cdecl

#if defined(_WINDLL)
#define PR_EXPORT(__type) extern __type _cdecl _export _loadds
#define PR_IMPORT(__type) extern __type _cdecl _export _loadds
#define PR_EXPORT_DATA(__type) extern __type _export
#define PR_IMPORT_DATA(__type) extern __type _export

#define PR_EXTERN(__type) extern __type _cdecl _export _loadds
#define PR_IMPLEMENT(__type) __type _cdecl _export _loadds
#define PR_EXTERN_DATA(__type) extern __type _export
#define PR_IMPLEMENT_DATA(__type) __type _export

#define PR_CALLBACK             __cdecl __loadds
#define PR_STATIC_CALLBACK(__x) static __x PR_CALLBACK

#else /* this must be .EXE */
#define PR_EXPORT(__type) extern __type _cdecl _export
#define PR_IMPORT(__type) extern __type _cdecl _export
#define PR_EXPORT_DATA(__type) extern __type _export
#define PR_IMPORT_DATA(__type) extern __type _export

#define PR_EXTERN(__type) extern __type _cdecl _export
#define PR_IMPLEMENT(__type) __type _cdecl _export
#define PR_EXTERN_DATA(__type) extern __type _export
#define PR_IMPLEMENT_DATA(__type) __type _export

#define PR_CALLBACK             __cdecl __loadds
#define PR_STATIC_CALLBACK(__x) __x PR_CALLBACK
#endif /* _WINDLL */

#elif defined(XP_MAC)

#define PR_EXPORT(__type) extern __declspec(export) __type
#define PR_EXPORT_DATA(__type) extern __declspec(export) __type
#define PR_IMPORT(__type) extern __declspec(export) __type
#define PR_IMPORT_DATA(__type) extern __declspec(export) __type

#define PR_EXTERN(__type) extern __declspec(export) __type
#define PR_IMPLEMENT(__type) __declspec(export) __type
#define PR_EXTERN_DATA(__type) extern __declspec(export) __type
#define PR_IMPLEMENT_DATA(__type) __declspec(export) __type

#define PR_CALLBACK
#define PR_CALLBACK_DECL
#define PR_STATIC_CALLBACK(__x) static __x

#elif defined(XP_OS2) &amp;&amp; defined(__declspec)

#define PR_EXPORT(__type) extern __declspec(dllexport) __type
#define PR_EXPORT_DATA(__type) extern __declspec(dllexport) __type
#define PR_IMPORT(__type) __declspec(dllimport) __type
#define PR_IMPORT_DATA(__type) __declspec(dllimport) __type

#define PR_EXTERN(__type) extern __declspec(dllexport) __type
#define PR_IMPLEMENT(__type) __declspec(dllexport) __type
#define PR_EXTERN_DATA(__type) extern __declspec(dllexport) __type
#define PR_IMPLEMENT_DATA(__type) __declspec(dllexport) __type

#define PR_CALLBACK
#define PR_CALLBACK_DECL
#define PR_STATIC_CALLBACK(__x) static __x

#elif defined(XP_OS2_VACPP)

#define PR_EXPORT(__type) extern __type
#define PR_EXPORT_DATA(__type) extern __type
#define PR_IMPORT(__type) extern __type
#define PR_IMPORT_DATA(__type) extern __type

#define PR_EXTERN(__type) extern __type
#define PR_IMPLEMENT(__type) __type
#define PR_EXTERN_DATA(__type) extern __type
#define PR_IMPLEMENT_DATA(__type) __type
#define PR_CALLBACK _Optlink
#define PR_CALLBACK_DECL
#define PR_STATIC_CALLBACK(__x) static __x PR_CALLBACK

#else /* Unix */

# ifdef VBOX_HAVE_VISIBILITY_HIDDEN
#  define PR_EXPORT(__type) __attribute__((visibility("default"))) extern __type
#  define PR_EXPORT_DATA(__type) __attribute__((visibility("default"))) extern __type
#  define PR_IMPORT(__type) extern __type
#  define PR_IMPORT_DATA(__type) extern __type
#  define PR_EXTERN(__type) __attribute__((visibility("default"))) extern __type
#  define PR_IMPLEMENT(__type) __attribute__((visibility("default"))) __type
#  define PR_EXTERN_DATA(__type) __attribute__((visibility("default"))) extern __type
#  define PR_IMPLEMENT_DATA(__type) __attribute__((visibility("default"))) __type
#  define PR_CALLBACK
#  define PR_CALLBACK_DECL
#  define PR_STATIC_CALLBACK(__x) static __x
# else
#  define PR_EXPORT(__type) extern __type
#  define PR_EXPORT_DATA(__type) extern __type
#  define PR_IMPORT(__type) extern __type
#  define PR_IMPORT_DATA(__type) extern __type
#  define PR_EXTERN(__type) extern __type
#  define PR_IMPLEMENT(__type) __type
#  define PR_EXTERN_DATA(__type) extern __type
#  define PR_IMPLEMENT_DATA(__type) __type
#  define PR_CALLBACK
#  define PR_CALLBACK_DECL
#  define PR_STATIC_CALLBACK(__x) static __x
# endif
#endif

#if defined(_NSPR_BUILD_)
#define NSPR_API(__type) PR_EXPORT(__type)
#define NSPR_DATA_API(__type) PR_EXPORT_DATA(__type)
#else
#define NSPR_API(__type) PR_IMPORT(__type)
#define NSPR_DATA_API(__type) PR_IMPORT_DATA(__type)
#endif

typedef unsigned char PRUint8;
#if (defined(HPUX) &amp;&amp; defined(__cplusplus) \
        &amp;&amp; !defined(__GNUC__) &amp;&amp; __cplusplus &lt; 199707L) \
    || (defined(SCO) &amp;&amp; defined(__cplusplus) \
        &amp;&amp; !defined(__GNUC__) &amp;&amp; __cplusplus == 1L)
typedef char PRInt8;
#else
typedef signed char PRInt8;
#endif

#define PR_INT8_MAX 127
#define PR_INT8_MIN (-128)
#define PR_UINT8_MAX 255U

typedef unsigned short PRUint16;
typedef short PRInt16;

#define PR_INT16_MAX 32767
#define PR_INT16_MIN (-32768)
#define PR_UINT16_MAX 65535U

typedef unsigned int PRUint32;
typedef int PRInt32;
#define PR_INT32(x)  x
#define PR_UINT32(x) x ## U

#define PR_INT32_MAX PR_INT32(2147483647)
#define PR_INT32_MIN (-PR_INT32_MAX - 1)
#define PR_UINT32_MAX PR_UINT32(4294967295)

typedef long PRInt64;
typedef unsigned long PRUint64;
typedef int PRIntn;
typedef unsigned int PRUintn;

typedef double          PRFloat64;
typedef size_t PRSize;

typedef ptrdiff_t PRPtrdiff;

typedef unsigned long PRUptrdiff;

typedef PRIntn PRBool;

#define PR_TRUE 1
#define PR_FALSE 0

typedef PRUint8 PRPackedBool;

/*
** Status code used by some routines that have a single point of failure or
** special status return.
*/
typedef enum { PR_FAILURE = -1, PR_SUCCESS = 0 } PRStatus;

#ifndef __PRUNICHAR__
#define __PRUNICHAR__
#if defined(WIN32) || defined(XP_MAC)
typedef wchar_t PRUnichar;
#else
typedef PRUint16 PRUnichar;
#endif
#endif

typedef long PRWord;
typedef unsigned long PRUword;

#define nsnull 0
typedef PRUint32 nsresult;

#if defined(__GNUC__) &amp;&amp; (__GNUC__ > 2)
#define NS_LIKELY(x)    (__builtin_expect((x), 1))
#define NS_UNLIKELY(x)  (__builtin_expect((x), 0))
#else
#define NS_LIKELY(x)    (x)
#define NS_UNLIKELY(x)  (x)
#endif

#define NS_FAILED(_nsresult) (NS_UNLIKELY((_nsresult) &amp; 0x80000000))
#define NS_SUCCEEDED(_nsresult) (NS_LIKELY(!((_nsresult) &amp; 0x80000000)))

#ifdef VBOX_WITH_XPCOM_NAMESPACE_CLEANUP
# define PR_IntervalNow VBoxNsprPR_IntervalNow
# define PR_TicksPerSecond VBoxNsprPR_TicksPerSecond
# define PR_SecondsToInterval VBoxNsprPR_SecondsToInterval
# define PR_MillisecondsToInterval VBoxNsprPR_MillisecondsToInterval
# define PR_MicrosecondsToInterval VBoxNsprPR_MicrosecondsToInterval
# define PR_IntervalToSeconds VBoxNsprPR_IntervalToSeconds
# define PR_IntervalToMilliseconds VBoxNsprPR_IntervalToMilliseconds
# define PR_IntervalToMicroseconds VBoxNsprPR_IntervalToMicroseconds
# define PR_EnterMonitor VBoxNsprPR_EnterMonitor
# define PR_ExitMonitor VBoxNsprPR_ExitMonitor
# define PR_Notify VBoxNsprPR_Notify
# define PR_NotifyAll VBoxNsprPR_NotifyAll
# define PR_Wait VBoxNsprPR_Wait
# define PR_NewMonitor VBoxNsprPR_NewMonitor
# define PR_DestroyMonitor VBoxNsprPR_DestroyMonitor
#endif /* VBOX_WITH_XPCOM_NAMESPACE_CLEANUP */

typedef PRUint32 PRIntervalTime;

#define PR_INTERVAL_MIN 1000UL
#define PR_INTERVAL_MAX 100000UL
#define PR_INTERVAL_NO_WAIT 0UL
#define PR_INTERVAL_NO_TIMEOUT 0xffffffffUL

NSPR_API(PRIntervalTime) PR_IntervalNow(void);
NSPR_API(PRUint32) PR_TicksPerSecond(void);
NSPR_API(PRIntervalTime) PR_SecondsToInterval(PRUint32 seconds);
NSPR_API(PRIntervalTime) PR_MillisecondsToInterval(PRUint32 milli);
NSPR_API(PRIntervalTime) PR_MicrosecondsToInterval(PRUint32 micro);
NSPR_API(PRUint32) PR_IntervalToSeconds(PRIntervalTime ticks);
NSPR_API(PRUint32) PR_IntervalToMilliseconds(PRIntervalTime ticks);
NSPR_API(PRUint32) PR_IntervalToMicroseconds(PRIntervalTime ticks);

typedef struct PRMonitor PRMonitor;

NSPR_API(PRMonitor*) PR_NewMonitor(void);
NSPR_API(void) PR_DestroyMonitor(PRMonitor *mon);
NSPR_API(void) PR_EnterMonitor(PRMonitor *mon);
NSPR_API(PRStatus) PR_ExitMonitor(PRMonitor *mon);
NSPR_API(PRStatus) PR_Wait(PRMonitor *mon, PRIntervalTime ticks);
NSPR_API(PRStatus) PR_Notify(PRMonitor *mon);
NSPR_API(PRStatus) PR_NotifyAll(PRMonitor *mon);

#ifdef VBOX_WITH_XPCOM_NAMESPACE_CLEANUP
# define PR_CreateThread VBoxNsprPR_CreateThread
# define PR_JoinThread VBoxNsprPR_JoinThread
# define PR_Sleep VBoxNsprPR_Sleep
# define PR_GetCurrentThread VBoxNsprPR_GetCurrentThread
# define PR_GetThreadState VBoxNsprPR_GetThreadState
# define PR_SetThreadPrivate VBoxNsprPR_SetThreadPrivate
# define PR_GetThreadPrivate VBoxNsprPR_GetThreadPrivate
# define PR_NewThreadPrivateIndex VBoxNsprPR_NewThreadPrivateIndex
# define PR_GetThreadPriority VBoxNsprPR_GetThreadPriority
# define PR_SetThreadPriority VBoxNsprPR_SetThreadPriority
# define PR_Interrupt VBoxNsprPR_Interrupt
# define PR_ClearInterrupt VBoxNsprPR_ClearInterrupt
# define PR_BlockInterrupt VBoxNsprPR_BlockInterrupt
# define PR_UnblockInterrupt VBoxNsprPR_UnblockInterrupt
# define PR_GetThreadScope VBoxNsprPR_GetThreadScope
# define PR_GetThreadType VBoxNsprPR_GetThreadType
#endif /* VBOX_WITH_XPCOM_NAMESPACE_CLEANUP */

typedef struct PRThread PRThread;
typedef struct PRThreadStack PRThreadStack;

typedef enum PRThreadType {
    PR_USER_THREAD,
    PR_SYSTEM_THREAD
} PRThreadType;

typedef enum PRThreadScope {
    PR_LOCAL_THREAD,
    PR_GLOBAL_THREAD,
    PR_GLOBAL_BOUND_THREAD
} PRThreadScope;

typedef enum PRThreadState {
    PR_JOINABLE_THREAD,
    PR_UNJOINABLE_THREAD
} PRThreadState;

typedef enum PRThreadPriority
{
    PR_PRIORITY_FIRST = 0,      /* just a placeholder */
    PR_PRIORITY_LOW = 0,        /* the lowest possible priority */
    PR_PRIORITY_NORMAL = 1,     /* most common expected priority */
    PR_PRIORITY_HIGH = 2,       /* slightly more aggressive scheduling */
    PR_PRIORITY_URGENT = 3,     /* it does little good to have more than one */
    PR_PRIORITY_LAST = 3        /* this is just a placeholder */
} PRThreadPriority;

NSPR_API(PRThread*) PR_CreateThread(PRThreadType type,
                     void (PR_CALLBACK *start)(void *arg),
                     void *arg,
                     PRThreadPriority priority,
                     PRThreadScope scope,
                     PRThreadState state,
                     PRUint32 stackSize);
NSPR_API(PRStatus) PR_JoinThread(PRThread *thread);
NSPR_API(PRThread*) PR_GetCurrentThread(void);
#ifndef NO_NSPR_10_SUPPORT
#define PR_CurrentThread() PR_GetCurrentThread() /* for nspr1.0 compat. */
#endif /* NO_NSPR_10_SUPPORT */
NSPR_API(PRThreadPriority) PR_GetThreadPriority(const PRThread *thread);
NSPR_API(void) PR_SetThreadPriority(PRThread *thread, PRThreadPriority priority);

typedef void (PR_CALLBACK *PRThreadPrivateDTOR)(void *priv);

NSPR_API(PRStatus) PR_NewThreadPrivateIndex(
    PRUintn *newIndex, PRThreadPrivateDTOR destructor);
NSPR_API(PRStatus) PR_SetThreadPrivate(PRUintn tpdIndex, void *priv);
NSPR_API(void*) PR_GetThreadPrivate(PRUintn tpdIndex);
NSPR_API(PRStatus) PR_Interrupt(PRThread *thread);
NSPR_API(void) PR_ClearInterrupt(void);
NSPR_API(void) PR_BlockInterrupt(void);
NSPR_API(void) PR_UnblockInterrupt(void);
NSPR_API(PRStatus) PR_Sleep(PRIntervalTime ticks);
NSPR_API(PRThreadScope) PR_GetThreadScope(const PRThread *thread);
NSPR_API(PRThreadType) PR_GetThreadType(const PRThread *thread);
NSPR_API(PRThreadState) PR_GetThreadState(const PRThread *thread);

#ifdef VBOX_WITH_XPCOM_NAMESPACE_CLEANUP
# define PR_DestroyLock VBoxNsprPR_DestroyLock
# define PR_Lock VBoxNsprPR_Lock
# define PR_NewLock VBoxNsprPR_NewLock
# define PR_Unlock VBoxNsprPR_Unlock
#endif /* VBOX_WITH_XPCOM_NAMESPACE_CLEANUP */

typedef struct PRLock PRLock;

NSPR_API(PRLock*) PR_NewLock(void);
NSPR_API(void) PR_DestroyLock(PRLock *lock);
NSPR_API(void) PR_Lock(PRLock *lock);
NSPR_API(PRStatus) PR_Unlock(PRLock *lock);

#ifdef VBOX_WITH_XPCOM_NAMESPACE_CLEANUP
# define PR_NewCondVar VBoxNsprPR_NewCondVar
# define PR_DestroyCondVar VBoxNsprPR_DestroyCondVar
# define PR_WaitCondVar VBoxNsprPR_WaitCondVar
# define PR_NotifyCondVar VBoxNsprPR_NotifyCondVar
# define PR_NotifyAllCondVar VBoxNsprPR_NotifyAllCondVar
#endif /* VBOX_WITH_XPCOM_NAMESPACE_CLEANUP */

typedef struct PRCondVar PRCondVar;

NSPR_API(PRCondVar*) PR_NewCondVar(PRLock *lock);
NSPR_API(void) PR_DestroyCondVar(PRCondVar *cvar);
NSPR_API(PRStatus) PR_WaitCondVar(PRCondVar *cvar, PRIntervalTime timeout);
NSPR_API(PRStatus) PR_NotifyCondVar(PRCondVar *cvar);
NSPR_API(PRStatus) PR_NotifyAllCondVar(PRCondVar *cvar);

typedef struct PRCListStr PRCList;

struct PRCListStr {
    PRCList *next;
    PRCList *prev;
};

#ifdef VBOX_WITH_XPCOM_NAMESPACE_CLEANUP
# define PL_DestroyEvent VBoxNsplPL_DestroyEvent
# define PL_HandleEvent VBoxNsplPL_HandleEvent
# define PL_InitEvent VBoxNsplPL_InitEvent
# define PL_CreateEventQueue VBoxNsplPL_CreateEventQueue
# define PL_CreateMonitoredEventQueue VBoxNsplPL_CreateMonitoredEventQueue
# define PL_CreateNativeEventQueue VBoxNsplPL_CreateNativeEventQueue
# define PL_DequeueEvent VBoxNsplPL_DequeueEvent
# define PL_DestroyEventQueue VBoxNsplPL_DestroyEventQueue
# define PL_EventAvailable VBoxNsplPL_EventAvailable
# define PL_EventLoop VBoxNsplPL_EventLoop
# define PL_GetEvent VBoxNsplPL_GetEvent
# define PL_GetEventOwner VBoxNsplPL_GetEventOwner
# define PL_GetEventQueueMonitor VBoxNsplPL_GetEventQueueMonitor
# define PL_GetEventQueueSelectFD VBoxNsplPL_GetEventQueueSelectFD
# define PL_MapEvents VBoxNsplPL_MapEvents
# define PL_PostEvent VBoxNsplPL_PostEvent
# define PL_PostSynchronousEvent VBoxNsplPL_PostSynchronousEvent
# define PL_ProcessEventsBeforeID VBoxNsplPL_ProcessEventsBeforeID
# define PL_ProcessPendingEvents VBoxNsplPL_ProcessPendingEvents
# define PL_RegisterEventIDFunc VBoxNsplPL_RegisterEventIDFunc
# define PL_RevokeEvents VBoxNsplPL_RevokeEvents
# define PL_UnregisterEventIDFunc VBoxNsplPL_UnregisterEventIDFunc
# define PL_WaitForEvent VBoxNsplPL_WaitForEvent
# define PL_IsQueueNative VBoxNsplPL_IsQueueNative
# define PL_IsQueueOnCurrentThread VBoxNsplPL_IsQueueOnCurrentThread
# define PL_FavorPerformanceHint VBoxNsplPL_FavorPerformanceHint
#endif /* VBOX_WITH_XPCOM_NAMESPACE_CLEANUP */

typedef struct PLEvent PLEvent;
typedef struct PLEventQueue PLEventQueue;

PR_EXTERN(PLEventQueue*)
PL_CreateEventQueue(const char* name, PRThread* handlerThread);
PR_EXTERN(PLEventQueue *)
    PL_CreateNativeEventQueue(
        const char *name,
        PRThread *handlerThread
    );
PR_EXTERN(PLEventQueue *)
    PL_CreateMonitoredEventQueue(
        const char *name,
        PRThread *handlerThread
    );
PR_EXTERN(void)
PL_DestroyEventQueue(PLEventQueue* self);
PR_EXTERN(PRMonitor*)
PL_GetEventQueueMonitor(PLEventQueue* self);

#define PL_ENTER_EVENT_QUEUE_MONITOR(queue) \
    PR_EnterMonitor(PL_GetEventQueueMonitor(queue))

#define PL_EXIT_EVENT_QUEUE_MONITOR(queue)  \
    PR_ExitMonitor(PL_GetEventQueueMonitor(queue))

PR_EXTERN(PRStatus) PL_PostEvent(PLEventQueue* self, PLEvent* event);
PR_EXTERN(void*) PL_PostSynchronousEvent(PLEventQueue* self, PLEvent* event);
PR_EXTERN(PLEvent*) PL_GetEvent(PLEventQueue* self);
PR_EXTERN(PRBool) PL_EventAvailable(PLEventQueue* self);

typedef void (PR_CALLBACK *PLEventFunProc)(PLEvent* event, void* data, PLEventQueue* queue);

PR_EXTERN(void) PL_MapEvents(PLEventQueue* self, PLEventFunProc fun, void* data);
PR_EXTERN(void) PL_RevokeEvents(PLEventQueue* self, void* owner);
PR_EXTERN(void) PL_ProcessPendingEvents(PLEventQueue* self);
PR_EXTERN(PLEvent*) PL_WaitForEvent(PLEventQueue* self);
PR_EXTERN(void) PL_EventLoop(PLEventQueue* self);
PR_EXTERN(PRInt32) PL_GetEventQueueSelectFD(PLEventQueue* self);
PR_EXTERN(PRBool) PL_IsQueueOnCurrentThread( PLEventQueue *queue );
PR_EXTERN(PRBool) PL_IsQueueNative(PLEventQueue *queue);

typedef void* (PR_CALLBACK *PLHandleEventProc)(PLEvent* self);
typedef void (PR_CALLBACK *PLDestroyEventProc)(PLEvent* self);
PR_EXTERN(void)
PL_InitEvent(PLEvent* self, void* owner,
             PLHandleEventProc handler,
             PLDestroyEventProc destructor);
PR_EXTERN(void*) PL_GetEventOwner(PLEvent* self);
PR_EXTERN(void) PL_HandleEvent(PLEvent* self);
PR_EXTERN(void) PL_DestroyEvent(PLEvent* self);
PR_EXTERN(void) PL_DequeueEvent(PLEvent* self, PLEventQueue* queue);
PR_EXTERN(void) PL_FavorPerformanceHint(PRBool favorPerformanceOverEventStarvation, PRUint32 starvationDelay);

struct PLEvent {
    PRCList             link;
    PLHandleEventProc   handler;
    PLDestroyEventProc  destructor;
    void*               owner;
    void*               synchronousResult;
    PRLock*             lock;
    PRCondVar*          condVar;
    PRBool              handled;
#ifdef PL_POST_TIMINGS
    PRIntervalTime      postTime;
#endif
#ifdef XP_UNIX
    unsigned long       id;
#endif /* XP_UNIX */
    /* other fields follow... */
};

#if defined(XP_WIN) || defined(XP_OS2)

PR_EXTERN(HWND)
    PL_GetNativeEventReceiverWindow(
        PLEventQueue *eqp
    );
#endif /* XP_WIN || XP_OS2 */

#ifdef XP_UNIX

PR_EXTERN(PRInt32)
PL_ProcessEventsBeforeID(PLEventQueue *aSelf, unsigned long aID);

typedef unsigned long (PR_CALLBACK *PLGetEventIDFunc)(void *aClosure);

PR_EXTERN(void)
PL_RegisterEventIDFunc(PLEventQueue *aSelf, PLGetEventIDFunc aFunc,
                       void *aClosure);
PR_EXTERN(void) PL_UnregisterEventIDFunc(PLEventQueue *aSelf);

#endif /* XP_UNIX */

/* Standard "it worked" return value */
#define NS_OK                              0

#define NS_ERROR_BASE                      ((nsresult) 0xC1F30000)

/* Returned when an instance is not initialized */
#define NS_ERROR_NOT_INITIALIZED           (NS_ERROR_BASE + 1)

/* Returned when an instance is already initialized */
#define NS_ERROR_ALREADY_INITIALIZED       (NS_ERROR_BASE + 2)

/* Returned by a not implemented function */
#define NS_ERROR_NOT_IMPLEMENTED           ((nsresult) 0x80004001L)

/* Returned when a given interface is not supported. */
#define NS_NOINTERFACE                     ((nsresult) 0x80004002L)
#define NS_ERROR_NO_INTERFACE              NS_NOINTERFACE

#define NS_ERROR_INVALID_POINTER           ((nsresult) 0x80004003L)
#define NS_ERROR_NULL_POINTER              NS_ERROR_INVALID_POINTER

/* Returned when a function aborts */
#define NS_ERROR_ABORT                     ((nsresult) 0x80004004L)

/* Returned when a function fails */
#define NS_ERROR_FAILURE                   ((nsresult) 0x80004005L)

/* Returned when an unexpected error occurs */
#define NS_ERROR_UNEXPECTED                ((nsresult) 0x8000ffffL)

/* Returned when a memory allocation fails */
#define NS_ERROR_OUT_OF_MEMORY             ((nsresult) 0x8007000eL)

/* Returned when an illegal value is passed */
#define NS_ERROR_ILLEGAL_VALUE             ((nsresult) 0x80070057L)
#define NS_ERROR_INVALID_ARG               NS_ERROR_ILLEGAL_VALUE

/* Returned when a class doesn't allow aggregation */
#define NS_ERROR_NO_AGGREGATION            ((nsresult) 0x80040110L)

/* Returned when an operation can't complete due to an unavailable resource */
#define NS_ERROR_NOT_AVAILABLE             ((nsresult) 0x80040111L)

/* Returned when a class is not registered */
#define NS_ERROR_FACTORY_NOT_REGISTERED    ((nsresult) 0x80040154L)

/* Returned when a class cannot be registered, but may be tried again later */
#define NS_ERROR_FACTORY_REGISTER_AGAIN    ((nsresult) 0x80040155L)

/* Returned when a dynamically loaded factory couldn't be found */
#define NS_ERROR_FACTORY_NOT_LOADED        ((nsresult) 0x800401f8L)

/* Returned when a factory doesn't support signatures */
#define NS_ERROR_FACTORY_NO_SIGNATURE_SUPPORT \
                                           (NS_ERROR_BASE + 0x101)

/* Returned when a factory already is registered */
#define NS_ERROR_FACTORY_EXISTS            (NS_ERROR_BASE + 0x100)


/**
 * An "interface id" which can be used to uniquely identify a given
 * interface.
 * A "unique identifier". This is modeled after OSF DCE UUIDs.
 */

struct nsID {
  PRUint32 m0;
  PRUint16 m1;
  PRUint16 m2;
  PRUint8 m3[8];
};

typedef struct nsID nsID;
typedef nsID nsIID;

struct nsISupports;   /* forward declaration */
struct nsIStackFrame; /* forward declaration */
struct nsIException;  /* forward declaration */
typedef struct nsISupports nsISupports;     /* forward declaration */
typedef struct nsIStackFrame nsIStackFrame; /* forward declaration */
typedef struct nsIException nsIException;   /* forward declaration */

/**
 * IID for the nsISupports interface
 * {00000000-0000-0000-c000-000000000046}
 *
 * To maintain binary compatibility with COM's IUnknown, we define the IID
 * of nsISupports to be the same as that of COM's IUnknown.
 */
#define NS_ISUPPORTS_IID                                                      \
  { 0x00000000, 0x0000, 0x0000,                                               \
    {0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x46} }

/**
 * Reference count values
 *
 * This is the return type for AddRef() and Release() in nsISupports.
 * IUnknown of COM returns an unsigned long from equivalent functions.
 * The following ifdef exists to maintain binary compatibility with
 * IUnknown.
 */

/**
 * Basic component object model interface. Objects which implement
 * this interface support runtime interface discovery (QueryInterface)
 * and a reference counted memory model (AddRef/Release). This is
 * modelled after the win32 IUnknown API.
 */
struct nsISupports_vtbl {

  /**
   * @name Methods
   */

  /**
   * A run time mechanism for interface discovery.
   * @param aIID         [in]  A requested interface IID
   * @param aInstancePtr [out] A pointer to an interface pointer to
   *                           receive the result.
   * @return            NS_OK if the interface is supported by the associated
   *                          instance, NS_NOINTERFACE if it is not.
   * NS_ERROR_INVALID_POINTER if aInstancePtr is NULL.
   */
  nsresult (*QueryInterface)(nsISupports *pThis, const nsID *iid, void **resultp);
  /**
   * Increases the reference count for this interface.
   * The associated instance will not be deleted unless
   * the reference count is returned to zero.
   *
   * @return The resulting reference count.
   */
  nsresult (*AddRef)(nsISupports *pThis);

  /**
   * Decreases the reference count for this interface.
   * Generally, if the reference count returns to zero,
   * the associated instance is deleted.
   *
   * @return The resulting reference count.
   */
  nsresult (*Release)(nsISupports *pThis);

};

struct nsISupports {
    struct nsISupports_vtbl *vtbl;
};

/* starting interface:    nsIException */
#define NS_IEXCEPTION_IID_STR "f3a8d3b4-c424-4edc-8bf6-8974c983ba78"

#define NS_IEXCEPTION_IID \
  {0xf3a8d3b4, 0xc424, 0x4edc, \
    { 0x8b, 0xf6, 0x89, 0x74, 0xc9, 0x83, 0xba, 0x78 }}

struct nsIException_vtbl {

  /* Methods from the Class nsISupports */
  struct nsISupports_vtbl nsisupports;

  /* readonly attribute string message; */
  nsresult (*GetMessage)(nsIException *pThis, PRUnichar * *aMessage);

  /* readonly attribute nsresult (*result; */
  nsresult (*GetResult)(nsIException *pThis, nsresult *aResult);

  /* readonly attribute string name; */
  nsresult (*GetName)(nsIException *pThis, PRUnichar * *aName);

  /* readonly attribute string filename; */
  nsresult (*GetFilename)(nsIException *pThis, PRUnichar * *aFilename);

  /* readonly attribute PRUint32 lineNumber; */
  nsresult (*GetLineNumber)(nsIException *pThis, PRUint32 *aLineNumber);

  /* readonly attribute PRUint32 columnNumber; */
  nsresult (*GetColumnNumber)(nsIException *pThis, PRUint32 *aColumnNumber);

  /* readonly attribute nsIStackFrame location; */
  nsresult (*GetLocation)(nsIException *pThis, nsIStackFrame * *aLocation);

  /* readonly attribute nsIException inner; */
  nsresult (*GetInner)(nsIException *pThis, nsIException * *aInner);

  /* readonly attribute nsISupports data; */
  nsresult (*GetData)(nsIException *pThis, nsISupports * *aData);

  /* string toString (); */
  nsresult (*ToString)(nsIException *pThis, PRUnichar **_retval);
};

struct nsIException {
    struct nsIException_vtbl *vtbl;
};

/* starting interface:    nsIStackFrame */
#define NS_ISTACKFRAME_IID_STR "91d82105-7c62-4f8b-9779-154277c0ee90"

#define NS_ISTACKFRAME_IID \
  {0x91d82105, 0x7c62, 0x4f8b, \
    { 0x97, 0x79, 0x15, 0x42, 0x77, 0xc0, 0xee, 0x90 }}

struct nsIStackFrame_vtbl {

  /* Methods from the Class nsISupports */
  struct nsISupports_vtbl nsisupports;

  /* readonly attribute PRUint32 language; */
  nsresult (*GetLanguage)(nsIStackFrame *pThis, PRUint32 *aLanguage);

  /* readonly attribute string languageName; */
  nsresult (*GetLanguageName)(nsIStackFrame *pThis, PRUnichar * *aLanguageName);

  /* readonly attribute string filename; */
  nsresult (*GetFilename)(nsIStackFrame *pThis, PRUnichar * *aFilename);

  /* readonly attribute string name; */
  nsresult (*GetName)(nsIStackFrame *pThis, PRUnichar * *aName);

  /* readonly attribute PRInt32 lineNumber; */
  nsresult (*GetLineNumber)(nsIStackFrame *pThis, PRInt32 *aLineNumber);

  /* readonly attribute string sourceLine; */
  nsresult (*GetSourceLine)(nsIStackFrame *pThis, PRUnichar * *aSourceLine);

  /* readonly attribute nsIStackFrame caller; */
  nsresult (*GetCaller)(nsIStackFrame *pThis, nsIStackFrame * *aCaller);

  /* string toString (); */
  nsresult (*ToString)(nsIStackFrame *pThis, PRUnichar **_retval);
};

struct nsIStackFrame {
    struct nsIStackFrame_vtbl *vtbl;
};

/* starting interface:    nsIEventTarget */
#define NS_IEVENTTARGET_IID_STR "ea99ad5b-cc67-4efb-97c9-2ef620a59f2a"

#define NS_IEVENTTARGET_IID \
  {0xea99ad5b, 0xcc67, 0x4efb, \
    { 0x97, 0xc9, 0x2e, 0xf6, 0x20, 0xa5, 0x9f, 0x2a }}

struct nsIEventTarget;
typedef struct nsIEventTarget nsIEventTarget;

struct nsIEventTarget_vtbl {

    struct nsISupports_vtbl nsisupports;

    nsresult (*PostEvent)(nsIEventTarget *pThis, PLEvent * aEvent);

    nsresult (*IsOnCurrentThread)(nsIEventTarget *pThis, PRBool *_retval);

};

struct nsIEventTarget {
    struct nsIEventTarget_vtbl *vtbl;
};

/* starting interface:    nsIEventQueue */
#define NS_IEVENTQUEUE_IID_STR "176afb41-00a4-11d3-9f2a-00400553eef0"

#define NS_IEVENTQUEUE_IID \
  {0x176afb41, 0x00a4, 0x11d3, \
    { 0x9f, 0x2a, 0x00, 0x40, 0x05, 0x53, 0xee, 0xf0 }}

struct nsIEventQueue;
typedef struct nsIEventQueue nsIEventQueue;

struct nsIEventQueue_vtbl {

    struct nsIEventTarget_vtbl nsieventtarget;

    nsresult (*InitEvent)(nsIEventQueue *pThis, PLEvent * aEvent, void * owner, PLHandleEventProc handler, PLDestroyEventProc destructor);

    nsresult (*PostSynchronousEvent)(nsIEventQueue *pThis, PLEvent * aEvent, void * *aResult);

    nsresult (*PendingEvents)(nsIEventQueue *pThis, PRBool *_retval);

    nsresult (*ProcessPendingEvents)(nsIEventQueue *pThis);

    nsresult (*EventLoop)(nsIEventQueue *pThis);

    nsresult (*EventAvailable)(nsIEventQueue *pThis, PRBool *aResult);

    nsresult (*GetEvent)(nsIEventQueue *pThis, PLEvent * *_retval);

    nsresult (*HandleEvent)(nsIEventQueue *pThis, PLEvent * aEvent);

    nsresult (*WaitForEvent)(nsIEventQueue *pThis, PLEvent * *_retval);

    PRInt32 (*GetEventQueueSelectFD)(nsIEventQueue *pThis);

    nsresult (*Init)(nsIEventQueue *pThis, PRBool aNative);

    nsresult (*InitFromPRThread)(nsIEventQueue *pThis, PRThread * thread, PRBool aNative);

    nsresult (*InitFromPLQueue)(nsIEventQueue *pThis, PLEventQueue * aQueue);

    nsresult (*EnterMonitor)(nsIEventQueue *pThis);

    nsresult (*ExitMonitor)(nsIEventQueue *pThis);

    nsresult (*RevokeEvents)(nsIEventQueue *pThis, void * owner);

    nsresult (*GetPLEventQueue)(nsIEventQueue *pThis, PLEventQueue * *_retval);

    nsresult (*IsQueueNative)(nsIEventQueue *pThis, PRBool *_retval);

    nsresult (*StopAcceptingEvents)(nsIEventQueue *pThis);

};

struct nsIEventQueue {
    struct nsIEventQueue_vtbl *vtbl;
};

</xsl:text>
 <xsl:apply-templates/>
<xsl:text>
#endif /* !__cplusplus */

#ifdef IN_VBOXXPCOMC
# define VBOXXPCOMC_DECL(type)  PR_EXPORT(type)
#else
# define VBOXXPCOMC_DECL(type)  PR_IMPORT(type)
#endif

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Function table for dynamic linking.
 * Use VBoxGetFunctions() to obtain the pointer to it.
 */
typedef struct VBOXXPCOMC
{
    /** The size of the structure. */
    unsigned cb;
    /** The structure version. */
    unsigned uVersion;

    unsigned int (*pfnGetVersion)(void);

    void  (*pfnComInitialize)(const char *pszVirtualBoxIID,
                              IVirtualBox **ppVirtualBox,
                              const char *pszSessionIID,
                              ISession **ppSession);
    void (*pfnComUninitialize)(void);

    void  (*pfnComUnallocMem)(void *pv);
    void  (*pfnUtf16Free)(PRUnichar *pwszString);
    void  (*pfnUtf8Free)(char *pszString);

    int   (*pfnUtf16ToUtf8)(const PRUnichar *pwszString, char **ppszString);
    int   (*pfnUtf8ToUtf16)(const char *pszString, PRUnichar **ppwszString);

    void  (*pfnGetEventQueue)(nsIEventQueue **eventQueue);

    /** Tail version, same as uVersion. */
    unsigned uEndVersion;
} VBOXXPCOMC;
/** Pointer to a const VBoxXPCOMC function table. */
typedef VBOXXPCOMC const *PCVBOXXPCOM;

/** The current interface version.
 * For use with VBoxGetXPCOMCFunctions and to be found in
 * VBOXXPCOMC::uVersion. */
#define VBOX_XPCOMC_VERSION     0x00020000U

VBOXXPCOMC_DECL(PCVBOXXPCOM) VBoxGetXPCOMCFunctions(unsigned uVersion);
/** Typedef for VBoxGetXPCOMCFunctions. */
typedef PCVBOXXPCOM (*PFNVBOXGETXPCOMCFUNCTIONS)(unsigned uVersion);

/** The symbol name of VBoxGetXPCOMCFunctions. */
#if defined(__OS2__)
# define VBOX_GET_XPCOMC_FUNCTIONS_SYMBOL_NAME   "_VBoxGetXPCOMCFunctions"
#else
# define VBOX_GET_XPCOMC_FUNCTIONS_SYMBOL_NAME   "VBoxGetXPCOMCFunctions"
#endif


#ifdef __cplusplus
}
#endif

#endif /* !___VirtualBox_CXPCOM_h */
</xsl:text>
</xsl:template>

<!--
 *  ignore all |if|s except those for XPIDL target
<xsl:template match="if">
  <xsl:if test="@target='xpidl'">
    <xsl:apply-templates/>
  </xsl:if>
</xsl:template>
<xsl:template match="if" mode="forward">
  <xsl:if test="@target='xpidl'">
    <xsl:apply-templates mode="forward"/>
  </xsl:if>
</xsl:template>
<xsl:template match="if" mode="forwarder">
  <xsl:if test="@target='midl'">
    <xsl:apply-templates mode="forwarder"/>
  </xsl:if>
</xsl:template>

-->

<!--
 *  cpp_quote
<xsl:template match="cpp">
  <xsl:if test="text()">
    <xsl:text>%{C++</xsl:text>
    <xsl:value-of select="text()"/>
    <xsl:text>&#x0A;%}&#x0A;&#x0A;</xsl:text>
  </xsl:if>
  <xsl:if test="not(text()) and @line">
    <xsl:text>%{C++&#x0A;</xsl:text>
    <xsl:value-of select="@line"/>
    <xsl:text>&#x0A;%}&#x0A;&#x0A;</xsl:text>
  </xsl:if>
</xsl:template>
-->


<!--
 *  #if statement (@if attribute)
 *  @note
 *      xpidl doesn't support any preprocessor defines other than #include
 *      (it just ignores them), so the generated IDL will most likely be
 *      invalid. So for now we forbid using @if attributes
-->
<xsl:template match="@if" mode="begin">
  <xsl:message terminate="yes">
    @if attributes are not currently allowed because xpidl lacks
    support for #ifdef and stuff.
  </xsl:message>
  <xsl:text>#if </xsl:text>
  <xsl:value-of select="."/>
  <xsl:text>&#x0A;</xsl:text>
</xsl:template>
<xsl:template match="@if" mode="end">
  <xsl:text>#endif&#x0A;</xsl:text>
</xsl:template>


<!--
 *  libraries
-->
<xsl:template match="library">
  <!-- result codes -->
  <xsl:text>&#x0A;</xsl:text>
  <xsl:for-each select="result">
    <xsl:apply-templates select="."/>
  </xsl:for-each>
  <xsl:text>&#x0A;&#x0A;</xsl:text>
  <!-- forward declarations -->
  <xsl:apply-templates select="if | interface" mode="forward"/>
  <xsl:text>&#x0A;</xsl:text>
  <!-- typedef'ing the struct declarations -->
  <xsl:apply-templates select="if | interface" mode="typedef"/>
  <xsl:text>&#x0A;</xsl:text>
  <!-- all enums go first -->
  <xsl:apply-templates select="enum | if/enum"/>
  <!-- everything else but result codes and enums -->
  <xsl:apply-templates select="*[not(self::result or self::enum) and
                                 not(self::if[result] or self::if[enum])]"/>
  <!-- -->
</xsl:template>


<!--
 *  result codes
-->
<xsl:template match="result">
  <xsl:value-of select="concat('#define ',@name,' ',@value)"/>
  <xsl:text>&#x0A;</xsl:text>
</xsl:template>


<!--
 *  forward declarations
-->
<xsl:template match="interface" mode="forward">
  <xsl:text>struct </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text>;&#x0A;</xsl:text>
</xsl:template>


<!--
 *  typedef'ing the struct declarations
-->
<xsl:template match="interface" mode="typedef">
  <xsl:text>typedef struct </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text> </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text>;&#x0A;</xsl:text>
</xsl:template>


<!--
 *  interfaces
-->
<xsl:template match="interface">
  <xsl:text>/* Start of struct </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text> Declaration */&#x0A;</xsl:text>
  <xsl:text>#define </xsl:text>
  <xsl:call-template name="uppercase">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:value-of select="concat('_IID_STR &quot;',@uuid,'&quot;')"/>
  <xsl:text>&#x0A;</xsl:text>
  <xsl:text>#define </xsl:text>
  <xsl:call-template name="uppercase">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text>_IID { \&#x0A;</xsl:text>
  <xsl:text>    0x</xsl:text><xsl:value-of select="substring(@uuid,1,8)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,10,4)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,15,4)"/>
  <xsl:text>, \&#x0A;    </xsl:text>
  <xsl:text>{ 0x</xsl:text><xsl:value-of select="substring(@uuid,20,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,22,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,25,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,27,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,29,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,31,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,33,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,35,2)"/>
  <xsl:text> } \&#x0A;}&#x0A;</xsl:text>
  <xsl:text>struct </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text>_vtbl&#x0A;{&#x0A;</xsl:text>
  <xsl:text>    </xsl:text>
  <xsl:choose>
    <xsl:when test="@extends='$unknown'">struct nsISupports_vtbl nsisupports;</xsl:when>
    <xsl:when test="@extends='$dispatched'">struct nsISupports_vtbl nsisupports;</xsl:when>
    <xsl:when test="@extends='$errorinfo'">struct nsIException_vtbl nsiexception;</xsl:when>
    <xsl:otherwise>
      <xsl:text>struct </xsl:text>
      <xsl:value-of select="@extends"/>
      <xsl:text>_vtbl </xsl:text>
      <xsl:call-template name="lowercase">
        <xsl:with-param name="str" select="@extends"/>
      </xsl:call-template>
      <xsl:text>;</xsl:text>
    </xsl:otherwise>
  </xsl:choose>
  <xsl:text>&#x0A;&#x0A;</xsl:text>
  <!-- attributes (properties) -->
  <xsl:apply-templates select="attribute"/>
  <!-- methods -->
  <xsl:apply-templates select="method"/>
  <!-- 'if' enclosed elements, unsorted -->
  <xsl:apply-templates select="if"/>
  <!-- -->
  <xsl:text>};</xsl:text>
  <xsl:text>&#x0A;&#x0A;</xsl:text>
  <xsl:text>struct </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text>&#x0A;{&#x0A;    struct </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text>_vtbl *vtbl;&#x0A;};&#x0A;</xsl:text>
  <xsl:text>/* End of struct </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text> Declaration */&#x0A;&#x0A;&#x0A;</xsl:text>
</xsl:template>


<!--
 *  attributes
-->
<xsl:template match="interface//attribute">
  <xsl:apply-templates select="@if" mode="begin"/>
  <xsl:if test="@mod='ptr'">
    <!-- attributes using native types must be non-scriptable
    <xsl:text>    [noscript]&#x0A;</xsl:text>-->
  </xsl:if>
  <xsl:choose>
    <!-- safearray pseudo attribute -->
    <xsl:when test="@safearray='yes'">
      <!-- getter -->
      <xsl:text>    nsresult (*Get</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text>)(</xsl:text>
      <xsl:value-of select="../@name" />
      <xsl:text> *pThis, </xsl:text>
      <!-- array size -->
      <xsl:text>PRUint32 *</xsl:text>
      <xsl:value-of select="@name"/>
      <xsl:text>Size, </xsl:text>
      <!-- array pointer -->
      <xsl:apply-templates select="@type" mode="forwarder"/>
      <xsl:text> **</xsl:text>
      <xsl:value-of select="@name"/>
      <xsl:text>);&#x0A;</xsl:text>
      <!-- setter -->
      <xsl:if test="not(@readonly='yes')">
        <xsl:text>    nsresult (*Set</xsl:text>
        <xsl:call-template name="capitalize">
          <xsl:with-param name="str" select="@name"/>
        </xsl:call-template>
        <xsl:text>)(</xsl:text>
        <xsl:value-of select="../@name" />
        <xsl:text> *pThis, </xsl:text>
        <!-- array size -->
        <xsl:text>PRUint32 </xsl:text>
        <xsl:value-of select="@name"/>
        <xsl:text>Size, </xsl:text>
        <!-- array pointer -->
        <xsl:apply-templates select="@type" mode="forwarder"/>
        <xsl:text> *</xsl:text>
        <xsl:value-of select="@name"/>
        <xsl:text>);&#x0A;</xsl:text>
      </xsl:if>
    </xsl:when>
    <!-- normal attribute -->
    <xsl:otherwise>
      <xsl:text>    </xsl:text>
      <xsl:if test="@readonly='yes'">
        <xsl:text>nsresult (*Get</xsl:text>
        <xsl:call-template name="capitalize">
          <xsl:with-param name="str" select="@name"/>
        </xsl:call-template>
        <xsl:text>)(</xsl:text>
        <xsl:value-of select="../@name" />
        <xsl:text> *pThis, </xsl:text>
        <xsl:apply-templates select="@type" mode="forwarder"/>
        <xsl:text> *</xsl:text>
        <xsl:value-of select="@name"/>
        <xsl:text>);&#x0A;</xsl:text>
      </xsl:if>
      <xsl:choose>
        <xsl:when test="@readonly='yes'">
        </xsl:when>
        <xsl:otherwise>
          <xsl:text>nsresult (*Get</xsl:text>
          <xsl:call-template name="capitalize">
            <xsl:with-param name="str" select="@name"/>
          </xsl:call-template>
          <xsl:text>)(</xsl:text>
          <xsl:value-of select="../@name" />
          <xsl:text> *pThis, </xsl:text>
          <xsl:apply-templates select="@type" mode="forwarder"/>
          <xsl:text> *</xsl:text>
          <xsl:value-of select="@name"/>
          <xsl:text>);&#x0A;    </xsl:text>
          <xsl:text>nsresult (*Set</xsl:text>
          <xsl:call-template name="capitalize">
            <xsl:with-param name="str" select="@name"/>
          </xsl:call-template>
          <xsl:text>)(</xsl:text>
          <xsl:value-of select="../@name" />
          <xsl:text> *pThis, </xsl:text>
          <xsl:apply-templates select="@type" mode="forwarder"/>
          <xsl:text> </xsl:text>
          <xsl:value-of select="@name"/>
          <xsl:text>);&#x0A;</xsl:text>
        </xsl:otherwise>
      </xsl:choose>
    </xsl:otherwise>
  </xsl:choose>
  <xsl:apply-templates select="@if" mode="end"/>
  <xsl:text>&#x0A;</xsl:text>
</xsl:template>

<xsl:template match="interface//attribute" mode="forwarder">

  <xsl:variable name="parent" select="ancestor::interface"/>

  <xsl:apply-templates select="@if" mode="begin"/>

  <!-- getter: COM_FORWARD_Interface_GETTER_Name_TO(smth) -->
  <xsl:text>#define COM_FORWARD_</xsl:text>
  <xsl:value-of select="$parent/@name"/>
  <xsl:text>_GETTER_</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text>_TO(smth) NS_IMETHOD Get</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text> (</xsl:text>
  <xsl:if test="@safearray='yes'">
    <xsl:text>PRUint32 * a</xsl:text>
    <xsl:call-template name="capitalize">
      <xsl:with-param name="str" select="@name"/>
    </xsl:call-template>
    <xsl:text>Size, </xsl:text>
  </xsl:if>
  <xsl:apply-templates select="@type" mode="forwarder"/>
  <xsl:if test="@safearray='yes'">
    <xsl:text> *</xsl:text>
  </xsl:if>
  <xsl:text> * a</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text>) { return smth Get</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text> (</xsl:text>
  <xsl:if test="@safearray='yes'">
    <xsl:text>a</xsl:text>
    <xsl:call-template name="capitalize">
      <xsl:with-param name="str" select="@name"/>
    </xsl:call-template>
    <xsl:text>Size, </xsl:text>
  </xsl:if>
  <xsl:text>a</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text>); }&#x0A;</xsl:text>
  <!-- getter: COM_FORWARD_Interface_GETTER_Name_TO_OBJ(obj) -->
  <xsl:text>#define COM_FORWARD_</xsl:text>
  <xsl:value-of select="$parent/@name"/>
  <xsl:text>_GETTER_</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text>_TO_OBJ(obj) COM_FORWARD_</xsl:text>
  <xsl:value-of select="$parent/@name"/>
  <xsl:text>_GETTER_</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text>_TO ((obj)->)&#x0A;</xsl:text>
  <!-- getter: COM_FORWARD_Interface_GETTER_Name_TO_BASE(base) -->
  <xsl:text>#define COM_FORWARD_</xsl:text>
  <xsl:value-of select="$parent/@name"/>
  <xsl:text>_GETTER_</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text>_TO_BASE(base) COM_FORWARD_</xsl:text>
  <xsl:value-of select="$parent/@name"/>
  <xsl:text>_GETTER_</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text>_TO (base::)&#x0A;</xsl:text>
  <!-- -->
  <xsl:if test="not(@readonly='yes')">
    <!-- setter: COM_FORWARD_Interface_SETTER_Name_TO(smth) -->
    <xsl:text>#define COM_FORWARD_</xsl:text>
    <xsl:value-of select="$parent/@name"/>
    <xsl:text>_SETTER_</xsl:text>
    <xsl:call-template name="capitalize">
      <xsl:with-param name="str" select="@name"/>
    </xsl:call-template>
    <xsl:text>_TO(smth) NS_IMETHOD Set</xsl:text>
    <xsl:call-template name="capitalize">
      <xsl:with-param name="str" select="@name"/>
    </xsl:call-template>
    <xsl:text> (</xsl:text>
    <xsl:if test="@safearray='yes'">
      <xsl:text>PRUint32 a</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text>Size, </xsl:text>
    </xsl:if>
    <xsl:if test="not(@safearray='yes') and (@type='string' or @type='wstring')">
      <xsl:text>const </xsl:text>
    </xsl:if>
    <xsl:apply-templates select="@type" mode="forwarder"/>
    <xsl:if test="@safearray='yes'">
      <xsl:text> *</xsl:text>
    </xsl:if>
    <xsl:text> a</xsl:text>
    <xsl:call-template name="capitalize">
      <xsl:with-param name="str" select="@name"/>
    </xsl:call-template>
    <xsl:text>) { return smth Set</xsl:text>
    <xsl:call-template name="capitalize">
      <xsl:with-param name="str" select="@name"/>
    </xsl:call-template>
    <xsl:text> (a</xsl:text>
    <xsl:call-template name="capitalize">
      <xsl:with-param name="str" select="@name"/>
    </xsl:call-template>
    <xsl:text>); }&#x0A;</xsl:text>
    <!-- setter: COM_FORWARD_Interface_SETTER_Name_TO_OBJ(obj) -->
    <xsl:text>#define COM_FORWARD_</xsl:text>
    <xsl:value-of select="$parent/@name"/>
    <xsl:text>_SETTER_</xsl:text>
    <xsl:call-template name="capitalize">
      <xsl:with-param name="str" select="@name"/>
    </xsl:call-template>
    <xsl:text>_TO_OBJ(obj) COM_FORWARD_</xsl:text>
    <xsl:value-of select="$parent/@name"/>
    <xsl:text>_SETTER_</xsl:text>
    <xsl:call-template name="capitalize">
      <xsl:with-param name="str" select="@name"/>
    </xsl:call-template>
    <xsl:text>_TO ((obj)->)&#x0A;</xsl:text>
    <!-- setter: COM_FORWARD_Interface_SETTER_Name_TO_BASE(base) -->
    <xsl:text>#define COM_FORWARD_</xsl:text>
    <xsl:value-of select="$parent/@name"/>
    <xsl:text>_SETTER_</xsl:text>
    <xsl:call-template name="capitalize">
      <xsl:with-param name="str" select="@name"/>
    </xsl:call-template>
    <xsl:text>_TO_BASE(base) COM_FORWARD_</xsl:text>
    <xsl:value-of select="$parent/@name"/>
    <xsl:text>_SETTER_</xsl:text>
    <xsl:call-template name="capitalize">
      <xsl:with-param name="str" select="@name"/>
    </xsl:call-template>
    <xsl:text>_TO (base::)&#x0A;</xsl:text>
  </xsl:if>

  <xsl:apply-templates select="@if" mode="end"/>

</xsl:template>


<!--
 *  methods
-->
<xsl:template match="interface//method">
  <xsl:apply-templates select="@if" mode="begin"/>
  <xsl:if test="param/@mod='ptr'">
    <!-- methods using native types must be non-scriptable
    <xsl:text>    [noscript]&#x0A;</xsl:text>-->
  </xsl:if>
  <xsl:text>    nsresult (*</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:if test="param">
    <xsl:text>)(&#x0A;</xsl:text>
    <xsl:text>        </xsl:text>
    <xsl:value-of select="../@name" />
    <xsl:text> *pThis,&#x0A;</xsl:text>
    <xsl:for-each select="param [position() != last()]">
      <xsl:text>        </xsl:text>
      <xsl:apply-templates select="."/>
      <xsl:text>,&#x0A;</xsl:text>
    </xsl:for-each>
    <xsl:text>        </xsl:text>
    <xsl:apply-templates select="param [last()]"/>
    <xsl:text>&#x0A;    );&#x0A;</xsl:text>
  </xsl:if>
  <xsl:if test="not(param)">
    <xsl:text>)(</xsl:text>
    <xsl:value-of select="../@name" />
    <xsl:text> *pThis );&#x0A;</xsl:text>
  </xsl:if>
  <xsl:apply-templates select="@if" mode="end"/>
  <xsl:text>&#x0A;</xsl:text>
</xsl:template>

<xsl:template match="interface//method" mode="forwarder">

  <xsl:variable name="parent" select="ancestor::interface"/>

  <xsl:apply-templates select="@if" mode="begin"/>

  <xsl:text>#define COM_FORWARD_</xsl:text>
  <xsl:value-of select="$parent/@name"/>
  <xsl:text>_</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text>_TO(smth) NS_IMETHOD </xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:choose>
    <xsl:when test="param">
      <xsl:text> (</xsl:text>
      <xsl:for-each select="param [position() != last()]">
        <xsl:apply-templates select="." mode="forwarder"/>
        <xsl:text>, </xsl:text>
      </xsl:for-each>
      <xsl:apply-templates select="param [last()]" mode="forwarder"/>
      <xsl:text>) { return smth </xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text> (</xsl:text>
      <xsl:for-each select="param [position() != last()]">
        <xsl:if test="@safearray='yes'">
          <xsl:text>a</xsl:text>
          <xsl:call-template name="capitalize">
            <xsl:with-param name="str" select="@name"/>
          </xsl:call-template>
          <xsl:text>Size+++, </xsl:text>
        </xsl:if>
        <xsl:text>a</xsl:text>
        <xsl:call-template name="capitalize">
          <xsl:with-param name="str" select="@name"/>
        </xsl:call-template>
        <xsl:text>, </xsl:text>
      </xsl:for-each>
      <xsl:if test="param [last()]/@safearray='yes'">
        <xsl:text>a</xsl:text>
        <xsl:call-template name="capitalize">
          <xsl:with-param name="str" select="param [last()]/@name"/>
        </xsl:call-template>
        <xsl:text>Size, </xsl:text>
      </xsl:if>
      <xsl:text>a</xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="param [last()]/@name"/>
      </xsl:call-template>
      <xsl:text>); }</xsl:text>
    </xsl:when>
    <xsl:otherwise test="not(param)">
      <xsl:text>() { return smth </xsl:text>
      <xsl:call-template name="capitalize">
        <xsl:with-param name="str" select="@name"/>
      </xsl:call-template>
      <xsl:text>(); }</xsl:text>
    </xsl:otherwise>
  </xsl:choose>
  <xsl:text>&#x0A;</xsl:text>
  <!-- COM_FORWARD_Interface_Method_TO_OBJ(obj) -->
  <xsl:text>#define COM_FORWARD_</xsl:text>
  <xsl:value-of select="$parent/@name"/>
  <xsl:text>_</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text>_TO_OBJ(obj) COM_FORWARD_</xsl:text>
  <xsl:value-of select="$parent/@name"/>
  <xsl:text>_</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text>_TO ((obj)->)&#x0A;</xsl:text>
  <!-- COM_FORWARD_Interface_Method_TO_BASE(base) -->
  <xsl:text>#define COM_FORWARD_</xsl:text>
  <xsl:value-of select="$parent/@name"/>
  <xsl:text>_</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text>_TO_BASE(base) COM_FORWARD_</xsl:text>
  <xsl:value-of select="$parent/@name"/>
  <xsl:text>_</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text>_TO (base::)&#x0A;</xsl:text>

  <xsl:apply-templates select="@if" mode="end"/>

</xsl:template>


<!--
 *  modules
-->
<xsl:template match="module">
  <xsl:apply-templates select="class"/>
</xsl:template>


<!--
 *  co-classes
-->
<xsl:template match="module/class">
  <!-- class and contract id -->
  <xsl:text>&#x0A;</xsl:text>
  <xsl:text>#define NS_</xsl:text>
  <xsl:call-template name="uppercase">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text>_CID { \&#x0A;</xsl:text>
  <xsl:text>    0x</xsl:text><xsl:value-of select="substring(@uuid,1,8)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,10,4)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,15,4)"/>
  <xsl:text>, \&#x0A;    </xsl:text>
  <xsl:text>{ 0x</xsl:text><xsl:value-of select="substring(@uuid,20,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,22,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,25,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,27,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,29,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,31,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,33,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,35,2)"/>
  <xsl:text> } \&#x0A;}&#x0A;</xsl:text>
  <xsl:text>#define NS_</xsl:text>
  <xsl:call-template name="uppercase">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <!-- Contract ID -->
  <xsl:text>_CONTRACTID &quot;@</xsl:text>
  <xsl:value-of select="@namespace"/>
  <xsl:text>/</xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text>;1&quot;&#x0A;</xsl:text>
  <!-- CLSID_xxx declarations for XPCOM, for compatibility with Win32 -->
  <xsl:text>/* for compatibility with Win32 */&#x0A;</xsl:text>
  <xsl:text>#define CLSID_</xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text> (nsCID) NS_</xsl:text>
  <xsl:call-template name="uppercase">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text>_CID&#x0A;</xsl:text>
  <xsl:text>&#x0A;&#x0A;</xsl:text>
</xsl:template>


<!--
 *  enums
-->
<xsl:template match="enum">
  <xsl:text>/* Start of enum </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text> Declaration */&#x0A;</xsl:text>
  <xsl:text>#define </xsl:text>
  <xsl:call-template name="uppercase">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:value-of select="concat('_IID_STR &quot;',@uuid,'&quot;')"/>
  <xsl:text>&#x0A;</xsl:text>
  <xsl:text>#define </xsl:text>
  <xsl:call-template name="uppercase">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
  <xsl:text>_IID { \&#x0A;</xsl:text>
  <xsl:text>    0x</xsl:text><xsl:value-of select="substring(@uuid,1,8)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,10,4)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,15,4)"/>
  <xsl:text>, \&#x0A;    </xsl:text>
  <xsl:text>{ 0x</xsl:text><xsl:value-of select="substring(@uuid,20,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,22,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,25,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,27,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,29,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,31,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,33,2)"/>
  <xsl:text>, 0x</xsl:text><xsl:value-of select="substring(@uuid,35,2)"/>
  <xsl:text> } \&#x0A;}&#x0A;</xsl:text>
  <xsl:text>enum </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text>&#x0A;{&#x0A;</xsl:text>
  <xsl:variable name="this" select="."/>
  <xsl:for-each select="const">
    <xsl:text>    </xsl:text>
    <xsl:value-of select="$this/@name"/>
    <xsl:text>_</xsl:text>
    <xsl:value-of select="@name"/> = <xsl:value-of select="@value"/>
    <xsl:if test="position() != last()">
      <xsl:text>,</xsl:text>
    </xsl:if>
    <xsl:text>&#x0A;</xsl:text>
  </xsl:for-each>
  <xsl:text>};&#x0A;</xsl:text>
  <xsl:text>/* End of enum </xsl:text>
  <xsl:value-of select="@name"/>
  <xsl:text> Declaration */&#x0A;&#x0A;&#x0A;</xsl:text>
</xsl:template>


<!--
 *  method parameters
-->
<xsl:template match="method/param">
  <xsl:choose>
    <!-- safearray parameters -->
    <xsl:when test="@safearray='yes'">
      <!-- array size -->
      <xsl:choose>
        <xsl:when test="@dir='in'">
          <xsl:text>PRUint32 </xsl:text>
          <xsl:value-of select="@name"/>
          <xsl:text>Size,&#x0A;</xsl:text>
        </xsl:when>
        <xsl:when test="@dir='out'">
          <xsl:text>PRUint32 *</xsl:text>
          <xsl:value-of select="@name"/>
          <xsl:text>Size,&#x0A;</xsl:text>
        </xsl:when>
        <xsl:when test="@dir='return'">
          <xsl:text>PRUint32 *</xsl:text>
          <xsl:value-of select="@name"/>
          <xsl:text>Size,&#x0A;</xsl:text>
        </xsl:when>
        <xsl:otherwise>
          <xsl:text>PRUint32 </xsl:text>
          <xsl:value-of select="@name"/>
          <xsl:text>Size,&#x0A;</xsl:text>
        </xsl:otherwise>
      </xsl:choose>
      <!-- array pointer -->
      <xsl:text>        </xsl:text>
      <xsl:choose>
        <xsl:when test="@dir='in'">
          <xsl:apply-templates select="@type" mode="forwarder"/>
          <xsl:text>*</xsl:text>
        </xsl:when>
        <xsl:when test="@dir='out'">
          <xsl:apply-templates select="@type" mode="forwarder"/>
          <xsl:if test="@type='wstring'">
            <xsl:text>*</xsl:text>
          </xsl:if>
          <xsl:text>*</xsl:text>
        </xsl:when>
        <xsl:when test="@dir='return'">
          <xsl:apply-templates select="@type" mode="forwarder"/>
          <xsl:text>**</xsl:text>
        </xsl:when>
        <xsl:otherwise>
          <xsl:apply-templates select="@type" mode="forwarder"/>
          <xsl:text>*</xsl:text>
        </xsl:otherwise>
      </xsl:choose>
      <xsl:text> </xsl:text>
      <xsl:value-of select="@name"/>
    </xsl:when>
    <!-- normal and array parameters -->
    <xsl:otherwise>
      <xsl:choose>
        <xsl:when test="@dir='in'">
          <xsl:apply-templates select="@type" mode="forwarder"/>
          <xsl:text></xsl:text>
        </xsl:when>
        <xsl:when test="@dir='out'">
          <xsl:apply-templates select="@type" mode="forwarder"/>
          <xsl:text> *</xsl:text>
        </xsl:when>
        <xsl:when test="@dir='return'">
          <xsl:apply-templates select="@type" mode="forwarder"/>
          <xsl:text> *</xsl:text>
        </xsl:when>
        <xsl:otherwise>
          <xsl:apply-templates select="@type" mode="forwarder"/>
          <xsl:text></xsl:text>
        </xsl:otherwise>
      </xsl:choose>
      <xsl:text> </xsl:text>
      <xsl:value-of select="@name"/>
    </xsl:otherwise>
  </xsl:choose>
</xsl:template>

<xsl:template match="method/param" mode="forwarder">
  <xsl:if test="@safearray='yes'">
    <xsl:text>PRUint32</xsl:text>
    <xsl:if test="@dir='out' or @dir='return'">
      <xsl:text> *</xsl:text>
    </xsl:if>
    <xsl:text> a</xsl:text>
    <xsl:call-template name="capitalize">
      <xsl:with-param name="str" select="@name"/>
    </xsl:call-template>
    <xsl:text>Size, </xsl:text>
  </xsl:if>
  <xsl:apply-templates select="@type" mode="forwarder"/>
  <xsl:if test="@dir='out' or @dir='return'">
    <xsl:text> *</xsl:text>
  </xsl:if>
  <xsl:if test="@safearray='yes'">
    <xsl:text> *</xsl:text>
  </xsl:if>
  <xsl:text> a</xsl:text>
  <xsl:call-template name="capitalize">
    <xsl:with-param name="str" select="@name"/>
  </xsl:call-template>
</xsl:template>


<!--
 *  attribute/parameter type conversion
-->
<xsl:template match="attribute/@type | param/@type">
  <xsl:variable name="self_target" select="current()/ancestor::if/@target"/>

  <xsl:choose>
    <!-- modifiers (ignored for 'enumeration' attributes)-->
    <xsl:when test="name(current())='type' and ../@mod">
      <xsl:choose>
        <xsl:when test="../@mod='ptr'">
          <xsl:choose>
            <!-- standard types -->
            <!--xsl:when test=".='result'">??</xsl:when-->
            <xsl:when test=".='boolean'">booleanPtr</xsl:when>
            <xsl:when test=".='octet'">octetPtr</xsl:when>
            <xsl:when test=".='short'">shortPtr</xsl:when>
            <xsl:when test=".='unsigned short'">ushortPtr</xsl:when>
            <xsl:when test=".='long'">longPtr</xsl:when>
            <xsl:when test=".='long long'">llongPtr</xsl:when>
            <xsl:when test=".='unsigned long'">ulongPtr</xsl:when>
            <xsl:when test=".='unsigned long long'">ullongPtr</xsl:when>
            <xsl:otherwise>
              <xsl:message terminate="yes">
                <xsl:value-of select="concat(../../../@name,'::',../../@name,'::',../@name,': ')"/>
                <xsl:text>attribute 'mod=</xsl:text>
                <xsl:value-of select="concat('&quot;',../@mod,'&quot;')"/>
                <xsl:text>' cannot be used with type </xsl:text>
                <xsl:value-of select="concat('&quot;',current(),'&quot;!')"/>
              </xsl:message>
            </xsl:otherwise>
          </xsl:choose>
        </xsl:when>
        <xsl:when test="../@mod='string'">
          <xsl:choose>
            <!-- standard types -->
            <!--xsl:when test=".='result'">??</xsl:when-->
            <xsl:when test=".='uuid'">wstring</xsl:when>
            <xsl:otherwise>
              <xsl:message terminate="yes">
                <xsl:value-of select="concat(../../../@name,'::',../../@name,'::',../@name,': ')"/>
                <xsl:text>attribute 'mod=</xsl:text>
                <xsl:value-of select="concat('&quot;',../@mod,'&quot;')"/>
                <xsl:text>' cannot be used with type </xsl:text>
                <xsl:value-of select="concat('&quot;',current(),'&quot;!')"/>
              </xsl:message>
            </xsl:otherwise>
          </xsl:choose>
        </xsl:when>
        <xsl:otherwise>
          <xsl:message terminate="yes">
            <xsl:value-of select="concat(../../../@name,'::',../../@name,'::',../@name,': ')"/>
            <xsl:value-of select="concat('value &quot;',../@mod,'&quot; ')"/>
            <xsl:text>of attribute 'mod' is invalid!</xsl:text>
          </xsl:message>
        </xsl:otherwise>
      </xsl:choose>
    </xsl:when>
    <!-- no modifiers -->
    <xsl:otherwise>
      <xsl:choose>
        <!-- standard types -->
        <xsl:when test=".='result'">nsresult</xsl:when>
        <xsl:when test=".='boolean'">boolean</xsl:when>
        <xsl:when test=".='octet'">octet</xsl:when>
        <xsl:when test=".='short'">short</xsl:when>
        <xsl:when test=".='unsigned short'">unsigned short</xsl:when>
        <xsl:when test=".='long'">long</xsl:when>
        <xsl:when test=".='long long'">long long</xsl:when>
        <xsl:when test=".='unsigned long'">unsigned long</xsl:when>
        <xsl:when test=".='unsigned long long'">unsigned long long</xsl:when>
        <xsl:when test=".='char'">char</xsl:when>
        <xsl:when test=".='wchar'">wchar</xsl:when>
        <xsl:when test=".='string'">string</xsl:when>
        <xsl:when test=".='wstring'">wstring</xsl:when>
        <!-- UUID type -->
        <xsl:when test=".='uuid'">
          <xsl:choose>
            <xsl:when test="name(..)='attribute'">
              <xsl:choose>
                <xsl:when test="../@readonly='yes'">
                  <xsl:text>nsIDPtr</xsl:text>
                </xsl:when>
                <xsl:otherwise>
                  <xsl:message terminate="yes">
                    <xsl:value-of select="../@name"/>
                    <xsl:text>: Non-readonly uuid attributes are not supported!</xsl:text>
                  </xsl:message>
                </xsl:otherwise>
              </xsl:choose>
            </xsl:when>
            <xsl:when test="name(..)='param'">
              <xsl:choose>
                <xsl:when test="../@dir='in' and not(../@safearray='yes')">
                  <xsl:text>nsIDRef</xsl:text>
                </xsl:when>
                <xsl:otherwise>
                  <xsl:text>nsIDPtr</xsl:text>
                </xsl:otherwise>
              </xsl:choose>
            </xsl:when>
          </xsl:choose>
        </xsl:when>
        <!-- system interface types -->
        <xsl:when test=".='$unknown'">nsISupports</xsl:when>
        <xsl:otherwise>
          <xsl:choose>
            <!-- enum types -->
            <xsl:when test="
              (ancestor::library/enum[@name=current()]) or
              (ancestor::library/if[@target=$self_target]/enum[@name=current()])
            ">
              <xsl:text>PRUint32</xsl:text>
            </xsl:when>
            <!-- custom interface types -->
            <xsl:when test="
              (name(current())='enumerator' and
               ((ancestor::library/enumerator[@name=current()]) or
                (ancestor::library/if[@target=$self_target]/enumerator[@name=current()]))
              ) or
              ((ancestor::library/interface[@name=current()]) or
               (ancestor::library/if[@target=$self_target]/interface[@name=current()])
              )
            ">
              <xsl:value-of select="."/>
            </xsl:when>
            <!-- other types -->
            <xsl:otherwise>
              <xsl:message terminate="yes">
                <xsl:text>Unknown parameter type: </xsl:text>
                <xsl:value-of select="."/>
              </xsl:message>
            </xsl:otherwise>
          </xsl:choose>
        </xsl:otherwise>
      </xsl:choose>
    </xsl:otherwise>
  </xsl:choose>
</xsl:template>

<xsl:template match="attribute/@type | param/@type" mode="forwarder">

  <xsl:variable name="self_target" select="current()/ancestor::if/@target"/>

  <xsl:choose>
    <!-- modifiers (ignored for 'enumeration' attributes)-->
    <xsl:when test="name(current())='type' and ../@mod">
      <xsl:choose>
        <xsl:when test="../@mod='ptr'">
          <xsl:choose>
            <!-- standard types -->
            <!--xsl:when test=".='result'">??</xsl:when-->
            <xsl:when test=".='boolean'">PRBool *</xsl:when>
            <xsl:when test=".='octet'">PRUint8 *</xsl:when>
            <xsl:when test=".='short'">PRInt16 *</xsl:when>
            <xsl:when test=".='unsigned short'">PRUint16 *</xsl:when>
            <xsl:when test=".='long'">PRInt32 *</xsl:when>
            <xsl:when test=".='long long'">PRInt64 *</xsl:when>
            <xsl:when test=".='unsigned long'">PRUint32 *</xsl:when>
            <xsl:when test=".='unsigned long long'">PRUint64 *</xsl:when>
            <xsl:otherwise>
              <xsl:message terminate="yes">
                <xsl:value-of select="concat(../../../@name,'::',../../@name,'::',../@name,': ')"/>
                <xsl:text>attribute 'mod=</xsl:text>
                <xsl:value-of select="concat('&quot;',../@mod,'&quot;')"/>
                <xsl:text>' cannot be used with type </xsl:text>
                <xsl:value-of select="concat('&quot;',current(),'&quot;!')"/>
              </xsl:message>
            </xsl:otherwise>
          </xsl:choose>
        </xsl:when>
        <xsl:when test="../@mod='string'">
          <xsl:choose>
            <!-- standard types -->
            <!--xsl:when test=".='result'">??</xsl:when-->
            <xsl:when test=".='uuid'">PRUnichar *</xsl:when>
            <xsl:otherwise>
              <xsl:message terminate="yes">
                <xsl:value-of select="concat(../../../@name,'::',../../@name,'::',../@name,': ')"/>
                <xsl:text>attribute 'mod=</xsl:text>
                <xsl:value-of select="concat('&quot;',../@mod,'&quot;')"/>
                <xsl:text>' cannot be used with type </xsl:text>
                <xsl:value-of select="concat('&quot;',current(),'&quot;!')"/>
              </xsl:message>
            </xsl:otherwise>
          </xsl:choose>
        </xsl:when>
      </xsl:choose>
    </xsl:when>
    <!-- no modifiers -->
    <xsl:otherwise>
      <xsl:choose>
        <!-- standard types -->
        <xsl:when test=".='result'">nsresult</xsl:when>
        <xsl:when test=".='boolean'">PRBool</xsl:when>
        <xsl:when test=".='octet'">PRUint8</xsl:when>
        <xsl:when test=".='short'">PRInt16</xsl:when>
        <xsl:when test=".='unsigned short'">PRUint16</xsl:when>
        <xsl:when test=".='long'">PRInt32</xsl:when>
        <xsl:when test=".='long long'">PRInt64</xsl:when>
        <xsl:when test=".='unsigned long'">PRUint32</xsl:when>
        <xsl:when test=".='unsigned long long'">PRUint64</xsl:when>
        <xsl:when test=".='char'">char</xsl:when>
        <xsl:when test=".='wchar'">PRUnichar</xsl:when>
        <!-- string types -->
        <xsl:when test=".='string'">char *</xsl:when>
        <xsl:when test=".='wstring'">PRUnichar *</xsl:when>
        <!-- UUID type -->
        <xsl:when test=".='uuid'">
          <xsl:choose>
            <xsl:when test="name(..)='attribute'">
              <xsl:choose>
                <xsl:when test="../@readonly='yes'">
                  <xsl:text>nsID *</xsl:text>
                </xsl:when>
              </xsl:choose>
            </xsl:when>
            <xsl:when test="name(..)='param'">
              <xsl:choose>
                <xsl:when test="../@dir='in' and not(../@safearray='yes')">
                  <xsl:text>const nsID *</xsl:text>
                </xsl:when>
                <xsl:otherwise>
                  <xsl:text>nsID *</xsl:text>
                </xsl:otherwise>
              </xsl:choose>
            </xsl:when>
          </xsl:choose>
        </xsl:when>
        <!-- system interface types -->
        <xsl:when test=".='$unknown'">nsISupports *</xsl:when>
        <xsl:otherwise>
          <xsl:choose>
            <!-- enum types -->
            <xsl:when test="
              (ancestor::library/enum[@name=current()]) or
              (ancestor::library/if[@target=$self_target]/enum[@name=current()])
            ">
              <xsl:text>PRUint32</xsl:text>
            </xsl:when>
            <!-- custom interface types -->
            <xsl:when test="
              (name(current())='enumerator' and
               ((ancestor::library/enumerator[@name=current()]) or
                (ancestor::library/if[@target=$self_target]/enumerator[@name=current()]))
              ) or
              ((ancestor::library/interface[@name=current()]) or
               (ancestor::library/if[@target=$self_target]/interface[@name=current()])
              )
            ">
              <xsl:value-of select="."/>
              <xsl:text> *</xsl:text>
            </xsl:when>
            <!-- other types -->
          </xsl:choose>
        </xsl:otherwise>
      </xsl:choose>
    </xsl:otherwise>
  </xsl:choose>
</xsl:template>

</xsl:stylesheet>

