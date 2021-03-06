# $Id: Makefile $
## @file
# VirtualBox Guest Additions Module Makefile.
#

#
# Copyright (C) 2006-2012 Oracle Corporation
#
# This file is part of VirtualBox Open Source Edition (OSE), as
# available from http://www.virtualbox.org. This file is free software;
# you can redistribute it and/or modify it under the terms of the GNU
# General Public License (GPL) as published by the Free Software
# Foundation, in version 2 as it comes in the "COPYING" file of the
# VirtualBox OSE distribution. VirtualBox OSE is distributed in the
# hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
#
KMOD = vboxguest

CFLAGS += -DRT_OS_NETBSD -DIN_RING0 -DIN_RT_R0 -DIN_SUP_R0 -DVBOX -DRT_WITH_VBOX -Iinclude -I. -Ir0drv -w -DVBGL_VBOXGUEST -DVBOX_WITH_HGCM -DVBOX_WITH_64_BITS_GUESTS

.if (${MACHINE_ARCH} == "i386")
 CFLAGS += -DRT_ARCH_X86
.elif (${MACHINE_ARCH} == "amd64")
 CFLAGS += -DRT_ARCH_AMD64
.endif

SRCS = \
	VBoxGuest.c \
	VBoxGuest2.c \
	VBoxGuest-netbsd.c \
	GenericRequest.c \
	HGCMInternal.c \
	Init.c \
	PhysHeap.c \
	SysHlp.c \
	VMMDev.c

# Include needed interface headers so they are created during build
SRCS += \
	device_if.h \
	bus_if.h \
	pci_if.h \

.PATH:	${.CURDIR}/alloc
SRCS += \
	heapsimple.c

.PATH:	${.CURDIR}/common/err
SRCS += \
	RTErrConvertFromErrno.c \
	RTErrConvertToErrno.c

.PATH:	${.CURDIR}/common/log
SRCS += \
	log.c \
	logellipsis.c \
	logrel.c \
	logrelellipsis.c \
	logcom.c \
	logformat.c

.PATH:	${.CURDIR}/common/misc
SRCS += \
	RTAssertMsg1Weak.c \
	RTAssertMsg2.c \
	RTAssertMsg2Add.c \
	RTAssertMsg2AddWeak.c \
	RTAssertMsg2AddWeakV.c \
	RTAssertMsg2Weak.c \
	RTAssertMsg2WeakV.c \
	assert.c \
	handletable.c \
	handletablectx.c \
	once.c \
	thread.c

.PATH:	${.CURDIR}/common/string
SRCS += \
	RTStrCopy.c \
	RTStrCopyP.c \
	strformat.c \
	strformatrt.c \
	strformattype.c \
	strprintf.c \
	strtonum.c \
	memchr.c

.PATH:	${.CURDIR}/common/rand
SRCS += \
	rand.c \
	randadv.c \
	randparkmiller.c

.PATH:	${.CURDIR}/common/path
SRCS += \
	RTPathStripFilename.c

.PATH:	${.CURDIR}/common/checksum
SRCS += \
	crc32.c \
	ipv4.c

.PATH:	${.CURDIR}/common/table
SRCS += \
	avlpv.c

.PATH:	${.CURDIR}/common/time
SRCS += \
	time.c

.PATH:	${.CURDIR}/generic
SRCS += \
	uuid-generic.c \
	RTAssertShouldPanic-generic.c \
	RTLogWriteDebugger-generic.c \
	RTLogWriteStdOut-stub-generic.c \
	RTLogWriteStdErr-stub-generic.c \
	RTRandAdvCreateSystemFaster-generic.c \
	RTRandAdvCreateSystemTruer-generic.c \
	RTSemEventWait-2-ex-generic.c \
	RTSemEventWaitNoResume-2-ex-generic.c \
	RTSemEventMultiWait-2-ex-generic.c \
	RTSemEventMultiWaitNoResume-2-ex-generic.c \
	RTTimerCreate-generic.c \
	timer-generic.c \
	errvars-generic.c \
	mppresent-generic.c

.PATH:	${.CURDIR}/r0drv
SRCS += \
	alloc-r0drv.c \
	initterm-r0drv.c \
	memobj-r0drv.c \
	powernotification-r0drv.c

.PATH:	${.CURDIR}/r0drv/netbsd
SRCS += \
	assert-r0drv-netbsd.c \
	alloc-r0drv-netbsd.c \
	initterm-r0drv-netbsd.c \
	memobj-r0drv-netbsd.c \
	memuserkernel-r0drv-netbsd.c \
	mp-r0drv-netbsd.c \
	process-r0drv-netbsd.c \
	semevent-r0drv-netbsd.c \
	semeventmulti-r0drv-netbsd.c \
	semfastmutex-r0drv-netbsd.c \
	semmutex-r0drv-netbsd.c \
	spinlock-r0drv-netbsd.c \
	thread-r0drv-netbsd.c \
	thread2-r0drv-netbsd.c \
	time-r0drv-netbsd.c

.PATH:	${.CURDIR}/r0drv/generic
SRCS += \
	semspinmutex-r0drv-generic.c \
	mpnotification-r0drv-generic.c \
	RTMpIsCpuWorkPending-r0drv-generic.c

.PATH:	${.CURDIR}/VBox
SRCS += \
	log-vbox.c \
	logbackdoor.c

.include <bsd.kmod.mk>

