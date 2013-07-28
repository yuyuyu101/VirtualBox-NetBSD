#
# Makefile for the VirtualBox NetBSD Guest Drivers.
#

#
#
# Copyright (C) 2009-2010 Oracle Corporation
#
# This file is part of VirtualBox Open Source Edition (OSE), as
# available from http://www.virtualbox.org. This file is free software;
# you can redistribute it and/or modify it under the terms of the GNU
# General Public License (GPL) as published by the Free Software
# Foundation, in version 2 as it comes in the "COPYING" file of the
# VirtualBox OSE distribution. VirtualBox OSE is distributed in the
# hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
#

KBUILD_VERBOSE =

all:
	@echo "*** Building 'vboxguest' module ***"
	@$(MAKE) KBUILD_VERBOSE=$(KBUILD_VERBOSE) -C vboxguest
	@if [ -f vboxguest/vboxguest.kmod ]; then \
	    cp vboxguest/vboxguest.kmod .; \
	 fi
	@echo
	@if [ -d vboxvfs ]; then \
	    echo "*** Building 'vboxvfs' module ***"; \
	    $(MAKE) KBUILD_VERBOSE=$(KBUILD_VERBOSE) -C vboxvfs; \
	    if [ -f vboxvfs/vboxvfs.kmod ]; then \
	        cp vboxvfs/vboxvfs.kmod .; \
	    fi; \
	fi


install:
	@$(MAKE) KBUILD_VERBOSE=$(KBUILD_VERBOSE) -C vboxguest install
	@if [ -d vboxvfs ]; then \
	    $(MAKE) KBUILD_VERBOSE=$(KBUILD_VERBOSE) -C vboxvfs install; \
	fi

clean:
	@$(MAKE) -C vboxguest clean
	@if [ -d vboxvfs ]; then \
	    $(MAKE) -C vboxvfs clean; \
	fi
	rm -f vboxguest.*mod vboxvfs.*mod

load:
	@/sbin/modunload vboxvfs || true
	@/sbin/modunload vboxguest || true
	@/sbin/modload ./vboxguest.kmod
	@if [ -f vboxvfs.kmod ]; then /sbin/modload ./vboxvfs.kmod; fi
