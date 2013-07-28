/* $Id: vboxvideo_drm.c $ */
/** @file
 * VirtualBox Guest Additions - vboxvideo DRM module.
 * NetBSD kernel OpenGL module.
 */

/*
 * Copyright (C) 2006-2010 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 * --------------------------------------------------------------------
 *
 * This code is based on:
 *
 * tdfx_drv.c -- tdfx driver -*- linux-c -*-
 * Created: Thu Oct  7 10:38:32 1999 by faith@precisioninsight.com
 *
 * Copyright 1999 Precision Insight, Inc., Cedar Park, Texas.
 * Copyright 2000 VA Linux Systems, Inc., Sunnyvale, California.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * PRECISION INSIGHT AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 *    Rickard E. (Rik) Faith <faith@valinux.com>
 *    Daryll Strauss <daryll@valinux.com>
 *    Gareth Hughes <gareth@valinux.com>
 *
 */

#include <sys/cdefs.h>
#include <sys/module.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/poll.h>
#include <sys/selinfo.h>
#include <sys/queue.h>
#include <sys/lock.h>
#include <sys/types.h>
#include <sys/conf.h>
#include <sys/malloc.h>
#include <sys/uio.h>
#include <sys/file.h>

#include "dev/drm/drmP.h"
#include "dev/drm/drm_pciids.h"

#define DRIVER_AUTHOR                   "Oracle Corporation"
#define DRIVER_NAME                     "vboxvideo"
#define DRIVER_DESC                     "VirtualBox DRM"
#define DRIVER_DATE                     "20090317"
#define DRIVER_MAJOR                    1
#define DRIVER_MINOR                    0
#define DRIVER_PATCHLEVEL               0

/** @todo Take PCI IDs from VBox/param.h; VBOX_VESA_VENDORID,
 *        VBOX_VESA_DEVICEID. */
#define vboxvideo_PCI_IDS           { 0x80ee, 0xbeef, 0, "VirtualBox Video" }, \
                                    { 0, 0, 0, NULL }

static drm_pci_id_list_t vboxvideo_pciidlist[] = {
	vboxvideo_PCI_IDS
};

struct cfdriver vboxvideo_cd = {
        NULL, "vboxvideo", DV_DULL
};

static void vboxvideo_configure(struct drm_device *dev)
{
	dev->driver->buf_priv_size	= 1; /* No dev_priv */

	dev->driver->max_ioctl		= 0;

	dev->driver->name		= DRIVER_NAME;
	dev->driver->desc		= DRIVER_DESC;
	dev->driver->date		= DRIVER_DATE;
	dev->driver->major		= DRIVER_MAJOR;
	dev->driver->minor		= DRIVER_MINOR;
	dev->driver->patchlevel		= DRIVER_PATCHLEVEL;
}

static int
vboxvideo_probe(device_t kdev)
{
	return drm_probe(kdev, vboxvideo_pciidlist);
}

static int
vboxvideo_attach(device_t kdev)
{
	struct drm_device *dev;
    dev = device_lookup_private(&vboxguest_cd, minor(device));

	dev->driver = malloc(sizeof(struct drm_driver_info), DRM_MEM_DRIVER,
	    M_WAITOK | M_ZERO);

	vboxvideo_configure(dev);

	return drm_attach(kdev, vboxvideo_pciidlist);
}

static int
vboxvideo_detach(device_t kdev)
{
	struct drm_device *dev;
    dev = device_lookup_private(&vboxguest_cd, minor(device));
	int ret;

	ret = drm_detach(kdev);

	free(dev->driver, DRM_MEM_DRIVER);
	return ret;
}

CFATTACH_DECL3_NEW(vboxvideo, sizeof(struct drm_device),
    NULL, vboxvideo_attach, vboxvideo_detach, vboxvideo_probe, NULL, NULL,
    NULL);

MODULE(MODULE_CLASS_DRIVER, vboxvideo, NULL);

static int vboxguest_modcmd(modcmd_t cmd, void *opaque)
{
    switch (cmd) {
        case MODULE_CMD_INIT:
                return 0;
        case MODULE_CMD_FINI:
                return 0;
        default:
                return ENOTTY;
    }
}
