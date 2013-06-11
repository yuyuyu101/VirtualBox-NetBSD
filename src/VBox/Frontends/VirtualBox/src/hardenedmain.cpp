/* $Id: hardenedmain.cpp $ */
/** @file
 * VirtualBox - Hardened main().
 */

/*
 * Copyright (C) 2008-2010 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#include <string.h>
#include <VBox/sup.h>


int main(int argc, char **argv, char **envp)
{
    /*
     * First check whether we're about to start a VM.
     */
    uint32_t fFlags = SUPSECMAIN_FLAGS_DONT_OPEN_DEV;
    for (int i = 1; i < argc; i++)
        /* NOTE: the check here must match the corresponding check for the
         * options to start a VM in main.cpp and VBoxGlobal.cpp exactly,
         * otherwise there will be weird error messages. */
        if (   !::strcmp(argv[i], "--startvm")
            || !::strcmp(argv[i], "-startvm"))
        {
            fFlags &= ~SUPSECMAIN_FLAGS_DONT_OPEN_DEV;
            break;
        }

    return SUPR3HardenedMain("VirtualBox", fFlags, argc, argv, envp);
}

