#!/bin/sh
#
# Oracle VM VirtualBox startup script, Linux hosts.
#
# Copyright (C) 2006-2011 Oracle Corporation
#
# This file is part of VirtualBox Open Source Edition (OSE), as
# available from http://www.virtualbox.org. This file is free software;
# you can redistribute it and/or modify it under the terms of the GNU
# General Public License (GPL) as published by the Free Software
# Foundation, in version 2 as it comes in the "COPYING" file of the
# VirtualBox OSE distribution. VirtualBox OSE is distributed in the
# hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
#

PATH="/usr/bin:/bin:/usr/sbin:/sbin"

# Note: This script must not fail if the module was not successfully installed
#       because the user might not want to run a VM but only change VM params!

if [ "$1" = "shutdown" ]; then
    SHUTDOWN="true"
elif ! lsmod|grep -q vboxdrv; then
    cat << EOF
WARNING: The vboxdrv kernel module is not loaded. Either there is no module
         available for the current kernel (`uname -r`) or it failed to
         load. Please recompile the kernel module and install it by

           sudo /etc/init.d/vboxdrv setup

         You will not be able to start VMs until this problem is fixed.
EOF
elif [ ! -c /dev/vboxdrv ]; then
    cat << EOF
WARNING: The character device /dev/vboxdrv does not exist. Try

           sudo /etc/init.d/vboxdrv restart

         and if that is not successful, try to re-install the package.

         You will not be able to start VMs until this problem is fixed.
EOF
fi

if [ -f /etc/vbox/module_not_compiled ]; then
    cat << EOF
WARNING: The compilation of the vboxdrv.ko kernel module failed during the
         installation for some reason. Starting a VM will not be possible.
         Please consult the User Manual for build instructions.
EOF
fi

SERVER_PID=`ps -U \`whoami\` | grep VBoxSVC | awk '{ print $1 }'`
if [ -z "$SERVER_PID" ]; then
    # Server not running yet/anymore, cleanup socket path.
    # See IPC_GetDefaultSocketPath()!
    if [ -n "$LOGNAME" ]; then
        rm -rf /tmp/.vbox-$LOGNAME-ipc > /dev/null 2>&1
    else
        rm -rf /tmp/.vbox-$USER-ipc > /dev/null 2>&1
    fi
fi

if [ "$SHUTDOWN" = "true" ]; then
    if [ -n "$SERVER_PID" ]; then
        kill -TERM $SERVER_PID
        sleep 2
    fi
    exit 0
fi

INSTALL_DIR=/usr/lib/virtualbox

APP=`basename $0`
case "$APP" in
    VirtualBox|virtualbox)
        exec "$INSTALL_DIR/VirtualBox" "$@"
        ;;
    VBoxManage|vboxmanage)
        exec "$INSTALL_DIR/VBoxManage" "$@"
        ;;
    VBoxSDL|vboxsdl)
        exec "$INSTALL_DIR/VBoxSDL" "$@"
        ;;
    VBoxVRDP|VBoxHeadless|vboxheadless)
        exec "$INSTALL_DIR/VBoxHeadless" "$@"
        ;;
    VBoxBalloonCtrl|vboxballoonctrl)
        exec "$INSTALL_DIR/VBoxBalloonCtrl" "$@"
        ;;
    vboxwebsrv)
        exec "$INSTALL_DIR/vboxwebsrv" "$@"
        ;;
    VBoxBFE|vboxbfe)
        exec "$INSTALL_DIR/VBoxBFE" "$@"
        ;;
    *)
        echo "Unknown application - $APP"
        exit 1
        ;;
esac
exit 0
