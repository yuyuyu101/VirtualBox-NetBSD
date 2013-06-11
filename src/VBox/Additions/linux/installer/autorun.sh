#!/bin/sh
# $Id: autorun.sh $
#
# VirtualBox Guest Additions installation script for *nix guests
#

#
# Copyright (C) 2009-2011 Oracle Corporation
#
# This file is part of VirtualBox Open Source Edition (OSE), as
# available from http://www.virtualbox.org. This file is free software;
# you can redistribute it and/or modify it under the terms of the GNU
# General Public License (GPL) as published by the Free Software
# Foundation, in version 2 as it comes in the "COPYING" file of the
# VirtualBox OSE distribution. VirtualBox OSE is distributed in the
# hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
#

PATH=$PATH:/bin:/sbin:/usr/sbin

#include sh-utils.sh

ostype=`uname -s`
if test "$ostype" != "Linux" && test "$ostype" != "SunOS" ; then
  echo "Linux/Solaris not detected."
  exit 1
fi

# 32-bit or 64-bit?
path=`dirname $0`
case `uname -m` in
  i[3456789]86|x86|i86pc)
    arch='x86'
    ;;
  x86_64|amd64|AMD64)
    arch='amd64'
    ;;
  *)
    echo "Unknown architecture `uname -m`."
    exit 1
    ;;
esac

# execute the installer
if test "$ostype" = "Linux"; then
    for i in "$path/VBoxLinuxAdditions.run" \
        "$path/VBoxLinuxAdditions-$arch.run"; do
        if test -f "$i"; then
            getxterm
            case "$gxtpath" in ?*)
                TITLE="VirtualBox Guest Additions installation"
                BINARY="`quotify "$i"`"
                exec "$gxtpath" "$gxttitle" "$TITLE" "$gxtexec" /bin/sh "$path/runasroot.sh" --has-terminal "$TITLE" "/bin/sh $BINARY --xwin" "Please try running "\""$i"\"" manually."
                exit
                ;;
            esac
        fi
    done

    # else: unknown failure
    echo "Linux guest additions installer not found -- try to start them manually."
    exit 1

elif test "$ostype" = "SunOS"; then

    # check for combined package
    installfile="$path/VBoxSolarisAdditions.pkg"
    if test -f "$installfile"; then

        # check for pkgadd bin
        pkgaddbin=pkgadd
        found=`which pkgadd | grep "no pkgadd"`
        if test ! -z "$found"; then
            if test -f "/usr/sbin/pkgadd"; then
                pkgaddbin=/usr/sbin/pkgadd
            else
                echo "Could not find pkgadd."
                exit 1
            fi
        fi

        # check for pfexec
        pfexecbin=pfexec
        found=`which pfexec | grep "no pfexec"`
        if test ! -z "$found"; then
            # Use su and prompt for password
            echo "Could not find pfexec."
            subin=`which su`
        else
            idbin=/usr/xpg4/bin/id
            if test ! -x "$idbin"; then
                found=`which id 2> /dev/null`
                if test ! -x "$found"; then
                    echo "Failed to find a suitable user id executable."
                    exit 1
                else
                    idbin=$found
                fi
            fi

            # check if pfexec can get the job done
            if test "$idbin" = "/usr/xpg4/bin/id"; then
                userid=`$pfexecbin $idbin -u`
            else
                userid=`$pfexecbin $idbin | cut -f1 -d'(' | cut -f2 -d'='`
            fi
            if test $userid != "0"; then
                # pfexec exists but user has no pfexec privileges, switch to using su and prompting password
                subin=`which su`
            fi
        fi

        # create temporary admin file for autoinstall
        echo "basedir=default
runlevel=nocheck
conflict=quit
setuid=nocheck
action=nocheck
partial=quit
instance=unique
idepend=quit
rdepend=quit
space=quit
mail=
" > /tmp/vbox.autoinstall

        # check gnome-terminal, use it if it exists.
        if test -f "/usr/bin/gnome-terminal"; then
            # use su/pfexec
            if test -z "$subin"; then
                /usr/bin/gnome-terminal --title "Installing VirtualBox Additions" --command "/bin/sh -c '$pfexecbin $pkgaddbin -G -d $installfile -n -a /tmp/vbox.autoinstall SUNWvboxguest; /bin/echo press ENTER to close this window; /bin/read; /bin/rm -f /tmp/vbox.autoinstall'"
            else
                /usr/bin/gnome-terminal --title "Installing VirtualBox Additions: Root password required." --command "/bin/sh -c '$subin - root -c \"$pkgaddbin -G -d $installfile -n -a /tmp/vbox.autoinstall SUNWvboxguest\"; /bin/echo press ENTER to close this window; /bin/read; /bin/rm -f /tmp/vbox.autoinstall'"
            fi
        elif test -f "/usr/X11/bin/xterm"; then
            # use xterm
            if test -z "$subin"; then
                /usr/X11/bin/xterm -title "Installing VirtualBox Additions" -e "$pfexecbin $pkgaddbin -G -d $installfile -n -a /tmp/vbox.autoinstall SUNWvboxguest; /bin/echo press ENTER to close this window; /bin/read; /bin/rm -f /tmp/vbox.autoinstall"
            else
                /usr/X11/bin/xterm -title "Installing VirtualBox Additions: Root password required." -e "$subin - root -c \"$pkgaddbin -G -d $installfile -n -a /tmp/vbox.autoinstall SUNWvboxguest\"; /bin/echo press ENTER to close this window; /bin/read; /bin/rm -f /tmp/vbox.autoinstall"
            fi
        else
            echo "No suitable terminal not found. -- install additions using pkgadd -d."
            rm -f /tmp/vbox.autoinstall
        fi

        exit 0
    fi

    echo "Solaris guest additions installer not found -- try to start them manually."
    exit 1
fi

