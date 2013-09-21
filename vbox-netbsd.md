VirtualBox-NetBSD-Additions
===========================

This port only support VirtualBox additions for NetBSD.


Prerequisite
============

1. kBuild

Goto [kBuild](http://svn.netlabs.org/repos/kbuild/trunk) download source package.
Before bootstrap kBuild, you need gettext-tools(/usr/pkgsrc/devel/gettext-tools).

./kBuild/env.sh --full

gmake -f bootstrap.kmk


2. GCC 3.2.3 or later (except for the GCC 4.0.x series) 
3. Yasm 0.6.2 or later 
4. bcc (Bruce Evans C Compiler; often part of the dev86 package) 
5. xsltproc (libxslt, XML style sheet processor)

more info can get from running configure result

Install
======

./kmk

./kmk install

You can find binary files at out/netbsd.amd64/release/bin/additions/

modload vboxvideo.ko

modload vboxguest.ko

./VboxClient

Main Jobs
======

1. Add NetBSD support to Config.kmk
2. Make Iprt library compatible with NetBSD
3. Add r0drv library of NetBSD
4. Add r3 Runtime library of NetBSD
5. Add vboxguest driver support for NetBSD
6. Add vbox-additions library for NetBSD

Main Problem
===========

1. Loading vboxguest module kernel crash
2. Incomplete vbox additions feature supports
