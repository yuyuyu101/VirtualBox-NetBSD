; $Id: VBoxBiosAlternative.asm $ 
;; @file
; Auto Generated source file. Do not edit.
;

;
; Source file: bios.c
;
;  
;  Copyright (C) 2006-2012 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
;  --------------------------------------------------------------------
;  
;  This code is based on:
;  
;   ROM BIOS for use with Bochs/Plex86/QEMU emulation environment
;  
;   Copyright (C) 2002  MandrakeSoft S.A.
;  
;     MandrakeSoft S.A.
;     43, rue d'Aboukir
;     75002 Paris - France
;     http://www.linux-mandrake.com/
;     http://www.mandrakesoft.com/
;  
;   This library is free software; you can redistribute it and/or
;   modify it under the terms of the GNU Lesser General Public
;   License as published by the Free Software Foundation; either
;   version 2 of the License, or (at your option) any later version.
;  
;   This library is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;   Lesser General Public License for more details.
;  
;   You should have received a copy of the GNU Lesser General Public
;   License along with this library; if not, write to the Free Software
;   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
;  

;
; Source file: print.c
;
;  
;  Copyright (C) 2006-2011 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
;  --------------------------------------------------------------------
;  
;  This code is based on:
;  
;   ROM BIOS for use with Bochs/Plex86/QEMU emulation environment
;  
;   Copyright (C) 2002  MandrakeSoft S.A.
;  
;     MandrakeSoft S.A.
;     43, rue d'Aboukir
;     75002 Paris - France
;     http://www.linux-mandrake.com/
;     http://www.mandrakesoft.com/
;  
;   This library is free software; you can redistribute it and/or
;   modify it under the terms of the GNU Lesser General Public
;   License as published by the Free Software Foundation; either
;   version 2 of the License, or (at your option) any later version.
;  
;   This library is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;   Lesser General Public License for more details.
;  
;   You should have received a copy of the GNU Lesser General Public
;   License along with this library; if not, write to the Free Software
;   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
;  

;
; Source file: ata.c
;
;  
;  Copyright (C) 2006-2012 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
;  --------------------------------------------------------------------
;  
;  This code is based on:
;  
;   ROM BIOS for use with Bochs/Plex86/QEMU emulation environment
;  
;   Copyright (C) 2002  MandrakeSoft S.A.
;  
;     MandrakeSoft S.A.
;     43, rue d'Aboukir
;     75002 Paris - France
;     http://www.linux-mandrake.com/
;     http://www.mandrakesoft.com/
;  
;   This library is free software; you can redistribute it and/or
;   modify it under the terms of the GNU Lesser General Public
;   License as published by the Free Software Foundation; either
;   version 2 of the License, or (at your option) any later version.
;  
;   This library is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;   Lesser General Public License for more details.
;  
;   You should have received a copy of the GNU Lesser General Public
;   License along with this library; if not, write to the Free Software
;   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
;  

;
; Source file: floppy.c
;
;  
;  Copyright (C) 2006-2011 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
;  --------------------------------------------------------------------
;  
;  This code is based on:
;  
;   ROM BIOS for use with Bochs/Plex86/QEMU emulation environment
;  
;   Copyright (C) 2002  MandrakeSoft S.A.
;  
;     MandrakeSoft S.A.
;     43, rue d'Aboukir
;     75002 Paris - France
;     http://www.linux-mandrake.com/
;     http://www.mandrakesoft.com/
;  
;   This library is free software; you can redistribute it and/or
;   modify it under the terms of the GNU Lesser General Public
;   License as published by the Free Software Foundation; either
;   version 2 of the License, or (at your option) any later version.
;  
;   This library is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;   Lesser General Public License for more details.
;  
;   You should have received a copy of the GNU Lesser General Public
;   License along with this library; if not, write to the Free Software
;   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
;  

;
; Source file: eltorito.c
;
;  
;  Copyright (C) 2006-2013 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
;  --------------------------------------------------------------------
;  
;  This code is based on:
;  
;   ROM BIOS for use with Bochs/Plex86/QEMU emulation environment
;  
;   Copyright (C) 2002  MandrakeSoft S.A.
;  
;     MandrakeSoft S.A.
;     43, rue d'Aboukir
;     75002 Paris - France
;     http://www.linux-mandrake.com/
;     http://www.mandrakesoft.com/
;  
;   This library is free software; you can redistribute it and/or
;   modify it under the terms of the GNU Lesser General Public
;   License as published by the Free Software Foundation; either
;   version 2 of the License, or (at your option) any later version.
;  
;   This library is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;   Lesser General Public License for more details.
;  
;   You should have received a copy of the GNU Lesser General Public
;   License along with this library; if not, write to the Free Software
;   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
;  

;
; Source file: boot.c
;
;  
;  Copyright (C) 2006-2012 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
;  --------------------------------------------------------------------
;  
;  This code is based on:
;  
;   ROM BIOS for use with Bochs/Plex86/QEMU emulation environment
;  
;   Copyright (C) 2002  MandrakeSoft S.A.
;  
;     MandrakeSoft S.A.
;     43, rue d'Aboukir
;     75002 Paris - France
;     http://www.linux-mandrake.com/
;     http://www.mandrakesoft.com/
;  
;   This library is free software; you can redistribute it and/or
;   modify it under the terms of the GNU Lesser General Public
;   License as published by the Free Software Foundation; either
;   version 2 of the License, or (at your option) any later version.
;  
;   This library is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;   Lesser General Public License for more details.
;  
;   You should have received a copy of the GNU Lesser General Public
;   License along with this library; if not, write to the Free Software
;   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
;  

;
; Source file: keyboard.c
;
;  
;  Copyright (C) 2006-2012 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
;  --------------------------------------------------------------------
;  
;  This code is based on:
;  
;   ROM BIOS for use with Bochs/Plex86/QEMU emulation environment
;  
;   Copyright (C) 2002  MandrakeSoft S.A.
;  
;     MandrakeSoft S.A.
;     43, rue d'Aboukir
;     75002 Paris - France
;     http://www.linux-mandrake.com/
;     http://www.mandrakesoft.com/
;  
;   This library is free software; you can redistribute it and/or
;   modify it under the terms of the GNU Lesser General Public
;   License as published by the Free Software Foundation; either
;   version 2 of the License, or (at your option) any later version.
;  
;   This library is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;   Lesser General Public License for more details.
;  
;   You should have received a copy of the GNU Lesser General Public
;   License along with this library; if not, write to the Free Software
;   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
;  

;
; Source file: disk.c
;
;  
;  Copyright (C) 2006-2011 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
;  --------------------------------------------------------------------
;  
;  This code is based on:
;  
;   ROM BIOS for use with Bochs/Plex86/QEMU emulation environment
;  
;   Copyright (C) 2002  MandrakeSoft S.A.
;  
;     MandrakeSoft S.A.
;     43, rue d'Aboukir
;     75002 Paris - France
;     http://www.linux-mandrake.com/
;     http://www.mandrakesoft.com/
;  
;   This library is free software; you can redistribute it and/or
;   modify it under the terms of the GNU Lesser General Public
;   License as published by the Free Software Foundation; either
;   version 2 of the License, or (at your option) any later version.
;  
;   This library is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;   Lesser General Public License for more details.
;  
;   You should have received a copy of the GNU Lesser General Public
;   License along with this library; if not, write to the Free Software
;   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
;  

;
; Source file: serial.c
;
;  
;  Copyright (C) 2006-2012 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
;  --------------------------------------------------------------------
;  
;  This code is based on:
;  
;   ROM BIOS for use with Bochs/Plex86/QEMU emulation environment
;  
;   Copyright (C) 2002  MandrakeSoft S.A.
;  
;     MandrakeSoft S.A.
;     43, rue d'Aboukir
;     75002 Paris - France
;     http://www.linux-mandrake.com/
;     http://www.mandrakesoft.com/
;  
;   This library is free software; you can redistribute it and/or
;   modify it under the terms of the GNU Lesser General Public
;   License as published by the Free Software Foundation; either
;   version 2 of the License, or (at your option) any later version.
;  
;   This library is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;   Lesser General Public License for more details.
;  
;   You should have received a copy of the GNU Lesser General Public
;   License along with this library; if not, write to the Free Software
;   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
;  

;
; Source file: system.c
;
;  
;  Copyright (C) 2006-2012 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
;  --------------------------------------------------------------------
;  
;  This code is based on:
;  
;   ROM BIOS for use with Bochs/Plex86/QEMU emulation environment
;  
;   Copyright (C) 2002  MandrakeSoft S.A.
;  
;     MandrakeSoft S.A.
;     43, rue d'Aboukir
;     75002 Paris - France
;     http://www.linux-mandrake.com/
;     http://www.mandrakesoft.com/
;  
;   This library is free software; you can redistribute it and/or
;   modify it under the terms of the GNU Lesser General Public
;   License as published by the Free Software Foundation; either
;   version 2 of the License, or (at your option) any later version.
;  
;   This library is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;   Lesser General Public License for more details.
;  
;   You should have received a copy of the GNU Lesser General Public
;   License along with this library; if not, write to the Free Software
;   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
;  

;
; Source file: timepci.c
;
;  
;  Copyright (C) 2006-2012 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
;  --------------------------------------------------------------------
;  
;  This code is based on:
;  
;   ROM BIOS for use with Bochs/Plex86/QEMU emulation environment
;  
;   Copyright (C) 2002  MandrakeSoft S.A.
;  
;     MandrakeSoft S.A.
;     43, rue d'Aboukir
;     75002 Paris - France
;     http://www.linux-mandrake.com/
;     http://www.mandrakesoft.com/
;  
;   This library is free software; you can redistribute it and/or
;   modify it under the terms of the GNU Lesser General Public
;   License as published by the Free Software Foundation; either
;   version 2 of the License, or (at your option) any later version.
;  
;   This library is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;   Lesser General Public License for more details.
;  
;   You should have received a copy of the GNU Lesser General Public
;   License along with this library; if not, write to the Free Software
;   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
;  

;
; Source file: ps2mouse.c
;
;  
;  Copyright (C) 2006-2011 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
;  --------------------------------------------------------------------
;  
;  This code is based on:
;  
;   ROM BIOS for use with Bochs/Plex86/QEMU emulation environment
;  
;   Copyright (C) 2002  MandrakeSoft S.A.
;  
;     MandrakeSoft S.A.
;     43, rue d'Aboukir
;     75002 Paris - France
;     http://www.linux-mandrake.com/
;     http://www.mandrakesoft.com/
;  
;   This library is free software; you can redistribute it and/or
;   modify it under the terms of the GNU Lesser General Public
;   License as published by the Free Software Foundation; either
;   version 2 of the License, or (at your option) any later version.
;  
;   This library is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;   Lesser General Public License for more details.
;  
;   You should have received a copy of the GNU Lesser General Public
;   License along with this library; if not, write to the Free Software
;   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
;  

;
; Source file: parallel.c
;
;  
;  Copyright (C) 2006-2012 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
;  --------------------------------------------------------------------
;  
;  This code is based on:
;  
;   ROM BIOS for use with Bochs/Plex86/QEMU emulation environment
;  
;   Copyright (C) 2002  MandrakeSoft S.A.
;  
;     MandrakeSoft S.A.
;     43, rue d'Aboukir
;     75002 Paris - France
;     http://www.linux-mandrake.com/
;     http://www.mandrakesoft.com/
;  
;   This library is free software; you can redistribute it and/or
;   modify it under the terms of the GNU Lesser General Public
;   License as published by the Free Software Foundation; either
;   version 2 of the License, or (at your option) any later version.
;  
;   This library is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;   Lesser General Public License for more details.
;  
;   You should have received a copy of the GNU Lesser General Public
;   License along with this library; if not, write to the Free Software
;   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
;  

;
; Source file: logo.c
;
;  $Id: VBoxBiosAlternative.asm $
;  Stuff for drawing the BIOS logo.
;  
;  
;  
;  Copyright (C) 2004-2012 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.

;
; Source file: scsi.c
;
;  $Id: VBoxBiosAlternative.asm $
;  SCSI host adapter driver to boot from SCSI disks
;  
;  
;  
;  Copyright (C) 2004-2012 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.

;
; Source file: ahci.c
;
;  $Id: VBoxBiosAlternative.asm $
;  AHCI host adapter driver to boot from SATA disks.
;  
;  
;  
;  Copyright (C) 2011-2012 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.

;
; Source file: apm.c
;
;  $Id: VBoxBiosAlternative.asm $
;  APM BIOS support. Implements APM version 1.2.
;  
;  
;  
;  Copyright (C) 2004-2012 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.

;
; Source file: pcibios.c
;
;  $Id: VBoxBiosAlternative.asm $
;  PCI BIOS support.
;  
;  
;  
;  Copyright (C) 2004-2012 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.

;
; Source file: pciutil.c
;
;  Utility routines for calling the PCI BIOS.
;  
;  
;  
;  Copyright (C) 2011 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.

;
; Source file: vds.c
;
;  Utility routines for calling the Virtual DMA Services.
;  
;  
;  
;  Copyright (C) 2011 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.

;
; Source file: support.asm
;
;  $Id: VBoxBiosAlternative.asm $
;  Compiler support routines.
;  
;  
;  
;  Copyright (C) 2012 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
;  

;
; Source file: pcibio32.asm
;
;  
;  Copyright (C) 2006-2012 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
;  
;  --------------------------------------------------------------------

;
; Source file: apm_pm.asm
;
;  
;  Copyright (C) 2006-2012 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
;  
;  --------------------------------------------------------------------
;  
;  Protected-mode APM implementation.
;  

;
; Source file: orgs.asm
;
;  
;  Copyright (C) 2006-2011 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
;  --------------------------------------------------------------------
;  
;  This code is based on:
;  
;   ROM BIOS for use with Bochs/Plex86/QEMU emulation environment
;  
;   Copyright (C) 2002  MandrakeSoft S.A.
;  
;     MandrakeSoft S.A.
;     43, rue d'Aboukir
;     75002 Paris - France
;     http://www.linux-mandrake.com/
;     http://www.mandrakesoft.com/
;  
;   This library is free software; you can redistribute it and/or
;   modify it under the terms of the GNU Lesser General Public
;   License as published by the Free Software Foundation; either
;   version 2 of the License, or (at your option) any later version.
;  
;   This library is distributed in the hope that it will be useful,
;   but WITHOUT ANY WARRANTY; without even the implied warranty of
;   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;   Lesser General Public License for more details.
;  
;   You should have received a copy of the GNU Lesser General Public
;   License along with this library; if not, write to the Free Software
;   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
;  
;  

;
; Source file: pci32.c
;
;  $Id: VBoxBiosAlternative.asm $
;  32-bit PCI BIOS wrapper.
;  
;  
;  
;  Copyright (C) 2004-2012 Oracle Corporation
;  
;  This file is part of VirtualBox Open Source Edition (OSE), as
;  available from http://www.virtualbox.org. This file is free software;
;  you can redistribute it and/or modify it under the terms of the GNU
;  General Public License (GPL) as published by the Free Software
;  Foundation, in version 2 as it comes in the "COPYING" file of the
;  VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;  hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.




section _DATA progbits vstart=0x0 align=1 ; size=0x30 class=DATA group=DGROUP
_dskacc:                                     ; 0xf0000 LB 0x30
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 053h, 027h, 0cah, 027h, 000h, 000h, 000h, 000h
    db  045h, 073h, 0e7h, 073h, 062h, 07eh, 0f3h, 07eh, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  05fh, 033h, 032h, 05fh, 000h, 0dah, 00fh, 000h, 000h, 001h, 0f3h, 000h, 000h, 000h, 000h, 000h

section CONST progbits vstart=0x30 align=1 ; size=0xc96 class=DATA group=DGROUP
    db   'NMI Handler called', 00ah, 000h
    db   'INT18: BOOT FAILURE', 00ah, 000h
    db   '%s', 00ah, 000h, 000h
    db   'FATAL: ', 000h
    db   'bios_printf: unknown format', 00ah, 000h, 000h
    db   'ata-detect: Failed to detect ATA device', 00ah, 000h
    db   'ata%d-%d: PCHS=%u/%d/%d LCHS=%u/%u/%u', 00ah, 000h
    db   'ata-detect: Failed to detect ATAPI device', 00ah, 000h
    db   ' slave', 000h
    db   'master', 000h
    db   'ata%d %s: ', 000h
    db   '%c', 000h
    db   ' ATA-%d Hard-Disk (%lu MBytes)', 00ah, 000h
    db   ' ATAPI-%d CD-ROM/DVD-ROM', 00ah, 000h
    db   ' ATAPI-%d Device', 00ah, 000h
    db   'ata%d %s: Unknown device', 00ah, 000h
    db   'ata_cmd_packet', 000h
    db   '%s: DATA_OUT not supported yet', 00ah, 000h
    db   'set_diskette_current_cyl: drive > 1', 00ah, 000h
    db   'int13_diskette_function', 000h
    db   '%s: drive>1 || head>1 ...', 00ah, 000h
    db   '%s: ctrl not ready', 00ah, 000h
    db   '%s: read error', 00ah, 000h
    db   '%s: write error', 00ah, 000h
    db   '%s: bad floppy type', 00ah, 000h
    db   '%s: unsupported AH=%02x', 00ah, 000h, 000h
    db   'int13_eltorito', 000h
    db   '%s: call with AX=%04x not implemented.', 00ah, 000h
    db   '%s: unsupported AH=%02x', 00ah, 000h
    db   'int13_cdemu', 000h
    db   '%s: function %02x, emulation not active for DL= %02x', 00ah, 000h
    db   '%s: function %02x, error %02x !', 00ah, 000h
    db   '%s: function AH=%02x unsupported, returns fail', 00ah, 000h
    db   'int13_cdrom', 000h
    db   '%s: function %02x, ELDL out of range %02x', 00ah, 000h
    db   '%s: function %02x, unmapped device for ELDL=%02x', 00ah, 000h
    db   '%s: function %02x. Can', 027h, 't use 64bits lba', 00ah, 000h
    db   '%s: function %02x, status %02x !', 00ah, 000h, 000h
    db   'Booting from %s...', 00ah, 000h
    db   'Boot from %s failed', 00ah, 000h
    db   'Boot from %s %d failed', 00ah, 000h
    db   'No bootable medium found! System halted.', 00ah, 000h
    db   'Could not read from the boot medium! System halted.', 00ah, 000h
    db   'CDROM boot failure code : %04x', 00ah, 000h
    db   'Boot : bseqnr=%d, bootseq=%x', 00dh, 00ah, 000h, 000h
    db   'Keyboard error:%u', 00ah, 000h
    db   'KBD: int09 handler: AL=0', 00ah, 000h
    db   'KBD: int09h_handler(): unknown scancode read: 0x%02x!', 00ah, 000h
    db   'KBD: int09h_handler(): scancode & asciicode are zero?', 00ah, 000h
    db   'KBD: int16h: out of keyboard input', 00ah, 000h
    db   'KBD: unsupported int 16h function %02x', 00ah, 000h
    db   'AX=%04x BX=%04x CX=%04x DX=%04x ', 00ah, 000h, 000h
    db   'int13_harddisk', 000h
    db   '%s: function %02x, ELDL out of range %02x', 00ah, 000h
    db   '%s: function %02x, unmapped device for ELDL=%02x', 00ah, 000h
    db   '%s: function %02x, count out of range!', 00ah, 000h
    db   '%s: function %02x, disk %02x, parameters out of range %04x/%04x/%04x!', 00ah
    db   000h
    db   '%s: function %02x, error %02x !', 00ah, 000h
    db   'format disk track called', 00ah, 000h
    db   '%s: function %02xh unimplemented, returns success', 00ah, 000h
    db   '%s: function %02xh unsupported, returns fail', 00ah, 000h
    db   'int13_harddisk_ext', 000h
    db   '%s: function %02x. Can', 027h, 't use 64bits lba', 00ah, 000h
    db   '%s: function %02x. LBA out of range', 00ah, 000h
    db   'int15: Func 24h, subfunc %02xh, A20 gate control not supported', 00ah, 000h
    db   '*** int 15h function AH=bf not yet supported!', 00ah, 000h
    db   'EISA BIOS not present', 00ah, 000h
    db   '*** int 15h function AX=%04x, BX=%04x not yet supported!', 00ah, 000h
    db   'sendmouse', 000h
    db   'setkbdcomm', 000h
    db   'Mouse reset returned %02x (should be ack)', 00ah, 000h
    db   'Mouse status returned %02x (should be ack)', 00ah, 000h
    db   'INT 15h C2 AL=6, BH=%02x', 00ah, 000h
    db   'INT 15h C2 default case entered', 00ah, 000h, 000h
    db   'Key pressed: %x', 00ah, 000h
    db   00ah, 00ah, 'AHCI controller:', 00ah, 000h
    db   00ah, '    %d) Hard disk', 000h
    db   00ah, 00ah, 'SCSI controller:', 00ah, 000h
    db   'IDE controller:', 00ah, 000h
    db   00ah, '    %d) ', 000h
    db   'Secondary ', 000h
    db   'Primary ', 000h
    db   'Slave', 000h
    db   'Master', 000h
    db   'No hard disks found', 000h
    db   00ah, 000h
    db   'Press F12 to select boot device.', 00ah, 000h
    db   00ah, 'VirtualBox temporary boot device selection', 00ah, 00ah, 'Detected H'
    db   'ard disks:', 00ah, 00ah, 000h
    db   00ah, 'Other boot devices:', 00ah, ' f) Floppy', 00ah, ' c) CD-ROM', 00ah
    db   ' l) LAN', 00ah, 00ah, ' b) Continue booting', 00ah, 000h
    db   'Delaying boot for %d seconds:', 000h
    db   ' %d', 000h
    db   'scsi_read_sectors: device_id out of range %d', 00ah, 000h
    db   'scsi_write_sectors: device_id out of range %d', 00ah, 000h
    db   'scsi_enumerate_attached_devices: SCSI_INQUIRY failed', 00ah, 000h
    db   'scsi_enumerate_attached_devices: SCSI_READ_CAPACITY failed', 00ah, 000h
    db   'Disk %d has an unsupported sector size of %u', 00ah, 000h, 000h
    db   'ahci_read_sectors', 000h
    db   '%s: device_id out of range %d', 00ah, 000h
    db   'ahci_write_sectors', 000h
    db   'ahci_cmd_packet', 000h
    db   '%s: DATA_OUT not supported yet', 00ah, 000h
    db   'AHCI %d-P#%d: PCHS=%u/%d/%d LCHS=%u/%u/%u %ld sectors', 00ah, 000h, 000h
    db   'Standby', 000h
    db   'Suspend', 000h
    db   'Shutdown', 000h
    db   'APM: Unsupported function AX=%04X BX=%04X called', 00ah, 000h, 000h
    db   'PCI: Unsupported function AX=%04X BX=%04X called', 00ah, 000h

section CONST2 progbits vstart=0xcc6 align=1 ; size=0x3fa class=DATA group=DGROUP
_bios_cvs_version_string:                    ; 0xf0cc6 LB 0x12
    db  'VirtualBox 4.2.12', 000h
_bios_prefix_string:                         ; 0xf0cd8 LB 0x8
    db  'BIOS: ', 000h, 000h
_isotag:                                     ; 0xf0ce0 LB 0x6
    db  'CD001', 000h
_eltorito:                                   ; 0xf0ce6 LB 0x18
    db  'EL TORITO SPECIFICATION', 000h
_drivetypes:                                 ; 0xf0cfe LB 0x28
    db  046h, 06ch, 06fh, 070h, 070h, 079h, 000h, 000h, 000h, 000h, 048h, 061h, 072h, 064h, 020h, 044h
    db  069h, 073h, 06bh, 000h, 043h, 044h, 02dh, 052h, 04fh, 04dh, 000h, 000h, 000h, 000h, 04ch, 041h
    db  04eh, 000h, 000h, 000h, 000h, 000h, 000h, 000h
_scan_to_scanascii:                          ; 0xf0d26 LB 0x37a
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 01bh, 001h, 01bh, 001h, 01bh, 001h
    db  000h, 001h, 000h, 000h, 031h, 002h, 021h, 002h, 000h, 000h, 000h, 078h, 000h, 000h, 032h, 003h
    db  040h, 003h, 000h, 003h, 000h, 079h, 000h, 000h, 033h, 004h, 023h, 004h, 000h, 000h, 000h, 07ah
    db  000h, 000h, 034h, 005h, 024h, 005h, 000h, 000h, 000h, 07bh, 000h, 000h, 035h, 006h, 025h, 006h
    db  000h, 000h, 000h, 07ch, 000h, 000h, 036h, 007h, 05eh, 007h, 01eh, 007h, 000h, 07dh, 000h, 000h
    db  037h, 008h, 026h, 008h, 000h, 000h, 000h, 07eh, 000h, 000h, 038h, 009h, 02ah, 009h, 000h, 000h
    db  000h, 07fh, 000h, 000h, 039h, 00ah, 028h, 00ah, 000h, 000h, 000h, 080h, 000h, 000h, 030h, 00bh
    db  029h, 00bh, 000h, 000h, 000h, 081h, 000h, 000h, 02dh, 00ch, 05fh, 00ch, 01fh, 00ch, 000h, 082h
    db  000h, 000h, 03dh, 00dh, 02bh, 00dh, 000h, 000h, 000h, 083h, 000h, 000h, 008h, 00eh, 008h, 00eh
    db  07fh, 00eh, 000h, 000h, 000h, 000h, 009h, 00fh, 000h, 00fh, 000h, 000h, 000h, 000h, 000h, 000h
    db  071h, 010h, 051h, 010h, 011h, 010h, 000h, 010h, 040h, 000h, 077h, 011h, 057h, 011h, 017h, 011h
    db  000h, 011h, 040h, 000h, 065h, 012h, 045h, 012h, 005h, 012h, 000h, 012h, 040h, 000h, 072h, 013h
    db  052h, 013h, 012h, 013h, 000h, 013h, 040h, 000h, 074h, 014h, 054h, 014h, 014h, 014h, 000h, 014h
    db  040h, 000h, 079h, 015h, 059h, 015h, 019h, 015h, 000h, 015h, 040h, 000h, 075h, 016h, 055h, 016h
    db  015h, 016h, 000h, 016h, 040h, 000h, 069h, 017h, 049h, 017h, 009h, 017h, 000h, 017h, 040h, 000h
    db  06fh, 018h, 04fh, 018h, 00fh, 018h, 000h, 018h, 040h, 000h, 070h, 019h, 050h, 019h, 010h, 019h
    db  000h, 019h, 040h, 000h, 05bh, 01ah, 07bh, 01ah, 01bh, 01ah, 000h, 000h, 000h, 000h, 05dh, 01bh
    db  07dh, 01bh, 01dh, 01bh, 000h, 000h, 000h, 000h, 00dh, 01ch, 00dh, 01ch, 00ah, 01ch, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 061h, 01eh, 041h, 01eh
    db  001h, 01eh, 000h, 01eh, 040h, 000h, 073h, 01fh, 053h, 01fh, 013h, 01fh, 000h, 01fh, 040h, 000h
    db  064h, 020h, 044h, 020h, 004h, 020h, 000h, 020h, 040h, 000h, 066h, 021h, 046h, 021h, 006h, 021h
    db  000h, 021h, 040h, 000h, 067h, 022h, 047h, 022h, 007h, 022h, 000h, 022h, 040h, 000h, 068h, 023h
    db  048h, 023h, 008h, 023h, 000h, 023h, 040h, 000h, 06ah, 024h, 04ah, 024h, 00ah, 024h, 000h, 024h
    db  040h, 000h, 06bh, 025h, 04bh, 025h, 00bh, 025h, 000h, 025h, 040h, 000h, 06ch, 026h, 04ch, 026h
    db  00ch, 026h, 000h, 026h, 040h, 000h, 03bh, 027h, 03ah, 027h, 000h, 000h, 000h, 000h, 000h, 000h
    db  027h, 028h, 022h, 028h, 000h, 000h, 000h, 000h, 000h, 000h, 060h, 029h, 07eh, 029h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 05ch, 02bh
    db  07ch, 02bh, 01ch, 02bh, 000h, 000h, 000h, 000h, 07ah, 02ch, 05ah, 02ch, 01ah, 02ch, 000h, 02ch
    db  040h, 000h, 078h, 02dh, 058h, 02dh, 018h, 02dh, 000h, 02dh, 040h, 000h, 063h, 02eh, 043h, 02eh
    db  003h, 02eh, 000h, 02eh, 040h, 000h, 076h, 02fh, 056h, 02fh, 016h, 02fh, 000h, 02fh, 040h, 000h
    db  062h, 030h, 042h, 030h, 002h, 030h, 000h, 030h, 040h, 000h, 06eh, 031h, 04eh, 031h, 00eh, 031h
    db  000h, 031h, 040h, 000h, 06dh, 032h, 04dh, 032h, 00dh, 032h, 000h, 032h, 040h, 000h, 02ch, 033h
    db  03ch, 033h, 000h, 000h, 000h, 000h, 000h, 000h, 02eh, 034h, 03eh, 034h, 000h, 000h, 000h, 000h
    db  000h, 000h, 02fh, 035h, 03fh, 035h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 02ah, 037h, 02ah, 037h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 020h, 039h, 020h, 039h, 020h, 039h
    db  020h, 039h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 03bh
    db  000h, 054h, 000h, 05eh, 000h, 068h, 000h, 000h, 000h, 03ch, 000h, 055h, 000h, 05fh, 000h, 069h
    db  000h, 000h, 000h, 03dh, 000h, 056h, 000h, 060h, 000h, 06ah, 000h, 000h, 000h, 03eh, 000h, 057h
    db  000h, 061h, 000h, 06bh, 000h, 000h, 000h, 03fh, 000h, 058h, 000h, 062h, 000h, 06ch, 000h, 000h
    db  000h, 040h, 000h, 059h, 000h, 063h, 000h, 06dh, 000h, 000h, 000h, 041h, 000h, 05ah, 000h, 064h
    db  000h, 06eh, 000h, 000h, 000h, 042h, 000h, 05bh, 000h, 065h, 000h, 06fh, 000h, 000h, 000h, 043h
    db  000h, 05ch, 000h, 066h, 000h, 070h, 000h, 000h, 000h, 044h, 000h, 05dh, 000h, 067h, 000h, 071h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 047h, 037h, 047h, 000h, 077h, 000h, 000h, 020h, 000h
    db  000h, 048h, 038h, 048h, 000h, 000h, 000h, 000h, 020h, 000h, 000h, 049h, 039h, 049h, 000h, 084h
    db  000h, 000h, 020h, 000h, 02dh, 04ah, 02dh, 04ah, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 04bh
    db  034h, 04bh, 000h, 073h, 000h, 000h, 020h, 000h, 000h, 04ch, 035h, 04ch, 000h, 000h, 000h, 000h
    db  020h, 000h, 000h, 04dh, 036h, 04dh, 000h, 074h, 000h, 000h, 020h, 000h, 02bh, 04eh, 02bh, 04eh
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 04fh, 031h, 04fh, 000h, 075h, 000h, 000h, 020h, 000h
    db  000h, 050h, 032h, 050h, 000h, 000h, 000h, 000h, 020h, 000h, 000h, 051h, 033h, 051h, 000h, 076h
    db  000h, 000h, 020h, 000h, 000h, 052h, 030h, 052h, 000h, 000h, 000h, 000h, 020h, 000h, 000h, 053h
    db  02eh, 053h, 000h, 000h, 000h, 000h, 020h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 05ch, 056h, 07ch, 056h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 085h, 000h, 087h, 000h, 089h, 000h, 08bh, 000h, 000h
    db  000h, 086h, 000h, 088h, 000h, 08ah, 000h, 08ch, 000h, 000h
_panic_msg_keyb_buffer_full:                 ; 0xf10a0 LB 0x20
    db  '%s: keyboard input buffer full', 00ah, 000h

  ; Padding 0x540 bytes at 0xf10c0
  times 1344 db 0

section _TEXT progbits vstart=0x1600 align=1 ; size=0x7745 class=CODE group=AUTO
read_byte_:                                  ; 0xf1600 LB 0xe
    push bx                                   ; 53
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bx, dx                                ; 89 d3
    mov es, ax                                ; 8e c0
    mov al, byte [es:bx]                      ; 26 8a 07
    pop bp                                    ; 5d
    pop bx                                    ; 5b
    retn                                      ; c3
write_byte_:                                 ; 0xf160e LB 0xe
    push si                                   ; 56
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov si, dx                                ; 89 d6
    mov es, ax                                ; 8e c0
    mov byte [es:si], bl                      ; 26 88 1c
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn                                      ; c3
read_word_:                                  ; 0xf161c LB 0xe
    push bx                                   ; 53
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bx, dx                                ; 89 d3
    mov es, ax                                ; 8e c0
    mov ax, word [es:bx]                      ; 26 8b 07
    pop bp                                    ; 5d
    pop bx                                    ; 5b
    retn                                      ; c3
write_word_:                                 ; 0xf162a LB 0xe
    push si                                   ; 56
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov si, dx                                ; 89 d6
    mov es, ax                                ; 8e c0
    mov word [es:si], bx                      ; 26 89 1c
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn                                      ; c3
read_dword_:                                 ; 0xf1638 LB 0x12
    push bx                                   ; 53
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bx, dx                                ; 89 d3
    mov es, ax                                ; 8e c0
    mov ax, word [es:bx]                      ; 26 8b 07
    mov dx, word [es:bx+002h]                 ; 26 8b 57 02
    pop bp                                    ; 5d
    pop bx                                    ; 5b
    retn                                      ; c3
write_dword_:                                ; 0xf164a LB 0x12
    push si                                   ; 56
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov si, dx                                ; 89 d6
    mov es, ax                                ; 8e c0
    mov word [es:si], bx                      ; 26 89 1c
    mov word [es:si+002h], cx                 ; 26 89 4c 02
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn                                      ; c3
inb_cmos_:                                   ; 0xf165c LB 0x11
    push dx                                   ; 52
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov dx, strict word 00070h                ; ba 70 00
    out DX, AL                                ; ee
    mov dx, strict word 00071h                ; ba 71 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    pop bp                                    ; 5d
    pop dx                                    ; 5a
    retn                                      ; c3
outb_cmos_:                                  ; 0xf166d LB 0x11
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov ah, dl                                ; 88 d4
    mov dx, strict word 00070h                ; ba 70 00
    out DX, AL                                ; ee
    mov al, ah                                ; 88 e0
    mov dx, strict word 00071h                ; ba 71 00
    out DX, AL                                ; ee
    pop bp                                    ; 5d
    retn                                      ; c3
_dummy_isr_function:                         ; 0xf167e LB 0x69
    enter 00002h, 000h                        ; c8 02 00 00
    mov CL, strict byte 0ffh                  ; b1 ff
    mov AL, strict byte 00bh                  ; b0 0b
    mov dx, strict word 00020h                ; ba 20 00
    out DX, AL                                ; ee
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov bx, ax                                ; 89 c3
    mov byte [bp-002h], al                    ; 88 46 fe
    test al, al                               ; 84 c0
    je short 016d9h                           ; 74 43
    mov AL, strict byte 00bh                  ; b0 0b
    mov dx, 000a0h                            ; ba a0 00
    out DX, AL                                ; ee
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov cx, ax                                ; 89 c1
    test al, al                               ; 84 c0
    je short 016bbh                           ; 74 16
    mov dx, 000a1h                            ; ba a1 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    xor ah, ah                                ; 30 e4
    movzx bx, cl                              ; 0f b6 d9
    or ax, bx                                 ; 09 d8
    out DX, AL                                ; ee
    mov AL, strict byte 020h                  ; b0 20
    mov dx, 000a0h                            ; ba a0 00
    out DX, AL                                ; ee
    jmp short 016d0h                          ; eb 15
    mov dx, strict word 00021h                ; ba 21 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    and bl, 0fbh                              ; 80 e3 fb
    mov byte [bp-002h], bl                    ; 88 5e fe
    xor ah, ah                                ; 30 e4
    movzx bx, byte [bp-002h]                  ; 0f b6 5e fe
    or ax, bx                                 ; 09 d8
    out DX, AL                                ; ee
    mov AL, strict byte 020h                  ; b0 20
    mov dx, strict word 00020h                ; ba 20 00
    out DX, AL                                ; ee
    mov cl, byte [bp-002h]                    ; 8a 4e fe
    movzx bx, cl                              ; 0f b6 d9
    mov dx, strict word 0006bh                ; ba 6b 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 29 ff
    leave                                     ; c9
    retn                                      ; c3
_nmi_handler_msg:                            ; 0xf16e7 LB 0x10
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    push strict word 00030h                   ; 68 30 00
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 f7 01
    add sp, strict byte 00004h                ; 83 c4 04
    pop bp                                    ; 5d
    retn                                      ; c3
_int18_panic_msg:                            ; 0xf16f7 LB 0x10
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    push strict word 00044h                   ; 68 44 00
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 e7 01
    add sp, strict byte 00004h                ; 83 c4 04
    pop bp                                    ; 5d
    retn                                      ; c3
_log_bios_start:                             ; 0xf1707 LB 0x1e
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 99 01
    push 00cc6h                               ; 68 c6 0c
    push strict word 00059h                   ; 68 59 00
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 c9 01
    add sp, strict byte 00006h                ; 83 c4 06
    pop bp                                    ; 5d
    retn                                      ; c3
_print_bios_banner:                          ; 0xf1725 LB 0x2c
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov dx, strict word 00072h                ; ba 72 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 eb fe
    mov cx, ax                                ; 89 c1
    xor bx, bx                                ; 31 db
    mov dx, strict word 00072h                ; ba 72 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0162ah                               ; e8 ec fe
    cmp cx, 01234h                            ; 81 f9 34 12
    jne short 0174ch                          ; 75 08
    mov AL, strict byte 003h                  ; b0 03
    mov AH, strict byte 000h                  ; b4 00
    int 010h                                  ; cd 10
    pop bp                                    ; 5d
    retn                                      ; c3
    call 06ffbh                               ; e8 ac 58
    pop bp                                    ; 5d
    retn                                      ; c3
send_:                                       ; 0xf1751 LB 0x38
    push bx                                   ; 53
    push cx                                   ; 51
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bx, ax                                ; 89 c3
    mov cl, dl                                ; 88 d1
    test AL, strict byte 008h                 ; a8 08
    je short 01764h                           ; 74 06
    mov al, dl                                ; 88 d0
    mov dx, 00403h                            ; ba 03 04
    out DX, AL                                ; ee
    test bl, 004h                             ; f6 c3 04
    je short 0176fh                           ; 74 06
    mov al, cl                                ; 88 c8
    mov dx, 00504h                            ; ba 04 05
    out DX, AL                                ; ee
    test bl, 002h                             ; f6 c3 02
    je short 01785h                           ; 74 11
    cmp cl, 00ah                              ; 80 f9 0a
    jne short 0177fh                          ; 75 06
    mov AL, strict byte 00dh                  ; b0 0d
    mov AH, strict byte 00eh                  ; b4 0e
    int 010h                                  ; cd 10
    mov al, cl                                ; 88 c8
    mov AH, strict byte 00eh                  ; b4 0e
    int 010h                                  ; cd 10
    pop bp                                    ; 5d
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
put_int_:                                    ; 0xf1789 LB 0x5b
    push si                                   ; 56
    push di                                   ; 57
    enter 00004h, 000h                        ; c8 04 00 00
    mov si, ax                                ; 89 c6
    mov word [bp-004h], dx                    ; 89 56 fc
    mov di, strict word 0000ah                ; bf 0a 00
    mov ax, dx                                ; 89 d0
    cwd                                       ; 99
    idiv di                                   ; f7 ff
    mov word [bp-002h], ax                    ; 89 46 fe
    test ax, ax                               ; 85 c0
    je short 017adh                           ; 74 0a
    dec bx                                    ; 4b
    mov dx, ax                                ; 89 c2
    mov ax, si                                ; 89 f0
    call 01789h                               ; e8 de ff
    jmp short 017c8h                          ; eb 1b
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jle short 017bch                          ; 7e 0a
    mov dx, strict word 00020h                ; ba 20 00
    mov ax, si                                ; 89 f0
    call 01751h                               ; e8 97 ff
    jmp short 017adh                          ; eb f1
    test cx, cx                               ; 85 c9
    je short 017c8h                           ; 74 08
    mov dx, strict word 0002dh                ; ba 2d 00
    mov ax, si                                ; 89 f0
    call 01751h                               ; e8 89 ff
    mov al, byte [bp-002h]                    ; 8a 46 fe
    mov DL, strict byte 00ah                  ; b2 0a
    mul dl                                    ; f6 e2
    mov dl, byte [bp-004h]                    ; 8a 56 fc
    sub dl, al                                ; 28 c2
    mov al, dl                                ; 88 d0
    add AL, strict byte 030h                  ; 04 30
    movzx dx, al                              ; 0f b6 d0
    mov ax, si                                ; 89 f0
    call 01751h                               ; e8 71 ff
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn                                      ; c3
put_uint_:                                   ; 0xf17e4 LB 0x5c
    push si                                   ; 56
    push di                                   ; 57
    enter 00004h, 000h                        ; c8 04 00 00
    mov si, ax                                ; 89 c6
    mov word [bp-004h], dx                    ; 89 56 fc
    mov ax, dx                                ; 89 d0
    xor dx, dx                                ; 31 d2
    mov di, strict word 0000ah                ; bf 0a 00
    div di                                    ; f7 f7
    mov word [bp-002h], ax                    ; 89 46 fe
    test ax, ax                               ; 85 c0
    je short 01809h                           ; 74 0a
    dec bx                                    ; 4b
    mov dx, ax                                ; 89 c2
    mov ax, si                                ; 89 f0
    call 017e4h                               ; e8 dd ff
    jmp short 01824h                          ; eb 1b
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jle short 01818h                          ; 7e 0a
    mov dx, strict word 00020h                ; ba 20 00
    mov ax, si                                ; 89 f0
    call 01751h                               ; e8 3b ff
    jmp short 01809h                          ; eb f1
    test cx, cx                               ; 85 c9
    je short 01824h                           ; 74 08
    mov dx, strict word 0002dh                ; ba 2d 00
    mov ax, si                                ; 89 f0
    call 01751h                               ; e8 2d ff
    mov al, byte [bp-002h]                    ; 8a 46 fe
    mov DL, strict byte 00ah                  ; b2 0a
    mul dl                                    ; f6 e2
    mov dl, byte [bp-004h]                    ; 8a 56 fc
    sub dl, al                                ; 28 c2
    mov al, dl                                ; 88 d0
    add AL, strict byte 030h                  ; 04 30
    movzx dx, al                              ; 0f b6 d0
    mov ax, si                                ; 89 f0
    call 01751h                               ; e8 15 ff
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn                                      ; c3
put_luint_:                                  ; 0xf1840 LB 0x6e
    push si                                   ; 56
    push di                                   ; 57
    enter 00004h, 000h                        ; c8 04 00 00
    mov si, ax                                ; 89 c6
    mov word [bp-002h], bx                    ; 89 5e fe
    mov di, dx                                ; 89 d7
    mov ax, bx                                ; 89 d8
    mov dx, cx                                ; 89 ca
    mov bx, strict word 0000ah                ; bb 0a 00
    xor cx, cx                                ; 31 c9
    call 08c50h                               ; e8 f7 73
    mov word [bp-004h], ax                    ; 89 46 fc
    mov cx, dx                                ; 89 d1
    mov dx, ax                                ; 89 c2
    or dx, cx                                 ; 09 ca
    je short 01873h                           ; 74 0f
    push word [bp+008h]                       ; ff 76 08
    lea dx, [di-001h]                         ; 8d 55 ff
    mov bx, ax                                ; 89 c3
    mov ax, si                                ; 89 f0
    call 01840h                               ; e8 cf ff
    jmp short 01890h                          ; eb 1d
    dec di                                    ; 4f
    test di, di                               ; 85 ff
    jle short 01882h                          ; 7e 0a
    mov dx, strict word 00020h                ; ba 20 00
    mov ax, si                                ; 89 f0
    call 01751h                               ; e8 d1 fe
    jmp short 01873h                          ; eb f1
    cmp word [bp+008h], strict byte 00000h    ; 83 7e 08 00
    je short 01890h                           ; 74 08
    mov dx, strict word 0002dh                ; ba 2d 00
    mov ax, si                                ; 89 f0
    call 01751h                               ; e8 c1 fe
    mov al, byte [bp-004h]                    ; 8a 46 fc
    mov DL, strict byte 00ah                  ; b2 0a
    mul dl                                    ; f6 e2
    mov dl, byte [bp-002h]                    ; 8a 56 fe
    sub dl, al                                ; 28 c2
    mov al, dl                                ; 88 d0
    add AL, strict byte 030h                  ; 04 30
    movzx dx, al                              ; 0f b6 d0
    mov ax, si                                ; 89 f0
    call 01751h                               ; e8 a9 fe
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn 00002h                               ; c2 02 00
put_str_:                                    ; 0xf18ae LB 0x1e
    push dx                                   ; 52
    push si                                   ; 56
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov si, ax                                ; 89 c6
    mov es, cx                                ; 8e c1
    mov dl, byte [es:bx]                      ; 26 8a 17
    test dl, dl                               ; 84 d2
    je short 018c8h                           ; 74 0a
    xor dh, dh                                ; 30 f6
    mov ax, si                                ; 89 f0
    call 01751h                               ; e8 8c fe
    inc bx                                    ; 43
    jmp short 018b5h                          ; eb ed
    pop bp                                    ; 5d
    pop si                                    ; 5e
    pop dx                                    ; 5a
    retn                                      ; c3
put_str_near_:                               ; 0xf18cc LB 0x1d
    push bx                                   ; 53
    push cx                                   ; 51
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov cx, ax                                ; 89 c1
    mov bx, dx                                ; 89 d3
    mov dl, byte [bx]                         ; 8a 17
    test dl, dl                               ; 84 d2
    je short 018e5h                           ; 74 0a
    xor dh, dh                                ; 30 f6
    mov ax, cx                                ; 89 c8
    call 01751h                               ; e8 6f fe
    inc bx                                    ; 43
    jmp short 018d5h                          ; eb f0
    pop bp                                    ; 5d
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
bios_printf_:                                ; 0xf18e9 LB 0x236
    push bx                                   ; 53
    push cx                                   ; 51
    push dx                                   ; 52
    push si                                   ; 56
    push di                                   ; 57
    enter 00008h, 000h                        ; c8 08 00 00
    lea bx, [bp+012h]                         ; 8d 5e 12
    mov word [bp-008h], bx                    ; 89 5e f8
    mov [bp-006h], ss                         ; 8c 56 fa
    xor cx, cx                                ; 31 c9
    xor si, si                                ; 31 f6
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    and ax, strict word 00007h                ; 25 07 00
    cmp ax, strict word 00007h                ; 3d 07 00
    jne short 01915h                          ; 75 0b
    push strict word 0005eh                   ; 68 5e 00
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 d7 ff
    add sp, strict byte 00004h                ; 83 c4 04
    mov bx, word [bp+010h]                    ; 8b 5e 10
    mov dl, byte [bx]                         ; 8a 17
    test dl, dl                               ; 84 d2
    je near 01b06h                            ; 0f 84 e6 01
    cmp dl, 025h                              ; 80 fa 25
    jne short 0192dh                          ; 75 08
    mov cx, strict word 00001h                ; b9 01 00
    xor si, si                                ; 31 f6
    jmp near 01b00h                           ; e9 d3 01
    test cx, cx                               ; 85 c9
    je near 01af8h                            ; 0f 84 c5 01
    cmp dl, 030h                              ; 80 fa 30
    jc short 0194bh                           ; 72 13
    cmp dl, 039h                              ; 80 fa 39
    jnbe short 0194bh                         ; 77 0e
    movzx ax, dl                              ; 0f b6 c2
    imul si, si, strict byte 0000ah           ; 6b f6 0a
    sub ax, strict word 00030h                ; 2d 30 00
    add si, ax                                ; 01 c6
    jmp near 01b00h                           ; e9 b5 01
    mov ax, word [bp-006h]                    ; 8b 46 fa
    mov word [bp-006h], ax                    ; 89 46 fa
    add word [bp-008h], strict byte 00002h    ; 83 46 f8 02
    les bx, [bp-008h]                         ; c4 5e f8
    mov ax, word [es:bx-002h]                 ; 26 8b 47 fe
    mov word [bp-002h], ax                    ; 89 46 fe
    cmp dl, 078h                              ; 80 fa 78
    je short 01969h                           ; 74 05
    cmp dl, 058h                              ; 80 fa 58
    jne short 019b2h                          ; 75 49
    test si, si                               ; 85 f6
    jne short 01970h                          ; 75 03
    mov si, strict word 00004h                ; be 04 00
    cmp dl, 078h                              ; 80 fa 78
    jne short 0197ah                          ; 75 05
    mov di, strict word 00061h                ; bf 61 00
    jmp short 0197dh                          ; eb 03
    mov di, strict word 00041h                ; bf 41 00
    lea bx, [si-001h]                         ; 8d 5c ff
    test bx, bx                               ; 85 db
    jl near 01af4h                            ; 0f 8c 6e 01
    mov cx, bx                                ; 89 d9
    sal cx, 002h                              ; c1 e1 02
    mov ax, word [bp-002h]                    ; 8b 46 fe
    shr ax, CL                                ; d3 e8
    xor ah, ah                                ; 30 e4
    and AL, strict byte 00fh                  ; 24 0f
    cmp ax, strict word 00009h                ; 3d 09 00
    jnbe short 019a0h                         ; 77 07
    mov dx, ax                                ; 89 c2
    add dx, strict byte 00030h                ; 83 c2 30
    jmp short 019a7h                          ; eb 07
    mov dx, ax                                ; 89 c2
    sub dx, strict byte 0000ah                ; 83 ea 0a
    add dx, di                                ; 01 fa
    xor dh, dh                                ; 30 f6
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    call 01751h                               ; e8 a2 fd
    dec bx                                    ; 4b
    jmp short 01980h                          ; eb ce
    cmp dl, 075h                              ; 80 fa 75
    jne short 019c6h                          ; 75 0f
    xor cx, cx                                ; 31 c9
    mov bx, si                                ; 89 f3
    mov dx, ax                                ; 89 c2
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    call 017e4h                               ; e8 21 fe
    jmp near 01af4h                           ; e9 2e 01
    lea bx, [si-001h]                         ; 8d 5c ff
    cmp dl, 06ch                              ; 80 fa 6c
    jne near 01a82h                           ; 0f 85 b2 00
    inc word [bp+010h]                        ; ff 46 10
    mov di, word [bp+010h]                    ; 8b 7e 10
    mov dl, byte [di]                         ; 8a 15
    mov ax, word [bp-006h]                    ; 8b 46 fa
    mov word [bp-006h], ax                    ; 89 46 fa
    add word [bp-008h], strict byte 00002h    ; 83 46 f8 02
    les di, [bp-008h]                         ; c4 7e f8
    mov ax, word [es:di-002h]                 ; 26 8b 45 fe
    mov word [bp-004h], ax                    ; 89 46 fc
    cmp dl, 064h                              ; 80 fa 64
    jne short 01a1eh                          ; 75 2d
    test byte [bp-003h], 080h                 ; f6 46 fd 80
    je short 01a0ch                           ; 74 15
    push strict byte 00001h                   ; 6a 01
    mov ax, word [bp-002h]                    ; 8b 46 fe
    mov cx, word [bp-004h]                    ; 8b 4e fc
    neg cx                                    ; f7 d9
    neg ax                                    ; f7 d8
    sbb cx, strict byte 00000h                ; 83 d9 00
    mov dx, bx                                ; 89 da
    mov bx, ax                                ; 89 c3
    jmp short 01a15h                          ; eb 09
    push strict byte 00000h                   ; 6a 00
    mov bx, word [bp-002h]                    ; 8b 5e fe
    mov dx, si                                ; 89 f2
    mov cx, ax                                ; 89 c1
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    call 01840h                               ; e8 25 fe
    jmp near 01af4h                           ; e9 d6 00
    cmp dl, 075h                              ; 80 fa 75
    jne short 01a25h                          ; 75 02
    jmp short 01a0ch                          ; eb e7
    cmp dl, 078h                              ; 80 fa 78
    je short 01a31h                           ; 74 07
    cmp dl, 058h                              ; 80 fa 58
    jne near 01af4h                           ; 0f 85 c3 00
    test si, si                               ; 85 f6
    jne short 01a38h                          ; 75 03
    mov si, strict word 00008h                ; be 08 00
    cmp dl, 078h                              ; 80 fa 78
    jne short 01a42h                          ; 75 05
    mov di, strict word 00061h                ; bf 61 00
    jmp short 01a45h                          ; eb 03
    mov di, strict word 00041h                ; bf 41 00
    lea bx, [si-001h]                         ; 8d 5c ff
    test bx, bx                               ; 85 db
    jl near 01af4h                            ; 0f 8c a6 00
    mov ax, word [bp-002h]                    ; 8b 46 fe
    mov cx, bx                                ; 89 d9
    sal cx, 002h                              ; c1 e1 02
    mov dx, word [bp-004h]                    ; 8b 56 fc
    jcxz 01a61h                               ; e3 06
    shr dx, 1                                 ; d1 ea
    rcr ax, 1                                 ; d1 d8
    loop 01a5bh                               ; e2 fa
    and ax, strict word 0000fh                ; 25 0f 00
    cmp ax, strict word 00009h                ; 3d 09 00
    jnbe short 01a70h                         ; 77 07
    mov dx, ax                                ; 89 c2
    add dx, strict byte 00030h                ; 83 c2 30
    jmp short 01a77h                          ; eb 07
    mov dx, ax                                ; 89 c2
    sub dx, strict byte 0000ah                ; 83 ea 0a
    add dx, di                                ; 01 fa
    xor dh, dh                                ; 30 f6
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    call 01751h                               ; e8 d2 fc
    dec bx                                    ; 4b
    jmp short 01a48h                          ; eb c6
    cmp dl, 064h                              ; 80 fa 64
    jne short 01aa4h                          ; 75 1d
    test byte [bp-001h], 080h                 ; f6 46 ff 80
    je short 01a96h                           ; 74 09
    mov dx, ax                                ; 89 c2
    neg dx                                    ; f7 da
    mov cx, strict word 00001h                ; b9 01 00
    jmp short 01a9ch                          ; eb 06
    xor cx, cx                                ; 31 c9
    mov bx, si                                ; 89 f3
    mov dx, ax                                ; 89 c2
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    call 01789h                               ; e8 e7 fc
    jmp short 01af4h                          ; eb 50
    cmp dl, 073h                              ; 80 fa 73
    jne short 01ab5h                          ; 75 0c
    mov cx, ds                                ; 8c d9
    mov bx, ax                                ; 89 c3
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    call 018aeh                               ; e8 fb fd
    jmp short 01af4h                          ; eb 3f
    cmp dl, 053h                              ; 80 fa 53
    jne short 01ad8h                          ; 75 1e
    mov word [bp-004h], ax                    ; 89 46 fc
    mov ax, word [bp-006h]                    ; 8b 46 fa
    mov word [bp-006h], ax                    ; 89 46 fa
    add word [bp-008h], strict byte 00002h    ; 83 46 f8 02
    les bx, [bp-008h]                         ; c4 5e f8
    mov ax, word [es:bx-002h]                 ; 26 8b 47 fe
    mov word [bp-002h], ax                    ; 89 46 fe
    mov bx, ax                                ; 89 c3
    mov cx, word [bp-004h]                    ; 8b 4e fc
    jmp short 01aadh                          ; eb d5
    cmp dl, 063h                              ; 80 fa 63
    jne short 01ae9h                          ; 75 0c
    movzx dx, byte [bp-002h]                  ; 0f b6 56 fe
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    call 01751h                               ; e8 6a fc
    jmp short 01af4h                          ; eb 0b
    push strict word 00066h                   ; 68 66 00
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 f8 fd
    add sp, strict byte 00004h                ; 83 c4 04
    xor cx, cx                                ; 31 c9
    jmp short 01b00h                          ; eb 08
    xor dh, dh                                ; 30 f6
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    call 01751h                               ; e8 51 fc
    inc word [bp+010h]                        ; ff 46 10
    jmp near 01915h                           ; e9 0f fe
    xor ax, ax                                ; 31 c0
    mov word [bp-008h], ax                    ; 89 46 f8
    mov word [bp-006h], ax                    ; 89 46 fa
    test byte [bp+00eh], 001h                 ; f6 46 0e 01
    je short 01b18h                           ; 74 04
    cli                                       ; fa
    hlt                                       ; f4
    jmp short 01b15h                          ; eb fd
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop dx                                    ; 5a
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
_ata_init:                                   ; 0xf1b1f LB 0xc1
    push si                                   ; 56
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 f0 fa
    mov si, 00122h                            ; be 22 01
    mov dx, ax                                ; 89 c2
    xor al, al                                ; 30 c0
    jmp short 01b39h                          ; eb 04
    cmp AL, strict byte 004h                  ; 3c 04
    jnc short 01b5dh                          ; 73 24
    movzx bx, al                              ; 0f b6 d8
    imul bx, bx, strict byte 00006h           ; 6b db 06
    mov es, dx                                ; 8e c2
    add bx, si                                ; 01 f3
    mov byte [es:bx+001c0h], 000h             ; 26 c6 87 c0 01 00
    db  066h, 026h, 0c7h, 087h, 0c2h, 001h, 000h, 000h, 000h, 000h
    ; mov dword [es:bx+001c2h], strict dword 000000000h ; 66 26 c7 87 c2 01 00 00 00 00
    mov byte [es:bx+001c1h], 000h             ; 26 c6 87 c1 01 00
    db  0feh, 0c0h
    ; inc al                                    ; fe c0
    jmp short 01b35h                          ; eb d8
    xor al, al                                ; 30 c0
    jmp short 01b65h                          ; eb 04
    cmp AL, strict byte 008h                  ; 3c 08
    jnc short 01bb0h                          ; 73 4b
    movzx bx, al                              ; 0f b6 d8
    imul bx, bx, strict byte 00018h           ; 6b db 18
    mov es, dx                                ; 8e c2
    add bx, si                                ; 01 f3
    db  066h, 026h, 0c7h, 047h, 01eh, 000h, 000h, 000h, 000h
    ; mov dword [es:bx+01eh], strict dword 000000000h ; 66 26 c7 47 1e 00 00 00 00
    mov byte [es:bx+022h], 000h               ; 26 c6 47 22 00
    mov word [es:bx+024h], 00200h             ; 26 c7 47 24 00 02
    mov byte [es:bx+023h], 000h               ; 26 c6 47 23 00
    db  066h, 026h, 0c7h, 047h, 026h, 000h, 000h, 000h, 000h
    ; mov dword [es:bx+026h], strict dword 000000000h ; 66 26 c7 47 26 00 00 00 00
    db  066h, 026h, 0c7h, 047h, 02ah, 000h, 000h, 000h, 000h
    ; mov dword [es:bx+02ah], strict dword 000000000h ; 66 26 c7 47 2a 00 00 00 00
    db  066h, 026h, 0c7h, 047h, 02eh, 000h, 000h, 000h, 000h
    ; mov dword [es:bx+02eh], strict dword 000000000h ; 66 26 c7 47 2e 00 00 00 00
    db  066h, 026h, 0c7h, 047h, 032h, 000h, 000h, 000h, 000h
    ; mov dword [es:bx+032h], strict dword 000000000h ; 66 26 c7 47 32 00 00 00 00
    db  0feh, 0c0h
    ; inc al                                    ; fe c0
    jmp short 01b61h                          ; eb b1
    xor al, al                                ; 30 c0
    jmp short 01bb8h                          ; eb 04
    cmp AL, strict byte 010h                  ; 3c 10
    jnc short 01bcfh                          ; 73 17
    movzx bx, al                              ; 0f b6 d8
    mov es, dx                                ; 8e c2
    add bx, si                                ; 01 f3
    mov byte [es:bx+0019fh], 010h             ; 26 c6 87 9f 01 10
    mov byte [es:bx+001b0h], 010h             ; 26 c6 87 b0 01 10
    db  0feh, 0c0h
    ; inc al                                    ; fe c0
    jmp short 01bb4h                          ; eb e5
    mov es, dx                                ; 8e c2
    mov byte [es:si+0019eh], 000h             ; 26 c6 84 9e 01 00
    mov byte [es:si+001afh], 000h             ; 26 c6 84 af 01 00
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn                                      ; c3
ata_reset_:                                  ; 0xf1be0 LB 0xda
    push bx                                   ; 53
    push cx                                   ; 51
    push dx                                   ; 52
    push si                                   ; 56
    push di                                   ; 57
    enter 00004h, 000h                        ; c8 04 00 00
    push ax                                   ; 50
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 29 fa
    mov word [bp-004h], 00122h                ; c7 46 fc 22 01
    mov di, ax                                ; 89 c7
    mov bx, word [bp-006h]                    ; 8b 5e fa
    shr bx, 1                                 ; d1 eb
    mov dl, byte [bp-006h]                    ; 8a 56 fa
    and dl, 001h                              ; 80 e2 01
    mov byte [bp-002h], dl                    ; 88 56 fe
    xor bh, bh                                ; 30 ff
    imul bx, bx, strict byte 00006h           ; 6b db 06
    mov es, ax                                ; 8e c0
    add bx, 00122h                            ; 81 c3 22 01
    mov cx, word [es:bx+001c2h]               ; 26 8b 8f c2 01
    mov si, word [es:bx+001c4h]               ; 26 8b b7 c4 01
    lea dx, [si+006h]                         ; 8d 54 06
    mov AL, strict byte 00eh                  ; b0 0e
    out DX, AL                                ; ee
    mov bx, 000ffh                            ; bb ff 00
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 01c37h                          ; 76 0c
    mov dx, cx                                ; 89 ca
    add dx, strict byte 00007h                ; 83 c2 07
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 080h                 ; a8 80
    je short 01c26h                           ; 74 ef
    lea dx, [si+006h]                         ; 8d 54 06
    mov AL, strict byte 00ah                  ; b0 0a
    out DX, AL                                ; ee
    imul bx, word [bp-006h], strict byte 00018h ; 6b 5e fa 18
    mov es, di                                ; 8e c7
    add bx, word [bp-004h]                    ; 03 5e fc
    cmp byte [es:bx+01eh], 000h               ; 26 80 7f 1e 00
    je short 01c99h                           ; 74 4c
    cmp byte [bp-002h], 000h                  ; 80 7e fe 00
    je short 01c58h                           ; 74 05
    mov ax, 000b0h                            ; b8 b0 00
    jmp short 01c5bh                          ; eb 03
    mov ax, 000a0h                            ; b8 a0 00
    mov dx, cx                                ; 89 ca
    add dx, strict byte 00006h                ; 83 c2 06
    out DX, AL                                ; ee
    mov dx, cx                                ; 89 ca
    inc dx                                    ; 42
    inc dx                                    ; 42
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov bx, ax                                ; 89 c3
    mov dx, cx                                ; 89 ca
    add dx, strict byte 00003h                ; 83 c2 03
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    cmp bl, 001h                              ; 80 fb 01
    jne short 01c99h                          ; 75 22
    cmp al, bl                                ; 38 d8
    jne short 01c99h                          ; 75 1e
    mov bx, strict word 0ffffh                ; bb ff ff
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 01c99h                          ; 76 16
    mov dx, cx                                ; 89 ca
    add dx, strict byte 00007h                ; 83 c2 07
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 080h                 ; a8 80
    je short 01c99h                           ; 74 0a
    mov ax, strict word 0ffffh                ; b8 ff ff
    dec ax                                    ; 48
    test ax, ax                               ; 85 c0
    jnbe short 01c92h                         ; 77 fb
    jmp short 01c7eh                          ; eb e5
    mov bx, strict word 00010h                ; bb 10 00
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 01cadh                          ; 76 0c
    mov dx, cx                                ; 89 ca
    add dx, strict byte 00007h                ; 83 c2 07
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 040h                 ; a8 40
    je short 01c9ch                           ; 74 ef
    lea dx, [si+006h]                         ; 8d 54 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop dx                                    ; 5a
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
ata_cmd_data_in_:                            ; 0xf1cba LB 0x263
    push si                                   ; 56
    push di                                   ; 57
    enter 0001ch, 000h                        ; c8 1c 00 00
    mov si, ax                                ; 89 c6
    mov word [bp-006h], dx                    ; 89 56 fa
    mov word [bp-012h], bx                    ; 89 5e ee
    mov word [bp-010h], cx                    ; 89 4e f0
    mov es, dx                                ; 8e c2
    movzx ax, byte [es:si+008h]               ; 26 0f b6 44 08
    mov dx, ax                                ; 89 c2
    shr dx, 1                                 ; d1 ea
    mov dh, al                                ; 88 c6
    and dh, 001h                              ; 80 e6 01
    mov byte [bp-004h], dh                    ; 88 76 fc
    movzx di, dl                              ; 0f b6 fa
    imul di, di, strict byte 00006h           ; 6b ff 06
    add di, si                                ; 01 f7
    mov bx, word [es:di+001c2h]               ; 26 8b 9d c2 01
    mov dx, word [es:di+001c4h]               ; 26 8b 95 c4 01
    mov word [bp-018h], dx                    ; 89 56 e8
    imul ax, ax, strict byte 00018h           ; 6b c0 18
    mov di, si                                ; 89 f7
    add di, ax                                ; 01 c7
    mov al, byte [es:di+022h]                 ; 26 8a 45 22
    mov byte [bp-002h], al                    ; 88 46 fe
    mov ax, word [es:di+024h]                 ; 26 8b 45 24
    mov word [bp-008h], ax                    ; 89 46 f8
    test ax, ax                               ; 85 c0
    jne short 01d20h                          ; 75 14
    cmp byte [bp-002h], 001h                  ; 80 7e fe 01
    jne short 01d19h                          ; 75 07
    mov word [bp-008h], 04000h                ; c7 46 f8 00 40
    jmp short 01d2fh                          ; eb 16
    mov word [bp-008h], 08000h                ; c7 46 f8 00 80
    jmp short 01d2fh                          ; eb 0f
    cmp byte [bp-002h], 001h                  ; 80 7e fe 01
    jne short 01d2ch                          ; 75 06
    shr word [bp-008h], 002h                  ; c1 6e f8 02
    jmp short 01d2fh                          ; eb 03
    shr word [bp-008h], 1                     ; d1 6e f8
    lea dx, [bx+007h]                         ; 8d 57 07
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 080h                 ; a8 80
    je short 01d48h                           ; 74 0f
    mov dx, word [bp-018h]                    ; 8b 56 e8
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    mov dx, strict word 00001h                ; ba 01 00
    jmp near 01f07h                           ; e9 bf 01
    mov es, [bp-006h]                         ; 8e 46 fa
    mov ax, word [es:si]                      ; 26 8b 04
    mov word [bp-01ch], ax                    ; 89 46 e4
    mov ax, word [es:si+002h]                 ; 26 8b 44 02
    mov word [bp-01ah], ax                    ; 89 46 e6
    mov di, word [es:si+004h]                 ; 26 8b 7c 04
    mov ax, word [es:si+006h]                 ; 26 8b 44 06
    mov word [bp-00ah], ax                    ; 89 46 f6
    mov ax, word [es:si+012h]                 ; 26 8b 44 12
    mov word [bp-00ch], ax                    ; 89 46 f4
    mov ax, word [es:si+00eh]                 ; 26 8b 44 0e
    mov word [bp-00eh], ax                    ; 89 46 f2
    mov ax, word [es:si+010h]                 ; 26 8b 44 10
    mov word [bp-014h], ax                    ; 89 46 ec
    mov ax, word [bp-00ch]                    ; 8b 46 f4
    test ax, ax                               ; 85 c0
    jne short 01de6h                          ; 75 67
    mov dx, word [bp-01ch]                    ; 8b 56 e4
    add dx, word [bp-010h]                    ; 03 56 f0
    adc ax, word [bp-01ah]                    ; 13 46 e6
    cmp ax, 01000h                            ; 3d 00 10
    jnbe short 01d8fh                         ; 77 02
    jne short 01dbah                          ; 75 2b
    mov ax, word [bp-01ah]                    ; 8b 46 e6
    xor al, al                                ; 30 c0
    shr ax, 008h                              ; c1 e8 08
    mov word [bp-00ch], ax                    ; 89 46 f4
    mov ax, word [bp-010h]                    ; 8b 46 f0
    xor al, al                                ; 30 c0
    shr ax, 008h                              ; c1 e8 08
    lea dx, [bx+002h]                         ; 8d 57 02
    out DX, AL                                ; ee
    lea dx, [bx+003h]                         ; 8d 57 03
    mov al, byte [bp-00ch]                    ; 8a 46 f4
    out DX, AL                                ; ee
    lea dx, [bx+004h]                         ; 8d 57 04
    xor al, al                                ; 30 c0
    out DX, AL                                ; ee
    lea dx, [bx+005h]                         ; 8d 57 05
    out DX, AL                                ; ee
    mov byte [bp-019h], al                    ; 88 46 e7
    mov ax, word [bp-01ch]                    ; 8b 46 e4
    xor ah, ah                                ; 30 e4
    mov word [bp-00ch], ax                    ; 89 46 f4
    mov cx, strict word 00008h                ; b9 08 00
    shr word [bp-01ah], 1                     ; d1 6e e6
    rcr word [bp-01ch], 1                     ; d1 5e e4
    loop 01dc5h                               ; e2 f8
    mov ax, word [bp-01ch]                    ; 8b 46 e4
    mov word [bp-00eh], ax                    ; 89 46 f2
    mov ax, word [bp-01ah]                    ; 8b 46 e6
    mov word [bp-01ch], ax                    ; 89 46 e4
    mov word [bp-01ah], strict word 00000h    ; c7 46 e6 00 00
    and ax, strict word 0000fh                ; 25 0f 00
    or AL, strict byte 040h                   ; 0c 40
    mov word [bp-014h], ax                    ; 89 46 ec
    mov dx, word [bp-018h]                    ; 8b 56 e8
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 00ah                  ; b0 0a
    out DX, AL                                ; ee
    lea dx, [bx+001h]                         ; 8d 57 01
    xor al, al                                ; 30 c0
    out DX, AL                                ; ee
    lea dx, [bx+002h]                         ; 8d 57 02
    mov al, byte [bp-010h]                    ; 8a 46 f0
    out DX, AL                                ; ee
    lea dx, [bx+003h]                         ; 8d 57 03
    mov al, byte [bp-00ch]                    ; 8a 46 f4
    out DX, AL                                ; ee
    mov ax, word [bp-00eh]                    ; 8b 46 f2
    lea dx, [bx+004h]                         ; 8d 57 04
    out DX, AL                                ; ee
    shr ax, 008h                              ; c1 e8 08
    lea dx, [bx+005h]                         ; 8d 57 05
    out DX, AL                                ; ee
    cmp byte [bp-004h], 000h                  ; 80 7e fc 00
    je short 01e1ch                           ; 74 05
    mov ax, 000b0h                            ; b8 b0 00
    jmp short 01e1fh                          ; eb 03
    mov ax, 000a0h                            ; b8 a0 00
    movzx dx, byte [bp-014h]                  ; 0f b6 56 ec
    or ax, dx                                 ; 09 d0
    lea dx, [bx+006h]                         ; 8d 57 06
    out DX, AL                                ; ee
    lea dx, [bx+007h]                         ; 8d 57 07
    mov al, byte [bp-012h]                    ; 8a 46 ee
    out DX, AL                                ; ee
    mov ax, word [bp-012h]                    ; 8b 46 ee
    cmp ax, 000c4h                            ; 3d c4 00
    je short 01e3dh                           ; 74 05
    cmp ax, strict word 00029h                ; 3d 29 00
    jne short 01e4ah                          ; 75 0d
    mov ax, word [bp-010h]                    ; 8b 46 f0
    mov word [bp-016h], ax                    ; 89 46 ea
    mov word [bp-010h], strict word 00001h    ; c7 46 f0 01 00
    jmp short 01e4fh                          ; eb 05
    mov word [bp-016h], strict word 00001h    ; c7 46 ea 01 00
    lea dx, [bx+007h]                         ; 8d 57 07
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov dl, al                                ; 88 c2
    test AL, strict byte 080h                 ; a8 80
    jne short 01e4fh                          ; 75 f4
    test AL, strict byte 001h                 ; a8 01
    je short 01e6eh                           ; 74 0f
    mov dx, word [bp-018h]                    ; 8b 56 e8
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    mov dx, strict word 00002h                ; ba 02 00
    jmp near 01f07h                           ; e9 99 00
    test dl, 008h                             ; f6 c2 08
    jne short 01e82h                          ; 75 0f
    mov dx, word [bp-018h]                    ; 8b 56 e8
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    mov dx, strict word 00003h                ; ba 03 00
    jmp near 01f07h                           ; e9 85 00
    sti                                       ; fb
    cmp di, 0f800h                            ; 81 ff 00 f8
    jc short 01e96h                           ; 72 0d
    sub di, 00800h                            ; 81 ef 00 08
    mov ax, word [bp-00ah]                    ; 8b 46 f6
    add ax, 00080h                            ; 05 80 00
    mov word [bp-00ah], ax                    ; 89 46 f6
    cmp byte [bp-002h], 001h                  ; 80 7e fe 01
    jne short 01ea9h                          ; 75 0d
    mov dx, bx                                ; 89 da
    mov cx, word [bp-008h]                    ; 8b 4e f8
    mov es, [bp-00ah]                         ; 8e 46 f6
    db  0f3h, 066h, 06dh
    ; rep insd                                  ; f3 66 6d
    jmp short 01eb3h                          ; eb 0a
    mov dx, bx                                ; 89 da
    mov cx, word [bp-008h]                    ; 8b 4e f8
    mov es, [bp-00ah]                         ; 8e 46 f6
    rep insw                                  ; f3 6d
    mov ax, word [bp-016h]                    ; 8b 46 ea
    mov es, [bp-006h]                         ; 8e 46 fa
    add word [es:si+014h], ax                 ; 26 01 44 14
    dec word [bp-010h]                        ; ff 4e f0
    lea dx, [bx+007h]                         ; 8d 57 07
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov dl, al                                ; 88 c2
    test AL, strict byte 080h                 ; a8 80
    jne short 01ec0h                          ; 75 f4
    cmp word [bp-010h], strict byte 00000h    ; 83 7e f0 00
    jne short 01ee6h                          ; 75 14
    and AL, strict byte 0c9h                  ; 24 c9
    cmp AL, strict byte 040h                  ; 3c 40
    je short 01efch                           ; 74 24
    mov dx, word [bp-018h]                    ; 8b 56 e8
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    mov dx, strict word 00004h                ; ba 04 00
    jmp short 01f07h                          ; eb 21
    mov al, dl                                ; 88 d0
    and AL, strict byte 0c9h                  ; 24 c9
    cmp AL, strict byte 048h                  ; 3c 48
    je short 01e83h                           ; 74 95
    mov dx, word [bp-018h]                    ; 8b 56 e8
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    mov dx, strict word 00005h                ; ba 05 00
    jmp short 01f07h                          ; eb 0b
    mov dx, word [bp-018h]                    ; 8b 56 e8
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    xor dx, dx                                ; 31 d2
    mov ax, dx                                ; 89 d0
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn                                      ; c3
    inc di                                    ; 47
    and word [bx+021h], di                    ; 21 7f 21
    and word [bx+di], strict byte 0ff87h      ; 83 21 87
    and word [bp+di-070dfh], cx               ; 21 8b 21 8f
    and word [bp+di-068dfh], dx               ; 21 93 21 97
    db  021h
_ata_detect:                                 ; 0xf1f1d LB 0x621
    push si                                   ; 56
    push di                                   ; 57
    enter 0025ah, 000h                        ; c8 5a 02 00
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 f0 f6
    mov word [bp-022h], ax                    ; 89 46 de
    mov bx, 00122h                            ; bb 22 01
    mov es, ax                                ; 8e c0
    mov si, bx                                ; 89 de
    mov word [bp-026h], ax                    ; 89 46 da
    mov byte [es:bx+001c0h], 000h             ; 26 c6 87 c0 01 00
    db  066h, 026h, 0c7h, 087h, 0c2h, 001h, 0f0h, 001h, 0f0h, 003h
    ; mov dword [es:bx+001c2h], strict dword 003f001f0h ; 66 26 c7 87 c2 01 f0 01 f0 03
    mov byte [es:bx+001c1h], 00eh             ; 26 c6 87 c1 01 0e
    mov byte [es:bx+001c6h], 000h             ; 26 c6 87 c6 01 00
    db  066h, 026h, 0c7h, 087h, 0c8h, 001h, 070h, 001h, 070h, 003h
    ; mov dword [es:bx+001c8h], strict dword 003700170h ; 66 26 c7 87 c8 01 70 01 70 03
    mov byte [es:bx+001c7h], 00fh             ; 26 c6 87 c7 01 0f
    xor al, al                                ; 30 c0
    mov byte [bp-006h], al                    ; 88 46 fa
    mov byte [bp-014h], al                    ; 88 46 ec
    mov byte [bp-016h], al                    ; 88 46 ea
    jmp near 024c9h                           ; e9 56 05
    mov ax, 000a0h                            ; b8 a0 00
    lea dx, [bx+006h]                         ; 8d 57 06
    out DX, AL                                ; ee
    lea di, [bx+002h]                         ; 8d 7f 02
    mov AL, strict byte 055h                  ; b0 55
    mov dx, di                                ; 89 fa
    out DX, AL                                ; ee
    lea cx, [bx+003h]                         ; 8d 4f 03
    mov AL, strict byte 0aah                  ; b0 aa
    mov dx, cx                                ; 89 ca
    out DX, AL                                ; ee
    mov dx, di                                ; 89 fa
    out DX, AL                                ; ee
    mov AL, strict byte 055h                  ; b0 55
    mov dx, cx                                ; 89 ca
    out DX, AL                                ; ee
    mov dx, di                                ; 89 fa
    out DX, AL                                ; ee
    mov AL, strict byte 0aah                  ; b0 aa
    mov dx, cx                                ; 89 ca
    out DX, AL                                ; ee
    mov dx, di                                ; 89 fa
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov word [bp-028h], ax                    ; 89 46 d8
    mov dx, cx                                ; 89 ca
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    cmp byte [bp-028h], 055h                  ; 80 7e d8 55
    jne near 0207eh                           ; 0f 85 cf 00
    cmp AL, strict byte 0aah                  ; 3c aa
    jne near 0207eh                           ; 0f 85 c9 00
    movzx ax, byte [bp-016h]                  ; 0f b6 46 ea
    imul ax, ax, strict byte 00018h           ; 6b c0 18
    mov es, [bp-026h]                         ; 8e 46 da
    mov di, si                                ; 89 f7
    add di, ax                                ; 01 c7
    mov byte [es:di+01eh], 001h               ; 26 c6 45 1e 01
    movzx ax, byte [bp-016h]                  ; 0f b6 46 ea
    call 01be0h                               ; e8 11 fc
    cmp byte [bp-004h], 000h                  ; 80 7e fc 00
    je short 01fdah                           ; 74 05
    mov ax, 000b0h                            ; b8 b0 00
    jmp short 01fddh                          ; eb 03
    mov ax, 000a0h                            ; b8 a0 00
    lea dx, [bx+006h]                         ; 8d 57 06
    out DX, AL                                ; ee
    lea dx, [bx+002h]                         ; 8d 57 02
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov cx, ax                                ; 89 c1
    lea dx, [bx+003h]                         ; 8d 57 03
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    cmp cl, 001h                              ; 80 f9 01
    jne near 0207eh                           ; 0f 85 88 00
    cmp al, cl                                ; 38 c8
    jne near 0207eh                           ; 0f 85 82 00
    lea dx, [bx+004h]                         ; 8d 57 04
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov word [bp-028h], ax                    ; 89 46 d8
    mov al, byte [bp-028h]                    ; 8a 46 d8
    mov byte [bp-00ch], al                    ; 88 46 f4
    lea dx, [bx+005h]                         ; 8d 57 05
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov cx, ax                                ; 89 c1
    mov byte [bp-00ah], al                    ; 88 46 f6
    lea dx, [bx+007h]                         ; 8d 57 07
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    cmp byte [bp-028h], 014h                  ; 80 7e d8 14
    jne short 0203ah                          ; 75 18
    cmp cl, 0ebh                              ; 80 f9 eb
    jne short 0203ah                          ; 75 13
    movzx bx, byte [bp-016h]                  ; 0f b6 5e ea
    imul bx, bx, strict byte 00018h           ; 6b db 18
    mov es, [bp-026h]                         ; 8e 46 da
    add bx, si                                ; 01 f3
    mov byte [es:bx+01eh], 003h               ; 26 c6 47 1e 03
    jmp short 0207eh                          ; eb 44
    cmp byte [bp-00ch], 000h                  ; 80 7e f4 00
    jne short 0205fh                          ; 75 1f
    cmp byte [bp-00ah], 000h                  ; 80 7e f6 00
    jne short 0205fh                          ; 75 19
    test al, al                               ; 84 c0
    je short 0205fh                           ; 74 15
    movzx ax, byte [bp-016h]                  ; 0f b6 46 ea
    imul ax, ax, strict byte 00018h           ; 6b c0 18
    mov es, [bp-026h]                         ; 8e 46 da
    mov bx, si                                ; 89 f3
    add bx, ax                                ; 01 c3
    mov byte [es:bx+01eh], 002h               ; 26 c6 47 1e 02
    jmp short 0207eh                          ; eb 1f
    cmp byte [bp-00ch], 0ffh                  ; 80 7e f4 ff
    jne short 0207eh                          ; 75 19
    cmp byte [bp-00ah], 0ffh                  ; 80 7e f6 ff
    jne short 0207eh                          ; 75 13
    movzx ax, byte [bp-016h]                  ; 0f b6 46 ea
    imul ax, ax, strict byte 00018h           ; 6b c0 18
    mov es, [bp-026h]                         ; 8e 46 da
    mov bx, si                                ; 89 f3
    add bx, ax                                ; 01 c3
    mov byte [es:bx+01eh], 000h               ; 26 c6 47 1e 00
    mov dx, word [bp-02ch]                    ; 8b 56 d4
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    movzx ax, byte [bp-016h]                  ; 0f b6 46 ea
    imul ax, ax, strict byte 00018h           ; 6b c0 18
    mov es, [bp-026h]                         ; 8e 46 da
    mov bx, si                                ; 89 f3
    add bx, ax                                ; 01 c3
    mov al, byte [es:bx+01eh]                 ; 26 8a 47 1e
    mov byte [bp-002h], al                    ; 88 46 fe
    cmp AL, strict byte 002h                  ; 3c 02
    jne near 0229dh                           ; 0f 85 fb 01
    mov byte [es:bx+01fh], 0ffh               ; 26 c6 47 1f ff
    mov byte [es:bx+022h], 000h               ; 26 c6 47 22 00
    lea dx, [bp-0025ah]                       ; 8d 96 a6 fd
    mov word [es:si+004h], dx                 ; 26 89 54 04
    mov [es:si+006h], ss                      ; 26 8c 54 06
    mov al, byte [bp-016h]                    ; 8a 46 ea
    mov byte [es:si+008h], al                 ; 26 88 44 08
    mov cx, strict word 00001h                ; b9 01 00
    mov bx, 000ech                            ; bb ec 00
    mov ax, si                                ; 89 f0
    mov dx, es                                ; 8c c2
    call 01cbah                               ; e8 ee fb
    test ax, ax                               ; 85 c0
    je short 020dbh                           ; 74 0b
    push 00084h                               ; 68 84 00
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 11 f8
    add sp, strict byte 00004h                ; 83 c4 04
    test byte [bp-0025ah], 080h               ; f6 86 a6 fd 80
    db  00fh, 095h, 0c0h
    ; setne al                                  ; 0f 95 c0
    xor ah, ah                                ; 30 e4
    mov byte [bp-008h], al                    ; 88 46 f8
    cmp byte [bp-001fah], 000h                ; 80 be 06 fe 00
    db  00fh, 095h, 0c0h
    ; setne al                                  ; 0f 95 c0
    xor ah, ah                                ; 30 e4
    mov byte [bp-00eh], al                    ; 88 46 f2
    mov word [bp-024h], 00200h                ; c7 46 dc 00 02
    mov ax, word [bp-00258h]                  ; 8b 86 a8 fd
    mov word [bp-01eh], ax                    ; 89 46 e2
    mov ax, word [bp-00254h]                  ; 8b 86 ac fd
    mov word [bp-01ch], ax                    ; 89 46 e4
    mov ax, word [bp-0024eh]                  ; 8b 86 b2 fd
    mov word [bp-018h], ax                    ; 89 46 e8
    mov ax, word [bp-001e2h]                  ; 8b 86 1e fe
    mov word [bp-020h], ax                    ; 89 46 e0
    mov ax, word [bp-001e0h]                  ; 8b 86 20 fe
    mov word [bp-02ah], ax                    ; 89 46 d6
    cmp ax, 00fffh                            ; 3d ff 0f
    jne short 02136h                          ; 75 14
    cmp word [bp-020h], strict byte 0ffffh    ; 83 7e e0 ff
    jne short 02136h                          ; 75 0e
    mov ax, word [bp-00192h]                  ; 8b 86 6e fe
    mov word [bp-020h], ax                    ; 89 46 e0
    mov ax, word [bp-00190h]                  ; 8b 86 70 fe
    mov word [bp-02ah], ax                    ; 89 46 d6
    mov al, byte [bp-016h]                    ; 8a 46 ea
    cmp AL, strict byte 007h                  ; 3c 07
    jnbe short 0219bh                         ; 77 5e
    movzx bx, al                              ; 0f b6 d8
    add bx, bx                                ; 01 db
    jmp word [cs:bx+01f0dh]                   ; 2e ff a7 0d 1f
    mov BL, strict byte 01eh                  ; b3 1e
    mov al, bl                                ; 88 d8
    db  0feh, 0c0h
    ; inc al                                    ; fe c0
    xor ah, ah                                ; 30 e4
    call 0165ch                               ; e8 0a f5
    xor ah, ah                                ; 30 e4
    mov di, ax                                ; 89 c7
    sal di, 008h                              ; c1 e7 08
    movzx ax, bl                              ; 0f b6 c3
    call 0165ch                               ; e8 fd f4
    xor ah, ah                                ; 30 e4
    add di, ax                                ; 01 c7
    mov al, bl                                ; 88 d8
    add AL, strict byte 002h                  ; 04 02
    xor ah, ah                                ; 30 e4
    call 0165ch                               ; e8 f0 f4
    movzx dx, al                              ; 0f b6 d0
    mov al, bl                                ; 88 d8
    add AL, strict byte 007h                  ; 04 07
    xor ah, ah                                ; 30 e4
    call 0165ch                               ; e8 e4 f4
    xor ah, ah                                ; 30 e4
    mov word [bp-01ah], ax                    ; 89 46 e6
    jmp short 021a2h                          ; eb 23
    mov BL, strict byte 026h                  ; b3 26
    jmp short 02149h                          ; eb c6
    mov BL, strict byte 067h                  ; b3 67
    jmp short 02149h                          ; eb c2
    mov BL, strict byte 070h                  ; b3 70
    jmp short 02149h                          ; eb be
    mov BL, strict byte 040h                  ; b3 40
    jmp short 02149h                          ; eb ba
    mov BL, strict byte 048h                  ; b3 48
    jmp short 02149h                          ; eb b6
    mov BL, strict byte 050h                  ; b3 50
    jmp short 02149h                          ; eb b2
    mov BL, strict byte 058h                  ; b3 58
    jmp short 02149h                          ; eb ae
    xor di, di                                ; 31 ff
    xor dx, dx                                ; 31 d2
    mov word [bp-01ah], di                    ; 89 7e e6
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 01 f7
    push word [bp-01ah]                       ; ff 76 e6
    push dx                                   ; 52
    push di                                   ; 57
    push word [bp-018h]                       ; ff 76 e8
    push dword [bp-01eh]                      ; 66 ff 76 e2
    movzx ax, byte [bp-004h]                  ; 0f b6 46 fc
    push ax                                   ; 50
    movzx ax, byte [bp-012h]                  ; 0f b6 46 ee
    push ax                                   ; 50
    push 000adh                               ; 68 ad 00
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 1e f7
    add sp, strict byte 00014h                ; 83 c4 14
    movzx ax, byte [bp-016h]                  ; 0f b6 46 ea
    imul ax, ax, strict byte 00018h           ; 6b c0 18
    mov es, [bp-026h]                         ; 8e 46 da
    mov bx, si                                ; 89 f3
    add bx, ax                                ; 01 c3
    mov byte [es:bx+01fh], 0ffh               ; 26 c6 47 1f ff
    mov al, byte [bp-008h]                    ; 8a 46 f8
    mov byte [es:bx+020h], al                 ; 26 88 47 20
    mov al, byte [bp-00eh]                    ; 8a 46 f2
    mov byte [es:bx+022h], al                 ; 26 88 47 22
    mov ax, word [bp-024h]                    ; 8b 46 dc
    mov word [es:bx+024h], ax                 ; 26 89 47 24
    mov ax, word [bp-01ch]                    ; 8b 46 e4
    mov word [es:bx+02ch], ax                 ; 26 89 47 2c
    mov ax, word [bp-01eh]                    ; 8b 46 e2
    mov word [es:bx+02eh], ax                 ; 26 89 47 2e
    mov ax, word [bp-018h]                    ; 8b 46 e8
    mov word [es:bx+030h], ax                 ; 26 89 47 30
    mov ax, word [bp-020h]                    ; 8b 46 e0
    mov word [es:bx+032h], ax                 ; 26 89 47 32
    mov ax, word [bp-02ah]                    ; 8b 46 d6
    mov word [es:bx+034h], ax                 ; 26 89 47 34
    mov word [es:bx+026h], dx                 ; 26 89 57 26
    mov word [es:bx+028h], di                 ; 26 89 7f 28
    mov ax, word [bp-01ah]                    ; 8b 46 e6
    mov word [es:bx+02ah], ax                 ; 26 89 47 2a
    mov al, byte [bp-016h]                    ; 8a 46 ea
    cmp AL, strict byte 002h                  ; 3c 02
    jnc short 02289h                          ; 73 5a
    test al, al                               ; 84 c0
    jne short 02238h                          ; 75 05
    mov bx, strict word 0003dh                ; bb 3d 00
    jmp short 0223bh                          ; eb 03
    mov bx, strict word 0004dh                ; bb 4d 00
    mov cx, word [bp-022h]                    ; 8b 4e de
    mov es, cx                                ; 8e c1
    mov word [es:bx], di                      ; 26 89 3f
    mov byte [es:bx+002h], dl                 ; 26 88 57 02
    mov byte [es:bx+003h], 0a0h               ; 26 c6 47 03 a0
    mov al, byte [bp-018h]                    ; 8a 46 e8
    mov byte [es:bx+004h], al                 ; 26 88 47 04
    mov ax, word [bp-01eh]                    ; 8b 46 e2
    mov word [es:bx+009h], ax                 ; 26 89 47 09
    mov al, byte [bp-01ch]                    ; 8a 46 e4
    mov byte [es:bx+00bh], al                 ; 26 88 47 0b
    mov al, byte [bp-018h]                    ; 8a 46 e8
    mov byte [es:bx+00eh], al                 ; 26 88 47 0e
    xor al, al                                ; 30 c0
    xor ah, ah                                ; 30 e4
    jmp short 02273h                          ; eb 05
    cmp ah, 00fh                              ; 80 fc 0f
    jnc short 02281h                          ; 73 0e
    movzx di, ah                              ; 0f b6 fc
    mov es, cx                                ; 8e c1
    add di, bx                                ; 01 df
    add al, byte [es:di]                      ; 26 02 05
    db  0feh, 0c4h
    ; inc ah                                    ; fe c4
    jmp short 0226eh                          ; eb ed
    neg al                                    ; f6 d8
    mov es, cx                                ; 8e c1
    mov byte [es:bx+00fh], al                 ; 26 88 47 0f
    movzx bx, byte [bp-014h]                  ; 0f b6 5e ec
    mov es, [bp-026h]                         ; 8e 46 da
    add bx, si                                ; 01 f3
    mov al, byte [bp-016h]                    ; 8a 46 ea
    mov byte [es:bx+0019fh], al               ; 26 88 87 9f 01
    inc byte [bp-014h]                        ; fe 46 ec
    cmp byte [bp-002h], 003h                  ; 80 7e fe 03
    jne near 02338h                           ; 0f 85 93 00
    movzx ax, byte [bp-016h]                  ; 0f b6 46 ea
    imul ax, ax, strict byte 00018h           ; 6b c0 18
    mov es, [bp-026h]                         ; 8e 46 da
    mov bx, si                                ; 89 f3
    add bx, ax                                ; 01 c3
    mov byte [es:bx+01fh], 005h               ; 26 c6 47 1f 05
    mov byte [es:bx+022h], 000h               ; 26 c6 47 22 00
    lea dx, [bp-0025ah]                       ; 8d 96 a6 fd
    mov word [es:si+004h], dx                 ; 26 89 54 04
    mov [es:si+006h], ss                      ; 26 8c 54 06
    mov al, byte [bp-016h]                    ; 8a 46 ea
    mov byte [es:si+008h], al                 ; 26 88 44 08
    mov cx, strict word 00001h                ; b9 01 00
    mov bx, 000a1h                            ; bb a1 00
    mov ax, si                                ; 89 f0
    mov dx, es                                ; 8c c2
    call 01cbah                               ; e8 dd f9
    test ax, ax                               ; 85 c0
    je short 022ech                           ; 74 0b
    push 000d4h                               ; 68 d4 00
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 00 f6
    add sp, strict byte 00004h                ; 83 c4 04
    mov dl, byte [bp-00259h]                  ; 8a 96 a7 fd
    and dl, 01fh                              ; 80 e2 1f
    test byte [bp-0025ah], 080h               ; f6 86 a6 fd 80
    db  00fh, 095h, 0c0h
    ; setne al                                  ; 0f 95 c0
    xor ah, ah                                ; 30 e4
    mov cx, ax                                ; 89 c1
    cmp byte [bp-001fah], 000h                ; 80 be 06 fe 00
    db  00fh, 095h, 0c0h
    ; setne al                                  ; 0f 95 c0
    xor ah, ah                                ; 30 e4
    movzx bx, byte [bp-016h]                  ; 0f b6 5e ea
    imul bx, bx, strict byte 00018h           ; 6b db 18
    mov es, [bp-026h]                         ; 8e 46 da
    add bx, si                                ; 01 f3
    mov byte [es:bx+01fh], dl                 ; 26 88 57 1f
    mov byte [es:bx+020h], cl                 ; 26 88 4f 20
    mov byte [es:bx+022h], al                 ; 26 88 47 22
    mov word [es:bx+024h], 00800h             ; 26 c7 47 24 00 08
    movzx bx, byte [bp-006h]                  ; 0f b6 5e fa
    add bx, si                                ; 01 f3
    mov al, byte [bp-016h]                    ; 8a 46 ea
    mov byte [es:bx+001b0h], al               ; 26 88 87 b0 01
    inc byte [bp-006h]                        ; fe 46 fa
    mov al, byte [bp-002h]                    ; 8a 46 fe
    cmp AL, strict byte 003h                  ; 3c 03
    je short 0236ch                           ; 74 2d
    cmp AL, strict byte 002h                  ; 3c 02
    jne near 023d1h                           ; 0f 85 8c 00
    movzx ax, byte [bp-016h]                  ; 0f b6 46 ea
    imul ax, ax, strict byte 00018h           ; 6b c0 18
    mov es, [bp-026h]                         ; 8e 46 da
    mov bx, si                                ; 89 f3
    add bx, ax                                ; 01 c3
    mov ax, word [es:bx+032h]                 ; 26 8b 47 32
    mov word [bp-030h], ax                    ; 89 46 d0
    mov ax, word [es:bx+034h]                 ; 26 8b 47 34
    mov word [bp-02eh], ax                    ; 89 46 d2
    mov cx, strict word 0000bh                ; b9 0b 00
    shr word [bp-02eh], 1                     ; d1 6e d2
    rcr word [bp-030h], 1                     ; d1 5e d0
    loop 02364h                               ; e2 f8
    movzx dx, byte [bp-001b9h]                ; 0f b6 96 47 fe
    sal dx, 008h                              ; c1 e2 08
    movzx ax, byte [bp-001bah]                ; 0f b6 86 46 fe
    or dx, ax                                 ; 09 c2
    mov byte [bp-010h], 00fh                  ; c6 46 f0 0f
    jmp short 0238ah                          ; eb 09
    dec byte [bp-010h]                        ; fe 4e f0
    cmp byte [bp-010h], 000h                  ; 80 7e f0 00
    jbe short 02397h                          ; 76 0d
    movzx cx, byte [bp-010h]                  ; 0f b6 4e f0
    mov ax, strict word 00001h                ; b8 01 00
    sal ax, CL                                ; d3 e0
    test dx, ax                               ; 85 c2
    je short 02381h                           ; 74 ea
    xor bx, bx                                ; 31 db
    jmp short 023a0h                          ; eb 05
    cmp bx, strict byte 00014h                ; 83 fb 14
    jnl short 023b5h                          ; 7d 15
    mov di, bx                                ; 89 df
    add di, bx                                ; 01 df
    mov al, byte [bp+di-00223h]               ; 8a 83 dd fd
    mov byte [bp+di-05ah], al                 ; 88 43 a6
    mov al, byte [bp+di-00224h]               ; 8a 83 dc fd
    mov byte [bp+di-059h], al                 ; 88 43 a7
    inc bx                                    ; 43
    jmp short 0239bh                          ; eb e6
    mov byte [bp-032h], 000h                  ; c6 46 ce 00
    mov bx, strict word 00027h                ; bb 27 00
    jmp short 023c3h                          ; eb 05
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jle short 023d1h                          ; 7e 0e
    mov di, bx                                ; 89 df
    cmp byte [bp+di-05ah], 020h               ; 80 7b a6 20
    jne short 023d1h                          ; 75 06
    mov byte [bp+di-05ah], 000h               ; c6 43 a6 00
    jmp short 023beh                          ; eb ed
    mov al, byte [bp-002h]                    ; 8a 46 fe
    cmp AL, strict byte 003h                  ; 3c 03
    je short 02436h                           ; 74 5e
    cmp AL, strict byte 002h                  ; 3c 02
    je short 023e5h                           ; 74 09
    cmp AL, strict byte 001h                  ; 3c 01
    je near 024a1h                            ; 0f 84 bf 00
    jmp near 024c0h                           ; e9 db 00
    cmp byte [bp-004h], 000h                  ; 80 7e fc 00
    je short 023f0h                           ; 74 05
    mov ax, 000ffh                            ; b8 ff 00
    jmp short 023f3h                          ; eb 03
    mov ax, 00106h                            ; b8 06 01
    push ax                                   ; 50
    movzx ax, byte [bp-012h]                  ; 0f b6 46 ee
    push ax                                   ; 50
    push 0010dh                               ; 68 0d 01
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 e8 f4
    add sp, strict byte 00008h                ; 83 c4 08
    xor bx, bx                                ; 31 db
    mov di, bx                                ; 89 df
    movzx ax, byte [bp+di-05ah]               ; 0f b6 43 a6
    inc bx                                    ; 43
    test ax, ax                               ; 85 c0
    je short 0241fh                           ; 74 0e
    push ax                                   ; 50
    push 00118h                               ; 68 18 01
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 cf f4
    add sp, strict byte 00006h                ; 83 c4 06
    jmp short 02406h                          ; eb e7
    push dword [bp-030h]                      ; 66 ff 76 d0
    movzx ax, byte [bp-010h]                  ; 0f b6 46 f0
    push ax                                   ; 50
    push 0011bh                               ; 68 1b 01
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 b9 f4
    add sp, strict byte 0000ah                ; 83 c4 0a
    jmp near 024c0h                           ; e9 8a 00
    cmp byte [bp-004h], 000h                  ; 80 7e fc 00
    je short 02441h                           ; 74 05
    mov ax, 000ffh                            ; b8 ff 00
    jmp short 02444h                          ; eb 03
    mov ax, 00106h                            ; b8 06 01
    push ax                                   ; 50
    movzx ax, byte [bp-012h]                  ; 0f b6 46 ee
    push ax                                   ; 50
    push 0010dh                               ; 68 0d 01
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 97 f4
    add sp, strict byte 00008h                ; 83 c4 08
    xor bx, bx                                ; 31 db
    mov di, bx                                ; 89 df
    movzx ax, byte [bp+di-05ah]               ; 0f b6 43 a6
    inc bx                                    ; 43
    test ax, ax                               ; 85 c0
    je short 02470h                           ; 74 0e
    push ax                                   ; 50
    push 00118h                               ; 68 18 01
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 7e f4
    add sp, strict byte 00006h                ; 83 c4 06
    jmp short 02457h                          ; eb e7
    movzx ax, byte [bp-016h]                  ; 0f b6 46 ea
    imul ax, ax, strict byte 00018h           ; 6b c0 18
    mov es, [bp-026h]                         ; 8e 46 da
    mov bx, si                                ; 89 f3
    add bx, ax                                ; 01 c3
    cmp byte [es:bx+01fh], 005h               ; 26 80 7f 1f 05
    jne short 0248fh                          ; 75 0a
    movzx ax, byte [bp-010h]                  ; 0f b6 46 f0
    push ax                                   ; 50
    push 0013bh                               ; 68 3b 01
    jmp short 02497h                          ; eb 08
    movzx ax, byte [bp-010h]                  ; 0f b6 46 f0
    push ax                                   ; 50
    push 00155h                               ; 68 55 01
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 4d f4
    add sp, strict byte 00006h                ; 83 c4 06
    jmp short 024c0h                          ; eb 1f
    cmp byte [bp-004h], 000h                  ; 80 7e fc 00
    je short 024ach                           ; 74 05
    mov ax, 000ffh                            ; b8 ff 00
    jmp short 024afh                          ; eb 03
    mov ax, 00106h                            ; b8 06 01
    push ax                                   ; 50
    movzx ax, byte [bp-012h]                  ; 0f b6 46 ee
    push ax                                   ; 50
    push 00167h                               ; 68 67 01
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 2c f4
    add sp, strict byte 00008h                ; 83 c4 08
    inc byte [bp-016h]                        ; fe 46 ea
    cmp byte [bp-016h], 008h                  ; 80 7e ea 08
    jnc short 0251ah                          ; 73 51
    movzx bx, byte [bp-016h]                  ; 0f b6 5e ea
    mov ax, bx                                ; 89 d8
    cwd                                       ; 99
    db  02bh, 0c2h
    ; sub ax, dx                                ; 2b c2
    sar ax, 1                                 ; d1 f8
    mov cx, ax                                ; 89 c1
    mov byte [bp-012h], al                    ; 88 46 ee
    mov ax, bx                                ; 89 d8
    cwd                                       ; 99
    mov bx, strict word 00002h                ; bb 02 00
    idiv bx                                   ; f7 fb
    mov word [bp-028h], dx                    ; 89 56 d8
    mov al, byte [bp-028h]                    ; 8a 46 d8
    mov byte [bp-004h], al                    ; 88 46 fc
    movzx ax, cl                              ; 0f b6 c1
    imul ax, ax, strict byte 00006h           ; 6b c0 06
    mov es, [bp-026h]                         ; 8e 46 da
    mov di, si                                ; 89 f7
    add di, ax                                ; 01 c7
    mov bx, word [es:di+001c2h]               ; 26 8b 9d c2 01
    mov ax, word [es:di+001c4h]               ; 26 8b 85 c4 01
    mov word [bp-02ch], ax                    ; 89 46 d4
    mov dx, ax                                ; 89 c2
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 00ah                  ; b0 0a
    out DX, AL                                ; ee
    cmp byte [bp-028h], 000h                  ; 80 7e d8 00
    je near 01f73h                            ; 0f 84 5f fa
    mov ax, 000b0h                            ; b8 b0 00
    jmp near 01f76h                           ; e9 5c fa
    mov al, byte [bp-014h]                    ; 8a 46 ec
    mov es, [bp-026h]                         ; 8e 46 da
    mov byte [es:si+0019eh], al               ; 26 88 84 9e 01
    mov al, byte [bp-006h]                    ; 8a 46 fa
    mov byte [es:si+001afh], al               ; 26 88 84 af 01
    movzx bx, byte [bp-014h]                  ; 0f b6 5e ec
    mov dx, strict word 00075h                ; ba 75 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 d4 f0
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn                                      ; c3
ata_cmd_data_out_:                           ; 0xf253e LB 0x215
    push si                                   ; 56
    push di                                   ; 57
    enter 0001ah, 000h                        ; c8 1a 00 00
    mov di, ax                                ; 89 c7
    mov word [bp-006h], dx                    ; 89 56 fa
    mov word [bp-016h], bx                    ; 89 5e ea
    mov word [bp-00ah], cx                    ; 89 4e f6
    mov es, dx                                ; 8e c2
    movzx ax, byte [es:di+008h]               ; 26 0f b6 45 08
    mov dx, ax                                ; 89 c2
    shr dx, 1                                 ; d1 ea
    mov dh, al                                ; 88 c6
    and dh, 001h                              ; 80 e6 01
    mov byte [bp-002h], dh                    ; 88 76 fe
    movzx si, dl                              ; 0f b6 f2
    imul si, si, strict byte 00006h           ; 6b f6 06
    add si, di                                ; 01 fe
    mov bx, word [es:si+001c2h]               ; 26 8b 9c c2 01
    mov dx, word [es:si+001c4h]               ; 26 8b 94 c4 01
    mov word [bp-008h], dx                    ; 89 56 f8
    imul ax, ax, strict byte 00018h           ; 6b c0 18
    mov si, di                                ; 89 fe
    add si, ax                                ; 01 c6
    mov al, byte [es:si+022h]                 ; 26 8a 44 22
    mov byte [bp-004h], al                    ; 88 46 fc
    cmp AL, strict byte 001h                  ; 3c 01
    jne short 02590h                          ; 75 07
    mov word [bp-00eh], 00080h                ; c7 46 f2 80 00
    jmp short 02595h                          ; eb 05
    mov word [bp-00eh], 00100h                ; c7 46 f2 00 01
    lea dx, [bx+007h]                         ; 8d 57 07
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 080h                 ; a8 80
    je short 025aeh                           ; 74 0f
    mov dx, word [bp-008h]                    ; 8b 56 f8
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    mov dx, strict word 00001h                ; ba 01 00
    jmp near 0274dh                           ; e9 9f 01
    mov es, [bp-006h]                         ; 8e 46 fa
    mov ax, word [es:di]                      ; 26 8b 05
    mov word [bp-01ah], ax                    ; 89 46 e6
    mov ax, word [es:di+002h]                 ; 26 8b 45 02
    mov word [bp-018h], ax                    ; 89 46 e8
    mov si, word [es:di+004h]                 ; 26 8b 75 04
    mov ax, word [es:di+006h]                 ; 26 8b 45 06
    mov word [bp-014h], ax                    ; 89 46 ec
    mov ax, word [es:di+012h]                 ; 26 8b 45 12
    mov word [bp-00ch], ax                    ; 89 46 f4
    mov ax, word [es:di+00eh]                 ; 26 8b 45 0e
    mov word [bp-012h], ax                    ; 89 46 ee
    mov ax, word [es:di+010h]                 ; 26 8b 45 10
    mov word [bp-010h], ax                    ; 89 46 f0
    mov ax, word [bp-00ch]                    ; 8b 46 f4
    test ax, ax                               ; 85 c0
    jne short 0264ch                          ; 75 67
    mov dx, word [bp-01ah]                    ; 8b 56 e6
    add dx, word [bp-00ah]                    ; 03 56 f6
    adc ax, word [bp-018h]                    ; 13 46 e8
    cmp ax, 01000h                            ; 3d 00 10
    jnbe short 025f5h                         ; 77 02
    jne short 02620h                          ; 75 2b
    mov ax, word [bp-018h]                    ; 8b 46 e8
    xor al, al                                ; 30 c0
    shr ax, 008h                              ; c1 e8 08
    mov word [bp-00ch], ax                    ; 89 46 f4
    mov ax, word [bp-00ah]                    ; 8b 46 f6
    xor al, al                                ; 30 c0
    shr ax, 008h                              ; c1 e8 08
    lea dx, [bx+002h]                         ; 8d 57 02
    out DX, AL                                ; ee
    lea dx, [bx+003h]                         ; 8d 57 03
    mov al, byte [bp-00ch]                    ; 8a 46 f4
    out DX, AL                                ; ee
    lea dx, [bx+004h]                         ; 8d 57 04
    xor al, al                                ; 30 c0
    out DX, AL                                ; ee
    lea dx, [bx+005h]                         ; 8d 57 05
    out DX, AL                                ; ee
    mov byte [bp-017h], al                    ; 88 46 e9
    mov ax, word [bp-01ah]                    ; 8b 46 e6
    xor ah, ah                                ; 30 e4
    mov word [bp-00ch], ax                    ; 89 46 f4
    mov cx, strict word 00008h                ; b9 08 00
    shr word [bp-018h], 1                     ; d1 6e e8
    rcr word [bp-01ah], 1                     ; d1 5e e6
    loop 0262bh                               ; e2 f8
    mov ax, word [bp-01ah]                    ; 8b 46 e6
    mov word [bp-012h], ax                    ; 89 46 ee
    mov ax, word [bp-018h]                    ; 8b 46 e8
    mov word [bp-01ah], ax                    ; 89 46 e6
    mov word [bp-018h], strict word 00000h    ; c7 46 e8 00 00
    and ax, strict word 0000fh                ; 25 0f 00
    or AL, strict byte 040h                   ; 0c 40
    mov word [bp-010h], ax                    ; 89 46 f0
    mov dx, word [bp-008h]                    ; 8b 56 f8
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 00ah                  ; b0 0a
    out DX, AL                                ; ee
    lea dx, [bx+001h]                         ; 8d 57 01
    xor al, al                                ; 30 c0
    out DX, AL                                ; ee
    lea dx, [bx+002h]                         ; 8d 57 02
    mov al, byte [bp-00ah]                    ; 8a 46 f6
    out DX, AL                                ; ee
    lea dx, [bx+003h]                         ; 8d 57 03
    mov al, byte [bp-00ch]                    ; 8a 46 f4
    out DX, AL                                ; ee
    mov ax, word [bp-012h]                    ; 8b 46 ee
    lea dx, [bx+004h]                         ; 8d 57 04
    out DX, AL                                ; ee
    shr ax, 008h                              ; c1 e8 08
    lea dx, [bx+005h]                         ; 8d 57 05
    out DX, AL                                ; ee
    cmp byte [bp-002h], 000h                  ; 80 7e fe 00
    je short 02682h                           ; 74 05
    mov ax, 000b0h                            ; b8 b0 00
    jmp short 02685h                          ; eb 03
    mov ax, 000a0h                            ; b8 a0 00
    movzx dx, byte [bp-010h]                  ; 0f b6 56 f0
    or ax, dx                                 ; 09 d0
    lea dx, [bx+006h]                         ; 8d 57 06
    out DX, AL                                ; ee
    lea dx, [bx+007h]                         ; 8d 57 07
    mov al, byte [bp-016h]                    ; 8a 46 ea
    out DX, AL                                ; ee
    lea dx, [bx+007h]                         ; 8d 57 07
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov dl, al                                ; 88 c2
    test AL, strict byte 080h                 ; a8 80
    jne short 02696h                          ; 75 f4
    test AL, strict byte 001h                 ; a8 01
    je short 026b5h                           ; 74 0f
    mov dx, word [bp-008h]                    ; 8b 56 f8
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    mov dx, strict word 00002h                ; ba 02 00
    jmp near 0274dh                           ; e9 98 00
    test dl, 008h                             ; f6 c2 08
    jne short 026c9h                          ; 75 0f
    mov dx, word [bp-008h]                    ; 8b 56 f8
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    mov dx, strict word 00003h                ; ba 03 00
    jmp near 0274dh                           ; e9 84 00
    sti                                       ; fb
    cmp si, 0f800h                            ; 81 fe 00 f8
    jc short 026ddh                           ; 72 0d
    sub si, 00800h                            ; 81 ee 00 08
    mov ax, word [bp-014h]                    ; 8b 46 ec
    add ax, 00080h                            ; 05 80 00
    mov word [bp-014h], ax                    ; 89 46 ec
    cmp byte [bp-004h], 001h                  ; 80 7e fc 01
    jne short 026f1h                          ; 75 0e
    mov dx, bx                                ; 89 da
    mov cx, word [bp-00eh]                    ; 8b 4e f2
    mov es, [bp-014h]                         ; 8e 46 ec
    db  0f3h, 066h, 026h, 06fh
    ; rep es outsd                              ; f3 66 26 6f
    jmp short 026fch                          ; eb 0b
    mov dx, bx                                ; 89 da
    mov cx, word [bp-00eh]                    ; 8b 4e f2
    mov es, [bp-014h]                         ; 8e 46 ec
    db  0f3h, 026h, 06fh
    ; rep es outsw                              ; f3 26 6f
    mov es, [bp-006h]                         ; 8e 46 fa
    inc word [es:di+014h]                     ; 26 ff 45 14
    dec word [bp-00ah]                        ; ff 4e f6
    lea dx, [bx+007h]                         ; 8d 57 07
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov dl, al                                ; 88 c2
    test AL, strict byte 080h                 ; a8 80
    jne short 02706h                          ; 75 f4
    cmp word [bp-00ah], strict byte 00000h    ; 83 7e f6 00
    jne short 0272ch                          ; 75 14
    and AL, strict byte 0e9h                  ; 24 e9
    cmp AL, strict byte 040h                  ; 3c 40
    je short 02742h                           ; 74 24
    mov dx, word [bp-008h]                    ; 8b 56 f8
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    mov dx, strict word 00006h                ; ba 06 00
    jmp short 0274dh                          ; eb 21
    mov al, dl                                ; 88 d0
    and AL, strict byte 0c9h                  ; 24 c9
    cmp AL, strict byte 048h                  ; 3c 48
    je short 026cah                           ; 74 96
    mov dx, word [bp-008h]                    ; 8b 56 f8
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    mov dx, strict word 00007h                ; ba 07 00
    jmp short 0274dh                          ; eb 0b
    mov dx, word [bp-008h]                    ; 8b 56 f8
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    xor dx, dx                                ; 31 d2
    mov ax, dx                                ; 89 d0
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn                                      ; c3
@ata_read_sectors:                           ; 0xf2753 LB 0x77
    push si                                   ; 56
    push di                                   ; 57
    enter 00002h, 000h                        ; c8 02 00 00
    mov si, word [bp+008h]                    ; 8b 76 08
    mov es, [bp+00ah]                         ; 8e 46 0a
    mov bl, byte [es:si+008h]                 ; 26 8a 5c 08
    mov cx, word [es:si+00ah]                 ; 26 8b 4c 0a
    mov dx, cx                                ; 89 ca
    sal dx, 009h                              ; c1 e2 09
    mov ax, word [es:si+012h]                 ; 26 8b 44 12
    test ax, ax                               ; 85 c0
    je short 02781h                           ; 74 0d
    movzx di, bl                              ; 0f b6 fb
    imul di, di, strict byte 00018h           ; 6b ff 18
    mov [bp-002h], es                         ; 8c 46 fe
    add di, si                                ; 01 f7
    jmp short 027adh                          ; eb 2c
    mov di, word [es:si]                      ; 26 8b 3c
    add di, cx                                ; 01 cf
    mov word [bp-002h], di                    ; 89 7e fe
    adc ax, word [es:si+002h]                 ; 26 13 44 02
    cmp ax, 01000h                            ; 3d 00 10
    jnbe short 02794h                         ; 77 02
    jne short 027a0h                          ; 75 0c
    mov bx, strict word 00024h                ; bb 24 00
    mov ax, si                                ; 89 f0
    mov dx, es                                ; 8c c2
    call 01cbah                               ; e8 1c f5
    jmp short 027c4h                          ; eb 24
    movzx ax, bl                              ; 0f b6 c3
    imul ax, ax, strict byte 00018h           ; 6b c0 18
    mov [bp-002h], es                         ; 8c 46 fe
    mov di, si                                ; 89 f7
    add di, ax                                ; 01 c7
    mov word [es:di+024h], dx                 ; 26 89 55 24
    mov bx, 000c4h                            ; bb c4 00
    mov ax, si                                ; 89 f0
    mov dx, es                                ; 8c c2
    call 01cbah                               ; e8 ff f4
    mov es, [bp-002h]                         ; 8e 46 fe
    mov word [es:di+024h], 00200h             ; 26 c7 45 24 00 02
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn 00004h                               ; c2 04 00
@ata_write_sectors:                          ; 0xf27ca LB 0x3a
    push si                                   ; 56
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    les si, [bp+006h]                         ; c4 76 06
    mov cx, word [es:si+00ah]                 ; 26 8b 4c 0a
    cmp word [es:si+012h], strict byte 00000h ; 26 83 7c 12 00
    je short 027e8h                           ; 74 0c
    mov bx, strict word 00030h                ; bb 30 00
    mov ax, si                                ; 89 f0
    mov dx, es                                ; 8c c2
    call 0253eh                               ; e8 58 fd
    jmp short 027ffh                          ; eb 17
    xor ax, ax                                ; 31 c0
    mov dx, word [es:si]                      ; 26 8b 14
    add dx, cx                                ; 01 ca
    adc ax, word [es:si+002h]                 ; 26 13 44 02
    cmp ax, 01000h                            ; 3d 00 10
    jnbe short 027fah                         ; 77 02
    jne short 027dch                          ; 75 e2
    mov bx, strict word 00034h                ; bb 34 00
    jmp short 027dfh                          ; eb e0
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn 00004h                               ; c2 04 00
ata_cmd_packet_:                             ; 0xf2804 LB 0x2e3
    push si                                   ; 56
    push di                                   ; 57
    enter 00014h, 000h                        ; c8 14 00 00
    push ax                                   ; 50
    mov byte [bp-004h], dl                    ; 88 56 fc
    mov di, bx                                ; 89 df
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 03 ee
    mov word [bp-00eh], 00122h                ; c7 46 f2 22 01
    mov word [bp-00ah], ax                    ; 89 46 f6
    mov ax, word [bp-016h]                    ; 8b 46 ea
    shr ax, 1                                 ; d1 e8
    mov ah, byte [bp-016h]                    ; 8a 66 ea
    and ah, 001h                              ; 80 e4 01
    mov byte [bp-002h], ah                    ; 88 66 fe
    cmp byte [bp+00eh], 002h                  ; 80 7e 0e 02
    jne short 02854h                          ; 75 1f
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 6e f0
    push 00181h                               ; 68 81 01
    push 00190h                               ; 68 90 01
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 9e f0
    add sp, strict byte 00006h                ; 83 c4 06
    mov dx, strict word 00001h                ; ba 01 00
    jmp near 02adfh                           ; e9 8b 02
    test byte [bp+008h], 001h                 ; f6 46 08 01
    jne short 0284eh                          ; 75 f4
    xor ah, ah                                ; 30 e4
    imul ax, ax, strict byte 00006h           ; 6b c0 06
    mov es, [bp-00ah]                         ; 8e 46 f6
    mov si, word [bp-00eh]                    ; 8b 76 f2
    add si, ax                                ; 01 c6
    mov bx, word [es:si+001c2h]               ; 26 8b 9c c2 01
    mov ax, word [es:si+001c4h]               ; 26 8b 84 c4 01
    mov word [bp-00ch], ax                    ; 89 46 f4
    imul si, word [bp-016h], strict byte 00018h ; 6b 76 ea 18
    add si, word [bp-00eh]                    ; 03 76 f2
    mov al, byte [es:si+022h]                 ; 26 8a 44 22
    mov byte [bp-006h], al                    ; 88 46 fa
    xor ax, ax                                ; 31 c0
    mov word [bp-014h], ax                    ; 89 46 ec
    mov word [bp-012h], ax                    ; 89 46 ee
    mov al, byte [bp-004h]                    ; 8a 46 fc
    cmp AL, strict byte 00ch                  ; 3c 0c
    jnc short 02897h                          ; 73 06
    mov byte [bp-004h], 00ch                  ; c6 46 fc 0c
    jmp short 0289dh                          ; eb 06
    jbe short 0289dh                          ; 76 04
    mov byte [bp-004h], 010h                  ; c6 46 fc 10
    shr byte [bp-004h], 1                     ; d0 6e fc
    mov es, [bp-00ah]                         ; 8e 46 f6
    mov si, word [bp-00eh]                    ; 8b 76 f2
    db  066h, 026h, 0c7h, 044h, 014h, 000h, 000h, 000h, 000h
    ; mov dword [es:si+014h], strict dword 000000000h ; 66 26 c7 44 14 00 00 00 00
    mov word [es:si+018h], strict word 00000h ; 26 c7 44 18 00 00
    lea dx, [bx+007h]                         ; 8d 57 07
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 080h                 ; a8 80
    je short 028c5h                           ; 74 06
    mov dx, strict word 00002h                ; ba 02 00
    jmp near 02adfh                           ; e9 1a 02
    mov dx, word [bp-00ch]                    ; 8b 56 f4
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 00ah                  ; b0 0a
    out DX, AL                                ; ee
    lea dx, [bx+004h]                         ; 8d 57 04
    mov AL, strict byte 0f0h                  ; b0 f0
    out DX, AL                                ; ee
    lea dx, [bx+005h]                         ; 8d 57 05
    mov AL, strict byte 0ffh                  ; b0 ff
    out DX, AL                                ; ee
    cmp byte [bp-002h], 000h                  ; 80 7e fe 00
    je short 028e5h                           ; 74 05
    mov ax, 000b0h                            ; b8 b0 00
    jmp short 028e8h                          ; eb 03
    mov ax, 000a0h                            ; b8 a0 00
    lea dx, [bx+006h]                         ; 8d 57 06
    out DX, AL                                ; ee
    lea dx, [bx+007h]                         ; 8d 57 07
    mov AL, strict byte 0a0h                  ; b0 a0
    out DX, AL                                ; ee
    lea dx, [bx+007h]                         ; 8d 57 07
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov dl, al                                ; 88 c2
    test AL, strict byte 080h                 ; a8 80
    jne short 028f2h                          ; 75 f4
    test AL, strict byte 001h                 ; a8 01
    je short 02911h                           ; 74 0f
    mov dx, word [bp-00ch]                    ; 8b 56 f4
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    mov dx, strict word 00003h                ; ba 03 00
    jmp near 02adfh                           ; e9 ce 01
    test dl, 008h                             ; f6 c2 08
    jne short 02925h                          ; 75 0f
    mov dx, word [bp-00ch]                    ; 8b 56 f4
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    mov dx, strict word 00004h                ; ba 04 00
    jmp near 02adfh                           ; e9 ba 01
    sti                                       ; fb
    mov ax, di                                ; 89 f8
    shr ax, 004h                              ; c1 e8 04
    add ax, cx                                ; 01 c8
    mov si, di                                ; 89 fe
    and si, strict byte 0000fh                ; 83 e6 0f
    movzx cx, byte [bp-004h]                  ; 0f b6 4e fc
    mov dx, bx                                ; 89 da
    mov es, ax                                ; 8e c0
    db  0f3h, 026h, 06fh
    ; rep es outsw                              ; f3 26 6f
    cmp byte [bp+00eh], 000h                  ; 80 7e 0e 00
    jne short 0294eh                          ; 75 0b
    lea dx, [bx+007h]                         ; 8d 57 07
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov dl, al                                ; 88 c2
    jmp near 02ac0h                           ; e9 72 01
    lea dx, [bx+007h]                         ; 8d 57 07
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov dl, al                                ; 88 c2
    test AL, strict byte 080h                 ; a8 80
    jne short 0294eh                          ; 75 f4
    test AL, strict byte 088h                 ; a8 88
    je near 02ac0h                            ; 0f 84 60 01
    test AL, strict byte 001h                 ; a8 01
    je short 0296fh                           ; 74 0b
    mov dx, word [bp-00ch]                    ; 8b 56 f4
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    jmp short 0290bh                          ; eb 9c
    mov al, dl                                ; 88 d0
    and AL, strict byte 0c9h                  ; 24 c9
    cmp AL, strict byte 048h                  ; 3c 48
    je short 02982h                           ; 74 0b
    mov dx, word [bp-00ch]                    ; 8b 56 f4
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    jmp short 0291fh                          ; eb 9d
    mov ax, word [bp+010h]                    ; 8b 46 10
    shr ax, 004h                              ; c1 e8 04
    mov dx, word [bp+012h]                    ; 8b 56 12
    add dx, ax                                ; 01 c2
    mov ax, word [bp+010h]                    ; 8b 46 10
    and ax, strict word 0000fh                ; 25 0f 00
    mov word [bp+010h], ax                    ; 89 46 10
    mov word [bp+012h], dx                    ; 89 56 12
    lea dx, [bx+005h]                         ; 8d 57 05
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov cx, ax                                ; 89 c1
    sal cx, 008h                              ; c1 e1 08
    lea dx, [bx+004h]                         ; 8d 57 04
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    add cx, ax                                ; 01 c1
    mov word [bp-010h], cx                    ; 89 4e f0
    mov ax, word [bp+008h]                    ; 8b 46 08
    cmp ax, cx                                ; 39 c8
    jbe short 029c2h                          ; 76 0c
    mov ax, cx                                ; 89 c8
    sub word [bp+008h], cx                    ; 29 4e 08
    xor ax, cx                                ; 31 c8
    mov word [bp-010h], ax                    ; 89 46 f0
    jmp short 029cch                          ; eb 0a
    mov cx, ax                                ; 89 c1
    mov word [bp+008h], strict word 00000h    ; c7 46 08 00 00
    sub word [bp-010h], ax                    ; 29 46 f0
    xor ax, ax                                ; 31 c0
    cmp word [bp+00ch], strict byte 00000h    ; 83 7e 0c 00
    jne short 029f5h                          ; 75 21
    mov dx, word [bp-010h]                    ; 8b 56 f0
    cmp dx, word [bp+00ah]                    ; 3b 56 0a
    jbe short 029f5h                          ; 76 19
    mov ax, word [bp-010h]                    ; 8b 46 f0
    sub ax, word [bp+00ah]                    ; 2b 46 0a
    mov word [bp-008h], ax                    ; 89 46 f8
    mov ax, word [bp+00ah]                    ; 8b 46 0a
    mov word [bp-010h], ax                    ; 89 46 f0
    xor ax, ax                                ; 31 c0
    mov word [bp+00ah], ax                    ; 89 46 0a
    mov word [bp+00ch], ax                    ; 89 46 0c
    jmp short 02a01h                          ; eb 0c
    mov word [bp-008h], ax                    ; 89 46 f8
    mov dx, word [bp-010h]                    ; 8b 56 f0
    sub word [bp+00ah], dx                    ; 29 56 0a
    sbb word [bp+00ch], ax                    ; 19 46 0c
    mov si, word [bp-010h]                    ; 8b 76 f0
    mov al, byte [bp-006h]                    ; 8a 46 fa
    test cl, 003h                             ; f6 c1 03
    je short 02a0eh                           ; 74 02
    xor al, al                                ; 30 c0
    test byte [bp-010h], 003h                 ; f6 46 f0 03
    je short 02a16h                           ; 74 02
    xor al, al                                ; 30 c0
    test byte [bp-008h], 003h                 ; f6 46 f8 03
    je short 02a1eh                           ; 74 02
    xor al, al                                ; 30 c0
    test byte [bp-010h], 001h                 ; f6 46 f0 01
    je short 02a36h                           ; 74 12
    inc word [bp-010h]                        ; ff 46 f0
    cmp word [bp-008h], strict byte 00000h    ; 83 7e f8 00
    jbe short 02a36h                          ; 76 09
    test byte [bp-008h], 001h                 ; f6 46 f8 01
    je short 02a36h                           ; 74 03
    dec word [bp-008h]                        ; ff 4e f8
    cmp AL, strict byte 001h                  ; 3c 01
    jne short 02a47h                          ; 75 0d
    shr word [bp-010h], 002h                  ; c1 6e f0 02
    shr cx, 002h                              ; c1 e9 02
    shr word [bp-008h], 002h                  ; c1 6e f8 02
    jmp short 02a4fh                          ; eb 08
    shr word [bp-010h], 1                     ; d1 6e f0
    shr cx, 1                                 ; d1 e9
    shr word [bp-008h], 1                     ; d1 6e f8
    cmp AL, strict byte 001h                  ; 3c 01
    jne short 02a7fh                          ; 75 2c
    test cx, cx                               ; 85 c9
    je short 02a61h                           ; 74 0a
    mov dx, bx                                ; 89 da
    push eax                                  ; 66 50
    in eax, DX                                ; 66 ed
    loop 02a5bh                               ; e2 fc
    pop eax                                   ; 66 58
    mov dx, bx                                ; 89 da
    mov cx, word [bp-010h]                    ; 8b 4e f0
    les di, [bp+010h]                         ; c4 7e 10
    db  0f3h, 066h, 06dh
    ; rep insd                                  ; f3 66 6d
    mov ax, word [bp-008h]                    ; 8b 46 f8
    test ax, ax                               ; 85 c0
    je short 02a9eh                           ; 74 2b
    mov cx, ax                                ; 89 c1
    push eax                                  ; 66 50
    in eax, DX                                ; 66 ed
    loop 02a77h                               ; e2 fc
    pop eax                                   ; 66 58
    jmp short 02a9eh                          ; eb 1f
    test cx, cx                               ; 85 c9
    je short 02a88h                           ; 74 05
    mov dx, bx                                ; 89 da
    in ax, DX                                 ; ed
    loop 02a85h                               ; e2 fd
    mov dx, bx                                ; 89 da
    mov cx, word [bp-010h]                    ; 8b 4e f0
    les di, [bp+010h]                         ; c4 7e 10
    rep insw                                  ; f3 6d
    mov ax, word [bp-008h]                    ; 8b 46 f8
    test ax, ax                               ; 85 c0
    je short 02a9eh                           ; 74 05
    mov cx, ax                                ; 89 c1
    in ax, DX                                 ; ed
    loop 02a9bh                               ; e2 fd
    add word [bp+010h], si                    ; 01 76 10
    xor ax, ax                                ; 31 c0
    add word [bp-014h], si                    ; 01 76 ec
    adc word [bp-012h], ax                    ; 11 46 ee
    mov ax, word [bp-014h]                    ; 8b 46 ec
    mov es, [bp-00ah]                         ; 8e 46 f6
    mov si, word [bp-00eh]                    ; 8b 76 f2
    mov word [es:si+016h], ax                 ; 26 89 44 16
    mov ax, word [bp-012h]                    ; 8b 46 ee
    mov word [es:si+018h], ax                 ; 26 89 44 18
    jmp near 0294eh                           ; e9 8e fe
    mov al, dl                                ; 88 d0
    and AL, strict byte 0e9h                  ; 24 e9
    cmp AL, strict byte 040h                  ; 3c 40
    je short 02ad4h                           ; 74 0c
    mov dx, word [bp-00ch]                    ; 8b 56 f4
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    jmp near 0291fh                           ; e9 4b fe
    mov dx, word [bp-00ch]                    ; 8b 56 f4
    add dx, strict byte 00006h                ; 83 c2 06
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    xor dx, dx                                ; 31 d2
    mov ax, dx                                ; 89 d0
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn 0000ch                               ; c2 0c 00
set_diskette_ret_status_:                    ; 0xf2ae7 LB 0x15
    push bx                                   ; 53
    push dx                                   ; 52
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 00041h                ; ba 41 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 16 eb
    pop bp                                    ; 5d
    pop dx                                    ; 5a
    pop bx                                    ; 5b
    retn                                      ; c3
set_diskette_current_cyl_:                   ; 0xf2afc LB 0x2a
    push bx                                   ; 53
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bl, al                                ; 88 c3
    cmp AL, strict byte 001h                  ; 3c 01
    jbe short 02b11h                          ; 76 0b
    push 001b0h                               ; 68 b0 01
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 db ed
    add sp, strict byte 00004h                ; 83 c4 04
    movzx ax, dl                              ; 0f b6 c2
    movzx dx, bl                              ; 0f b6 d3
    add dx, 00094h                            ; 81 c2 94 00
    mov bx, ax                                ; 89 c3
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 eb ea
    pop bp                                    ; 5d
    pop bx                                    ; 5b
    retn                                      ; c3
floppy_reset_controller_:                    ; 0xf2b26 LB 0x28
    push bx                                   ; 53
    push dx                                   ; 52
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov dx, 003f2h                            ; ba f2 03
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov bx, ax                                ; 89 c3
    movzx ax, bl                              ; 0f b6 c3
    and AL, strict byte 0fbh                  ; 24 fb
    out DX, AL                                ; ee
    mov al, bl                                ; 88 d8
    or AL, strict byte 004h                   ; 0c 04
    out DX, AL                                ; ee
    mov dx, 003f4h                            ; ba f4 03
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    and AL, strict byte 0c0h                  ; 24 c0
    cmp AL, strict byte 080h                  ; 3c 80
    jne short 02b3eh                          ; 75 f4
    pop bp                                    ; 5d
    pop dx                                    ; 5a
    pop bx                                    ; 5b
    retn                                      ; c3
floppy_prepare_controller_:                  ; 0xf2b4e LB 0x8c
    push bx                                   ; 53
    push cx                                   ; 51
    push dx                                   ; 52
    enter 00002h, 000h                        ; c8 02 00 00
    mov cx, ax                                ; 89 c1
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 a0 ea
    and AL, strict byte 07fh                  ; 24 7f
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 a0 ea
    mov dx, 003f2h                            ; ba f2 03
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    and AL, strict byte 004h                  ; 24 04
    mov byte [bp-002h], al                    ; 88 46 fe
    test cx, cx                               ; 85 c9
    je short 02b81h                           ; 74 04
    mov AL, strict byte 020h                  ; b0 20
    jmp short 02b83h                          ; eb 02
    mov AL, strict byte 010h                  ; b0 10
    or AL, strict byte 00ch                   ; 0c 0c
    or al, cl                                 ; 08 c8
    mov dx, 003f2h                            ; ba f2 03
    out DX, AL                                ; ee
    mov bx, strict word 00025h                ; bb 25 00
    mov dx, strict word 00040h                ; ba 40 00
    mov ax, dx                                ; 89 d0
    call 0160eh                               ; e8 78 ea
    mov dx, 0008bh                            ; ba 8b 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 61 ea
    shr al, 006h                              ; c0 e8 06
    mov dx, 003f7h                            ; ba f7 03
    out DX, AL                                ; ee
    mov dx, 003f4h                            ; ba f4 03
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    and AL, strict byte 0c0h                  ; 24 c0
    cmp AL, strict byte 080h                  ; 3c 80
    jne short 02ba6h                          ; 75 f4
    cmp byte [bp-002h], 000h                  ; 80 7e fe 00
    jne short 02bd5h                          ; 75 1d
    sti                                       ; fb
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 3e ea
    test AL, strict byte 080h                 ; a8 80
    je short 02bb9h                           ; 74 f3
    and AL, strict byte 07fh                  ; 24 7f
    cli                                       ; fa
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 39 ea
    leave                                     ; c9
    pop dx                                    ; 5a
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
floppy_media_known_:                         ; 0xf2bda LB 0x40
    push bx                                   ; 53
    push dx                                   ; 52
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bx, ax                                ; 89 c3
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 16 ea
    mov ah, al                                ; 88 c4
    test bx, bx                               ; 85 db
    je short 02bf2h                           ; 74 02
    shr al, 1                                 ; d0 e8
    and AL, strict byte 001h                  ; 24 01
    jne short 02bfah                          ; 75 04
    xor ah, ah                                ; 30 e4
    jmp short 02c16h                          ; eb 1c
    mov dx, 00090h                            ; ba 90 00
    test bx, bx                               ; 85 db
    je short 02c04h                           ; 74 03
    mov dx, 00091h                            ; ba 91 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 f6 e9
    xor ah, ah                                ; 30 e4
    sar ax, 004h                              ; c1 f8 04
    and AL, strict byte 001h                  ; 24 01
    je short 02bf6h                           ; 74 e3
    mov ax, strict word 00001h                ; b8 01 00
    pop bp                                    ; 5d
    pop dx                                    ; 5a
    pop bx                                    ; 5b
    retn                                      ; c3
floppy_read_id_:                             ; 0xf2c1a LB 0x49
    push bx                                   ; 53
    push dx                                   ; 52
    push si                                   ; 56
    enter 00008h, 000h                        ; c8 08 00 00
    mov bx, ax                                ; 89 c3
    call 02b4eh                               ; e8 28 ff
    mov AL, strict byte 04ah                  ; b0 4a
    mov dx, 003f5h                            ; ba f5 03
    out DX, AL                                ; ee
    mov al, bl                                ; 88 d8
    out DX, AL                                ; ee
    sti                                       ; fb
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 c7 e9
    and AL, strict byte 080h                  ; 24 80
    test al, al                               ; 84 c0
    je short 02c30h                           ; 74 f1
    cli                                       ; fa
    xor si, si                                ; 31 f6
    jmp short 02c49h                          ; eb 05
    cmp si, strict byte 00007h                ; 83 fe 07
    jnl short 02c55h                          ; 7d 0c
    mov dx, 003f5h                            ; ba f5 03
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov byte [bp+si-008h], al                 ; 88 42 f8
    inc si                                    ; 46
    jmp short 02c44h                          ; eb ef
    test byte [bp-008h], 0c0h                 ; f6 46 f8 c0
    db  00fh, 094h, 0c0h
    ; sete al                                   ; 0f 94 c0
    xor ah, ah                                ; 30 e4
    leave                                     ; c9
    pop si                                    ; 5e
    pop dx                                    ; 5a
    pop bx                                    ; 5b
    retn                                      ; c3
floppy_drive_recal_:                         ; 0xf2c63 LB 0x5e
    push bx                                   ; 53
    push cx                                   ; 51
    push dx                                   ; 52
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bx, ax                                ; 89 c3
    call 02b4eh                               ; e8 e0 fe
    mov AL, strict byte 007h                  ; b0 07
    mov dx, 003f5h                            ; ba f5 03
    out DX, AL                                ; ee
    mov al, bl                                ; 88 d8
    out DX, AL                                ; ee
    sti                                       ; fb
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 7f e9
    and AL, strict byte 080h                  ; 24 80
    test al, al                               ; 84 c0
    je short 02c78h                           ; 74 f1
    cli                                       ; fa
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 6f e9
    and AL, strict byte 07fh                  ; 24 7f
    test bx, bx                               ; 85 db
    je short 02c9eh                           ; 74 07
    or AL, strict byte 002h                   ; 0c 02
    mov cx, 00095h                            ; b9 95 00
    jmp short 02ca3h                          ; eb 05
    or AL, strict byte 001h                   ; 0c 01
    mov cx, 00094h                            ; b9 94 00
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 5f e9
    xor bx, bx                                ; 31 db
    mov dx, cx                                ; 89 ca
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 55 e9
    mov ax, strict word 00001h                ; b8 01 00
    pop bp                                    ; 5d
    pop dx                                    ; 5a
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
floppy_media_sense_:                         ; 0xf2cc1 LB 0xfa
    push bx                                   ; 53
    push cx                                   ; 51
    push dx                                   ; 52
    push si                                   ; 56
    push di                                   ; 57
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov di, ax                                ; 89 c7
    call 02c63h                               ; e8 95 ff
    test ax, ax                               ; 85 c0
    jne short 02cd7h                          ; 75 05
    xor cx, cx                                ; 31 c9
    jmp near 02db2h                           ; e9 db 00
    mov ax, strict word 00010h                ; b8 10 00
    call 0165ch                               ; e8 7f e9
    test di, di                               ; 85 ff
    jne short 02ce8h                          ; 75 07
    mov cl, al                                ; 88 c1
    shr cl, 004h                              ; c0 e9 04
    jmp short 02cedh                          ; eb 05
    mov cl, al                                ; 88 c1
    and cl, 00fh                              ; 80 e1 0f
    cmp cl, 001h                              ; 80 f9 01
    jne short 02cfbh                          ; 75 09
    xor cl, cl                                ; 30 c9
    mov CH, strict byte 015h                  ; b5 15
    mov si, strict word 00001h                ; be 01 00
    jmp short 02d46h                          ; eb 4b
    cmp cl, 002h                              ; 80 f9 02
    jne short 02d06h                          ; 75 06
    xor cl, cl                                ; 30 c9
    mov CH, strict byte 035h                  ; b5 35
    jmp short 02cf6h                          ; eb f0
    cmp cl, 003h                              ; 80 f9 03
    jne short 02d11h                          ; 75 06
    xor cl, cl                                ; 30 c9
    mov CH, strict byte 017h                  ; b5 17
    jmp short 02cf6h                          ; eb e5
    cmp cl, 004h                              ; 80 f9 04
    jne short 02d1ch                          ; 75 06
    xor cl, cl                                ; 30 c9
    mov CH, strict byte 017h                  ; b5 17
    jmp short 02cf6h                          ; eb da
    cmp cl, 005h                              ; 80 f9 05
    jne short 02d27h                          ; 75 06
    mov CL, strict byte 0cch                  ; b1 cc
    mov CH, strict byte 0d7h                  ; b5 d7
    jmp short 02cf6h                          ; eb cf
    cmp cl, 006h                              ; 80 f9 06
    jne short 02d32h                          ; 75 06
    xor cl, cl                                ; 30 c9
    mov CH, strict byte 027h                  ; b5 27
    jmp short 02cf6h                          ; eb c4
    cmp cl, 007h                              ; 80 f9 07
    jne short 02d39h                          ; 75 02
    jmp short 02d2ch                          ; eb f3
    cmp cl, 008h                              ; 80 f9 08
    jne short 02d40h                          ; 75 02
    jmp short 02d2ch                          ; eb ec
    xor cl, cl                                ; 30 c9
    xor ch, ch                                ; 30 ed
    xor si, si                                ; 31 f6
    movzx bx, cl                              ; 0f b6 d9
    mov dx, 0008bh                            ; ba 8b 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 bc e8
    mov ax, di                                ; 89 f8
    call 02c1ah                               ; e8 c3 fe
    test ax, ax                               ; 85 c0
    jne short 02d8dh                          ; 75 32
    mov al, cl                                ; 88 c8
    and AL, strict byte 0c0h                  ; 24 c0
    cmp AL, strict byte 080h                  ; 3c 80
    je short 02d8dh                           ; 74 2a
    mov al, cl                                ; 88 c8
    and AL, strict byte 0c0h                  ; 24 c0
    cmp AL, strict byte 0c0h                  ; 3c c0
    je short 02d7ah                           ; 74 0f
    mov ah, cl                                ; 88 cc
    and ah, 03fh                              ; 80 e4 3f
    cmp AL, strict byte 040h                  ; 3c 40
    je short 02d86h                           ; 74 12
    test al, al                               ; 84 c0
    je short 02d7fh                           ; 74 07
    jmp short 02d46h                          ; eb cc
    and cl, 03fh                              ; 80 e1 3f
    jmp short 02d46h                          ; eb c7
    mov cl, ah                                ; 88 e1
    or cl, 040h                               ; 80 c9 40
    jmp short 02d46h                          ; eb c0
    mov cl, ah                                ; 88 e1
    or cl, 080h                               ; 80 c9 80
    jmp short 02d46h                          ; eb b9
    test di, di                               ; 85 ff
    db  00fh, 095h, 0c0h
    ; setne al                                  ; 0f 95 c0
    movzx di, al                              ; 0f b6 f8
    add di, 00090h                            ; 81 c7 90 00
    movzx bx, cl                              ; 0f b6 d9
    mov dx, 0008bh                            ; ba 8b 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 69 e8
    movzx bx, ch                              ; 0f b6 dd
    mov dx, di                                ; 89 fa
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 5e e8
    mov cx, si                                ; 89 f1
    mov ax, cx                                ; 89 c8
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop dx                                    ; 5a
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
floppy_drive_exists_:                        ; 0xf2dbb LB 0x33
    push dx                                   ; 52
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov dx, ax                                ; 89 c2
    mov ax, strict word 00010h                ; b8 10 00
    call 0165ch                               ; e8 95 e8
    test dx, dx                               ; 85 d2
    jne short 02dd0h                          ; 75 05
    shr al, 004h                              ; c0 e8 04
    jmp short 02dd2h                          ; eb 02
    and AL, strict byte 00fh                  ; 24 0f
    test al, al                               ; 84 c0
    db  00fh, 095h, 0c0h
    ; setne al                                  ; 0f 95 c0
    xor ah, ah                                ; 30 e4
    pop bp                                    ; 5d
    pop dx                                    ; 5a
    retn                                      ; c3
    mov byte [0ad34h], AL                     ; a2 34 ad
    xor AL, strict byte 0b7h                  ; 34 b7
    xor AL, strict byte 0beh                  ; 34 be
    xor AL, strict byte 0c5h                  ; 34 c5
    xor AL, strict byte 0cch                  ; 34 cc
    xor AL, strict byte 0d3h                  ; 34 d3
    xor AL, strict byte 0ddh                  ; 34 dd
    xor AL, strict byte 0e4h                  ; 34 e4
    db  034h
_int13_diskette_function:                    ; 0xf2dee LB 0x799
    push si                                   ; 56
    push di                                   ; 57
    enter 00012h, 000h                        ; c8 12 00 00
    mov bx, word [bp+01ah]                    ; 8b 5e 1a
    shr bx, 008h                              ; c1 eb 08
    mov ch, bl                                ; 88 dd
    mov si, word [bp+01ah]                    ; 8b 76 1a
    and si, 000ffh                            ; 81 e6 ff 00
    mov ah, byte [bp+012h]                    ; 8a 66 12
    cmp bl, 008h                              ; 80 fb 08
    jc short 02e45h                           ; 72 3a
    mov dx, word [bp+020h]                    ; 8b 56 20
    or dl, 001h                               ; 80 ca 01
    cmp bl, 008h                              ; 80 fb 08
    jbe near 03423h                           ; 0f 86 0b 06
    cmp bl, 016h                              ; 80 fb 16
    jc short 02e3bh                           ; 72 1e
    or si, 00100h                             ; 81 ce 00 01
    mov cx, si                                ; 89 f1
    cmp bl, 016h                              ; 80 fb 16
    jbe near 03542h                           ; 0f 86 18 07
    cmp bl, 018h                              ; 80 fb 18
    je near 03547h                            ; 0f 84 16 07
    cmp bl, 017h                              ; 80 fb 17
    je near 03547h                            ; 0f 84 0f 07
    jmp near 03564h                           ; e9 29 07
    cmp bl, 015h                              ; 80 fb 15
    je near 03506h                            ; 0f 84 c4 06
    jmp near 03564h                           ; e9 1f 07
    cmp bl, 001h                              ; 80 fb 01
    jc short 02e5fh                           ; 72 15
    jbe near 02ed5h                           ; 0f 86 87 00
    cmp bl, 005h                              ; 80 fb 05
    je near 0327ch                            ; 0f 84 27 04
    cmp bl, 004h                              ; 80 fb 04
    jbe near 02ef3h                           ; 0f 86 97 00
    jmp near 03564h                           ; e9 05 07
    test bl, bl                               ; 84 db
    jne near 03564h                           ; 0f 85 ff 06
    mov al, byte [bp+012h]                    ; 8a 46 12
    mov byte [bp-00ah], al                    ; 88 46 f6
    cmp AL, strict byte 001h                  ; 3c 01
    jbe short 02e83h                          ; 76 14
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    xor ah, ah                                ; 30 e4
    or ah, 001h                               ; 80 cc 01
    mov word [bp+01ah], ax                    ; 89 46 1a
    mov ax, strict word 00001h                ; b8 01 00
    call 02ae7h                               ; e8 67 fc
    jmp near 0324eh                           ; e9 cb 03
    mov ax, strict word 00010h                ; b8 10 00
    call 0165ch                               ; e8 d3 e7
    cmp byte [bp-00ah], 000h                  ; 80 7e f6 00
    jne short 02e96h                          ; 75 07
    mov bl, al                                ; 88 c3
    shr bl, 004h                              ; c0 eb 04
    jmp short 02e9bh                          ; eb 05
    mov bl, al                                ; 88 c3
    and bl, 00fh                              ; 80 e3 0f
    test bl, bl                               ; 84 db
    jne short 02eafh                          ; 75 10
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    xor ah, ah                                ; 30 e4
    or ah, 080h                               ; 80 cc 80
    mov word [bp+01ah], ax                    ; 89 46 1a
    mov ax, 00080h                            ; b8 80 00
    jmp short 02e7dh                          ; eb ce
    xor bx, bx                                ; 31 db
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 54 e7
    xor al, al                                ; 30 c0
    mov byte [bp+01bh], al                    ; 88 46 1b
    xor ah, ah                                ; 30 e4
    call 02ae7h                               ; e8 23 fc
    and byte [bp+020h], 0feh                  ; 80 66 20 fe
    movzx ax, byte [bp-00ah]                  ; 0f b6 46 f6
    xor dx, dx                                ; 31 d2
    call 02afch                               ; e8 2b fc
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn                                      ; c3
    and byte [bp+020h], 0feh                  ; 80 66 20 fe
    mov dx, 00441h                            ; ba 41 04
    xor ax, ax                                ; 31 c0
    call 01600h                               ; e8 1f e7
    movzx dx, al                              ; 0f b6 d0
    sal dx, 008h                              ; c1 e2 08
    or si, dx                                 ; 09 d6
    mov word [bp+01ah], si                    ; 89 76 1a
    test al, al                               ; 84 c0
    je short 02ed1h                           ; 74 e1
    jmp near 0324eh                           ; e9 5b 03
    mov al, byte [bp+01ah]                    ; 8a 46 1a
    mov byte [bp-008h], al                    ; 88 46 f8
    mov dx, word [bp+018h]                    ; 8b 56 18
    shr dx, 008h                              ; c1 ea 08
    mov byte [bp-004h], dl                    ; 88 56 fc
    mov al, byte [bp+018h]                    ; 8a 46 18
    mov byte [bp-006h], al                    ; 88 46 fa
    mov dx, word [bp+016h]                    ; 8b 56 16
    shr dx, 008h                              ; c1 ea 08
    mov byte [bp-002h], dl                    ; 88 56 fe
    mov byte [bp-00ah], ah                    ; 88 66 f6
    cmp ah, 001h                              ; 80 fc 01
    jnbe short 02f29h                         ; 77 10
    cmp dl, 001h                              ; 80 fa 01
    jnbe short 02f29h                         ; 77 0b
    mov al, byte [bp-008h]                    ; 8a 46 f8
    test al, al                               ; 84 c0
    je short 02f29h                           ; 74 04
    cmp AL, strict byte 048h                  ; 3c 48
    jbe short 02f52h                          ; 76 29
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 7a e9
    push 001d5h                               ; 68 d5 01
    push 001edh                               ; 68 ed 01
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 aa e9
    add sp, strict byte 00006h                ; 83 c4 06
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    xor ah, ah                                ; 30 e4
    or ah, 001h                               ; 80 cc 01
    mov word [bp+01ah], ax                    ; 89 46 1a
    mov ax, strict word 00001h                ; b8 01 00
    jmp short 02fc3h                          ; eb 71
    movzx ax, byte [bp-00ah]                  ; 0f b6 46 f6
    call 02dbbh                               ; e8 62 fe
    test ax, ax                               ; 85 c0
    je near 0305ch                            ; 0f 84 fd 00
    movzx dx, byte [bp-00ah]                  ; 0f b6 56 f6
    mov ax, dx                                ; 89 d0
    call 02bdah                               ; e8 72 fc
    test ax, ax                               ; 85 c0
    jne short 02f85h                          ; 75 19
    mov ax, dx                                ; 89 d0
    call 02cc1h                               ; e8 50 fd
    test ax, ax                               ; 85 c0
    jne short 02f85h                          ; 75 10
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    xor ah, ah                                ; 30 e4
    or ah, 00ch                               ; 80 cc 0c
    mov word [bp+01ah], ax                    ; 89 46 1a
    mov ax, strict word 0000ch                ; b8 0c 00
    jmp short 02fc3h                          ; eb 3e
    cmp ch, 002h                              ; 80 fd 02
    jne near 0311ah                           ; 0f 85 8e 01
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    shr dx, 00ch                              ; c1 ea 0c
    mov ah, dl                                ; 88 d4
    mov cx, word [bp+00ah]                    ; 8b 4e 0a
    sal cx, 004h                              ; c1 e1 04
    mov bx, word [bp+014h]                    ; 8b 5e 14
    add bx, cx                                ; 01 cb
    cmp bx, cx                                ; 39 cb
    jnc short 02fa5h                          ; 73 02
    db  0feh, 0c4h
    ; inc ah                                    ; fe c4
    movzx cx, byte [bp-008h]                  ; 0f b6 4e f8
    sal cx, 009h                              ; c1 e1 09
    dec cx                                    ; 49
    mov dx, bx                                ; 89 da
    add dx, cx                                ; 01 ca
    cmp dx, bx                                ; 39 da
    jnc short 02fcdh                          ; 73 18
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    xor ah, ah                                ; 30 e4
    or ah, 009h                               ; 80 cc 09
    mov word [bp+01ah], ax                    ; 89 46 1a
    mov ax, strict word 00009h                ; b8 09 00
    call 02ae7h                               ; e8 21 fb
    mov byte [bp+01ah], 000h                  ; c6 46 1a 00
    jmp near 0324eh                           ; e9 81 02
    mov AL, strict byte 006h                  ; b0 06
    mov dx, strict word 0000ah                ; ba 0a 00
    out DX, AL                                ; ee
    xor al, al                                ; 30 c0
    mov dx, strict word 0000ch                ; ba 0c 00
    out DX, AL                                ; ee
    mov al, bl                                ; 88 d8
    mov dx, strict word 00004h                ; ba 04 00
    out DX, AL                                ; ee
    shr bx, 008h                              ; c1 eb 08
    mov al, bl                                ; 88 d8
    out DX, AL                                ; ee
    xor al, bl                                ; 30 d8
    mov dx, strict word 0000ch                ; ba 0c 00
    out DX, AL                                ; ee
    mov al, cl                                ; 88 c8
    mov dx, strict word 00005h                ; ba 05 00
    out DX, AL                                ; ee
    shr cx, 008h                              ; c1 e9 08
    mov al, cl                                ; 88 c8
    out DX, AL                                ; ee
    mov AL, strict byte 046h                  ; b0 46
    mov dx, strict word 0000bh                ; ba 0b 00
    out DX, AL                                ; ee
    mov al, ah                                ; 88 e0
    mov dx, 00081h                            ; ba 81 00
    out DX, AL                                ; ee
    mov AL, strict byte 002h                  ; b0 02
    mov dx, strict word 0000ah                ; ba 0a 00
    out DX, AL                                ; ee
    out DX, AL                                ; ee
    movzx ax, byte [bp-00ah]                  ; 0f b6 46 f6
    call 02b4eh                               ; e8 3d fb
    mov AL, strict byte 0e6h                  ; b0 e6
    mov dx, 003f5h                            ; ba f5 03
    out DX, AL                                ; ee
    movzx dx, byte [bp-002h]                  ; 0f b6 56 fe
    sal dx, 002h                              ; c1 e2 02
    movzx ax, byte [bp-00ah]                  ; 0f b6 46 f6
    or ax, dx                                 ; 09 d0
    mov dx, 003f5h                            ; ba f5 03
    out DX, AL                                ; ee
    mov al, byte [bp-004h]                    ; 8a 46 fc
    out DX, AL                                ; ee
    mov al, byte [bp-002h]                    ; 8a 46 fe
    out DX, AL                                ; ee
    mov al, byte [bp-006h]                    ; 8a 46 fa
    out DX, AL                                ; ee
    mov AL, strict byte 002h                  ; b0 02
    out DX, AL                                ; ee
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    movzx dx, byte [bp-008h]                  ; 0f b6 56 f8
    add ax, dx                                ; 01 d0
    dec ax                                    ; 48
    mov dx, 003f5h                            ; ba f5 03
    out DX, AL                                ; ee
    xor al, al                                ; 30 c0
    out DX, AL                                ; ee
    mov AL, strict byte 0ffh                  ; b0 ff
    out DX, AL                                ; ee
    sti                                       ; fb
    mov dx, strict word 00040h                ; ba 40 00
    mov ax, dx                                ; 89 d0
    call 01600h                               ; e8 ab e5
    test al, al                               ; 84 c0
    jne short 0306dh                          ; 75 14
    call 02b26h                               ; e8 ca fa
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    xor ah, ah                                ; 30 e4
    or ah, 080h                               ; 80 cc 80
    mov word [bp+01ah], ax                    ; 89 46 1a
    mov ax, 00080h                            ; b8 80 00
    jmp near 02fc3h                           ; e9 56 ff
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 8a e5
    and AL, strict byte 080h                  ; 24 80
    test al, al                               ; 84 c0
    je short 0304dh                           ; 74 d1
    cli                                       ; fa
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 7a e5
    and AL, strict byte 07fh                  ; 24 7f
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 7a e5
    mov dx, 003f4h                            ; ba f4 03
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    and AL, strict byte 0c0h                  ; 24 c0
    cmp AL, strict byte 0c0h                  ; 3c c0
    je short 030aeh                           ; 74 0e
    push 001d5h                               ; 68 d5 01
    push 00208h                               ; 68 08 02
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 3e e8
    add sp, strict byte 00006h                ; 83 c4 06
    xor si, si                                ; 31 f6
    jmp short 030b7h                          ; eb 05
    cmp si, strict byte 00007h                ; 83 fe 07
    jnl short 030cfh                          ; 7d 18
    mov dx, 003f5h                            ; ba f5 03
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov byte [bp+si-012h], al                 ; 88 42 ee
    movzx bx, al                              ; 0f b6 d8
    lea dx, [si+042h]                         ; 8d 54 42
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 42 e5
    inc si                                    ; 46
    jmp short 030b2h                          ; eb e3
    test byte [bp-012h], 0c0h                 ; f6 46 ee c0
    je short 030e6h                           ; 74 11
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    xor ah, ah                                ; 30 e4
    or ah, 020h                               ; 80 cc 20
    mov word [bp+01ah], ax                    ; 89 46 1a
    mov ax, strict word 00020h                ; b8 20 00
    jmp near 02fc3h                           ; e9 dd fe
    movzx ax, byte [bp-008h]                  ; 0f b6 46 f8
    sal ax, 009h                              ; c1 e0 09
    cwd                                       ; 99
    db  02bh, 0c2h
    ; sub ax, dx                                ; 2b c2
    sar ax, 1                                 ; d1 f8
    mov cx, ax                                ; 89 c1
    mov si, word [bp+014h]                    ; 8b 76 14
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    mov di, si                                ; 89 f7
    mov es, dx                                ; 8e c2
    push DS                                   ; 1e
    mov ds, dx                                ; 8e da
    rep movsw                                 ; f3 a5
    pop DS                                    ; 1f
    movzx dx, byte [bp-004h]                  ; 0f b6 56 fc
    movzx ax, byte [bp-00ah]                  ; 0f b6 46 f6
    call 02afch                               ; e8 ed f9
    mov byte [bp+01bh], 000h                  ; c6 46 1b 00
    and byte [bp+020h], 0feh                  ; 80 66 20 fe
    jmp near 02ed1h                           ; e9 b7 fd
    cmp ch, 003h                              ; 80 fd 03
    jne near 03266h                           ; 0f 85 45 01
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    shr dx, 00ch                              ; c1 ea 0c
    mov ah, dl                                ; 88 d4
    mov cx, word [bp+00ah]                    ; 8b 4e 0a
    sal cx, 004h                              ; c1 e1 04
    mov bx, word [bp+014h]                    ; 8b 5e 14
    add bx, cx                                ; 01 cb
    cmp bx, cx                                ; 39 cb
    jnc short 0313ah                          ; 73 02
    db  0feh, 0c4h
    ; inc ah                                    ; fe c4
    movzx cx, byte [bp-008h]                  ; 0f b6 4e f8
    sal cx, 009h                              ; c1 e1 09
    dec cx                                    ; 49
    mov dx, bx                                ; 89 da
    add dx, cx                                ; 01 ca
    cmp dx, bx                                ; 39 da
    jc near 02fb5h                            ; 0f 82 69 fe
    mov AL, strict byte 006h                  ; b0 06
    mov dx, strict word 0000ah                ; ba 0a 00
    out DX, AL                                ; ee
    xor al, al                                ; 30 c0
    mov dx, strict word 0000ch                ; ba 0c 00
    out DX, AL                                ; ee
    mov al, bl                                ; 88 d8
    mov dx, strict word 00004h                ; ba 04 00
    out DX, AL                                ; ee
    shr bx, 008h                              ; c1 eb 08
    mov al, bl                                ; 88 d8
    out DX, AL                                ; ee
    xor al, bl                                ; 30 d8
    mov dx, strict word 0000ch                ; ba 0c 00
    out DX, AL                                ; ee
    mov al, cl                                ; 88 c8
    mov dx, strict word 00005h                ; ba 05 00
    out DX, AL                                ; ee
    shr cx, 008h                              ; c1 e9 08
    mov al, cl                                ; 88 c8
    out DX, AL                                ; ee
    mov AL, strict byte 04ah                  ; b0 4a
    mov dx, strict word 0000bh                ; ba 0b 00
    out DX, AL                                ; ee
    mov al, ah                                ; 88 e0
    mov dx, 00081h                            ; ba 81 00
    out DX, AL                                ; ee
    mov AL, strict byte 002h                  ; b0 02
    mov dx, strict word 0000ah                ; ba 0a 00
    out DX, AL                                ; ee
    movzx ax, byte [bp-00ah]                  ; 0f b6 46 f6
    call 02b4eh                               ; e8 bf f9
    mov AL, strict byte 0c5h                  ; b0 c5
    mov dx, 003f5h                            ; ba f5 03
    out DX, AL                                ; ee
    movzx dx, byte [bp-002h]                  ; 0f b6 56 fe
    sal dx, 002h                              ; c1 e2 02
    movzx ax, byte [bp-00ah]                  ; 0f b6 46 f6
    or ax, dx                                 ; 09 d0
    mov dx, 003f5h                            ; ba f5 03
    out DX, AL                                ; ee
    mov al, byte [bp-004h]                    ; 8a 46 fc
    out DX, AL                                ; ee
    mov al, byte [bp-002h]                    ; 8a 46 fe
    out DX, AL                                ; ee
    mov al, byte [bp-006h]                    ; 8a 46 fa
    out DX, AL                                ; ee
    mov AL, strict byte 002h                  ; b0 02
    out DX, AL                                ; ee
    movzx dx, byte [bp-006h]                  ; 0f b6 56 fa
    movzx ax, byte [bp-008h]                  ; 0f b6 46 f8
    add ax, dx                                ; 01 d0
    dec ax                                    ; 48
    mov dx, 003f5h                            ; ba f5 03
    out DX, AL                                ; ee
    xor al, al                                ; 30 c0
    out DX, AL                                ; ee
    mov AL, strict byte 0ffh                  ; b0 ff
    out DX, AL                                ; ee
    sti                                       ; fb
    mov dx, strict word 00040h                ; ba 40 00
    mov ax, dx                                ; 89 d0
    call 01600h                               ; e8 2d e4
    test al, al                               ; 84 c0
    je near 03059h                            ; 0f 84 80 fe
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 1e e4
    and AL, strict byte 080h                  ; 24 80
    test al, al                               ; 84 c0
    je short 031cbh                           ; 74 e3
    cli                                       ; fa
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 0e e4
    and AL, strict byte 07fh                  ; 24 7f
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 0e e4
    mov dx, 003f4h                            ; ba f4 03
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    and AL, strict byte 0c0h                  ; 24 c0
    cmp AL, strict byte 0c0h                  ; 3c c0
    je short 0321ah                           ; 74 0e
    push 001d5h                               ; 68 d5 01
    push 00208h                               ; 68 08 02
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 d2 e6
    add sp, strict byte 00006h                ; 83 c4 06
    xor si, si                                ; 31 f6
    jmp short 03223h                          ; eb 05
    cmp si, strict byte 00007h                ; 83 fe 07
    jnl short 0323bh                          ; 7d 18
    mov dx, 003f5h                            ; ba f5 03
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov byte [bp+si-012h], al                 ; 88 42 ee
    movzx bx, al                              ; 0f b6 d8
    lea dx, [si+042h]                         ; 8d 54 42
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 d6 e3
    inc si                                    ; 46
    jmp short 0321eh                          ; eb e3
    test byte [bp-012h], 0c0h                 ; f6 46 ee c0
    je near 03104h                            ; 0f 84 c1 fe
    test byte [bp-011h], 002h                 ; f6 46 ef 02
    je short 03255h                           ; 74 0c
    mov word [bp+01ah], 00300h                ; c7 46 1a 00 03
    or byte [bp+020h], 001h                   ; 80 4e 20 01
    jmp near 02ed1h                           ; e9 7c fc
    push 001d5h                               ; 68 d5 01
    push 0021ch                               ; 68 1c 02
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 89 e6
    add sp, strict byte 00006h                ; 83 c4 06
    jmp near 03104h                           ; e9 9e fe
    movzx dx, byte [bp-004h]                  ; 0f b6 56 fc
    movzx ax, byte [bp-00ah]                  ; 0f b6 46 f6
    call 02afch                               ; e8 8b f8
    and byte [bp+020h], 0feh                  ; 80 66 20 fe
    mov byte [bp+01bh], 000h                  ; c6 46 1b 00
    jmp near 02ed1h                           ; e9 55 fc
    mov al, byte [bp+01ah]                    ; 8a 46 1a
    mov byte [bp-008h], al                    ; 88 46 f8
    mov dx, word [bp+018h]                    ; 8b 56 18
    shr dx, 008h                              ; c1 ea 08
    mov ax, word [bp+016h]                    ; 8b 46 16
    shr ax, 008h                              ; c1 e8 08
    mov byte [bp-002h], al                    ; 88 46 fe
    mov bl, byte [bp+012h]                    ; 8a 5e 12
    mov byte [bp-00ah], bl                    ; 88 5e f6
    cmp bl, 001h                              ; 80 fb 01
    jnbe short 032b0h                         ; 77 14
    cmp AL, strict byte 001h                  ; 3c 01
    jnbe short 032b0h                         ; 77 10
    cmp dl, 04fh                              ; 80 fa 4f
    jnbe short 032b0h                         ; 77 0b
    mov al, byte [bp-008h]                    ; 8a 46 f8
    test al, al                               ; 84 c0
    je short 032b0h                           ; 74 04
    cmp AL, strict byte 012h                  ; 3c 12
    jbe short 032c5h                          ; 76 15
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    xor ah, ah                                ; 30 e4
    or ah, 001h                               ; 80 cc 01
    mov word [bp+01ah], ax                    ; 89 46 1a
    mov ax, strict word 00001h                ; b8 01 00
    call 02ae7h                               ; e8 26 f8
    or byte [bp+020h], 001h                   ; 80 4e 20 01
    movzx ax, byte [bp-00ah]                  ; 0f b6 46 f6
    call 02dbbh                               ; e8 ef fa
    test ax, ax                               ; 85 c0
    je near 02e9fh                            ; 0f 84 cd fb
    movzx dx, byte [bp-00ah]                  ; 0f b6 56 f6
    mov ax, dx                                ; 89 d0
    call 02bdah                               ; e8 ff f8
    test ax, ax                               ; 85 c0
    jne short 032eah                          ; 75 0b
    mov ax, dx                                ; 89 d0
    call 02cc1h                               ; e8 dd f9
    test ax, ax                               ; 85 c0
    je near 02f75h                            ; 0f 84 8b fc
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    shr dx, 00ch                              ; c1 ea 0c
    mov ah, dl                                ; 88 d4
    mov cx, word [bp+00ah]                    ; 8b 4e 0a
    sal cx, 004h                              ; c1 e1 04
    mov bx, word [bp+014h]                    ; 8b 5e 14
    add bx, cx                                ; 01 cb
    cmp bx, cx                                ; 39 cb
    jnc short 03303h                          ; 73 02
    db  0feh, 0c4h
    ; inc ah                                    ; fe c4
    movzx cx, byte [bp-008h]                  ; 0f b6 4e f8
    sal cx, 002h                              ; c1 e1 02
    dec cx                                    ; 49
    mov dx, bx                                ; 89 da
    add dx, cx                                ; 01 ca
    cmp dx, bx                                ; 39 da
    jc near 02fb5h                            ; 0f 82 a0 fc
    mov AL, strict byte 006h                  ; b0 06
    mov dx, strict word 0000ah                ; ba 0a 00
    out DX, AL                                ; ee
    xor al, al                                ; 30 c0
    mov dx, strict word 0000ch                ; ba 0c 00
    out DX, AL                                ; ee
    mov al, bl                                ; 88 d8
    mov dx, strict word 00004h                ; ba 04 00
    out DX, AL                                ; ee
    shr bx, 008h                              ; c1 eb 08
    mov al, bl                                ; 88 d8
    out DX, AL                                ; ee
    xor al, bl                                ; 30 d8
    mov dx, strict word 0000ch                ; ba 0c 00
    out DX, AL                                ; ee
    mov al, cl                                ; 88 c8
    mov dx, strict word 00005h                ; ba 05 00
    out DX, AL                                ; ee
    shr cx, 008h                              ; c1 e9 08
    mov al, cl                                ; 88 c8
    out DX, AL                                ; ee
    mov AL, strict byte 04ah                  ; b0 4a
    mov dx, strict word 0000bh                ; ba 0b 00
    out DX, AL                                ; ee
    mov al, ah                                ; 88 e0
    mov dx, 00081h                            ; ba 81 00
    out DX, AL                                ; ee
    mov AL, strict byte 002h                  ; b0 02
    mov dx, strict word 0000ah                ; ba 0a 00
    out DX, AL                                ; ee
    movzx ax, byte [bp-00ah]                  ; 0f b6 46 f6
    call 02b4eh                               ; e8 f6 f7
    mov AL, strict byte 04dh                  ; b0 4d
    mov dx, 003f5h                            ; ba f5 03
    out DX, AL                                ; ee
    movzx dx, byte [bp-002h]                  ; 0f b6 56 fe
    sal dx, 002h                              ; c1 e2 02
    movzx ax, byte [bp-00ah]                  ; 0f b6 46 f6
    or ax, dx                                 ; 09 d0
    mov dx, 003f5h                            ; ba f5 03
    out DX, AL                                ; ee
    mov AL, strict byte 002h                  ; b0 02
    out DX, AL                                ; ee
    mov al, byte [bp-008h]                    ; 8a 46 f8
    out DX, AL                                ; ee
    xor al, al                                ; 30 c0
    out DX, AL                                ; ee
    mov AL, strict byte 0f6h                  ; b0 f6
    out DX, AL                                ; ee
    sti                                       ; fb
    mov dx, strict word 00040h                ; ba 40 00
    mov ax, dx                                ; 89 d0
    call 01600h                               ; e8 7b e2
    test al, al                               ; 84 c0
    jne short 0338fh                          ; 75 06
    call 02b26h                               ; e8 9a f7
    jmp near 02e9fh                           ; e9 10 fb
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 68 e2
    and AL, strict byte 080h                  ; 24 80
    test al, al                               ; 84 c0
    je short 0337dh                           ; 74 df
    cli                                       ; fa
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 58 e2
    and AL, strict byte 07fh                  ; 24 7f
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 0003eh                ; ba 3e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 58 e2
    mov dx, 003f4h                            ; ba f4 03
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    and AL, strict byte 0c0h                  ; 24 c0
    cmp AL, strict byte 0c0h                  ; 3c c0
    je short 033d0h                           ; 74 0e
    push 001d5h                               ; 68 d5 01
    push 00208h                               ; 68 08 02
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 1c e5
    add sp, strict byte 00006h                ; 83 c4 06
    xor si, si                                ; 31 f6
    jmp short 033d9h                          ; eb 05
    cmp si, strict byte 00007h                ; 83 fe 07
    jnl short 033f1h                          ; 7d 18
    mov dx, 003f5h                            ; ba f5 03
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov byte [bp+si-012h], al                 ; 88 42 ee
    movzx bx, al                              ; 0f b6 d8
    lea dx, [si+042h]                         ; 8d 54 42
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 20 e2
    inc si                                    ; 46
    jmp short 033d4h                          ; eb e3
    test byte [bp-012h], 0c0h                 ; f6 46 ee c0
    je short 0340dh                           ; 74 16
    test byte [bp-011h], 002h                 ; f6 46 ef 02
    jne near 03249h                           ; 0f 85 4a fe
    push 001d5h                               ; 68 d5 01
    push 0022ch                               ; 68 2c 02
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 df e4
    add sp, strict byte 00006h                ; 83 c4 06
    xor al, al                                ; 30 c0
    mov byte [bp+01bh], al                    ; 88 46 1b
    xor ah, ah                                ; 30 e4
    call 02ae7h                               ; e8 d0 f6
    movzx ax, byte [bp-00ah]                  ; 0f b6 46 f6
    xor dx, dx                                ; 31 d2
    call 02afch                               ; e8 dc f6
    jmp near 03113h                           ; e9 f0 fc
    mov byte [bp-00ah], ah                    ; 88 66 f6
    cmp ah, 001h                              ; 80 fc 01
    jbe short 0344bh                          ; 76 20
    xor ax, ax                                ; 31 c0
    mov word [bp+01ah], ax                    ; 89 46 1a
    mov word [bp+014h], ax                    ; 89 46 14
    mov word [bp+018h], ax                    ; 89 46 18
    mov word [bp+016h], ax                    ; 89 46 16
    mov word [bp+00ah], ax                    ; 89 46 0a
    mov word [bp+00ch], ax                    ; 89 46 0c
    movzx ax, cl                              ; 0f b6 c1
    mov word [bp+016h], ax                    ; 89 46 16
    mov word [bp+020h], dx                    ; 89 56 20
    jmp near 02ed1h                           ; e9 86 fa
    mov ax, strict word 00010h                ; b8 10 00
    call 0165ch                               ; e8 0b e2
    mov bl, al                                ; 88 c3
    xor cl, cl                                ; 30 c9
    test AL, strict byte 0f0h                 ; a8 f0
    je short 0345bh                           ; 74 02
    mov CL, strict byte 001h                  ; b1 01
    test bl, 00fh                             ; f6 c3 0f
    je short 03462h                           ; 74 02
    db  0feh, 0c1h
    ; inc cl                                    ; fe c1
    cmp byte [bp-00ah], 000h                  ; 80 7e f6 00
    jne short 0346dh                          ; 75 05
    shr bl, 004h                              ; c0 eb 04
    jmp short 03470h                          ; eb 03
    and bl, 00fh                              ; 80 e3 0f
    xor al, al                                ; 30 c0
    mov byte [bp+015h], al                    ; 88 46 15
    movzx si, bl                              ; 0f b6 f3
    mov word [bp+014h], si                    ; 89 76 14
    xor ah, ah                                ; 30 e4
    mov word [bp+01ah], ax                    ; 89 46 1a
    mov dx, word [bp+016h]                    ; 8b 56 16
    xor dl, dl                                ; 30 d2
    movzx ax, cl                              ; 0f b6 c1
    or dx, ax                                 ; 09 c2
    mov word [bp+016h], dx                    ; 89 56 16
    cmp bl, 008h                              ; 80 fb 08
    jnbe short 034ebh                         ; 77 59
    add si, si                                ; 01 f6
    mov ax, dx                                ; 89 d0
    xor ah, dh                                ; 30 f4
    mov bx, ax                                ; 89 c3
    or bh, 001h                               ; 80 cf 01
    jmp word [cs:si+02ddch]                   ; 2e ff a4 dc 2d
    mov word [bp+018h], strict word 00000h    ; c7 46 18 00 00
    mov byte [bp+017h], 000h                  ; c6 46 17 00
    jmp short 034f9h                          ; eb 4c
    mov word [bp+018h], 02709h                ; c7 46 18 09 27
    mov word [bp+016h], bx                    ; 89 5e 16
    jmp short 034f9h                          ; eb 42
    mov word [bp+018h], 04f0fh                ; c7 46 18 0f 4f
    jmp short 034b2h                          ; eb f4
    mov word [bp+018h], 04f09h                ; c7 46 18 09 4f
    jmp short 034b2h                          ; eb ed
    mov word [bp+018h], 04f12h                ; c7 46 18 12 4f
    jmp short 034b2h                          ; eb e6
    mov word [bp+018h], 04f24h                ; c7 46 18 24 4f
    jmp short 034b2h                          ; eb df
    mov word [bp+018h], 02708h                ; c7 46 18 08 27
    mov word [bp+016h], ax                    ; 89 46 16
    jmp short 034f9h                          ; eb 1c
    mov word [bp+018h], 02709h                ; c7 46 18 09 27
    jmp short 034d8h                          ; eb f4
    mov word [bp+018h], 02708h                ; c7 46 18 08 27
    jmp short 034b2h                          ; eb c7
    push 001d5h                               ; 68 d5 01
    push 0023dh                               ; 68 3d 02
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 f3 e3
    add sp, strict byte 00006h                ; 83 c4 06
    mov word [bp+00ah], 0f000h                ; c7 46 0a 00 f0
    mov word [bp+00ch], 0efc7h                ; c7 46 0c c7 ef
    jmp near 03113h                           ; e9 0d fc
    mov byte [bp-00ah], ah                    ; 88 66 f6
    cmp ah, 001h                              ; 80 fc 01
    jbe short 03514h                          ; 76 06
    mov word [bp+01ah], si                    ; 89 76 1a
    jmp near 03445h                           ; e9 31 ff
    mov ax, strict word 00010h                ; b8 10 00
    call 0165ch                               ; e8 42 e1
    cmp byte [bp-00ah], 000h                  ; 80 7e f6 00
    jne short 03527h                          ; 75 07
    mov bl, al                                ; 88 c3
    shr bl, 004h                              ; c0 eb 04
    jmp short 0352ch                          ; eb 05
    mov bl, al                                ; 88 c3
    and bl, 00fh                              ; 80 e3 0f
    and byte [bp+020h], 0feh                  ; 80 66 20 fe
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    xor ah, ah                                ; 30 e4
    test bl, bl                               ; 84 db
    je short 0353ch                           ; 74 03
    or ah, 001h                               ; 80 cc 01
    mov word [bp+01ah], ax                    ; 89 46 1a
    jmp near 02ed1h                           ; e9 8f f9
    cmp ah, 001h                              ; 80 fc 01
    jbe short 03553h                          ; 76 0c
    mov word [bp+01ah], si                    ; 89 76 1a
    mov ax, strict word 00001h                ; b8 01 00
    call 02ae7h                               ; e8 97 f5
    jmp near 03445h                           ; e9 f2 fe
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    xor ah, ah                                ; 30 e4
    or ah, 006h                               ; 80 cc 06
    mov word [bp+01ah], ax                    ; 89 46 1a
    mov ax, strict word 00006h                ; b8 06 00
    jmp near 02e7dh                           ; e9 19 f9
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 3f e3
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 001d5h                               ; 68 d5 01
    push 00252h                               ; 68 52 02
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 68 e3
    add sp, strict byte 00008h                ; 83 c4 08
    jmp near 02e6fh                           ; e9 e8 f8
_cdemu_init:                                 ; 0xf3587 LB 0x16
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 89 e0
    xor bx, bx                                ; 31 db
    mov dx, 00322h                            ; ba 22 03
    call 0160eh                               ; e8 73 e0
    pop bp                                    ; 5d
    retn                                      ; c3
_cdemu_isactive:                             ; 0xf359d LB 0x14
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 73 e0
    mov dx, 00322h                            ; ba 22 03
    call 01600h                               ; e8 51 e0
    pop bp                                    ; 5d
    retn                                      ; c3
_cdemu_emulated_drive:                       ; 0xf35b1 LB 0x14
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 5f e0
    mov dx, 00324h                            ; ba 24 03
    call 01600h                               ; e8 3d e0
    pop bp                                    ; 5d
    retn                                      ; c3
_int13_eltorito:                             ; 0xf35c5 LB 0x186
    push si                                   ; 56
    push di                                   ; 57
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 49 e0
    mov si, 00322h                            ; be 22 03
    mov di, ax                                ; 89 c7
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    shr ax, 008h                              ; c1 e8 08
    cmp ax, strict word 0004bh                ; 3d 4b 00
    jc short 035edh                           ; 72 0a
    jbe short 03613h                          ; 76 2e
    cmp ax, strict word 0004dh                ; 3d 4d 00
    jbe short 035f4h                          ; 76 0a
    jmp near 0370fh                           ; e9 22 01
    cmp ax, strict word 0004ah                ; 3d 4a 00
    jne near 0370fh                           ; 0f 85 1b 01
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 af e2
    push word [bp+01ah]                       ; ff 76 1a
    push 0026ch                               ; 68 6c 02
    push 0027bh                               ; 68 7b 02
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 dc e2
    add sp, strict byte 00008h                ; 83 c4 08
    jmp near 0372ah                           ; e9 17 01
    mov dx, word [bp+00eh]                    ; 8b 56 0e
    mov ax, word [bp+008h]                    ; 8b 46 08
    mov bx, strict word 00013h                ; bb 13 00
    call 0160eh                               ; e8 ef df
    mov es, di                                ; 8e c7
    movzx bx, byte [es:si+001h]               ; 26 0f b6 5c 01
    mov dx, word [bp+00eh]                    ; 8b 56 0e
    inc dx                                    ; 42
    mov ax, word [bp+008h]                    ; 8b 46 08
    call 0160eh                               ; e8 de df
    mov es, di                                ; 8e c7
    movzx bx, byte [es:si+002h]               ; 26 0f b6 5c 02
    mov dx, word [bp+00eh]                    ; 8b 56 0e
    inc dx                                    ; 42
    inc dx                                    ; 42
    mov ax, word [bp+008h]                    ; 8b 46 08
    call 0160eh                               ; e8 cc df
    mov es, di                                ; 8e c7
    movzx bx, byte [es:si+003h]               ; 26 0f b6 5c 03
    mov dx, word [bp+00eh]                    ; 8b 56 0e
    add dx, strict byte 00003h                ; 83 c2 03
    mov ax, word [bp+008h]                    ; 8b 46 08
    call 0160eh                               ; e8 b9 df
    mov es, di                                ; 8e c7
    mov bx, word [es:si+008h]                 ; 26 8b 5c 08
    mov cx, word [es:si+00ah]                 ; 26 8b 4c 0a
    mov dx, word [bp+00eh]                    ; 8b 56 0e
    add dx, strict byte 00004h                ; 83 c2 04
    mov ax, word [bp+008h]                    ; 8b 46 08
    call 0164ah                               ; e8 df df
    mov es, di                                ; 8e c7
    mov bx, word [es:si+004h]                 ; 26 8b 5c 04
    mov dx, word [bp+00eh]                    ; 8b 56 0e
    add dx, strict byte 00008h                ; 83 c2 08
    mov ax, word [bp+008h]                    ; 8b 46 08
    call 0162ah                               ; e8 ad df
    mov es, di                                ; 8e c7
    mov bx, word [es:si+006h]                 ; 26 8b 5c 06
    mov dx, word [bp+00eh]                    ; 8b 56 0e
    add dx, strict byte 0000ah                ; 83 c2 0a
    mov ax, word [bp+008h]                    ; 8b 46 08
    call 0162ah                               ; e8 9b df
    mov es, di                                ; 8e c7
    mov bx, word [es:si+00ch]                 ; 26 8b 5c 0c
    mov dx, word [bp+00eh]                    ; 8b 56 0e
    add dx, strict byte 0000ch                ; 83 c2 0c
    mov ax, word [bp+008h]                    ; 8b 46 08
    call 0162ah                               ; e8 89 df
    mov es, di                                ; 8e c7
    mov bx, word [es:si+00eh]                 ; 26 8b 5c 0e
    mov dx, word [bp+00eh]                    ; 8b 56 0e
    add dx, strict byte 0000eh                ; 83 c2 0e
    mov ax, word [bp+008h]                    ; 8b 46 08
    call 0162ah                               ; e8 77 df
    mov es, di                                ; 8e c7
    movzx bx, byte [es:si+012h]               ; 26 0f b6 5c 12
    mov dx, word [bp+00eh]                    ; 8b 56 0e
    add dx, strict byte 00010h                ; 83 c2 10
    mov ax, word [bp+008h]                    ; 8b 46 08
    call 0160eh                               ; e8 48 df
    mov es, di                                ; 8e c7
    movzx bx, byte [es:si+014h]               ; 26 0f b6 5c 14
    mov dx, word [bp+00eh]                    ; 8b 56 0e
    add dx, strict byte 00011h                ; 83 c2 11
    mov ax, word [bp+008h]                    ; 8b 46 08
    call 0160eh                               ; e8 35 df
    mov es, di                                ; 8e c7
    movzx bx, byte [es:si+010h]               ; 26 0f b6 5c 10
    mov dx, word [bp+00eh]                    ; 8b 56 0e
    add dx, strict byte 00012h                ; 83 c2 12
    mov ax, word [bp+008h]                    ; 8b 46 08
    call 0160eh                               ; e8 22 df
    test byte [bp+01ah], 0ffh                 ; f6 46 1a ff
    jne short 036f8h                          ; 75 06
    mov es, di                                ; 8e c7
    mov byte [es:si], 000h                    ; 26 c6 04 00
    mov byte [bp+01bh], 000h                  ; c6 46 1b 00
    xor bx, bx                                ; 31 db
    mov dx, strict word 00074h                ; ba 74 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 07 df
    and byte [bp+020h], 0feh                  ; 80 66 20 fe
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn                                      ; c3
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 94 e1
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 0026ch                               ; 68 6c 02
    push 002a3h                               ; 68 a3 02
    jmp near 03608h                           ; e9 de fe
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    xor ah, ah                                ; 30 e4
    or ah, 001h                               ; 80 cc 01
    mov word [bp+01ah], ax                    ; 89 46 1a
    mov bx, ax                                ; 89 c3
    shr bx, 008h                              ; c1 eb 08
    xor bh, bh                                ; 30 ff
    mov dx, strict word 00074h                ; ba 74 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 c9 de
    or byte [bp+020h], 001h                   ; 80 4e 20 01
    jmp short 0370bh                          ; eb c0
device_is_cdrom_:                            ; 0xf374b LB 0x32
    push bx                                   ; 53
    push dx                                   ; 52
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bl, al                                ; 88 c3
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 c1 de
    cmp bl, 010h                              ; 80 fb 10
    jc short 03764h                           ; 72 04
    xor ax, ax                                ; 31 c0
    jmp short 03779h                          ; eb 15
    xor bh, bh                                ; 30 ff
    imul bx, bx, strict byte 00018h           ; 6b db 18
    mov es, ax                                ; 8e c0
    add bx, 00122h                            ; 81 c3 22 01
    cmp byte [es:bx+01fh], 005h               ; 26 80 7f 1f 05
    jne short 03760h                          ; 75 ea
    mov ax, strict word 00001h                ; b8 01 00
    pop bp                                    ; 5d
    pop dx                                    ; 5a
    pop bx                                    ; 5b
    retn                                      ; c3
cdrom_boot_:                                 ; 0xf377d LB 0x42f
    push bx                                   ; 53
    push cx                                   ; 51
    push dx                                   ; 52
    push si                                   ; 56
    push di                                   ; 57
    enter 0081ah, 000h                        ; c8 1a 08 00
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 8d de
    mov word [bp-00ch], ax                    ; 89 46 f4
    mov si, 00322h                            ; be 22 03
    mov word [bp-00eh], ax                    ; 89 46 f2
    mov word [bp-008h], 00122h                ; c7 46 f8 22 01
    mov word [bp-006h], ax                    ; 89 46 fa
    mov byte [bp-004h], 000h                  ; c6 46 fc 00
    jmp short 037afh                          ; eb 09
    inc byte [bp-004h]                        ; fe 46 fc
    cmp byte [bp-004h], 010h                  ; 80 7e fc 10
    jnc short 037bah                          ; 73 0b
    movzx ax, byte [bp-004h]                  ; 0f b6 46 fc
    call 0374bh                               ; e8 95 ff
    test ax, ax                               ; 85 c0
    je short 037a6h                           ; 74 ec
    cmp byte [bp-004h], 010h                  ; 80 7e fc 10
    jc short 037c6h                           ; 72 06
    mov ax, strict word 00002h                ; b8 02 00
    jmp near 03b4ch                           ; e9 86 03
    mov cx, strict word 0000ch                ; b9 0c 00
    xor bx, bx                                ; 31 db
    mov dx, ss                                ; 8c d2
    lea ax, [bp-01ah]                         ; 8d 46 e6
    call 08cbah                               ; e8 e7 54
    mov word [bp-01ah], strict word 00028h    ; c7 46 e6 28 00
    mov ax, strict word 00011h                ; b8 11 00
    xor dx, dx                                ; 31 d2
    xchg ah, al                               ; 86 c4
    xchg dh, dl                               ; 86 d6
    xchg dx, ax                               ; 92
    mov word [bp-018h], ax                    ; 89 46 e8
    mov word [bp-016h], dx                    ; 89 56 ea
    mov ax, strict word 00001h                ; b8 01 00
    xchg ah, al                               ; 86 c4
    mov word [bp-013h], ax                    ; 89 46 ed
    les bx, [bp-008h]                         ; c4 5e f8
    db  066h, 026h, 0c7h, 047h, 00ah, 001h, 000h, 000h, 008h
    ; mov dword [es:bx+00ah], strict dword 008000001h ; 66 26 c7 47 0a 01 00 00 08
    mov byte [bp-002h], 000h                  ; c6 46 fe 00
    jmp short 0382dh                          ; eb 2b
    lea dx, [bp-0081ah]                       ; 8d 96 e6 f7
    push SS                                   ; 16
    push dx                                   ; 52
    push strict byte 00001h                   ; 6a 01
    push strict byte 00000h                   ; 6a 00
    push 00800h                               ; 68 00 08
    push strict byte 00000h                   ; 6a 00
    movzx ax, byte [bp-004h]                  ; 0f b6 46 fc
    mov cx, ss                                ; 8c d1
    lea bx, [bp-01ah]                         ; 8d 5e e6
    mov dx, strict word 0000ch                ; ba 0c 00
    call 02804h                               ; e8 e4 ef
    test ax, ax                               ; 85 c0
    je short 03853h                           ; 74 2f
    inc byte [bp-002h]                        ; fe 46 fe
    cmp byte [bp-002h], 004h                  ; 80 7e fe 04
    jnbe short 03853h                         ; 77 26
    cmp byte [bp-004h], 008h                  ; 80 7e fc 08
    jbe short 03802h                          ; 76 cf
    lea dx, [bp-0081ah]                       ; 8d 96 e6 f7
    push SS                                   ; 16
    push dx                                   ; 52
    push strict byte 00001h                   ; 6a 01
    push strict byte 00000h                   ; 6a 00
    push 00800h                               ; 68 00 08
    push strict byte 00000h                   ; 6a 00
    movzx ax, byte [bp-004h]                  ; 0f b6 46 fc
    mov cx, ss                                ; 8c d1
    lea bx, [bp-01ah]                         ; 8d 5e e6
    mov dx, strict word 0000ch                ; ba 0c 00
    call 07f60h                               ; e8 0f 47
    jmp short 03820h                          ; eb cd
    test ax, ax                               ; 85 c0
    je short 0385dh                           ; 74 06
    mov ax, strict word 00003h                ; b8 03 00
    jmp near 03b4ch                           ; e9 ef 02
    cmp byte [bp-0081ah], 000h                ; 80 be e6 f7 00
    je short 0386ah                           ; 74 06
    mov ax, strict word 00004h                ; b8 04 00
    jmp near 03b4ch                           ; e9 e2 02
    xor di, di                                ; 31 ff
    jmp short 03874h                          ; eb 06
    inc di                                    ; 47
    cmp di, strict byte 00005h                ; 83 ff 05
    jnc short 03884h                          ; 73 10
    mov al, byte [bp+di-00819h]               ; 8a 83 e7 f7
    cmp al, byte [di+00ce0h]                  ; 3a 85 e0 0c
    je short 0386eh                           ; 74 f0
    mov ax, strict word 00005h                ; b8 05 00
    jmp near 03b4ch                           ; e9 c8 02
    xor di, di                                ; 31 ff
    jmp short 0388eh                          ; eb 06
    inc di                                    ; 47
    cmp di, strict byte 00017h                ; 83 ff 17
    jnc short 0389eh                          ; 73 10
    mov al, byte [bp+di-00813h]               ; 8a 83 ed f7
    cmp al, byte [di+00ce6h]                  ; 3a 85 e6 0c
    je short 03888h                           ; 74 f0
    mov ax, strict word 00006h                ; b8 06 00
    jmp near 03b4ch                           ; e9 ae 02
    mov ax, word [bp-007d3h]                  ; 8b 86 2d f8
    mov dx, word [bp-007d1h]                  ; 8b 96 2f f8
    mov word [bp-01ah], strict word 00028h    ; c7 46 e6 28 00
    xchg ah, al                               ; 86 c4
    xchg dh, dl                               ; 86 d6
    xchg dx, ax                               ; 92
    mov word [bp-018h], ax                    ; 89 46 e8
    mov word [bp-016h], dx                    ; 89 56 ea
    mov ax, strict word 00001h                ; b8 01 00
    xchg ah, al                               ; 86 c4
    mov word [bp-013h], ax                    ; 89 46 ed
    cmp byte [bp-004h], 008h                  ; 80 7e fc 08
    jbe short 038e4h                          ; 76 20
    lea dx, [bp-0081ah]                       ; 8d 96 e6 f7
    push SS                                   ; 16
    push dx                                   ; 52
    push strict byte 00001h                   ; 6a 01
    push strict byte 00000h                   ; 6a 00
    push 00800h                               ; 68 00 08
    push strict byte 00000h                   ; 6a 00
    movzx ax, byte [bp-004h]                  ; 0f b6 46 fc
    mov cx, ss                                ; 8c d1
    lea bx, [bp-01ah]                         ; 8d 5e e6
    mov dx, strict word 0000ch                ; ba 0c 00
    call 07f60h                               ; e8 7e 46
    jmp short 03902h                          ; eb 1e
    lea dx, [bp-0081ah]                       ; 8d 96 e6 f7
    push SS                                   ; 16
    push dx                                   ; 52
    push strict byte 00001h                   ; 6a 01
    push strict byte 00000h                   ; 6a 00
    push 00800h                               ; 68 00 08
    push strict byte 00000h                   ; 6a 00
    movzx ax, byte [bp-004h]                  ; 0f b6 46 fc
    mov cx, ss                                ; 8c d1
    lea bx, [bp-01ah]                         ; 8d 5e e6
    mov dx, strict word 0000ch                ; ba 0c 00
    call 02804h                               ; e8 02 ef
    test ax, ax                               ; 85 c0
    je short 0390ch                           ; 74 06
    mov ax, strict word 00007h                ; b8 07 00
    jmp near 03b4ch                           ; e9 40 02
    cmp byte [bp-0081ah], 001h                ; 80 be e6 f7 01
    je short 03919h                           ; 74 06
    mov ax, strict word 00008h                ; b8 08 00
    jmp near 03b4ch                           ; e9 33 02
    cmp byte [bp-00819h], 000h                ; 80 be e7 f7 00
    je short 03926h                           ; 74 06
    mov ax, strict word 00009h                ; b8 09 00
    jmp near 03b4ch                           ; e9 26 02
    cmp byte [bp-007fch], 055h                ; 80 be 04 f8 55
    je short 03933h                           ; 74 06
    mov ax, strict word 0000ah                ; b8 0a 00
    jmp near 03b4ch                           ; e9 19 02
    cmp byte [bp-007fbh], 0aah                ; 80 be 05 f8 aa
    jne short 0392dh                          ; 75 f3
    cmp byte [bp-007fah], 088h                ; 80 be 06 f8 88
    je short 03947h                           ; 74 06
    mov ax, strict word 0000bh                ; b8 0b 00
    jmp near 03b4ch                           ; e9 05 02
    mov al, byte [bp-007f9h]                  ; 8a 86 07 f8
    mov es, [bp-00eh]                         ; 8e 46 f2
    mov byte [es:si+001h], al                 ; 26 88 44 01
    cmp byte [bp-007f9h], 000h                ; 80 be 07 f8 00
    jne short 03960h                          ; 75 07
    mov byte [es:si+002h], 0e0h               ; 26 c6 44 02 e0
    jmp short 03973h                          ; eb 13
    cmp byte [bp-007f9h], 004h                ; 80 be 07 f8 04
    jnc short 0396eh                          ; 73 07
    mov byte [es:si+002h], 000h               ; 26 c6 44 02 00
    jmp short 03973h                          ; eb 05
    mov byte [es:si+002h], 080h               ; 26 c6 44 02 80
    movzx di, byte [bp-004h]                  ; 0f b6 7e fc
    mov ax, di                                ; 89 f8
    cwd                                       ; 99
    db  02bh, 0c2h
    ; sub ax, dx                                ; 2b c2
    sar ax, 1                                 ; d1 f8
    mov es, [bp-00eh]                         ; 8e 46 f2
    mov byte [es:si+003h], al                 ; 26 88 44 03
    mov ax, di                                ; 89 f8
    cwd                                       ; 99
    mov bx, strict word 00002h                ; bb 02 00
    idiv bx                                   ; f7 fb
    mov word [es:si+004h], dx                 ; 26 89 54 04
    mov di, word [bp-007f8h]                  ; 8b be 08 f8
    test di, di                               ; 85 ff
    jne short 0399ch                          ; 75 03
    mov di, 007c0h                            ; bf c0 07
    mov es, [bp-00eh]                         ; 8e 46 f2
    mov word [es:si+00ch], di                 ; 26 89 7c 0c
    mov word [es:si+006h], strict word 00000h ; 26 c7 44 06 00 00
    mov ax, word [bp-007f4h]                  ; 8b 86 0c f8
    mov word [bp-00ah], ax                    ; 89 46 f6
    mov word [es:si+00eh], ax                 ; 26 89 44 0e
    mov ax, word [bp-007f2h]                  ; 8b 86 0e f8
    mov dx, word [bp-007f0h]                  ; 8b 96 10 f8
    mov word [es:si+008h], ax                 ; 26 89 44 08
    mov word [es:si+00ah], dx                 ; 26 89 54 0a
    mov word [bp-01ah], strict word 00028h    ; c7 46 e6 28 00
    xchg ah, al                               ; 86 c4
    xchg dh, dl                               ; 86 d6
    xchg dx, ax                               ; 92
    mov word [bp-018h], ax                    ; 89 46 e8
    mov word [bp-016h], dx                    ; 89 56 ea
    mov dx, word [bp-00ah]                    ; 8b 56 f6
    dec dx                                    ; 4a
    shr dx, 002h                              ; c1 ea 02
    inc dx                                    ; 42
    mov ax, dx                                ; 89 d0
    xchg ah, al                               ; 86 c4
    mov word [bp-013h], ax                    ; 89 46 ed
    les bx, [bp-008h]                         ; c4 5e f8
    mov word [es:bx+00ah], dx                 ; 26 89 57 0a
    mov word [es:bx+00ch], 00200h             ; 26 c7 47 0c 00 02
    mov ax, word [bp-00ah]                    ; 8b 46 f6
    sal ax, 009h                              ; c1 e0 09
    and ah, 007h                              ; 80 e4 07
    mov dx, 00800h                            ; ba 00 08
    sub dx, ax                                ; 29 c2
    mov word [es:bx+01ch], dx                 ; 26 89 57 1c
    cmp byte [bp-004h], 008h                  ; 80 7e fc 08
    jbe short 03a2fh                          ; 76 27
    push di                                   ; 57
    push dword 000000001h                     ; 66 6a 01
    mov ax, word [bp-00ah]                    ; 8b 46 f6
    xor dx, dx                                ; 31 d2
    mov cx, strict word 00009h                ; b9 09 00
    sal ax, 1                                 ; d1 e0
    rcl dx, 1                                 ; d1 d2
    loop 03a14h                               ; e2 fa
    push dx                                   ; 52
    push ax                                   ; 50
    push strict byte 00000h                   ; 6a 00
    movzx ax, byte [bp-004h]                  ; 0f b6 46 fc
    mov cx, ss                                ; 8c d1
    lea bx, [bp-01ah]                         ; 8d 5e e6
    mov dx, strict word 0000ch                ; ba 0c 00
    call 07f60h                               ; e8 33 45
    jmp short 03a54h                          ; eb 25
    push di                                   ; 57
    push dword 000000001h                     ; 66 6a 01
    mov ax, word [bp-00ah]                    ; 8b 46 f6
    xor dx, dx                                ; 31 d2
    mov cx, strict word 00009h                ; b9 09 00
    sal ax, 1                                 ; d1 e0
    rcl dx, 1                                 ; d1 d2
    loop 03a3bh                               ; e2 fa
    push dx                                   ; 52
    push ax                                   ; 50
    push strict byte 00000h                   ; 6a 00
    movzx ax, byte [bp-004h]                  ; 0f b6 46 fc
    mov cx, ss                                ; 8c d1
    lea bx, [bp-01ah]                         ; 8d 5e e6
    mov dx, strict word 0000ch                ; ba 0c 00
    call 02804h                               ; e8 b0 ed
    les bx, [bp-008h]                         ; c4 5e f8
    mov word [es:bx+01ch], strict word 00000h ; 26 c7 47 1c 00 00
    test ax, ax                               ; 85 c0
    je short 03a67h                           ; 74 06
    mov ax, strict word 0000ch                ; b8 0c 00
    jmp near 03b4ch                           ; e9 e5 00
    mov es, [bp-00eh]                         ; 8e 46 f2
    mov al, byte [es:si+001h]                 ; 26 8a 44 01
    cmp AL, strict byte 002h                  ; 3c 02
    jc short 03a7fh                           ; 72 0d
    jbe short 03a97h                          ; 76 23
    cmp AL, strict byte 004h                  ; 3c 04
    je short 03aadh                           ; 74 35
    cmp AL, strict byte 003h                  ; 3c 03
    je short 03aa2h                           ; 74 26
    jmp near 03af5h                           ; e9 76 00
    cmp AL, strict byte 001h                  ; 3c 01
    jne short 03af5h                          ; 75 72
    mov es, [bp-00eh]                         ; 8e 46 f2
    db  066h, 026h, 0c7h, 044h, 012h, 050h, 000h, 00fh, 000h
    ; mov dword [es:si+012h], strict dword 0000f0050h ; 66 26 c7 44 12 50 00 0f 00
    mov word [es:si+010h], strict word 00002h ; 26 c7 44 10 02 00
    jmp short 03af5h                          ; eb 5e
    db  066h, 026h, 0c7h, 044h, 012h, 050h, 000h, 012h, 000h
    ; mov dword [es:si+012h], strict dword 000120050h ; 66 26 c7 44 12 50 00 12 00
    jmp short 03a8fh                          ; eb ed
    db  066h, 026h, 0c7h, 044h, 012h, 050h, 000h, 024h, 000h
    ; mov dword [es:si+012h], strict dword 000240050h ; 66 26 c7 44 12 50 00 24 00
    jmp short 03a8fh                          ; eb e2
    mov dx, 001c4h                            ; ba c4 01
    mov ax, di                                ; 89 f8
    call 01600h                               ; e8 4b db
    and AL, strict byte 03fh                  ; 24 3f
    xor ah, ah                                ; 30 e4
    mov es, [bp-00eh]                         ; 8e 46 f2
    mov word [es:si+014h], ax                 ; 26 89 44 14
    mov dx, 001c4h                            ; ba c4 01
    mov ax, di                                ; 89 f8
    call 01600h                               ; e8 38 db
    xor ah, ah                                ; 30 e4
    mov bx, ax                                ; 89 c3
    sal bx, 002h                              ; c1 e3 02
    mov dx, 001c5h                            ; ba c5 01
    mov ax, di                                ; 89 f8
    call 01600h                               ; e8 29 db
    xor ah, ah                                ; 30 e4
    add ax, bx                                ; 01 d8
    inc ax                                    ; 40
    mov es, [bp-00eh]                         ; 8e 46 f2
    mov word [es:si+012h], ax                 ; 26 89 44 12
    mov dx, 001c3h                            ; ba c3 01
    mov ax, di                                ; 89 f8
    call 01600h                               ; e8 15 db
    xor ah, ah                                ; 30 e4
    inc ax                                    ; 40
    mov es, [bp-00eh]                         ; 8e 46 f2
    mov word [es:si+010h], ax                 ; 26 89 44 10
    mov es, [bp-00eh]                         ; 8e 46 f2
    cmp byte [es:si+001h], 000h               ; 26 80 7c 01 00
    je short 03b33h                           ; 74 34
    cmp byte [es:si+002h], 000h               ; 26 80 7c 02 00
    jne short 03b1ch                          ; 75 16
    mov dx, strict word 00010h                ; ba 10 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 f1 da
    or AL, strict byte 041h                   ; 0c 41
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 00010h                ; ba 10 00
    mov ax, strict word 00040h                ; b8 40 00
    jmp short 03b30h                          ; eb 14
    mov dx, 002c0h                            ; ba c0 02
    mov ax, word [bp-00ch]                    ; 8b 46 f4
    call 01600h                               ; e8 db da
    db  0feh, 0c0h
    ; inc al                                    ; fe c0
    movzx bx, al                              ; 0f b6 d8
    mov dx, 002c0h                            ; ba c0 02
    mov ax, word [bp-00ch]                    ; 8b 46 f4
    call 0160eh                               ; e8 db da
    mov es, [bp-00eh]                         ; 8e 46 f2
    cmp byte [es:si+001h], 000h               ; 26 80 7c 01 00
    je short 03b41h                           ; 74 04
    mov byte [es:si], 001h                    ; 26 c6 04 01
    mov es, [bp-00eh]                         ; 8e 46 f2
    movzx ax, byte [es:si+002h]               ; 26 0f b6 44 02
    sal ax, 008h                              ; c1 e0 08
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop dx                                    ; 5a
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
    push ax                                   ; 50
    dec si                                    ; 4e
    dec cx                                    ; 49
    dec ax                                    ; 48
    inc di                                    ; 47
    inc si                                    ; 46
    inc bp                                    ; 45
    inc sp                                    ; 44
    inc bx                                    ; 43
    inc dx                                    ; 42
    inc cx                                    ; 41
    sbb byte [01415h], dl                     ; 18 16 15 14
    adc word [bx+si], dx                      ; 11 10
    or ax, 00b0ch                             ; 0d 0c 0b
    or cl, byte [bx+di]                       ; 0a 09
    or byte [di], al                          ; 08 05
    add AL, strict byte 003h                  ; 04 03
    add al, byte [bx+di]                      ; 02 01
    add byte [bp+si], bh                      ; 00 3a
    aas                                       ; 3f
    sbb di, word [bx]                         ; 1b 3f
    pop bx                                    ; 5b
    cmp AL, strict byte 085h                  ; 3c 85
    cmp AL, strict byte 050h                  ; 3c 50
    cmp AL, strict byte 085h                  ; 3c 85
    cmp AL, strict byte 050h                  ; 3c 50
    cmp AL, strict byte 077h                  ; 3c 77
    sbb di, word [ds:bx]                      ; 3e 1b 3f
    cmp bh, byte [bx]                         ; 3a 3f
    cmp bh, byte [bx]                         ; 3a 3f
    sbb di, word [bx]                         ; 1b 3f
    sbb di, word [bx]                         ; 1b 3f
    sbb di, word [bx]                         ; 1b 3f
    sbb di, word [bx]                         ; 1b 3f
    sbb di, word [bx]                         ; 1b 3f
    xor bh, byte [bx]                         ; 32 3f
    sbb di, word [bx]                         ; 1b 3f
    cmp bh, byte [bx]                         ; 3a 3f
    cmp bh, byte [bx]                         ; 3a 3f
    cmp bh, byte [bx]                         ; 3a 3f
    cmp bh, byte [bx]                         ; 3a 3f
    cmp bh, byte [bx]                         ; 3a 3f
    cmp bh, byte [bx]                         ; 3a 3f
    cmp bh, byte [bx]                         ; 3a 3f
    cmp bh, byte [bx]                         ; 3a 3f
    cmp bh, byte [bx]                         ; 3a 3f
    cmp bh, byte [bx]                         ; 3a 3f
    cmp bh, byte [bx]                         ; 3a 3f
    cmp bh, byte [bx]                         ; 3a 3f
_int13_cdemu:                                ; 0xf3bac LB 0x429
    push si                                   ; 56
    push di                                   ; 57
    enter 0002ah, 000h                        ; c8 2a 00 00
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 61 da
    mov di, 00322h                            ; bf 22 03
    mov cx, ax                                ; 89 c1
    mov si, di                                ; 89 fe
    mov word [bp-004h], ax                    ; 89 46 fc
    mov word [bp-008h], 00122h                ; c7 46 f8 22 01
    mov word [bp-006h], ax                    ; 89 46 fa
    mov es, ax                                ; 8e c0
    mov al, byte [es:di+003h]                 ; 26 8a 45 03
    add al, al                                ; 00 c0
    mov byte [bp-002h], al                    ; 88 46 fe
    mov al, byte [es:di+004h]                 ; 26 8a 45 04
    add byte [bp-002h], al                    ; 00 46 fe
    xor bx, bx                                ; 31 db
    mov dx, strict word 00074h                ; ba 74 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 24 da
    mov es, cx                                ; 8e c1
    cmp byte [es:di], 000h                    ; 26 80 3d 00
    je short 03c00h                           ; 74 0e
    movzx ax, byte [es:di+002h]               ; 26 0f b6 45 02
    mov dx, word [bp+016h]                    ; 8b 56 16
    xor dh, dh                                ; 30 f6
    cmp ax, dx                                ; 39 d0
    je short 03c29h                           ; 74 29
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 a3 dc
    mov ax, word [bp+016h]                    ; 8b 46 16
    xor ah, ah                                ; 30 e4
    push ax                                   ; 50
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 002bch                               ; 68 bc 02
    push 002c8h                               ; 68 c8 02
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 c6 dc
    add sp, strict byte 0000ah                ; 83 c4 0a
    jmp near 03f5ah                           ; e9 31 03
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    shr ax, 008h                              ; c1 e8 08
    cmp ax, strict word 00050h                ; 3d 50 00
    jnbe near 03f3ah                          ; 0f 87 04 03
    push CS                                   ; 0e
    pop ES                                    ; 07
    mov cx, strict word 0001eh                ; b9 1e 00
    mov di, 03b53h                            ; bf 53 3b
    repne scasb                               ; f2 ae
    sal cx, 1                                 ; d1 e1
    mov di, cx                                ; 89 cf
    mov ax, word [cs:di+03b70h]               ; 2e 8b 85 70 3b
    mov bx, word [bp+01ah]                    ; 8b 5e 1a
    xor bh, bh                                ; 30 ff
    jmp ax                                    ; ff e0
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    xor ah, ah                                ; 30 e4
    or ah, 003h                               ; 80 cc 03
    jmp near 03f62h                           ; e9 07 03
    mov dx, strict word 00074h                ; ba 74 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 9c d9
    mov cl, al                                ; 88 c1
    movzx ax, cl                              ; 0f b6 c1
    sal ax, 008h                              ; c1 e0 08
    or bx, ax                                 ; 09 c3
    mov word [bp+01ah], bx                    ; 89 5e 1a
    xor bx, bx                                ; 31 db
    mov dx, strict word 00074h                ; ba 74 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 92 d9
    test cl, cl                               ; 84 c9
    je near 03f1fh                            ; 0f 84 9d 02
    jmp near 03f76h                           ; e9 f1 02
    mov es, [bp-004h]                         ; 8e 46 fc
    mov di, word [es:si+014h]                 ; 26 8b 7c 14
    mov dx, word [es:si+012h]                 ; 26 8b 54 12
    mov bx, word [es:si+010h]                 ; 26 8b 5c 10
    mov ax, word [es:si+008h]                 ; 26 8b 44 08
    mov word [bp-014h], ax                    ; 89 46 ec
    mov ax, word [es:si+00ah]                 ; 26 8b 44 0a
    mov word [bp-012h], ax                    ; 89 46 ee
    mov ax, word [bp+018h]                    ; 8b 46 18
    and ax, strict word 0003fh                ; 25 3f 00
    mov word [bp-010h], ax                    ; 89 46 f0
    mov ax, word [bp+018h]                    ; 8b 46 18
    xor ah, ah                                ; 30 e4
    and AL, strict byte 0c0h                  ; 24 c0
    sal ax, 002h                              ; c1 e0 02
    mov cx, word [bp+018h]                    ; 8b 4e 18
    shr cx, 008h                              ; c1 e9 08
    or ax, cx                                 ; 09 c8
    mov si, word [bp+016h]                    ; 8b 76 16
    shr si, 008h                              ; c1 ee 08
    mov cx, word [bp+01ah]                    ; 8b 4e 1a
    xor ch, ch                                ; 30 ed
    mov word [bp-00ah], cx                    ; 89 4e f6
    test cx, cx                               ; 85 c9
    je near 03f1bh                            ; 0f 84 4a 02
    cmp di, word [bp-010h]                    ; 3b 7e f0
    jc near 03f5ah                            ; 0f 82 82 02
    cmp ax, dx                                ; 39 d0
    jnc near 03f5ah                           ; 0f 83 7c 02
    cmp si, bx                                ; 39 de
    jnc near 03f5ah                           ; 0f 83 76 02
    mov dx, word [bp+01ah]                    ; 8b 56 1a
    shr dx, 008h                              ; c1 ea 08
    cmp dx, strict byte 00004h                ; 83 fa 04
    je near 03f1bh                            ; 0f 84 2a 02
    mov dx, word [bp+014h]                    ; 8b 56 14
    shr dx, 004h                              ; c1 ea 04
    mov cx, word [bp+00ah]                    ; 8b 4e 0a
    add cx, dx                                ; 01 d1
    mov word [bp-00eh], cx                    ; 89 4e f2
    mov dx, word [bp+014h]                    ; 8b 56 14
    and dx, strict byte 0000fh                ; 83 e2 0f
    mov word [bp-00ch], dx                    ; 89 56 f4
    xor dl, dl                                ; 30 d2
    xor cx, cx                                ; 31 c9
    call 08c89h                               ; e8 7a 4f
    xor bx, bx                                ; 31 db
    add ax, si                                ; 01 f0
    adc dx, bx                                ; 11 da
    mov bx, di                                ; 89 fb
    xor cx, cx                                ; 31 c9
    call 08c89h                               ; e8 6d 4f
    mov bx, word [bp-010h]                    ; 8b 5e f0
    dec bx                                    ; 4b
    xor cx, cx                                ; 31 c9
    add ax, bx                                ; 01 d8
    adc dx, cx                                ; 11 ca
    mov bx, word [bp+01ah]                    ; 8b 5e 1a
    xor bl, bl                                ; 30 db
    mov cx, word [bp-00ah]                    ; 8b 4e f6
    or cx, bx                                 ; 09 d9
    mov word [bp+01ah], cx                    ; 89 4e 1a
    mov si, ax                                ; 89 c6
    mov di, dx                                ; 89 d7
    shr di, 1                                 ; d1 ef
    rcr si, 1                                 ; d1 de
    shr di, 1                                 ; d1 ef
    rcr si, 1                                 ; d1 de
    mov word [bp-01ah], di                    ; 89 7e e6
    mov di, ax                                ; 89 c7
    and di, strict byte 00003h                ; 83 e7 03
    xor bh, bh                                ; 30 ff
    add ax, word [bp-00ah]                    ; 03 46 f6
    adc dx, bx                                ; 11 da
    mov bx, ax                                ; 89 c3
    add bx, strict byte 0ffffh                ; 83 c3 ff
    mov ax, dx                                ; 89 d0
    adc ax, strict word 0ffffh                ; 15 ff ff
    mov word [bp-01eh], bx                    ; 89 5e e2
    mov word [bp-01ch], ax                    ; 89 46 e4
    shr word [bp-01ch], 1                     ; d1 6e e4
    rcr word [bp-01eh], 1                     ; d1 5e e2
    shr word [bp-01ch], 1                     ; d1 6e e4
    rcr word [bp-01eh], 1                     ; d1 5e e2
    mov cx, strict word 0000ch                ; b9 0c 00
    xor bx, bx                                ; 31 db
    mov dx, ss                                ; 8c d2
    lea ax, [bp-02ah]                         ; 8d 46 d6
    call 08cbah                               ; e8 43 4f
    mov word [bp-02ah], strict word 00028h    ; c7 46 d6 28 00
    mov ax, word [bp-014h]                    ; 8b 46 ec
    add ax, si                                ; 01 f0
    mov dx, word [bp-012h]                    ; 8b 56 ee
    adc dx, word [bp-01ah]                    ; 13 56 e6
    xchg ah, al                               ; 86 c4
    xchg dh, dl                               ; 86 d6
    xchg dx, ax                               ; 92
    mov word [bp-028h], ax                    ; 89 46 d8
    mov word [bp-026h], dx                    ; 89 56 da
    mov dx, word [bp-01eh]                    ; 8b 56 e2
    sub dx, si                                ; 29 f2
    inc dx                                    ; 42
    mov ax, dx                                ; 89 d0
    xchg ah, al                               ; 86 c4
    mov word [bp-023h], ax                    ; 89 46 dd
    les bx, [bp-008h]                         ; c4 5e f8
    mov word [es:bx+00ah], dx                 ; 26 89 57 0a
    mov word [es:bx+00ch], 00200h             ; 26 c7 47 0c 00 02
    mov ax, di                                ; 89 f8
    sal ax, 009h                              ; c1 e0 09
    mov word [es:bx+01ah], ax                 ; 26 89 47 1a
    mov dx, word [bp-00ah]                    ; 8b 56 f6
    sal dx, 009h                              ; c1 e2 09
    and dh, 007h                              ; 80 e6 07
    mov bx, 00800h                            ; bb 00 08
    sub bx, dx                                ; 29 d3
    mov dx, bx                                ; 89 da
    mov bx, word [bp-008h]                    ; 8b 5e f8
    mov bx, word [es:bx+01ah]                 ; 26 8b 5f 1a
    sub dx, bx                                ; 29 da
    mov bx, word [bp-008h]                    ; 8b 5e f8
    mov word [es:bx+01ch], dx                 ; 26 89 57 1c
    cmp byte [bp-002h], 008h                  ; 80 7e fe 08
    jbe short 03e05h                          ; 76 2a
    push word [bp-00eh]                       ; ff 76 f2
    push word [bp-00ch]                       ; ff 76 f4
    push strict byte 00001h                   ; 6a 01
    mov si, word [bp-00ah]                    ; 8b 76 f6
    xor di, di                                ; 31 ff
    mov cx, strict word 00009h                ; b9 09 00
    sal si, 1                                 ; d1 e6
    rcl di, 1                                 ; d1 d7
    loop 03debh                               ; e2 fa
    push di                                   ; 57
    push si                                   ; 56
    push ax                                   ; 50
    movzx ax, byte [bp-002h]                  ; 0f b6 46 fe
    mov cx, ss                                ; 8c d1
    lea bx, [bp-02ah]                         ; 8d 5e d6
    mov dx, strict word 0000ch                ; ba 0c 00
    call 07f60h                               ; e8 5d 41
    jmp short 03e2dh                          ; eb 28
    push word [bp-00eh]                       ; ff 76 f2
    push word [bp-00ch]                       ; ff 76 f4
    push strict byte 00001h                   ; 6a 01
    mov si, word [bp-00ah]                    ; 8b 76 f6
    xor di, di                                ; 31 ff
    mov cx, strict word 00009h                ; b9 09 00
    sal si, 1                                 ; d1 e6
    rcl di, 1                                 ; d1 d7
    loop 03e15h                               ; e2 fa
    push di                                   ; 57
    push si                                   ; 56
    push ax                                   ; 50
    movzx ax, byte [bp-002h]                  ; 0f b6 46 fe
    mov cx, ss                                ; 8c d1
    lea bx, [bp-02ah]                         ; 8d 5e d6
    mov dx, strict word 0000ch                ; ba 0c 00
    call 02804h                               ; e8 d7 e9
    mov dl, al                                ; 88 c2
    les bx, [bp-008h]                         ; c4 5e f8
    db  066h, 026h, 0c7h, 047h, 01ah, 000h, 000h, 000h, 000h
    ; mov dword [es:bx+01ah], strict dword 000000000h ; 66 26 c7 47 1a 00 00 00 00
    test dl, dl                               ; 84 d2
    je near 03f1bh                            ; 0f 84 da 00
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 62 da
    movzx ax, dl                              ; 0f b6 c2
    push ax                                   ; 50
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 002bch                               ; 68 bc 02
    push 002feh                               ; 68 fe 02
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 87 da
    add sp, strict byte 0000ah                ; 83 c4 0a
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    xor ah, ah                                ; 30 e4
    or ah, 002h                               ; 80 cc 02
    mov word [bp+01ah], ax                    ; 89 46 1a
    mov byte [bp+01ah], 000h                  ; c6 46 1a 00
    jmp near 03f65h                           ; e9 ee 00
    mov es, [bp-004h]                         ; 8e 46 fc
    mov di, word [es:si+014h]                 ; 26 8b 7c 14
    mov dx, word [es:si+012h]                 ; 26 8b 54 12
    dec dx                                    ; 4a
    mov bx, word [es:si+010h]                 ; 26 8b 5c 10
    dec bx                                    ; 4b
    mov byte [bp+01ah], 000h                  ; c6 46 1a 00
    mov ax, word [bp+014h]                    ; 8b 46 14
    xor al, al                                ; 30 c0
    mov cx, word [bp+018h]                    ; 8b 4e 18
    xor ch, ch                                ; 30 ed
    mov word [bp-018h], cx                    ; 89 4e e8
    mov cx, dx                                ; 89 d1
    xor ch, dh                                ; 30 f5
    sal cx, 008h                              ; c1 e1 08
    mov word [bp-016h], cx                    ; 89 4e ea
    mov cx, word [bp-018h]                    ; 8b 4e e8
    or cx, word [bp-016h]                     ; 0b 4e ea
    mov word [bp+018h], cx                    ; 89 4e 18
    shr dx, 002h                              ; c1 ea 02
    xor dh, dh                                ; 30 f6
    and dl, 0c0h                              ; 80 e2 c0
    mov word [bp-016h], dx                    ; 89 56 ea
    mov dx, di                                ; 89 fa
    xor dh, dh                                ; 30 f6
    and dl, 03fh                              ; 80 e2 3f
    or dx, word [bp-016h]                     ; 0b 56 ea
    xor cl, cl                                ; 30 c9
    or cx, dx                                 ; 09 d1
    mov word [bp+018h], cx                    ; 89 4e 18
    mov dx, word [bp+016h]                    ; 8b 56 16
    xor dh, dh                                ; 30 f6
    sal bx, 008h                              ; c1 e3 08
    or dx, bx                                 ; 09 da
    mov word [bp+016h], dx                    ; 89 56 16
    xor dl, dl                                ; 30 d2
    or dl, 002h                               ; 80 ca 02
    mov word [bp+016h], dx                    ; 89 56 16
    mov dl, byte [es:si+001h]                 ; 26 8a 54 01
    mov word [bp+014h], ax                    ; 89 46 14
    cmp dl, 003h                              ; 80 fa 03
    je short 03f03h                           ; 74 1a
    cmp dl, 002h                              ; 80 fa 02
    je short 03effh                           ; 74 11
    cmp dl, 001h                              ; 80 fa 01
    jne short 03f07h                          ; 75 14
    mov ax, word [bp+014h]                    ; 8b 46 14
    xor al, al                                ; 30 c0
    or AL, strict byte 002h                   ; 0c 02
    mov word [bp+014h], ax                    ; 89 46 14
    jmp short 03f07h                          ; eb 08
    or AL, strict byte 004h                   ; 0c 04
    jmp short 03efah                          ; eb f7
    or AL, strict byte 005h                   ; 0c 05
    jmp short 03efah                          ; eb f3
    mov es, [bp-004h]                         ; 8e 46 fc
    cmp byte [es:si+001h], 004h               ; 26 80 7c 01 04
    jnc short 03f1bh                          ; 73 0a
    mov word [bp+00ch], 0efc7h                ; c7 46 0c c7 ef
    mov word [bp+00ah], 0f000h                ; c7 46 0a 00 f0
    mov byte [bp+01bh], 000h                  ; c6 46 1b 00
    xor bx, bx                                ; 31 db
    mov dx, strict word 00074h                ; ba 74 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 e4 d6
    and byte [bp+020h], 0feh                  ; 80 66 20 fe
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn                                      ; c3
    or bh, 003h                               ; 80 cf 03
    mov word [bp+01ah], bx                    ; 89 5e 1a
    jmp short 03f1fh                          ; eb e5
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 69 d9
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 002bch                               ; 68 bc 02
    push 0031fh                               ; 68 1f 03
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 92 d9
    add sp, strict byte 00008h                ; 83 c4 08
    mov ax, word [bp+01ah]                    ; 8b 46 1a
    xor ah, ah                                ; 30 e4
    or ah, 001h                               ; 80 cc 01
    mov word [bp+01ah], ax                    ; 89 46 1a
    mov bx, word [bp+01ah]                    ; 8b 5e 1a
    shr bx, 008h                              ; c1 eb 08
    xor bh, bh                                ; 30 ff
    mov dx, strict word 00074h                ; ba 74 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 98 d6
    or byte [bp+020h], 001h                   ; 80 4e 20 01
    jmp short 03f2eh                          ; eb b2
    push ax                                   ; 50
    dec si                                    ; 4e
    dec cx                                    ; 49
    dec ax                                    ; 48
    inc di                                    ; 47
    inc si                                    ; 46
    inc bp                                    ; 45
    inc sp                                    ; 44
    inc bx                                    ; 43
    inc dx                                    ; 42
    inc cx                                    ; 41
    sbb byte [01415h], dl                     ; 18 16 15 14
    adc word [bx+si], dx                      ; 11 10
    or ax, 00b0ch                             ; 0d 0c 0b
    or cl, byte [bx+di]                       ; 0a 09
    or byte [di], al                          ; 08 05
    add AL, strict byte 003h                  ; 04 03
    add al, byte [bx+di]                      ; 02 01
    add byte [di], ah                         ; 00 25
    inc bp                                    ; 45
    mov word [08042h], ax                     ; a3 42 80
    inc ax                                    ; 40
    and ax, 07545h                            ; 25 45 75
    inc ax                                    ; 40
    and ax, 07545h                            ; 25 45 75
    inc ax                                    ; 40
    and ax, 0a345h                            ; 25 45 a3
    inc dx                                    ; 42
    and ax, 02545h                            ; 25 45 25
    inc bp                                    ; 45
    mov word [0a342h], ax                     ; a3 42 a3
    inc dx                                    ; 42
    mov word [0a342h], ax                     ; a3 42 a3
    inc dx                                    ; 42
    mov word [0aa42h], ax                     ; a3 42 aa
    inc ax                                    ; 40
    mov word [02542h], ax                     ; a3 42 25
    inc bp                                    ; 45
    mov BL, strict byte 040h                  ; b3 40
    mov byte [bx+si+075h], 040h               ; c6 40 75 40
    mov byte [bx+si+014h], 042h               ; c6 40 14 42
    mov dx, 0c642h                            ; ba 42 c6
    inc ax                                    ; 40
    in ax, 042h                               ; e5 42
    fiadd word [si-01ah]                      ; de 44 e6
    inc sp                                    ; 44
    db  025h
    inc bp                                    ; 45
_int13_cdrom:                                ; 0xf3fd5 LB 0x56d
    push si                                   ; 56
    push di                                   ; 57
    enter 00028h, 000h                        ; c8 28 00 00
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 38 d6
    mov word [bp-018h], ax                    ; 89 46 e8
    mov si, 00122h                            ; be 22 01
    mov word [bp-00ah], ax                    ; 89 46 f6
    xor bx, bx                                ; 31 db
    mov dx, strict word 00074h                ; ba 74 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 16 d6
    mov ax, word [bp+014h]                    ; 8b 46 14
    xor ah, ah                                ; 30 e4
    cmp ax, 000e0h                            ; 3d e0 00
    jc short 04007h                           ; 72 05
    cmp ax, 000f0h                            ; 3d f0 00
    jc short 04025h                           ; 72 1e
    mov ax, word [bp+014h]                    ; 8b 46 14
    xor ah, ah                                ; 30 e4
    push ax                                   ; 50
    mov ax, word [bp+01ch]                    ; 8b 46 1c
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 0034fh                               ; 68 4f 03
    push 0035bh                               ; 68 5b 03
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 ca d8
    add sp, strict byte 0000ah                ; 83 c4 0a
    jmp near 04502h                           ; e9 dd 04
    mov ax, word [bp+014h]                    ; 8b 46 14
    xor ah, ah                                ; 30 e4
    mov es, [bp-00ah]                         ; 8e 46 f6
    mov bx, si                                ; 89 f3
    add bx, ax                                ; 01 c3
    mov dl, byte [es:bx+000d0h]               ; 26 8a 97 d0 00
    mov byte [bp-006h], dl                    ; 88 56 fa
    cmp dl, 010h                              ; 80 fa 10
    jc short 0404eh                           ; 72 10
    push ax                                   ; 50
    mov ax, word [bp+01ch]                    ; 8b 46 1c
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 0034fh                               ; 68 4f 03
    push 00386h                               ; 68 86 03
    jmp short 0401ah                          ; eb cc
    mov ax, word [bp+01ch]                    ; 8b 46 1c
    shr ax, 008h                              ; c1 e8 08
    cmp ax, strict word 00050h                ; 3d 50 00
    jnbe near 04525h                          ; 0f 87 ca 04
    push CS                                   ; 0e
    pop ES                                    ; 07
    mov cx, strict word 0001eh                ; b9 1e 00
    mov di, 03f7ch                            ; bf 7c 3f
    repne scasb                               ; f2 ae
    sal cx, 1                                 ; d1 e1
    mov di, cx                                ; 89 cf
    mov ax, word [cs:di+03f99h]               ; 2e 8b 85 99 3f
    mov bx, word [bp+01ch]                    ; 8b 5e 1c
    xor bh, bh                                ; 30 ff
    jmp ax                                    ; ff e0
    mov ax, word [bp+01ch]                    ; 8b 46 1c
    xor ah, ah                                ; 30 e4
    or ah, 003h                               ; 80 cc 03
    jmp near 0450ah                           ; e9 8a 04
    mov dx, strict word 00074h                ; ba 74 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 77 d5
    mov cl, al                                ; 88 c1
    movzx ax, cl                              ; 0f b6 c1
    sal ax, 008h                              ; c1 e0 08
    or bx, ax                                 ; 09 c3
    mov word [bp+01ch], bx                    ; 89 5e 1c
    xor bx, bx                                ; 31 db
    mov dx, strict word 00074h                ; ba 74 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 6d d5
    test cl, cl                               ; 84 c9
    je near 042a7h                            ; 0f 84 00 02
    jmp near 0451eh                           ; e9 74 04
    or bh, 002h                               ; 80 cf 02
    mov word [bp+01ch], bx                    ; 89 5e 1c
    jmp near 0450dh                           ; e9 5a 04
    mov word [bp+016h], 0aa55h                ; c7 46 16 55 aa
    or bh, 030h                               ; 80 cf 30
    mov word [bp+01ch], bx                    ; 89 5e 1c
    mov word [bp+01ah], strict word 00007h    ; c7 46 1a 07 00
    jmp near 042a7h                           ; e9 e1 01
    mov bx, word [bp+010h]                    ; 8b 5e 10
    mov es, [bp+00ah]                         ; 8e 46 0a
    mov di, bx                                ; 89 df
    mov [bp-012h], es                         ; 8c 46 ee
    mov ax, word [es:bx+002h]                 ; 26 8b 47 02
    mov word [bp-00ch], ax                    ; 89 46 f4
    mov ax, word [es:bx+006h]                 ; 26 8b 47 06
    mov word [bp-014h], ax                    ; 89 46 ec
    mov ax, word [es:bx+004h]                 ; 26 8b 47 04
    mov word [bp-016h], ax                    ; 89 46 ea
    mov ax, word [es:bx+00ch]                 ; 26 8b 47 0c
    mov word [bp-010h], ax                    ; 89 46 f0
    mov ax, word [es:bx+00eh]                 ; 26 8b 47 0e
    mov word [bp-01ch], ax                    ; 89 46 e4
    or ax, word [bp-010h]                     ; 0b 46 f0
    je short 04111h                           ; 74 18
    mov ax, word [bp+01ch]                    ; 8b 46 1c
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 0034fh                               ; 68 4f 03
    push 003b8h                               ; 68 b8 03
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 de d7
    add sp, strict byte 00008h                ; 83 c4 08
    jmp near 04502h                           ; e9 f1 03
    mov es, [bp-012h]                         ; 8e 46 ee
    mov ax, word [es:di+008h]                 ; 26 8b 45 08
    mov word [bp-010h], ax                    ; 89 46 f0
    mov ax, word [es:di+00ah]                 ; 26 8b 45 0a
    mov word [bp-01ch], ax                    ; 89 46 e4
    mov ax, word [bp+01ch]                    ; 8b 46 1c
    shr ax, 008h                              ; c1 e8 08
    cmp ax, strict word 00044h                ; 3d 44 00
    je near 042a3h                            ; 0f 84 74 01
    cmp ax, strict word 00047h                ; 3d 47 00
    je near 042a3h                            ; 0f 84 6d 01
    mov cx, strict word 0000ch                ; b9 0c 00
    xor bx, bx                                ; 31 db
    mov dx, ss                                ; 8c d2
    lea ax, [bp-028h]                         ; 8d 46 d8
    call 08cbah                               ; e8 77 4b
    mov word [bp-028h], strict word 00028h    ; c7 46 d8 28 00
    mov ax, word [bp-010h]                    ; 8b 46 f0
    mov dx, word [bp-01ch]                    ; 8b 56 e4
    xchg ah, al                               ; 86 c4
    xchg dh, dl                               ; 86 d6
    xchg dx, ax                               ; 92
    mov word [bp-026h], ax                    ; 89 46 da
    mov word [bp-024h], dx                    ; 89 56 dc
    mov ax, word [bp-00ch]                    ; 8b 46 f4
    xchg ah, al                               ; 86 c4
    mov word [bp-021h], ax                    ; 89 46 df
    les ax, [bp-00ch]                         ; c4 46 f4
    mov word [es:si+00ah], ax                 ; 26 89 44 0a
    mov word [es:si+00ch], 00800h             ; 26 c7 44 0c 00 08
    cmp byte [bp-006h], 008h                  ; 80 7e fa 08
    jbe short 0419ah                          ; 76 26
    push dword [bp-016h]                      ; 66 ff 76 ea
    push strict byte 00001h                   ; 6a 01
    xor bx, bx                                ; 31 db
    mov cx, strict word 0000bh                ; b9 0b 00
    sal ax, 1                                 ; d1 e0
    rcl bx, 1                                 ; d1 d3
    loop 0417fh                               ; e2 fa
    push bx                                   ; 53
    push ax                                   ; 50
    push strict byte 00000h                   ; 6a 00
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    mov cx, ss                                ; 8c d1
    lea bx, [bp-028h]                         ; 8d 5e d8
    mov dx, strict word 0000ch                ; ba 0c 00
    call 07f60h                               ; e8 c8 3d
    jmp short 041beh                          ; eb 24
    push dword [bp-016h]                      ; 66 ff 76 ea
    push strict byte 00001h                   ; 6a 01
    xor bx, bx                                ; 31 db
    mov cx, strict word 0000bh                ; b9 0b 00
    sal ax, 1                                 ; d1 e0
    rcl bx, 1                                 ; d1 d3
    loop 041a5h                               ; e2 fa
    push bx                                   ; 53
    push ax                                   ; 50
    push strict byte 00000h                   ; 6a 00
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    mov cx, ss                                ; 8c d1
    lea bx, [bp-028h]                         ; 8d 5e d8
    mov dx, strict word 0000ch                ; ba 0c 00
    call 02804h                               ; e8 46 e6
    mov byte [bp-004h], al                    ; 88 46 fc
    mov es, [bp-00ah]                         ; 8e 46 f6
    mov ax, word [es:si+016h]                 ; 26 8b 44 16
    mov bx, word [es:si+018h]                 ; 26 8b 5c 18
    mov cx, strict word 0000bh                ; b9 0b 00
    shr bx, 1                                 ; d1 eb
    rcr ax, 1                                 ; d1 d8
    loop 041cfh                               ; e2 fa
    mov es, [bp-012h]                         ; 8e 46 ee
    mov word [es:di+002h], ax                 ; 26 89 45 02
    cmp byte [bp-004h], 000h                  ; 80 7e fc 00
    je near 042a3h                            ; 0f 84 bf 00
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 bf d6
    movzx ax, byte [bp-004h]                  ; 0f b6 46 fc
    push ax                                   ; 50
    mov ax, word [bp+01ch]                    ; 8b 46 1c
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 0034fh                               ; 68 4f 03
    push 003e1h                               ; 68 e1 03
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 e3 d6
    add sp, strict byte 0000ah                ; 83 c4 0a
    mov ax, word [bp+01ch]                    ; 8b 46 1c
    xor ah, ah                                ; 30 e4
    or ah, 00ch                               ; 80 cc 0c
    jmp near 0450ah                           ; e9 f6 02
    cmp bx, strict byte 00002h                ; 83 fb 02
    jnbe near 04502h                          ; 0f 87 e7 02
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    imul ax, ax, strict byte 00018h           ; 6b c0 18
    mov es, [bp-00ah]                         ; 8e 46 f6
    mov di, si                                ; 89 f7
    add di, ax                                ; 01 c7
    mov al, byte [es:di+021h]                 ; 26 8a 45 21
    cmp bx, strict byte 00002h                ; 83 fb 02
    je short 04294h                           ; 74 62
    cmp bx, strict byte 00001h                ; 83 fb 01
    je short 04272h                           ; 74 3b
    test bx, bx                               ; 85 db
    jne near 042a3h                           ; 0f 85 66 00
    cmp AL, strict byte 0ffh                  ; 3c ff
    jne short 04253h                          ; 75 12
    mov ax, word [bp+01ch]                    ; 8b 46 1c
    xor ah, ah                                ; 30 e4
    or ah, 0b4h                               ; 80 cc b4
    mov word [bp+01ch], ax                    ; 89 46 1c
    xor al, al                                ; 30 c0
    or AL, strict byte 001h                   ; 0c 01
    jmp near 0450ah                           ; e9 b7 02
    movzx dx, byte [bp-006h]                  ; 0f b6 56 fa
    imul dx, dx, strict byte 00018h           ; 6b d2 18
    db  0feh, 0c0h
    ; inc al                                    ; fe c0
    mov es, [bp-00ah]                         ; 8e 46 f6
    add si, dx                                ; 01 d6
    mov byte [es:si+021h], al                 ; 26 88 44 21
    mov ax, word [bp+01ch]                    ; 8b 46 1c
    xor al, al                                ; 30 c0
    or AL, strict byte 001h                   ; 0c 01
    mov word [bp+01ch], ax                    ; 89 46 1c
    jmp near 042a3h                           ; e9 31 00
    test al, al                               ; 84 c0
    jne short 04282h                          ; 75 0c
    or bh, 0b0h                               ; 80 cf b0
    mov word [bp+01ch], bx                    ; 89 5e 1c
    mov byte [bp+01ch], al                    ; 88 46 1c
    jmp near 0450dh                           ; e9 8b 02
    movzx dx, byte [bp-006h]                  ; 0f b6 56 fa
    imul dx, dx, strict byte 00018h           ; 6b d2 18
    db  0feh, 0c8h
    ; dec al                                    ; fe c8
    mov es, [bp-00ah]                         ; 8e 46 f6
    add si, dx                                ; 01 d6
    mov byte [es:si+021h], al                 ; 26 88 44 21
    test al, al                               ; 84 c0
    db  00fh, 095h, 0c0h
    ; setne al                                  ; 0f 95 c0
    xor ah, ah                                ; 30 e4
    mov dx, word [bp+01ch]                    ; 8b 56 1c
    mov dl, al                                ; 88 c2
    mov word [bp+01ch], dx                    ; 89 56 1c
    mov byte [bp+01dh], 000h                  ; c6 46 1d 00
    xor bx, bx                                ; 31 db
    mov dx, strict word 00074h                ; ba 74 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 5c d3
    and byte [bp+022h], 0feh                  ; 80 66 22 fe
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn                                      ; c3
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    imul ax, ax, strict byte 00018h           ; 6b c0 18
    mov es, [bp-00ah]                         ; 8e 46 f6
    add si, ax                                ; 01 c6
    mov al, byte [es:si+021h]                 ; 26 8a 44 21
    test al, al                               ; 84 c0
    je short 042d4h                           ; 74 06
    or bh, 0b1h                               ; 80 cf b1
    jmp near 040adh                           ; e9 d9 fd
    cmp byte [bp-004h], 000h                  ; 80 7e fc 00
    je short 042a3h                           ; 74 c9
    mov ax, word [bp+01ch]                    ; 8b 46 1c
    xor ah, ah                                ; 30 e4
    or ah, 0b1h                               ; 80 cc b1
    jmp near 0450ah                           ; e9 25 02
    mov dx, word [bp+010h]                    ; 8b 56 10
    mov cx, word [bp+00ah]                    ; 8b 4e 0a
    mov bx, dx                                ; 89 d3
    mov word [bp-008h], cx                    ; 89 4e f8
    mov es, cx                                ; 8e c1
    mov di, dx                                ; 89 d7
    mov ax, word [es:di]                      ; 26 8b 05
    mov word [bp-00eh], ax                    ; 89 46 f2
    cmp ax, strict word 0001ah                ; 3d 1a 00
    jc near 04502h                            ; 0f 82 01 02
    jc short 04350h                           ; 72 4d
    movzx di, byte [bp-006h]                  ; 0f b6 7e fa
    imul di, di, strict byte 00018h           ; 6b ff 18
    mov es, [bp-00ah]                         ; 8e 46 f6
    add di, si                                ; 01 f7
    mov ax, word [es:di+024h]                 ; 26 8b 45 24
    mov es, cx                                ; 8e c1
    mov di, dx                                ; 89 d7
    db  066h, 026h, 0c7h, 005h, 01ah, 000h, 074h, 000h
    ; mov dword [es:di], strict dword 00074001ah ; 66 26 c7 05 1a 00 74 00
    db  066h, 026h, 0c7h, 045h, 004h, 0ffh, 0ffh, 0ffh, 0ffh
    ; mov dword [es:di+004h], strict dword 0ffffffffh ; 66 26 c7 45 04 ff ff ff ff
    db  066h, 026h, 0c7h, 045h, 008h, 0ffh, 0ffh, 0ffh, 0ffh
    ; mov dword [es:di+008h], strict dword 0ffffffffh ; 66 26 c7 45 08 ff ff ff ff
    db  066h, 026h, 0c7h, 045h, 00ch, 0ffh, 0ffh, 0ffh, 0ffh
    ; mov dword [es:di+00ch], strict dword 0ffffffffh ; 66 26 c7 45 0c ff ff ff ff
    mov word [es:di+018h], ax                 ; 26 89 45 18
    db  066h, 026h, 0c7h, 045h, 010h, 0ffh, 0ffh, 0ffh, 0ffh
    ; mov dword [es:di+010h], strict dword 0ffffffffh ; 66 26 c7 45 10 ff ff ff ff
    db  066h, 026h, 0c7h, 045h, 014h, 0ffh, 0ffh, 0ffh, 0ffh
    ; mov dword [es:di+014h], strict dword 0ffffffffh ; 66 26 c7 45 14 ff ff ff ff
    cmp word [bp-00eh], strict byte 0001eh    ; 83 7e f2 1e
    jc near 04426h                            ; 0f 82 ce 00
    mov es, [bp-008h]                         ; 8e 46 f8
    mov word [es:bx], strict word 0001eh      ; 26 c7 07 1e 00
    mov ax, word [bp-018h]                    ; 8b 46 e8
    mov word [es:bx+01ch], ax                 ; 26 89 47 1c
    mov word [es:bx+01ah], 00312h             ; 26 c7 47 1a 12 03
    movzx cx, byte [bp-006h]                  ; 0f b6 4e fa
    mov ax, cx                                ; 89 c8
    cwd                                       ; 99
    db  02bh, 0c2h
    ; sub ax, dx                                ; 2b c2
    sar ax, 1                                 ; d1 f8
    xor ah, ah                                ; 30 e4
    imul ax, ax, strict byte 00006h           ; 6b c0 06
    mov es, [bp-00ah]                         ; 8e 46 f6
    mov di, si                                ; 89 f7
    add di, ax                                ; 01 c7
    mov dx, word [es:di+001c2h]               ; 26 8b 95 c2 01
    mov ax, word [es:di+001c4h]               ; 26 8b 85 c4 01
    mov word [bp-01ah], ax                    ; 89 46 e6
    mov al, byte [es:di+001c1h]               ; 26 8a 85 c1 01
    mov byte [bp-002h], al                    ; 88 46 fe
    imul cx, cx, strict byte 00018h           ; 6b c9 18
    mov di, si                                ; 89 f7
    add di, cx                                ; 01 cf
    mov al, byte [es:di+022h]                 ; 26 8a 45 22
    cmp AL, strict byte 001h                  ; 3c 01
    db  00fh, 094h, 0c0h
    ; sete al                                   ; 0f 94 c0
    xor ah, ah                                ; 30 e4
    or AL, strict byte 070h                   ; 0c 70
    mov di, ax                                ; 89 c7
    mov word [es:si+001f0h], dx               ; 26 89 94 f0 01
    mov ax, word [bp-01ah]                    ; 8b 46 e6
    mov word [es:si+001f2h], ax               ; 26 89 84 f2 01
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    cwd                                       ; 99
    mov cx, strict word 00002h                ; b9 02 00
    idiv cx                                   ; f7 f9
    or dl, 00eh                               ; 80 ca 0e
    sal dx, 004h                              ; c1 e2 04
    mov byte [es:si+001f4h], dl               ; 26 88 94 f4 01
    mov byte [es:si+001f5h], 0cbh             ; 26 c6 84 f5 01 cb
    mov al, byte [bp-002h]                    ; 8a 46 fe
    mov byte [es:si+001f6h], al               ; 26 88 84 f6 01
    mov word [es:si+001f7h], strict word 00001h ; 26 c7 84 f7 01 01 00
    mov byte [es:si+001f9h], 000h             ; 26 c6 84 f9 01 00
    mov word [es:si+001fah], di               ; 26 89 bc fa 01
    mov word [es:si+001fch], strict word 00000h ; 26 c7 84 fc 01 00 00
    mov byte [es:si+001feh], 011h             ; 26 c6 84 fe 01 11
    xor cl, cl                                ; 30 c9
    xor ch, ch                                ; 30 ed
    jmp short 04409h                          ; eb 05
    cmp ch, 00fh                              ; 80 fd 0f
    jnc short 0441ch                          ; 73 13
    movzx dx, ch                              ; 0f b6 d5
    add dx, 00312h                            ; 81 c2 12 03
    mov ax, word [bp-018h]                    ; 8b 46 e8
    call 01600h                               ; e8 ea d1
    add cl, al                                ; 00 c1
    db  0feh, 0c5h
    ; inc ch                                    ; fe c5
    jmp short 04404h                          ; eb e8
    neg cl                                    ; f6 d9
    mov es, [bp-00ah]                         ; 8e 46 f6
    mov byte [es:si+001ffh], cl               ; 26 88 8c ff 01
    cmp word [bp-00eh], strict byte 00042h    ; 83 7e f2 42
    jc near 042a3h                            ; 0f 82 75 fe
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    cwd                                       ; 99
    db  02bh, 0c2h
    ; sub ax, dx                                ; 2b c2
    sar ax, 1                                 ; d1 f8
    xor ah, ah                                ; 30 e4
    imul ax, ax, strict byte 00006h           ; 6b c0 06
    mov es, [bp-00ah]                         ; 8e 46 f6
    add si, ax                                ; 01 c6
    mov al, byte [es:si+001c0h]               ; 26 8a 84 c0 01
    mov dx, word [es:si+001c2h]               ; 26 8b 94 c2 01
    mov es, [bp-008h]                         ; 8e 46 f8
    mov word [es:bx], strict word 00042h      ; 26 c7 07 42 00
    db  066h, 026h, 0c7h, 047h, 01eh, 0ddh, 0beh, 024h, 000h
    ; mov dword [es:bx+01eh], strict dword 00024beddh ; 66 26 c7 47 1e dd be 24 00
    mov word [es:bx+022h], strict word 00000h ; 26 c7 47 22 00 00
    test al, al                               ; 84 c0
    jne short 0446fh                          ; 75 09
    db  066h, 026h, 0c7h, 047h, 024h, 049h, 053h, 041h, 020h
    ; mov dword [es:bx+024h], strict dword 020415349h ; 66 26 c7 47 24 49 53 41 20
    mov es, [bp-008h]                         ; 8e 46 f8
    db  066h, 026h, 0c7h, 047h, 028h, 041h, 054h, 041h, 020h
    ; mov dword [es:bx+028h], strict dword 020415441h ; 66 26 c7 47 28 41 54 41 20
    db  066h, 026h, 0c7h, 047h, 02ch, 020h, 020h, 020h, 020h
    ; mov dword [es:bx+02ch], strict dword 020202020h ; 66 26 c7 47 2c 20 20 20 20
    test al, al                               ; 84 c0
    jne short 0449bh                          ; 75 13
    mov word [es:bx+030h], dx                 ; 26 89 57 30
    db  066h, 026h, 0c7h, 047h, 032h, 000h, 000h, 000h, 000h
    ; mov dword [es:bx+032h], strict dword 000000000h ; 66 26 c7 47 32 00 00 00 00
    mov word [es:bx+036h], strict word 00000h ; 26 c7 47 36 00 00
    mov al, byte [bp-006h]                    ; 8a 46 fa
    and AL, strict byte 001h                  ; 24 01
    xor ah, ah                                ; 30 e4
    mov es, [bp-008h]                         ; 8e 46 f8
    mov word [es:bx+038h], ax                 ; 26 89 47 38
    db  066h, 026h, 0c7h, 047h, 03ah, 000h, 000h, 000h, 000h
    ; mov dword [es:bx+03ah], strict dword 000000000h ; 66 26 c7 47 3a 00 00 00 00
    mov word [es:bx+03eh], strict word 00000h ; 26 c7 47 3e 00 00
    xor al, al                                ; 30 c0
    mov AH, strict byte 01eh                  ; b4 1e
    jmp short 044c3h                          ; eb 05
    cmp ah, 040h                              ; 80 fc 40
    jnc short 044d2h                          ; 73 0f
    movzx si, ah                              ; 0f b6 f4
    mov es, [bp-008h]                         ; 8e 46 f8
    add si, bx                                ; 01 de
    add al, byte [es:si]                      ; 26 02 04
    db  0feh, 0c4h
    ; inc ah                                    ; fe c4
    jmp short 044beh                          ; eb ec
    neg al                                    ; f6 d8
    mov es, [bp-008h]                         ; 8e 46 f8
    mov byte [es:bx+041h], al                 ; 26 88 47 41
    jmp near 042a3h                           ; e9 c5 fd
    or bh, 006h                               ; 80 cf 06
    mov word [bp+01ch], bx                    ; 89 5e 1c
    jmp short 0451eh                          ; eb 38
    cmp bx, strict byte 00006h                ; 83 fb 06
    je near 042a3h                            ; 0f 84 b6 fd
    cmp bx, strict byte 00001h                ; 83 fb 01
    jc short 04502h                           ; 72 10
    jbe near 042a3h                           ; 0f 86 ad fd
    cmp bx, strict byte 00003h                ; 83 fb 03
    jc short 04502h                           ; 72 07
    cmp bx, strict byte 00004h                ; 83 fb 04
    jbe near 042a3h                           ; 0f 86 a1 fd
    mov ax, word [bp+01ch]                    ; 8b 46 1c
    xor ah, ah                                ; 30 e4
    or ah, 001h                               ; 80 cc 01
    mov word [bp+01ch], ax                    ; 89 46 1c
    mov bx, word [bp+01ch]                    ; 8b 5e 1c
    shr bx, 008h                              ; c1 eb 08
    xor bh, bh                                ; 30 ff
    mov dx, strict word 00074h                ; ba 74 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 f0 d0
    or byte [bp+022h], 001h                   ; 80 4e 22 01
    jmp near 042b6h                           ; e9 91 fd
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 7e d3
    mov ax, word [bp+01ch]                    ; 8b 46 1c
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 0034fh                               ; 68 4f 03
    push 002a3h                               ; 68 a3 02
    push strict byte 00004h                   ; 6a 04
    jmp near 04108h                           ; e9 c6 fb
print_boot_device_:                          ; 0xf4542 LB 0x48
    push cx                                   ; 51
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    test al, al                               ; 84 c0
    je short 0454fh                           ; 74 05
    mov dx, strict word 00002h                ; ba 02 00
    jmp short 04569h                          ; eb 1a
    test dl, dl                               ; 84 d2
    je short 04558h                           ; 74 05
    mov dx, strict word 00003h                ; ba 03 00
    jmp short 04569h                          ; eb 11
    test bl, 080h                             ; f6 c3 80
    jne short 04561h                          ; 75 04
    xor dh, dh                                ; 30 f6
    jmp short 04569h                          ; eb 08
    test bl, 080h                             ; f6 c3 80
    je short 04587h                           ; 74 21
    mov dx, strict word 00001h                ; ba 01 00
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 3a d3
    imul dx, dx, strict byte 0000ah           ; 6b d2 0a
    add dx, 00cfeh                            ; 81 c2 fe 0c
    push dx                                   ; 52
    push 00404h                               ; 68 04 04
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 65 d3
    add sp, strict byte 00006h                ; 83 c4 06
    pop bp                                    ; 5d
    pop cx                                    ; 59
    retn                                      ; c3
print_boot_failure_:                         ; 0xf458a LB 0x90
    push si                                   ; 56
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov dh, cl                                ; 88 ce
    mov ah, bl                                ; 88 dc
    and ah, 07fh                              ; 80 e4 7f
    movzx si, ah                              ; 0f b6 f4
    test al, al                               ; 84 c0
    je short 045b7h                           ; 74 1b
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 07 d3
    push 00d12h                               ; 68 12 0d
    push 00418h                               ; 68 18 04
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 37 d3
    add sp, strict byte 00006h                ; 83 c4 06
    jmp short 045fbh                          ; eb 44
    test dl, dl                               ; 84 d2
    je short 045cbh                           ; 74 10
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 e8 d2
    push 00d1ch                               ; 68 1c 0d
    jmp short 045aah                          ; eb df
    test bl, 080h                             ; f6 c3 80
    je short 045e1h                           ; 74 11
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 d3 d2
    push si                                   ; 56
    push 00d08h                               ; 68 08 0d
    jmp short 045f0h                          ; eb 0f
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 c2 d2
    push si                                   ; 56
    push 00cfeh                               ; 68 fe 0c
    push 0042dh                               ; 68 2d 04
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 f1 d2
    add sp, strict byte 00008h                ; 83 c4 08
    cmp byte [bp+006h], 001h                  ; 80 7e 06 01
    jne short 04615h                          ; 75 14
    test dh, dh                               ; 84 f6
    jne short 0460ah                          ; 75 05
    push 00445h                               ; 68 45 04
    jmp short 0460dh                          ; eb 03
    push 0046fh                               ; 68 6f 04
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 d7 d2
    add sp, strict byte 00004h                ; 83 c4 04
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn 00002h                               ; c2 02 00
print_cdromboot_failure_:                    ; 0xf461a LB 0x24
    push bx                                   ; 53
    push cx                                   ; 51
    push dx                                   ; 52
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov dx, ax                                ; 89 c2
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 81 d2
    push dx                                   ; 52
    push 004a4h                               ; 68 a4 04
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 b3 d2
    add sp, strict byte 00006h                ; 83 c4 06
    pop bp                                    ; 5d
    pop dx                                    ; 5a
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
_int19_function:                             ; 0xf463e LB 0x251
    push si                                   ; 56
    push di                                   ; 57
    enter 0000eh, 000h                        ; c8 0e 00 00
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 cf cf
    mov bx, ax                                ; 89 c3
    mov word [bp-00ah], ax                    ; 89 46 f6
    mov byte [bp-004h], 000h                  ; c6 46 fc 00
    mov ax, strict word 0003dh                ; b8 3d 00
    call 0165ch                               ; e8 00 d0
    movzx si, al                              ; 0f b6 f0
    mov ax, strict word 00038h                ; b8 38 00
    call 0165ch                               ; e8 f7 cf
    and AL, strict byte 0f0h                  ; 24 f0
    xor ah, ah                                ; 30 e4
    sal ax, 004h                              ; c1 e0 04
    or si, ax                                 ; 09 c6
    mov ax, strict word 0003ch                ; b8 3c 00
    call 0165ch                               ; e8 e8 cf
    and AL, strict byte 00fh                  ; 24 0f
    xor ah, ah                                ; 30 e4
    sal ax, 00ch                              ; c1 e0 0c
    or si, ax                                 ; 09 c6
    mov dx, 00339h                            ; ba 39 03
    mov ax, bx                                ; 89 d8
    call 01600h                               ; e8 7b cf
    test al, al                               ; 84 c0
    je short 04694h                           ; 74 0b
    mov dx, 00339h                            ; ba 39 03
    mov ax, bx                                ; 89 d8
    call 01600h                               ; e8 6f cf
    movzx si, al                              ; 0f b6 f0
    cmp byte [bp+008h], 001h                  ; 80 7e 08 01
    jne short 046aah                          ; 75 10
    mov ax, strict word 0003ch                ; b8 3c 00
    call 0165ch                               ; e8 bc cf
    and AL, strict byte 0f0h                  ; 24 f0
    xor ah, ah                                ; 30 e4
    sar ax, 004h                              ; c1 f8 04
    call 0721ah                               ; e8 70 2b
    cmp byte [bp+008h], 002h                  ; 80 7e 08 02
    jne short 046b3h                          ; 75 03
    shr si, 004h                              ; c1 ee 04
    cmp byte [bp+008h], 003h                  ; 80 7e 08 03
    jne short 046bch                          ; 75 03
    shr si, 008h                              ; c1 ee 08
    cmp byte [bp+008h], 004h                  ; 80 7e 08 04
    jne short 046c5h                          ; 75 03
    shr si, 00ch                              ; c1 ee 0c
    cmp si, strict byte 00010h                ; 83 fe 10
    jnc short 046ceh                          ; 73 04
    mov byte [bp-004h], 001h                  ; c6 46 fc 01
    xor al, al                                ; 30 c0
    mov byte [bp-002h], al                    ; 88 46 fe
    mov byte [bp-006h], al                    ; 88 46 fa
    mov byte [bp-008h], al                    ; 88 46 f8
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 ca d1
    push si                                   ; 56
    movzx ax, byte [bp+008h]                  ; 0f b6 46 08
    push ax                                   ; 50
    push 004c4h                               ; 68 c4 04
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 f7 d1
    add sp, strict byte 00008h                ; 83 c4 08
    and si, strict byte 0000fh                ; 83 e6 0f
    cmp si, strict byte 00002h                ; 83 fe 02
    jc short 0470bh                           ; 72 0e
    jbe short 0471ah                          ; 76 1b
    cmp si, strict byte 00004h                ; 83 fe 04
    je short 04738h                           ; 74 34
    cmp si, strict byte 00003h                ; 83 fe 03
    je short 0472eh                           ; 74 25
    jmp short 04765h                          ; eb 5a
    cmp si, strict byte 00001h                ; 83 fe 01
    jne short 04765h                          ; 75 55
    xor al, al                                ; 30 c0
    mov byte [bp-002h], al                    ; 88 46 fe
    mov byte [bp-006h], al                    ; 88 46 fa
    jmp short 0477dh                          ; eb 63
    mov dx, 00338h                            ; ba 38 03
    mov ax, word [bp-00ah]                    ; 8b 46 f6
    call 01600h                               ; e8 dd ce
    add AL, strict byte 080h                  ; 04 80
    mov byte [bp-002h], al                    ; 88 46 fe
    mov byte [bp-006h], 000h                  ; c6 46 fa 00
    jmp short 0477dh                          ; eb 4f
    mov byte [bp-002h], 000h                  ; c6 46 fe 00
    mov byte [bp-006h], 001h                  ; c6 46 fa 01
    jmp short 04742h                          ; eb 0a
    mov byte [bp-008h], 001h                  ; c6 46 f8 01
    cmp byte [bp-006h], 000h                  ; 80 7e fa 00
    je short 0477dh                           ; 74 3b
    call 0377dh                               ; e8 38 f0
    mov bx, ax                                ; 89 c3
    test AL, strict byte 0ffh                 ; a8 ff
    je short 0476ch                           ; 74 21
    call 0461ah                               ; e8 cc fe
    movzx ax, byte [bp-004h]                  ; 0f b6 46 fc
    push ax                                   ; 50
    movzx bx, byte [bp-002h]                  ; 0f b6 5e fe
    movzx dx, byte [bp-008h]                  ; 0f b6 56 f8
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    mov cx, strict word 00001h                ; b9 01 00
    call 0458ah                               ; e8 25 fe
    xor ax, ax                                ; 31 c0
    xor dx, dx                                ; 31 d2
    jmp near 0488bh                           ; e9 1f 01
    mov dx, 0032eh                            ; ba 2e 03
    mov ax, word [bp-00ah]                    ; 8b 46 f6
    call 0161ch                               ; e8 a7 ce
    mov di, ax                                ; 89 c7
    shr bx, 008h                              ; c1 eb 08
    mov byte [bp-002h], bl                    ; 88 5e fe
    cmp byte [bp-008h], 001h                  ; 80 7e f8 01
    jne near 047f9h                           ; 0f 85 74 00
    xor si, si                                ; 31 f6
    mov ax, 0e200h                            ; b8 00 e2
    mov es, ax                                ; 8e c0
    cmp word [es:si], 0aa55h                  ; 26 81 3c 55 aa
    jne short 0474eh                          ; 75 bb
    mov cx, ax                                ; 89 c1
    mov si, word [es:si+01ah]                 ; 26 8b 74 1a
    cmp word [es:si+002h], 0506eh             ; 26 81 7c 02 6e 50
    jne short 0474eh                          ; 75 ad
    cmp word [es:si], 05024h                  ; 26 81 3c 24 50
    jne short 0474eh                          ; 75 a6
    mov di, word [es:si+00eh]                 ; 26 8b 7c 0e
    mov dx, word [es:di]                      ; 26 8b 15
    mov ax, word [es:di+002h]                 ; 26 8b 45 02
    cmp ax, 06568h                            ; 3d 68 65
    jne short 047d7h                          ; 75 1f
    cmp dx, 07445h                            ; 81 fa 45 74
    jne short 047d7h                          ; 75 19
    movzx bx, byte [bp-002h]                  ; 0f b6 5e fe
    movzx dx, byte [bp-008h]                  ; 0f b6 56 f8
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    call 04542h                               ; e8 75 fd
    mov word [bp-00eh], strict word 00006h    ; c7 46 f2 06 00
    mov word [bp-00ch], cx                    ; 89 4e f4
    jmp short 047f3h                          ; eb 1c
    movzx bx, byte [bp-002h]                  ; 0f b6 5e fe
    movzx dx, byte [bp-008h]                  ; 0f b6 56 f8
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    call 04542h                               ; e8 5c fd
    sti                                       ; fb
    mov word [bp-00ch], cx                    ; 89 4e f4
    mov es, cx                                ; 8e c1
    mov ax, word [es:si+01ah]                 ; 26 8b 44 1a
    mov word [bp-00eh], ax                    ; 89 46 f2
    call far [bp-00eh]                        ; ff 5e f2
    jmp near 0474eh                           ; e9 55 ff
    cmp byte [bp-006h], 000h                  ; 80 7e fa 00
    jne short 04825h                          ; 75 26
    cmp byte [bp-008h], 000h                  ; 80 7e f8 00
    jne short 04825h                          ; 75 20
    mov di, 007c0h                            ; bf c0 07
    mov es, di                                ; 8e c7
    mov dl, byte [bp-002h]                    ; 8a 56 fe
    mov ax, 00201h                            ; b8 01 02
    mov DH, strict byte 000h                  ; b6 00
    mov cx, strict word 00001h                ; b9 01 00
    db  033h, 0dbh
    ; xor bx, bx                                ; 33 db
    int 013h                                  ; cd 13
    mov ax, strict word 00000h                ; b8 00 00
    sbb ax, strict byte 00000h                ; 83 d8 00
    test ax, ax                               ; 85 c0
    jne near 0474eh                           ; 0f 85 29 ff
    cmp byte [bp-002h], 000h                  ; 80 7e fe 00
    db  00fh, 094h, 0c1h
    ; sete cl                                   ; 0f 94 c1
    cmp byte [bp-006h], 000h                  ; 80 7e fa 00
    je short 04834h                           ; 74 02
    mov CL, strict byte 001h                  ; b1 01
    xor dx, dx                                ; 31 d2
    mov ax, di                                ; 89 f8
    call 0161ch                               ; e8 e1 cd
    mov bx, ax                                ; 89 c3
    mov dx, strict word 00002h                ; ba 02 00
    mov ax, di                                ; 89 f8
    call 0161ch                               ; e8 d7 cd
    cmp bx, ax                                ; 39 c3
    je short 0485ah                           ; 74 11
    test cl, cl                               ; 84 c9
    jne short 04870h                          ; 75 23
    mov dx, 001feh                            ; ba fe 01
    mov ax, di                                ; 89 f8
    call 0161ch                               ; e8 c7 cd
    cmp ax, 0aa55h                            ; 3d 55 aa
    je short 04870h                           ; 74 16
    movzx ax, byte [bp-004h]                  ; 0f b6 46 fc
    push ax                                   ; 50
    movzx bx, byte [bp-002h]                  ; 0f b6 5e fe
    movzx dx, byte [bp-008h]                  ; 0f b6 56 f8
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    xor cx, cx                                ; 31 c9
    jmp near 04762h                           ; e9 f2 fe
    movzx bx, byte [bp-002h]                  ; 0f b6 5e fe
    movzx dx, byte [bp-008h]                  ; 0f b6 56 f8
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    call 04542h                               ; e8 c3 fc
    movzx bx, byte [bp-002h]                  ; 0f b6 5e fe
    xor dx, dx                                ; 31 d2
    xor ax, ax                                ; 31 c0
    add ax, di                                ; 01 f8
    adc dx, bx                                ; 11 da
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn                                      ; c3
keyboard_panic_:                             ; 0xf488f LB 0x11
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    push ax                                   ; 50
    push 004e4h                               ; 68 e4 04
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 4e d0
    add sp, strict byte 00006h                ; 83 c4 06
    pop bp                                    ; 5d
    retn                                      ; c3
_keyboard_init:                              ; 0xf48a0 LB 0x27a
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov AL, strict byte 0aah                  ; b0 aa
    mov dx, strict word 00064h                ; ba 64 00
    out DX, AL                                ; ee
    mov bx, strict word 0ffffh                ; bb ff ff
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 002h                 ; a8 02
    je short 048c3h                           ; 74 0d
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 048c3h                          ; 76 08
    xor al, al                                ; 30 c0
    mov dx, 00080h                            ; ba 80 00
    out DX, AL                                ; ee
    jmp short 048ach                          ; eb e9
    test bx, bx                               ; 85 db
    jne short 048cch                          ; 75 05
    xor ax, ax                                ; 31 c0
    call 0488fh                               ; e8 c3 ff
    mov bx, strict word 0ffffh                ; bb ff ff
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 001h                 ; a8 01
    jne short 048e6h                          ; 75 0d
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 048e6h                          ; 76 08
    mov AL, strict byte 001h                  ; b0 01
    mov dx, 00080h                            ; ba 80 00
    out DX, AL                                ; ee
    jmp short 048cfh                          ; eb e9
    test bx, bx                               ; 85 db
    jne short 048f0h                          ; 75 06
    mov ax, strict word 00001h                ; b8 01 00
    call 0488fh                               ; e8 9f ff
    mov dx, strict word 00060h                ; ba 60 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    cmp ax, strict word 00055h                ; 3d 55 00
    je short 04901h                           ; 74 06
    mov ax, 003dfh                            ; b8 df 03
    call 0488fh                               ; e8 8e ff
    mov AL, strict byte 0abh                  ; b0 ab
    mov dx, strict word 00064h                ; ba 64 00
    out DX, AL                                ; ee
    mov bx, strict word 0ffffh                ; bb ff ff
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 002h                 ; a8 02
    je short 04921h                           ; 74 0d
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 04921h                          ; 76 08
    mov AL, strict byte 010h                  ; b0 10
    mov dx, 00080h                            ; ba 80 00
    out DX, AL                                ; ee
    jmp short 0490ah                          ; eb e9
    test bx, bx                               ; 85 db
    jne short 0492bh                          ; 75 06
    mov ax, strict word 0000ah                ; b8 0a 00
    call 0488fh                               ; e8 64 ff
    mov bx, strict word 0ffffh                ; bb ff ff
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 001h                 ; a8 01
    jne short 04945h                          ; 75 0d
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 04945h                          ; 76 08
    mov AL, strict byte 011h                  ; b0 11
    mov dx, 00080h                            ; ba 80 00
    out DX, AL                                ; ee
    jmp short 0492eh                          ; eb e9
    test bx, bx                               ; 85 db
    jne short 0494fh                          ; 75 06
    mov ax, strict word 0000bh                ; b8 0b 00
    call 0488fh                               ; e8 40 ff
    mov dx, strict word 00060h                ; ba 60 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test ax, ax                               ; 85 c0
    je short 0495fh                           ; 74 06
    mov ax, 003e0h                            ; b8 e0 03
    call 0488fh                               ; e8 30 ff
    mov AL, strict byte 0ffh                  ; b0 ff
    mov dx, strict word 00060h                ; ba 60 00
    out DX, AL                                ; ee
    mov bx, strict word 0ffffh                ; bb ff ff
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 002h                 ; a8 02
    je short 0497fh                           ; 74 0d
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 0497fh                          ; 76 08
    mov AL, strict byte 020h                  ; b0 20
    mov dx, 00080h                            ; ba 80 00
    out DX, AL                                ; ee
    jmp short 04968h                          ; eb e9
    test bx, bx                               ; 85 db
    jne short 04989h                          ; 75 06
    mov ax, strict word 00014h                ; b8 14 00
    call 0488fh                               ; e8 06 ff
    mov bx, strict word 0ffffh                ; bb ff ff
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 001h                 ; a8 01
    jne short 049a3h                          ; 75 0d
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 049a3h                          ; 76 08
    mov AL, strict byte 021h                  ; b0 21
    mov dx, 00080h                            ; ba 80 00
    out DX, AL                                ; ee
    jmp short 0498ch                          ; eb e9
    test bx, bx                               ; 85 db
    jne short 049adh                          ; 75 06
    mov ax, strict word 00015h                ; b8 15 00
    call 0488fh                               ; e8 e2 fe
    mov dx, strict word 00060h                ; ba 60 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    cmp ax, 000fah                            ; 3d fa 00
    je short 049beh                           ; 74 06
    mov ax, 003e1h                            ; b8 e1 03
    call 0488fh                               ; e8 d1 fe
    mov bx, strict word 0ffffh                ; bb ff ff
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 001h                 ; a8 01
    jne short 049d8h                          ; 75 0d
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 049d8h                          ; 76 08
    mov AL, strict byte 031h                  ; b0 31
    mov dx, 00080h                            ; ba 80 00
    out DX, AL                                ; ee
    jmp short 049c1h                          ; eb e9
    test bx, bx                               ; 85 db
    jne short 049e2h                          ; 75 06
    mov ax, strict word 0001fh                ; b8 1f 00
    call 0488fh                               ; e8 ad fe
    mov dx, strict word 00060h                ; ba 60 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    cmp ax, 000aah                            ; 3d aa 00
    je short 049fbh                           ; 74 0e
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    cmp ax, 000aah                            ; 3d aa 00
    je short 049fbh                           ; 74 06
    mov ax, 003e2h                            ; b8 e2 03
    call 0488fh                               ; e8 94 fe
    mov AL, strict byte 0f5h                  ; b0 f5
    mov dx, strict word 00060h                ; ba 60 00
    out DX, AL                                ; ee
    mov bx, strict word 0ffffh                ; bb ff ff
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 002h                 ; a8 02
    je short 04a1bh                           ; 74 0d
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 04a1bh                          ; 76 08
    mov AL, strict byte 040h                  ; b0 40
    mov dx, 00080h                            ; ba 80 00
    out DX, AL                                ; ee
    jmp short 04a04h                          ; eb e9
    test bx, bx                               ; 85 db
    jne short 04a25h                          ; 75 06
    mov ax, strict word 00028h                ; b8 28 00
    call 0488fh                               ; e8 6a fe
    mov bx, strict word 0ffffh                ; bb ff ff
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 001h                 ; a8 01
    jne short 04a3fh                          ; 75 0d
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 04a3fh                          ; 76 08
    mov AL, strict byte 041h                  ; b0 41
    mov dx, 00080h                            ; ba 80 00
    out DX, AL                                ; ee
    jmp short 04a28h                          ; eb e9
    test bx, bx                               ; 85 db
    jne short 04a49h                          ; 75 06
    mov ax, strict word 00029h                ; b8 29 00
    call 0488fh                               ; e8 46 fe
    mov dx, strict word 00060h                ; ba 60 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    cmp ax, 000fah                            ; 3d fa 00
    je short 04a5ah                           ; 74 06
    mov ax, 003e3h                            ; b8 e3 03
    call 0488fh                               ; e8 35 fe
    mov AL, strict byte 060h                  ; b0 60
    mov dx, strict word 00064h                ; ba 64 00
    out DX, AL                                ; ee
    mov bx, strict word 0ffffh                ; bb ff ff
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 002h                 ; a8 02
    je short 04a7ah                           ; 74 0d
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 04a7ah                          ; 76 08
    mov AL, strict byte 050h                  ; b0 50
    mov dx, 00080h                            ; ba 80 00
    out DX, AL                                ; ee
    jmp short 04a63h                          ; eb e9
    test bx, bx                               ; 85 db
    jne short 04a84h                          ; 75 06
    mov ax, strict word 00032h                ; b8 32 00
    call 0488fh                               ; e8 0b fe
    mov AL, strict byte 065h                  ; b0 65
    mov dx, strict word 00060h                ; ba 60 00
    out DX, AL                                ; ee
    mov bx, strict word 0ffffh                ; bb ff ff
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 002h                 ; a8 02
    je short 04aa4h                           ; 74 0d
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 04aa4h                          ; 76 08
    mov AL, strict byte 060h                  ; b0 60
    mov dx, 00080h                            ; ba 80 00
    out DX, AL                                ; ee
    jmp short 04a8dh                          ; eb e9
    test bx, bx                               ; 85 db
    jne short 04aaeh                          ; 75 06
    mov ax, strict word 0003ch                ; b8 3c 00
    call 0488fh                               ; e8 e1 fd
    mov AL, strict byte 0f4h                  ; b0 f4
    mov dx, strict word 00060h                ; ba 60 00
    out DX, AL                                ; ee
    mov bx, strict word 0ffffh                ; bb ff ff
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 002h                 ; a8 02
    je short 04aceh                           ; 74 0d
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 04aceh                          ; 76 08
    mov AL, strict byte 070h                  ; b0 70
    mov dx, 00080h                            ; ba 80 00
    out DX, AL                                ; ee
    jmp short 04ab7h                          ; eb e9
    test bx, bx                               ; 85 db
    jne short 04ad8h                          ; 75 06
    mov ax, strict word 00046h                ; b8 46 00
    call 0488fh                               ; e8 b7 fd
    mov bx, strict word 0ffffh                ; bb ff ff
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 001h                 ; a8 01
    jne short 04af2h                          ; 75 0d
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 04af2h                          ; 76 08
    mov AL, strict byte 071h                  ; b0 71
    mov dx, 00080h                            ; ba 80 00
    out DX, AL                                ; ee
    jmp short 04adbh                          ; eb e9
    test bx, bx                               ; 85 db
    jne short 04afch                          ; 75 06
    mov ax, strict word 00046h                ; b8 46 00
    call 0488fh                               ; e8 93 fd
    mov dx, strict word 00060h                ; ba 60 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    cmp ax, 000fah                            ; 3d fa 00
    je short 04b0dh                           ; 74 06
    mov ax, 003e4h                            ; b8 e4 03
    call 0488fh                               ; e8 82 fd
    mov AL, strict byte 0a8h                  ; b0 a8
    mov dx, strict word 00064h                ; ba 64 00
    out DX, AL                                ; ee
    xor ax, ax                                ; 31 c0
    call 05cfeh                               ; e8 e6 11
    pop bp                                    ; 5d
    retn                                      ; c3
enqueue_key_:                                ; 0xf4b1a LB 0x90
    push bx                                   ; 53
    push cx                                   ; 51
    push si                                   ; 56
    push di                                   ; 57
    enter 00002h, 000h                        ; c8 02 00 00
    mov byte [bp-002h], al                    ; 88 46 fe
    mov bl, dl                                ; 88 d3
    mov dx, strict word 0001ah                ; ba 1a 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 ec ca
    mov di, ax                                ; 89 c7
    mov dx, strict word 0001ch                ; ba 1c 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 e1 ca
    mov si, ax                                ; 89 c6
    lea cx, [si+002h]                         ; 8d 4c 02
    cmp cx, strict byte 0003eh                ; 83 f9 3e
    jc short 04b48h                           ; 72 03
    mov cx, strict word 0001eh                ; b9 1e 00
    cmp cx, di                                ; 39 f9
    jne short 04b50h                          ; 75 04
    xor ax, ax                                ; 31 c0
    jmp short 04b75h                          ; eb 25
    xor bh, bh                                ; 30 ff
    mov dx, si                                ; 89 f2
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 b4 ca
    movzx bx, byte [bp-002h]                  ; 0f b6 5e fe
    lea dx, [si+001h]                         ; 8d 54 01
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 a7 ca
    mov bx, cx                                ; 89 cb
    mov dx, strict word 0001ch                ; ba 1c 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0162ah                               ; e8 b8 ca
    mov ax, strict word 00001h                ; b8 01 00
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
    db  0c6h, 0c5h, 0bah
    ; mov ch, 0bah                              ; c6 c5 ba
    mov ax, 0aab6h                            ; b8 b6 aa
    popfw                                     ; 9d
    push bx                                   ; 53
    inc si                                    ; 46
    inc bp                                    ; 45
    cmp bh, byte [bx+si]                      ; 3a 38
    sub bl, byte [ss:di]                      ; 36 2a 1d
    loop 04bd9h                               ; e2 4d
    outsw                                     ; 6f
    dec sp                                    ; 4c
    db  03eh, 04ch
    ; ds dec sp                                 ; 3e 4c
    db  03eh, 04ch
    ; ds dec sp                                 ; 3e 4c
    db  0f1h
    dec sp                                    ; 4c
    pop SS                                    ; 17
    dec sp                                    ; 4c
    arpl word [di-04eh], cx                   ; 63 4d b2
    dec bp                                    ; 4d
    aad 04dh                                  ; d5 4d
    mov AL, strict byte 04ch                  ; b0 4c
    db  03eh, 04ch
    ; ds dec sp                                 ; 3e 4c
    db  03eh, 04ch
    ; ds dec sp                                 ; 3e 4c
    sub cl, byte [di+030h]                    ; 2a 4d 30
    dec sp                                    ; 4c
    xchg bx, ax                               ; 93
    dec bp                                    ; 4d
    into                                      ; ce
    dec bp                                    ; 4d
_int09_function:                             ; 0xf4baa LB 0x358
    push si                                   ; 56
    push di                                   ; 57
    enter 0000ch, 000h                        ; c8 0c 00 00
    mov al, byte [bp+018h]                    ; 8a 46 18
    mov byte [bp-006h], al                    ; 88 46 fa
    test al, al                               ; 84 c0
    jne short 04bd3h                          ; 75 19
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 e9 cc
    push 004f7h                               ; 68 f7 04
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 1c cd
    add sp, strict byte 00004h                ; 83 c4 04
    jmp near 04efeh                           ; e9 2b 03
    mov dx, strict word 00017h                ; ba 17 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 24 ca
    mov byte [bp-00ah], al                    ; 88 46 f6
    mov bl, al                                ; 88 c3
    mov dx, strict word 00018h                ; ba 18 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 16 ca
    mov byte [bp-00ch], al                    ; 88 46 f4
    mov byte [bp-008h], al                    ; 88 46 f8
    mov dx, 00096h                            ; ba 96 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 07 ca
    mov byte [bp-004h], al                    ; 88 46 fc
    mov byte [bp-002h], al                    ; 88 46 fe
    mov al, byte [bp-006h]                    ; 8a 46 fa
    push CS                                   ; 0e
    pop ES                                    ; 07
    mov cx, strict word 00010h                ; b9 10 00
    mov di, 04b7bh                            ; bf 7b 4b
    repne scasb                               ; f2 ae
    sal cx, 1                                 ; d1 e1
    mov di, cx                                ; 89 cf
    mov ax, word [cs:di+04b8ah]               ; 2e 8b 85 8a 4b
    jmp ax                                    ; ff e0
    xor bl, 040h                              ; 80 f3 40
    xor bh, bh                                ; 30 ff
    mov dx, strict word 00017h                ; ba 17 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 e9 c9
    or byte [bp-008h], 040h                   ; 80 4e f8 40
    movzx bx, byte [bp-008h]                  ; 0f b6 5e f8
    jmp near 04da6h                           ; e9 76 01
    mov al, byte [bp-00ch]                    ; 8a 46 f4
    and AL, strict byte 0bfh                  ; 24 bf
    mov byte [bp-008h], al                    ; 88 46 f8
    movzx bx, al                              ; 0f b6 d8
    jmp near 04da6h                           ; e9 68 01
    test byte [bp-002h], 002h                 ; f6 46 fe 02
    jne near 04ee0h                           ; 0f 85 9a 02
    mov al, byte [bp-006h]                    ; 8a 46 fa
    and AL, strict byte 07fh                  ; 24 7f
    cmp AL, strict byte 02ah                  ; 3c 2a
    db  00fh, 094h, 0c0h
    ; sete al                                   ; 0f 94 c0
    xor ah, ah                                ; 30 e4
    inc ax                                    ; 40
    test byte [bp-006h], 080h                 ; f6 46 fa 80
    je short 04c5fh                           ; 74 06
    not al                                    ; f6 d0
    and bl, al                                ; 20 c3
    jmp short 04c61h                          ; eb 02
    or bl, al                                 ; 08 c3
    xor bh, bh                                ; 30 ff
    mov dx, strict word 00017h                ; ba 17 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 a2 c9
    jmp near 04ee0h                           ; e9 71 02
    test byte [bp-004h], 001h                 ; f6 46 fc 01
    jne near 04ee0h                           ; 0f 85 69 02
    or bl, 004h                               ; 80 cb 04
    xor bh, bh                                ; 30 ff
    mov dx, strict word 00017h                ; ba 17 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 89 c9
    mov al, byte [bp-004h]                    ; 8a 46 fc
    test AL, strict byte 002h                 ; a8 02
    je short 04c99h                           ; 74 0d
    or AL, strict byte 004h                   ; 0c 04
    mov byte [bp-002h], al                    ; 88 46 fe
    movzx bx, al                              ; 0f b6 d8
    mov dx, 00096h                            ; ba 96 00
    jmp short 04ca7h                          ; eb 0e
    mov al, byte [bp-00ch]                    ; 8a 46 f4
    or AL, strict byte 001h                   ; 0c 01
    mov byte [bp-008h], al                    ; 88 46 f8
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 00018h                ; ba 18 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 61 c9
    jmp near 04ee0h                           ; e9 30 02
    test byte [bp-004h], 001h                 ; f6 46 fc 01
    jne near 04ee0h                           ; 0f 85 28 02
    and bl, 0fbh                              ; 80 e3 fb
    xor bh, bh                                ; 30 ff
    mov dx, strict word 00017h                ; ba 17 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 48 c9
    mov al, byte [bp-004h]                    ; 8a 46 fc
    test AL, strict byte 002h                 ; a8 02
    je short 04cdah                           ; 74 0d
    and AL, strict byte 0fbh                  ; 24 fb
    mov byte [bp-002h], al                    ; 88 46 fe
    movzx bx, al                              ; 0f b6 d8
    mov dx, 00096h                            ; ba 96 00
    jmp short 04ce8h                          ; eb 0e
    mov al, byte [bp-00ch]                    ; 8a 46 f4
    and AL, strict byte 0feh                  ; 24 fe
    mov byte [bp-008h], al                    ; 88 46 f8
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 00018h                ; ba 18 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 20 c9
    jmp near 04ee0h                           ; e9 ef 01
    or bl, 008h                               ; 80 cb 08
    xor bh, bh                                ; 30 ff
    mov dx, strict word 00017h                ; ba 17 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 0f c9
    mov al, byte [bp-004h]                    ; 8a 46 fc
    test AL, strict byte 002h                 ; a8 02
    je short 04d13h                           ; 74 0d
    or AL, strict byte 008h                   ; 0c 08
    mov byte [bp-002h], al                    ; 88 46 fe
    movzx bx, al                              ; 0f b6 d8
    mov dx, 00096h                            ; ba 96 00
    jmp short 04d21h                          ; eb 0e
    mov al, byte [bp-00ch]                    ; 8a 46 f4
    or AL, strict byte 002h                   ; 0c 02
    mov byte [bp-008h], al                    ; 88 46 f8
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 00018h                ; ba 18 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 e7 c8
    jmp near 04ee0h                           ; e9 b6 01
    and bl, 0f7h                              ; 80 e3 f7
    xor bh, bh                                ; 30 ff
    mov dx, strict word 00017h                ; ba 17 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 d6 c8
    mov al, byte [bp-004h]                    ; 8a 46 fc
    test AL, strict byte 002h                 ; a8 02
    je short 04d4ch                           ; 74 0d
    and AL, strict byte 0f7h                  ; 24 f7
    mov byte [bp-002h], al                    ; 88 46 fe
    movzx bx, al                              ; 0f b6 d8
    mov dx, 00096h                            ; ba 96 00
    jmp short 04d5ah                          ; eb 0e
    mov al, byte [bp-00ch]                    ; 8a 46 f4
    and AL, strict byte 0fdh                  ; 24 fd
    mov byte [bp-008h], al                    ; 88 46 f8
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 00018h                ; ba 18 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 ae c8
    jmp near 04ee0h                           ; e9 7d 01
    test byte [bp-004h], 003h                 ; f6 46 fc 03
    jne near 04ee0h                           ; 0f 85 75 01
    mov al, byte [bp-00ch]                    ; 8a 46 f4
    or AL, strict byte 020h                   ; 0c 20
    mov byte [bp-008h], al                    ; 88 46 f8
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 00018h                ; ba 18 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 8f c8
    mov bl, byte [bp-00ah]                    ; 8a 5e f6
    xor bl, 020h                              ; 80 f3 20
    xor bh, bh                                ; 30 ff
    mov dx, strict word 00017h                ; ba 17 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 7e c8
    jmp near 04ee0h                           ; e9 4d 01
    test byte [bp-004h], 003h                 ; f6 46 fc 03
    jne near 04ee0h                           ; 0f 85 45 01
    mov al, byte [bp-00ch]                    ; 8a 46 f4
    and AL, strict byte 0dfh                  ; 24 df
    mov byte [bp-008h], al                    ; 88 46 f8
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 00018h                ; ba 18 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 5f c8
    jmp near 04ee0h                           ; e9 2e 01
    mov al, byte [bp-00ch]                    ; 8a 46 f4
    or AL, strict byte 010h                   ; 0c 10
    mov byte [bp-008h], al                    ; 88 46 f8
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 00018h                ; ba 18 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 48 c8
    mov bl, byte [bp-00ah]                    ; 8a 5e f6
    xor bl, 010h                              ; 80 f3 10
    jmp short 04d85h                          ; eb b7
    mov al, byte [bp-00ch]                    ; 8a 46 f4
    and AL, strict byte 0efh                  ; 24 ef
    jmp short 04da0h                          ; eb cb
    mov al, bl                                ; 88 d8
    and AL, strict byte 00ch                  ; 24 0c
    cmp AL, strict byte 00ch                  ; 3c 0c
    jne short 04de2h                          ; 75 05
    jmp far 0f000h:0e05bh                     ; ea 5b e0 00 f0
    test byte [bp-006h], 080h                 ; f6 46 fa 80
    jne near 04ee0h                           ; 0f 85 f6 00
    cmp byte [bp-006h], 058h                  ; 80 7e fa 58
    jbe short 04e0eh                          ; 76 1e
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 b3 ca
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    push ax                                   ; 50
    push 00511h                               ; 68 11 05
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 e1 ca
    add sp, strict byte 00006h                ; 83 c4 06
    jmp near 04efeh                           ; e9 f0 00
    test bl, 008h                             ; f6 c3 08
    je short 04e25h                           ; 74 12
    movzx si, byte [bp-006h]                  ; 0f b6 76 fa
    imul si, si, strict byte 0000ah           ; 6b f6 0a
    mov dl, byte [si+00d2ch]                  ; 8a 94 2c 0d
    mov ax, word [si+00d2ch]                  ; 8b 84 2c 0d
    jmp near 04eb1h                           ; e9 8c 00
    test bl, 004h                             ; f6 c3 04
    je short 04e3ch                           ; 74 12
    movzx si, byte [bp-006h]                  ; 0f b6 76 fa
    imul si, si, strict byte 0000ah           ; 6b f6 0a
    mov dl, byte [si+00d2ah]                  ; 8a 94 2a 0d
    mov ax, word [si+00d2ah]                  ; 8b 84 2a 0d
    jmp near 04eb1h                           ; e9 75 00
    mov al, byte [bp-002h]                    ; 8a 46 fe
    and AL, strict byte 002h                  ; 24 02
    test al, al                               ; 84 c0
    jbe short 04e5ah                          ; 76 15
    mov al, byte [bp-006h]                    ; 8a 46 fa
    cmp AL, strict byte 047h                  ; 3c 47
    jc short 04e5ah                           ; 72 0e
    cmp AL, strict byte 053h                  ; 3c 53
    jnbe short 04e5ah                         ; 77 0a
    mov DL, strict byte 0e0h                  ; b2 e0
    movzx si, al                              ; 0f b6 f0
    imul si, si, strict byte 0000ah           ; 6b f6 0a
    jmp short 04eadh                          ; eb 53
    test bl, 003h                             ; f6 c3 03
    je short 04e8ch                           ; 74 2d
    movzx si, byte [bp-006h]                  ; 0f b6 76 fa
    imul si, si, strict byte 0000ah           ; 6b f6 0a
    movzx ax, byte [si+00d2eh]                ; 0f b6 84 2e 0d
    movzx dx, bl                              ; 0f b6 d3
    test dx, ax                               ; 85 c2
    je short 04e7ch                           ; 74 0a
    mov dl, byte [si+00d26h]                  ; 8a 94 26 0d
    mov ax, word [si+00d26h]                  ; 8b 84 26 0d
    jmp short 04e84h                          ; eb 08
    mov dl, byte [si+00d28h]                  ; 8a 94 28 0d
    mov ax, word [si+00d28h]                  ; 8b 84 28 0d
    shr ax, 008h                              ; c1 e8 08
    mov byte [bp-006h], al                    ; 88 46 fa
    jmp short 04eb7h                          ; eb 2b
    movzx si, byte [bp-006h]                  ; 0f b6 76 fa
    imul si, si, strict byte 0000ah           ; 6b f6 0a
    movzx ax, byte [si+00d2eh]                ; 0f b6 84 2e 0d
    movzx dx, bl                              ; 0f b6 d3
    test dx, ax                               ; 85 c2
    je short 04ea9h                           ; 74 0a
    mov dl, byte [si+00d28h]                  ; 8a 94 28 0d
    mov ax, word [si+00d28h]                  ; 8b 84 28 0d
    jmp short 04eb1h                          ; eb 08
    mov dl, byte [si+00d26h]                  ; 8a 94 26 0d
    mov ax, word [si+00d26h]                  ; 8b 84 26 0d
    shr ax, 008h                              ; c1 e8 08
    mov byte [bp-006h], al                    ; 88 46 fa
    cmp byte [bp-006h], 000h                  ; 80 7e fa 00
    jne short 04ed7h                          ; 75 1a
    test dl, dl                               ; 84 d2
    jne short 04ed7h                          ; 75 16
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 e2 c9
    push 00548h                               ; 68 48 05
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 15 ca
    add sp, strict byte 00004h                ; 83 c4 04
    xor dh, dh                                ; 30 f6
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    call 04b1ah                               ; e8 3a fc
    mov al, byte [bp-006h]                    ; 8a 46 fa
    and AL, strict byte 07fh                  ; 24 7f
    cmp AL, strict byte 01dh                  ; 3c 1d
    je short 04eedh                           ; 74 04
    and byte [bp-002h], 0feh                  ; 80 66 fe fe
    and byte [bp-002h], 0fdh                  ; 80 66 fe fd
    movzx bx, byte [bp-002h]                  ; 0f b6 5e fe
    mov dx, 00096h                            ; ba 96 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 10 c7
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn                                      ; c3
dequeue_key_:                                ; 0xf4f02 LB 0x90
    push si                                   ; 56
    push di                                   ; 57
    enter 00004h, 000h                        ; c8 04 00 00
    mov di, ax                                ; 89 c7
    mov word [bp-002h], dx                    ; 89 56 fe
    mov si, bx                                ; 89 de
    mov word [bp-004h], cx                    ; 89 4e fc
    mov dx, strict word 0001ah                ; ba 1a 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 01 c7
    mov bx, ax                                ; 89 c3
    mov dx, strict word 0001ch                ; ba 1c 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 f6 c6
    cmp bx, ax                                ; 39 c3
    je short 04f67h                           ; 74 3d
    mov dx, bx                                ; 89 da
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 ce c6
    mov cl, al                                ; 88 c1
    lea dx, [bx+001h]                         ; 8d 57 01
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 c3 c6
    mov es, [bp-004h]                         ; 8e 46 fc
    mov byte [es:si], cl                      ; 26 88 0c
    mov es, [bp-002h]                         ; 8e 46 fe
    mov byte [es:di], al                      ; 26 88 05
    cmp word [bp+008h], strict byte 00000h    ; 83 7e 08 00
    je short 04f62h                           ; 74 13
    inc bx                                    ; 43
    inc bx                                    ; 43
    cmp bx, strict byte 0003eh                ; 83 fb 3e
    jc short 04f59h                           ; 72 03
    mov bx, strict word 0001eh                ; bb 1e 00
    mov dx, strict word 0001ah                ; ba 1a 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0162ah                               ; e8 c8 c6
    mov ax, strict word 00001h                ; b8 01 00
    jmp short 04f69h                          ; eb 02
    xor ax, ax                                ; 31 c0
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn 00002h                               ; c2 02 00
    mov byte [01292h], AL                     ; a2 92 12
    adc word [bx+si], dx                      ; 11 10
    or cl, byte [bx+di]                       ; 0a 09
    add ax, 00102h                            ; 05 02 01
    add byte [04452h], ch                     ; 00 2e 52 44
    push ax                                   ; 50
    mov dx, word [bx+si-029h]                 ; 8b 50 d7
    push ax                                   ; 50
    out 050h, ax                              ; e7 50
    adc word [bx+di+01ah], dx                 ; 11 51 1a
    push cx                                   ; 51
    mov dx, word [bx+di-044h]                 ; 8b 51 bc
    push cx                                   ; 51
    jmp near 072e0h                           ; e9 51 23
    push dx                                   ; 52
    jno short 04fe4h                          ; 71 52
_int16_function:                             ; 0xf4f92 LB 0x314
    push di                                   ; 57
    enter 00006h, 000h                        ; c8 06 00 00
    mov dx, strict word 00017h                ; ba 17 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 60 c6
    mov cl, al                                ; 88 c1
    mov bh, al                                ; 88 c7
    mov dx, 00097h                            ; ba 97 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 53 c6
    mov bl, al                                ; 88 c3
    movzx dx, cl                              ; 0f b6 d1
    sar dx, 004h                              ; c1 fa 04
    and dl, 007h                              ; 80 e2 07
    and AL, strict byte 007h                  ; 24 07
    xor ah, ah                                ; 30 e4
    xor al, dl                                ; 30 d0
    test ax, ax                               ; 85 c0
    je short 05022h                           ; 74 60
    cli                                       ; fa
    mov AL, strict byte 0edh                  ; b0 ed
    mov dx, strict word 00060h                ; ba 60 00
    out DX, AL                                ; ee
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 001h                 ; a8 01
    jne short 04fdbh                          ; 75 08
    mov AL, strict byte 021h                  ; b0 21
    mov dx, 00080h                            ; ba 80 00
    out DX, AL                                ; ee
    jmp short 04fc9h                          ; eb ee
    mov dx, strict word 00060h                ; ba 60 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    cmp ax, 000fah                            ; 3d fa 00
    jne short 05021h                          ; 75 3b
    and bl, 0f8h                              ; 80 e3 f8
    movzx ax, bh                              ; 0f b6 c7
    sar ax, 004h                              ; c1 f8 04
    and ax, strict word 00007h                ; 25 07 00
    movzx cx, bl                              ; 0f b6 cb
    or cx, ax                                 ; 09 c1
    mov bl, cl                                ; 88 cb
    mov al, cl                                ; 88 c8
    and AL, strict byte 007h                  ; 24 07
    out DX, AL                                ; ee
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 001h                 ; a8 01
    jne short 05010h                          ; 75 08
    mov AL, strict byte 021h                  ; b0 21
    mov dx, 00080h                            ; ba 80 00
    out DX, AL                                ; ee
    jmp short 04ffeh                          ; eb ee
    mov dx, strict word 00060h                ; ba 60 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    xor bh, bh                                ; 30 ff
    mov dx, 00097h                            ; ba 97 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 ed c5
    sti                                       ; fb
    mov ax, word [bp+014h]                    ; 8b 46 14
    shr ax, 008h                              ; c1 e8 08
    cmp ax, 000a2h                            ; 3d a2 00
    jnbe near 0522eh                          ; 0f 87 ff 01
    push CS                                   ; 0e
    pop ES                                    ; 07
    mov cx, strict word 0000ch                ; b9 0c 00
    mov di, 04f6fh                            ; bf 6f 4f
    repne scasb                               ; f2 ae
    sal cx, 1                                 ; d1 e1
    mov di, cx                                ; 89 cf
    mov ax, word [cs:di+04f7ah]               ; 2e 8b 85 7a 4f
    jmp ax                                    ; ff e0
    push strict byte 00001h                   ; 6a 01
    mov cx, ss                                ; 8c d1
    lea bx, [bp-006h]                         ; 8d 5e fa
    mov dx, ss                                ; 8c d2
    lea ax, [bp-004h]                         ; 8d 46 fc
    call 04f02h                               ; e8 af fe
    test ax, ax                               ; 85 c0
    jne short 05062h                          ; 75 0b
    push 0057fh                               ; 68 7f 05
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 8a c8
    add sp, strict byte 00004h                ; 83 c4 04
    cmp byte [bp-004h], 000h                  ; 80 7e fc 00
    je short 0506eh                           ; 74 06
    cmp byte [bp-006h], 0f0h                  ; 80 7e fa f0
    je short 05074h                           ; 74 06
    cmp byte [bp-006h], 0e0h                  ; 80 7e fa e0
    jne short 05078h                          ; 75 04
    mov byte [bp-006h], 000h                  ; c6 46 fa 00
    movzx dx, byte [bp-004h]                  ; 0f b6 56 fc
    sal dx, 008h                              ; c1 e2 08
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    or dx, ax                                 ; 09 c2
    mov word [bp+014h], dx                    ; 89 56 14
    leave                                     ; c9
    pop di                                    ; 5f
    retn                                      ; c3
    or word [bp+01eh], 00200h                 ; 81 4e 1e 00 02
    push strict byte 00000h                   ; 6a 00
    mov cx, ss                                ; 8c d1
    lea bx, [bp-006h]                         ; 8d 5e fa
    mov dx, ss                                ; 8c d2
    lea ax, [bp-004h]                         ; 8d 46 fc
    call 04f02h                               ; e8 63 fe
    test ax, ax                               ; 85 c0
    jne short 050aah                          ; 75 07
    or word [bp+01eh], strict byte 00040h     ; 83 4e 1e 40
    leave                                     ; c9
    pop di                                    ; 5f
    retn                                      ; c3
    cmp byte [bp-004h], 000h                  ; 80 7e fc 00
    je short 050b6h                           ; 74 06
    cmp byte [bp-006h], 0f0h                  ; 80 7e fa f0
    je short 050bch                           ; 74 06
    cmp byte [bp-006h], 0e0h                  ; 80 7e fa e0
    jne short 050c0h                          ; 75 04
    mov byte [bp-006h], 000h                  ; c6 46 fa 00
    movzx dx, byte [bp-004h]                  ; 0f b6 56 fc
    sal dx, 008h                              ; c1 e2 08
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    or dx, ax                                 ; 09 c2
    mov word [bp+014h], dx                    ; 89 56 14
    and word [bp+01eh], strict byte 0ffbfh    ; 83 66 1e bf
    leave                                     ; c9
    pop di                                    ; 5f
    retn                                      ; c3
    mov dx, strict word 00017h                ; ba 17 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 20 c5
    mov dx, word [bp+014h]                    ; 8b 56 14
    mov dl, al                                ; 88 c2
    jmp short 05085h                          ; eb 9e
    mov al, byte [bp+012h]                    ; 8a 46 12
    movzx dx, al                              ; 0f b6 d0
    mov ax, word [bp+012h]                    ; 8b 46 12
    shr ax, 008h                              ; c1 e8 08
    xor ah, ah                                ; 30 e4
    call 04b1ah                               ; e8 22 fa
    test ax, ax                               ; 85 c0
    jne short 05109h                          ; 75 0d
    mov ax, word [bp+014h]                    ; 8b 46 14
    xor al, al                                ; 30 c0
    or AL, strict byte 001h                   ; 0c 01
    mov word [bp+014h], ax                    ; 89 46 14
    leave                                     ; c9
    pop di                                    ; 5f
    retn                                      ; c3
    and word [bp+014h], 0ff00h                ; 81 66 14 00 ff
    leave                                     ; c9
    pop di                                    ; 5f
    retn                                      ; c3
    mov ax, word [bp+014h]                    ; 8b 46 14
    xor al, al                                ; 30 c0
    or AL, strict byte 030h                   ; 0c 30
    jmp short 05103h                          ; eb e9
    mov byte [bp-002h], 002h                  ; c6 46 fe 02
    xor cx, cx                                ; 31 c9
    cli                                       ; fa
    mov AL, strict byte 0f2h                  ; b0 f2
    mov dx, strict word 00060h                ; ba 60 00
    out DX, AL                                ; ee
    mov bx, strict word 0ffffh                ; bb ff ff
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 001h                 ; a8 01
    jne short 05141h                          ; 75 0d
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 05141h                          ; 76 08
    mov dx, 00080h                            ; ba 80 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    jmp short 0512ah                          ; eb e9
    test bx, bx                               ; 85 db
    jbe short 05185h                          ; 76 40
    mov dx, strict word 00060h                ; ba 60 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    cmp ax, 000fah                            ; 3d fa 00
    jne short 05185h                          ; 75 35
    mov bx, strict word 0ffffh                ; bb ff ff
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 001h                 ; a8 01
    jne short 0516ah                          ; 75 0d
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 0516ah                          ; 76 08
    mov dx, 00080h                            ; ba 80 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    jmp short 05153h                          ; eb e9
    test bx, bx                               ; 85 db
    jbe short 0517ch                          ; 76 0e
    shr cx, 008h                              ; c1 e9 08
    mov dx, strict word 00060h                ; ba 60 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    sal ax, 008h                              ; c1 e0 08
    or cx, ax                                 ; 09 c1
    dec byte [bp-002h]                        ; fe 4e fe
    cmp byte [bp-002h], 000h                  ; 80 7e fe 00
    jnbe short 05150h                         ; 77 cb
    mov word [bp+00eh], cx                    ; 89 4e 0e
    leave                                     ; c9
    pop di                                    ; 5f
    retn                                      ; c3
    push strict byte 00001h                   ; 6a 01
    mov cx, ss                                ; 8c d1
    lea bx, [bp-006h]                         ; 8d 5e fa
    mov dx, ss                                ; 8c d2
    lea ax, [bp-004h]                         ; 8d 46 fc
    call 04f02h                               ; e8 68 fd
    test ax, ax                               ; 85 c0
    jne short 051a9h                          ; 75 0b
    push 0057fh                               ; 68 7f 05
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 43 c7
    add sp, strict byte 00004h                ; 83 c4 04
    cmp byte [bp-004h], 000h                  ; 80 7e fc 00
    je near 05078h                            ; 0f 84 c7 fe
    cmp byte [bp-006h], 0f0h                  ; 80 7e fa f0
    je near 05074h                            ; 0f 84 bb fe
    jmp near 05078h                           ; e9 bc fe
    or word [bp+01eh], 00200h                 ; 81 4e 1e 00 02
    push strict byte 00000h                   ; 6a 00
    mov cx, ss                                ; 8c d1
    lea bx, [bp-006h]                         ; 8d 5e fa
    mov dx, ss                                ; 8c d2
    lea ax, [bp-004h]                         ; 8d 46 fc
    call 04f02h                               ; e8 32 fd
    test ax, ax                               ; 85 c0
    je near 050a3h                            ; 0f 84 cd fe
    cmp byte [bp-004h], 000h                  ; 80 7e fc 00
    je near 050c0h                            ; 0f 84 e2 fe
    cmp byte [bp-006h], 0f0h                  ; 80 7e fa f0
    je near 050bch                            ; 0f 84 d6 fe
    jmp near 050c0h                           ; e9 d7 fe
    mov dx, strict word 00017h                ; ba 17 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 0e c4
    mov dx, word [bp+014h]                    ; 8b 56 14
    mov dl, al                                ; 88 c2
    mov word [bp+014h], dx                    ; 89 56 14
    mov dx, strict word 00018h                ; ba 18 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 fd c3
    mov bh, al                                ; 88 c7
    and bh, 073h                              ; 80 e7 73
    mov dx, 00096h                            ; ba 96 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 ef c3
    and AL, strict byte 00ch                  ; 24 0c
    or bh, al                                 ; 08 c7
    mov dx, word [bp+014h]                    ; 8b 56 14
    xor dh, dh                                ; 30 f6
    movzx ax, bh                              ; 0f b6 c7
    sal ax, 008h                              ; c1 e0 08
    jmp near 05083h                           ; e9 60 fe
    mov ax, word [bp+014h]                    ; 8b 46 14
    xor ah, ah                                ; 30 e4
    or ah, 080h                               ; 80 cc 80
    jmp near 05103h                           ; e9 d5 fe
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 75 c6
    mov ax, word [bp+014h]                    ; 8b 46 14
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 005a3h                               ; 68 a3 05
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 a1 c6
    add sp, strict byte 00006h                ; 83 c4 06
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 58 c6
    mov ax, word [bp+010h]                    ; 8b 46 10
    push ax                                   ; 50
    mov ax, word [bp+012h]                    ; 8b 46 12
    push ax                                   ; 50
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    push ax                                   ; 50
    mov ax, word [bp+014h]                    ; 8b 46 14
    push ax                                   ; 50
    push 005cbh                               ; 68 cb 05
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 7b c6
    add sp, strict byte 0000ch                ; 83 c4 0c
    leave                                     ; c9
    pop di                                    ; 5f
    retn                                      ; c3
    db  02eh, 053h
    ; cs push bx                                ; 2e 53
    push dx                                   ; 52
    push bx                                   ; 53
    jnle short 052cdh                         ; 7f 53
    jnle short 052cfh                         ; 7f 53
    jnle short 052d1h                         ; 7f 53
    push di                                   ; 57
    push bp                                   ; 55
    test word [bp-07bh], dx                   ; 85 56 85
    push si                                   ; 56
    jo short 052dbh                           ; 70 55
    bound dx, [bp-07bh]                       ; 62 56 85
    push si                                   ; 56
    test word [bp+062h], dx                   ; 85 56 62
    push si                                   ; 56
    bound dx, [bp-07bh]                       ; 62 56 85
    push si                                   ; 56
    test word [bp-01ah], dx                   ; 85 56 e6
    push bp                                   ; 55
    bound dx, [bp-07bh]                       ; 62 56 85
    push si                                   ; 56
    test word [bp+062h], dx                   ; 85 56 62
    push si                                   ; 56
    push SS                                   ; 16
    push si                                   ; 56
    test word [bp-07bh], dx                   ; 85 56 85
    push si                                   ; 56
    db  085h
    push si                                   ; 56
_int13_harddisk:                             ; 0xf52a6 LB 0x43c
    enter 00010h, 000h                        ; c8 10 00 00
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 69 c3
    mov si, 00122h                            ; be 22 01
    mov word [bp-004h], ax                    ; 89 46 fc
    xor bx, bx                                ; 31 db
    mov dx, 0008eh                            ; ba 8e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 4a c3
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    xor ah, ah                                ; 30 e4
    cmp ax, 00080h                            ; 3d 80 00
    jc short 052d3h                           ; 72 05
    cmp ax, 00090h                            ; 3d 90 00
    jc short 052f1h                           ; 72 1e
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    xor ah, ah                                ; 30 e4
    push ax                                   ; 50
    mov ax, word [bp+016h]                    ; 8b 46 16
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 005eeh                               ; 68 ee 05
    push 005fdh                               ; 68 fd 05
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 fe c5
    add sp, strict byte 0000ah                ; 83 c4 0a
    jmp near 056a0h                           ; e9 af 03
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    xor ah, ah                                ; 30 e4
    mov es, [bp-004h]                         ; 8e 46 fc
    mov bx, si                                ; 89 f3
    add bx, ax                                ; 01 c3
    mov dl, byte [es:bx+0011fh]               ; 26 8a 97 1f 01
    mov byte [bp-002h], dl                    ; 88 56 fe
    cmp dl, 010h                              ; 80 fa 10
    jc short 0531ah                           ; 72 10
    push ax                                   ; 50
    mov ax, word [bp+016h]                    ; 8b 46 16
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 005eeh                               ; 68 ee 05
    push 00628h                               ; 68 28 06
    jmp short 052e6h                          ; eb cc
    mov bx, word [bp+016h]                    ; 8b 5e 16
    shr bx, 008h                              ; c1 eb 08
    cmp bx, strict byte 00018h                ; 83 fb 18
    jnbe near 05685h                          ; 0f 87 5e 03
    add bx, bx                                ; 01 db
    jmp word [cs:bx+05274h]                   ; 2e ff a7 74 52
    cmp byte [bp-002h], 008h                  ; 80 7e fe 08
    jnc near 0533dh                           ; 0f 83 07 00
    movzx ax, byte [bp-002h]                  ; 0f b6 46 fe
    call 01be0h                               ; e8 a3 c8
    mov byte [bp+017h], 000h                  ; c6 46 17 00
    xor bx, bx                                ; 31 db
    mov dx, strict word 00074h                ; ba 74 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 c2 c2
    and byte [bp+01ch], 0feh                  ; 80 66 1c fe
    leave                                     ; c9
    retn                                      ; c3
    mov dx, strict word 00074h                ; ba 74 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 a5 c2
    mov cl, al                                ; 88 c1
    mov dx, word [bp+016h]                    ; 8b 56 16
    xor dh, dh                                ; 30 f6
    movzx ax, cl                              ; 0f b6 c1
    sal ax, 008h                              ; c1 e0 08
    or dx, ax                                 ; 09 c2
    mov word [bp+016h], dx                    ; 89 56 16
    xor bx, bx                                ; 31 db
    mov dx, strict word 00074h                ; ba 74 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 96 c2
    test cl, cl                               ; 84 c9
    je short 05341h                           ; 74 c5
    jmp near 056bch                           ; e9 3d 03
    mov ax, word [bp+016h]                    ; 8b 46 16
    xor ah, ah                                ; 30 e4
    mov word [bp-00ch], ax                    ; 89 46 f4
    mov di, word [bp+014h]                    ; 8b 7e 14
    shr di, 008h                              ; c1 ef 08
    mov ax, word [bp+014h]                    ; 8b 46 14
    xor ah, ah                                ; 30 e4
    sal ax, 002h                              ; c1 e0 02
    xor al, al                                ; 30 c0
    and ah, 003h                              ; 80 e4 03
    or di, ax                                 ; 09 c7
    mov ax, word [bp+014h]                    ; 8b 46 14
    and ax, strict word 0003fh                ; 25 3f 00
    mov word [bp-006h], ax                    ; 89 46 fa
    mov ax, word [bp+012h]                    ; 8b 46 12
    shr ax, 008h                              ; c1 e8 08
    mov word [bp-008h], ax                    ; 89 46 f8
    mov ax, word [bp-00ch]                    ; 8b 46 f4
    cmp ax, 00080h                            ; 3d 80 00
    jnbe short 053bah                         ; 77 04
    test ax, ax                               ; 85 c0
    jne short 053ddh                          ; 75 23
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 e9 c4
    mov ax, word [bp+016h]                    ; 8b 46 16
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 005eeh                               ; 68 ee 05
    push 0065ah                               ; 68 5a 06
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 12 c5
    add sp, strict byte 00008h                ; 83 c4 08
    jmp near 056a0h                           ; e9 c3 02
    movzx ax, byte [bp-002h]                  ; 0f b6 46 fe
    imul ax, ax, strict byte 00018h           ; 6b c0 18
    mov es, [bp-004h]                         ; 8e 46 fc
    mov bx, si                                ; 89 f3
    add bx, ax                                ; 01 c3
    mov ax, word [es:bx+028h]                 ; 26 8b 47 28
    mov cx, word [es:bx+026h]                 ; 26 8b 4f 26
    mov dx, word [es:bx+02ah]                 ; 26 8b 57 2a
    mov word [bp-00ah], dx                    ; 89 56 f6
    cmp di, ax                                ; 39 c7
    jnc short 0540ah                          ; 73 0c
    cmp cx, word [bp-008h]                    ; 3b 4e f8
    jbe short 0540ah                          ; 76 07
    mov ax, word [bp-006h]                    ; 8b 46 fa
    cmp ax, dx                                ; 39 d0
    jbe short 05438h                          ; 76 2e
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 99 c4
    push dword [bp-008h]                      ; 66 ff 76 f8
    push di                                   ; 57
    mov ax, word [bp+012h]                    ; 8b 46 12
    xor ah, ah                                ; 30 e4
    push ax                                   ; 50
    mov ax, word [bp+016h]                    ; 8b 46 16
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 005eeh                               ; 68 ee 05
    push 00682h                               ; 68 82 06
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 b7 c4
    add sp, strict byte 00010h                ; 83 c4 10
    jmp near 056a0h                           ; e9 68 02
    mov ax, word [bp+016h]                    ; 8b 46 16
    shr ax, 008h                              ; c1 e8 08
    cmp ax, strict word 00004h                ; 3d 04 00
    jne short 05446h                          ; 75 03
    jmp near 0533dh                           ; e9 f7 fe
    movzx bx, byte [bp-002h]                  ; 0f b6 5e fe
    imul bx, bx, strict byte 00018h           ; 6b db 18
    mov es, [bp-004h]                         ; 8e 46 fc
    add bx, si                                ; 01 f3
    cmp cx, word [es:bx+02ch]                 ; 26 3b 4f 2c
    jne short 05467h                          ; 75 0f
    mov ax, word [es:bx+030h]                 ; 26 8b 47 30
    cmp ax, word [bp-00ah]                    ; 3b 46 f6
    jne short 05467h                          ; 75 06
    cmp byte [bp-002h], 008h                  ; 80 7e fe 08
    jc short 05497h                           ; 72 30
    mov ax, di                                ; 89 f8
    xor dx, dx                                ; 31 d2
    mov bx, cx                                ; 89 cb
    xor cx, cx                                ; 31 c9
    call 08c89h                               ; e8 17 38
    xor bx, bx                                ; 31 db
    add ax, word [bp-008h]                    ; 03 46 f8
    adc dx, bx                                ; 11 da
    mov bx, word [bp-00ah]                    ; 8b 5e f6
    xor cx, cx                                ; 31 c9
    call 08c89h                               ; e8 08 38
    xor bx, bx                                ; 31 db
    add ax, word [bp-006h]                    ; 03 46 fa
    db  011h, 0dah, 005h, 0ffh, 0ffh, 089h, 046h, 0f0h, 083h, 0d2h, 0ffh, 089h, 056h, 0f2h, 089h, 05eh
    db  0fah, 08eh, 046h, 0fch, 066h, 026h, 0c7h, 044h, 014h, 000h, 000h, 000h, 000h, 026h, 0c7h, 044h
    db  018h, 000h, 000h, 08bh, 046h, 0f0h, 026h, 089h, 004h, 08bh, 046h, 0f2h, 026h, 089h, 044h, 002h
    db  08bh, 046h, 010h, 08bh, 056h, 006h, 026h, 089h, 044h, 004h, 026h, 089h, 054h, 006h, 08bh, 046h
    db  0f4h, 026h, 089h, 044h, 00ah, 026h, 0c7h, 044h, 00ch, 000h, 002h, 026h, 089h, 07ch, 00eh, 08bh
    db  046h, 0f8h, 026h, 089h, 044h, 010h, 08bh, 046h, 0fah, 026h, 089h, 044h, 012h, 08ah, 046h, 0feh
    db  026h, 088h, 044h, 008h, 00fh, 0b6h, 046h, 0feh, 06bh, 0c0h, 018h, 089h, 0f3h, 001h, 0c3h, 026h
    db  00fh, 0b6h, 047h, 01eh, 089h, 0c3h, 0c1h, 0e3h, 002h, 08bh, 046h, 016h, 0c1h, 0e8h, 008h, 001h
    db  0c0h, 001h, 0c3h, 006h, 056h, 0ffh, 097h, 0fch, 0ffh, 089h, 0c2h, 08bh, 046h, 016h, 030h, 0c0h
    db  08eh, 046h, 0fch, 026h, 08bh, 05ch, 014h, 009h, 0c3h, 089h, 05eh, 016h, 084h, 0d2h, 00fh, 084h
    db  015h, 0feh, 0bbh, 0d8h, 00ch, 08ch, 0d9h, 0b8h, 004h, 000h, 0e8h, 07bh, 0c3h, 00fh, 0b6h, 0c2h
    db  050h, 08bh, 046h, 016h, 0c1h, 0e8h, 008h, 050h, 068h, 0eeh, 005h, 068h, 0c9h, 006h, 06ah, 004h
    db  0e8h, 0a0h, 0c3h, 083h, 0c4h, 00ah, 08bh, 046h, 016h, 030h, 0e4h, 080h, 0cch, 00ch, 0e9h, 051h
    db  001h, 0bbh, 0d8h, 00ch, 08ch, 0d9h, 0b8h, 004h, 000h, 0e8h, 04ch, 0c3h, 068h, 0eah, 006h, 06ah
    db  004h, 0e8h, 07fh, 0c3h, 083h, 0c4h, 004h, 0e9h, 0cdh, 0fdh, 00fh, 0b6h, 046h, 0feh, 06bh, 0c0h
    db  018h, 08eh, 046h, 0fch, 089h, 0f3h, 001h, 0c3h, 026h, 08bh, 07fh, 028h, 026h, 08bh, 04fh, 026h
    db  026h, 08bh, 047h, 02ah, 089h, 046h, 0f6h, 026h, 00fh, 0b6h, 084h, 09eh, 001h, 089h, 046h, 0f4h
    db  0c6h, 046h, 016h, 000h, 08bh, 056h, 014h, 030h, 0f6h, 04fh, 089h, 0f8h, 030h, 0e4h, 0c1h, 0e0h
    db  008h, 009h, 0c2h, 089h, 056h, 014h, 0c1h, 0efh, 002h, 081h, 0e7h, 0c0h, 000h, 08bh, 046h, 0f6h
    db  030h, 0e4h, 024h, 03fh, 009h, 0c7h, 089h, 0d0h, 030h, 0d0h, 009h, 0f8h, 089h, 046h, 014h, 08bh
    db  056h, 012h, 030h, 0f6h, 089h, 0c8h, 0c1h, 0e0h, 008h, 02dh, 000h, 001h, 009h, 0c2h, 089h, 056h
    db  012h, 089h, 0d0h, 030h, 0d0h, 08bh, 056h, 0f4h, 009h, 0c2h, 089h, 056h, 012h, 0e9h, 057h, 0fdh
    db  00fh, 0b6h, 046h, 0feh, 099h, 02bh, 0c2h, 0d1h, 0f8h, 06bh, 0c0h, 006h, 08eh, 046h, 0fch, 001h
    db  0c6h, 026h, 08bh, 094h, 0c2h, 001h, 083h, 0c2h, 007h, 0ech, 02ah, 0e4h, 024h, 0c0h, 03ch, 040h
    db  075h, 003h, 0e9h, 032h, 0fdh, 08bh, 046h, 016h, 030h, 0e4h, 080h, 0cch, 0aah, 0e9h, 092h, 000h
    db  00fh, 0b6h, 046h, 0feh, 06bh, 0c0h, 018h, 08eh, 046h, 0fch, 001h, 0c6h, 026h, 08bh, 07ch, 02eh
    db  026h, 08bh, 044h, 02ch, 089h, 046h, 0f8h, 026h, 08bh, 044h, 030h, 089h, 046h, 0fah, 089h, 0f8h
    db  031h, 0d2h, 08bh, 05eh, 0f8h, 031h, 0c9h, 0e8h, 049h, 036h, 08bh, 05eh, 0fah, 031h, 0c9h, 0e8h
    db  041h, 036h, 089h, 046h, 0f0h, 089h, 056h, 0f2h, 089h, 056h, 014h, 089h, 046h, 012h, 08bh, 046h
    db  016h, 030h, 0e4h, 080h, 0cch, 003h, 089h, 046h, 016h, 0e9h, 0dfh, 0fch, 0bbh, 0d8h, 00ch, 08ch
    db  0d9h, 0b8h, 004h, 000h, 0e8h, 041h, 0c2h, 08bh, 046h, 016h, 0c1h, 0e8h, 008h, 050h, 068h, 0eeh
    db  005h, 068h, 004h, 007h, 06ah, 004h, 0e8h, 06ah, 0c2h, 083h, 0c4h, 008h, 0e9h, 0b8h, 0fch, 0bbh
    db  0d8h, 00ch, 08ch, 0d9h, 0b8h, 004h, 000h, 0e8h, 01eh, 0c2h, 08bh, 046h, 016h, 0c1h, 0e8h, 008h
    db  050h, 068h, 0eeh, 005h, 068h, 037h, 007h, 0e9h, 032h, 0fdh, 08bh, 046h, 016h, 030h, 0e4h, 080h
    db  0cch, 001h, 089h, 046h, 016h, 08bh, 05eh, 016h, 0c1h, 0ebh, 008h, 030h, 0ffh, 0bah, 074h, 000h
    db  0b8h, 040h, 000h, 0e8h, 052h, 0bfh, 080h, 04eh, 01ch, 001h, 0c9h, 0c3h, 07bh, 057h, 093h, 057h
    db  093h, 057h, 093h, 057h, 042h, 05bh, 0d6h, 058h, 093h, 057h, 0dch, 058h, 042h, 05bh, 08fh, 05bh
    db  08fh, 05bh, 08fh, 05bh, 08fh, 05bh, 057h, 05bh, 08fh, 05bh, 08fh, 05bh
_int13_harddisk_ext:                         ; 0xf56e2 LB 0x4c8
    enter 00028h, 000h                        ; c8 28 00 00
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 2d bf
    mov word [bp-014h], ax                    ; 89 46 ec
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 21 bf
    mov si, 00122h                            ; be 22 01
    mov word [bp-026h], ax                    ; 89 46 da
    xor bx, bx                                ; 31 db
    mov dx, 0008eh                            ; ba 8e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 02 bf
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    xor ah, ah                                ; 30 e4
    cmp ax, 00080h                            ; 3d 80 00
    jc short 0571bh                           ; 72 05
    cmp ax, 00090h                            ; 3d 90 00
    jc short 05739h                           ; 72 1e
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    xor ah, ah                                ; 30 e4
    push ax                                   ; 50
    mov ax, word [bp+016h]                    ; 8b 46 16
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 00765h                               ; 68 65 07
    push 005fdh                               ; 68 fd 05
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 b6 c1
    add sp, strict byte 0000ah                ; 83 c4 0a
    jmp near 05b6dh                           ; e9 34 04
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    xor ah, ah                                ; 30 e4
    mov es, [bp-026h]                         ; 8e 46 da
    mov bx, si                                ; 89 f3
    add bx, ax                                ; 01 c3
    mov cl, byte [es:bx+0011fh]               ; 26 8a 8f 1f 01
    cmp cl, 010h                              ; 80 f9 10
    jc short 0575fh                           ; 72 10
    push ax                                   ; 50
    mov ax, word [bp+016h]                    ; 8b 46 16
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 00765h                               ; 68 65 07
    push 00628h                               ; 68 28 06
    jmp short 0572eh                          ; eb cf
    mov bx, word [bp+016h]                    ; 8b 5e 16
    shr bx, 008h                              ; c1 eb 08
    sub bx, strict byte 00041h                ; 83 eb 41
    cmp bx, strict byte 0000fh                ; 83 fb 0f
    jnbe near 05b8fh                          ; 0f 87 20 04
    add bx, bx                                ; 01 db
    mov ax, word [bp+016h]                    ; 8b 46 16
    xor ah, ah                                ; 30 e4
    jmp word [cs:bx+056c2h]                   ; 2e ff a7 c2 56
    mov word [bp+010h], 0aa55h                ; c7 46 10 55 aa
    mov ax, word [bp+016h]                    ; 8b 46 16
    xor ah, ah                                ; 30 e4
    or ah, 030h                               ; 80 cc 30
    mov word [bp+016h], ax                    ; 89 46 16
    mov word [bp+014h], strict word 00007h    ; c7 46 14 07 00
    jmp near 05b46h                           ; e9 b3 03
    mov bx, word [bp+00ah]                    ; 8b 5e 0a
    mov es, [bp+004h]                         ; 8e 46 04
    mov di, bx                                ; 89 df
    mov [bp-010h], es                         ; 8c 46 f0
    mov ax, word [es:bx+002h]                 ; 26 8b 47 02
    mov word [bp-012h], ax                    ; 89 46 ee
    mov ax, word [es:bx+006h]                 ; 26 8b 47 06
    mov word [bp-018h], ax                    ; 89 46 e8
    mov ax, word [es:bx+004h]                 ; 26 8b 47 04
    mov word [bp-016h], ax                    ; 89 46 ea
    mov ax, word [es:bx+00ch]                 ; 26 8b 47 0c
    mov word [bp-00ah], ax                    ; 89 46 f6
    mov ax, word [es:bx+00eh]                 ; 26 8b 47 0e
    or ax, word [bp-00ah]                     ; 0b 46 f6
    je short 057d4h                           ; 74 11
    mov ax, word [bp+016h]                    ; 8b 46 16
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 00765h                               ; 68 65 07
    push 00778h                               ; 68 78 07
    push strict byte 00007h                   ; 6a 07
    jmp short 0581eh                          ; eb 4a
    mov es, [bp-010h]                         ; 8e 46 f0
    mov ax, word [es:di+008h]                 ; 26 8b 45 08
    mov word [bp-00ah], ax                    ; 89 46 f6
    mov ax, word [es:di+00ah]                 ; 26 8b 45 0a
    movzx dx, cl                              ; 0f b6 d1
    imul dx, dx, strict byte 00018h           ; 6b d2 18
    mov es, [bp-026h]                         ; 8e 46 da
    mov bx, si                                ; 89 f3
    add bx, dx                                ; 01 d3
    mov ch, byte [es:bx+01eh]                 ; 26 8a 6f 1e
    cmp ax, word [es:bx+034h]                 ; 26 3b 47 34
    jnbe short 05804h                         ; 77 0b
    jne short 05827h                          ; 75 2c
    mov dx, word [bp-00ah]                    ; 8b 56 f6
    cmp dx, word [es:bx+032h]                 ; 26 3b 57 32
    jc short 05827h                           ; 72 23
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 9f c0
    mov ax, word [bp+016h]                    ; 8b 46 16
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 00765h                               ; 68 65 07
    push 007a1h                               ; 68 a1 07
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 c8 c0
    add sp, strict byte 00008h                ; 83 c4 08
    jmp near 05b6dh                           ; e9 46 03
    mov dx, word [bp+016h]                    ; 8b 56 16
    shr dx, 008h                              ; c1 ea 08
    mov word [bp-00ch], dx                    ; 89 56 f4
    cmp dx, strict byte 00044h                ; 83 fa 44
    je near 05b42h                            ; 0f 84 0b 03
    cmp dx, strict byte 00047h                ; 83 fa 47
    je near 05b42h                            ; 0f 84 04 03
    mov es, [bp-026h]                         ; 8e 46 da
    db  066h, 026h, 0c7h, 044h, 014h, 000h, 000h, 000h, 000h
    ; mov dword [es:si+014h], strict dword 000000000h ; 66 26 c7 44 14 00 00 00 00
    mov word [es:si+018h], strict word 00000h ; 26 c7 44 18 00 00
    mov dx, word [bp-00ah]                    ; 8b 56 f6
    mov word [es:si], dx                      ; 26 89 14
    mov word [es:si+002h], ax                 ; 26 89 44 02
    mov ax, word [bp-016h]                    ; 8b 46 ea
    mov word [es:si+004h], ax                 ; 26 89 44 04
    mov ax, word [bp-018h]                    ; 8b 46 e8
    mov word [es:si+006h], ax                 ; 26 89 44 06
    mov ax, word [bp-012h]                    ; 8b 46 ee
    mov word [es:si+00ah], ax                 ; 26 89 44 0a
    mov word [es:si+00ch], 00200h             ; 26 c7 44 0c 00 02
    mov word [es:si+012h], strict word 00000h ; 26 c7 44 12 00 00
    mov byte [es:si+008h], cl                 ; 26 88 4c 08
    mov bx, word [bp-00ch]                    ; 8b 5e f4
    add bx, bx                                ; 01 db
    movzx ax, ch                              ; 0f b6 c5
    sal ax, 002h                              ; c1 e0 02
    add bx, ax                                ; 01 c3
    push ES                                   ; 06
    push si                                   ; 56
    call word [bx-00084h]                     ; ff 97 7c ff
    mov dx, ax                                ; 89 c2
    mov es, [bp-026h]                         ; 8e 46 da
    mov ax, word [es:si+014h]                 ; 26 8b 44 14
    mov word [bp-012h], ax                    ; 89 46 ee
    mov es, [bp-010h]                         ; 8e 46 f0
    mov word [es:di+002h], ax                 ; 26 89 45 02
    test dl, dl                               ; 84 d2
    je near 05b42h                            ; 0f 84 97 02
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 f8 bf
    movzx ax, dl                              ; 0f b6 c2
    push ax                                   ; 50
    push word [bp-00ch]                       ; ff 76 f4
    push 00765h                               ; 68 65 07
    push 006c9h                               ; 68 c9 06
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 21 c0
    add sp, strict byte 0000ah                ; 83 c4 0a
    mov ax, word [bp+016h]                    ; 8b 46 16
    xor ah, ah                                ; 30 e4
    or ah, 00ch                               ; 80 cc 0c
    jmp near 05b75h                           ; e9 9f 02
    or ah, 0b2h                               ; 80 cc b2
    jmp near 05b75h                           ; e9 99 02
    mov bx, word [bp+00ah]                    ; 8b 5e 0a
    mov ax, word [bp+004h]                    ; 8b 46 04
    mov word [bp-006h], ax                    ; 89 46 fa
    mov word [bp-004h], ax                    ; 89 46 fc
    mov es, ax                                ; 8e c0
    mov di, bx                                ; 89 df
    mov ax, word [es:di]                      ; 26 8b 05
    mov word [bp-00eh], ax                    ; 89 46 f2
    cmp ax, strict word 0001ah                ; 3d 1a 00
    jc near 05b6dh                            ; 0f 82 74 02
    jc near 05980h                            ; 0f 82 83 00
    movzx ax, cl                              ; 0f b6 c1
    imul ax, ax, strict byte 00018h           ; 6b c0 18
    mov es, [bp-026h]                         ; 8e 46 da
    mov di, si                                ; 89 f7
    add di, ax                                ; 01 c7
    mov ax, word [es:di+02eh]                 ; 26 8b 45 2e
    mov word [bp-028h], ax                    ; 89 46 d8
    mov ax, word [es:di+02ch]                 ; 26 8b 45 2c
    mov word [bp-01eh], ax                    ; 89 46 e2
    mov ax, word [es:di+030h]                 ; 26 8b 45 30
    mov word [bp-024h], ax                    ; 89 46 dc
    mov ax, word [es:di+032h]                 ; 26 8b 45 32
    mov word [bp-00ah], ax                    ; 89 46 f6
    mov ax, word [es:di+034h]                 ; 26 8b 45 34
    mov dx, word [es:di+024h]                 ; 26 8b 55 24
    mov word [bp-022h], dx                    ; 89 56 de
    mov es, [bp-006h]                         ; 8e 46 fa
    mov di, bx                                ; 89 df
    db  066h, 026h, 0c7h, 005h, 01ah, 000h, 002h, 000h
    ; mov dword [es:di], strict dword 00002001ah ; 66 26 c7 05 1a 00 02 00
    mov dx, word [bp-028h]                    ; 8b 56 d8
    mov word [es:di+004h], dx                 ; 26 89 55 04
    mov word [es:di+006h], strict word 00000h ; 26 c7 45 06 00 00
    mov dx, word [bp-01eh]                    ; 8b 56 e2
    mov word [es:di+008h], dx                 ; 26 89 55 08
    mov word [es:di+00ah], strict word 00000h ; 26 c7 45 0a 00 00
    mov dx, word [bp-024h]                    ; 8b 56 dc
    mov word [es:di+00ch], dx                 ; 26 89 55 0c
    mov word [es:di+00eh], strict word 00000h ; 26 c7 45 0e 00 00
    mov dx, word [bp-022h]                    ; 8b 56 de
    mov word [es:di+018h], dx                 ; 26 89 55 18
    mov dx, word [bp-00ah]                    ; 8b 56 f6
    mov word [es:di+010h], dx                 ; 26 89 55 10
    mov word [es:di+012h], ax                 ; 26 89 45 12
    db  066h, 026h, 0c7h, 045h, 014h, 000h, 000h, 000h, 000h
    ; mov dword [es:di+014h], strict dword 000000000h ; 66 26 c7 45 14 00 00 00 00
    cmp word [bp-00eh], strict byte 0001eh    ; 83 7e f2 1e
    jc near 05a8ch                            ; 0f 82 04 01
    mov es, [bp-004h]                         ; 8e 46 fc
    mov word [es:bx], strict word 0001eh      ; 26 c7 07 1e 00
    mov ax, word [bp-014h]                    ; 8b 46 ec
    mov word [es:bx+01ch], ax                 ; 26 89 47 1c
    mov word [es:bx+01ah], 00312h             ; 26 c7 47 1a 12 03
    movzx ax, cl                              ; 0f b6 c1
    mov word [bp-020h], ax                    ; 89 46 e0
    cwd                                       ; 99
    db  02bh, 0c2h
    ; sub ax, dx                                ; 2b c2
    sar ax, 1                                 ; d1 f8
    movzx di, al                              ; 0f b6 f8
    imul di, di, strict byte 00006h           ; 6b ff 06
    mov es, [bp-026h]                         ; 8e 46 da
    add di, si                                ; 01 f7
    mov ax, word [es:di+001c2h]               ; 26 8b 85 c2 01
    mov word [bp-01ah], ax                    ; 89 46 e6
    mov ax, word [es:di+001c4h]               ; 26 8b 85 c4 01
    mov word [bp-01ch], ax                    ; 89 46 e4
    mov ch, byte [es:di+001c1h]               ; 26 8a ad c1 01
    imul di, word [bp-020h], strict byte 00018h ; 6b 7e e0 18
    add di, si                                ; 01 f7
    mov ah, byte [es:di+022h]                 ; 26 8a 65 22
    mov al, byte [es:di+023h]                 ; 26 8a 45 23
    test al, al                               ; 84 c0
    jne short 059deh                          ; 75 04
    xor dx, dx                                ; 31 d2
    jmp short 059e1h                          ; eb 03
    mov dx, strict word 00008h                ; ba 08 00
    or dl, 010h                               ; 80 ca 10
    mov word [bp-008h], dx                    ; 89 56 f8
    cmp ah, 001h                              ; 80 fc 01
    db  00fh, 094h, 0c4h
    ; sete ah                                   ; 0f 94 c4
    movzx dx, ah                              ; 0f b6 d4
    or word [bp-008h], dx                     ; 09 56 f8
    cmp AL, strict byte 001h                  ; 3c 01
    db  00fh, 094h, 0c4h
    ; sete ah                                   ; 0f 94 c4
    movzx dx, ah                              ; 0f b6 d4
    or word [bp-008h], dx                     ; 09 56 f8
    cmp AL, strict byte 003h                  ; 3c 03
    jne short 05a07h                          ; 75 05
    mov ax, strict word 00003h                ; b8 03 00
    jmp short 05a09h                          ; eb 02
    xor ax, ax                                ; 31 c0
    or word [bp-008h], ax                     ; 09 46 f8
    mov ax, word [bp-01ah]                    ; 8b 46 e6
    mov es, [bp-026h]                         ; 8e 46 da
    mov word [es:si+001f0h], ax               ; 26 89 84 f0 01
    mov ax, word [bp-01ch]                    ; 8b 46 e4
    mov word [es:si+001f2h], ax               ; 26 89 84 f2 01
    movzx ax, cl                              ; 0f b6 c1
    cwd                                       ; 99
    mov di, strict word 00002h                ; bf 02 00
    idiv di                                   ; f7 ff
    or dl, 00eh                               ; 80 ca 0e
    sal dx, 004h                              ; c1 e2 04
    mov byte [es:si+001f4h], dl               ; 26 88 94 f4 01
    mov byte [es:si+001f5h], 0cbh             ; 26 c6 84 f5 01 cb
    mov byte [es:si+001f6h], ch               ; 26 88 ac f6 01
    mov word [es:si+001f7h], strict word 00001h ; 26 c7 84 f7 01 01 00
    mov byte [es:si+001f9h], 000h             ; 26 c6 84 f9 01 00
    mov ax, word [bp-008h]                    ; 8b 46 f8
    mov word [es:si+001fah], ax               ; 26 89 84 fa 01
    mov word [es:si+001fch], strict word 00000h ; 26 c7 84 fc 01 00 00
    mov byte [es:si+001feh], 011h             ; 26 c6 84 fe 01 11
    xor ch, ch                                ; 30 ed
    mov byte [bp-002h], ch                    ; 88 6e fe
    jmp short 05a6dh                          ; eb 06
    cmp byte [bp-002h], 00fh                  ; 80 7e fe 0f
    jnc short 05a82h                          ; 73 15
    movzx dx, byte [bp-002h]                  ; 0f b6 56 fe
    add dx, 00312h                            ; 81 c2 12 03
    mov ax, word [bp-014h]                    ; 8b 46 ec
    call 01600h                               ; e8 85 bb
    add ch, al                                ; 00 c5
    inc byte [bp-002h]                        ; fe 46 fe
    jmp short 05a67h                          ; eb e5
    neg ch                                    ; f6 dd
    mov es, [bp-026h]                         ; 8e 46 da
    mov byte [es:si+001ffh], ch               ; 26 88 ac ff 01
    cmp word [bp-00eh], strict byte 00042h    ; 83 7e f2 42
    jc near 05b42h                            ; 0f 82 ae 00
    movzx ax, cl                              ; 0f b6 c1
    cwd                                       ; 99
    db  02bh, 0c2h
    ; sub ax, dx                                ; 2b c2
    sar ax, 1                                 ; d1 f8
    xor ah, ah                                ; 30 e4
    imul ax, ax, strict byte 00006h           ; 6b c0 06
    mov es, [bp-026h]                         ; 8e 46 da
    add si, ax                                ; 01 c6
    mov al, byte [es:si+001c0h]               ; 26 8a 84 c0 01
    mov dx, word [es:si+001c2h]               ; 26 8b 94 c2 01
    mov es, [bp-004h]                         ; 8e 46 fc
    mov word [es:bx], strict word 00042h      ; 26 c7 07 42 00
    db  066h, 026h, 0c7h, 047h, 01eh, 0ddh, 0beh, 024h, 000h
    ; mov dword [es:bx+01eh], strict dword 00024beddh ; 66 26 c7 47 1e dd be 24 00
    mov word [es:bx+022h], strict word 00000h ; 26 c7 47 22 00 00
    test al, al                               ; 84 c0
    jne short 05ad4h                          ; 75 09
    db  066h, 026h, 0c7h, 047h, 024h, 049h, 053h, 041h, 020h
    ; mov dword [es:bx+024h], strict dword 020415349h ; 66 26 c7 47 24 49 53 41 20
    mov es, [bp-004h]                         ; 8e 46 fc
    db  066h, 026h, 0c7h, 047h, 028h, 041h, 054h, 041h, 020h
    ; mov dword [es:bx+028h], strict dword 020415441h ; 66 26 c7 47 28 41 54 41 20
    db  066h, 026h, 0c7h, 047h, 02ch, 020h, 020h, 020h, 020h
    ; mov dword [es:bx+02ch], strict dword 020202020h ; 66 26 c7 47 2c 20 20 20 20
    test al, al                               ; 84 c0
    jne short 05b00h                          ; 75 13
    mov word [es:bx+030h], dx                 ; 26 89 57 30
    db  066h, 026h, 0c7h, 047h, 032h, 000h, 000h, 000h, 000h
    ; mov dword [es:bx+032h], strict dword 000000000h ; 66 26 c7 47 32 00 00 00 00
    mov word [es:bx+036h], strict word 00000h ; 26 c7 47 36 00 00
    mov al, cl                                ; 88 c8
    and AL, strict byte 001h                  ; 24 01
    xor ah, ah                                ; 30 e4
    mov es, [bp-004h]                         ; 8e 46 fc
    mov word [es:bx+038h], ax                 ; 26 89 47 38
    db  066h, 026h, 0c7h, 047h, 03ah, 000h, 000h, 000h, 000h
    ; mov dword [es:bx+03ah], strict dword 000000000h ; 66 26 c7 47 3a 00 00 00 00
    mov word [es:bx+03eh], strict word 00000h ; 26 c7 47 3e 00 00
    xor cl, cl                                ; 30 c9
    mov CH, strict byte 01eh                  ; b5 1e
    jmp short 05b27h                          ; eb 05
    cmp ch, 040h                              ; 80 fd 40
    jnc short 05b39h                          ; 73 12
    movzx dx, ch                              ; 0f b6 d5
    add dx, word [bp+00ah]                    ; 03 56 0a
    mov ax, word [bp+004h]                    ; 8b 46 04
    call 01600h                               ; e8 cd ba
    add cl, al                                ; 00 c1
    db  0feh, 0c5h
    ; inc ch                                    ; fe c5
    jmp short 05b22h                          ; eb e9
    neg cl                                    ; f6 d9
    mov es, [bp-004h]                         ; 8e 46 fc
    mov byte [es:bx+041h], cl                 ; 26 88 4f 41
    mov byte [bp+017h], 000h                  ; c6 46 17 00
    xor bx, bx                                ; 31 db
    mov dx, strict word 00074h                ; ba 74 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 bd ba
    and byte [bp+01ch], 0feh                  ; 80 66 1c fe
    leave                                     ; c9
    retn                                      ; c3
    cmp ax, strict word 00006h                ; 3d 06 00
    je short 05b42h                           ; 74 e6
    cmp ax, strict word 00001h                ; 3d 01 00
    jc short 05b6dh                           ; 72 0c
    jbe short 05b42h                          ; 76 df
    cmp ax, strict word 00003h                ; 3d 03 00
    jc short 05b6dh                           ; 72 05
    cmp ax, strict word 00004h                ; 3d 04 00
    jbe short 05b42h                          ; 76 d5
    mov ax, word [bp+016h]                    ; 8b 46 16
    xor ah, ah                                ; 30 e4
    or ah, 001h                               ; 80 cc 01
    mov word [bp+016h], ax                    ; 89 46 16
    mov bx, word [bp+016h]                    ; 8b 5e 16
    shr bx, 008h                              ; c1 eb 08
    xor bh, bh                                ; 30 ff
    mov dx, strict word 00074h                ; ba 74 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 85 ba
    or byte [bp+01ch], 001h                   ; 80 4e 1c 01
    leave                                     ; c9
    retn                                      ; c3
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 14 bd
    mov ax, word [bp+016h]                    ; 8b 46 16
    shr ax, 008h                              ; c1 e8 08
    push ax                                   ; 50
    push 00765h                               ; 68 65 07
    push 00737h                               ; 68 37 07
    jmp near 0581ch                           ; e9 72 fc
_int14_function:                             ; 0xf5baa LB 0x154
    push si                                   ; 56
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    sti                                       ; fb
    mov dx, word [bp+010h]                    ; 8b 56 10
    add dx, dx                                ; 01 d2
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 62 ba
    mov si, ax                                ; 89 c6
    mov bx, ax                                ; 89 c3
    mov dx, word [bp+010h]                    ; 8b 56 10
    add dx, strict byte 0007ch                ; 83 c2 7c
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 36 ba
    mov cl, al                                ; 88 c1
    cmp word [bp+010h], strict byte 00004h    ; 83 7e 10 04
    jnc near 05cf7h                           ; 0f 83 23 01
    test si, si                               ; 85 f6
    jbe near 05cf7h                           ; 0f 86 1d 01
    mov al, byte [bp+015h]                    ; 8a 46 15
    cmp AL, strict byte 001h                  ; 3c 01
    jc short 05bf2h                           ; 72 11
    jbe short 05c46h                          ; 76 63
    cmp AL, strict byte 003h                  ; 3c 03
    je near 05cdfh                            ; 0f 84 f6 00
    cmp AL, strict byte 002h                  ; 3c 02
    je near 05c94h                            ; 0f 84 a5 00
    jmp near 05cf0h                           ; e9 fe 00
    test al, al                               ; 84 c0
    jne near 05cf0h                           ; 0f 85 f8 00
    lea dx, [bx+003h]                         ; 8d 57 03
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    or AL, strict byte 080h                   ; 0c 80
    out DX, AL                                ; ee
    mov al, byte [bp+014h]                    ; 8a 46 14
    and AL, strict byte 0e0h                  ; 24 e0
    movzx cx, al                              ; 0f b6 c8
    sar cx, 005h                              ; c1 f9 05
    mov ax, 00600h                            ; b8 00 06
    sar ax, CL                                ; d3 f8
    mov dx, bx                                ; 89 da
    out DX, AL                                ; ee
    shr ax, 008h                              ; c1 e8 08
    lea dx, [bx+001h]                         ; 8d 57 01
    out DX, AL                                ; ee
    mov al, byte [bp+014h]                    ; 8a 46 14
    and AL, strict byte 01fh                  ; 24 1f
    lea dx, [bx+003h]                         ; 8d 57 03
    out DX, AL                                ; ee
    lea dx, [bx+005h]                         ; 8d 57 05
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov byte [bp+015h], al                    ; 88 46 15
    lea dx, [bx+006h]                         ; 8d 57 06
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov byte [bp+014h], al                    ; 88 46 14
    jmp near 05cd0h                           ; e9 97 00
    mov AL, strict byte 017h                  ; b0 17
    mov dx, bx                                ; 89 da
    out DX, AL                                ; ee
    lea dx, [bx+001h]                         ; 8d 57 01
    mov AL, strict byte 004h                  ; b0 04
    out DX, AL                                ; ee
    jmp short 05c1bh                          ; eb d5
    mov dx, strict word 0006ch                ; ba 6c 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 cd b9
    mov si, ax                                ; 89 c6
    lea dx, [bx+005h]                         ; 8d 57 05
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    and ax, strict word 00060h                ; 25 60 00
    cmp ax, strict word 00060h                ; 3d 60 00
    je short 05c76h                           ; 74 17
    test cl, cl                               ; 84 c9
    je short 05c76h                           ; 74 13
    mov dx, strict word 0006ch                ; ba 6c 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 b0 b9
    cmp ax, si                                ; 39 f0
    je short 05c51h                           ; 74 e1
    mov si, ax                                ; 89 c6
    db  0feh, 0c9h
    ; dec cl                                    ; fe c9
    jmp short 05c51h                          ; eb db
    test cl, cl                               ; 84 c9
    je short 05c80h                           ; 74 06
    mov al, byte [bp+014h]                    ; 8a 46 14
    mov dx, bx                                ; 89 da
    out DX, AL                                ; ee
    lea dx, [bx+005h]                         ; 8d 57 05
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov byte [bp+015h], al                    ; 88 46 15
    test cl, cl                               ; 84 c9
    jne short 05cd0h                          ; 75 43
    or AL, strict byte 080h                   ; 0c 80
    mov byte [bp+015h], al                    ; 88 46 15
    jmp short 05cd0h                          ; eb 3c
    mov dx, strict word 0006ch                ; ba 6c 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 7f b9
    mov si, ax                                ; 89 c6
    lea dx, [bx+005h]                         ; 8d 57 05
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 001h                 ; a8 01
    jne short 05cc0h                          ; 75 17
    test cl, cl                               ; 84 c9
    je short 05cc0h                           ; 74 13
    mov dx, strict word 0006ch                ; ba 6c 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 66 b9
    cmp ax, si                                ; 39 f0
    je short 05c9fh                           ; 74 e5
    mov si, ax                                ; 89 c6
    db  0feh, 0c9h
    ; dec cl                                    ; fe c9
    jmp short 05c9fh                          ; eb df
    test cl, cl                               ; 84 c9
    je short 05cd7h                           ; 74 13
    mov byte [bp+015h], 000h                  ; c6 46 15 00
    mov dx, bx                                ; 89 da
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov byte [bp+014h], al                    ; 88 46 14
    and byte [bp+01eh], 0feh                  ; 80 66 1e fe
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn                                      ; c3
    lea dx, [bx+005h]                         ; 8d 57 05
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    jmp short 05c8fh                          ; eb b0
    lea dx, [si+005h]                         ; 8d 54 05
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov byte [bp+015h], al                    ; 88 46 15
    lea dx, [si+006h]                         ; 8d 54 06
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    jmp short 05ccdh                          ; eb dd
    or byte [bp+01eh], 001h                   ; 80 4e 1e 01
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn                                      ; c3
    or byte [bp+01eh], 001h                   ; 80 4e 1e 01
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn                                      ; c3
set_enable_a20_:                             ; 0xf5cfe LB 0x29
    push bx                                   ; 53
    push cx                                   ; 51
    push dx                                   ; 52
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bx, ax                                ; 89 c3
    mov dx, 00092h                            ; ba 92 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov cl, al                                ; 88 c1
    test bx, bx                               ; 85 db
    je short 05d17h                           ; 74 05
    or AL, strict byte 002h                   ; 0c 02
    out DX, AL                                ; ee
    jmp short 05d1ah                          ; eb 03
    and AL, strict byte 0fdh                  ; 24 fd
    out DX, AL                                ; ee
    test cl, 002h                             ; f6 c1 02
    db  00fh, 095h, 0c0h
    ; setne al                                  ; 0f 95 c0
    xor ah, ah                                ; 30 e4
    pop bp                                    ; 5d
    pop dx                                    ; 5a
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
set_e820_range_:                             ; 0xf5d27 LB 0x8c
    push si                                   ; 56
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov es, ax                                ; 8e c0
    mov si, dx                                ; 89 d6
    mov word [es:si], bx                      ; 26 89 1c
    mov word [es:si+002h], cx                 ; 26 89 4c 02
    movzx ax, byte [bp+00ah]                  ; 0f b6 46 0a
    mov word [es:si+004h], ax                 ; 26 89 44 04
    mov word [es:si+006h], strict word 00000h ; 26 c7 44 06 00 00
    sub word [bp+006h], bx                    ; 29 5e 06
    sbb word [bp+008h], cx                    ; 19 4e 08
    mov al, byte [bp+00ah]                    ; 8a 46 0a
    sub byte [bp+00ch], al                    ; 28 46 0c
    mov ax, word [bp+006h]                    ; 8b 46 06
    mov word [es:si+008h], ax                 ; 26 89 44 08
    mov ax, word [bp+008h]                    ; 8b 46 08
    mov word [es:si+00ah], ax                 ; 26 89 44 0a
    movzx ax, byte [bp+00ch]                  ; 0f b6 46 0c
    mov word [es:si+00ch], ax                 ; 26 89 44 0c
    mov word [es:si+00eh], strict word 00000h ; 26 c7 44 0e 00 00
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    mov word [es:si+010h], ax                 ; 26 89 44 10
    mov word [es:si+012h], strict word 00000h ; 26 c7 44 12 00 00
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn 0000ah                               ; c2 0a 00
    in AL, DX                                 ; ec
    jmp near 01f5ah                           ; e9 d8 c1
    sar byte [bx-06f6fh], 089h                ; c0 bf 91 90 89
    mov byte [bx+05283h], al                  ; 88 87 83 52
    dec di                                    ; 4f
    inc cx                                    ; 41
    and AL, strict byte 000h                  ; 24 00
    dec si                                    ; 4e
    db  062h
    db  0f1h
    pop bp                                    ; 5d
    add AL, strict byte 05eh                  ; 04 5e
    xchg si, ax                               ; 96
    pop si                                    ; 5e
    pushfw                                    ; 9c
    pop si                                    ; 5e
    mov ax, word [0a65eh]                     ; a1 5e a6
    pop si                                    ; 5e
    dec ax                                    ; 48
    pop di                                    ; 5f
    in ax, 060h                               ; e5 60
    or sp, word [bx+di-06eh]                  ; 0b 61 92
    pop si                                    ; 5e
    xchg dx, ax                               ; 92
    pop si                                    ; 5e
    fsub dword [byte bx+di+000h]              ; d8 61 00
    bound dx, [bp+di]                         ; 62 13
    bound sp, [bp+si]                         ; 62 22
    bound dx, [bp+0295eh]                     ; 62 96 5e 29
    db  062h
_int15_function:                             ; 0xf5db3 LB 0x4c9
    push si                                   ; 56
    push di                                   ; 57
    enter 00004h, 000h                        ; c8 04 00 00
    mov ax, word [bp+016h]                    ; 8b 46 16
    shr ax, 008h                              ; c1 e8 08
    cmp ax, 000ech                            ; 3d ec 00
    jnbe near 0624eh                          ; 0f 87 88 04
    push CS                                   ; 0e
    pop ES                                    ; 07
    mov cx, strict word 00012h                ; b9 12 00
    mov di, 05d7eh                            ; bf 7e 5d
    repne scasb                               ; f2 ae
    sal cx, 1                                 ; d1 e1
    mov di, cx                                ; 89 cf
    mov si, word [cs:di+05d8fh]               ; 2e 8b b5 8f 5d
    mov ax, word [bp+016h]                    ; 8b 46 16
    xor ah, ah                                ; 30 e4
    mov cx, word [bp+01ch]                    ; 8b 4e 1c
    and cl, 0feh                              ; 80 e1 fe
    mov bx, word [bp+01ch]                    ; 8b 5e 1c
    or bl, 001h                               ; 80 cb 01
    mov dx, ax                                ; 89 c2
    or dh, 086h                               ; 80 ce 86
    jmp si                                    ; ff e6
    mov ax, word [bp+016h]                    ; 8b 46 16
    xor ah, ah                                ; 30 e4
    cmp ax, 000c0h                            ; 3d c0 00
    jne near 0624eh                           ; 0f 85 51 04
    or byte [bp+01ch], 001h                   ; 80 4e 1c 01
    jmp near 061f7h                           ; e9 f3 03
    mov dx, ax                                ; 89 c2
    cmp ax, strict word 00001h                ; 3d 01 00
    jc short 05e19h                           ; 72 0e
    jbe short 05e2dh                          ; 76 20
    cmp ax, strict word 00003h                ; 3d 03 00
    je short 05e5ah                           ; 74 48
    cmp ax, strict word 00002h                ; 3d 02 00
    je short 05e3dh                           ; 74 26
    jmp short 05e67h                          ; eb 4e
    test ax, ax                               ; 85 c0
    jne short 05e67h                          ; 75 4a
    xor ax, ax                                ; 31 c0
    call 05cfeh                               ; e8 dc fe
    and byte [bp+01ch], 0feh                  ; 80 66 1c fe
    mov byte [bp+017h], 000h                  ; c6 46 17 00
    jmp near 05e92h                           ; e9 65 00
    mov ax, strict word 00001h                ; b8 01 00
    call 05cfeh                               ; e8 cb fe
    and byte [bp+01ch], 0feh                  ; 80 66 1c fe
    mov byte [bp+017h], dh                    ; 88 76 17
    jmp near 05e92h                           ; e9 55 00
    mov dx, 00092h                            ; ba 92 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    shr ax, 1                                 ; d1 e8
    and ax, strict word 00001h                ; 25 01 00
    mov dx, word [bp+016h]                    ; 8b 56 16
    mov dl, al                                ; 88 c2
    mov word [bp+016h], dx                    ; 89 56 16
    and byte [bp+01ch], 0feh                  ; 80 66 1c fe
    mov byte [bp+017h], ah                    ; 88 66 17
    jmp near 05e92h                           ; e9 38 00
    and byte [bp+01ch], 0feh                  ; 80 66 1c fe
    mov byte [bp+017h], ah                    ; 88 66 17
    mov word [bp+010h], ax                    ; 89 46 10
    jmp near 05e92h                           ; e9 2b 00
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 3c ba
    mov ax, word [bp+016h]                    ; 8b 46 16
    xor ah, ah                                ; 30 e4
    push ax                                   ; 50
    push 007c6h                               ; 68 c6 07
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 69 ba
    add sp, strict byte 00006h                ; 83 c4 06
    or byte [bp+01ch], 001h                   ; 80 4e 1c 01
    mov ax, word [bp+016h]                    ; 8b 46 16
    xor ah, ah                                ; 30 e4
    or ah, 086h                               ; 80 cc 86
    mov word [bp+016h], ax                    ; 89 46 16
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn                                      ; c3
    mov word [bp+01ch], bx                    ; 89 5e 1c
    jmp near 05f42h                           ; e9 a6 00
    mov word [bp+01ch], bx                    ; 89 5e 1c
    jmp short 05e92h                          ; eb f1
    mov word [bp+01ch], cx                    ; 89 4e 1c
    jmp short 05e8fh                          ; eb e9
    test byte [bp+016h], 0ffh                 ; f6 46 16 ff
    je short 05f18h                           ; 74 6c
    mov dx, 000a0h                            ; ba a0 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 4b b7
    test AL, strict byte 001h                 ; a8 01
    jne near 061eeh                           ; 0f 85 33 03
    mov bx, strict word 00001h                ; bb 01 00
    mov dx, 000a0h                            ; ba a0 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 47 b7
    mov bx, word [bp+018h]                    ; 8b 5e 18
    mov dx, 00098h                            ; ba 98 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0162ah                               ; e8 57 b7
    mov bx, word [bp+010h]                    ; 8b 5e 10
    mov dx, 0009ah                            ; ba 9a 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0162ah                               ; e8 4b b7
    mov bx, word [bp+012h]                    ; 8b 5e 12
    mov dx, 0009ch                            ; ba 9c 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0162ah                               ; e8 3f b7
    mov bx, word [bp+014h]                    ; 8b 5e 14
    mov dx, 0009eh                            ; ba 9e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0162ah                               ; e8 33 b7
    and byte [bp+01ch], 0feh                  ; 80 66 1c fe
    mov dx, 000a1h                            ; ba a1 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    and AL, strict byte 0feh                  ; 24 fe
    out DX, AL                                ; ee
    mov ax, strict word 0000bh                ; b8 0b 00
    call 0165ch                               ; e8 52 b7
    or AL, strict byte 040h                   ; 0c 40
    movzx dx, al                              ; 0f b6 d0
    mov ax, strict word 0000bh                ; b8 0b 00
    call 0166dh                               ; e8 58 b7
    jmp near 05e92h                           ; e9 7a ff
    cmp ax, strict word 00001h                ; 3d 01 00
    jne short 05f36h                          ; 75 19
    xor bx, bx                                ; 31 db
    mov dx, 000a0h                            ; ba a0 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 e6 b6
    and byte [bp+01ch], 0feh                  ; 80 66 1c fe
    mov ax, strict word 0000bh                ; b8 0b 00
    call 0165ch                               ; e8 2a b7
    and AL, strict byte 0bfh                  ; 24 bf
    jmp short 05f0ch                          ; eb d6
    mov word [bp+01ch], bx                    ; 89 5e 1c
    mov ax, dx                                ; 89 d0
    xor ah, dh                                ; 30 f4
    xor dl, dl                                ; 30 d2
    dec ax                                    ; 48
    or dx, ax                                 ; 09 c2
    mov word [bp+016h], dx                    ; 89 56 16
    jmp near 05e92h                           ; e9 4a ff
    cli                                       ; fa
    mov ax, strict word 00001h                ; b8 01 00
    call 05cfeh                               ; e8 af fd
    mov di, ax                                ; 89 c7
    mov ax, word [bp+018h]                    ; 8b 46 18
    sal ax, 004h                              ; c1 e0 04
    mov cx, word [bp+00ah]                    ; 8b 4e 0a
    add cx, ax                                ; 01 c1
    mov dx, word [bp+018h]                    ; 8b 56 18
    shr dx, 00ch                              ; c1 ea 0c
    mov byte [bp-002h], dl                    ; 88 56 fe
    cmp cx, ax                                ; 39 c1
    jnc short 05f6eh                          ; 73 05
    db  0feh, 0c2h
    ; inc dl                                    ; fe c2
    mov byte [bp-002h], dl                    ; 88 56 fe
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 00008h                ; 83 c2 08
    mov ax, word [bp+018h]                    ; 8b 46 18
    mov bx, strict word 0002fh                ; bb 2f 00
    call 0162ah                               ; e8 ad b6
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 0000ah                ; 83 c2 0a
    mov ax, word [bp+018h]                    ; 8b 46 18
    mov bx, cx                                ; 89 cb
    call 0162ah                               ; e8 9f b6
    movzx bx, byte [bp-002h]                  ; 0f b6 5e fe
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 0000ch                ; 83 c2 0c
    mov ax, word [bp+018h]                    ; 8b 46 18
    call 0160eh                               ; e8 73 b6
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 0000dh                ; 83 c2 0d
    mov ax, word [bp+018h]                    ; 8b 46 18
    mov bx, 00093h                            ; bb 93 00
    call 0160eh                               ; e8 64 b6
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 0000eh                ; 83 c2 0e
    mov ax, word [bp+018h]                    ; 8b 46 18
    xor bx, bx                                ; 31 db
    call 0162ah                               ; e8 72 b6
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 00020h                ; 83 c2 20
    mov ax, word [bp+018h]                    ; 8b 46 18
    mov bx, strict word 0ffffh                ; bb ff ff
    call 0162ah                               ; e8 63 b6
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 00022h                ; 83 c2 22
    mov ax, word [bp+018h]                    ; 8b 46 18
    xor bx, bx                                ; 31 db
    call 0162ah                               ; e8 55 b6
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 00024h                ; 83 c2 24
    mov ax, word [bp+018h]                    ; 8b 46 18
    mov bx, strict word 0000fh                ; bb 0f 00
    call 0160eh                               ; e8 2a b6
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 00025h                ; 83 c2 25
    mov ax, word [bp+018h]                    ; 8b 46 18
    mov bx, 0009bh                            ; bb 9b 00
    call 0160eh                               ; e8 1b b6
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 00026h                ; 83 c2 26
    mov ax, word [bp+018h]                    ; 8b 46 18
    xor bx, bx                                ; 31 db
    call 0162ah                               ; e8 29 b6
    mov ax, ss                                ; 8c d0
    mov cx, ax                                ; 89 c1
    sal cx, 004h                              ; c1 e1 04
    shr ax, 00ch                              ; c1 e8 0c
    mov word [bp-004h], ax                    ; 89 46 fc
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 00028h                ; 83 c2 28
    mov ax, word [bp+018h]                    ; 8b 46 18
    mov bx, strict word 0ffffh                ; bb ff ff
    call 0162ah                               ; e8 0d b6
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 0002ah                ; 83 c2 2a
    mov ax, word [bp+018h]                    ; 8b 46 18
    mov bx, cx                                ; 89 cb
    call 0162ah                               ; e8 ff b5
    movzx bx, byte [bp-004h]                  ; 0f b6 5e fc
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 0002ch                ; 83 c2 2c
    mov ax, word [bp+018h]                    ; 8b 46 18
    call 0160eh                               ; e8 d3 b5
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 0002dh                ; 83 c2 2d
    mov ax, word [bp+018h]                    ; 8b 46 18
    mov bx, 00093h                            ; bb 93 00
    call 0160eh                               ; e8 c4 b5
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 0002eh                ; 83 c2 2e
    mov ax, word [bp+018h]                    ; 8b 46 18
    xor bx, bx                                ; 31 db
    call 0162ah                               ; e8 d2 b5
    mov si, word [bp+00ah]                    ; 8b 76 0a
    mov es, [bp+018h]                         ; 8e 46 18
    mov cx, word [bp+014h]                    ; 8b 4e 14
    push DS                                   ; 1e
    push eax                                  ; 66 50
    db  066h, 033h, 0c0h
    ; xor eax, eax                              ; 66 33 c0
    mov ds, ax                                ; 8e d8
    mov word [00467h], sp                     ; 89 26 67 04
    mov [00469h], ss                          ; 8c 16 69 04
    call 06074h                               ; e8 00 00
    pop di                                    ; 5f
    add di, strict byte 0001bh                ; 83 c7 1b
    push strict byte 00020h                   ; 6a 20
    push di                                   ; 57
    lgdt [es:si+008h]                         ; 26 0f 01 54 08
    lidt [cs:0efe1h]                          ; 2e 0f 01 1e e1 ef
    mov eax, cr0                              ; 0f 20 c0
    or AL, strict byte 001h                   ; 0c 01
    mov cr0, eax                              ; 0f 22 c0
    retf                                      ; cb
    mov ax, strict word 00028h                ; b8 28 00
    mov ss, ax                                ; 8e d0
    mov ax, strict word 00010h                ; b8 10 00
    mov ds, ax                                ; 8e d8
    mov ax, strict word 00018h                ; b8 18 00
    mov es, ax                                ; 8e c0
    db  033h, 0f6h
    ; xor si, si                                ; 33 f6
    db  033h, 0ffh
    ; xor di, di                                ; 33 ff
    cld                                       ; fc
    rep movsw                                 ; f3 a5
    call 060a8h                               ; e8 00 00
    pop ax                                    ; 58
    push 0f000h                               ; 68 00 f0
    add ax, strict byte 00018h                ; 83 c0 18
    push ax                                   ; 50
    mov ax, strict word 00028h                ; b8 28 00
    mov ds, ax                                ; 8e d8
    mov es, ax                                ; 8e c0
    mov eax, cr0                              ; 0f 20 c0
    and AL, strict byte 0feh                  ; 24 fe
    mov cr0, eax                              ; 0f 22 c0
    retf                                      ; cb
    lidt [cs:0efe7h]                          ; 2e 0f 01 1e e7 ef
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    mov ds, ax                                ; 8e d8
    mov es, ax                                ; 8e c0
    lss sp, [00467h]                          ; 0f b2 26 67 04
    pop eax                                   ; 66 58
    pop DS                                    ; 1f
    mov ax, di                                ; 89 f8
    call 05cfeh                               ; e8 25 fc
    sti                                       ; fb
    mov byte [bp+017h], 000h                  ; c6 46 17 00
    and byte [bp+01ch], 0feh                  ; 80 66 1c fe
    jmp near 05e92h                           ; e9 ad fd
    mov ax, strict word 00031h                ; b8 31 00
    call 0165ch                               ; e8 71 b5
    xor ah, ah                                ; 30 e4
    mov dx, ax                                ; 89 c2
    sal dx, 008h                              ; c1 e2 08
    mov ax, strict word 00030h                ; b8 30 00
    call 0165ch                               ; e8 64 b5
    xor ah, ah                                ; 30 e4
    or dx, ax                                 ; 09 c2
    mov word [bp+016h], dx                    ; 89 56 16
    cmp dx, strict byte 0ffc0h                ; 83 fa c0
    jbe short 060deh                          ; 76 da
    mov word [bp+016h], strict word 0ffc0h    ; c7 46 16 c0 ff
    jmp short 060deh                          ; eb d3
    cli                                       ; fa
    mov ax, strict word 00001h                ; b8 01 00
    call 05cfeh                               ; e8 ec fb
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 00038h                ; 83 c2 38
    mov ax, word [bp+018h]                    ; 8b 46 18
    mov bx, strict word 0ffffh                ; bb ff ff
    call 0162ah                               ; e8 09 b5
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 0003ah                ; 83 c2 3a
    mov ax, word [bp+018h]                    ; 8b 46 18
    xor bx, bx                                ; 31 db
    call 0162ah                               ; e8 fb b4
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 0003ch                ; 83 c2 3c
    mov ax, word [bp+018h]                    ; 8b 46 18
    mov bx, strict word 0000fh                ; bb 0f 00
    call 0160eh                               ; e8 d0 b4
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 0003dh                ; 83 c2 3d
    mov ax, word [bp+018h]                    ; 8b 46 18
    mov bx, 0009bh                            ; bb 9b 00
    call 0160eh                               ; e8 c1 b4
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    add dx, strict byte 0003eh                ; 83 c2 3e
    mov ax, word [bp+018h]                    ; 8b 46 18
    xor bx, bx                                ; 31 db
    call 0162ah                               ; e8 cf b4
    mov AL, strict byte 011h                  ; b0 11
    mov dx, strict word 00020h                ; ba 20 00
    out DX, AL                                ; ee
    mov dx, 000a0h                            ; ba a0 00
    out DX, AL                                ; ee
    mov ax, word [bp+010h]                    ; 8b 46 10
    shr ax, 008h                              ; c1 e8 08
    mov dx, strict word 00021h                ; ba 21 00
    out DX, AL                                ; ee
    mov ax, word [bp+010h]                    ; 8b 46 10
    mov dx, 000a1h                            ; ba a1 00
    out DX, AL                                ; ee
    mov AL, strict byte 004h                  ; b0 04
    mov dx, strict word 00021h                ; ba 21 00
    out DX, AL                                ; ee
    mov AL, strict byte 002h                  ; b0 02
    mov dx, 000a1h                            ; ba a1 00
    out DX, AL                                ; ee
    mov AL, strict byte 001h                  ; b0 01
    mov dx, strict word 00021h                ; ba 21 00
    out DX, AL                                ; ee
    mov dx, 000a1h                            ; ba a1 00
    out DX, AL                                ; ee
    mov AL, strict byte 0ffh                  ; b0 ff
    mov dx, strict word 00021h                ; ba 21 00
    out DX, AL                                ; ee
    mov dx, 000a1h                            ; ba a1 00
    out DX, AL                                ; ee
    mov si, word [bp+00ah]                    ; 8b 76 0a
    call 0619ch                               ; e8 00 00
    pop di                                    ; 5f
    add di, strict byte 00018h                ; 83 c7 18
    push strict byte 00038h                   ; 6a 38
    push di                                   ; 57
    lgdt [es:si+008h]                         ; 26 0f 01 54 08
    lidt [es:si+010h]                         ; 26 0f 01 5c 10
    mov ax, strict word 00001h                ; b8 01 00
    lmsw ax                                   ; 0f 01 f0
    retf                                      ; cb
    mov ax, strict word 00028h                ; b8 28 00
    mov ss, ax                                ; 8e d0
    mov ax, strict word 00018h                ; b8 18 00
    mov ds, ax                                ; 8e d8
    mov ax, strict word 00020h                ; b8 20 00
    mov es, ax                                ; 8e c0
    lea ax, [bp+008h]                         ; 8d 46 08
    db  08bh, 0e0h
    ; mov sp, ax                                ; 8b e0
    popaw                                     ; 61
    add sp, strict byte 00006h                ; 83 c4 06
    pop cx                                    ; 59
    pop ax                                    ; 58
    pop ax                                    ; 58
    mov ax, strict word 00030h                ; b8 30 00
    push ax                                   ; 50
    push cx                                   ; 51
    retf                                      ; cb
    jmp near 05e92h                           ; e9 ba fc
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 cb b6
    push 00806h                               ; 68 06 08
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 fe b6
    add sp, strict byte 00004h                ; 83 c4 04
    or byte [bp+01ch], 001h                   ; 80 4e 1c 01
    mov ax, word [bp+016h]                    ; 8b 46 16
    xor ah, ah                                ; 30 e4
    or ah, 086h                               ; 80 cc 86
    mov word [bp+016h], ax                    ; 89 46 16
    jmp near 05e92h                           ; e9 92 fc
    mov word [bp+01ch], cx                    ; 89 4e 1c
    mov word [bp+016h], ax                    ; 89 46 16
    mov word [bp+010h], 0e6f5h                ; c7 46 10 f5 e6
    mov word [bp+018h], 0f000h                ; c7 46 18 00 f0
    jmp near 05e92h                           ; e9 7f fc
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 00 b4
    mov word [bp+018h], ax                    ; 89 46 18
    jmp near 060deh                           ; e9 bc fe
    push 00835h                               ; 68 35 08
    push strict byte 00008h                   ; 6a 08
    jmp short 061e8h                          ; eb bf
    test byte [bp+016h], 0ffh                 ; f6 46 16 ff
    jne short 0624eh                          ; 75 1f
    mov word [bp+016h], ax                    ; 89 46 16
    mov ax, word [bp+010h]                    ; 8b 46 10
    xor ah, ah                                ; 30 e4
    cmp ax, strict word 00001h                ; 3d 01 00
    jc short 06247h                           ; 72 0b
    cmp ax, strict word 00003h                ; 3d 03 00
    jnbe short 06247h                         ; 77 06
    mov word [bp+01ch], cx                    ; 89 4e 1c
    jmp near 05e92h                           ; e9 4b fc
    or byte [bp+01ch], 001h                   ; 80 4e 1c 01
    jmp near 05e92h                           ; e9 44 fc
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 55 b6
    push word [bp+010h]                       ; ff 76 10
    push word [bp+016h]                       ; ff 76 16
    push 0084ch                               ; 68 4c 08
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 82 b6
    add sp, strict byte 00008h                ; 83 c4 08
    jmp short 061eeh                          ; eb 82
    scasb                                     ; ae
    arpl sp, sp                               ; 63 e4
    arpl word [02764h], ax                    ; 63 06 64 27
    db  064h, 046h
    ; fs inc si                                 ; 64 46
    db  064h, 065h, 064h, 0a7h
    ; fs cmpsw                                  ; 64 65 64 a7
    db  064h, 0d4h, 064h
    ; fs aam 064h                               ; 64 d4 64
_int15_function32:                           ; 0xf627c LB 0x309
    push si                                   ; 56
    enter 00008h, 000h                        ; c8 08 00 00
    mov ax, word [bp+022h]                    ; 8b 46 22
    shr ax, 008h                              ; c1 e8 08
    cmp ax, 000e8h                            ; 3d e8 00
    je short 062cdh                           ; 74 41
    cmp ax, 00086h                            ; 3d 86 00
    jne near 06558h                           ; 0f 85 c5 02
    sti                                       ; fb
    mov ax, word [bp+01eh]                    ; 8b 46 1e
    mov dx, word [bp+01ah]                    ; 8b 56 1a
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c2h
    ; mov ax, dx                                ; 8b c2
    mov ebx, strict dword 00000000fh          ; 66 bb 0f 00 00 00
    db  066h, 033h, 0d2h
    ; xor edx, edx                              ; 66 33 d2
    div ebx                                   ; 66 f7 f3
    db  066h, 08bh, 0c8h
    ; mov ecx, eax                              ; 66 8b c8
    in AL, strict byte 061h                   ; e4 61
    and AL, strict byte 010h                  ; 24 10
    db  08ah, 0e0h
    ; mov ah, al                                ; 8a e0
    db  066h, 00bh, 0c9h
    ; or ecx, ecx                               ; 66 0b c9
    je near 062cah                            ; 0f 84 0e 00
    in AL, strict byte 061h                   ; e4 61
    and AL, strict byte 010h                  ; 24 10
    db  03ah, 0c4h
    ; cmp al, ah                                ; 3a c4
    je short 062bch                           ; 74 f8
    db  08ah, 0e0h
    ; mov ah, al                                ; 8a e0
    dec ecx                                   ; 66 49
    jne short 062bch                          ; 75 f2
    leave                                     ; c9
    pop si                                    ; 5e
    retn                                      ; c3
    mov ax, word [bp+022h]                    ; 8b 46 22
    xor ah, ah                                ; 30 e4
    cmp ax, strict word 00020h                ; 3d 20 00
    je short 062e1h                           ; 74 0a
    cmp ax, strict word 00001h                ; 3d 01 00
    je near 0650ah                            ; 0f 84 2c 02
    jmp near 06558h                           ; e9 77 02
    cmp word [bp+01ch], 0534dh                ; 81 7e 1c 4d 53
    jne near 06558h                           ; 0f 85 6e 02
    cmp word [bp+01ah], 04150h                ; 81 7e 1a 50 41
    jne near 06558h                           ; 0f 85 65 02
    mov ax, strict word 00035h                ; b8 35 00
    call 0165ch                               ; e8 63 b3
    movzx bx, al                              ; 0f b6 d8
    xor dx, dx                                ; 31 d2
    mov cx, strict word 00008h                ; b9 08 00
    sal bx, 1                                 ; d1 e3
    rcl dx, 1                                 ; d1 d2
    loop 06301h                               ; e2 fa
    mov ax, strict word 00034h                ; b8 34 00
    call 0165ch                               ; e8 4f b3
    xor ah, ah                                ; 30 e4
    mov dx, bx                                ; 89 da
    or dx, ax                                 ; 09 c2
    xor bx, bx                                ; 31 db
    add bx, bx                                ; 01 db
    adc dx, 00100h                            ; 81 d2 00 01
    cmp dx, 00100h                            ; 81 fa 00 01
    jc short 06327h                           ; 72 06
    jne short 06354h                          ; 75 31
    test bx, bx                               ; 85 db
    jnbe short 06354h                         ; 77 2d
    mov ax, strict word 00031h                ; b8 31 00
    call 0165ch                               ; e8 2f b3
    movzx bx, al                              ; 0f b6 d8
    xor dx, dx                                ; 31 d2
    mov cx, strict word 00008h                ; b9 08 00
    sal bx, 1                                 ; d1 e3
    rcl dx, 1                                 ; d1 d2
    loop 06335h                               ; e2 fa
    mov ax, strict word 00030h                ; b8 30 00
    call 0165ch                               ; e8 1b b3
    xor ah, ah                                ; 30 e4
    or bx, ax                                 ; 09 c3
    mov cx, strict word 0000ah                ; b9 0a 00
    sal bx, 1                                 ; d1 e3
    rcl dx, 1                                 ; d1 d2
    loop 06348h                               ; e2 fa
    add bx, strict byte 00000h                ; 83 c3 00
    adc dx, strict byte 00010h                ; 83 d2 10
    mov ax, strict word 00062h                ; b8 62 00
    call 0165ch                               ; e8 02 b3
    xor ah, ah                                ; 30 e4
    mov word [bp-008h], ax                    ; 89 46 f8
    xor al, al                                ; 30 c0
    mov word [bp-006h], ax                    ; 89 46 fa
    mov cx, strict word 00008h                ; b9 08 00
    sal word [bp-008h], 1                     ; d1 66 f8
    rcl word [bp-006h], 1                     ; d1 56 fa
    loop 06367h                               ; e2 f8
    mov ax, strict word 00061h                ; b8 61 00
    call 0165ch                               ; e8 e7 b2
    xor ah, ah                                ; 30 e4
    or word [bp-008h], ax                     ; 09 46 f8
    mov ax, word [bp-008h]                    ; 8b 46 f8
    mov word [bp-006h], ax                    ; 89 46 fa
    mov word [bp-008h], strict word 00000h    ; c7 46 f8 00 00
    mov ax, strict word 00063h                ; b8 63 00
    call 0165ch                               ; e8 d1 b2
    mov byte [bp-004h], al                    ; 88 46 fc
    mov byte [bp-002h], al                    ; 88 46 fe
    mov ax, word [bp+016h]                    ; 8b 46 16
    cmp ax, strict word 00007h                ; 3d 07 00
    jnbe near 06558h                          ; 0f 87 bd 01
    mov si, ax                                ; 89 c6
    add si, ax                                ; 01 c6
    mov ax, bx                                ; 89 d8
    add ax, strict word 00000h                ; 05 00 00
    mov cx, dx                                ; 89 d1
    adc cx, strict byte 0ffffh                ; 83 d1 ff
    jmp word [cs:si+0626ch]                   ; 2e ff a4 6c 62
    push strict byte 00001h                   ; 6a 01
    push dword 000000000h                     ; 66 6a 00
    push strict byte 00009h                   ; 6a 09
    push 0fc00h                               ; 68 00 fc
    mov dx, word [bp+006h]                    ; 8b 56 06
    mov ax, word [bp+026h]                    ; 8b 46 26
    xor bx, bx                                ; 31 db
    xor cx, cx                                ; 31 c9
    call 05d27h                               ; e8 62 f9
    mov dword [bp+016h], strict dword 000000001h ; 66 c7 46 16 01 00 00 00
    mov dword [bp+022h], strict dword 0534d4150h ; 66 c7 46 22 50 41 4d 53
    mov dword [bp+01eh], strict dword 000000014h ; 66 c7 46 1e 14 00 00 00
    and byte [bp+02ah], 0feh                  ; 80 66 2a fe
    leave                                     ; c9
    pop si                                    ; 5e
    retn                                      ; c3
    push strict byte 00002h                   ; 6a 02
    push dword 000000000h                     ; 66 6a 00
    push strict byte 0000ah                   ; 6a 0a
    push strict byte 00000h                   ; 6a 00
    mov dx, word [bp+006h]                    ; 8b 56 06
    mov ax, word [bp+026h]                    ; 8b 46 26
    mov bx, 0fc00h                            ; bb 00 fc
    mov cx, strict word 00009h                ; b9 09 00
    call 05d27h                               ; e8 2b f9
    mov dword [bp+016h], strict dword 000000002h ; 66 c7 46 16 02 00 00 00
    jmp short 063cdh                          ; eb c7
    push strict byte 00002h                   ; 6a 02
    push dword 000000000h                     ; 66 6a 00
    push strict byte 00010h                   ; 6a 10
    push strict byte 00000h                   ; 6a 00
    mov dx, word [bp+006h]                    ; 8b 56 06
    mov ax, word [bp+026h]                    ; 8b 46 26
    xor bx, bx                                ; 31 db
    mov cx, strict word 0000fh                ; b9 0f 00
    call 05d27h                               ; e8 0a f9
    mov dword [bp+016h], strict dword 000000003h ; 66 c7 46 16 03 00 00 00
    jmp short 063cdh                          ; eb a6
    push strict byte 00001h                   ; 6a 01
    push dword 000000000h                     ; 66 6a 00
    push cx                                   ; 51
    push ax                                   ; 50
    mov dx, word [bp+006h]                    ; 8b 56 06
    mov ax, word [bp+026h]                    ; 8b 46 26
    xor bx, bx                                ; 31 db
    mov cx, strict word 00010h                ; b9 10 00
    call 05d27h                               ; e8 eb f8
    mov dword [bp+016h], strict dword 000000004h ; 66 c7 46 16 04 00 00 00
    jmp short 063cdh                          ; eb 87
    push strict byte 00003h                   ; 6a 03
    push dword 000000000h                     ; 66 6a 00
    push dx                                   ; 52
    push bx                                   ; 53
    mov dx, word [bp+006h]                    ; 8b 56 06
    mov si, word [bp+026h]                    ; 8b 76 26
    mov bx, ax                                ; 89 c3
    mov ax, si                                ; 89 f0
    call 05d27h                               ; e8 cd f8
    mov dword [bp+016h], strict dword 000000005h ; 66 c7 46 16 05 00 00 00
    jmp near 063cdh                           ; e9 68 ff
    push strict byte 00002h                   ; 6a 02
    push dword 000000000h                     ; 66 6a 00
    push dword 000000000h                     ; 66 6a 00
    mov dx, word [bp+006h]                    ; 8b 56 06
    mov ax, word [bp+026h]                    ; 8b 46 26
    xor bx, bx                                ; 31 db
    mov cx, strict word 0fffch                ; b9 fc ff
    call 05d27h                               ; e8 ac f8
    cmp byte [bp-004h], 000h                  ; 80 7e fc 00
    jne short 06488h                          ; 75 07
    mov ax, word [bp-006h]                    ; 8b 46 fa
    test ax, ax                               ; 85 c0
    je short 0649eh                           ; 74 16
    mov dword [bp+016h], strict dword 000000007h ; 66 c7 46 16 07 00 00 00
    jmp near 063cdh                           ; e9 3a ff
    mov dword [bp+016h], strict dword 000000006h ; 66 c7 46 16 06 00 00 00
    jmp near 063cdh                           ; e9 2f ff
    mov word [bp+016h], ax                    ; 89 46 16
    mov word [bp+018h], ax                    ; 89 46 18
    jmp near 063cdh                           ; e9 26 ff
    push strict byte 00002h                   ; 6a 02
    push dword 000000000h                     ; 66 6a 00
    push dword 000000000h                     ; 66 6a 00
    mov dx, word [bp+006h]                    ; 8b 56 06
    mov ax, word [bp+026h]                    ; 8b 46 26
    xor bx, bx                                ; 31 db
    xor cx, cx                                ; 31 c9
    call 05d27h                               ; e8 6b f8
    cmp byte [bp-004h], 000h                  ; 80 7e fc 00
    jne short 064c9h                          ; 75 07
    mov ax, word [bp-006h]                    ; 8b 46 fa
    test ax, ax                               ; 85 c0
    je short 064cbh                           ; 74 02
    jmp short 06488h                          ; eb bd
    mov word [bp+016h], ax                    ; 89 46 16
    mov word [bp+018h], ax                    ; 89 46 18
    jmp near 063cdh                           ; e9 f9 fe
    cmp byte [bp-004h], 000h                  ; 80 7e fc 00
    jne short 064e2h                          ; 75 08
    cmp word [bp-006h], strict byte 00000h    ; 83 7e fa 00
    je near 063cdh                            ; 0f 84 eb fe
    push strict byte 00001h                   ; 6a 01
    mov al, byte [bp-002h]                    ; 8a 46 fe
    db  0feh, 0c0h
    ; inc al                                    ; fe c0
    xor ah, ah                                ; 30 e4
    push ax                                   ; 50
    push strict byte 00001h                   ; 6a 01
    push dword [bp-008h]                      ; 66 ff 76 f8
    mov dx, word [bp+006h]                    ; 8b 56 06
    mov ax, word [bp+026h]                    ; 8b 46 26
    xor bx, bx                                ; 31 db
    xor cx, cx                                ; 31 c9
    call 05d27h                               ; e8 28 f8
    xor ax, ax                                ; 31 c0
    mov word [bp+016h], ax                    ; 89 46 16
    mov word [bp+018h], ax                    ; 89 46 18
    jmp near 063cdh                           ; e9 c3 fe
    and byte [bp+02ah], 0feh                  ; 80 66 2a fe
    mov ax, strict word 00031h                ; b8 31 00
    call 0165ch                               ; e8 48 b1
    xor ah, ah                                ; 30 e4
    mov dx, ax                                ; 89 c2
    sal dx, 008h                              ; c1 e2 08
    mov ax, strict word 00030h                ; b8 30 00
    call 0165ch                               ; e8 3b b1
    xor ah, ah                                ; 30 e4
    or dx, ax                                 ; 09 c2
    mov word [bp+01eh], dx                    ; 89 56 1e
    cmp dx, 03c00h                            ; 81 fa 00 3c
    jbe short 06533h                          ; 76 05
    mov word [bp+01eh], 03c00h                ; c7 46 1e 00 3c
    mov ax, strict word 00035h                ; b8 35 00
    call 0165ch                               ; e8 23 b1
    movzx dx, al                              ; 0f b6 d0
    sal dx, 008h                              ; c1 e2 08
    mov ax, strict word 00034h                ; b8 34 00
    call 0165ch                               ; e8 17 b1
    xor ah, ah                                ; 30 e4
    or dx, ax                                 ; 09 c2
    mov word [bp+01ah], dx                    ; 89 56 1a
    mov ax, word [bp+01eh]                    ; 8b 46 1e
    mov word [bp+022h], ax                    ; 89 46 22
    mov word [bp+016h], dx                    ; 89 56 16
    leave                                     ; c9
    pop si                                    ; 5e
    retn                                      ; c3
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 4b b3
    push word [bp+016h]                       ; ff 76 16
    push word [bp+022h]                       ; ff 76 22
    push 0084ch                               ; 68 4c 08
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 78 b3
    add sp, strict byte 00008h                ; 83 c4 08
    or byte [bp+02ah], 001h                   ; 80 4e 2a 01
    mov ax, word [bp+022h]                    ; 8b 46 22
    xor al, al                                ; 30 c0
    or AL, strict byte 086h                   ; 0c 86
    mov word [bp+022h], ax                    ; 89 46 22
    leave                                     ; c9
    pop si                                    ; 5e
    retn                                      ; c3
init_rtc_:                                   ; 0xf6585 LB 0x25
    push dx                                   ; 52
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov dx, strict word 00026h                ; ba 26 00
    mov ax, strict word 0000ah                ; b8 0a 00
    call 0166dh                               ; e8 db b0
    mov dx, strict word 00002h                ; ba 02 00
    mov ax, strict word 0000bh                ; b8 0b 00
    call 0166dh                               ; e8 d2 b0
    mov ax, strict word 0000ch                ; b8 0c 00
    call 0165ch                               ; e8 bb b0
    mov ax, strict word 0000dh                ; b8 0d 00
    call 0165ch                               ; e8 b5 b0
    pop bp                                    ; 5d
    pop dx                                    ; 5a
    retn                                      ; c3
rtc_updating_:                               ; 0xf65aa LB 0x1f
    push dx                                   ; 52
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov dx, 061a8h                            ; ba a8 61
    dec dx                                    ; 4a
    je short 065c3h                           ; 74 0f
    mov ax, strict word 0000ah                ; b8 0a 00
    call 0165ch                               ; e8 a2 b0
    test AL, strict byte 080h                 ; a8 80
    jne short 065b1h                          ; 75 f3
    xor ax, ax                                ; 31 c0
    pop bp                                    ; 5d
    pop dx                                    ; 5a
    retn                                      ; c3
    mov ax, strict word 00001h                ; b8 01 00
    pop bp                                    ; 5d
    pop dx                                    ; 5a
    retn                                      ; c3
_int70_function:                             ; 0xf65c9 LB 0xbb
    push si                                   ; 56
    enter 00002h, 000h                        ; c8 02 00 00
    mov ax, strict word 0000bh                ; b8 0b 00
    call 0165ch                               ; e8 88 b0
    mov dl, al                                ; 88 c2
    mov byte [bp-002h], al                    ; 88 46 fe
    mov ax, strict word 0000ch                ; b8 0c 00
    call 0165ch                               ; e8 7d b0
    mov dh, al                                ; 88 c6
    test dl, 060h                             ; f6 c2 60
    je near 0666eh                            ; 0f 84 86 00
    test AL, strict byte 020h                 ; a8 20
    je short 065f0h                           ; 74 04
    sti                                       ; fb
    int 04ah                                  ; cd 4a
    cli                                       ; fa
    test dh, 040h                             ; f6 c6 40
    je near 0666eh                            ; 0f 84 77 00
    mov dx, 000a0h                            ; ba a0 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 00 b0
    test al, al                               ; 84 c0
    je short 0666eh                           ; 74 6a
    mov dx, 0009ch                            ; ba 9c 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01638h                               ; e8 2b b0
    test dx, dx                               ; 85 d2
    jne short 0665ah                          ; 75 49
    cmp ax, 003d1h                            ; 3d d1 03
    jnc short 0665ah                          ; 73 44
    mov dx, 00098h                            ; ba 98 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 fd af
    mov si, ax                                ; 89 c6
    mov dx, 0009ah                            ; ba 9a 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 f2 af
    mov cx, ax                                ; 89 c1
    xor bx, bx                                ; 31 db
    mov dx, 000a0h                            ; ba a0 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 d7 af
    mov al, byte [bp-002h]                    ; 8a 46 fe
    and AL, strict byte 037h                  ; 24 37
    movzx dx, al                              ; 0f b6 d0
    mov ax, strict word 0000bh                ; b8 0b 00
    call 0166dh                               ; e8 28 b0
    mov dx, cx                                ; 89 ca
    mov ax, si                                ; 89 f0
    call 01600h                               ; e8 b4 af
    or AL, strict byte 080h                   ; 0c 80
    movzx bx, al                              ; 0f b6 d8
    mov dx, cx                                ; 89 ca
    mov ax, si                                ; 89 f0
    call 0160eh                               ; e8 b6 af
    jmp short 0666eh                          ; eb 14
    mov bx, ax                                ; 89 c3
    add bx, 0fc2fh                            ; 81 c3 2f fc
    mov cx, dx                                ; 89 d1
    adc cx, strict byte 0ffffh                ; 83 d1 ff
    mov dx, 0009ch                            ; ba 9c 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0164ah                               ; e8 dc af
    call 0e03bh                               ; e8 ca 79
    leave                                     ; c9
    pop si                                    ; 5e
    retn                                      ; c3
    db  09bh, 066h, 0c2h, 066h, 0e7h, 066h, 019h, 067h, 067h, 067h, 09dh, 067h, 0dfh, 067h, 034h, 068h
_int1a_function:                             ; 0xf6684 LB 0x1c0
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    sti                                       ; fb
    mov al, byte [bp+013h]                    ; 8a 46 13
    cmp AL, strict byte 007h                  ; 3c 07
    jnbe near 066c0h                          ; 0f 87 2f 00
    movzx bx, al                              ; 0f b6 d8
    add bx, bx                                ; 01 db
    jmp word [cs:bx+06674h]                   ; 2e ff a7 74 66
    cli                                       ; fa
    mov bx, 0046eh                            ; bb 6e 04
    xor ax, ax                                ; 31 c0
    mov es, ax                                ; 8e c0
    mov ax, word [es:bx]                      ; 26 8b 07
    mov word [bp+010h], ax                    ; 89 46 10
    mov bx, 0046ch                            ; bb 6c 04
    mov ax, word [es:bx]                      ; 26 8b 07
    mov word [bp+00eh], ax                    ; 89 46 0e
    mov bx, 00470h                            ; bb 70 04
    mov al, byte [es:bx]                      ; 26 8a 07
    mov byte [bp+012h], al                    ; 88 46 12
    mov byte [es:bx], 000h                    ; 26 c6 07 00
    sti                                       ; fb
    pop bp                                    ; 5d
    retn                                      ; c3
    cli                                       ; fa
    mov bx, 0046eh                            ; bb 6e 04
    xor ax, ax                                ; 31 c0
    mov es, ax                                ; 8e c0
    mov ax, word [bp+010h]                    ; 8b 46 10
    mov word [es:bx], ax                      ; 26 89 07
    mov bx, 0046ch                            ; bb 6c 04
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    mov word [es:bx], ax                      ; 26 89 07
    mov bx, 00470h                            ; bb 70 04
    mov byte [es:bx], 000h                    ; 26 c6 07 00
    sti                                       ; fb
    mov byte [bp+013h], 000h                  ; c6 46 13 00
    pop bp                                    ; 5d
    retn                                      ; c3
    call 065aah                               ; e8 c0 fe
    test ax, ax                               ; 85 c0
    je short 066f0h                           ; 74 02
    pop bp                                    ; 5d
    retn                                      ; c3
    xor ax, ax                                ; 31 c0
    call 0165ch                               ; e8 67 af
    mov byte [bp+00fh], al                    ; 88 46 0f
    mov ax, strict word 00002h                ; b8 02 00
    call 0165ch                               ; e8 5e af
    mov byte [bp+010h], al                    ; 88 46 10
    mov ax, strict word 00004h                ; b8 04 00
    call 0165ch                               ; e8 55 af
    mov bl, al                                ; 88 c3
    mov byte [bp+011h], al                    ; 88 46 11
    mov ax, strict word 0000bh                ; b8 0b 00
    call 0165ch                               ; e8 4a af
    and AL, strict byte 001h                  ; 24 01
    mov byte [bp+00eh], al                    ; 88 46 0e
    jmp short 0675eh                          ; eb 45
    call 065aah                               ; e8 8e fe
    test ax, ax                               ; 85 c0
    je short 06723h                           ; 74 03
    call 06585h                               ; e8 62 fe
    movzx dx, byte [bp+00fh]                  ; 0f b6 56 0f
    xor ax, ax                                ; 31 c0
    call 0166dh                               ; e8 41 af
    movzx dx, byte [bp+010h]                  ; 0f b6 56 10
    mov ax, strict word 00002h                ; b8 02 00
    call 0166dh                               ; e8 37 af
    movzx dx, byte [bp+011h]                  ; 0f b6 56 11
    mov ax, strict word 00004h                ; b8 04 00
    call 0166dh                               ; e8 2d af
    mov ax, strict word 0000bh                ; b8 0b 00
    call 0165ch                               ; e8 16 af
    mov bl, al                                ; 88 c3
    and bl, 060h                              ; 80 e3 60
    or bl, 002h                               ; 80 cb 02
    mov al, byte [bp+00eh]                    ; 8a 46 0e
    and AL, strict byte 001h                  ; 24 01
    or bl, al                                 ; 08 c3
    movzx dx, bl                              ; 0f b6 d3
    mov ax, strict word 0000bh                ; b8 0b 00
    call 0166dh                               ; e8 0f af
    mov byte [bp+013h], 000h                  ; c6 46 13 00
    mov byte [bp+012h], bl                    ; 88 5e 12
    pop bp                                    ; 5d
    retn                                      ; c3
    mov byte [bp+013h], 000h                  ; c6 46 13 00
    call 065aah                               ; e8 3c fe
    test ax, ax                               ; 85 c0
    je short 06774h                           ; 74 02
    pop bp                                    ; 5d
    retn                                      ; c3
    mov ax, strict word 00009h                ; b8 09 00
    call 0165ch                               ; e8 e2 ae
    mov byte [bp+010h], al                    ; 88 46 10
    mov ax, strict word 00008h                ; b8 08 00
    call 0165ch                               ; e8 d9 ae
    mov byte [bp+00fh], al                    ; 88 46 0f
    mov ax, strict word 00007h                ; b8 07 00
    call 0165ch                               ; e8 d0 ae
    mov byte [bp+00eh], al                    ; 88 46 0e
    mov ax, strict word 00032h                ; b8 32 00
    call 0165ch                               ; e8 c7 ae
    mov byte [bp+011h], al                    ; 88 46 11
    mov byte [bp+012h], al                    ; 88 46 12
    pop bp                                    ; 5d
    retn                                      ; c3
    call 065aah                               ; e8 0a fe
    test ax, ax                               ; 85 c0
    je short 067a9h                           ; 74 05
    call 06585h                               ; e8 de fd
    pop bp                                    ; 5d
    retn                                      ; c3
    movzx dx, byte [bp+010h]                  ; 0f b6 56 10
    mov ax, strict word 00009h                ; b8 09 00
    call 0166dh                               ; e8 ba ae
    movzx dx, byte [bp+00fh]                  ; 0f b6 56 0f
    mov ax, strict word 00008h                ; b8 08 00
    call 0166dh                               ; e8 b0 ae
    movzx dx, byte [bp+00eh]                  ; 0f b6 56 0e
    mov ax, strict word 00007h                ; b8 07 00
    call 0166dh                               ; e8 a6 ae
    movzx dx, byte [bp+011h]                  ; 0f b6 56 11
    mov ax, strict word 00032h                ; b8 32 00
    call 0166dh                               ; e8 9c ae
    mov ax, strict word 0000bh                ; b8 0b 00
    call 0165ch                               ; e8 85 ae
    mov bl, al                                ; 88 c3
    and bl, 07fh                              ; 80 e3 7f
    jmp near 06755h                           ; e9 76 ff
    mov ax, strict word 0000bh                ; b8 0b 00
    call 0165ch                               ; e8 77 ae
    mov bl, al                                ; 88 c3
    mov word [bp+012h], strict word 00000h    ; c7 46 12 00 00
    test AL, strict byte 020h                 ; a8 20
    je short 067f2h                           ; 74 02
    pop bp                                    ; 5d
    retn                                      ; c3
    call 065aah                               ; e8 b5 fd
    test ax, ax                               ; 85 c0
    je short 067fch                           ; 74 03
    call 06585h                               ; e8 89 fd
    movzx dx, byte [bp+00fh]                  ; 0f b6 56 0f
    mov ax, strict word 00001h                ; b8 01 00
    call 0166dh                               ; e8 67 ae
    movzx dx, byte [bp+010h]                  ; 0f b6 56 10
    mov ax, strict word 00003h                ; b8 03 00
    call 0166dh                               ; e8 5d ae
    movzx dx, byte [bp+011h]                  ; 0f b6 56 11
    mov ax, strict word 00005h                ; b8 05 00
    call 0166dh                               ; e8 53 ae
    mov dx, 000a1h                            ; ba a1 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    and AL, strict byte 0feh                  ; 24 fe
    out DX, AL                                ; ee
    mov al, bl                                ; 88 d8
    and AL, strict byte 05fh                  ; 24 5f
    or AL, strict byte 020h                   ; 0c 20
    movzx dx, al                              ; 0f b6 d0
    mov ax, strict word 0000bh                ; b8 0b 00
    call 0166dh                               ; e8 3b ae
    pop bp                                    ; 5d
    retn                                      ; c3
    mov ax, strict word 0000bh                ; b8 0b 00
    call 0165ch                               ; e8 22 ae
    mov bl, al                                ; 88 c3
    and AL, strict byte 057h                  ; 24 57
    movzx dx, al                              ; 0f b6 d0
    jmp near 06758h                           ; e9 14 ff
send_to_mouse_ctrl_:                         ; 0xf6844 LB 0x31
    push bx                                   ; 53
    push dx                                   ; 52
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bl, al                                ; 88 c3
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 002h                 ; a8 02
    je short 06863h                           ; 74 0e
    push 00886h                               ; 68 86 08
    push 010a0h                               ; 68 a0 10
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 89 b0
    add sp, strict byte 00006h                ; 83 c4 06
    mov AL, strict byte 0d4h                  ; b0 d4
    mov dx, strict word 00064h                ; ba 64 00
    out DX, AL                                ; ee
    mov al, bl                                ; 88 d8
    mov dx, strict word 00060h                ; ba 60 00
    out DX, AL                                ; ee
    xor al, bl                                ; 30 d8
    pop bp                                    ; 5d
    pop dx                                    ; 5a
    pop bx                                    ; 5b
    retn                                      ; c3
get_mouse_data_:                             ; 0xf6875 LB 0x38
    push bx                                   ; 53
    push cx                                   ; 51
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bx, ax                                ; 89 c3
    mov es, dx                                ; 8e c2
    mov cx, 02710h                            ; b9 10 27
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    and ax, strict word 00021h                ; 25 21 00
    cmp ax, strict word 00021h                ; 3d 21 00
    je short 06896h                           ; 74 07
    test cx, cx                               ; 85 c9
    je short 06896h                           ; 74 03
    dec cx                                    ; 49
    jmp short 06881h                          ; eb eb
    test cx, cx                               ; 85 c9
    jne short 0689eh                          ; 75 04
    mov AL, strict byte 001h                  ; b0 01
    jmp short 068a9h                          ; eb 0b
    mov dx, strict word 00060h                ; ba 60 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov byte [es:bx], al                      ; 26 88 07
    xor al, al                                ; 30 c0
    pop bp                                    ; 5d
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
set_kbd_command_byte_:                       ; 0xf68ad LB 0x2f
    push bx                                   ; 53
    push dx                                   ; 52
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bl, al                                ; 88 c3
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 002h                 ; a8 02
    je short 068cch                           ; 74 0e
    push 00890h                               ; 68 90 08
    push 010a0h                               ; 68 a0 10
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 20 b0
    add sp, strict byte 00006h                ; 83 c4 06
    mov AL, strict byte 060h                  ; b0 60
    mov dx, strict word 00064h                ; ba 64 00
    out DX, AL                                ; ee
    mov al, bl                                ; 88 d8
    mov dx, strict word 00060h                ; ba 60 00
    out DX, AL                                ; ee
    pop bp                                    ; 5d
    pop dx                                    ; 5a
    pop bx                                    ; 5b
    retn                                      ; c3
_int74_function:                             ; 0xf68dc LB 0xc6
    enter 00008h, 000h                        ; c8 08 00 00
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 33 ad
    mov cx, ax                                ; 89 c1
    mov word [bp+004h], strict word 00000h    ; c7 46 04 00 00
    mov dx, strict word 00064h                ; ba 64 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    and AL, strict byte 021h                  ; 24 21
    cmp AL, strict byte 021h                  ; 3c 21
    jne near 06990h                           ; 0f 85 92 00
    mov dx, strict word 00060h                ; ba 60 00
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov bl, al                                ; 88 c3
    mov dx, strict word 00026h                ; ba 26 00
    mov ax, cx                                ; 89 c8
    call 01600h                               ; e8 f2 ac
    mov byte [bp-006h], al                    ; 88 46 fa
    mov dx, strict word 00027h                ; ba 27 00
    mov ax, cx                                ; 89 c8
    call 01600h                               ; e8 e7 ac
    mov byte [bp-008h], al                    ; 88 46 f8
    test AL, strict byte 080h                 ; a8 80
    je short 06990h                           ; 74 70
    mov al, byte [bp-008h]                    ; 8a 46 f8
    and AL, strict byte 007h                  ; 24 07
    mov byte [bp-002h], al                    ; 88 46 fe
    mov al, byte [bp-006h]                    ; 8a 46 fa
    and AL, strict byte 007h                  ; 24 07
    mov byte [bp-004h], al                    ; 88 46 fc
    xor bh, bh                                ; 30 ff
    movzx dx, al                              ; 0f b6 d0
    add dx, strict byte 00028h                ; 83 c2 28
    mov ax, cx                                ; 89 c8
    call 0160eh                               ; e8 d1 ac
    mov al, byte [bp-004h]                    ; 8a 46 fc
    cmp al, byte [bp-002h]                    ; 3a 46 fe
    jc short 06981h                           ; 72 3c
    mov dx, strict word 00028h                ; ba 28 00
    mov ax, cx                                ; 89 c8
    call 01600h                               ; e8 b3 ac
    xor ah, ah                                ; 30 e4
    mov word [bp+00ch], ax                    ; 89 46 0c
    mov dx, strict word 00029h                ; ba 29 00
    mov ax, cx                                ; 89 c8
    call 01600h                               ; e8 a6 ac
    xor ah, ah                                ; 30 e4
    mov word [bp+00ah], ax                    ; 89 46 0a
    mov dx, strict word 0002ah                ; ba 2a 00
    mov ax, cx                                ; 89 c8
    call 01600h                               ; e8 99 ac
    xor ah, ah                                ; 30 e4
    mov word [bp+008h], ax                    ; 89 46 08
    xor al, al                                ; 30 c0
    mov word [bp+006h], ax                    ; 89 46 06
    mov byte [bp-006h], ah                    ; 88 66 fa
    test byte [bp-008h], 080h                 ; f6 46 f8 80
    je short 06984h                           ; 74 0a
    mov word [bp+004h], strict word 00001h    ; c7 46 04 01 00
    jmp short 06984h                          ; eb 03
    inc byte [bp-006h]                        ; fe 46 fa
    movzx bx, byte [bp-006h]                  ; 0f b6 5e fa
    mov dx, strict word 00026h                ; ba 26 00
    mov ax, cx                                ; 89 c8
    call 0160eh                               ; e8 7e ac
    leave                                     ; c9
    retn                                      ; c3
    in AL, strict byte 069h                   ; e4 69
    pop dx                                    ; 5a
    push strict byte 0ffddh                   ; 6a dd
    push strict byte 0006eh                   ; 6a 6e
    imul bx, sp, strict byte 0006bh           ; 6b dc 6b
    xor byte [bp+si+004h], ch                 ; 30 6a 04
    insb                                      ; 6c
    leave                                     ; c9
    insb                                      ; 6c
_int15_function_mouse:                       ; 0xf69a2 LB 0x386
    push si                                   ; 56
    enter 00006h, 000h                        ; c8 06 00 00
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 6c ac
    mov cx, ax                                ; 89 c1
    cmp byte [bp+014h], 007h                  ; 80 7e 14 07
    jbe short 069c3h                          ; 76 0b
    or word [bp+01ah], strict byte 00001h     ; 83 4e 1a 01
    mov byte [bp+015h], 001h                  ; c6 46 15 01
    leave                                     ; c9
    pop si                                    ; 5e
    retn                                      ; c3
    mov ax, strict word 00065h                ; b8 65 00
    call 068adh                               ; e8 e4 fe
    and word [bp+01ah], strict byte 0fffeh    ; 83 66 1a fe
    mov byte [bp+015h], 000h                  ; c6 46 15 00
    mov al, byte [bp+014h]                    ; 8a 46 14
    cmp AL, strict byte 007h                  ; 3c 07
    jnbe near 06d0ch                          ; 0f 87 32 03
    movzx si, al                              ; 0f b6 f0
    add si, si                                ; 01 f6
    jmp word [cs:si+06992h]                   ; 2e ff a4 92 69
    cmp byte [bp+00fh], 001h                  ; 80 7e 0f 01
    jnbe near 06d17h                          ; 0f 87 2b 03
    mov dx, strict word 00027h                ; ba 27 00
    mov ax, cx                                ; 89 c8
    call 01600h                               ; e8 0c ac
    test AL, strict byte 080h                 ; a8 80
    jne short 06a03h                          ; 75 0b
    or word [bp+01ah], strict byte 00001h     ; 83 4e 1a 01
    mov byte [bp+015h], 005h                  ; c6 46 15 05
    jmp near 06d1fh                           ; e9 1c 03
    cmp byte [bp+00fh], 000h                  ; 80 7e 0f 00
    db  00fh, 094h, 0c0h
    ; sete al                                   ; 0f 94 c0
    add AL, strict byte 0f4h                  ; 04 f4
    xor ah, ah                                ; 30 e4
    call 06844h                               ; e8 33 fe
    test al, al                               ; 84 c0
    jne near 06ca5h                           ; 0f 85 8e 02
    mov dx, ss                                ; 8c d2
    lea ax, [bp-006h]                         ; 8d 46 fa
    call 06875h                               ; e8 56 fe
    test al, al                               ; 84 c0
    je near 06d1fh                            ; 0f 84 fa 02
    cmp byte [bp-006h], 0fah                  ; 80 7e fa fa
    jne near 06ca5h                           ; 0f 85 78 02
    jmp near 06d1fh                           ; e9 ef 02
    mov al, byte [bp+00fh]                    ; 8a 46 0f
    cmp AL, strict byte 001h                  ; 3c 01
    jc short 06a3bh                           ; 72 04
    cmp AL, strict byte 008h                  ; 3c 08
    jbe short 06a3eh                          ; 76 03
    jmp near 06bd1h                           ; e9 93 01
    mov dx, strict word 00027h                ; ba 27 00
    mov ax, cx                                ; 89 c8
    call 01600h                               ; e8 ba ab
    mov ah, byte [bp+00fh]                    ; 8a 66 0f
    db  0feh, 0cch
    ; dec ah                                    ; fe cc
    and AL, strict byte 0f8h                  ; 24 f8
    or al, ah                                 ; 08 e0
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 00027h                ; ba 27 00
    mov ax, cx                                ; 89 c8
    call 0160eh                               ; e8 b4 ab
    mov dx, strict word 00026h                ; ba 26 00
    mov ax, cx                                ; 89 c8
    call 01600h                               ; e8 9e ab
    and AL, strict byte 0f8h                  ; 24 f8
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 00026h                ; ba 26 00
    mov ax, cx                                ; 89 c8
    call 0160eh                               ; e8 9f ab
    mov ax, 000ffh                            ; b8 ff 00
    call 06844h                               ; e8 cf fd
    test al, al                               ; 84 c0
    jne near 06ca5h                           ; 0f 85 2a 02
    mov dx, ss                                ; 8c d2
    lea ax, [bp-002h]                         ; 8d 46 fe
    call 06875h                               ; e8 f2 fd
    mov cl, al                                ; 88 c1
    cmp byte [bp-002h], 0feh                  ; 80 7e fe fe
    jne short 06a96h                          ; 75 0b
    or word [bp+01ah], strict byte 00001h     ; 83 4e 1a 01
    mov byte [bp+015h], 004h                  ; c6 46 15 04
    jmp near 06d1fh                           ; e9 89 02
    cmp byte [bp-002h], 0fah                  ; 80 7e fe fa
    je short 06aach                           ; 74 10
    movzx ax, byte [bp-002h]                  ; 0f b6 46 fe
    push ax                                   ; 50
    push 0089bh                               ; 68 9b 08
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 40 ae
    add sp, strict byte 00006h                ; 83 c4 06
    test cl, cl                               ; 84 c9
    jne near 06ca5h                           ; 0f 85 f3 01
    mov dx, ss                                ; 8c d2
    lea ax, [bp-006h]                         ; 8d 46 fa
    call 06875h                               ; e8 bb fd
    test al, al                               ; 84 c0
    jne near 06ca5h                           ; 0f 85 e5 01
    mov dx, ss                                ; 8c d2
    lea ax, [bp-004h]                         ; 8d 46 fc
    call 06875h                               ; e8 ad fd
    test al, al                               ; 84 c0
    jne near 06ca5h                           ; 0f 85 d7 01
    mov al, byte [bp-006h]                    ; 8a 46 fa
    mov byte [bp+00eh], al                    ; 88 46 0e
    mov al, byte [bp-004h]                    ; 8a 46 fc
    mov byte [bp+00fh], al                    ; 88 46 0f
    jmp near 06d1fh                           ; e9 42 02
    mov al, byte [bp+00fh]                    ; 8a 46 0f
    cmp AL, strict byte 003h                  ; 3c 03
    jc short 06af4h                           ; 72 10
    jbe short 06b12h                          ; 76 2c
    cmp AL, strict byte 006h                  ; 3c 06
    je short 06b24h                           ; 74 3a
    cmp AL, strict byte 005h                  ; 3c 05
    je short 06b1eh                           ; 74 30
    cmp AL, strict byte 004h                  ; 3c 04
    je short 06b18h                           ; 74 26
    jmp short 06b2ah                          ; eb 36
    cmp AL, strict byte 002h                  ; 3c 02
    je short 06b0ch                           ; 74 14
    cmp AL, strict byte 001h                  ; 3c 01
    je short 06b06h                           ; 74 0a
    test al, al                               ; 84 c0
    jne short 06b2ah                          ; 75 2a
    mov byte [bp-006h], 00ah                  ; c6 46 fa 0a
    jmp short 06b2eh                          ; eb 28
    mov byte [bp-006h], 014h                  ; c6 46 fa 14
    jmp short 06b2eh                          ; eb 22
    mov byte [bp-006h], 028h                  ; c6 46 fa 28
    jmp short 06b2eh                          ; eb 1c
    mov byte [bp-006h], 03ch                  ; c6 46 fa 3c
    jmp short 06b2eh                          ; eb 16
    mov byte [bp-006h], 050h                  ; c6 46 fa 50
    jmp short 06b2eh                          ; eb 10
    mov byte [bp-006h], 064h                  ; c6 46 fa 64
    jmp short 06b2eh                          ; eb 0a
    mov byte [bp-006h], 0c8h                  ; c6 46 fa c8
    jmp short 06b2eh                          ; eb 04
    mov byte [bp-006h], 000h                  ; c6 46 fa 00
    cmp byte [bp-006h], 000h                  ; 80 7e fa 00
    jbe short 06b63h                          ; 76 2f
    mov ax, 000f3h                            ; b8 f3 00
    call 06844h                               ; e8 0a fd
    test al, al                               ; 84 c0
    jne short 06b58h                          ; 75 1a
    mov dx, ss                                ; 8c d2
    lea ax, [bp-004h]                         ; 8d 46 fc
    call 06875h                               ; e8 2f fd
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    call 06844h                               ; e8 f7 fc
    mov dx, ss                                ; 8c d2
    lea ax, [bp-004h]                         ; 8d 46 fc
    call 06875h                               ; e8 20 fd
    jmp near 06d1fh                           ; e9 c7 01
    or word [bp+01ah], strict byte 00001h     ; 83 4e 1a 01
    mov byte [bp+015h], 003h                  ; c6 46 15 03
    jmp near 06d1fh                           ; e9 bc 01
    or word [bp+01ah], strict byte 00001h     ; 83 4e 1a 01
    mov byte [bp+015h], 002h                  ; c6 46 15 02
    jmp near 06d1fh                           ; e9 b1 01
    cmp byte [bp+00fh], 004h                  ; 80 7e 0f 04
    jnc short 06bd1h                          ; 73 5d
    mov ax, 000e8h                            ; b8 e8 00
    call 06844h                               ; e8 ca fc
    test al, al                               ; 84 c0
    jne short 06bc6h                          ; 75 48
    mov dx, ss                                ; 8c d2
    lea ax, [bp-006h]                         ; 8d 46 fa
    call 06875h                               ; e8 ef fc
    cmp byte [bp-006h], 0fah                  ; 80 7e fa fa
    je short 06b9ch                           ; 74 10
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    push ax                                   ; 50
    push 008c6h                               ; 68 c6 08
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 50 ad
    add sp, strict byte 00006h                ; 83 c4 06
    movzx ax, byte [bp+00fh]                  ; 0f b6 46 0f
    call 06844h                               ; e8 a1 fc
    mov dx, ss                                ; 8c d2
    lea ax, [bp-006h]                         ; 8d 46 fa
    call 06875h                               ; e8 ca fc
    cmp byte [bp-006h], 0fah                  ; 80 7e fa fa
    je near 06d1fh                            ; 0f 84 6c 01
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    push ax                                   ; 50
    push 008c6h                               ; 68 c6 08
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 29 ad
    add sp, strict byte 00006h                ; 83 c4 06
    jmp near 06d1fh                           ; e9 59 01
    or word [bp+01ah], strict byte 00001h     ; 83 4e 1a 01
    mov byte [bp+015h], 003h                  ; c6 46 15 03
    jmp near 06d1fh                           ; e9 4e 01
    or word [bp+01ah], strict byte 00001h     ; 83 4e 1a 01
    mov byte [bp+015h], 002h                  ; c6 46 15 02
    jmp near 06d1fh                           ; e9 43 01
    mov ax, 000f2h                            ; b8 f2 00
    call 06844h                               ; e8 62 fc
    test al, al                               ; 84 c0
    jne short 06bf9h                          ; 75 13
    mov dx, ss                                ; 8c d2
    lea ax, [bp-006h]                         ; 8d 46 fa
    call 06875h                               ; e8 87 fc
    mov dx, ss                                ; 8c d2
    lea ax, [bp-004h]                         ; 8d 46 fc
    call 06875h                               ; e8 7f fc
    jmp near 06ad4h                           ; e9 db fe
    or word [bp+01ah], strict byte 00001h     ; 83 4e 1a 01
    mov byte [bp+015h], 003h                  ; c6 46 15 03
    jmp near 06d1fh                           ; e9 1b 01
    mov al, byte [bp+00fh]                    ; 8a 46 0f
    test al, al                               ; 84 c0
    jbe short 06c12h                          ; 76 07
    cmp AL, strict byte 002h                  ; 3c 02
    jbe short 06c7bh                          ; 76 6c
    jmp near 06cafh                           ; e9 9d 00
    mov ax, 000e9h                            ; b8 e9 00
    call 06844h                               ; e8 2c fc
    test al, al                               ; 84 c0
    jne near 06ca5h                           ; 0f 85 87 00
    mov dx, ss                                ; 8c d2
    lea ax, [bp-006h]                         ; 8d 46 fa
    call 06875h                               ; e8 4f fc
    mov cl, al                                ; 88 c1
    cmp byte [bp-006h], 0fah                  ; 80 7e fa fa
    je short 06c3eh                           ; 74 10
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    push ax                                   ; 50
    push 008c6h                               ; 68 c6 08
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 ae ac
    add sp, strict byte 00006h                ; 83 c4 06
    test cl, cl                               ; 84 c9
    jne short 06ca5h                          ; 75 63
    mov dx, ss                                ; 8c d2
    lea ax, [bp-006h]                         ; 8d 46 fa
    call 06875h                               ; e8 2b fc
    test al, al                               ; 84 c0
    jne short 06ca5h                          ; 75 57
    mov dx, ss                                ; 8c d2
    lea ax, [bp-004h]                         ; 8d 46 fc
    call 06875h                               ; e8 1f fc
    test al, al                               ; 84 c0
    jne short 06ca5h                          ; 75 4b
    mov dx, ss                                ; 8c d2
    lea ax, [bp-002h]                         ; 8d 46 fe
    call 06875h                               ; e8 13 fc
    test al, al                               ; 84 c0
    jne short 06ca5h                          ; 75 3f
    mov al, byte [bp-006h]                    ; 8a 46 fa
    mov byte [bp+00eh], al                    ; 88 46 0e
    mov al, byte [bp-004h]                    ; 8a 46 fc
    mov byte [bp+012h], al                    ; 88 46 12
    mov al, byte [bp-002h]                    ; 8a 46 fe
    mov byte [bp+010h], al                    ; 88 46 10
    jmp near 06d1fh                           ; e9 a4 00
    cmp AL, strict byte 001h                  ; 3c 01
    jne short 06c84h                          ; 75 05
    mov ax, 000e6h                            ; b8 e6 00
    jmp short 06c87h                          ; eb 03
    mov ax, 000e7h                            ; b8 e7 00
    call 06844h                               ; e8 ba fb
    mov cl, al                                ; 88 c1
    test cl, cl                               ; 84 c9
    jne short 06c9fh                          ; 75 0f
    mov dx, ss                                ; 8c d2
    lea ax, [bp-006h]                         ; 8d 46 fa
    call 06875h                               ; e8 dd fb
    cmp byte [bp-006h], 0fah                  ; 80 7e fa fa
    db  00fh, 095h, 0c1h
    ; setne cl                                  ; 0f 95 c1
    test cl, cl                               ; 84 c9
    je near 06d1fh                            ; 0f 84 7a 00
    or word [bp+01ah], strict byte 00001h     ; 83 4e 1a 01
    mov byte [bp+015h], 003h                  ; c6 46 15 03
    jmp short 06d1fh                          ; eb 70
    movzx ax, byte [bp+00fh]                  ; 0f b6 46 0f
    push ax                                   ; 50
    push 008f2h                               ; 68 f2 08
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 2d ac
    add sp, strict byte 00006h                ; 83 c4 06
    or word [bp+01ah], strict byte 00001h     ; 83 4e 1a 01
    mov byte [bp+015h], 001h                  ; c6 46 15 01
    jmp short 06d1fh                          ; eb 56
    mov si, word [bp+00eh]                    ; 8b 76 0e
    mov bx, si                                ; 89 f3
    mov dx, strict word 00022h                ; ba 22 00
    mov ax, cx                                ; 89 c8
    call 0162ah                               ; e8 54 a9
    mov bx, word [bp+016h]                    ; 8b 5e 16
    mov dx, strict word 00024h                ; ba 24 00
    mov ax, cx                                ; 89 c8
    call 0162ah                               ; e8 49 a9
    mov dx, strict word 00027h                ; ba 27 00
    mov ax, cx                                ; 89 c8
    call 01600h                               ; e8 17 a9
    mov ah, al                                ; 88 c4
    test si, si                               ; 85 f6
    jne short 06cfdh                          ; 75 0e
    cmp word [bp+016h], strict byte 00000h    ; 83 7e 16 00
    jne short 06cfdh                          ; 75 08
    test AL, strict byte 080h                 ; a8 80
    je short 06cffh                           ; 74 06
    and AL, strict byte 07fh                  ; 24 7f
    jmp short 06cffh                          ; eb 02
    or AL, strict byte 080h                   ; 0c 80
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 00027h                ; ba 27 00
    mov ax, cx                                ; 89 c8
    call 0160eh                               ; e8 04 a9
    jmp short 06d1fh                          ; eb 13
    push 0090ch                               ; 68 0c 09
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 d5 ab
    add sp, strict byte 00004h                ; 83 c4 04
    or word [bp+01ah], strict byte 00001h     ; 83 4e 1a 01
    mov byte [bp+015h], 001h                  ; c6 46 15 01
    mov ax, strict word 00047h                ; b8 47 00
    call 068adh                               ; e8 88 fb
    leave                                     ; c9
    pop si                                    ; 5e
    retn                                      ; c3
_int17_function:                             ; 0xf6d28 LB 0xb1
    push si                                   ; 56
    enter 00002h, 000h                        ; c8 02 00 00
    sti                                       ; fb
    mov dx, word [bp+010h]                    ; 8b 56 10
    add dx, dx                                ; 01 d2
    add dx, strict byte 00008h                ; 83 c2 08
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 e0 a8
    mov bx, ax                                ; 89 c3
    mov si, ax                                ; 89 c6
    cmp byte [bp+015h], 003h                  ; 80 7e 15 03
    jnc near 06dd2h                           ; 0f 83 8a 00
    mov ax, word [bp+010h]                    ; 8b 46 10
    cmp ax, strict word 00003h                ; 3d 03 00
    jnc near 06dd2h                           ; 0f 83 80 00
    test bx, bx                               ; 85 db
    jbe near 06dd2h                           ; 0f 86 7a 00
    mov dx, ax                                ; 89 c2
    add dx, strict byte 00078h                ; 83 c2 78
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 9d a8
    movzx cx, al                              ; 0f b6 c8
    sal cx, 008h                              ; c1 e1 08
    cmp byte [bp+015h], 000h                  ; 80 7e 15 00
    jne short 06d9ch                          ; 75 2d
    mov al, byte [bp+014h]                    ; 8a 46 14
    mov dx, bx                                ; 89 da
    out DX, AL                                ; ee
    lea dx, [bx+002h]                         ; 8d 57 02
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov word [bp-002h], ax                    ; 89 46 fe
    mov al, byte [bp-002h]                    ; 8a 46 fe
    or AL, strict byte 001h                   ; 0c 01
    out DX, AL                                ; ee
    movzx ax, byte [bp-002h]                  ; 0f b6 46 fe
    and AL, strict byte 0feh                  ; 24 fe
    out DX, AL                                ; ee
    lea dx, [si+001h]                         ; 8d 54 01
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 040h                 ; a8 40
    je short 06d9ch                           ; 74 07
    test cx, cx                               ; 85 c9
    je short 06d9ch                           ; 74 03
    dec cx                                    ; 49
    jmp short 06d8bh                          ; eb ef
    cmp byte [bp+015h], 001h                  ; 80 7e 15 01
    jne short 06db8h                          ; 75 16
    lea dx, [si+002h]                         ; 8d 54 02
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov word [bp-002h], ax                    ; 89 46 fe
    movzx ax, byte [bp-002h]                  ; 0f b6 46 fe
    and AL, strict byte 0fbh                  ; 24 fb
    out DX, AL                                ; ee
    mov al, byte [bp-002h]                    ; 8a 46 fe
    or AL, strict byte 004h                   ; 0c 04
    out DX, AL                                ; ee
    lea dx, [si+001h]                         ; 8d 54 01
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    xor AL, strict byte 048h                  ; 34 48
    mov byte [bp+015h], al                    ; 88 46 15
    test cx, cx                               ; 85 c9
    jne short 06dcbh                          ; 75 04
    or byte [bp+015h], 001h                   ; 80 4e 15 01
    and byte [bp+01eh], 0feh                  ; 80 66 1e fe
    leave                                     ; c9
    pop si                                    ; 5e
    retn                                      ; c3
    or byte [bp+01eh], 001h                   ; 80 4e 1e 01
    leave                                     ; c9
    pop si                                    ; 5e
    retn                                      ; c3
wait_:                                       ; 0xf6dd9 LB 0xad
    push bx                                   ; 53
    push cx                                   ; 51
    push si                                   ; 56
    push di                                   ; 57
    enter 0000ah, 000h                        ; c8 0a 00 00
    mov si, ax                                ; 89 c6
    mov byte [bp-004h], dl                    ; 88 56 fc
    mov byte [bp-002h], 000h                  ; c6 46 fe 00
    pushfw                                    ; 9c
    pop ax                                    ; 58
    mov word [bp-008h], ax                    ; 89 46 f8
    sti                                       ; fb
    xor cx, cx                                ; 31 c9
    mov dx, 0046ch                            ; ba 6c 04
    xor ax, ax                                ; 31 c0
    call 01638h                               ; e8 3e a8
    mov word [bp-006h], ax                    ; 89 46 fa
    mov bx, dx                                ; 89 d3
    hlt                                       ; f4
    mov dx, 0046ch                            ; ba 6c 04
    xor ax, ax                                ; 31 c0
    call 01638h                               ; e8 30 a8
    mov word [bp-00ah], ax                    ; 89 46 f6
    mov di, dx                                ; 89 d7
    cmp dx, bx                                ; 39 da
    jnbe short 06e18h                         ; 77 07
    jne short 06e1fh                          ; 75 0c
    cmp ax, word [bp-006h]                    ; 3b 46 fa
    jbe short 06e1fh                          ; 76 07
    sub ax, word [bp-006h]                    ; 2b 46 fa
    sbb dx, bx                                ; 19 da
    jmp short 06e2ah                          ; eb 0b
    cmp dx, bx                                ; 39 da
    jc short 06e2ah                           ; 72 07
    jne short 06e2eh                          ; 75 09
    cmp ax, word [bp-006h]                    ; 3b 46 fa
    jnc short 06e2eh                          ; 73 04
    sub si, ax                                ; 29 c6
    sbb cx, dx                                ; 19 d1
    mov ax, word [bp-00ah]                    ; 8b 46 f6
    mov word [bp-006h], ax                    ; 89 46 fa
    mov bx, di                                ; 89 fb
    mov ax, 00100h                            ; b8 00 01
    int 016h                                  ; cd 16
    je near 06e44h                            ; 0f 84 05 00
    mov AL, strict byte 001h                  ; b0 01
    jmp near 06e46h                           ; e9 02 00
    db  032h, 0c0h
    ; xor al, al                                ; 32 c0
    test al, al                               ; 84 c0
    je short 06e6eh                           ; 74 24
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    int 016h                                  ; cd 16
    xchg ah, al                               ; 86 c4
    mov dl, al                                ; 88 c2
    mov byte [bp-002h], al                    ; 88 46 fe
    movzx ax, dl                              ; 0f b6 c2
    push ax                                   ; 50
    push 0092eh                               ; 68 2e 09
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 88 aa
    add sp, strict byte 00006h                ; 83 c4 06
    cmp byte [bp-004h], 000h                  ; 80 7e fc 00
    je short 06e6eh                           ; 74 04
    mov al, dl                                ; 88 d0
    jmp short 06e80h                          ; eb 12
    test cx, cx                               ; 85 c9
    jnle short 06dffh                         ; 7f 8d
    jne short 06e78h                          ; 75 04
    test si, si                               ; 85 f6
    jnbe short 06dffh                         ; 77 87
    mov ax, word [bp-008h]                    ; 8b 46 f8
    push ax                                   ; 50
    popfw                                     ; 9d
    mov al, byte [bp-002h]                    ; 8a 46 fe
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
read_logo_byte_:                             ; 0xf6e86 LB 0x13
    push dx                                   ; 52
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    xor ah, ah                                ; 30 e4
    or ah, 001h                               ; 80 cc 01
    mov dx, 003b8h                            ; ba b8 03
    out DX, ax                                ; ef
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    pop bp                                    ; 5d
    pop dx                                    ; 5a
    retn                                      ; c3
read_logo_word_:                             ; 0xf6e99 LB 0x11
    push dx                                   ; 52
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    xor ah, ah                                ; 30 e4
    or ah, 001h                               ; 80 cc 01
    mov dx, 003b8h                            ; ba b8 03
    out DX, ax                                ; ef
    in ax, DX                                 ; ed
    pop bp                                    ; 5d
    pop dx                                    ; 5a
    retn                                      ; c3
print_detected_harddisks_:                   ; 0xf6eaa LB 0x12c
    push bx                                   ; 53
    push cx                                   ; 51
    push dx                                   ; 52
    push si                                   ; 56
    push di                                   ; 57
    enter 00004h, 000h                        ; c8 04 00 00
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 60 a7
    mov si, ax                                ; 89 c6
    mov byte [bp-002h], 000h                  ; c6 46 fe 00
    xor cl, cl                                ; 30 c9
    xor ch, ch                                ; 30 ed
    mov dx, 002c0h                            ; ba c0 02
    call 01600h                               ; e8 34 a7
    mov byte [bp-004h], al                    ; 88 46 fc
    xor bl, bl                                ; 30 db
    cmp bl, byte [bp-004h]                    ; 3a 5e fc
    jnc near 06fabh                           ; 0f 83 d3 00
    movzx dx, bl                              ; 0f b6 d3
    add dx, 002c1h                            ; 81 c2 c1 02
    mov ax, si                                ; 89 f0
    call 01600h                               ; e8 1c a7
    mov bh, al                                ; 88 c7
    cmp AL, strict byte 00ch                  ; 3c 0c
    jc short 06f0eh                           ; 72 24
    test cl, cl                               ; 84 c9
    jne short 06efbh                          ; 75 0d
    push 0093fh                               ; 68 3f 09
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 f3 a9
    add sp, strict byte 00004h                ; 83 c4 04
    mov CL, strict byte 001h                  ; b1 01
    movzx ax, bl                              ; 0f b6 c3
    inc ax                                    ; 40
    push ax                                   ; 50
    push 00953h                               ; 68 53 09
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 e1 a9
    add sp, strict byte 00006h                ; 83 c4 06
    jmp near 06fa6h                           ; e9 98 00
    cmp AL, strict byte 008h                  ; 3c 08
    jc short 06f25h                           ; 72 13
    test ch, ch                               ; 84 ed
    jne short 06f23h                          ; 75 0d
    push 00966h                               ; 68 66 09
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 cb a9
    add sp, strict byte 00004h                ; 83 c4 04
    mov CH, strict byte 001h                  ; b5 01
    jmp short 06efbh                          ; eb d6
    cmp AL, strict byte 004h                  ; 3c 04
    jnc short 06f40h                          ; 73 17
    cmp byte [bp-002h], 000h                  ; 80 7e fe 00
    jne short 06f40h                          ; 75 11
    push 0097ah                               ; 68 7a 09
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 b2 a9
    add sp, strict byte 00004h                ; 83 c4 04
    mov byte [bp-002h], 001h                  ; c6 46 fe 01
    jmp short 06f56h                          ; eb 16
    cmp bh, 004h                              ; 80 ff 04
    jc short 06f56h                           ; 72 11
    test cl, cl                               ; 84 c9
    jne short 06f56h                          ; 75 0d
    push 0093fh                               ; 68 3f 09
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 98 a9
    add sp, strict byte 00004h                ; 83 c4 04
    mov CL, strict byte 001h                  ; b1 01
    movzx ax, bl                              ; 0f b6 c3
    inc ax                                    ; 40
    push ax                                   ; 50
    push 0098bh                               ; 68 8b 09
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 86 a9
    add sp, strict byte 00006h                ; 83 c4 06
    cmp bh, 004h                              ; 80 ff 04
    jc short 06f6eh                           ; 72 03
    sub bh, 004h                              ; 80 ef 04
    movzx ax, bh                              ; 0f b6 c7
    cwd                                       ; 99
    db  02bh, 0c2h
    ; sub ax, dx                                ; 2b c2
    sar ax, 1                                 ; d1 f8
    test ax, ax                               ; 85 c0
    je short 06f7fh                           ; 74 05
    push 00995h                               ; 68 95 09
    jmp short 06f82h                          ; eb 03
    push 009a0h                               ; 68 a0 09
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 62 a9
    add sp, strict byte 00004h                ; 83 c4 04
    movzx ax, bh                              ; 0f b6 c7
    mov di, strict word 00002h                ; bf 02 00
    cwd                                       ; 99
    idiv di                                   ; f7 ff
    test dx, dx                               ; 85 d2
    je short 06f9ch                           ; 74 05
    push 009a9h                               ; 68 a9 09
    jmp short 06f9fh                          ; eb 03
    push 009afh                               ; 68 af 09
    push di                                   ; 57
    call 018e9h                               ; e8 46 a9
    add sp, strict byte 00004h                ; 83 c4 04
    db  0feh, 0c3h
    ; inc bl                                    ; fe c3
    jmp near 06ed1h                           ; e9 26 ff
    cmp byte [bp-002h], 000h                  ; 80 7e fe 00
    jne short 06fc4h                          ; 75 13
    test cl, cl                               ; 84 c9
    jne short 06fc4h                          ; 75 0f
    test ch, ch                               ; 84 ed
    jne short 06fc4h                          ; 75 0b
    push 009b6h                               ; 68 b6 09
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 28 a9
    add sp, strict byte 00004h                ; 83 c4 04
    push 009cah                               ; 68 ca 09
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 1d a9
    add sp, strict byte 00004h                ; 83 c4 04
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop dx                                    ; 5a
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
get_boot_drive_:                             ; 0xf6fd6 LB 0x25
    push bx                                   ; 53
    push dx                                   ; 52
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bl, al                                ; 88 c3
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 36 a6
    mov dx, 002c0h                            ; ba c0 02
    call 01600h                               ; e8 14 a6
    sub bl, 002h                              ; 80 eb 02
    cmp bl, al                                ; 38 c3
    jc short 06ff5h                           ; 72 02
    mov BL, strict byte 0ffh                  ; b3 ff
    mov al, bl                                ; 88 d8
    pop bp                                    ; 5d
    pop dx                                    ; 5a
    pop bx                                    ; 5b
    retn                                      ; c3
show_logo_:                                  ; 0xf6ffb LB 0x21f
    push bx                                   ; 53
    push cx                                   ; 51
    push dx                                   ; 52
    push si                                   ; 56
    push di                                   ; 57
    enter 0000ch, 000h                        ; c8 0c 00 00
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 0f a6
    mov si, ax                                ; 89 c6
    xor cl, cl                                ; 30 c9
    xor dx, dx                                ; 31 d2
    mov AL, strict byte 034h                  ; b0 34
    out strict byte 043h, AL                  ; e6 43
    mov AL, strict byte 0d3h                  ; b0 d3
    out strict byte 040h, AL                  ; e6 40
    mov AL, strict byte 048h                  ; b0 48
    out strict byte 040h, AL                  ; e6 40
    movzx ax, dl                              ; 0f b6 c2
    call 06e99h                               ; e8 74 fe
    cmp ax, 066bbh                            ; 3d bb 66
    jne near 070fdh                           ; 0f 85 d1 00
    push SS                                   ; 16
    pop ES                                    ; 07
    lea di, [bp-00ch]                         ; 8d 7e f4
    mov ax, 04f03h                            ; b8 03 4f
    int 010h                                  ; cd 10
    mov word [es:di], bx                      ; 26 89 1d
    cmp ax, strict word 0004fh                ; 3d 4f 00
    jne near 070fdh                           ; 0f 85 bd 00
    mov al, dl                                ; 88 d0
    add AL, strict byte 004h                  ; 04 04
    xor ah, ah                                ; 30 e4
    call 06e86h                               ; e8 3d fe
    mov ch, al                                ; 88 c5
    mov byte [bp-002h], al                    ; 88 46 fe
    mov al, dl                                ; 88 d0
    add AL, strict byte 005h                  ; 04 05
    xor ah, ah                                ; 30 e4
    call 06e86h                               ; e8 2f fe
    mov dh, al                                ; 88 c6
    mov byte [bp-006h], al                    ; 88 46 fa
    mov al, dl                                ; 88 d0
    add AL, strict byte 002h                  ; 04 02
    xor ah, ah                                ; 30 e4
    call 06e99h                               ; e8 34 fe
    mov bx, ax                                ; 89 c3
    mov word [bp-00ah], ax                    ; 89 46 f6
    mov al, dl                                ; 88 d0
    add AL, strict byte 006h                  ; 04 06
    xor ah, ah                                ; 30 e4
    call 06e86h                               ; e8 13 fe
    mov byte [bp-008h], al                    ; 88 46 f8
    test ch, ch                               ; 84 ed
    jne short 07084h                          ; 75 0a
    test dh, dh                               ; 84 f6
    jne short 07084h                          ; 75 06
    test bx, bx                               ; 85 db
    je near 070fdh                            ; 0f 84 79 00
    mov bx, 00142h                            ; bb 42 01
    mov ax, 04f02h                            ; b8 02 4f
    int 010h                                  ; cd 10
    cmp byte [bp-002h], 000h                  ; 80 7e fe 00
    je short 070b5h                           ; 74 23
    xor bx, bx                                ; 31 db
    jmp short 0709ch                          ; eb 06
    inc bx                                    ; 43
    cmp bx, strict byte 00010h                ; 83 fb 10
    jnbe short 070bch                         ; 77 20
    mov ax, bx                                ; 89 d8
    or ah, 002h                               ; 80 cc 02
    mov dx, 003b8h                            ; ba b8 03
    out DX, ax                                ; ef
    xor dx, dx                                ; 31 d2
    mov ax, strict word 00001h                ; b8 01 00
    call 06dd9h                               ; e8 2c fd
    cmp AL, strict byte 086h                  ; 3c 86
    jne short 07096h                          ; 75 e5
    mov CL, strict byte 001h                  ; b1 01
    jmp short 070bch                          ; eb 07
    mov ax, 00210h                            ; b8 10 02
    mov dx, 003b8h                            ; ba b8 03
    out DX, ax                                ; ef
    test cl, cl                               ; 84 c9
    jne short 070d2h                          ; 75 12
    mov ax, word [bp-00ah]                    ; 8b 46 f6
    shr ax, 004h                              ; c1 e8 04
    mov dx, strict word 00001h                ; ba 01 00
    call 06dd9h                               ; e8 0d fd
    cmp AL, strict byte 086h                  ; 3c 86
    jne short 070d2h                          ; 75 02
    mov CL, strict byte 001h                  ; b1 01
    cmp byte [bp-006h], 000h                  ; 80 7e fa 00
    je short 070fdh                           ; 74 25
    test cl, cl                               ; 84 c9
    jne short 070fdh                          ; 75 21
    mov bx, strict word 00010h                ; bb 10 00
    jmp short 070e6h                          ; eb 05
    dec bx                                    ; 4b
    test bx, bx                               ; 85 db
    jbe short 070fdh                          ; 76 17
    mov ax, bx                                ; 89 d8
    or ah, 002h                               ; 80 cc 02
    mov dx, 003b8h                            ; ba b8 03
    out DX, ax                                ; ef
    xor dx, dx                                ; 31 d2
    mov ax, strict word 00001h                ; b8 01 00
    call 06dd9h                               ; e8 e2 fc
    cmp AL, strict byte 086h                  ; 3c 86
    jne short 070e1h                          ; 75 e6
    mov CL, strict byte 001h                  ; b1 01
    xor bx, bx                                ; 31 db
    mov dx, 00339h                            ; ba 39 03
    mov ax, si                                ; 89 f0
    call 0160eh                               ; e8 07 a5
    mov AL, strict byte 003h                  ; b0 03
    mov AH, strict byte 000h                  ; b4 00
    int 010h                                  ; cd 10
    cmp byte [bp-008h], 000h                  ; 80 7e f8 00
    je near 071feh                            ; 0f 84 e9 00
    cmp byte [bp-002h], 000h                  ; 80 7e fe 00
    jne short 0714bh                          ; 75 30
    cmp byte [bp-006h], 000h                  ; 80 7e fa 00
    jne short 0714bh                          ; 75 2a
    cmp word [bp-00ah], strict byte 00000h    ; 83 7e f6 00
    jne short 0714bh                          ; 75 24
    cmp byte [bp-008h], 002h                  ; 80 7e f8 02
    jne short 07138h                          ; 75 0b
    push 009cch                               ; 68 cc 09
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 b4 a7
    add sp, strict byte 00004h                ; 83 c4 04
    test cl, cl                               ; 84 c9
    jne short 0714bh                          ; 75 0f
    mov dx, strict word 00001h                ; ba 01 00
    mov ax, 000c0h                            ; b8 c0 00
    call 06dd9h                               ; e8 94 fc
    cmp AL, strict byte 086h                  ; 3c 86
    jne short 0714bh                          ; 75 02
    mov CL, strict byte 001h                  ; b1 01
    test cl, cl                               ; 84 c9
    je near 071feh                            ; 0f 84 ad 00
    mov byte [bp-004h], 000h                  ; c6 46 fc 00
    mov ax, 00100h                            ; b8 00 01
    mov cx, 01000h                            ; b9 00 10
    int 010h                                  ; cd 10
    mov ax, 00700h                            ; b8 00 07
    mov BH, strict byte 007h                  ; b7 07
    db  033h, 0c9h
    ; xor cx, cx                                ; 33 c9
    mov dx, 0184fh                            ; ba 4f 18
    int 010h                                  ; cd 10
    mov ax, 00200h                            ; b8 00 02
    db  033h, 0dbh
    ; xor bx, bx                                ; 33 db
    db  033h, 0d2h
    ; xor dx, dx                                ; 33 d2
    int 010h                                  ; cd 10
    push 009eeh                               ; 68 ee 09
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 6f a7
    add sp, strict byte 00004h                ; 83 c4 04
    call 06eaah                               ; e8 2a fd
    push 00a32h                               ; 68 32 0a
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 61 a7
    add sp, strict byte 00004h                ; 83 c4 04
    mov dx, strict word 00001h                ; ba 01 00
    mov ax, strict word 00040h                ; b8 40 00
    call 06dd9h                               ; e8 45 fc
    mov bl, al                                ; 88 c3
    test al, al                               ; 84 c0
    je short 0718bh                           ; 74 f1
    cmp AL, strict byte 030h                  ; 3c 30
    je short 071ech                           ; 74 4e
    cmp bl, 002h                              ; 80 fb 02
    jc short 071c5h                           ; 72 22
    cmp bl, 009h                              ; 80 fb 09
    jnbe short 071c5h                         ; 77 1d
    movzx ax, bl                              ; 0f b6 c3
    call 06fd6h                               ; e8 28 fe
    cmp AL, strict byte 0ffh                  ; 3c ff
    jne short 071b4h                          ; 75 02
    jmp short 0718bh                          ; eb d7
    movzx bx, al                              ; 0f b6 d8
    mov dx, 00338h                            ; ba 38 03
    mov ax, si                                ; 89 f0
    call 0160eh                               ; e8 4f a4
    mov byte [bp-004h], 002h                  ; c6 46 fc 02
    jmp short 071ech                          ; eb 27
    cmp bl, 02eh                              ; 80 fb 2e
    je short 071dah                           ; 74 10
    cmp bl, 026h                              ; 80 fb 26
    je short 071e0h                           ; 74 11
    cmp bl, 021h                              ; 80 fb 21
    jne short 071e6h                          ; 75 12
    mov byte [bp-004h], 001h                  ; c6 46 fc 01
    jmp short 071ech                          ; eb 12
    mov byte [bp-004h], 003h                  ; c6 46 fc 03
    jmp short 071ech                          ; eb 0c
    mov byte [bp-004h], 004h                  ; c6 46 fc 04
    jmp short 071ech                          ; eb 06
    cmp byte [bp-004h], 000h                  ; 80 7e fc 00
    je short 0718bh                           ; 74 9f
    movzx bx, byte [bp-004h]                  ; 0f b6 5e fc
    mov dx, 00339h                            ; ba 39 03
    mov ax, si                                ; 89 f0
    call 0160eh                               ; e8 16 a4
    mov AL, strict byte 003h                  ; b0 03
    mov AH, strict byte 000h                  ; b4 00
    int 010h                                  ; cd 10
    mov AL, strict byte 034h                  ; b0 34
    out strict byte 043h, AL                  ; e6 43
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    out strict byte 040h, AL                  ; e6 40
    out strict byte 040h, AL                  ; e6 40
    pushad                                    ; 66 60
    push DS                                   ; 1e
    mov ds, ax                                ; 8e d8
    call 0edbfh                               ; e8 af 7b
    pop DS                                    ; 1f
    popad                                     ; 66 61
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop dx                                    ; 5a
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
delay_boot_:                                 ; 0xf721a LB 0x64
    push bx                                   ; 53
    push dx                                   ; 52
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov dx, ax                                ; 89 c2
    test ax, ax                               ; 85 c0
    je short 0727ah                           ; 74 55
    mov AL, strict byte 034h                  ; b0 34
    out strict byte 043h, AL                  ; e6 43
    mov AL, strict byte 0d3h                  ; b0 d3
    out strict byte 040h, AL                  ; e6 40
    mov AL, strict byte 048h                  ; b0 48
    out strict byte 040h, AL                  ; e6 40
    push dx                                   ; 52
    push 00a7ch                               ; 68 7c 0a
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 af a6
    add sp, strict byte 00006h                ; 83 c4 06
    mov bx, dx                                ; 89 d3
    test bx, bx                               ; 85 db
    jbe short 0725ah                          ; 76 17
    push bx                                   ; 53
    push 00a9ah                               ; 68 9a 0a
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 9d a6
    add sp, strict byte 00006h                ; 83 c4 06
    xor dx, dx                                ; 31 d2
    mov ax, strict word 00040h                ; b8 40 00
    call 06dd9h                               ; e8 82 fb
    dec bx                                    ; 4b
    jmp short 0723fh                          ; eb e5
    push 009cah                               ; 68 ca 09
    push strict byte 00002h                   ; 6a 02
    call 018e9h                               ; e8 87 a6
    add sp, strict byte 00004h                ; 83 c4 04
    mov AL, strict byte 034h                  ; b0 34
    out strict byte 043h, AL                  ; e6 43
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    out strict byte 040h, AL                  ; e6 40
    out strict byte 040h, AL                  ; e6 40
    pushad                                    ; 66 60
    push DS                                   ; 1e
    mov ds, ax                                ; 8e d8
    call 0edbfh                               ; e8 48 7b
    pop DS                                    ; 1f
    popad                                     ; 66 61
    pop bp                                    ; 5d
    pop dx                                    ; 5a
    pop bx                                    ; 5b
    retn                                      ; c3
scsi_cmd_data_in_:                           ; 0xf727e LB 0x63
    push si                                   ; 56
    push di                                   ; 57
    enter 00004h, 000h                        ; c8 04 00 00
    mov si, ax                                ; 89 c6
    mov byte [bp-002h], dl                    ; 88 56 fe
    mov word [bp-004h], bx                    ; 89 5e fc
    mov dx, si                                ; 89 f2
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 001h                 ; a8 01
    jne short 0728ch                          ; 75 f7
    mov al, byte [bp-002h]                    ; 8a 46 fe
    out DX, AL                                ; ee
    xor al, al                                ; 30 c0
    out DX, AL                                ; ee
    mov al, byte [bp+008h]                    ; 8a 46 08
    out DX, AL                                ; ee
    mov al, byte [bp+00eh]                    ; 8a 46 0e
    out DX, AL                                ; ee
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    shr ax, 008h                              ; c1 e8 08
    out DX, AL                                ; ee
    xor bx, bx                                ; 31 db
    movzx ax, byte [bp+008h]                  ; 0f b6 46 08
    cmp bx, ax                                ; 39 c3
    jnc short 072c5h                          ; 73 10
    mov es, cx                                ; 8e c1
    mov di, word [bp-004h]                    ; 8b 7e fc
    add di, bx                                ; 01 df
    mov al, byte [es:di]                      ; 26 8a 05
    mov dx, si                                ; 89 f2
    out DX, AL                                ; ee
    inc bx                                    ; 43
    jmp short 072adh                          ; eb e8
    mov dx, si                                ; 89 f2
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 001h                 ; a8 01
    jne short 072c5h                          ; 75 f7
    lea dx, [si+001h]                         ; 8d 54 01
    mov cx, word [bp+00eh]                    ; 8b 4e 0e
    les di, [bp+00ah]                         ; c4 7e 0a
    rep insb                                  ; f3 6c
    xor ax, ax                                ; 31 c0
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn 00008h                               ; c2 08 00
scsi_cmd_data_out_:                          ; 0xf72e1 LB 0x64
    push si                                   ; 56
    push di                                   ; 57
    enter 00004h, 000h                        ; c8 04 00 00
    mov di, ax                                ; 89 c7
    mov byte [bp-002h], dl                    ; 88 56 fe
    mov word [bp-004h], bx                    ; 89 5e fc
    mov dx, di                                ; 89 fa
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 001h                 ; a8 01
    jne short 072efh                          ; 75 f7
    mov al, byte [bp-002h]                    ; 8a 46 fe
    out DX, AL                                ; ee
    mov AL, strict byte 001h                  ; b0 01
    out DX, AL                                ; ee
    mov al, byte [bp+008h]                    ; 8a 46 08
    out DX, AL                                ; ee
    mov al, byte [bp+00eh]                    ; 8a 46 0e
    out DX, AL                                ; ee
    mov ax, word [bp+00eh]                    ; 8b 46 0e
    shr ax, 008h                              ; c1 e8 08
    out DX, AL                                ; ee
    xor bx, bx                                ; 31 db
    movzx ax, byte [bp+008h]                  ; 0f b6 46 08
    cmp bx, ax                                ; 39 c3
    jnc short 07328h                          ; 73 10
    mov es, cx                                ; 8e c1
    mov si, word [bp-004h]                    ; 8b 76 fc
    add si, bx                                ; 01 de
    mov al, byte [es:si]                      ; 26 8a 04
    mov dx, di                                ; 89 fa
    out DX, AL                                ; ee
    inc bx                                    ; 43
    jmp short 07310h                          ; eb e8
    lea dx, [di+001h]                         ; 8d 55 01
    mov cx, word [bp+00eh]                    ; 8b 4e 0e
    les si, [bp+00ah]                         ; c4 76 0a
    db  0f3h, 026h, 06eh
    ; rep es outsb                              ; f3 26 6e
    mov dx, di                                ; 89 fa
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    test AL, strict byte 001h                 ; a8 01
    jne short 07334h                          ; 75 f7
    xor ax, ax                                ; 31 c0
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn 00008h                               ; c2 08 00
@scsi_read_sectors:                          ; 0xf7345 LB 0xa2
    push si                                   ; 56
    push di                                   ; 57
    enter 0000ch, 000h                        ; c8 0c 00 00
    mov si, word [bp+008h]                    ; 8b 76 08
    mov es, [bp+00ah]                         ; 8e 46 0a
    mov bl, byte [es:si+008h]                 ; 26 8a 5c 08
    sub bl, 008h                              ; 80 eb 08
    cmp bl, 004h                              ; 80 fb 04
    jbe short 0736ch                          ; 76 0f
    movzx ax, bl                              ; 0f b6 c3
    push ax                                   ; 50
    push 00a9eh                               ; 68 9e 0a
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 80 a5
    add sp, strict byte 00006h                ; 83 c4 06
    mov es, [bp+00ah]                         ; 8e 46 0a
    mov di, word [es:si+00ah]                 ; 26 8b 7c 0a
    mov word [bp-00ch], strict word 00028h    ; c7 46 f4 28 00
    mov ax, word [es:si]                      ; 26 8b 04
    mov dx, word [es:si+002h]                 ; 26 8b 54 02
    xchg ah, al                               ; 86 c4
    xchg dh, dl                               ; 86 d6
    xchg dx, ax                               ; 92
    mov word [bp-00ah], ax                    ; 89 46 f6
    mov word [bp-008h], dx                    ; 89 56 f8
    mov byte [bp-006h], 000h                  ; c6 46 fa 00
    mov ax, di                                ; 89 f8
    xchg ah, al                               ; 86 c4
    mov word [bp-005h], ax                    ; 89 46 fb
    mov byte [bp-003h], 000h                  ; c6 46 fd 00
    xor bh, bh                                ; 30 ff
    sal bx, 002h                              ; c1 e3 02
    add bx, si                                ; 01 f3
    mov ax, word [es:bx+001d8h]               ; 26 8b 87 d8 01
    mov dl, byte [es:bx+001dah]               ; 26 8a 97 da 01
    mov bx, di                                ; 89 fb
    sal bx, 009h                              ; c1 e3 09
    mov word [bp-002h], bx                    ; 89 5e fe
    push bx                                   ; 53
    db  066h, 026h, 0ffh, 074h, 004h
    ; push dword [es:si+004h]                   ; 66 26 ff 74 04
    push strict byte 0000ah                   ; 6a 0a
    xor dh, dh                                ; 30 f6
    mov cx, ss                                ; 8c d1
    lea bx, [bp-00ch]                         ; 8d 5e f4
    call 0727eh                               ; e8 ba fe
    mov ah, al                                ; 88 c4
    test al, al                               ; 84 c0
    jne short 073deh                          ; 75 14
    mov es, [bp+00ah]                         ; 8e 46 0a
    mov word [es:si+014h], di                 ; 26 89 7c 14
    mov dx, word [bp-002h]                    ; 8b 56 fe
    mov word [es:si+016h], dx                 ; 26 89 54 16
    mov word [es:si+018h], strict word 00000h ; 26 c7 44 18 00 00
    movzx ax, ah                              ; 0f b6 c4
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn 00004h                               ; c2 04 00
@scsi_write_sectors:                         ; 0xf73e7 LB 0xa2
    push si                                   ; 56
    push di                                   ; 57
    enter 0000ch, 000h                        ; c8 0c 00 00
    mov si, word [bp+008h]                    ; 8b 76 08
    mov es, [bp+00ah]                         ; 8e 46 0a
    mov bl, byte [es:si+008h]                 ; 26 8a 5c 08
    sub bl, 008h                              ; 80 eb 08
    cmp bl, 004h                              ; 80 fb 04
    jbe short 0740eh                          ; 76 0f
    movzx ax, bl                              ; 0f b6 c3
    push ax                                   ; 50
    push 00acch                               ; 68 cc 0a
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 de a4
    add sp, strict byte 00006h                ; 83 c4 06
    mov es, [bp+00ah]                         ; 8e 46 0a
    mov di, word [es:si+00ah]                 ; 26 8b 7c 0a
    mov word [bp-00ch], strict word 0002ah    ; c7 46 f4 2a 00
    mov ax, word [es:si]                      ; 26 8b 04
    mov dx, word [es:si+002h]                 ; 26 8b 54 02
    xchg ah, al                               ; 86 c4
    xchg dh, dl                               ; 86 d6
    xchg dx, ax                               ; 92
    mov word [bp-00ah], ax                    ; 89 46 f6
    mov word [bp-008h], dx                    ; 89 56 f8
    mov byte [bp-006h], 000h                  ; c6 46 fa 00
    mov ax, di                                ; 89 f8
    xchg ah, al                               ; 86 c4
    mov word [bp-005h], ax                    ; 89 46 fb
    mov byte [bp-003h], 000h                  ; c6 46 fd 00
    xor bh, bh                                ; 30 ff
    sal bx, 002h                              ; c1 e3 02
    add bx, si                                ; 01 f3
    mov ax, word [es:bx+001d8h]               ; 26 8b 87 d8 01
    mov dl, byte [es:bx+001dah]               ; 26 8a 97 da 01
    mov bx, di                                ; 89 fb
    sal bx, 009h                              ; c1 e3 09
    mov word [bp-002h], bx                    ; 89 5e fe
    push bx                                   ; 53
    db  066h, 026h, 0ffh, 074h, 004h
    ; push dword [es:si+004h]                   ; 66 26 ff 74 04
    push strict byte 0000ah                   ; 6a 0a
    xor dh, dh                                ; 30 f6
    mov cx, ss                                ; 8c d1
    lea bx, [bp-00ch]                         ; 8d 5e f4
    call 072e1h                               ; e8 7b fe
    mov ah, al                                ; 88 c4
    test al, al                               ; 84 c0
    jne short 07480h                          ; 75 14
    mov es, [bp+00ah]                         ; 8e 46 0a
    mov word [es:si+014h], di                 ; 26 89 7c 14
    mov dx, word [bp-002h]                    ; 8b 56 fe
    mov word [es:si+016h], dx                 ; 26 89 54 16
    mov word [es:si+018h], strict word 00000h ; 26 c7 44 18 00 00
    movzx ax, ah                              ; 0f b6 c4
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn 00004h                               ; c2 04 00
scsi_enumerate_attached_devices_:            ; 0xf7489 LB 0x285
    push bx                                   ; 53
    push cx                                   ; 51
    push dx                                   ; 52
    push si                                   ; 56
    push di                                   ; 57
    enter 0021ah, 000h                        ; c8 1a 02 00
    push ax                                   ; 50
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 80 a1
    mov si, 00122h                            ; be 22 01
    mov word [bp-006h], ax                    ; 89 46 fa
    mov word [bp-008h], strict word 00000h    ; c7 46 f8 00 00
    jmp near 076b6h                           ; e9 0c 02
    mov es, [bp-006h]                         ; 8e 46 fa
    cmp byte [es:si+001e8h], 004h             ; 26 80 bc e8 01 04
    jnc near 07707h                           ; 0f 83 50 02
    mov cx, strict word 0000ah                ; b9 0a 00
    xor bx, bx                                ; 31 db
    mov dx, ss                                ; 8c d2
    lea ax, [bp-01ah]                         ; 8d 46 e6
    call 08cbah                               ; e8 f6 17
    mov byte [bp-01ah], 025h                  ; c6 46 e6 25
    push strict byte 00008h                   ; 6a 08
    lea dx, [bp-0021ah]                       ; 8d 96 e6 fd
    push SS                                   ; 16
    push dx                                   ; 52
    push strict byte 0000ah                   ; 6a 0a
    movzx dx, byte [bp-008h]                  ; 0f b6 56 f8
    mov cx, ss                                ; 8c d1
    lea bx, [bp-01ah]                         ; 8d 5e e6
    mov ax, word [bp-0021ch]                  ; 8b 86 e4 fd
    call 0727eh                               ; e8 9c fd
    test al, al                               ; 84 c0
    je short 074f1h                           ; 74 0b
    push 00b31h                               ; 68 31 0b
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 fb a3
    add sp, strict byte 00004h                ; 83 c4 04
    movzx dx, byte [bp-00219h]                ; 0f b6 96 e7 fd
    movzx di, byte [bp-0021ah]                ; 0f b6 be e6 fd
    sal di, 008h                              ; c1 e7 08
    xor bx, bx                                ; 31 db
    or di, dx                                 ; 09 d7
    movzx ax, byte [bp-00218h]                ; 0f b6 86 e8 fd
    xor dx, dx                                ; 31 d2
    mov cx, strict word 00008h                ; b9 08 00
    sal ax, 1                                 ; d1 e0
    rcl dx, 1                                 ; d1 d2
    loop 0750ch                               ; e2 fa
    or bx, ax                                 ; 09 c3
    or di, dx                                 ; 09 d7
    movzx ax, byte [bp-00217h]                ; 0f b6 86 e9 fd
    or bx, ax                                 ; 09 c3
    mov word [bp-00ah], bx                    ; 89 5e f6
    movzx ax, byte [bp-00216h]                ; 0f b6 86 ea fd
    sal ax, 008h                              ; c1 e0 08
    movzx bx, byte [bp-00215h]                ; 0f b6 9e eb fd
    mov word [bp-010h], strict word 00000h    ; c7 46 f0 00 00
    or bx, ax                                 ; 09 c3
    movzx ax, byte [bp-00214h]                ; 0f b6 86 ec fd
    xor dx, dx                                ; 31 d2
    mov cx, strict word 00008h                ; b9 08 00
    sal ax, 1                                 ; d1 e0
    rcl dx, 1                                 ; d1 d2
    loop 0753eh                               ; e2 fa
    mov cx, word [bp-010h]                    ; 8b 4e f0
    or cx, ax                                 ; 09 c1
    or dx, bx                                 ; 09 da
    movzx ax, byte [bp-00213h]                ; 0f b6 86 ed fd
    mov bx, cx                                ; 89 cb
    or bx, ax                                 ; 09 c3
    mov word [bp-00ch], bx                    ; 89 5e f4
    test dx, dx                               ; 85 d2
    jne short 07561h                          ; 75 06
    cmp bx, 00200h                            ; 81 fb 00 02
    je short 07581h                           ; 74 20
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 42 a3
    push dx                                   ; 52
    push word [bp-00ch]                       ; ff 76 f4
    push word [bp-008h]                       ; ff 76 f8
    push 00b6dh                               ; 68 6d 0b
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 6e a3
    add sp, strict byte 0000ah                ; 83 c4 0a
    jmp near 076adh                           ; e9 2c 01
    cmp di, strict byte 00040h                ; 83 ff 40
    jnbe short 07588h                         ; 77 02
    jne short 07592h                          ; 75 0a
    mov dword [bp-004h], strict dword 000ff003fh ; 66 c7 46 fc 3f 00 ff 00
    jmp short 075abh                          ; eb 19
    cmp di, strict byte 00020h                ; 83 ff 20
    jnbe short 07599h                         ; 77 02
    jne short 075a3h                          ; 75 0a
    mov dword [bp-004h], strict dword 000800020h ; 66 c7 46 fc 20 00 80 00
    jmp short 075abh                          ; eb 08
    mov dword [bp-004h], strict dword 000400020h ; 66 c7 46 fc 20 00 40 00
    mov bx, word [bp-002h]                    ; 8b 5e fe
    imul bx, word [bp-004h]                   ; 0f af 5e fc
    mov ax, word [bp-00ah]                    ; 8b 46 f6
    mov dx, di                                ; 89 fa
    xor cx, cx                                ; 31 c9
    call 08c50h                               ; e8 94 16
    mov word [bp-00eh], dx                    ; 89 56 f2
    mov es, [bp-006h]                         ; 8e 46 fa
    mov cl, byte [es:si+001e8h]               ; 26 8a 8c e8 01
    mov ch, cl                                ; 88 cd
    add ch, 008h                              ; 80 c5 08
    movzx dx, cl                              ; 0f b6 d1
    sal dx, 002h                              ; c1 e2 02
    mov bx, si                                ; 89 f3
    add bx, dx                                ; 01 d3
    mov dx, word [bp-0021ch]                  ; 8b 96 e4 fd
    mov word [es:bx+001d8h], dx               ; 26 89 97 d8 01
    mov dl, byte [bp-008h]                    ; 8a 56 f8
    mov byte [es:bx+001dah], dl               ; 26 88 97 da 01
    movzx dx, ch                              ; 0f b6 d5
    imul dx, dx, strict byte 00018h           ; 6b d2 18
    mov bx, si                                ; 89 f3
    add bx, dx                                ; 01 d3
    db  066h, 026h, 0c7h, 047h, 01eh, 004h, 0ffh, 000h, 000h
    ; mov dword [es:bx+01eh], strict dword 00000ff04h ; 66 26 c7 47 1e 04 ff 00 00
    mov dx, word [bp-00ch]                    ; 8b 56 f4
    mov word [es:bx+024h], dx                 ; 26 89 57 24
    mov byte [es:bx+023h], 001h               ; 26 c6 47 23 01
    mov dx, word [bp-002h]                    ; 8b 56 fe
    mov word [es:bx+026h], dx                 ; 26 89 57 26
    mov dx, word [bp-004h]                    ; 8b 56 fc
    mov word [es:bx+02ah], dx                 ; 26 89 57 2a
    cmp word [bp-00eh], strict byte 00000h    ; 83 7e f2 00
    jne short 0761fh                          ; 75 05
    cmp ax, 00400h                            ; 3d 00 04
    jbe short 07627h                          ; 76 08
    mov word [es:bx+028h], 00400h             ; 26 c7 47 28 00 04
    jmp short 0762bh                          ; eb 04
    mov word [es:bx+028h], ax                 ; 26 89 47 28
    movzx bx, ch                              ; 0f b6 dd
    imul bx, bx, strict byte 00018h           ; 6b db 18
    mov es, [bp-006h]                         ; 8e 46 fa
    add bx, si                                ; 01 f3
    mov dx, word [bp-002h]                    ; 8b 56 fe
    mov word [es:bx+02ch], dx                 ; 26 89 57 2c
    mov dx, word [bp-004h]                    ; 8b 56 fc
    mov word [es:bx+030h], dx                 ; 26 89 57 30
    cmp word [bp-00eh], strict byte 00000h    ; 83 7e f2 00
    jne short 0764fh                          ; 75 05
    cmp ax, 00400h                            ; 3d 00 04
    jbe short 07657h                          ; 76 08
    mov word [es:bx+02eh], 00400h             ; 26 c7 47 2e 00 04
    jmp short 0765bh                          ; eb 04
    mov word [es:bx+02eh], ax                 ; 26 89 47 2e
    movzx bx, ch                              ; 0f b6 dd
    imul bx, bx, strict byte 00018h           ; 6b db 18
    mov es, [bp-006h]                         ; 8e 46 fa
    add bx, si                                ; 01 f3
    mov ax, word [bp-00ah]                    ; 8b 46 f6
    mov word [es:bx+032h], ax                 ; 26 89 47 32
    mov word [es:bx+034h], di                 ; 26 89 7f 34
    mov al, byte [es:si+0019eh]               ; 26 8a 84 9e 01
    mov ah, cl                                ; 88 cc
    add ah, 008h                              ; 80 c4 08
    movzx bx, al                              ; 0f b6 d8
    add bx, si                                ; 01 f3
    mov byte [es:bx+0019fh], ah               ; 26 88 a7 9f 01
    db  0feh, 0c0h
    ; inc al                                    ; fe c0
    mov byte [es:si+0019eh], al               ; 26 88 84 9e 01
    mov dx, strict word 00075h                ; ba 75 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 6b 9f
    db  0feh, 0c0h
    ; inc al                                    ; fe c0
    movzx bx, al                              ; 0f b6 d8
    mov dx, strict word 00075h                ; ba 75 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 6b 9f
    db  0feh, 0c1h
    ; inc cl                                    ; fe c1
    mov es, [bp-006h]                         ; 8e 46 fa
    mov byte [es:si+001e8h], cl               ; 26 88 8c e8 01
    inc word [bp-008h]                        ; ff 46 f8
    cmp word [bp-008h], strict byte 00010h    ; 83 7e f8 10
    jnl short 07707h                          ; 7d 51
    mov byte [bp-01ah], 012h                  ; c6 46 e6 12
    xor al, al                                ; 30 c0
    mov byte [bp-019h], al                    ; 88 46 e7
    mov byte [bp-018h], al                    ; 88 46 e8
    mov byte [bp-017h], al                    ; 88 46 e9
    mov byte [bp-016h], 005h                  ; c6 46 ea 05
    mov byte [bp-015h], al                    ; 88 46 eb
    push strict byte 00005h                   ; 6a 05
    lea dx, [bp-0021ah]                       ; 8d 96 e6 fd
    push SS                                   ; 16
    push dx                                   ; 52
    push strict byte 00006h                   ; 6a 06
    movzx dx, byte [bp-008h]                  ; 0f b6 56 f8
    mov cx, ss                                ; 8c d1
    lea bx, [bp-01ah]                         ; 8d 5e e6
    mov ax, word [bp-0021ch]                  ; 8b 86 e4 fd
    call 0727eh                               ; e8 98 fb
    test al, al                               ; 84 c0
    je short 076f5h                           ; 74 0b
    push 00afbh                               ; 68 fb 0a
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 f7 a1
    add sp, strict byte 00004h                ; 83 c4 04
    test byte [bp-0021ah], 0e0h               ; f6 86 e6 fd e0
    jne short 076adh                          ; 75 b1
    test byte [bp-0021ah], 01fh               ; f6 86 e6 fd 1f
    je near 074aah                            ; 0f 84 a5 fd
    jmp short 076adh                          ; eb a6
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop dx                                    ; 5a
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
_scsi_init:                                  ; 0xf770e LB 0x64
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 02 9f
    mov bx, 00122h                            ; bb 22 01
    mov es, ax                                ; 8e c0
    mov byte [es:bx+001e8h], 000h             ; 26 c6 87 e8 01 00
    mov AL, strict byte 055h                  ; b0 55
    mov dx, 00332h                            ; ba 32 03
    out DX, AL                                ; ee
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    cmp AL, strict byte 055h                  ; 3c 55
    jne short 0773eh                          ; 75 0c
    xor al, al                                ; 30 c0
    mov dx, 00333h                            ; ba 33 03
    out DX, AL                                ; ee
    mov ax, 00330h                            ; b8 30 03
    call 07489h                               ; e8 4b fd
    mov AL, strict byte 055h                  ; b0 55
    mov dx, 00342h                            ; ba 42 03
    out DX, AL                                ; ee
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    cmp AL, strict byte 055h                  ; 3c 55
    jne short 07757h                          ; 75 0c
    xor al, al                                ; 30 c0
    mov dx, 00343h                            ; ba 43 03
    out DX, AL                                ; ee
    mov ax, 00340h                            ; b8 40 03
    call 07489h                               ; e8 32 fd
    mov AL, strict byte 055h                  ; b0 55
    mov dx, 00352h                            ; ba 52 03
    out DX, AL                                ; ee
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    cmp AL, strict byte 055h                  ; 3c 55
    jne short 07770h                          ; 75 0c
    xor al, al                                ; 30 c0
    mov dx, 00353h                            ; ba 53 03
    out DX, AL                                ; ee
    mov ax, 00350h                            ; b8 50 03
    call 07489h                               ; e8 19 fd
    pop bp                                    ; 5d
    retn                                      ; c3
high_bits_save_:                             ; 0xf7772 LB 0x14
    push bx                                   ; 53
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bx, ax                                ; 89 c3
    shr eax, 010h                             ; 66 c1 e8 10
    mov es, dx                                ; 8e c2
    mov word [es:bx+00268h], ax               ; 26 89 87 68 02
    pop bp                                    ; 5d
    pop bx                                    ; 5b
    retn                                      ; c3
high_bits_restore_:                          ; 0xf7786 LB 0x14
    push bx                                   ; 53
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bx, ax                                ; 89 c3
    mov es, dx                                ; 8e c2
    mov ax, word [es:bx+00268h]               ; 26 8b 87 68 02
    sal eax, 010h                             ; 66 c1 e0 10
    pop bp                                    ; 5d
    pop bx                                    ; 5b
    retn                                      ; c3
ahci_ctrl_set_bits_:                         ; 0xf779a LB 0x40
    push si                                   ; 56
    push di                                   ; 57
    enter 00002h, 000h                        ; c8 02 00 00
    mov si, ax                                ; 89 c6
    mov ax, dx                                ; 89 d0
    mov word [bp-002h], bx                    ; 89 5e fe
    mov di, cx                                ; 89 cf
    xor cx, cx                                ; 31 c9
    mov dx, si                                ; 89 f2
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    lea bx, [si+004h]                         ; 8d 5c 04
    mov dx, bx                                ; 89 da
    in eax, DX                                ; 66 ed
    db  08bh, 0d0h
    ; mov dx, ax                                ; 8b d0
    shr eax, 010h                             ; 66 c1 e8 10
    xchg dx, ax                               ; 92
    or ax, word [bp-002h]                     ; 0b 46 fe
    mov cx, dx                                ; 89 d1
    or cx, di                                 ; 09 f9
    mov dx, bx                                ; 89 da
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn                                      ; c3
ahci_ctrl_clear_bits_:                       ; 0xf77da LB 0x44
    push si                                   ; 56
    push di                                   ; 57
    enter 00002h, 000h                        ; c8 02 00 00
    mov si, ax                                ; 89 c6
    mov ax, dx                                ; 89 d0
    mov di, bx                                ; 89 df
    mov word [bp-002h], cx                    ; 89 4e fe
    xor cx, cx                                ; 31 c9
    mov dx, si                                ; 89 f2
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    lea bx, [si+004h]                         ; 8d 5c 04
    mov dx, bx                                ; 89 da
    in eax, DX                                ; 66 ed
    db  08bh, 0d0h
    ; mov dx, ax                                ; 8b d0
    shr eax, 010h                             ; 66 c1 e8 10
    xchg dx, ax                               ; 92
    not di                                    ; f7 d7
    mov cx, word [bp-002h]                    ; 8b 4e fe
    not cx                                    ; f7 d1
    and ax, di                                ; 21 f8
    and cx, dx                                ; 21 d1
    mov dx, bx                                ; 89 da
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn                                      ; c3
ahci_ctrl_is_bit_set_:                       ; 0xf781e LB 0x36
    push si                                   ; 56
    push di                                   ; 57
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov si, ax                                ; 89 c6
    mov ax, dx                                ; 89 d0
    mov di, cx                                ; 89 cf
    xor cx, cx                                ; 31 c9
    mov dx, si                                ; 89 f2
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    lea dx, [si+004h]                         ; 8d 54 04
    in eax, DX                                ; 66 ed
    db  08bh, 0d0h
    ; mov dx, ax                                ; 8b d0
    shr eax, 010h                             ; 66 c1 e8 10
    xchg dx, ax                               ; 92
    test dx, di                               ; 85 fa
    jne short 0784ah                          ; 75 04
    test ax, bx                               ; 85 d8
    je short 0784eh                           ; 74 04
    mov AL, strict byte 001h                  ; b0 01
    jmp short 07850h                          ; eb 02
    xor al, al                                ; 30 c0
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn                                      ; c3
ahci_ctrl_extract_bits_:                     ; 0xf7854 LB 0x1b
    push si                                   ; 56
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov si, bx                                ; 89 de
    and ax, bx                                ; 21 d8
    and dx, cx                                ; 21 ca
    movzx cx, byte [bp+006h]                  ; 0f b6 4e 06
    jcxz 0786ah                               ; e3 06
    shr dx, 1                                 ; d1 ea
    rcr ax, 1                                 ; d1 d8
    loop 07864h                               ; e2 fa
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn 00002h                               ; c2 02 00
ahci_addr_to_phys_:                          ; 0xf786f LB 0x1e
    push bx                                   ; 53
    push cx                                   ; 51
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bx, ax                                ; 89 c3
    mov ax, dx                                ; 89 d0
    xor dx, dx                                ; 31 d2
    mov cx, strict word 00004h                ; b9 04 00
    sal ax, 1                                 ; d1 e0
    rcl dx, 1                                 ; d1 d2
    loop 0787dh                               ; e2 fa
    xor cx, cx                                ; 31 c9
    add ax, bx                                ; 01 d8
    adc dx, cx                                ; 11 ca
    pop bp                                    ; 5d
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
ahci_port_cmd_sync_:                         ; 0xf788d LB 0xd0
    push cx                                   ; 51
    push si                                   ; 56
    push di                                   ; 57
    enter 00006h, 000h                        ; c8 06 00 00
    mov si, ax                                ; 89 c6
    mov word [bp-004h], dx                    ; 89 56 fc
    mov es, dx                                ; 8e c2
    mov al, byte [es:si+00262h]               ; 26 8a 84 62 02
    mov byte [bp-002h], al                    ; 88 46 fe
    mov di, word [es:si+00260h]               ; 26 8b bc 60 02
    cmp AL, strict byte 0ffh                  ; 3c ff
    je near 07958h                            ; 0f 84 aa 00
    movzx cx, byte [es:si+00263h]             ; 26 0f b6 8c 63 02
    xor dx, dx                                ; 31 d2
    or dl, 080h                               ; 80 ca 80
    movzx ax, bl                              ; 0f b6 c3
    or dx, ax                                 ; 09 c2
    mov word [es:si], dx                      ; 26 89 14
    mov word [es:si+002h], cx                 ; 26 89 4c 02
    db  066h, 026h, 0c7h, 044h, 004h, 000h, 000h, 000h, 000h
    ; mov dword [es:si+004h], strict dword 000000000h ; 66 26 c7 44 04 00 00 00 00
    lea ax, [si+00080h]                       ; 8d 84 80 00
    mov dx, es                                ; 8c c2
    call 0786fh                               ; e8 98 ff
    mov es, [bp-004h]                         ; 8e 46 fc
    mov word [es:si+008h], ax                 ; 26 89 44 08
    mov word [es:si+00ah], dx                 ; 26 89 54 0a
    movzx si, byte [bp-002h]                  ; 0f b6 76 fe
    sal si, 007h                              ; c1 e6 07
    lea dx, [si+00118h]                       ; 8d 94 18 01
    mov bx, strict word 00011h                ; bb 11 00
    xor cx, cx                                ; 31 c9
    mov ax, di                                ; 89 f8
    call 0779ah                               ; e8 a3 fe
    lea ax, [si+00138h]                       ; 8d 84 38 01
    cwd                                       ; 99
    mov cx, dx                                ; 89 d1
    mov dx, di                                ; 89 fa
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    lea dx, [di+004h]                         ; 8d 55 04
    mov ax, strict word 00001h                ; b8 01 00
    xor cx, cx                                ; 31 c9
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    movzx ax, byte [bp-002h]                  ; 0f b6 46 fe
    sal ax, 007h                              ; c1 e0 07
    mov word [bp-006h], ax                    ; 89 46 fa
    mov si, ax                                ; 89 c6
    add si, 00110h                            ; 81 c6 10 01
    mov bx, strict word 00001h                ; bb 01 00
    mov cx, 04000h                            ; b9 00 40
    mov dx, si                                ; 89 f2
    mov ax, di                                ; 89 f8
    call 0781eh                               ; e8 e7 fe
    test al, al                               ; 84 c0
    je short 0791ah                           ; 74 df
    mov bx, strict word 00001h                ; bb 01 00
    xor cx, cx                                ; 31 c9
    mov dx, si                                ; 89 f2
    mov ax, di                                ; 89 f8
    call 0779ah                               ; e8 53 fe
    mov dx, word [bp-006h]                    ; 8b 56 fa
    add dx, 00118h                            ; 81 c2 18 01
    mov bx, strict word 00001h                ; bb 01 00
    xor cx, cx                                ; 31 c9
    mov ax, di                                ; 89 f8
    call 077dah                               ; e8 82 fe
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop cx                                    ; 59
    retn                                      ; c3
ahci_cmd_data_:                              ; 0xf795d LB 0x1c5
    push cx                                   ; 51
    push si                                   ; 56
    push di                                   ; 57
    enter 00010h, 000h                        ; c8 10 00 00
    mov di, ax                                ; 89 c7
    mov word [bp-00ch], dx                    ; 89 56 f4
    mov byte [bp-002h], bl                    ; 88 5e fe
    xor si, si                                ; 31 f6
    mov es, dx                                ; 8e c2
    mov ax, word [es:di+001eeh]               ; 26 8b 85 ee 01
    mov word [bp-004h], ax                    ; 89 46 fc
    mov word [bp-008h], si                    ; 89 76 f8
    mov word [bp-006h], ax                    ; 89 46 fa
    mov ax, word [es:di+00ah]                 ; 26 8b 45 0a
    mov word [bp-00ah], ax                    ; 89 46 f6
    mov ax, word [es:di+00ch]                 ; 26 8b 45 0c
    mov word [bp-010h], ax                    ; 89 46 f0
    mov cx, strict word 00040h                ; b9 40 00
    xor bx, bx                                ; 31 db
    mov ax, 00080h                            ; b8 80 00
    mov dx, word [bp-004h]                    ; 8b 56 fc
    call 08cbah                               ; e8 20 13
    mov es, [bp-004h]                         ; 8e 46 fc
    mov word [es:si+00080h], 08027h           ; 26 c7 84 80 00 27 80
    mov al, byte [bp-002h]                    ; 8a 46 fe
    mov byte [es:si+00082h], al               ; 26 88 84 82 00
    mov byte [es:si+00083h], 000h             ; 26 c6 84 83 00 00
    mov es, [bp-00ch]                         ; 8e 46 f4
    mov al, byte [es:di]                      ; 26 8a 05
    mov es, [bp-004h]                         ; 8e 46 fc
    mov byte [es:si+00084h], al               ; 26 88 84 84 00
    mov es, [bp-00ch]                         ; 8e 46 f4
    mov ax, word [es:di]                      ; 26 8b 05
    mov bx, word [es:di+002h]                 ; 26 8b 5d 02
    mov cx, strict word 00008h                ; b9 08 00
    shr bx, 1                                 ; d1 eb
    rcr ax, 1                                 ; d1 d8
    loop 079cdh                               ; e2 fa
    mov es, [bp-004h]                         ; 8e 46 fc
    mov byte [es:si+00085h], al               ; 26 88 84 85 00
    mov es, [bp-00ch]                         ; 8e 46 f4
    mov ax, word [es:di+002h]                 ; 26 8b 45 02
    mov es, [bp-004h]                         ; 8e 46 fc
    mov byte [es:si+00086h], al               ; 26 88 84 86 00
    mov byte [es:si+00087h], 040h             ; 26 c6 84 87 00 40
    mov es, [bp-00ch]                         ; 8e 46 f4
    mov ax, word [es:di+002h]                 ; 26 8b 45 02
    shr ax, 008h                              ; c1 e8 08
    mov es, [bp-004h]                         ; 8e 46 fc
    mov byte [es:si+00088h], al               ; 26 88 84 88 00
    mov word [es:si+00089h], strict word 00000h ; 26 c7 84 89 00 00 00
    mov byte [es:si+0008bh], 000h             ; 26 c6 84 8b 00 00
    mov al, byte [bp-00ah]                    ; 8a 46 f6
    mov byte [es:si+0008ch], al               ; 26 88 84 8c 00
    mov ax, word [bp-00ah]                    ; 8b 46 f6
    shr ax, 008h                              ; c1 e8 08
    mov byte [es:si+0008dh], al               ; 26 88 84 8d 00
    mov word [es:si+00276h], strict word 00010h ; 26 c7 84 76 02 10 00
    mov ax, word [bp-00ah]                    ; 8b 46 f6
    xor dx, dx                                ; 31 d2
    mov bx, word [bp-010h]                    ; 8b 5e f0
    xor cx, cx                                ; 31 c9
    call 08c89h                               ; e8 53 12
    push dx                                   ; 52
    push ax                                   ; 50
    mov es, [bp-00ch]                         ; 8e 46 f4
    mov bx, word [es:di+004h]                 ; 26 8b 5d 04
    mov cx, word [es:di+006h]                 ; 26 8b 4d 06
    mov ax, 0026ah                            ; b8 6a 02
    mov dx, word [bp-004h]                    ; 8b 56 fc
    call 08baah                               ; e8 5e 11
    mov es, [bp-004h]                         ; 8e 46 fc
    movzx ax, byte [es:si+00263h]             ; 26 0f b6 84 63 02
    mov dx, word [es:si+0027eh]               ; 26 8b 94 7e 02
    add dx, strict byte 0ffffh                ; 83 c2 ff
    mov bx, word [es:si+00280h]               ; 26 8b 9c 80 02
    adc bx, strict byte 0ffffh                ; 83 d3 ff
    mov word [bp-00eh], bx                    ; 89 5e f2
    mov bx, ax                                ; 89 c3
    sal bx, 004h                              ; c1 e3 04
    mov word [es:bx+0010ch], dx               ; 26 89 97 0c 01
    mov dx, word [bp-00eh]                    ; 8b 56 f2
    mov word [es:bx+0010eh], dx               ; 26 89 97 0e 01
    mov cx, word [es:si+0027ah]               ; 26 8b 8c 7a 02
    mov dx, word [es:si+0027ch]               ; 26 8b 94 7c 02
    mov word [es:bx+00100h], cx               ; 26 89 8f 00 01
    mov word [es:bx+00102h], dx               ; 26 89 97 02 01
    inc ax                                    ; 40
    mov es, [bp-00ch]                         ; 8e 46 f4
    cmp word [es:di+01ch], strict byte 00000h ; 26 83 7d 1c 00
    je short 07ac5h                           ; 74 2c
    mov dx, word [es:di+01ch]                 ; 26 8b 55 1c
    dec dx                                    ; 4a
    mov di, ax                                ; 89 c7
    sal di, 004h                              ; c1 e7 04
    mov es, [bp-004h]                         ; 8e 46 fc
    mov word [es:di+0010ch], dx               ; 26 89 95 0c 01
    mov word [es:di+0010eh], si               ; 26 89 b5 0e 01
    mov dx, word [es:si+00264h]               ; 26 8b 94 64 02
    mov bx, word [es:si+00266h]               ; 26 8b 9c 66 02
    mov word [es:di+00100h], dx               ; 26 89 95 00 01
    mov word [es:di+00102h], bx               ; 26 89 9d 02 01
    inc ax                                    ; 40
    les bx, [bp-008h]                         ; c4 5e f8
    mov byte [es:bx+00263h], al               ; 26 88 87 63 02
    xor ax, ax                                ; 31 c0
    les bx, [bp-008h]                         ; c4 5e f8
    movzx dx, byte [es:bx+00263h]             ; 26 0f b6 97 63 02
    cmp ax, dx                                ; 39 d0
    jnc short 07adfh                          ; 73 03
    inc ax                                    ; 40
    jmp short 07acfh                          ; eb f0
    mov al, byte [bp-002h]                    ; 8a 46 fe
    cmp AL, strict byte 035h                  ; 3c 35
    jne short 07aech                          ; 75 06
    mov byte [bp-002h], 040h                  ; c6 46 fe 40
    jmp short 07b00h                          ; eb 14
    cmp AL, strict byte 0a0h                  ; 3c a0
    jne short 07afch                          ; 75 0c
    or byte [bp-002h], 020h                   ; 80 4e fe 20
    or byte [es:bx+00083h], 001h              ; 26 80 8f 83 00 01
    jmp short 07b00h                          ; eb 04
    mov byte [bp-002h], 000h                  ; c6 46 fe 00
    or byte [bp-002h], 005h                   ; 80 4e fe 05
    movzx bx, byte [bp-002h]                  ; 0f b6 5e fe
    mov ax, word [bp-008h]                    ; 8b 46 f8
    mov dx, word [bp-006h]                    ; 8b 56 fa
    call 0788dh                               ; e8 7c fd
    mov ax, word [bp-008h]                    ; 8b 46 f8
    add ax, 0026ah                            ; 05 6a 02
    mov dx, word [bp-006h]                    ; 8b 56 fa
    call 08c20h                               ; e8 03 11
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop cx                                    ; 59
    retn                                      ; c3
ahci_port_deinit_current_:                   ; 0xf7b22 LB 0x13f
    push bx                                   ; 53
    push cx                                   ; 51
    push si                                   ; 56
    push di                                   ; 57
    enter 00006h, 000h                        ; c8 06 00 00
    mov di, ax                                ; 89 c7
    mov word [bp-004h], dx                    ; 89 56 fc
    mov es, dx                                ; 8e c2
    mov si, word [es:di+00260h]               ; 26 8b b5 60 02
    mov al, byte [es:di+00262h]               ; 26 8a 85 62 02
    mov byte [bp-002h], al                    ; 88 46 fe
    cmp AL, strict byte 0ffh                  ; 3c ff
    je near 07c5bh                            ; 0f 84 17 01
    movzx dx, al                              ; 0f b6 d0
    sal dx, 007h                              ; c1 e2 07
    add dx, 00118h                            ; 81 c2 18 01
    mov bx, strict word 00011h                ; bb 11 00
    xor cx, cx                                ; 31 c9
    mov ax, si                                ; 89 f0
    call 077dah                               ; e8 82 fc
    movzx ax, byte [bp-002h]                  ; 0f b6 46 fe
    sal ax, 007h                              ; c1 e0 07
    mov word [bp-006h], ax                    ; 89 46 fa
    mov dx, ax                                ; 89 c2
    add dx, 00118h                            ; 81 c2 18 01
    mov bx, 0c011h                            ; bb 11 c0
    xor cx, cx                                ; 31 c9
    mov ax, si                                ; 89 f0
    call 0781eh                               ; e8 ac fc
    cmp AL, strict byte 001h                  ; 3c 01
    je short 07b58h                           ; 74 e2
    mov cx, strict word 00020h                ; b9 20 00
    xor bx, bx                                ; 31 db
    mov ax, di                                ; 89 f8
    mov dx, word [bp-004h]                    ; 8b 56 fc
    call 08cbah                               ; e8 37 11
    lea ax, [di+00080h]                       ; 8d 85 80 00
    mov cx, strict word 00040h                ; b9 40 00
    xor bx, bx                                ; 31 db
    mov dx, word [bp-004h]                    ; 8b 56 fc
    call 08cbah                               ; e8 28 11
    lea ax, [di+00200h]                       ; 8d 85 00 02
    mov cx, strict word 00060h                ; b9 60 00
    xor bx, bx                                ; 31 db
    mov dx, word [bp-004h]                    ; 8b 56 fc
    call 08cbah                               ; e8 19 11
    mov ax, word [bp-006h]                    ; 8b 46 fa
    add ax, 00108h                            ; 05 08 01
    cwd                                       ; 99
    mov cx, dx                                ; 89 d1
    mov dx, si                                ; 89 f2
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    lea bx, [si+004h]                         ; 8d 5c 04
    xor ax, ax                                ; 31 c0
    xor cx, cx                                ; 31 c9
    mov dx, bx                                ; 89 da
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov ax, word [bp-006h]                    ; 8b 46 fa
    add ax, 0010ch                            ; 05 0c 01
    cwd                                       ; 99
    mov cx, dx                                ; 89 d1
    mov dx, si                                ; 89 f2
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    xor ax, ax                                ; 31 c0
    xor cx, cx                                ; 31 c9
    mov dx, bx                                ; 89 da
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov ax, word [bp-006h]                    ; 8b 46 fa
    db  0feh, 0c4h
    ; inc ah                                    ; fe c4
    cwd                                       ; 99
    mov cx, dx                                ; 89 d1
    mov dx, si                                ; 89 f2
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    xor ax, ax                                ; 31 c0
    xor cx, cx                                ; 31 c9
    mov dx, bx                                ; 89 da
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov ax, word [bp-006h]                    ; 8b 46 fa
    add ax, 00104h                            ; 05 04 01
    cwd                                       ; 99
    mov cx, dx                                ; 89 d1
    mov dx, si                                ; 89 f2
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    xor ax, ax                                ; 31 c0
    xor cx, cx                                ; 31 c9
    mov dx, bx                                ; 89 da
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov ax, word [bp-006h]                    ; 8b 46 fa
    add ax, 00114h                            ; 05 14 01
    cwd                                       ; 99
    mov cx, dx                                ; 89 d1
    mov dx, si                                ; 89 f2
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    xor ax, ax                                ; 31 c0
    xor cx, cx                                ; 31 c9
    mov dx, bx                                ; 89 da
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov es, [bp-004h]                         ; 8e 46 fc
    mov byte [es:di+00262h], 0ffh             ; 26 c6 85 62 02 ff
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
ahci_port_init_:                             ; 0xf7c61 LB 0x201
    push cx                                   ; 51
    push si                                   ; 56
    push di                                   ; 57
    enter 00006h, 000h                        ; c8 06 00 00
    mov si, ax                                ; 89 c6
    mov word [bp-004h], dx                    ; 89 56 fc
    mov byte [bp-002h], bl                    ; 88 5e fe
    call 07b22h                               ; e8 af fe
    movzx dx, bl                              ; 0f b6 d3
    sal dx, 007h                              ; c1 e2 07
    add dx, 00118h                            ; 81 c2 18 01
    mov es, [bp-004h]                         ; 8e 46 fc
    mov ax, word [es:si+00260h]               ; 26 8b 84 60 02
    mov bx, strict word 00011h                ; bb 11 00
    xor cx, cx                                ; 31 c9
    call 077dah                               ; e8 4d fb
    movzx di, byte [bp-002h]                  ; 0f b6 7e fe
    sal di, 007h                              ; c1 e7 07
    lea dx, [di+00118h]                       ; 8d 95 18 01
    mov es, [bp-004h]                         ; 8e 46 fc
    mov ax, word [es:si+00260h]               ; 26 8b 84 60 02
    mov bx, 0c011h                            ; bb 11 c0
    xor cx, cx                                ; 31 c9
    call 0781eh                               ; e8 76 fb
    cmp AL, strict byte 001h                  ; 3c 01
    je short 07c8dh                           ; 74 e1
    mov cx, strict word 00020h                ; b9 20 00
    xor bx, bx                                ; 31 db
    mov ax, si                                ; 89 f0
    mov dx, word [bp-004h]                    ; 8b 56 fc
    call 08cbah                               ; e8 01 10
    lea ax, [si+00080h]                       ; 8d 84 80 00
    mov cx, strict word 00040h                ; b9 40 00
    xor bx, bx                                ; 31 db
    mov dx, word [bp-004h]                    ; 8b 56 fc
    call 08cbah                               ; e8 f2 0f
    mov ax, si                                ; 89 f0
    add ah, 002h                              ; 80 c4 02
    mov word [bp-006h], ax                    ; 89 46 fa
    mov cx, strict word 00060h                ; b9 60 00
    xor bx, bx                                ; 31 db
    mov dx, word [bp-004h]                    ; 8b 56 fc
    call 08cbah                               ; e8 df 0f
    lea ax, [di+00108h]                       ; 8d 85 08 01
    cwd                                       ; 99
    mov es, [bp-004h]                         ; 8e 46 fc
    mov bx, word [es:si+00260h]               ; 26 8b 9c 60 02
    mov cx, dx                                ; 89 d1
    mov dx, bx                                ; 89 da
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov ax, word [bp-006h]                    ; 8b 46 fa
    mov dx, word [bp-004h]                    ; 8b 56 fc
    call 0786fh                               ; e8 71 fb
    mov es, [bp-004h]                         ; 8e 46 fc
    mov bx, word [es:si+00260h]               ; 26 8b 9c 60 02
    add bx, strict byte 00004h                ; 83 c3 04
    mov cx, dx                                ; 89 d1
    mov dx, bx                                ; 89 da
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    lea ax, [di+0010ch]                       ; 8d 85 0c 01
    cwd                                       ; 99
    mov es, [bp-004h]                         ; 8e 46 fc
    mov bx, word [es:si+00260h]               ; 26 8b 9c 60 02
    mov cx, dx                                ; 89 d1
    mov dx, bx                                ; 89 da
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov es, [bp-004h]                         ; 8e 46 fc
    mov dx, word [es:si+00260h]               ; 26 8b 94 60 02
    add dx, strict byte 00004h                ; 83 c2 04
    xor ax, ax                                ; 31 c0
    xor cx, cx                                ; 31 c9
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    lea ax, [di+00100h]                       ; 8d 85 00 01
    cwd                                       ; 99
    mov es, [bp-004h]                         ; 8e 46 fc
    mov bx, word [es:si+00260h]               ; 26 8b 9c 60 02
    mov cx, dx                                ; 89 d1
    mov dx, bx                                ; 89 da
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov ax, si                                ; 89 f0
    mov dx, word [bp-004h]                    ; 8b 56 fc
    call 0786fh                               ; e8 05 fb
    mov es, [bp-004h]                         ; 8e 46 fc
    mov bx, word [es:si+00260h]               ; 26 8b 9c 60 02
    add bx, strict byte 00004h                ; 83 c3 04
    mov cx, dx                                ; 89 d1
    mov dx, bx                                ; 89 da
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    lea ax, [di+00104h]                       ; 8d 85 04 01
    cwd                                       ; 99
    mov es, [bp-004h]                         ; 8e 46 fc
    mov bx, word [es:si+00260h]               ; 26 8b 9c 60 02
    mov cx, dx                                ; 89 d1
    mov dx, bx                                ; 89 da
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov es, [bp-004h]                         ; 8e 46 fc
    mov dx, word [es:si+00260h]               ; 26 8b 94 60 02
    add dx, strict byte 00004h                ; 83 c2 04
    xor ax, ax                                ; 31 c0
    xor cx, cx                                ; 31 c9
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    lea ax, [di+00114h]                       ; 8d 85 14 01
    cwd                                       ; 99
    mov es, [bp-004h]                         ; 8e 46 fc
    mov bx, word [es:si+00260h]               ; 26 8b 9c 60 02
    mov cx, dx                                ; 89 d1
    mov dx, bx                                ; 89 da
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov es, [bp-004h]                         ; 8e 46 fc
    mov dx, word [es:si+00260h]               ; 26 8b 94 60 02
    add dx, strict byte 00004h                ; 83 c2 04
    xor ax, ax                                ; 31 c0
    xor cx, cx                                ; 31 c9
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    lea ax, [di+00110h]                       ; 8d 85 10 01
    cwd                                       ; 99
    mov es, [bp-004h]                         ; 8e 46 fc
    mov bx, word [es:si+00260h]               ; 26 8b 9c 60 02
    mov cx, dx                                ; 89 d1
    mov dx, bx                                ; 89 da
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov es, [bp-004h]                         ; 8e 46 fc
    mov dx, word [es:si+00260h]               ; 26 8b 94 60 02
    add dx, strict byte 00004h                ; 83 c2 04
    mov ax, strict word 0ffffh                ; b8 ff ff
    mov cx, ax                                ; 89 c1
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    lea ax, [di+00130h]                       ; 8d 85 30 01
    cwd                                       ; 99
    mov es, [bp-004h]                         ; 8e 46 fc
    mov bx, word [es:si+00260h]               ; 26 8b 9c 60 02
    mov cx, dx                                ; 89 d1
    mov dx, bx                                ; 89 da
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov es, [bp-004h]                         ; 8e 46 fc
    mov dx, word [es:si+00260h]               ; 26 8b 94 60 02
    add dx, strict byte 00004h                ; 83 c2 04
    mov ax, strict word 0ffffh                ; b8 ff ff
    mov cx, ax                                ; 89 c1
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov al, byte [bp-002h]                    ; 8a 46 fe
    mov es, [bp-004h]                         ; 8e 46 fc
    mov byte [es:si+00262h], al               ; 26 88 84 62 02
    mov byte [es:si+00263h], 000h             ; 26 c6 84 63 02 00
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop cx                                    ; 59
    retn                                      ; c3
@ahci_read_sectors:                          ; 0xf7e62 LB 0x91
    push si                                   ; 56
    push di                                   ; 57
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    les di, [bp+008h]                         ; c4 7e 08
    movzx di, byte [es:di+008h]               ; 26 0f b6 7d 08
    sub di, strict byte 0000ch                ; 83 ef 0c
    cmp di, strict byte 00004h                ; 83 ff 04
    jbe short 07e86h                          ; 76 0f
    push di                                   ; 57
    push 00b9ch                               ; 68 9c 0b
    push 00baeh                               ; 68 ae 0b
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 66 9a
    add sp, strict byte 00008h                ; 83 c4 08
    les bx, [bp+008h]                         ; c4 5e 08
    mov dx, word [es:bx+001eeh]               ; 26 8b 97 ee 01
    xor ax, ax                                ; 31 c0
    call 07772h                               ; e8 df f8
    mov es, [bp+00ah]                         ; 8e 46 0a
    add di, bx                                ; 01 df
    movzx bx, byte [es:di+001e9h]             ; 26 0f b6 9d e9 01
    mov di, word [bp+008h]                    ; 8b 7e 08
    mov dx, word [es:di+001eeh]               ; 26 8b 95 ee 01
    xor ax, ax                                ; 31 c0
    call 07c61h                               ; e8 b6 fd
    mov bx, strict word 00025h                ; bb 25 00
    mov ax, di                                ; 89 f8
    mov dx, word [bp+00ah]                    ; 8b 56 0a
    call 0795dh                               ; e8 a7 fa
    mov es, [bp+00ah]                         ; 8e 46 0a
    mov bx, di                                ; 89 fb
    mov ax, word [es:bx+00ah]                 ; 26 8b 47 0a
    mov word [es:bx+014h], ax                 ; 26 89 47 14
    mov cx, ax                                ; 89 c1
    sal cx, 009h                              ; c1 e1 09
    shr cx, 1                                 ; d1 e9
    mov di, word [es:di+004h]                 ; 26 8b 7d 04
    mov ax, word [es:bx+006h]                 ; 26 8b 47 06
    mov si, di                                ; 89 fe
    mov dx, ax                                ; 89 c2
    mov es, ax                                ; 8e c0
    push DS                                   ; 1e
    mov ds, dx                                ; 8e da
    rep movsw                                 ; f3 a5
    pop DS                                    ; 1f
    mov es, [bp+00ah]                         ; 8e 46 0a
    mov dx, word [es:bx+001eeh]               ; 26 8b 97 ee 01
    xor ax, ax                                ; 31 c0
    call 07786h                               ; e8 9b f8
    xor ax, ax                                ; 31 c0
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn 00004h                               ; c2 04 00
@ahci_write_sectors:                         ; 0xf7ef3 LB 0x6d
    push si                                   ; 56
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov si, word [bp+006h]                    ; 8b 76 06
    mov cx, word [bp+008h]                    ; 8b 4e 08
    mov es, cx                                ; 8e c1
    movzx bx, byte [es:si+008h]               ; 26 0f b6 5c 08
    sub bx, strict byte 0000ch                ; 83 eb 0c
    cmp bx, strict byte 00004h                ; 83 fb 04
    jbe short 07f1bh                          ; 76 0f
    push bx                                   ; 53
    push 00bcdh                               ; 68 cd 0b
    push 00baeh                               ; 68 ae 0b
    push strict byte 00007h                   ; 6a 07
    call 018e9h                               ; e8 d1 99
    add sp, strict byte 00008h                ; 83 c4 08
    mov es, cx                                ; 8e c1
    mov dx, word [es:si+001eeh]               ; 26 8b 94 ee 01
    xor ax, ax                                ; 31 c0
    call 07772h                               ; e8 4b f8
    mov es, cx                                ; 8e c1
    add bx, si                                ; 01 f3
    movzx bx, byte [es:bx+001e9h]             ; 26 0f b6 9f e9 01
    mov dx, word [es:si+001eeh]               ; 26 8b 94 ee 01
    xor ax, ax                                ; 31 c0
    call 07c61h                               ; e8 26 fd
    mov bx, strict word 00035h                ; bb 35 00
    mov ax, si                                ; 89 f0
    mov dx, cx                                ; 89 ca
    call 0795dh                               ; e8 18 fa
    mov es, cx                                ; 8e c1
    mov dx, word [es:si+00ah]                 ; 26 8b 54 0a
    mov word [es:si+014h], dx                 ; 26 89 54 14
    mov dx, word [es:si+001eeh]               ; 26 8b 94 ee 01
    xor ax, ax                                ; 31 c0
    call 07786h                               ; e8 2d f8
    xor ax, ax                                ; 31 c0
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn 00004h                               ; c2 04 00
ahci_cmd_packet_:                            ; 0xf7f60 LB 0x16e
    push si                                   ; 56
    push di                                   ; 57
    enter 0000eh, 000h                        ; c8 0e 00 00
    push ax                                   ; 50
    mov byte [bp-002h], dl                    ; 88 56 fe
    mov word [bp-00eh], bx                    ; 89 5e f2
    mov word [bp-00ch], cx                    ; 89 4e f4
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 a3 96
    mov si, 00122h                            ; be 22 01
    mov word [bp-004h], ax                    ; 89 46 fc
    cmp byte [bp+00eh], 002h                  ; 80 7e 0e 02
    jne short 07fa4h                          ; 75 1f
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 1e 99
    push 00be0h                               ; 68 e0 0b
    push 00bf0h                               ; 68 f0 0b
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 4e 99
    add sp, strict byte 00006h                ; 83 c4 06
    mov ax, strict word 00001h                ; b8 01 00
    jmp near 080c8h                           ; e9 24 01
    test byte [bp+008h], 001h                 ; f6 46 08 01
    jne short 07f9eh                          ; 75 f4
    mov ax, word [bp+00ah]                    ; 8b 46 0a
    mov dx, word [bp+00ch]                    ; 8b 56 0c
    mov cx, strict word 00008h                ; b9 08 00
    sal ax, 1                                 ; d1 e0
    rcl dx, 1                                 ; d1 d2
    loop 07fb3h                               ; e2 fa
    mov es, [bp-004h]                         ; 8e 46 fc
    mov word [es:si], ax                      ; 26 89 04
    mov word [es:si+002h], dx                 ; 26 89 54 02
    mov ax, word [bp+010h]                    ; 8b 46 10
    mov word [es:si+004h], ax                 ; 26 89 44 04
    mov ax, word [bp+012h]                    ; 8b 46 12
    mov word [es:si+006h], ax                 ; 26 89 44 06
    mov bx, word [es:si+00ch]                 ; 26 8b 5c 0c
    mov ax, word [bp+00ah]                    ; 8b 46 0a
    mov dx, word [bp+00ch]                    ; 8b 56 0c
    xor cx, cx                                ; 31 c9
    call 08c50h                               ; e8 70 0c
    mov word [es:si+00ah], ax                 ; 26 89 44 0a
    xor di, di                                ; 31 ff
    mov ax, word [es:si+001eeh]               ; 26 8b 84 ee 01
    mov word [bp-006h], ax                    ; 89 46 fa
    mov word [bp-00ah], di                    ; 89 7e f6
    mov word [bp-008h], ax                    ; 89 46 f8
    sub word [bp-010h], strict byte 0000ch    ; 83 6e f0 0c
    xor ax, ax                                ; 31 c0
    mov dx, word [bp-006h]                    ; 8b 56 fa
    call 07772h                               ; e8 72 f7
    mov es, [bp-004h]                         ; 8e 46 fc
    mov bx, word [bp-010h]                    ; 8b 5e f0
    add bx, si                                ; 01 f3
    movzx bx, byte [es:bx+001e9h]             ; 26 0f b6 9f e9 01
    mov dx, word [es:si+001eeh]               ; 26 8b 94 ee 01
    xor ax, ax                                ; 31 c0
    call 07c61h                               ; e8 49 fc
    movzx ax, byte [bp-002h]                  ; 0f b6 46 fe
    push ax                                   ; 50
    mov bx, word [bp-00eh]                    ; 8b 5e f2
    mov cx, word [bp-00ch]                    ; 8b 4e f4
    mov ax, 000c0h                            ; b8 c0 00
    mov dx, word [bp-006h]                    ; 8b 56 fa
    call 08cc7h                               ; e8 9b 0c
    mov es, [bp-004h]                         ; 8e 46 fc
    mov word [es:si+014h], di                 ; 26 89 7c 14
    mov word [es:si+016h], di                 ; 26 89 7c 16
    mov word [es:si+018h], di                 ; 26 89 7c 18
    mov ax, word [es:si+01ah]                 ; 26 8b 44 1a
    test ax, ax                               ; 85 c0
    je short 0806ah                           ; 74 27
    dec ax                                    ; 48
    mov es, [bp-006h]                         ; 8e 46 fa
    mov word [es:di+0010ch], ax               ; 26 89 85 0c 01
    mov word [es:di+0010eh], di               ; 26 89 bd 0e 01
    mov ax, word [es:di+00264h]               ; 26 8b 85 64 02
    mov dx, word [es:di+00266h]               ; 26 8b 95 66 02
    mov word [es:di+00100h], ax               ; 26 89 85 00 01
    mov word [es:di+00102h], dx               ; 26 89 95 02 01
    inc byte [es:di+00263h]                   ; 26 fe 85 63 02
    mov bx, 000a0h                            ; bb a0 00
    mov ax, si                                ; 89 f0
    mov dx, word [bp-004h]                    ; 8b 56 fc
    call 0795dh                               ; e8 e8 f8
    les bx, [bp-00ah]                         ; c4 5e f6
    mov ax, word [es:bx+004h]                 ; 26 8b 47 04
    mov dx, word [es:bx+006h]                 ; 26 8b 57 06
    mov es, [bp-004h]                         ; 8e 46 fc
    mov word [es:si+016h], ax                 ; 26 89 44 16
    mov word [es:si+018h], dx                 ; 26 89 54 18
    mov bx, word [es:si+016h]                 ; 26 8b 5c 16
    mov cx, dx                                ; 89 d1
    shr cx, 1                                 ; d1 e9
    rcr bx, 1                                 ; d1 db
    mov di, word [es:si+004h]                 ; 26 8b 7c 04
    mov ax, word [es:si+006h]                 ; 26 8b 44 06
    mov cx, bx                                ; 89 d9
    mov si, di                                ; 89 fe
    mov dx, ax                                ; 89 c2
    mov es, ax                                ; 8e c0
    push DS                                   ; 1e
    mov ds, dx                                ; 8e da
    rep movsw                                 ; f3 a5
    pop DS                                    ; 1f
    mov ax, word [bp-00ah]                    ; 8b 46 f6
    mov dx, word [bp-008h]                    ; 8b 56 f8
    call 07786h                               ; e8 d2 f6
    les bx, [bp-00ah]                         ; c4 5e f6
    mov ax, word [es:bx+006h]                 ; 26 8b 47 06
    or ax, word [es:bx+004h]                  ; 26 0b 47 04
    jne short 080c6h                          ; 75 05
    mov ax, strict word 00004h                ; b8 04 00
    jmp short 080c8h                          ; eb 02
    xor ax, ax                                ; 31 c0
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn 0000ch                               ; c2 0c 00
ahci_port_detect_device_:                    ; 0xf80ce LB 0x3c3
    push cx                                   ; 51
    push si                                   ; 56
    push di                                   ; 57
    enter 00220h, 000h                        ; c8 20 02 00
    mov di, ax                                ; 89 c7
    mov word [bp-00ah], dx                    ; 89 56 f6
    mov byte [bp-002h], bl                    ; 88 5e fe
    movzx cx, bl                              ; 0f b6 cb
    mov bx, cx                                ; 89 cb
    call 07c61h                               ; e8 7c fb
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 2e 95
    mov word [bp-01eh], ax                    ; 89 46 e2
    mov si, 00122h                            ; be 22 01
    mov word [bp-008h], ax                    ; 89 46 f8
    mov word [bp-00eh], si                    ; 89 76 f2
    mov word [bp-00ch], ax                    ; 89 46 f4
    sal cx, 007h                              ; c1 e1 07
    mov word [bp-012h], cx                    ; 89 4e ee
    mov ax, cx                                ; 89 c8
    add ax, 0012ch                            ; 05 2c 01
    cwd                                       ; 99
    mov word [bp-020h], ax                    ; 89 46 e0
    mov bx, dx                                ; 89 d3
    mov es, [bp-00ah]                         ; 8e 46 f6
    mov dx, word [es:di+00260h]               ; 26 8b 95 60 02
    mov cx, bx                                ; 89 d9
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov es, [bp-00ah]                         ; 8e 46 f6
    mov dx, word [es:di+00260h]               ; 26 8b 95 60 02
    add dx, strict byte 00004h                ; 83 c2 04
    mov ax, strict word 00001h                ; b8 01 00
    xor cx, cx                                ; 31 c9
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov es, [bp-00ah]                         ; 8e 46 f6
    mov dx, word [es:di+00260h]               ; 26 8b 95 60 02
    mov ax, word [bp-020h]                    ; 8b 46 e0
    mov cx, bx                                ; 89 d9
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov es, [bp-00ah]                         ; 8e 46 f6
    mov dx, word [es:di+00260h]               ; 26 8b 95 60 02
    add dx, strict byte 00004h                ; 83 c2 04
    xor ax, ax                                ; 31 c0
    xor cx, cx                                ; 31 c9
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov ax, word [bp-012h]                    ; 8b 46 ee
    add ax, 00128h                            ; 05 28 01
    cwd                                       ; 99
    mov es, [bp-00ah]                         ; 8e 46 f6
    mov bx, word [es:di+00260h]               ; 26 8b 9d 60 02
    mov cx, dx                                ; 89 d1
    mov dx, bx                                ; 89 da
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov es, [bp-00ah]                         ; 8e 46 f6
    mov dx, word [es:di+00260h]               ; 26 8b 95 60 02
    add dx, strict byte 00004h                ; 83 c2 04
    in eax, DX                                ; 66 ed
    db  08bh, 0d0h
    ; mov dx, ax                                ; 8b d0
    shr eax, 010h                             ; 66 c1 e8 10
    xchg dx, ax                               ; 92
    push strict byte 00000h                   ; 6a 00
    mov bx, strict word 0000fh                ; bb 0f 00
    xor cx, cx                                ; 31 c9
    call 07854h                               ; e8 b2 f6
    cmp ax, strict word 00003h                ; 3d 03 00
    jne near 0848ch                           ; 0f 85 e3 02
    mov es, [bp-008h]                         ; 8e 46 f8
    mov al, byte [es:si+001edh]               ; 26 8a 84 ed 01
    mov byte [bp-004h], al                    ; 88 46 fc
    cmp AL, strict byte 004h                  ; 3c 04
    jnc near 0848ch                           ; 0f 83 d2 02
    mov dx, word [bp-012h]                    ; 8b 56 ee
    add dx, 00118h                            ; 81 c2 18 01
    mov es, [bp-00ah]                         ; 8e 46 f6
    mov ax, word [es:di+00260h]               ; 26 8b 85 60 02
    mov bx, strict word 00010h                ; bb 10 00
    xor cx, cx                                ; 31 c9
    call 0779ah                               ; e8 c9 f5
    mov ax, word [bp-012h]                    ; 8b 46 ee
    add ax, 00124h                            ; 05 24 01
    cwd                                       ; 99
    mov cx, dx                                ; 89 d1
    mov es, [bp-00ah]                         ; 8e 46 f6
    mov dx, word [es:di+00260h]               ; 26 8b 95 60 02
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov es, [bp-00ah]                         ; 8e 46 f6
    mov dx, word [es:di+00260h]               ; 26 8b 95 60 02
    add dx, strict byte 00004h                ; 83 c2 04
    in eax, DX                                ; 66 ed
    db  08bh, 0d0h
    ; mov dx, ax                                ; 8b d0
    shr eax, 010h                             ; 66 c1 e8 10
    xchg dx, ax                               ; 92
    mov bx, ax                                ; 89 c3
    mov ax, dx                                ; 89 d0
    mov cl, byte [bp-004h]                    ; 8a 4e fc
    add cl, 00ch                              ; 80 c1 0c
    test dx, dx                               ; 85 d2
    jne near 083e6h                           ; 0f 85 d7 01
    cmp bx, 00101h                            ; 81 fb 01 01
    jne near 083e6h                           ; 0f 85 cf 01
    mov es, [bp-008h]                         ; 8e 46 f8
    db  066h, 026h, 0c7h, 004h, 000h, 000h, 000h, 000h
    ; mov dword [es:si], strict dword 000000000h ; 66 26 c7 04 00 00 00 00
    lea dx, [bp-00220h]                       ; 8d 96 e0 fd
    mov word [es:si+004h], dx                 ; 26 89 54 04
    mov [es:si+006h], ss                      ; 26 8c 54 06
    db  066h, 026h, 0c7h, 044h, 00ah, 001h, 000h, 000h, 002h
    ; mov dword [es:si+00ah], strict dword 002000001h ; 66 26 c7 44 0a 01 00 00 02
    mov bx, 000ech                            ; bb ec 00
    mov ax, si                                ; 89 f0
    mov dx, word [bp-01eh]                    ; 8b 56 e2
    call 0795dh                               ; e8 1b f7
    mov byte [bp-006h], cl                    ; 88 4e fa
    test byte [bp-00220h], 080h               ; f6 86 e0 fd 80
    db  00fh, 095h, 0c0h
    ; setne al                                  ; 0f 95 c0
    xor ah, ah                                ; 30 e4
    mov dx, word [bp-0021eh]                  ; 8b 96 e2 fd
    mov word [bp-018h], dx                    ; 89 56 e8
    mov dx, word [bp-0021ah]                  ; 8b 96 e6 fd
    mov word [bp-014h], dx                    ; 89 56 ec
    mov dx, word [bp-00214h]                  ; 8b 96 ec fd
    mov word [bp-01ah], dx                    ; 89 56 e6
    mov dx, word [bp-001a8h]                  ; 8b 96 58 fe
    mov word [bp-010h], dx                    ; 89 56 f0
    mov di, word [bp-001a6h]                  ; 8b be 5a fe
    cmp di, 00fffh                            ; 81 ff ff 0f
    jne short 08285h                          ; 75 10
    cmp dx, strict byte 0ffffh                ; 83 fa ff
    jne short 08285h                          ; 75 0b
    mov dx, word [bp-00158h]                  ; 8b 96 a8 fe
    mov word [bp-010h], dx                    ; 89 56 f0
    mov di, word [bp-00156h]                  ; 8b be aa fe
    movzx bx, byte [bp-004h]                  ; 0f b6 5e fc
    mov es, [bp-00ch]                         ; 8e 46 f4
    add bx, word [bp-00eh]                    ; 03 5e f2
    mov ah, byte [bp-002h]                    ; 8a 66 fe
    mov byte [es:bx+001e9h], ah               ; 26 88 a7 e9 01
    movzx dx, byte [bp-006h]                  ; 0f b6 56 fa
    imul dx, dx, strict byte 00018h           ; 6b d2 18
    mov si, word [bp-00eh]                    ; 8b 76 f2
    add si, dx                                ; 01 d6
    mov word [es:si+01eh], 0ff05h             ; 26 c7 44 1e 05 ff
    mov byte [es:si+020h], al                 ; 26 88 44 20
    mov byte [es:si+021h], 000h               ; 26 c6 44 21 00
    mov word [es:si+024h], 00200h             ; 26 c7 44 24 00 02
    mov byte [es:si+023h], 001h               ; 26 c6 44 23 01
    mov ax, word [bp-010h]                    ; 8b 46 f0
    mov word [es:si+032h], ax                 ; 26 89 44 32
    mov word [es:si+034h], di                 ; 26 89 7c 34
    mov ax, word [bp-014h]                    ; 8b 46 ec
    mov word [es:si+02ch], ax                 ; 26 89 44 2c
    mov ax, word [bp-018h]                    ; 8b 46 e8
    mov word [es:si+02eh], ax                 ; 26 89 44 2e
    mov ax, word [bp-01ah]                    ; 8b 46 e6
    mov word [es:si+030h], ax                 ; 26 89 44 30
    mov al, byte [bp-004h]                    ; 8a 46 fc
    cmp AL, strict byte 001h                  ; 3c 01
    jc short 082f0h                           ; 72 0c
    jbe short 082f8h                          ; 76 12
    cmp AL, strict byte 003h                  ; 3c 03
    je short 08300h                           ; 74 16
    cmp AL, strict byte 002h                  ; 3c 02
    je short 082fch                           ; 74 0e
    jmp short 08347h                          ; eb 57
    test al, al                               ; 84 c0
    jne short 08347h                          ; 75 53
    mov DL, strict byte 040h                  ; b2 40
    jmp short 08302h                          ; eb 0a
    mov DL, strict byte 048h                  ; b2 48
    jmp short 08302h                          ; eb 06
    mov DL, strict byte 050h                  ; b2 50
    jmp short 08302h                          ; eb 02
    mov DL, strict byte 058h                  ; b2 58
    mov al, dl                                ; 88 d0
    add AL, strict byte 007h                  ; 04 07
    movzx bx, al                              ; 0f b6 d8
    mov ax, bx                                ; 89 d8
    call 0165ch                               ; e8 4e 93
    test al, al                               ; 84 c0
    je short 08347h                           ; 74 35
    mov al, dl                                ; 88 d0
    db  0feh, 0c0h
    ; inc al                                    ; fe c0
    xor ah, ah                                ; 30 e4
    call 0165ch                               ; e8 41 93
    xor ah, ah                                ; 30 e4
    mov cx, ax                                ; 89 c1
    sal cx, 008h                              ; c1 e1 08
    movzx ax, dl                              ; 0f b6 c2
    call 0165ch                               ; e8 34 93
    xor ah, ah                                ; 30 e4
    add ax, cx                                ; 01 c8
    mov word [bp-01ch], ax                    ; 89 46 e4
    mov al, dl                                ; 88 d0
    add AL, strict byte 002h                  ; 04 02
    xor ah, ah                                ; 30 e4
    call 0165ch                               ; e8 24 93
    xor ah, ah                                ; 30 e4
    mov word [bp-016h], ax                    ; 89 46 ea
    mov ax, bx                                ; 89 d8
    call 0165ch                               ; e8 1a 93
    movzx dx, al                              ; 0f b6 d0
    jmp short 08356h                          ; eb 0f
    mov ax, word [bp-018h]                    ; 8b 46 e8
    mov word [bp-01ch], ax                    ; 89 46 e4
    mov ax, word [bp-014h]                    ; 8b 46 ec
    mov word [bp-016h], ax                    ; 89 46 ea
    mov dx, word [bp-01ah]                    ; 8b 56 e6
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 4d 95
    push di                                   ; 57
    push word [bp-010h]                       ; ff 76 f0
    push dx                                   ; 52
    push word [bp-016h]                       ; ff 76 ea
    push word [bp-01ch]                       ; ff 76 e4
    push word [bp-01ah]                       ; ff 76 e6
    push word [bp-014h]                       ; ff 76 ec
    push word [bp-018h]                       ; ff 76 e8
    movzx ax, byte [bp-002h]                  ; 0f b6 46 fe
    push ax                                   ; 50
    movzx ax, byte [bp-004h]                  ; 0f b6 46 fc
    push ax                                   ; 50
    push 00c10h                               ; 68 10 0c
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 62 95
    add sp, strict byte 00018h                ; 83 c4 18
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    imul ax, ax, strict byte 00018h           ; 6b c0 18
    les si, [bp-00eh]                         ; c4 76 f2
    add si, ax                                ; 01 c6
    mov ax, word [bp-016h]                    ; 8b 46 ea
    mov word [es:si+026h], ax                 ; 26 89 44 26
    mov ax, word [bp-01ch]                    ; 8b 46 e4
    mov word [es:si+028h], ax                 ; 26 89 44 28
    mov word [es:si+02ah], dx                 ; 26 89 54 2a
    mov bx, word [bp-00eh]                    ; 8b 5e f2
    mov dl, byte [es:bx+0019eh]               ; 26 8a 97 9e 01
    mov al, byte [bp-004h]                    ; 8a 46 fc
    add AL, strict byte 00ch                  ; 04 0c
    movzx bx, dl                              ; 0f b6 da
    add bx, word [bp-00eh]                    ; 03 5e f2
    mov byte [es:bx+0019fh], al               ; 26 88 87 9f 01
    db  0feh, 0c2h
    ; inc dl                                    ; fe c2
    mov bx, word [bp-00eh]                    ; 8b 5e f2
    mov byte [es:bx+0019eh], dl               ; 26 88 97 9e 01
    mov dx, strict word 00075h                ; ba 75 00
    mov ax, strict word 00040h                ; b8 40 00
    call 01600h                               ; e8 2d 92
    mov dl, al                                ; 88 c2
    db  0feh, 0c2h
    ; inc dl                                    ; fe c2
    movzx bx, dl                              ; 0f b6 da
    mov dx, strict word 00075h                ; ba 75 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0160eh                               ; e8 2b 92
    jmp near 0847eh                           ; e9 98 00
    cmp dx, 0eb14h                            ; 81 fa 14 eb
    jne near 0847eh                           ; 0f 85 90 00
    cmp bx, 00101h                            ; 81 fb 01 01
    jne near 0847eh                           ; 0f 85 88 00
    mov es, [bp-008h]                         ; 8e 46 f8
    db  066h, 026h, 0c7h, 004h, 000h, 000h, 000h, 000h
    ; mov dword [es:si], strict dword 000000000h ; 66 26 c7 04 00 00 00 00
    lea dx, [bp-00220h]                       ; 8d 96 e0 fd
    mov word [es:si+004h], dx                 ; 26 89 54 04
    mov [es:si+006h], ss                      ; 26 8c 54 06
    db  066h, 026h, 0c7h, 044h, 00ah, 001h, 000h, 000h, 002h
    ; mov dword [es:si+00ah], strict dword 002000001h ; 66 26 c7 44 0a 01 00 00 02
    mov bx, 000a1h                            ; bb a1 00
    mov ax, si                                ; 89 f0
    mov dx, word [bp-01eh]                    ; 8b 56 e2
    call 0795dh                               ; e8 3c f5
    test byte [bp-00220h], 080h               ; f6 86 e0 fd 80
    db  00fh, 095h, 0c0h
    ; setne al                                  ; 0f 95 c0
    xor ah, ah                                ; 30 e4
    mov dx, ax                                ; 89 c2
    movzx bx, byte [bp-004h]                  ; 0f b6 5e fc
    mov es, [bp-01eh]                         ; 8e 46 e2
    add bx, si                                ; 01 f3
    mov al, byte [bp-002h]                    ; 8a 46 fe
    mov byte [es:bx+001e9h], al               ; 26 88 87 e9 01
    movzx si, cl                              ; 0f b6 f1
    imul si, si, strict byte 00018h           ; 6b f6 18
    add si, 00122h                            ; 81 c6 22 01
    mov word [es:si+01eh], 00505h             ; 26 c7 44 1e 05 05
    mov byte [es:si+020h], dl                 ; 26 88 54 20
    mov word [es:si+024h], 00800h             ; 26 c7 44 24 00 08
    les bx, [bp-00eh]                         ; c4 5e f2
    mov dl, byte [es:bx+001afh]               ; 26 8a 97 af 01
    mov al, byte [bp-004h]                    ; 8a 46 fc
    add AL, strict byte 00ch                  ; 04 0c
    movzx bx, dl                              ; 0f b6 da
    mov es, [bp-01eh]                         ; 8e 46 e2
    add bx, 00122h                            ; 81 c3 22 01
    mov byte [es:bx+001b0h], al               ; 26 88 87 b0 01
    db  0feh, 0c2h
    ; inc dl                                    ; fe c2
    les bx, [bp-00eh]                         ; c4 5e f2
    mov byte [es:bx+001afh], dl               ; 26 88 97 af 01
    inc byte [bp-004h]                        ; fe 46 fc
    mov al, byte [bp-004h]                    ; 8a 46 fc
    les bx, [bp-00eh]                         ; c4 5e f2
    mov byte [es:bx+001edh], al               ; 26 88 87 ed 01
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop cx                                    ; 59
    retn                                      ; c3
ahci_mem_alloc_:                             ; 0xf8491 LB 0x40
    push bx                                   ; 53
    push cx                                   ; 51
    push dx                                   ; 52
    push si                                   ; 56
    push di                                   ; 57
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov dx, 00413h                            ; ba 13 04
    xor ax, ax                                ; 31 c0
    call 0161ch                               ; e8 7b 91
    test ax, ax                               ; 85 c0
    je short 084cah                           ; 74 25
    dec ax                                    ; 48
    mov bx, ax                                ; 89 c3
    xor dx, dx                                ; 31 d2
    mov cx, strict word 0000ah                ; b9 0a 00
    sal ax, 1                                 ; d1 e0
    rcl dx, 1                                 ; d1 d2
    loop 084adh                               ; e2 fa
    mov si, ax                                ; 89 c6
    mov di, dx                                ; 89 d7
    mov cx, strict word 00004h                ; b9 04 00
    shr di, 1                                 ; d1 ef
    rcr si, 1                                 ; d1 de
    loop 084bah                               ; e2 fa
    mov dx, 00413h                            ; ba 13 04
    xor ax, ax                                ; 31 c0
    call 0162ah                               ; e8 62 91
    mov ax, si                                ; 89 f0
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop dx                                    ; 5a
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
ahci_hba_init_:                              ; 0xf84d1 LB 0x120
    push bx                                   ; 53
    push cx                                   ; 51
    push dx                                   ; 52
    push si                                   ; 56
    push di                                   ; 57
    enter 00006h, 000h                        ; c8 06 00 00
    mov si, ax                                ; 89 c6
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, strict word 00040h                ; b8 40 00
    call 0161ch                               ; e8 37 91
    mov bx, 00122h                            ; bb 22 01
    mov word [bp-006h], ax                    ; 89 46 fa
    mov ax, strict word 00010h                ; b8 10 00
    xor cx, cx                                ; 31 c9
    mov dx, si                                ; 89 f2
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    lea dx, [si+004h]                         ; 8d 54 04
    in eax, DX                                ; 66 ed
    db  08bh, 0d0h
    ; mov dx, ax                                ; 8b d0
    shr eax, 010h                             ; 66 c1 e8 10
    xchg dx, ax                               ; 92
    call 08491h                               ; e8 87 ff
    mov di, ax                                ; 89 c7
    test ax, ax                               ; 85 c0
    je near 085d3h                            ; 0f 84 c1 00
    mov es, [bp-006h]                         ; 8e 46 fa
    mov word [es:bx+001eeh], di               ; 26 89 bf ee 01
    mov byte [es:bx+001edh], 000h             ; 26 c6 87 ed 01 00
    xor bx, bx                                ; 31 db
    mov es, di                                ; 8e c7
    mov byte [es:bx+00262h], 0ffh             ; 26 c6 87 62 02 ff
    mov word [es:bx+00260h], si               ; 26 89 b7 60 02
    db  066h, 026h, 0c7h, 087h, 064h, 002h, 000h, 0c0h, 00ch, 000h
    ; mov dword [es:bx+00264h], strict dword 0000cc000h ; 66 26 c7 87 64 02 00 c0 0c 00
    mov bx, strict word 00001h                ; bb 01 00
    xor cx, cx                                ; 31 c9
    mov dx, strict word 00004h                ; ba 04 00
    mov ax, si                                ; 89 f0
    call 0779ah                               ; e8 54 f2
    mov ax, strict word 00004h                ; b8 04 00
    xor cx, cx                                ; 31 c9
    mov dx, si                                ; 89 f2
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    lea bx, [si+004h]                         ; 8d 5c 04
    mov dx, bx                                ; 89 da
    in eax, DX                                ; 66 ed
    db  08bh, 0d0h
    ; mov dx, ax                                ; 8b d0
    shr eax, 010h                             ; 66 c1 e8 10
    xchg dx, ax                               ; 92
    test AL, strict byte 001h                 ; a8 01
    jne short 08546h                          ; 75 de
    xor ax, ax                                ; 31 c0
    xor cx, cx                                ; 31 c9
    mov dx, si                                ; 89 f2
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    mov dx, bx                                ; 89 da
    in eax, DX                                ; 66 ed
    db  08bh, 0d0h
    ; mov dx, ax                                ; 8b d0
    shr eax, 010h                             ; 66 c1 e8 10
    xchg dx, ax                               ; 92
    push strict byte 00000h                   ; 6a 00
    mov bx, strict word 0001fh                ; bb 1f 00
    xor cx, cx                                ; 31 c9
    call 07854h                               ; e8 c8 f2
    db  0feh, 0c0h
    ; inc al                                    ; fe c0
    mov byte [bp-004h], al                    ; 88 46 fc
    mov byte [bp-002h], 000h                  ; c6 46 fe 00
    jmp short 085a0h                          ; eb 09
    inc byte [bp-002h]                        ; fe 46 fe
    cmp byte [bp-002h], 020h                  ; 80 7e fe 20
    jnc short 085d1h                          ; 73 31
    movzx cx, byte [bp-002h]                  ; 0f b6 4e fe
    mov ax, strict word 00001h                ; b8 01 00
    xor dx, dx                                ; 31 d2
    jcxz 085b1h                               ; e3 06
    sal ax, 1                                 ; d1 e0
    rcl dx, 1                                 ; d1 d2
    loop 085abh                               ; e2 fa
    mov bx, ax                                ; 89 c3
    mov cx, dx                                ; 89 d1
    mov dx, strict word 0000ch                ; ba 0c 00
    mov ax, si                                ; 89 f0
    call 0781eh                               ; e8 61 f2
    test al, al                               ; 84 c0
    je short 08597h                           ; 74 d6
    movzx bx, byte [bp-002h]                  ; 0f b6 5e fe
    xor ax, ax                                ; 31 c0
    mov dx, di                                ; 89 fa
    call 080ceh                               ; e8 02 fb
    dec byte [bp-004h]                        ; fe 4e fc
    jne short 08597h                          ; 75 c6
    xor ax, ax                                ; 31 c0
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop dx                                    ; 5a
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
    or ax, word [di]                          ; 0b 05
    add AL, strict byte 003h                  ; 04 03
    add al, byte [bx+di]                      ; 02 01
    add byte [bp-0637ah], bh                  ; 00 be 86 9c
    xchg byte [bp+si-0577ah], ah              ; 86 a2 86 a8
    xchg byte [bp-04b7ah], ch                 ; 86 ae 86 b4
    xchg byte [bp+si-0417ah], bh              ; 86 ba 86 be
    db  086h
_ahci_init:                                  ; 0xf85f1 LB 0xf9
    push si                                   ; 56
    push di                                   ; 57
    enter 00006h, 000h                        ; c8 06 00 00
    mov ax, 00601h                            ; b8 01 06
    mov dx, strict word 00001h                ; ba 01 00
    call 08ae3h                               ; e8 e3 04
    mov dx, ax                                ; 89 c2
    cmp ax, strict word 0ffffh                ; 3d ff ff
    je near 086e6h                            ; 0f 84 dd 00
    xor al, al                                ; 30 c0
    shr ax, 008h                              ; c1 e8 08
    mov byte [bp-006h], al                    ; 88 46 fa
    mov byte [bp-004h], dl                    ; 88 56 fc
    xor dh, dh                                ; 30 f6
    xor ah, ah                                ; 30 e4
    mov bx, strict word 00034h                ; bb 34 00
    call 08b0bh                               ; e8 ed 04
    mov cl, al                                ; 88 c1
    test cl, cl                               ; 84 c9
    je short 08647h                           ; 74 23
    movzx bx, cl                              ; 0f b6 d9
    movzx di, byte [bp-004h]                  ; 0f b6 7e fc
    movzx si, byte [bp-006h]                  ; 0f b6 76 fa
    mov dx, di                                ; 89 fa
    mov ax, si                                ; 89 f0
    call 08b0bh                               ; e8 d5 04
    cmp AL, strict byte 012h                  ; 3c 12
    je short 08647h                           ; 74 0d
    mov al, cl                                ; 88 c8
    db  0feh, 0c0h
    ; inc al                                    ; fe c0
    movzx bx, al                              ; 0f b6 d8
    mov dx, di                                ; 89 fa
    mov ax, si                                ; 89 f0
    jmp short 0861bh                          ; eb d4
    test cl, cl                               ; 84 c9
    je near 086e6h                            ; 0f 84 99 00
    add cl, 002h                              ; 80 c1 02
    movzx bx, cl                              ; 0f b6 d9
    movzx di, byte [bp-004h]                  ; 0f b6 7e fc
    movzx si, byte [bp-006h]                  ; 0f b6 76 fa
    mov dx, di                                ; 89 fa
    mov ax, si                                ; 89 f0
    call 08b0bh                               ; e8 a9 04
    cmp AL, strict byte 010h                  ; 3c 10
    jne near 086e6h                           ; 0f 85 7e 00
    mov byte [bp-002h], 000h                  ; c6 46 fe 00
    mov al, cl                                ; 88 c8
    add AL, strict byte 002h                  ; 04 02
    movzx bx, al                              ; 0f b6 d8
    mov dx, di                                ; 89 fa
    mov ax, si                                ; 89 f0
    call 08b2ch                               ; e8 b2 04
    mov dx, ax                                ; 89 c2
    and ax, strict word 0000fh                ; 25 0f 00
    sub ax, strict word 00004h                ; 2d 04 00
    cmp ax, strict word 0000bh                ; 3d 0b 00
    jnbe short 086beh                         ; 77 37
    push CS                                   ; 0e
    pop ES                                    ; 07
    mov cx, strict word 00008h                ; b9 08 00
    mov di, 085dah                            ; bf da 85
    repne scasb                               ; f2 ae
    sal cx, 1                                 ; d1 e1
    mov di, cx                                ; 89 cf
    mov ax, word [cs:di-07a1fh]               ; 2e 8b 85 e1 85
    jmp ax                                    ; ff e0
    mov byte [bp-002h], 010h                  ; c6 46 fe 10
    jmp short 086beh                          ; eb 1c
    mov byte [bp-002h], 014h                  ; c6 46 fe 14
    jmp short 086beh                          ; eb 16
    mov byte [bp-002h], 018h                  ; c6 46 fe 18
    jmp short 086beh                          ; eb 10
    mov byte [bp-002h], 01ch                  ; c6 46 fe 1c
    jmp short 086beh                          ; eb 0a
    mov byte [bp-002h], 020h                  ; c6 46 fe 20
    jmp short 086beh                          ; eb 04
    mov byte [bp-002h], 024h                  ; c6 46 fe 24
    mov si, dx                                ; 89 d6
    shr si, 004h                              ; c1 ee 04
    sal si, 002h                              ; c1 e6 02
    mov al, byte [bp-002h]                    ; 8a 46 fe
    test al, al                               ; 84 c0
    je short 086e6h                           ; 74 19
    movzx bx, al                              ; 0f b6 d8
    movzx dx, byte [bp-004h]                  ; 0f b6 56 fc
    movzx ax, byte [bp-006h]                  ; 0f b6 46 fa
    call 08b4bh                               ; e8 70 04
    test AL, strict byte 001h                 ; a8 01
    je short 086e6h                           ; 74 07
    and AL, strict byte 0f0h                  ; 24 f0
    add ax, si                                ; 01 f0
    call 084d1h                               ; e8 eb fd
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn                                      ; c3
apm_out_str_:                                ; 0xf86ea LB 0x36
    push bx                                   ; 53
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bx, ax                                ; 89 c3
    cmp byte [bx], 000h                       ; 80 3f 00
    je short 086ffh                           ; 74 0a
    mov al, byte [bx]                         ; 8a 07
    out DX, AL                                ; ee
    inc bx                                    ; 43
    mov al, byte [bx]                         ; 8a 07
    db  00ah, 0c0h
    ; or al, al                                 ; 0a c0
    jne short 086f7h                          ; 75 f8
    pop bp                                    ; 5d
    pop bx                                    ; 5b
    retn                                      ; c3
    inc bx                                    ; 43
    xchg word [bp+si], dx                     ; 87 12
    mov byte [di-079h], dl                    ; 88 55 87
    jo short 08691h                           ; 70 87
    adc cl, byte [bx+si-07865h]               ; 12 88 9b 87
    adc cl, byte [bx+si-07860h]               ; 12 88 a0 87
    out 087h, ax                              ; e7 87
    out 087h, ax                              ; e7 87
    out 087h, ax                              ; e7 87
    loop 086a1h                               ; e2 87
    out 087h, ax                              ; e7 87
    out 087h, ax                              ; e7 87
    db  0dah
    db  087h
_apm_function:                               ; 0xf8720 LB 0xf5
    push si                                   ; 56
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    and byte [bp+01ah], 0feh                  ; 80 66 1a fe
    mov ax, word [bp+014h]                    ; 8b 46 14
    xor ah, ah                                ; 30 e4
    cmp ax, strict word 0000eh                ; 3d 0e 00
    jnbe near 087e7h                          ; 0f 87 b3 00
    mov bx, ax                                ; 89 c3
    add bx, ax                                ; 01 c3
    mov dx, word [bp+01ah]                    ; 8b 56 1a
    or dl, 001h                               ; 80 ca 01
    jmp word [cs:bx-078feh]                   ; 2e ff a7 02 87
    mov word [bp+014h], 00102h                ; c7 46 14 02 01
    mov word [bp+00eh], 0504dh                ; c7 46 0e 4d 50
    mov word [bp+012h], strict word 00003h    ; c7 46 12 03 00
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn                                      ; c3
    mov word [bp+014h], 0f000h                ; c7 46 14 00 f0
    mov word [bp+00eh], 08d34h                ; c7 46 0e 34 8d
    mov word [bp+012h], 0f000h                ; c7 46 12 00 f0
    mov ax, strict word 0fff0h                ; b8 f0 ff
    mov word [bp+008h], ax                    ; 89 46 08
    mov word [bp+006h], ax                    ; 89 46 06
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn                                      ; c3
    mov word [bp+014h], 0f000h                ; c7 46 14 00 f0
    mov word [bp+00eh], 0da40h                ; c7 46 0e 40 da
    mov ax, 0f000h                            ; b8 00 f0
    mov word [bp+012h], ax                    ; 89 46 12
    mov word [bp+010h], ax                    ; 89 46 10
    mov ax, strict word 0fff0h                ; b8 f0 ff
    mov word [bp+008h], ax                    ; 89 46 08
    mov word [bp+006h], ax                    ; 89 46 06
    xor bx, bx                                ; 31 db
    sal ebx, 010h                             ; 66 c1 e3 10
    mov si, ax                                ; 89 c6
    sal esi, 010h                             ; 66 c1 e6 10
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn                                      ; c3
    sti                                       ; fb
    hlt                                       ; f4
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn                                      ; c3
    cmp word [bp+012h], strict byte 00003h    ; 83 7e 12 03
    je short 087c6h                           ; 74 20
    cmp word [bp+012h], strict byte 00002h    ; 83 7e 12 02
    je short 087beh                           ; 74 12
    cmp word [bp+012h], strict byte 00001h    ; 83 7e 12 01
    jne short 087ceh                          ; 75 1c
    mov dx, 08900h                            ; ba 00 89
    mov ax, 00c48h                            ; b8 48 0c
    call 086eah                               ; e8 2f ff
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn                                      ; c3
    mov dx, 08900h                            ; ba 00 89
    mov ax, 00c50h                            ; b8 50 0c
    jmp short 087b8h                          ; eb f2
    mov dx, 08900h                            ; ba 00 89
    mov ax, 00c58h                            ; b8 58 0c
    jmp short 087b8h                          ; eb ea
    or ah, 00ah                               ; 80 cc 0a
    mov word [bp+014h], ax                    ; 89 46 14
    mov word [bp+01ah], dx                    ; 89 56 1a
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn                                      ; c3
    mov word [bp+014h], 00102h                ; c7 46 14 02 01
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn                                      ; c3
    or ah, 080h                               ; 80 cc 80
    jmp short 087d1h                          ; eb ea
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 bc 90
    push word [bp+00eh]                       ; ff 76 0e
    push word [bp+014h]                       ; ff 76 14
    push 00c61h                               ; 68 61 0c
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 e9 90
    add sp, strict byte 00008h                ; 83 c4 08
    mov ax, word [bp+014h]                    ; 8b 46 14
    xor ah, ah                                ; 30 e4
    or ah, 00ch                               ; 80 cc 0c
    mov word [bp+014h], ax                    ; 89 46 14
    or byte [bp+01ah], 001h                   ; 80 4e 1a 01
    pop bp                                    ; 5d
    pop si                                    ; 5e
    retn                                      ; c3
pci16_select_reg_:                           ; 0xf8815 LB 0x21
    push bx                                   ; 53
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    and dl, 0fch                              ; 80 e2 fc
    mov bx, dx                                ; 89 d3
    mov dx, 00cf8h                            ; ba f8 0c
    movzx eax, ax                             ; 66 0f b7 c0
    sal eax, 008h                             ; 66 c1 e0 08
    or eax, strict dword 080000000h           ; 66 0d 00 00 00 80
    db  08ah, 0c3h
    ; mov al, bl                                ; 8a c3
    out DX, eax                               ; 66 ef
    pop bp                                    ; 5d
    pop bx                                    ; 5b
    retn                                      ; c3
pci16_find_device_:                          ; 0xf8836 LB 0xf2
    push si                                   ; 56
    push di                                   ; 57
    enter 0000ch, 000h                        ; c8 0c 00 00
    push ax                                   ; 50
    push dx                                   ; 52
    mov si, bx                                ; 89 de
    mov di, cx                                ; 89 cf
    test cx, cx                               ; 85 c9
    xor bx, bx                                ; 31 db
    mov byte [bp-004h], 000h                  ; c6 46 fc 00
    test bl, 007h                             ; f6 c3 07
    jne short 0887ch                          ; 75 2d
    mov dx, strict word 0000eh                ; ba 0e 00
    mov ax, bx                                ; 89 d8
    call 08815h                               ; e8 be ff
    mov dx, 00cfeh                            ; ba fe 0c
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    mov byte [bp-002h], al                    ; 88 46 fe
    cmp AL, strict byte 0ffh                  ; 3c ff
    jne short 0886ah                          ; 75 06
    add bx, strict byte 00008h                ; 83 c3 08
    jmp near 088fdh                           ; e9 93 00
    test byte [bp-002h], 080h                 ; f6 46 fe 80
    je short 08877h                           ; 74 07
    mov word [bp-006h], strict word 00001h    ; c7 46 fa 01 00
    jmp short 0887ch                          ; eb 05
    mov word [bp-006h], strict word 00008h    ; c7 46 fa 08 00
    mov al, byte [bp-002h]                    ; 8a 46 fe
    and AL, strict byte 007h                  ; 24 07
    cmp AL, strict byte 001h                  ; 3c 01
    jne short 088a4h                          ; 75 1f
    mov ax, bx                                ; 89 d8
    shr ax, 008h                              ; c1 e8 08
    test ax, ax                               ; 85 c0
    jne short 088a4h                          ; 75 16
    mov dx, strict word 0001ah                ; ba 1a 00
    mov ax, bx                                ; 89 d8
    call 08815h                               ; e8 7f ff
    mov dx, 00cfeh                            ; ba fe 0c
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    cmp al, byte [bp-004h]                    ; 3a 46 fc
    jbe short 088a4h                          ; 76 03
    mov byte [bp-004h], al                    ; 88 46 fc
    test di, di                               ; 85 ff
    je short 088adh                           ; 74 05
    mov dx, strict word 00008h                ; ba 08 00
    jmp short 088afh                          ; eb 02
    xor dx, dx                                ; 31 d2
    mov ax, bx                                ; 89 d8
    call 08815h                               ; e8 61 ff
    mov dx, 00cfch                            ; ba fc 0c
    in eax, DX                                ; 66 ed
    db  08bh, 0d0h
    ; mov dx, ax                                ; 8b d0
    shr eax, 010h                             ; 66 c1 e8 10
    xchg dx, ax                               ; 92
    mov word [bp-00ah], ax                    ; 89 46 f6
    mov word [bp-008h], dx                    ; 89 56 f8
    mov word [bp-00ch], strict word 00000h    ; c7 46 f4 00 00
    test di, di                               ; 85 ff
    je short 088deh                           ; 74 0f
    mov cx, strict word 00008h                ; b9 08 00
    shr dx, 1                                 ; d1 ea
    rcr ax, 1                                 ; d1 d8
    loop 088d2h                               ; e2 fa
    mov word [bp-00ah], ax                    ; 89 46 f6
    mov word [bp-008h], dx                    ; 89 56 f8
    mov ax, word [bp-008h]                    ; 8b 46 f8
    cmp ax, word [bp-010h]                    ; 3b 46 f0
    jne short 088eeh                          ; 75 08
    mov ax, word [bp-00ah]                    ; 8b 46 f6
    cmp ax, word [bp-00eh]                    ; 3b 46 f2
    je short 088f4h                           ; 74 06
    cmp word [bp-00ch], strict byte 00000h    ; 83 7e f4 00
    je short 088fah                           ; 74 06
    dec si                                    ; 4e
    cmp si, strict byte 0ffffh                ; 83 fe ff
    je short 0890ch                           ; 74 12
    add bx, word [bp-006h]                    ; 03 5e fa
    mov dx, bx                                ; 89 da
    shr dx, 008h                              ; c1 ea 08
    movzx ax, byte [bp-004h]                  ; 0f b6 46 fc
    cmp dx, ax                                ; 39 c2
    jbe near 0884ah                           ; 0f 86 3e ff
    cmp si, strict byte 0ffffh                ; 83 fe ff
    jne short 08915h                          ; 75 04
    mov ax, bx                                ; 89 d8
    jmp short 08918h                          ; eb 03
    mov ax, strict word 0ffffh                ; b8 ff ff
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn                                      ; c3
    or byte [bp+si-075deh], cl                ; 08 8a 22 8a
    xor ax, 04a8ah                            ; 35 8a 4a
    mov bl, byte [di-076h]                    ; 8a 5d 8a
    outsw                                     ; 6f
    db  08ah
_pci16_function:                             ; 0xf8928 LB 0x1bb
    push si                                   ; 56
    push di                                   ; 57
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    and word [bp+024h], 000ffh                ; 81 66 24 ff 00
    and word [bp+030h], strict byte 0fffeh    ; 83 66 30 fe
    mov bx, word [bp+024h]                    ; 8b 5e 24
    xor bh, bh                                ; 30 ff
    mov ax, word [bp+024h]                    ; 8b 46 24
    xor ah, ah                                ; 30 e4
    cmp bx, strict byte 00003h                ; 83 fb 03
    jc short 0895fh                           ; 72 1a
    jbe short 089b7h                          ; 76 70
    cmp bx, strict byte 0000eh                ; 83 fb 0e
    je near 08a83h                            ; 0f 84 35 01
    cmp bx, strict byte 00008h                ; 83 fb 08
    jc near 08ab2h                            ; 0f 82 5d 01
    cmp bx, strict byte 0000dh                ; 83 fb 0d
    jbe near 089dch                           ; 0f 86 80 00
    jmp near 08ab2h                           ; e9 53 01
    cmp bx, strict byte 00002h                ; 83 fb 02
    je short 08987h                           ; 74 23
    cmp bx, strict byte 00001h                ; 83 fb 01
    jne near 08ab2h                           ; 0f 85 47 01
    mov word [bp+024h], strict word 00001h    ; c7 46 24 01 00
    mov word [bp+018h], 00210h                ; c7 46 18 10 02
    mov word [bp+020h], strict word 00000h    ; c7 46 20 00 00
    mov word [bp+01ch], 04350h                ; c7 46 1c 50 43
    mov word [bp+01eh], 02049h                ; c7 46 1e 49 20
    jmp near 08adfh                           ; e9 58 01
    cmp word [bp+01ch], strict byte 0ffffh    ; 83 7e 1c ff
    jne short 08993h                          ; 75 06
    or ah, 083h                               ; 80 cc 83
    jmp near 08ad8h                           ; e9 45 01
    mov bx, word [bp+00ch]                    ; 8b 5e 0c
    mov dx, word [bp+020h]                    ; 8b 56 20
    mov ax, word [bp+01ch]                    ; 8b 46 1c
    xor cx, cx                                ; 31 c9
    call 08836h                               ; e8 95 fe
    cmp ax, strict word 0ffffh                ; 3d ff ff
    jne short 089b1h                          ; 75 0b
    mov ax, word [bp+024h]                    ; 8b 46 24
    xor ah, ah                                ; 30 e4
    or ah, 086h                               ; 80 cc 86
    jmp near 08ad8h                           ; e9 27 01
    mov word [bp+018h], ax                    ; 89 46 18
    jmp near 08adfh                           ; e9 28 01
    mov bx, word [bp+00ch]                    ; 8b 5e 0c
    mov ax, word [bp+020h]                    ; 8b 46 20
    mov dx, word [bp+022h]                    ; 8b 56 22
    mov cx, strict word 00001h                ; b9 01 00
    call 08836h                               ; e8 70 fe
    cmp ax, strict word 0ffffh                ; 3d ff ff
    jne short 089d6h                          ; 75 0b
    mov ax, word [bp+024h]                    ; 8b 46 24
    xor ah, ah                                ; 30 e4
    or ah, 086h                               ; 80 cc 86
    jmp near 08ad8h                           ; e9 02 01
    mov word [bp+018h], ax                    ; 89 46 18
    jmp near 08adfh                           ; e9 03 01
    cmp word [bp+008h], 00100h                ; 81 7e 08 00 01
    jc short 089e9h                           ; 72 06
    or ah, 087h                               ; 80 cc 87
    jmp near 08ad8h                           ; e9 ef 00
    mov dx, word [bp+008h]                    ; 8b 56 08
    mov ax, word [bp+018h]                    ; 8b 46 18
    call 08815h                               ; e8 23 fe
    mov bx, word [bp+024h]                    ; 8b 5e 24
    xor bh, bh                                ; 30 ff
    sub bx, strict byte 00008h                ; 83 eb 08
    cmp bx, strict byte 00005h                ; 83 fb 05
    jnbe near 08adfh                          ; 0f 87 de 00
    add bx, bx                                ; 01 db
    jmp word [cs:bx-076e4h]                   ; 2e ff a7 1c 89
    mov bx, word [bp+020h]                    ; 8b 5e 20
    xor bl, bl                                ; 30 db
    mov dx, word [bp+008h]                    ; 8b 56 08
    and dx, strict byte 00003h                ; 83 e2 03
    add dx, 00cfch                            ; 81 c2 fc 0c
    in AL, DX                                 ; ec
    db  02ah, 0e4h
    ; sub ah, ah                                ; 2a e4
    or bx, ax                                 ; 09 c3
    mov word [bp+020h], bx                    ; 89 5e 20
    jmp near 08adfh                           ; e9 bd 00
    mov dx, word [bp+008h]                    ; 8b 56 08
    xor dh, dh                                ; 30 f6
    and dl, 002h                              ; 80 e2 02
    add dx, 00cfch                            ; 81 c2 fc 0c
    in ax, DX                                 ; ed
    mov word [bp+020h], ax                    ; 89 46 20
    jmp near 08adfh                           ; e9 aa 00
    mov dx, 00cfch                            ; ba fc 0c
    in eax, DX                                ; 66 ed
    db  08bh, 0d0h
    ; mov dx, ax                                ; 8b d0
    shr eax, 010h                             ; 66 c1 e8 10
    xchg dx, ax                               ; 92
    mov word [bp+020h], ax                    ; 89 46 20
    mov word [bp+022h], dx                    ; 89 56 22
    jmp near 08adfh                           ; e9 95 00
    mov ax, word [bp+020h]                    ; 8b 46 20
    mov dx, word [bp+008h]                    ; 8b 56 08
    xor dh, dh                                ; 30 f6
    and dl, 003h                              ; 80 e2 03
    add dx, 00cfch                            ; 81 c2 fc 0c
    out DX, AL                                ; ee
    jmp near 08adfh                           ; e9 82 00
    mov ax, word [bp+020h]                    ; 8b 46 20
    mov dx, word [bp+008h]                    ; 8b 56 08
    xor dh, dh                                ; 30 f6
    and dl, 002h                              ; 80 e2 02
    add dx, 00cfch                            ; 81 c2 fc 0c
    out DX, ax                                ; ef
    jmp short 08adfh                          ; eb 70
    mov ax, word [bp+020h]                    ; 8b 46 20
    mov cx, word [bp+022h]                    ; 8b 4e 22
    mov dx, 00cfch                            ; ba fc 0c
    xchg cx, ax                               ; 91
    sal eax, 010h                             ; 66 c1 e0 10
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    out DX, eax                               ; 66 ef
    jmp short 08adfh                          ; eb 5c
    mov bx, word [bp+008h]                    ; 8b 5e 08
    mov es, [bp+028h]                         ; 8e 46 28
    mov cx, word [word 00000h]                ; 8b 0e 00 00
    cmp cx, word [es:bx]                      ; 26 3b 0f
    jbe short 08a9ch                          ; 76 0a
    mov ax, word [bp+024h]                    ; 8b 46 24
    xor ah, ah                                ; 30 e4
    or ah, 089h                               ; 80 cc 89
    jmp short 08ad8h                          ; eb 3c
    les di, [es:bx+002h]                      ; 26 c4 7f 02
    mov si, 0f2c0h                            ; be c0 f2
    mov dx, ds                                ; 8c da
    push DS                                   ; 1e
    mov ds, dx                                ; 8e da
    rep movsb                                 ; f3 a4
    pop DS                                    ; 1f
    mov word [bp+018h], 00a00h                ; c7 46 18 00 0a
    jmp short 08adfh                          ; eb 2d
    mov bx, 00cd8h                            ; bb d8 0c
    mov cx, ds                                ; 8c d9
    mov ax, strict word 00004h                ; b8 04 00
    call 018aeh                               ; e8 f1 8d
    mov ax, word [bp+018h]                    ; 8b 46 18
    push ax                                   ; 50
    mov ax, word [bp+024h]                    ; 8b 46 24
    push ax                                   ; 50
    push 00c94h                               ; 68 94 0c
    push strict byte 00004h                   ; 6a 04
    call 018e9h                               ; e8 1c 8e
    add sp, strict byte 00008h                ; 83 c4 08
    mov ax, word [bp+024h]                    ; 8b 46 24
    xor ah, ah                                ; 30 e4
    or ah, 081h                               ; 80 cc 81
    mov word [bp+024h], ax                    ; 89 46 24
    or word [bp+030h], strict byte 00001h     ; 83 4e 30 01
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn                                      ; c3
pci_find_classcode_:                         ; 0xf8ae3 LB 0x28
    push bx                                   ; 53
    push cx                                   ; 51
    push si                                   ; 56
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov cx, dx                                ; 89 d1
    xor si, si                                ; 31 f6
    mov dx, ax                                ; 89 c2
    mov ax, 0b103h                            ; b8 03 b1
    sal ecx, 010h                             ; 66 c1 e1 10
    db  08bh, 0cah
    ; mov cx, dx                                ; 8b ca
    int 01ah                                  ; cd 1a
    cmp ah, 000h                              ; 80 fc 00
    je near 08b04h                            ; 0f 84 03 00
    mov bx, strict word 0ffffh                ; bb ff ff
    mov ax, bx                                ; 89 d8
    pop bp                                    ; 5d
    pop si                                    ; 5e
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
pci_read_config_byte_:                       ; 0xf8b0b LB 0x21
    push cx                                   ; 51
    push di                                   ; 57
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    movzx di, bl                              ; 0f b6 fb
    movzx bx, al                              ; 0f b6 d8
    sal bx, 008h                              ; c1 e3 08
    movzx ax, dl                              ; 0f b6 c2
    or bx, ax                                 ; 09 c3
    mov ax, 0b108h                            ; b8 08 b1
    int 01ah                                  ; cd 1a
    movzx ax, cl                              ; 0f b6 c1
    xor dx, dx                                ; 31 d2
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop cx                                    ; 59
    retn                                      ; c3
pci_read_config_word_:                       ; 0xf8b2c LB 0x1f
    push cx                                   ; 51
    push di                                   ; 57
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    movzx di, bl                              ; 0f b6 fb
    xor ah, ah                                ; 30 e4
    sal ax, 008h                              ; c1 e0 08
    movzx bx, dl                              ; 0f b6 da
    or bx, ax                                 ; 09 c3
    mov ax, 0b109h                            ; b8 09 b1
    int 01ah                                  ; cd 1a
    mov ax, cx                                ; 89 c8
    xor dx, dx                                ; 31 d2
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop cx                                    ; 59
    retn                                      ; c3
pci_read_config_dword_:                      ; 0xf8b4b LB 0x24
    push cx                                   ; 51
    push di                                   ; 57
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    movzx di, bl                              ; 0f b6 fb
    movzx bx, al                              ; 0f b6 d8
    sal bx, 008h                              ; c1 e3 08
    movzx ax, dl                              ; 0f b6 c2
    or bx, ax                                 ; 09 c3
    mov ax, 0b10ah                            ; b8 0a b1
    int 01ah                                  ; cd 1a
    db  08bh, 0c1h
    ; mov ax, cx                                ; 8b c1
    shr ecx, 010h                             ; 66 c1 e9 10
    mov dx, cx                                ; 89 ca
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop cx                                    ; 59
    retn                                      ; c3
vds_is_present_:                             ; 0xf8b6f LB 0x1d
    push bx                                   ; 53
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bx, strict word 0007bh                ; bb 7b 00
    mov ax, strict word 00040h                ; b8 40 00
    mov es, ax                                ; 8e c0
    test byte [es:bx], 020h                   ; 26 f6 07 20
    je short 08b87h                           ; 74 06
    mov ax, strict word 00001h                ; b8 01 00
    pop bp                                    ; 5d
    pop bx                                    ; 5b
    retn                                      ; c3
    xor ax, ax                                ; 31 c0
    pop bp                                    ; 5d
    pop bx                                    ; 5b
    retn                                      ; c3
vds_real_to_lin_:                            ; 0xf8b8c LB 0x1e
    push bx                                   ; 53
    push cx                                   ; 51
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bx, ax                                ; 89 c3
    mov ax, dx                                ; 89 d0
    xor dx, dx                                ; 31 d2
    mov cx, strict word 00004h                ; b9 04 00
    sal ax, 1                                 ; d1 e0
    rcl dx, 1                                 ; d1 d2
    loop 08b9ah                               ; e2 fa
    xor cx, cx                                ; 31 c9
    add ax, bx                                ; 01 d8
    adc dx, cx                                ; 11 ca
    pop bp                                    ; 5d
    pop cx                                    ; 59
    pop bx                                    ; 5b
    retn                                      ; c3
vds_build_sg_list_:                          ; 0xf8baa LB 0x76
    push si                                   ; 56
    push di                                   ; 57
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov di, ax                                ; 89 c7
    mov si, dx                                ; 89 d6
    mov ax, bx                                ; 89 d8
    mov dx, cx                                ; 89 ca
    mov bx, word [bp+008h]                    ; 8b 5e 08
    mov es, si                                ; 8e c6
    mov word [es:di], bx                      ; 26 89 1d
    mov bx, word [bp+00ah]                    ; 8b 5e 0a
    mov word [es:di+002h], bx                 ; 26 89 5d 02
    call 08b8ch                               ; e8 c3 ff
    mov es, si                                ; 8e c6
    mov word [es:di+004h], ax                 ; 26 89 45 04
    mov word [es:di+006h], dx                 ; 26 89 55 06
    mov word [es:di+008h], strict word 00000h ; 26 c7 45 08 00 00
    call 08b6fh                               ; e8 93 ff
    test ax, ax                               ; 85 c0
    je short 08bf3h                           ; 74 13
    mov es, si                                ; 8e c6
    mov ax, 08105h                            ; b8 05 81
    mov dx, strict word 00000h                ; ba 00 00
    int 04bh                                  ; cd 4b
    jc near 08bf0h                            ; 0f 82 02 00
    db  032h, 0c0h
    ; xor al, al                                ; 32 c0
    cbw                                       ; 98
    jmp short 08c1ah                          ; eb 27
    mov es, si                                ; 8e c6
    mov word [es:di+00eh], strict word 00001h ; 26 c7 45 0e 01 00
    mov dx, word [es:di+004h]                 ; 26 8b 55 04
    mov ax, word [es:di+006h]                 ; 26 8b 45 06
    mov word [es:di+010h], dx                 ; 26 89 55 10
    mov word [es:di+012h], ax                 ; 26 89 45 12
    mov ax, word [bp+008h]                    ; 8b 46 08
    mov word [es:di+014h], ax                 ; 26 89 45 14
    mov ax, bx                                ; 89 d8
    mov word [es:di+016h], bx                 ; 26 89 5d 16
    xor ax, bx                                ; 31 d8
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop si                                    ; 5e
    retn 00004h                               ; c2 04 00
vds_free_sg_list_:                           ; 0xf8c20 LB 0x30
    push bx                                   ; 53
    push di                                   ; 57
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    mov bx, ax                                ; 89 c3
    call 08b6fh                               ; e8 45 ff
    test ax, ax                               ; 85 c0
    je short 08c41h                           ; 74 13
    mov di, bx                                ; 89 df
    mov es, dx                                ; 8e c2
    mov ax, 08106h                            ; b8 06 81
    mov dx, strict word 00000h                ; ba 00 00
    int 04bh                                  ; cd 4b
    jc near 08c40h                            ; 0f 82 02 00
    db  032h, 0c0h
    ; xor al, al                                ; 32 c0
    cbw                                       ; 98
    mov es, dx                                ; 8e c2
    mov word [es:bx+00eh], strict word 00000h ; 26 c7 47 0e 00 00
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop bx                                    ; 5b
    retn                                      ; c3
    times 0x3 db 0
__U4D:                                       ; 0xf8c50 LB 0x39
    pushfw                                    ; 9c
    push eax                                  ; 66 50
    push edx                                  ; 66 52
    push ecx                                  ; 66 51
    rol eax, 010h                             ; 66 c1 c0 10
    db  08bh, 0c2h
    ; mov ax, dx                                ; 8b c2
    ror eax, 010h                             ; 66 c1 c8 10
    db  066h, 033h, 0d2h
    ; xor edx, edx                              ; 66 33 d2
    shr ecx, 010h                             ; 66 c1 e9 10
    db  08bh, 0cbh
    ; mov cx, bx                                ; 8b cb
    div ecx                                   ; 66 f7 f1
    db  08bh, 0dah
    ; mov bx, dx                                ; 8b da
    pop ecx                                   ; 66 59
    shr edx, 010h                             ; 66 c1 ea 10
    db  08bh, 0cah
    ; mov cx, dx                                ; 8b ca
    pop edx                                   ; 66 5a
    ror eax, 010h                             ; 66 c1 c8 10
    db  08bh, 0d0h
    ; mov dx, ax                                ; 8b d0
    add sp, strict byte 00002h                ; 83 c4 02
    pop ax                                    ; 58
    rol eax, 010h                             ; 66 c1 c0 10
    popfw                                     ; 9d
    retn                                      ; c3
__U4M:                                       ; 0xf8c89 LB 0x31
    pushfw                                    ; 9c
    push eax                                  ; 66 50
    push edx                                  ; 66 52
    push ecx                                  ; 66 51
    rol eax, 010h                             ; 66 c1 c0 10
    db  08bh, 0c2h
    ; mov ax, dx                                ; 8b c2
    ror eax, 010h                             ; 66 c1 c8 10
    db  066h, 033h, 0d2h
    ; xor edx, edx                              ; 66 33 d2
    shr ecx, 010h                             ; 66 c1 e9 10
    db  08bh, 0cbh
    ; mov cx, bx                                ; 8b cb
    mul ecx                                   ; 66 f7 e1
    pop ecx                                   ; 66 59
    pop edx                                   ; 66 5a
    ror eax, 010h                             ; 66 c1 c8 10
    db  08bh, 0d0h
    ; mov dx, ax                                ; 8b d0
    add sp, strict byte 00002h                ; 83 c4 02
    pop ax                                    ; 58
    rol eax, 010h                             ; 66 c1 c0 10
    popfw                                     ; 9d
    retn                                      ; c3
_fmemset_:                                   ; 0xf8cba LB 0xd
    push di                                   ; 57
    mov es, dx                                ; 8e c2
    db  08bh, 0f8h
    ; mov di, ax                                ; 8b f8
    xchg al, bl                               ; 86 d8
    rep stosb                                 ; f3 aa
    xchg al, bl                               ; 86 d8
    pop di                                    ; 5f
    retn                                      ; c3
_fmemcpy_:                                   ; 0xf8cc7 LB 0x33
    push bp                                   ; 55
    db  08bh, 0ech
    ; mov bp, sp                                ; 8b ec
    push di                                   ; 57
    push DS                                   ; 1e
    push si                                   ; 56
    mov es, dx                                ; 8e c2
    db  08bh, 0f8h
    ; mov di, ax                                ; 8b f8
    mov ds, cx                                ; 8e d9
    db  08bh, 0f3h
    ; mov si, bx                                ; 8b f3
    mov cx, word [bp+004h]                    ; 8b 4e 04
    rep movsb                                 ; f3 a4
    pop si                                    ; 5e
    pop DS                                    ; 1f
    pop di                                    ; 5f
    leave                                     ; c9
    retn                                      ; c3
    add byte [bx+si], dl                      ; 00 10
    lea dx, [bp+si]                           ; 8d 12
    lea dx, [0168dh]                          ; 8d 16 8d 16
    lea dx, [0188dh]                          ; 8d 16 8d 18
    lea bx, [bx+si]                           ; 8d 18
    lea bx, [bp+si]                           ; 8d 1a
    lea bx, [01e8dh]                          ; 8d 1e 8d 1e
    lea sp, [bx+si]                           ; 8d 20
    lea sp, [di]                              ; 8d 25
    lea sp, [bx]                              ; 8d 27
    db  08dh
apm_worker:                                  ; 0xf8cfa LB 0x3a
    sti                                       ; fb
    push ax                                   ; 50
    db  032h, 0e4h
    ; xor ah, ah                                ; 32 e4
    sub AL, strict byte 004h                  ; 2c 04
    db  08bh, 0e8h
    ; mov bp, ax                                ; 8b e8
    sal bp, 1                                 ; d1 e5
    cmp AL, strict byte 00dh                  ; 3c 0d
    pop ax                                    ; 58
    mov AH, strict byte 053h                  ; b4 53
    jnc short 08d30h                          ; 73 25
    jmp word [cs:bp-07320h]                   ; 2e ff a6 e0 8c
    jmp short 08d2eh                          ; eb 1c
    sti                                       ; fb
    hlt                                       ; f4
    jmp short 08d2eh                          ; eb 18
    jmp short 08d2eh                          ; eb 16
    jmp short 08d30h                          ; eb 16
    mov AH, strict byte 080h                  ; b4 80
    jmp short 08d32h                          ; eb 14
    jmp short 08d30h                          ; eb 10
    mov ax, 00102h                            ; b8 02 01
    jmp short 08d2eh                          ; eb 09
    jmp short 08d2eh                          ; eb 07
    mov BL, strict byte 000h                  ; b3 00
    mov cx, strict word 00000h                ; b9 00 00
    jmp short 08d2eh                          ; eb 00
    clc                                       ; f8
    retn                                      ; c3
    mov AH, strict byte 009h                  ; b4 09
    stc                                       ; f9
    retn                                      ; c3
apm_pm16_entry:                              ; 0xf8d34 LB 0x11
    mov AH, strict byte 002h                  ; b4 02
    push DS                                   ; 1e
    push bp                                   ; 55
    push CS                                   ; 0e
    pop bp                                    ; 5d
    add bp, strict byte 00008h                ; 83 c5 08
    mov ds, bp                                ; 8e dd
    call 08cfah                               ; e8 b8 ff
    pop bp                                    ; 5d
    pop DS                                    ; 1f
    retf                                      ; cb

  ; Padding 0x4cbb bytes at 0xf8d45
  times 19643 db 0

section BIOS32 progbits vstart=0xda00 align=1 ; size=0x3aa class=CODE group=AUTO
bios32_service:                              ; 0xfda00 LB 0x26
    pushfw                                    ; 9c
    cmp bl, 000h                              ; 80 fb 00
    jne short 0da22h                          ; 75 1c
    cmp ax, 05024h                            ; 3d 24 50
    inc bx                                    ; 43
    dec cx                                    ; 49
    mov AL, strict byte 080h                  ; b0 80
    jne short 0da20h                          ; 75 11
    mov bx, strict word 00000h                ; bb 00 00
    db  00fh
    add byte [bx+di-01000h], bh               ; 00 b9 00 f0
    add byte [bx+si], al                      ; 00 00
    mov dx, 0da26h                            ; ba 26 da
    add byte [bx+si], al                      ; 00 00
    db  032h, 0c0h
    ; xor al, al                                ; 32 c0
    popfw                                     ; 9d
    retf                                      ; cb
    mov AL, strict byte 081h                  ; b0 81
    jmp short 0da20h                          ; eb fa
pcibios32_entry:                             ; 0xfda26 LB 0x1a
    pushfw                                    ; 9c
    cld                                       ; fc
    push ES                                   ; 06
    pushaw                                    ; 60
    call 0db73h                               ; e8 46 01
    add byte [bx+si], al                      ; 00 00
    popaw                                     ; 61
    pop ES                                    ; 07
    popfw                                     ; 9d
    retf                                      ; cb
    times 0xd db 0
apm_pm32_entry:                              ; 0xfda40 LB 0x21
    push bp                                   ; 55
    mov ebp, cs                               ; 66 8c cd
    push ebp                                  ; 66 55
    mov bp, 0da5fh                            ; bd 5f da
    add byte [bx+si], al                      ; 00 00
    push ebp                                  ; 66 55
    push CS                                   ; 0e
    pop bp                                    ; 5d
    add bp, strict byte 00008h                ; 83 c5 08
    push ebp                                  ; 66 55
    mov bp, 08d36h                            ; bd 36 8d
    add byte [bx+si], al                      ; 00 00
    push ebp                                  ; 66 55
    mov AH, strict byte 003h                  ; b4 03
    db  066h, 0cbh
    ; retf                                      ; 66 cb
    pop bp                                    ; 5d
    retf                                      ; cb
pci32_select_reg_:                           ; 0xfda61 LB 0x1f
    push bx                                   ; 53
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    and dl, 0fch                              ; 80 e2 fc
    mov bx, dx                                ; 89 d3
    mov dx, 00cf8h                            ; ba f8 0c
    add byte [bx+si], al                      ; 00 00
    db  00fh, 0b7h, 0c0h
    ; movzx ax, ax                              ; 0f b7 c0
    sal ax, 008h                              ; c1 e0 08
    or ax, strict word 00000h                 ; 0d 00 00
    add byte [bx+si-03c76h], al               ; 00 80 8a c3
    out DX, ax                                ; ef
    pop bp                                    ; 5d
    pop bx                                    ; 5b
    retn                                      ; c3
pci32_find_device_:                          ; 0xfda80 LB 0xf5
    push cx                                   ; 51
    push si                                   ; 56
    push di                                   ; 57
    enter 00014h, 000h                        ; c8 14 00 00
    push ax                                   ; 50
    mov cx, dx                                ; 89 d1
    mov si, bx                                ; 89 de
    test bx, bx                               ; 85 db
    xor bx, bx                                ; 31 db
    mov byte [di-004h], 000h                  ; c6 45 fc 00
    test bl, 007h                             ; f6 c3 07
    jne short 0dacfh                          ; 75 36
    db  00fh, 0b7h, 0c3h
    ; movzx ax, bx                              ; 0f b7 c3
    mov dx, strict word 0000eh                ; ba 0e 00
    add byte [bx+si], al                      ; 00 00
    call 0da5fh                               ; e8 bb ff
    db  0ffh
    db  0ffh
    mov dx, 00cfeh                            ; ba fe 0c
    add byte [bx+si], al                      ; 00 00
    db  02bh, 0c0h
    ; sub ax, ax                                ; 2b c0
    in AL, DX                                 ; ec
    mov byte [di-008h], al                    ; 88 45 f8
    cmp AL, strict byte 0ffh                  ; 3c ff
    jne short 0dabdh                          ; 75 08
    add bx, strict byte 00008h                ; 83 c3 08
    jmp near 0db45h                           ; e9 8a 00
    add byte [bx+si], al                      ; 00 00
    test byte [di-008h], 080h                 ; f6 45 f8 80
    je short 0dacah                           ; 74 07
    mov di, strict word 00001h                ; bf 01 00
    add byte [bx+si], al                      ; 00 00
    jmp short 0dacfh                          ; eb 05
    mov di, strict word 00008h                ; bf 08 00
    add byte [bx+si], al                      ; 00 00
    mov al, byte [di-008h]                    ; 8a 45 f8
    and AL, strict byte 007h                  ; 24 07
    cmp AL, strict byte 001h                  ; 3c 01
    jne short 0dafeh                          ; 75 26
    db  00fh, 0b7h, 0c3h
    ; movzx ax, bx                              ; 0f b7 c3
    mov dx, ax                                ; 89 c2
    sar dx, 008h                              ; c1 fa 08
    test dx, dx                               ; 85 d2
    jne short 0dafeh                          ; 75 1a
    mov dx, strict word 0001ah                ; ba 1a 00
    add byte [bx+si], al                      ; 00 00
    call 0da5fh                               ; e8 73 ff
    db  0ffh
    db  0ffh
    mov dx, 00cfeh                            ; ba fe 0c
    add byte [bx+si], al                      ; 00 00
    db  02bh, 0c0h
    ; sub ax, ax                                ; 2b c0
    in AL, DX                                 ; ec
    cmp al, byte [di-004h]                    ; 3a 45 fc
    jbe short 0dafeh                          ; 76 03
    mov byte [di-004h], al                    ; 88 45 fc
    test si, si                               ; 85 f6
    je short 0db09h                           ; 74 07
    mov ax, strict word 00008h                ; b8 08 00
    add byte [bx+si], al                      ; 00 00
    jmp short 0db0bh                          ; eb 02
    xor ax, ax                                ; 31 c0
    db  00fh, 0b7h, 0d0h
    ; movzx dx, ax                              ; 0f b7 d0
    db  00fh, 0b7h, 0c3h
    ; movzx ax, bx                              ; 0f b7 c3
    call 0da5fh                               ; e8 4b ff
    db  0ffh
    db  0ffh
    mov dx, 00cfch                            ; ba fc 0c
    add byte [bx+si], al                      ; 00 00
    in ax, DX                                 ; ed
    mov word [di-00ch], ax                    ; 89 45 f4
    mov word [di-014h], strict word 00000h    ; c7 45 ec 00 00
    add byte [bx+si], al                      ; 00 00
    test si, si                               ; 85 f6
    je short 0db30h                           ; 74 06
    shr ax, 008h                              ; c1 e8 08
    mov word [di-00ch], ax                    ; 89 45 f4
    mov ax, word [di-00ch]                    ; 8b 45 f4
    cmp ax, word [di-018h]                    ; 3b 45 e8
    je short 0db3eh                           ; 74 06
    cmp word [di-014h], strict byte 00000h    ; 83 7d ec 00
    je short 0db45h                           ; 74 07
    dec cx                                    ; 49
    cmp ecx, strict byte 0ffffffffh           ; 66 83 f9 ff
    je short 0db5dh                           ; 74 18
    add bx, di                                ; 01 fb
    db  00fh, 0b7h, 0c3h
    ; movzx ax, bx                              ; 0f b7 c3
    sar ax, 008h                              ; c1 f8 08
    mov word [di-010h], ax                    ; 89 45 f0
    movzx ax, byte [di-004h]                  ; 0f b6 45 fc
    cmp ax, word [di-010h]                    ; 3b 45 f0
    jnl near 0da92h                           ; 0f 8d 37 ff
    db  0ffh
    jmp word [bp-07dh]                        ; ff 66 83
    stc                                       ; f9
    push word [di+008h]                       ; ff 75 08
    db  00fh, 0b7h, 0c3h
    ; movzx ax, bx                              ; 0f b7 c3
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop cx                                    ; 59
    retn                                      ; c3
    mov ax, strict word 0ffffh                ; b8 ff ff
    add byte [bx+si], al                      ; 00 00
    leave                                     ; c9
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop cx                                    ; 59
    retn                                      ; c3
_pci32_function:                             ; 0xfdb75 LB 0x235
    push bx                                   ; 53
    push si                                   ; 56
    push di                                   ; 57
    push bp                                   ; 55
    mov bp, sp                                ; 89 e5
    and dword [di+030h], strict dword 0658100ffh ; 66 81 65 30 ff 00 81 65
    cmp dh, bh                                ; 38 fe
    inc word [bx+si]                          ; ff 00
    add byte [bp+di+03045h], cl               ; 00 8b 45 30
    xor ah, ah                                ; 30 e4
    cmp eax, strict dword 029720003h          ; 66 3d 03 00 72 29
    jbe near 0dc30h                           ; 0f 86 99 00
    add byte [bx+si], al                      ; 00 00
    cmp eax, strict dword 0840f000eh          ; 66 3d 0e 00 0f 84
    lodsw                                     ; ad
    add word [bx+si], ax                      ; 01 00
    add byte [bp+03dh], ah                    ; 00 66 3d
    or byte [bx+si], al                       ; 08 00
    jc near 0dd93h                            ; 0f 82 e8 01
    add byte [bx+si], al                      ; 00 00
    cmp eax, strict dword 0860f000dh          ; 66 3d 0d 00 0f 86
    test AL, strict byte 000h                 ; a8 00
    add byte [bx+si], al                      ; 00 00
    jmp near 0dd93h                           ; e9 d9 01
    add byte [bx+si], al                      ; 00 00
    cmp eax, strict dword 028740002h          ; 66 3d 02 00 74 28
    cmp eax, strict dword 0850f0001h          ; 66 3d 01 00 0f 85
    leave                                     ; c9
    add word [bx+si], ax                      ; 01 00
    add byte [bp-039h], ah                    ; 00 66 c7
    inc bp                                    ; 45
    xor byte [bx+di], al                      ; 30 01
    add byte [bp-039h], ah                    ; 00 66 c7
    inc bp                                    ; 45
    and AL, strict byte 010h                  ; 24 10
    add dh, byte [bx+di]                      ; 02 31
    sal byte [bp-077h], 045h                  ; c0 66 89 45
    sub AL, strict byte 0c7h                  ; 2c c7
    inc bp                                    ; 45
    sub byte [bx+si+043h], dl                 ; 28 50 43
    dec cx                                    ; 49
    and byte [di+05fh], bl                    ; 20 5d 5f
    pop si                                    ; 5e
    pop bx                                    ; 5b
    retn                                      ; c3
    cmp dword [di+028h], strict byte 0ffffffffh ; 66 83 7d 28 ff
    jne short 0dbfeh                          ; 75 0d
    mov ax, word [di+030h]                    ; 8b 45 30
    xor ah, ah                                ; 30 e4
    or ah, 083h                               ; 80 cc 83
    jmp near 0dd9bh                           ; e9 9f 01
    add byte [bx+si], al                      ; 00 00
    xor bx, bx                                ; 31 db
    db  00fh, 0b7h, 055h, 018h
    ; movzx dx, word [di+018h]                  ; 0f b7 55 18
    db  00fh, 0b7h, 04dh, 02ch
    ; movzx cx, word [di+02ch]                  ; 0f b7 4d 2c
    sal cx, 010h                              ; c1 e1 10
    db  00fh, 0b7h, 045h, 028h
    ; movzx ax, word [di+028h]                  ; 0f b7 45 28
    or ax, cx                                 ; 09 c8
    call 0da7eh                               ; e8 6a fe
    db  0ffh
    jmp word [bp+03dh]                        ; ff 66 3d
    db  0ffh
    push word [di+00dh]                       ; ff 75 0d
    mov ax, word [di+030h]                    ; 8b 45 30
    xor ah, ah                                ; 30 e4
    or ah, 086h                               ; 80 cc 86
    jmp near 0dd9bh                           ; e9 74 01
    add byte [bx+si], al                      ; 00 00
    mov dword [di+024h], eax                  ; 66 89 45 24
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop bx                                    ; 5b
    retn                                      ; c3
    db  00fh, 0b7h, 055h, 018h
    ; movzx dx, word [di+018h]                  ; 0f b7 55 18
    mov ax, word [di+02ch]                    ; 8b 45 2c
    mov bx, strict word 00001h                ; bb 01 00
    add byte [bx+si], al                      ; 00 00
    call 0da7eh                               ; e8 3d fe
    db  0ffh
    jmp word [bp+03dh]                        ; ff 66 3d
    db  0ffh
    push word [di+00dh]                       ; ff 75 0d
    mov ax, word [di+030h]                    ; 8b 45 30
    xor ah, ah                                ; 30 e4
    or ah, 086h                               ; 80 cc 86
    jmp near 0dd9bh                           ; e9 47 01
    add byte [bx+si], al                      ; 00 00
    mov dword [di+024h], eax                  ; 66 89 45 24
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop bx                                    ; 5b
    retn                                      ; c3
    cmp dword [di+014h], strict dword 00d720100h ; 66 81 7d 14 00 01 72 0d
    mov ax, word [di+030h]                    ; 8b 45 30
    xor ah, ah                                ; 30 e4
    or ah, 087h                               ; 80 cc 87
    jmp near 0dd9bh                           ; e9 29 01
    add byte [bx+si], al                      ; 00 00
    db  00fh, 0b7h, 055h, 014h
    ; movzx dx, word [di+014h]                  ; 0f b7 55 14
    db  00fh, 0b7h, 045h, 024h
    ; movzx ax, word [di+024h]                  ; 0f b7 45 24
    call 0da5fh                               ; e8 e0 fd
    db  0ffh
    dec word [bp+di+03045h]                   ; ff 8b 45 30
    xor ah, ah                                ; 30 e4
    cmp eax, strict dword 02172000ah          ; 66 3d 0a 00 72 21
    jbe short 0dcfeh                          ; 76 70
    cmp eax, strict dword 0840f000dh          ; 66 3d 0d 00 0f 84
    stosb                                     ; aa
    add byte [bx+si], al                      ; 00 00
    add byte [bp+03dh], ah                    ; 00 66 3d
    or AL, strict byte 000h                   ; 0c 00
    je near 0dd24h                            ; 0f 84 84 00
    add byte [bx+si], al                      ; 00 00
    cmp eax, strict dword 06474000bh          ; 66 3d 0b 00 74 64
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop bx                                    ; 5b
    retn                                      ; c3
    cmp eax, strict dword 02e740009h          ; 66 3d 09 00 74 2e
    cmp eax, strict dword 005740008h          ; 66 3d 08 00 74 05
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop bx                                    ; 5b
    retn                                      ; c3
    mov bx, word [di+02ch]                    ; 8b 5d 2c
    xor bl, bl                                ; 30 db
    mov ax, word [di+014h]                    ; 8b 45 14
    xor ah, ah                                ; 30 e4
    and AL, strict byte 003h                  ; 24 03
    db  00fh, 0b7h, 0d0h
    ; movzx dx, ax                              ; 0f b7 d0
    add dx, 00cfch                            ; 81 c2 fc 0c
    add byte [bx+si], al                      ; 00 00
    db  02bh, 0c0h
    ; sub ax, ax                                ; 2b c0
    in AL, DX                                 ; ec
    or bx, ax                                 ; 09 c3
    mov dword [di+02ch], ebx                  ; 66 89 5d 2c
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop bx                                    ; 5b
    retn                                      ; c3
    mov ax, word [di+014h]                    ; 8b 45 14
    xor ah, ah                                ; 30 e4
    and AL, strict byte 002h                  ; 24 02
    db  00fh, 0b7h, 0d0h
    ; movzx dx, ax                              ; 0f b7 d0
    add dx, 00cfch                            ; 81 c2 fc 0c
    add byte [bx+si], al                      ; 00 00
    db  02bh, 0c0h
    ; sub ax, ax                                ; 2b c0
    in eax, DX                                ; 66 ed
    mov dword [di+02ch], eax                  ; 66 89 45 2c
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop bx                                    ; 5b
    retn                                      ; c3
    mov dx, 00cfch                            ; ba fc 0c
    add byte [bx+si], al                      ; 00 00
    in ax, DX                                 ; ed
    mov word [di+02ch], ax                    ; 89 45 2c
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop bx                                    ; 5b
    retn                                      ; c3
    mov ax, word [di+02ch]                    ; 8b 45 2c
    mov dx, word [di+014h]                    ; 8b 55 14
    xor dh, dh                                ; 30 f6
    and dl, 003h                              ; 80 e2 03
    db  00fh, 0b7h, 0d2h
    ; movzx dx, dx                              ; 0f b7 d2
    add dx, 00cfch                            ; 81 c2 fc 0c
    add byte [bx+si], al                      ; 00 00
    out DX, AL                                ; ee
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop bx                                    ; 5b
    retn                                      ; c3
    db  00fh, 0b7h, 045h, 02ch
    ; movzx ax, word [di+02ch]                  ; 0f b7 45 2c
    mov dx, word [di+014h]                    ; 8b 55 14
    xor dh, dh                                ; 30 f6
    and dl, 002h                              ; 80 e2 02
    db  00fh, 0b7h, 0d2h
    ; movzx dx, dx                              ; 0f b7 d2
    add dx, 00cfch                            ; 81 c2 fc 0c
    add byte [bx+si], al                      ; 00 00
    out DX, eax                               ; 66 ef
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop bx                                    ; 5b
    retn                                      ; c3
    mov ax, word [di+02ch]                    ; 8b 45 2c
    mov dx, 00cfch                            ; ba fc 0c
    add byte [bx+si], al                      ; 00 00
    out DX, ax                                ; ef
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop bx                                    ; 5b
    retn                                      ; c3
    db  00fh, 0b7h, 045h, 014h
    ; movzx ax, word [di+014h]                  ; 0f b7 45 14
    mov es, [di+034h]                         ; 8e 45 34
    mov ecx, dword [di]                       ; 66 8b 0d
    add byte [bx+si], al                      ; 00 00
    add byte [bx+si], al                      ; 00 00
    db  066h, 026h, 03bh, 008h
    ; cmp ecx, dword [es:bx+si]                 ; 66 26 3b 08
    jbe short 0dd6eh                          ; 76 0a
    mov ax, word [di+030h]                    ; 8b 45 30
    xor ah, ah                                ; 30 e4
    or ah, 089h                               ; 80 cc 89
    jmp short 0dd9dh                          ; eb 2f
    db  00fh, 0b7h, 0c9h
    ; movzx cx, cx                              ; 0f b7 c9
    db  066h, 026h, 08bh, 058h, 006h
    ; mov ebx, dword [es:bx+si+006h]            ; 66 26 8b 58 06
    mov di, word [es:bx+si+002h]              ; 26 8b 78 02
    mov dx, ds                                ; 8c da
    mov si, 0f2c0h                            ; be c0 f2
    add byte [bx+si], al                      ; 00 00
    mov es, bx                                ; 8e c3
    push DS                                   ; 1e
    db  066h, 08eh, 0dah
    ; mov ds, edx                               ; 66 8e da
    rep movsb                                 ; f3 a4
    pop DS                                    ; 1f
    mov dword [di+024h], strict dword 05f5d0a00h ; 66 c7 45 24 00 0a 5d 5f
    pop si                                    ; 5e
    pop bx                                    ; 5b
    retn                                      ; c3
    mov ax, word [di+030h]                    ; 8b 45 30
    xor ah, ah                                ; 30 e4
    or ah, 081h                               ; 80 cc 81
    mov dword [di+030h], eax                  ; 66 89 45 30
    or word [di+038h], strict byte 00001h     ; 83 4d 38 01
    pop bp                                    ; 5d
    pop di                                    ; 5f
    pop si                                    ; 5e
    pop bx                                    ; 5b
    retn                                      ; c3

  ; Padding 0x2 bytes at 0xfddaa
  times 2 db 0

section BIOS32CONST progbits vstart=0xddac align=1 ; size=0x0 class=FAR_DATA group=BIOS32_GROUP

section BIOS32CONST2 progbits vstart=0xddac align=1 ; size=0x0 class=FAR_DATA group=BIOS32_GROUP

section BIOS32_DATA progbits vstart=0xddac align=1 ; size=0x0 class=FAR_DATA group=BIOS32_GROUP

  ; Padding 0x254 bytes at 0xfddac
  times 596 db 0

section BIOSSEG progbits vstart=0xe000 align=1 ; size=0x2000 class=CODE group=AUTO
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 058h, 04dh
eoi_jmp_post:                                ; 0xfe030 LB 0xb
    call 0e03bh                               ; e8 08 00
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    mov ds, ax                                ; 8e d8
    jmp far [00467h]                          ; ff 2e 67 04
eoi_both_pics:                               ; 0xfe03b LB 0x4
    mov AL, strict byte 020h                  ; b0 20
    out strict byte 0a0h, AL                  ; e6 a0
eoi_master_pic:                              ; 0xfe03f LB 0x1c
    mov AL, strict byte 020h                  ; b0 20
    out strict byte 020h, AL                  ; e6 20
    retn                                      ; c3
    times 0x15 db 0
    db  'XM'
post:                                        ; 0xfe05b LB 0x32
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    out strict byte 00dh, AL                  ; e6 0d
    out strict byte 0dah, AL                  ; e6 da
    mov AL, strict byte 0c0h                  ; b0 c0
    out strict byte 0d6h, AL                  ; e6 d6
    mov AL, strict byte 000h                  ; b0 00
    out strict byte 0d4h, AL                  ; e6 d4
    mov AL, strict byte 00fh                  ; b0 0f
    out strict byte 070h, AL                  ; e6 70
    in AL, strict byte 071h                   ; e4 71
    db  08ah, 0d8h
    ; mov bl, al                                ; 8a d8
    mov AL, strict byte 00fh                  ; b0 0f
    out strict byte 070h, AL                  ; e6 70
    mov AL, strict byte 000h                  ; b0 00
    out strict byte 071h, AL                  ; e6 71
    db  08ah, 0c3h
    ; mov al, bl                                ; 8a c3
    cmp AL, strict byte 000h                  ; 3c 00
    je short 0e098h                           ; 74 19
    cmp AL, strict byte 00dh                  ; 3c 0d
    jnc short 0e098h                          ; 73 15
    cmp AL, strict byte 009h                  ; 3c 09
    je short 0e098h                           ; 74 11
    cmp AL, strict byte 005h                  ; 3c 05
    je short 0e030h                           ; 74 a5
    jmp short 0e098h                          ; eb 0b
set_int_vects:                               ; 0xfe08d LB 0xb
    mov word [bx], ax                         ; 89 07
    mov word [bx+002h], dx                    ; 89 57 02
    add bx, strict byte 00004h                ; 83 c3 04
    loop 0e08dh                               ; e2 f6
    retn                                      ; c3
normal_post:                                 ; 0xfe098 LB 0x22b
    cli                                       ; fa
    mov ax, 07800h                            ; b8 00 78
    db  08bh, 0e0h
    ; mov sp, ax                                ; 8b e0
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    mov ds, ax                                ; 8e d8
    mov ss, ax                                ; 8e d0
    mov es, ax                                ; 8e c0
    db  033h, 0ffh
    ; xor di, di                                ; 33 ff
    cld                                       ; fc
    mov cx, 00239h                            ; b9 39 02
    rep stosw                                 ; f3 ab
    inc di                                    ; 47
    inc di                                    ; 47
    mov cx, 005c6h                            ; b9 c6 05
    rep stosw                                 ; f3 ab
    db  033h, 0dbh
    ; xor bx, bx                                ; 33 db
    add bx, 01000h                            ; 81 c3 00 10
    cmp bx, 09000h                            ; 81 fb 00 90
    jnc short 0e0cch                          ; 73 0b
    mov es, bx                                ; 8e c3
    db  033h, 0ffh
    ; xor di, di                                ; 33 ff
    mov cx, 08000h                            ; b9 00 80
    rep stosw                                 ; f3 ab
    jmp short 0e0b7h                          ; eb eb
    mov es, bx                                ; 8e c3
    db  033h, 0ffh
    ; xor di, di                                ; 33 ff
    mov cx, 07ff8h                            ; b9 f8 7f
    rep stosw                                 ; f3 ab
    db  033h, 0dbh
    ; xor bx, bx                                ; 33 db
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    call 01707h                               ; e8 2a 36
    call 0e8e0h                               ; e8 00 08
    db  033h, 0dbh
    ; xor bx, bx                                ; 33 db
    mov ds, bx                                ; 8e db
    mov cx, strict word 00060h                ; b9 60 00
    mov ax, 0ff53h                            ; b8 53 ff
    mov dx, 0f000h                            ; ba 00 f0
    call 0e08dh                               ; e8 9d ff
    mov bx, 001a0h                            ; bb a0 01
    mov cx, strict word 00010h                ; b9 10 00
    call 0e08dh                               ; e8 94 ff
    mov ax, 0027fh                            ; b8 7f 02
    mov word [00413h], ax                     ; a3 13 04
    mov ax, 0f84dh                            ; b8 4d f8
    mov word [00044h], ax                     ; a3 44 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [00046h], ax                     ; a3 46 00
    mov ax, 0f841h                            ; b8 41 f8
    mov word [00048h], ax                     ; a3 48 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [0004ah], ax                     ; a3 4a 00
    mov ax, 0f859h                            ; b8 59 f8
    mov word [00054h], ax                     ; a3 54 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [00056h], ax                     ; a3 56 00
    mov ax, 0efd4h                            ; b8 d4 ef
    mov word [0005ch], ax                     ; a3 5c 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [0005eh], ax                     ; a3 5e 00
    mov ax, 0f0a4h                            ; b8 a4 f0
    mov word [00060h], ax                     ; a3 60 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [00062h], ax                     ; a3 62 00
    mov ax, 0e6f2h                            ; b8 f2 e6
    mov word [00064h], ax                     ; a3 64 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [00066h], ax                     ; a3 66 00
    mov ax, 0efedh                            ; b8 ed ef
    mov word [00070h], ax                     ; a3 70 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [00072h], ax                     ; a3 72 00
    call 0e7c0h                               ; e8 6a 06
    mov ax, 0fea5h                            ; b8 a5 fe
    mov word [00020h], ax                     ; a3 20 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [00022h], ax                     ; a3 22 00
    mov AL, strict byte 034h                  ; b0 34
    out strict byte 043h, AL                  ; e6 43
    mov AL, strict byte 000h                  ; b0 00
    out strict byte 040h, AL                  ; e6 40
    out strict byte 040h, AL                  ; e6 40
    mov ax, 0e987h                            ; b8 87 e9
    mov word [00024h], ax                     ; a3 24 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [00026h], ax                     ; a3 26 00
    mov ax, 0e82eh                            ; b8 2e e8
    mov word [00058h], ax                     ; a3 58 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [0005ah], ax                     ; a3 5a 00
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    mov ds, ax                                ; 8e d8
    mov byte [00417h], AL                     ; a2 17 04
    mov byte [00418h], AL                     ; a2 18 04
    mov byte [00419h], AL                     ; a2 19 04
    mov byte [00471h], AL                     ; a2 71 04
    mov byte [00497h], AL                     ; a2 97 04
    mov AL, strict byte 010h                  ; b0 10
    mov byte [00496h], AL                     ; a2 96 04
    mov bx, strict word 0001eh                ; bb 1e 00
    mov word [0041ah], bx                     ; 89 1e 1a 04
    mov word [0041ch], bx                     ; 89 1e 1c 04
    mov word [00480h], bx                     ; 89 1e 80 04
    mov bx, strict word 0003eh                ; bb 3e 00
    mov word [00482h], bx                     ; 89 1e 82 04
    push DS                                   ; 1e
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    call 048a0h                               ; e8 e7 66
    pop DS                                    ; 1f
    mov AL, strict byte 014h                  ; b0 14
    out strict byte 070h, AL                  ; e6 70
    in AL, strict byte 071h                   ; e4 71
    mov byte [00410h], AL                     ; a2 10 04
    mov ax, 0ff53h                            ; b8 53 ff
    mov word [0003ch], ax                     ; a3 3c 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [0003eh], ax                     ; a3 3e 00
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    mov ds, ax                                ; 8e d8
    db  033h, 0dbh
    ; xor bx, bx                                ; 33 db
    mov CL, strict byte 014h                  ; b1 14
    mov dx, 00378h                            ; ba 78 03
    call 0ecedh                               ; e8 10 0b
    mov dx, 00278h                            ; ba 78 02
    call 0ecedh                               ; e8 0a 0b
    sal bx, 00eh                              ; c1 e3 0e
    mov ax, word [00410h]                     ; a1 10 04
    and ax, 03fffh                            ; 25 ff 3f
    db  00bh, 0c3h
    ; or ax, bx                                 ; 0b c3
    mov word [00410h], ax                     ; a3 10 04
    mov ax, 0e746h                            ; b8 46 e7
    mov word [0002ch], ax                     ; a3 2c 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [0002eh], ax                     ; a3 2e 00
    mov ax, 0e746h                            ; b8 46 e7
    mov word [00030h], ax                     ; a3 30 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [00032h], ax                     ; a3 32 00
    mov ax, 0e739h                            ; b8 39 e7
    mov word [00050h], ax                     ; a3 50 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [00052h], ax                     ; a3 52 00
    db  033h, 0dbh
    ; xor bx, bx                                ; 33 db
    mov CL, strict byte 00ah                  ; b1 0a
    mov dx, 003f8h                            ; ba f8 03
    call 0ed0bh                               ; e8 ec 0a
    mov dx, 002f8h                            ; ba f8 02
    call 0ed0bh                               ; e8 e6 0a
    mov dx, 003e8h                            ; ba e8 03
    call 0ed0bh                               ; e8 e0 0a
    mov dx, 002e8h                            ; ba e8 02
    call 0ed0bh                               ; e8 da 0a
    sal bx, 009h                              ; c1 e3 09
    mov ax, word [00410h]                     ; a1 10 04
    and ax, 0f1ffh                            ; 25 ff f1
    db  00bh, 0c3h
    ; or ax, bx                                 ; 0b c3
    mov word [00410h], ax                     ; a3 10 04
    mov ax, 0fe6eh                            ; b8 6e fe
    mov word [00068h], ax                     ; a3 68 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [0006ah], ax                     ; a3 6a 00
    mov ax, 0ff53h                            ; b8 53 ff
    mov word [00128h], ax                     ; a3 28 01
    mov ax, 0f000h                            ; b8 00 f0
    mov word [0012ah], ax                     ; a3 2a 01
    mov ax, 0fe8fh                            ; b8 8f fe
    mov word [001c0h], ax                     ; a3 c0 01
    mov ax, 0f000h                            ; b8 00 f0
    mov word [001c2h], ax                     ; a3 c2 01
    call 0edbfh                               ; e8 59 0b
    mov ax, 0f8a4h                            ; b8 a4 f8
    mov word [001d0h], ax                     ; a3 d0 01
    mov ax, 0f000h                            ; b8 00 f0
    mov word [001d2h], ax                     ; a3 d2 01
    mov ax, 0e2cah                            ; b8 ca e2
    mov word [001d4h], ax                     ; a3 d4 01
    mov ax, 0f000h                            ; b8 00 f0
    mov word [001d6h], ax                     ; a3 d6 01
    mov ax, 0f065h                            ; b8 65 f0
    mov word [00040h], ax                     ; a3 40 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [00042h], ax                     ; a3 42 00
    call 0e79bh                               ; e8 0e 05
    call 0f13ch                               ; e8 ac 0e
    call 0f1c1h                               ; e8 2e 0f
    call 0e758h                               ; e8 c2 04
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    call 01b1fh                               ; e8 83 38
    call 01f1dh                               ; e8 7e 3c
    call 085f1h                               ; e8 4f a3
    call 0770eh                               ; e8 69 94
    call 0ed2fh                               ; e8 87 0a
    call 0e2d2h                               ; e8 27 00
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    call 01725h                               ; e8 74 34
    call 03587h                               ; e8 d3 52
    sti                                       ; fb
    int 019h                                  ; cd 19
    sti                                       ; fb
    hlt                                       ; f4
    jmp short 0e2b8h                          ; eb fd
    cli                                       ; fa
    hlt                                       ; f4
    add byte [bx+si], al                      ; 00 00
    add byte [bx+si], al                      ; 00 00
    pop ax                                    ; 58
    dec bp                                    ; 4d
nmi:                                         ; 0xfe2c3 LB 0x7
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    call 016e7h                               ; e8 1e 34
    iret                                      ; cf
int75_handler:                               ; 0xfe2ca LB 0x8
    out strict byte 0f0h, AL                  ; e6 f0
    call 0e03bh                               ; e8 6c fd
    int 002h                                  ; cd 02
    iret                                      ; cf
hard_drive_post:                             ; 0xfe2d2 LB 0x12c
    mov AL, strict byte 00ah                  ; b0 0a
    mov dx, 003f6h                            ; ba f6 03
    out DX, AL                                ; ee
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    mov ds, ax                                ; 8e d8
    mov byte [00474h], AL                     ; a2 74 04
    mov byte [00477h], AL                     ; a2 77 04
    mov byte [0048ch], AL                     ; a2 8c 04
    mov byte [0048dh], AL                     ; a2 8d 04
    mov byte [0048eh], AL                     ; a2 8e 04
    mov AL, strict byte 0c0h                  ; b0 c0
    mov byte [00476h], AL                     ; a2 76 04
    mov ax, 0e3feh                            ; b8 fe e3
    mov word [0004ch], ax                     ; a3 4c 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [0004eh], ax                     ; a3 4e 00
    mov ax, 0f8d2h                            ; b8 d2 f8
    mov word [001d8h], ax                     ; a3 d8 01
    mov ax, 0f000h                            ; b8 00 f0
    mov word [001dah], ax                     ; a3 da 01
    mov ax, strict word 0003dh                ; b8 3d 00
    mov word [00104h], ax                     ; a3 04 01
    mov ax, 09fc0h                            ; b8 c0 9f
    mov word [00106h], ax                     ; a3 06 01
    mov ax, strict word 0004dh                ; b8 4d 00
    mov word [00118h], ax                     ; a3 18 01
    mov ax, 09fc0h                            ; b8 c0 9f
    mov word [0011ah], ax                     ; a3 1a 01
    retn                                      ; c3
    times 0xdb db 0
    db  'XM'
int13_handler:                               ; 0xfe3fe LB 0x3
    jmp near 0ec5bh                           ; e9 5a 08
rom_fdpt:                                    ; 0xfe401 LB 0x2f1
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 058h
    db  04dh
int19_handler:                               ; 0xfe6f2 LB 0x61
    jmp near 0f0ach                           ; e9 b7 09
    or word [bx+si], ax                       ; 09 00
    cld                                       ; fc
    add byte [bx+di], al                      ; 00 01
    je short 0e73ch                           ; 74 40
    times 0x2b db 0
    pop ax                                    ; 58
    dec bp                                    ; 4d
    times 0xe db 0
    pop ax                                    ; 58
    dec bp                                    ; 4d
    push DS                                   ; 1e
    push ES                                   ; 06
    pushaw                                    ; 60
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    call 05baah                               ; e8 68 74
    popaw                                     ; 61
    pop ES                                    ; 07
    pop DS                                    ; 1f
    iret                                      ; cf
    push DS                                   ; 1e
    push ES                                   ; 06
    pushaw                                    ; 60
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    call 0167eh                               ; e8 2f 2f
    popaw                                     ; 61
    pop ES                                    ; 07
    pop DS                                    ; 1f
    iret                                      ; cf
rom_checksum:                                ; 0xfe753 LB 0x5
    push ax                                   ; 50
    db  032h, 0c0h
    ; xor al, al                                ; 32 c0
    pop ax                                    ; 58
    retn                                      ; c3
rom_scan:                                    ; 0xfe758 LB 0x43
    mov cx, 0c000h                            ; b9 00 c0
    mov ds, cx                                ; 8e d9
    mov ax, strict word 00004h                ; b8 04 00
    cmp word [word 00000h], 0aa55h            ; 81 3e 00 00 55 aa
    jne short 0e78bh                          ; 75 23
    call 0e753h                               ; e8 e8 ff
    jne short 0e78bh                          ; 75 1e
    mov AL, byte [00002h]                     ; a0 02 00
    test AL, strict byte 003h                 ; a8 03
    je short 0e778h                           ; 74 04
    and AL, strict byte 0fch                  ; 24 fc
    add AL, strict byte 004h                  ; 04 04
    db  033h, 0dbh
    ; xor bx, bx                                ; 33 db
    mov ds, bx                                ; 8e db
    push ax                                   ; 50
    push cx                                   ; 51
    push strict byte 00003h                   ; 6a 03
    db  08bh, 0ech
    ; mov bp, sp                                ; 8b ec
    call far [byte bp+000h]                   ; ff 5e 00
    cli                                       ; fa
    add sp, strict byte 00002h                ; 83 c4 02
    pop cx                                    ; 59
    pop ax                                    ; 58
    sal ax, 005h                              ; c1 e0 05
    db  003h, 0c8h
    ; add cx, ax                                ; 03 c8
    cmp cx, 0e800h                            ; 81 f9 00 e8
    jbe short 0e75bh                          ; 76 c5
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    mov ds, ax                                ; 8e d8
    retn                                      ; c3
init_pic:                                    ; 0xfe79b LB 0x25
    mov AL, strict byte 011h                  ; b0 11
    out strict byte 020h, AL                  ; e6 20
    out strict byte 0a0h, AL                  ; e6 a0
    mov AL, strict byte 008h                  ; b0 08
    out strict byte 021h, AL                  ; e6 21
    mov AL, strict byte 070h                  ; b0 70
    out strict byte 0a1h, AL                  ; e6 a1
    mov AL, strict byte 004h                  ; b0 04
    out strict byte 021h, AL                  ; e6 21
    mov AL, strict byte 002h                  ; b0 02
    out strict byte 0a1h, AL                  ; e6 a1
    mov AL, strict byte 001h                  ; b0 01
    out strict byte 021h, AL                  ; e6 21
    out strict byte 0a1h, AL                  ; e6 a1
    mov AL, strict byte 0b8h                  ; b0 b8
    out strict byte 021h, AL                  ; e6 21
    mov AL, strict byte 08fh                  ; b0 8f
    out strict byte 0a1h, AL                  ; e6 a1
    retn                                      ; c3
ebda_post:                                   ; 0xfe7c0 LB 0xa4
    mov ax, 0e746h                            ; b8 46 e7
    mov word [00034h], ax                     ; a3 34 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [00036h], ax                     ; a3 36 00
    mov ax, 0e746h                            ; b8 46 e7
    mov word [0003ch], ax                     ; a3 3c 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [0003eh], ax                     ; a3 3e 00
    mov ax, 0e746h                            ; b8 46 e7
    mov word [001c8h], ax                     ; a3 c8 01
    mov ax, 0f000h                            ; b8 00 f0
    mov word [001cah], ax                     ; a3 ca 01
    mov ax, 0e746h                            ; b8 46 e7
    mov word [001dch], ax                     ; a3 dc 01
    mov ax, 0f000h                            ; b8 00 f0
    mov word [001deh], ax                     ; a3 de 01
    mov ax, 09fc0h                            ; b8 c0 9f
    mov ds, ax                                ; 8e d8
    mov byte [word 00000h], 001h              ; c6 06 00 00 01
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    mov ds, ax                                ; 8e d8
    mov word [0040eh], 09fc0h                 ; c7 06 0e 04 c0 9f
    retn                                      ; c3
    times 0x27 db 0
    pop ax                                    ; 58
    dec bp                                    ; 4d
    sti                                       ; fb
    push ES                                   ; 06
    push DS                                   ; 1e
    pushaw                                    ; 60
    cmp ah, 000h                              ; 80 fc 00
    je short 0e846h                           ; 74 0f
    cmp ah, 010h                              ; 80 fc 10
    je short 0e846h                           ; 74 0a
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    call 04f92h                               ; e8 50 67
    popaw                                     ; 61
    pop DS                                    ; 1f
    pop ES                                    ; 07
    iret                                      ; cf
    mov bx, strict word 00040h                ; bb 40 00
    mov ds, bx                                ; 8e db
    cli                                       ; fa
    mov bx, word [word 0001ah]                ; 8b 1e 1a 00
    cmp bx, word [word 0001ch]                ; 3b 1e 1c 00
    jne short 0e85ah                          ; 75 04
    sti                                       ; fb
    nop                                       ; 90
    jmp short 0e84bh                          ; eb f1
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    call 04f92h                               ; e8 32 67
    popaw                                     ; 61
    pop DS                                    ; 1f
    pop ES                                    ; 07
    iret                                      ; cf
pmode_enter:                                 ; 0xfe864 LB 0x1b
    push CS                                   ; 0e
    pop DS                                    ; 1f
    lgdt [cs:0e892h]                          ; 2e 0f 01 16 92 e8
    mov eax, cr0                              ; 0f 20 c0
    or AL, strict byte 001h                   ; 0c 01
    mov cr0, eax                              ; 0f 22 c0
    jmp far 00020h:0e879h                     ; ea 79 e8 20 00
    mov ax, strict word 00018h                ; b8 18 00
    mov ds, ax                                ; 8e d8
    retn                                      ; c3
pmode_exit:                                  ; 0xfe87f LB 0x13
    mov ax, strict word 00040h                ; b8 40 00
    mov ds, ax                                ; 8e d8
    mov eax, cr0                              ; 0f 20 c0
    and AL, strict byte 0feh                  ; 24 fe
    mov cr0, eax                              ; 0f 22 c0
    jmp far 0f000h:0e891h                     ; ea 91 e8 00 f0
    retn                                      ; c3
pmbios_gdt_desc:                             ; 0xfe892 LB 0x6
    db  047h, 000h, 098h, 0e8h, 00fh, 000h
pmbios_gdt:                                  ; 0xfe898 LB 0x48
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  0ffh, 0ffh, 000h, 000h, 000h, 09bh, 0cfh, 000h, 0ffh, 0ffh, 000h, 000h, 000h, 093h, 0cfh, 000h
    db  0ffh, 0ffh, 000h, 000h, 00fh, 09bh, 000h, 000h, 0ffh, 0ffh, 000h, 000h, 000h, 093h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  0ffh, 0ffh, 000h, 004h, 000h, 093h, 000h, 000h
pmode_setup:                                 ; 0xfe8e0 LB 0x37b
    push eax                                  ; 66 50
    push esi                                  ; 66 56
    pushfw                                    ; 9c
    cli                                       ; fa
    call 0e864h                               ; e8 7b ff
    mov eax, cr0                              ; 0f 20 c0
    and eax, strict dword 09fffffffh          ; 66 25 ff ff ff 9f
    mov cr0, eax                              ; 0f 22 c0
    mov esi, strict dword 0fee00350h          ; 66 be 50 03 e0 fe
    mov eax, dword [esi]                      ; 67 66 8b 06
    and eax, strict dword 0fffe00ffh          ; 66 25 ff 00 fe ff
    or ah, 007h                               ; 80 cc 07
    mov dword [esi], eax                      ; 67 66 89 06
    mov esi, strict dword 0fee00360h          ; 66 be 60 03 e0 fe
    mov eax, dword [esi]                      ; 67 66 8b 06
    and eax, strict dword 0fffe00ffh          ; 66 25 ff 00 fe ff
    or ah, 004h                               ; 80 cc 04
    mov dword [esi], eax                      ; 67 66 89 06
    call 0e87fh                               ; e8 59 ff
    popfw                                     ; 9d
    pop esi                                   ; 66 5e
    pop eax                                   ; 66 58
    retn                                      ; c3
    times 0x59 db 0
    pop ax                                    ; 58
    dec bp                                    ; 4d
    cli                                       ; fa
    push ax                                   ; 50
    mov AL, strict byte 0adh                  ; b0 ad
    out strict byte 064h, AL                  ; e6 64
    mov AL, strict byte 00bh                  ; b0 0b
    out strict byte 020h, AL                  ; e6 20
    in AL, strict byte 020h                   ; e4 20
    and AL, strict byte 002h                  ; 24 02
    je short 0e9d6h                           ; 74 3f
    in AL, strict byte 060h                   ; e4 60
    push DS                                   ; 1e
    pushaw                                    ; 60
    cld                                       ; fc
    mov AH, strict byte 04fh                  ; b4 4f
    stc                                       ; f9
    int 015h                                  ; cd 15
    jnc short 0e9d0h                          ; 73 2d
    sti                                       ; fb
    cmp AL, strict byte 0e0h                  ; 3c e0
    jne short 0e9b6h                          ; 75 0e
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    mov ds, ax                                ; 8e d8
    mov AL, byte [00496h]                     ; a0 96 04
    or AL, strict byte 002h                   ; 0c 02
    mov byte [00496h], AL                     ; a2 96 04
    jmp short 0e9d0h                          ; eb 1a
    cmp AL, strict byte 0e1h                  ; 3c e1
    jne short 0e9c8h                          ; 75 0e
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    mov ds, ax                                ; 8e d8
    mov AL, byte [00496h]                     ; a0 96 04
    or AL, strict byte 001h                   ; 0c 01
    mov byte [00496h], AL                     ; a2 96 04
    jmp short 0e9d0h                          ; eb 08
    push ES                                   ; 06
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    call 04baah                               ; e8 db 61
    pop ES                                    ; 07
    popaw                                     ; 61
    pop DS                                    ; 1f
    cli                                       ; fa
    call 0e03fh                               ; e8 69 f6
    mov AL, strict byte 0aeh                  ; b0 ae
    out strict byte 064h, AL                  ; e6 64
    pop ax                                    ; 58
    iret                                      ; cf
    times 0x27b db 0
    pop ax                                    ; 58
    dec bp                                    ; 4d
    jmp short 0ecb0h                          ; eb 55
int13_relocated:                             ; 0xfec5b LB 0x55
    cmp ah, 04ah                              ; 80 fc 4a
    jc short 0ec71h                           ; 72 11
    cmp ah, 04dh                              ; 80 fc 4d
    jnbe short 0ec71h                         ; 77 0c
    pushaw                                    ; 60
    push ES                                   ; 06
    push DS                                   ; 1e
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    push 0ece9h                               ; 68 e9 ec
    jmp near 035c5h                           ; e9 54 49
    push ES                                   ; 06
    push ax                                   ; 50
    push bx                                   ; 53
    push cx                                   ; 51
    push dx                                   ; 52
    call 0359dh                               ; e8 24 49
    cmp AL, strict byte 000h                  ; 3c 00
    je short 0ecabh                           ; 74 2e
    call 035b1h                               ; e8 31 49
    pop dx                                    ; 5a
    push dx                                   ; 52
    db  03ah, 0c2h
    ; cmp al, dl                                ; 3a c2
    jne short 0ec97h                          ; 75 11
    pop dx                                    ; 5a
    pop cx                                    ; 59
    pop bx                                    ; 5b
    pop ax                                    ; 58
    pop ES                                    ; 07
    pushaw                                    ; 60
    push ES                                   ; 06
    push DS                                   ; 1e
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    push 0ece9h                               ; 68 e9 ec
    jmp near 03bach                           ; e9 15 4f
    and dl, 0e0h                              ; 80 e2 e0
    db  03ah, 0c2h
    ; cmp al, dl                                ; 3a c2
    jne short 0ecabh                          ; 75 0d
    pop dx                                    ; 5a
    pop cx                                    ; 59
    pop bx                                    ; 5b
    pop ax                                    ; 58
    pop ES                                    ; 07
    push ax                                   ; 50
    push cx                                   ; 51
    push dx                                   ; 52
    push bx                                   ; 53
    db  0feh, 0cah
    ; dec dl                                    ; fe ca
    jmp short 0ecb4h                          ; eb 09
    pop dx                                    ; 5a
    pop cx                                    ; 59
    pop bx                                    ; 5b
    pop ax                                    ; 58
    pop ES                                    ; 07
int13_noeltorito:                            ; 0xfecb0 LB 0x4
    push ax                                   ; 50
    push cx                                   ; 51
    push dx                                   ; 52
    push bx                                   ; 53
int13_legacy:                                ; 0xfecb4 LB 0x14
    push dx                                   ; 52
    push bp                                   ; 55
    push si                                   ; 56
    push di                                   ; 57
    push ES                                   ; 06
    push DS                                   ; 1e
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    test dl, 080h                             ; f6 c2 80
    jne short 0ecc8h                          ; 75 06
    push 0ece9h                               ; 68 e9 ec
    jmp near 02deeh                           ; e9 26 41
int13_notfloppy:                             ; 0xfecc8 LB 0x14
    cmp dl, 0e0h                              ; 80 fa e0
    jc short 0ecdch                           ; 72 0f
    shr ebx, 010h                             ; 66 c1 eb 10
    push bx                                   ; 53
    call 03fd5h                               ; e8 00 53
    pop bx                                    ; 5b
    sal ebx, 010h                             ; 66 c1 e3 10
    jmp short 0ece9h                          ; eb 0d
int13_disk:                                  ; 0xfecdc LB 0xd
    cmp ah, 040h                              ; 80 fc 40
    jnbe short 0ece6h                         ; 77 05
    call 052a6h                               ; e8 c2 65
    jmp short 0ece9h                          ; eb 03
    call 056e2h                               ; e8 f9 69
int13_out:                                   ; 0xfece9 LB 0x4
    pop DS                                    ; 1f
    pop ES                                    ; 07
    popaw                                     ; 61
    iret                                      ; cf
detect_parport:                              ; 0xfeced LB 0x1e
    push dx                                   ; 52
    inc dx                                    ; 42
    inc dx                                    ; 42
    in AL, DX                                 ; ec
    and AL, strict byte 0dfh                  ; 24 df
    out DX, AL                                ; ee
    pop dx                                    ; 5a
    mov AL, strict byte 0aah                  ; b0 aa
    out DX, AL                                ; ee
    in AL, DX                                 ; ec
    cmp AL, strict byte 0aah                  ; 3c aa
    jne short 0ed0ah                          ; 75 0d
    push bx                                   ; 53
    sal bx, 1                                 ; d1 e3
    mov word [bx+00408h], dx                  ; 89 97 08 04
    pop bx                                    ; 5b
    mov byte [bx+00478h], cl                  ; 88 8f 78 04
    inc bx                                    ; 43
    retn                                      ; c3
detect_serial:                               ; 0xfed0b LB 0x24
    push dx                                   ; 52
    inc dx                                    ; 42
    mov AL, strict byte 002h                  ; b0 02
    out DX, AL                                ; ee
    in AL, DX                                 ; ec
    cmp AL, strict byte 002h                  ; 3c 02
    jne short 0ed2dh                          ; 75 18
    inc dx                                    ; 42
    in AL, DX                                 ; ec
    cmp AL, strict byte 002h                  ; 3c 02
    jne short 0ed2dh                          ; 75 12
    dec dx                                    ; 4a
    db  032h, 0c0h
    ; xor al, al                                ; 32 c0
    pop dx                                    ; 5a
    push bx                                   ; 53
    sal bx, 1                                 ; d1 e3
    mov word [bx+00400h], dx                  ; 89 97 00 04
    pop bx                                    ; 5b
    mov byte [bx+0047ch], cl                  ; 88 8f 7c 04
    inc bx                                    ; 43
    retn                                      ; c3
    pop dx                                    ; 5a
    retn                                      ; c3
floppy_post:                                 ; 0xfed2f LB 0x87
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    mov ds, ax                                ; 8e d8
    mov AL, strict byte 000h                  ; b0 00
    mov byte [0043eh], AL                     ; a2 3e 04
    mov byte [0043fh], AL                     ; a2 3f 04
    mov byte [00440h], AL                     ; a2 40 04
    mov byte [00441h], AL                     ; a2 41 04
    mov byte [00442h], AL                     ; a2 42 04
    mov byte [00443h], AL                     ; a2 43 04
    mov byte [00444h], AL                     ; a2 44 04
    mov byte [00445h], AL                     ; a2 45 04
    mov byte [00446h], AL                     ; a2 46 04
    mov byte [00447h], AL                     ; a2 47 04
    mov byte [00448h], AL                     ; a2 48 04
    mov byte [0048bh], AL                     ; a2 8b 04
    mov AL, strict byte 010h                  ; b0 10
    out strict byte 070h, AL                  ; e6 70
    in AL, strict byte 071h                   ; e4 71
    db  08ah, 0e0h
    ; mov ah, al                                ; 8a e0
    shr al, 004h                              ; c0 e8 04
    je short 0ed6ah                           ; 74 04
    mov BL, strict byte 007h                  ; b3 07
    jmp short 0ed6ch                          ; eb 02
    mov BL, strict byte 000h                  ; b3 00
    db  08ah, 0c4h
    ; mov al, ah                                ; 8a c4
    and AL, strict byte 00fh                  ; 24 0f
    je short 0ed75h                           ; 74 03
    or bl, 070h                               ; 80 cb 70
    mov byte [0048fh], bl                     ; 88 1e 8f 04
    mov AL, strict byte 000h                  ; b0 00
    mov byte [00490h], AL                     ; a2 90 04
    mov byte [00491h], AL                     ; a2 91 04
    mov byte [00492h], AL                     ; a2 92 04
    mov byte [00493h], AL                     ; a2 93 04
    mov byte [00494h], AL                     ; a2 94 04
    mov byte [00495h], AL                     ; a2 95 04
    mov AL, strict byte 002h                  ; b0 02
    out strict byte 00ah, AL                  ; e6 0a
    mov ax, 0efc7h                            ; b8 c7 ef
    mov word [00078h], ax                     ; a3 78 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [0007ah], ax                     ; a3 7a 00
    mov ax, 0ec59h                            ; b8 59 ec
    mov word [00100h], ax                     ; a3 00 01
    mov ax, 0f000h                            ; b8 00 f0
    mov word [00102h], ax                     ; a3 02 01
    mov ax, 0ef57h                            ; b8 57 ef
    mov word [00038h], ax                     ; a3 38 00
    mov ax, 0f000h                            ; b8 00 f0
    mov word [0003ah], ax                     ; a3 3a 00
    retn                                      ; c3
bcd_to_bin:                                  ; 0xfedb6 LB 0x9
    sal ax, 004h                              ; c1 e0 04
    shr al, 004h                              ; c0 e8 04
    aad 00ah                                  ; d5 0a
    retn                                      ; c3
rtc_post:                                    ; 0xfedbf LB 0x198
    db  066h, 033h, 0c0h
    ; xor eax, eax                              ; 66 33 c0
    mov AL, strict byte 000h                  ; b0 00
    out strict byte 070h, AL                  ; e6 70
    in AL, strict byte 071h                   ; e4 71
    call 0edb6h                               ; e8 eb ff
    mov edx, strict dword 00115cf2bh          ; 66 ba 2b cf 15 01
    mul edx                                   ; 66 f7 e2
    mov ebx, strict dword 0000f4240h          ; 66 bb 40 42 0f 00
    db  066h, 033h, 0d2h
    ; xor edx, edx                              ; 66 33 d2
    div ebx                                   ; 66 f7 f3
    db  066h, 08bh, 0c8h
    ; mov ecx, eax                              ; 66 8b c8
    db  066h, 033h, 0c0h
    ; xor eax, eax                              ; 66 33 c0
    mov AL, strict byte 002h                  ; b0 02
    out strict byte 070h, AL                  ; e6 70
    in AL, strict byte 071h                   ; e4 71
    call 0edb6h                               ; e8 c7 ff
    mov edx, strict dword 000a6af80h          ; 66 ba 80 af a6 00
    mul edx                                   ; 66 f7 e2
    mov ebx, strict dword 000002710h          ; 66 bb 10 27 00 00
    db  066h, 033h, 0d2h
    ; xor edx, edx                              ; 66 33 d2
    div ebx                                   ; 66 f7 f3
    db  066h, 003h, 0c8h
    ; add ecx, eax                              ; 66 03 c8
    db  066h, 033h, 0c0h
    ; xor eax, eax                              ; 66 33 c0
    mov AL, strict byte 004h                  ; b0 04
    out strict byte 070h, AL                  ; e6 70
    in AL, strict byte 071h                   ; e4 71
    call 0edb6h                               ; e8 a3 ff
    mov edx, strict dword 003e81d03h          ; 66 ba 03 1d e8 03
    mul edx                                   ; 66 f7 e2
    mov ebx, strict dword 0000003e8h          ; 66 bb e8 03 00 00
    db  066h, 033h, 0d2h
    ; xor edx, edx                              ; 66 33 d2
    div ebx                                   ; 66 f7 f3
    db  066h, 003h, 0c8h
    ; add ecx, eax                              ; 66 03 c8
    mov dword [0046ch], ecx                   ; 66 89 0e 6c 04
    db  032h, 0c0h
    ; xor al, al                                ; 32 c0
    mov byte [00470h], AL                     ; a2 70 04
    retn                                      ; c3
    times 0x11f db 0
    db  'XM'
int0e_handler:                               ; 0xfef57 LB 0x70
    push ax                                   ; 50
    push dx                                   ; 52
    mov dx, 003f4h                            ; ba f4 03
    in AL, DX                                 ; ec
    and AL, strict byte 0c0h                  ; 24 c0
    cmp AL, strict byte 0c0h                  ; 3c c0
    je short 0ef81h                           ; 74 1e
    mov dx, 003f5h                            ; ba f5 03
    mov AL, strict byte 008h                  ; b0 08
    out DX, AL                                ; ee
    mov dx, 003f4h                            ; ba f4 03
    in AL, DX                                 ; ec
    and AL, strict byte 0c0h                  ; 24 c0
    cmp AL, strict byte 0c0h                  ; 3c c0
    jne short 0ef69h                          ; 75 f6
    mov dx, 003f5h                            ; ba f5 03
    in AL, DX                                 ; ec
    mov dx, 003f4h                            ; ba f4 03
    in AL, DX                                 ; ec
    and AL, strict byte 0c0h                  ; 24 c0
    cmp AL, strict byte 0c0h                  ; 3c c0
    je short 0ef73h                           ; 74 f2
    push DS                                   ; 1e
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    mov ds, ax                                ; 8e d8
    call 0e03fh                               ; e8 b6 f0
    or byte [0043eh], 080h                    ; 80 0e 3e 04 80
    pop DS                                    ; 1f
    pop dx                                    ; 5a
    pop ax                                    ; 58
    iret                                      ; cf
    times 0x33 db 0
    db  'XM'
_diskette_param_table:                       ; 0xfefc7 LB 0xd
    scasw                                     ; af
    add ah, byte [di]                         ; 02 25
    add dl, byte [bp+si]                      ; 02 12
    db  01bh, 0ffh
    ; sbb di, di                                ; 1b ff
    insb                                      ; 6c
    db  0f6h
    invd                                      ; 0f 08
    jmp short 0efd4h                          ; eb 00
int17_handler:                               ; 0xfefd4 LB 0xd
    push DS                                   ; 1e
    push ES                                   ; 06
    pushaw                                    ; 60
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    call 06d28h                               ; e8 4b 7d
    popaw                                     ; 61
    pop ES                                    ; 07
    pop DS                                    ; 1f
    iret                                      ; cf
_pmode_IDT:                                  ; 0xfefe1 LB 0x6
    db  000h, 000h, 000h, 000h, 00fh, 000h
_rmode_IDT:                                  ; 0xfefe7 LB 0x6
    db  0ffh, 003h, 000h, 000h, 000h, 000h
int1c_handler:                               ; 0xfefed LB 0x78
    iret                                      ; cf
    times 0x55 db 0
    pop ax                                    ; 58
    dec bp                                    ; 4d
    iret                                      ; cf
    times 0x1d db 0
    db  'XM'
int10_handler:                               ; 0xff065 LB 0x47
    iret                                      ; cf
    times 0x3c db 0
    pop ax                                    ; 58
    dec bp                                    ; 4d
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    call 016f7h                               ; e8 4d 26
    hlt                                       ; f4
    iret                                      ; cf
int19_relocated:                             ; 0xff0ac LB 0x90
    db  08bh, 0ech
    ; mov bp, sp                                ; 8b ec
    mov ax, word [bp+002h]                    ; 8b 46 02
    cmp ax, 0f000h                            ; 3d 00 f0
    je short 0f0c3h                           ; 74 0d
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    mov ds, ax                                ; 8e d8
    mov ax, 01234h                            ; b8 34 12
    mov word [001d8h], ax                     ; a3 d8 01
    jmp near 0e05bh                           ; e9 98 ef
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    push bp                                   ; 55
    db  08bh, 0ech
    ; mov bp, sp                                ; 8b ec
    mov ax, strict word 00001h                ; b8 01 00
    push ax                                   ; 50
    call 0463eh                               ; e8 6e 55
    inc sp                                    ; 44
    inc sp                                    ; 44
    test ax, ax                               ; 85 c0
    jne short 0f0feh                          ; 75 28
    mov ax, strict word 00002h                ; b8 02 00
    push ax                                   ; 50
    call 0463eh                               ; e8 61 55
    inc sp                                    ; 44
    inc sp                                    ; 44
    test ax, ax                               ; 85 c0
    jne short 0f0feh                          ; 75 1b
    mov ax, strict word 00003h                ; b8 03 00
    push strict byte 00003h                   ; 6a 03
    call 0463eh                               ; e8 53 55
    inc sp                                    ; 44
    inc sp                                    ; 44
    test ax, ax                               ; 85 c0
    jne short 0f0feh                          ; 75 0d
    mov ax, strict word 00004h                ; b8 04 00
    push ax                                   ; 50
    call 0463eh                               ; e8 46 55
    inc sp                                    ; 44
    inc sp                                    ; 44
    test ax, ax                               ; 85 c0
    je short 0f0a4h                           ; 74 a6
    sal eax, 004h                             ; 66 c1 e0 04
    mov word [bp+002h], ax                    ; 89 46 02
    shr eax, 004h                             ; 66 c1 e8 04
    and ax, 0f000h                            ; 25 00 f0
    mov word [bp+004h], ax                    ; 89 46 04
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    mov ds, ax                                ; 8e d8
    mov es, ax                                ; 8e c0
    mov word [byte bp+000h], ax               ; 89 46 00
    mov ax, 0aa55h                            ; b8 55 aa
    pop bp                                    ; 5d
    iret                                      ; cf
    or cx, word [bp+si]                       ; 0b 0a
    or word [di], ax                          ; 09 05
    push eax                                  ; 66 50
    mov eax, strict dword 000800000h          ; 66 b8 00 00 80 00
    db  08bh, 0c3h
    ; mov ax, bx                                ; 8b c3
    sal eax, 008h                             ; 66 c1 e0 08
    and dl, 0fch                              ; 80 e2 fc
    db  00ah, 0c2h
    ; or al, dl                                 ; 0a c2
    mov dx, 00cf8h                            ; ba f8 0c
    out DX, eax                               ; 66 ef
    pop eax                                   ; 66 58
    retn                                      ; c3
pcibios_init_iomem_bases:                    ; 0xff13c LB 0x16
    push bp                                   ; 55
    db  08bh, 0ech
    ; mov bp, sp                                ; 8b ec
    mov eax, strict dword 0e0000000h          ; 66 b8 00 00 00 e0
    push eax                                  ; 66 50
    mov ax, 0d000h                            ; b8 00 d0
    push ax                                   ; 50
    mov ax, strict word 00010h                ; b8 10 00
    push ax                                   ; 50
    mov bx, strict word 00008h                ; bb 08 00
pci_init_io_loop1:                           ; 0xff152 LB 0xe
    mov DL, strict byte 000h                  ; b2 00
    call 0f121h                               ; e8 ca ff
    mov dx, 00cfch                            ; ba fc 0c
    in ax, DX                                 ; ed
    cmp ax, strict byte 0ffffh                ; 83 f8 ff
    je short 0f199h                           ; 74 39
enable_iomem_space:                          ; 0xff160 LB 0x39
    mov DL, strict byte 004h                  ; b2 04
    call 0f121h                               ; e8 bc ff
    mov dx, 00cfch                            ; ba fc 0c
    in AL, DX                                 ; ec
    or AL, strict byte 007h                   ; 0c 07
    out DX, AL                                ; ee
    mov DL, strict byte 000h                  ; b2 00
    call 0f121h                               ; e8 b0 ff
    mov dx, 00cfch                            ; ba fc 0c
    in eax, DX                                ; 66 ed
    cmp eax, strict dword 020001022h          ; 66 3d 22 10 00 20
    jne short 0f199h                          ; 75 1b
    mov DL, strict byte 010h                  ; b2 10
    call 0f121h                               ; e8 9e ff
    mov dx, 00cfch                            ; ba fc 0c
    in ax, DX                                 ; ed
    and ax, strict byte 0fffch                ; 83 e0 fc
    db  08bh, 0c8h
    ; mov cx, ax                                ; 8b c8
    db  08bh, 0d1h
    ; mov dx, cx                                ; 8b d1
    add dx, strict byte 00014h                ; 83 c2 14
    in ax, DX                                 ; ed
    db  08bh, 0d1h
    ; mov dx, cx                                ; 8b d1
    add dx, strict byte 00018h                ; 83 c2 18
    in eax, DX                                ; 66 ed
next_pci_dev:                                ; 0xff199 LB 0xf
    mov byte [bp-008h], 010h                  ; c6 46 f8 10
    inc bx                                    ; 43
    cmp bx, 00100h                            ; 81 fb 00 01
    jne short 0f152h                          ; 75 ae
    db  08bh, 0e5h
    ; mov sp, bp                                ; 8b e5
    pop bp                                    ; 5d
    retn                                      ; c3
pcibios_init_set_elcr:                       ; 0xff1a8 LB 0xc
    push ax                                   ; 50
    push cx                                   ; 51
    mov dx, 004d0h                            ; ba d0 04
    test AL, strict byte 008h                 ; a8 08
    je short 0f1b4h                           ; 74 03
    inc dx                                    ; 42
    and AL, strict byte 007h                  ; 24 07
is_master_pic:                               ; 0xff1b4 LB 0xd
    db  08ah, 0c8h
    ; mov cl, al                                ; 8a c8
    mov BL, strict byte 001h                  ; b3 01
    sal bl, CL                                ; d2 e3
    in AL, DX                                 ; ec
    db  00ah, 0c3h
    ; or al, bl                                 ; 0a c3
    out DX, AL                                ; ee
    pop cx                                    ; 59
    pop ax                                    ; 58
    retn                                      ; c3
pcibios_init_irqs:                           ; 0xff1c1 LB 0x53
    push DS                                   ; 1e
    push bp                                   ; 55
    mov ax, 0f000h                            ; b8 00 f0
    mov ds, ax                                ; 8e d8
    mov dx, 004d0h                            ; ba d0 04
    mov AL, strict byte 000h                  ; b0 00
    out DX, AL                                ; ee
    inc dx                                    ; 42
    out DX, AL                                ; ee
    mov si, 0f2a0h                            ; be a0 f2
    mov bh, byte [si+008h]                    ; 8a 7c 08
    mov bl, byte [si+009h]                    ; 8a 5c 09
    mov DL, strict byte 000h                  ; b2 00
    call 0f121h                               ; e8 43 ff
    mov dx, 00cfch                            ; ba fc 0c
    in eax, DX                                ; 66 ed
    cmp eax, dword [si+00ch]                  ; 66 3b 44 0c
    jne near 0f291h                           ; 0f 85 a6 00
    mov dl, byte [si+022h]                    ; 8a 54 22
    call 0f121h                               ; e8 30 ff
    push bx                                   ; 53
    mov dx, 00cfch                            ; ba fc 0c
    mov ax, 08080h                            ; b8 80 80
    out DX, ax                                ; ef
    add dx, strict byte 00002h                ; 83 c2 02
    out DX, ax                                ; ef
    mov ax, word [si+006h]                    ; 8b 44 06
    sub ax, strict byte 00020h                ; 83 e8 20
    shr ax, 004h                              ; c1 e8 04
    db  08bh, 0c8h
    ; mov cx, ax                                ; 8b c8
    add si, strict byte 00020h                ; 83 c6 20
    db  08bh, 0ech
    ; mov bp, sp                                ; 8b ec
    mov ax, 0f11dh                            ; b8 1d f1
    push ax                                   ; 50
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    push ax                                   ; 50
pci_init_irq_loop1:                          ; 0xff214 LB 0x5
    mov bh, byte [si]                         ; 8a 3c
    mov bl, byte [si+001h]                    ; 8a 5c 01
pci_init_irq_loop2:                          ; 0xff219 LB 0x15
    mov DL, strict byte 000h                  ; b2 00
    call 0f121h                               ; e8 03 ff
    mov dx, 00cfch                            ; ba fc 0c
    in ax, DX                                 ; ed
    cmp ax, strict byte 0ffffh                ; 83 f8 ff
    jne short 0f22eh                          ; 75 07
    test bl, 007h                             ; f6 c3 07
    je short 0f285h                           ; 74 59
    jmp short 0f27bh                          ; eb 4d
pci_test_int_pin:                            ; 0xff22e LB 0x3c
    mov DL, strict byte 03ch                  ; b2 3c
    call 0f121h                               ; e8 ee fe
    mov dx, 00cfdh                            ; ba fd 0c
    in AL, DX                                 ; ec
    and AL, strict byte 007h                  ; 24 07
    je short 0f27bh                           ; 74 40
    db  0feh, 0c8h
    ; dec al                                    ; fe c8
    mov DL, strict byte 003h                  ; b2 03
    mul dl                                    ; f6 e2
    add AL, strict byte 002h                  ; 04 02
    db  032h, 0e4h
    ; xor ah, ah                                ; 32 e4
    db  08bh, 0d8h
    ; mov bx, ax                                ; 8b d8
    mov al, byte [bx+si]                      ; 8a 00
    db  08ah, 0d0h
    ; mov dl, al                                ; 8a d0
    mov bx, word [byte bp+000h]               ; 8b 5e 00
    call 0f121h                               ; e8 d0 fe
    mov dx, 00cfch                            ; ba fc 0c
    and AL, strict byte 003h                  ; 24 03
    db  002h, 0d0h
    ; add dl, al                                ; 02 d0
    in AL, DX                                 ; ec
    cmp AL, strict byte 080h                  ; 3c 80
    jc short 0f26ah                           ; 72 0d
    mov bx, word [bp-002h]                    ; 8b 5e fe
    mov al, byte [bx]                         ; 8a 07
    out DX, AL                                ; ee
    inc bx                                    ; 43
    mov word [bp-002h], bx                    ; 89 5e fe
    call 0f1a8h                               ; e8 3e ff
pirq_found:                                  ; 0xff26a LB 0x11
    mov bh, byte [si]                         ; 8a 3c
    mov bl, byte [si+001h]                    ; 8a 5c 01
    add bl, byte [bp-003h]                    ; 02 5e fd
    mov DL, strict byte 03ch                  ; b2 3c
    call 0f121h                               ; e8 aa fe
    mov dx, 00cfch                            ; ba fc 0c
    out DX, AL                                ; ee
next_pci_func:                               ; 0xff27b LB 0xa
    inc byte [bp-003h]                        ; fe 46 fd
    db  0feh, 0c3h
    ; inc bl                                    ; fe c3
    test bl, 007h                             ; f6 c3 07
    jne short 0f219h                          ; 75 94
next_pir_entry:                              ; 0xff285 LB 0xc
    add si, strict byte 00010h                ; 83 c6 10
    mov byte [bp-003h], 000h                  ; c6 46 fd 00
    loop 0f214h                               ; e2 86
    db  08bh, 0e5h
    ; mov sp, bp                                ; 8b e5
    pop bx                                    ; 5b
pci_init_end:                                ; 0xff291 LB 0x2f
    pop bp                                    ; 5d
    pop DS                                    ; 1f
    retn                                      ; c3
    db  089h, 0c0h, 089h, 0c0h, 089h, 0c0h, 089h, 0c0h, 089h, 0c0h, 089h, 0c0h, 024h, 050h, 049h, 052h
    db  000h, 001h, 000h, 002h, 000h, 008h, 000h, 000h, 086h, 080h, 000h, 070h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 031h
_pci_routing_table:                          ; 0xff2c0 LB 0x581
    db  000h, 008h, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 000h, 000h
    db  000h, 010h, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 001h, 000h
    db  000h, 018h, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 002h, 000h
    db  000h, 020h, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 003h, 000h
    db  000h, 028h, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 004h, 000h
    db  000h, 030h, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 005h, 000h
    db  000h, 038h, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 006h, 000h
    db  000h, 040h, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 007h, 000h
    db  000h, 048h, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 008h, 000h
    db  000h, 050h, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 009h, 000h
    db  000h, 058h, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 00ah, 000h
    db  000h, 060h, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 00bh, 000h
    db  000h, 068h, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 00ch, 000h
    db  000h, 070h, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 00dh, 000h
    db  000h, 078h, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 00eh, 000h
    db  000h, 080h, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 00fh, 000h
    db  000h, 088h, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 010h, 000h
    db  000h, 090h, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 011h, 000h
    db  000h, 098h, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 012h, 000h
    db  000h, 0a0h, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 013h, 000h
    db  000h, 0a8h, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 014h, 000h
    db  000h, 0b0h, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 015h, 000h
    db  000h, 0b8h, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 016h, 000h
    db  000h, 0c0h, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 017h, 000h
    db  000h, 0c8h, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 018h, 000h
    db  000h, 0d0h, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 019h, 000h
    db  000h, 0d8h, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 01ah, 000h
    db  000h, 0e0h, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 01bh, 000h
    db  000h, 0e8h, 060h, 0f8h, 0deh, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 01ch, 000h
    db  000h, 0f0h, 061h, 0f8h, 0deh, 062h, 0f8h, 0deh, 063h, 0f8h, 0deh, 060h, 0f8h, 0deh, 01dh, 000h
    db  0e0h, 001h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 058h
    db  04dh
int12_handler:                               ; 0xff841 LB 0xc
    sti                                       ; fb
    push DS                                   ; 1e
    mov ax, strict word 00040h                ; b8 40 00
    mov ds, ax                                ; 8e d8
    mov ax, word [00013h]                     ; a1 13 00
    pop DS                                    ; 1f
    iret                                      ; cf
int11_handler:                               ; 0xff84d LB 0xc
    sti                                       ; fb
    push DS                                   ; 1e
    mov ax, strict word 00040h                ; b8 40 00
    mov ds, ax                                ; 8e d8
    mov ax, word [00010h]                     ; a1 10 00
    pop DS                                    ; 1f
    iret                                      ; cf
int15_handler:                               ; 0xff859 LB 0x29
    pushfw                                    ; 9c
    push DS                                   ; 1e
    push ES                                   ; 06
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    cmp ah, 086h                              ; 80 fc 86
    je short 0f887h                           ; 74 23
    cmp ah, 0e8h                              ; 80 fc e8
    je short 0f887h                           ; 74 1e
    pushaw                                    ; 60
    cmp ah, 053h                              ; 80 fc 53
    je short 0f87dh                           ; 74 0e
    cmp ah, 0c2h                              ; 80 fc c2
    je short 0f882h                           ; 74 0e
    call 05db3h                               ; e8 3c 65
    popaw                                     ; 61
    pop ES                                    ; 07
    pop DS                                    ; 1f
    popfw                                     ; 9d
    jmp short 0f890h                          ; eb 13
    call 08720h                               ; e8 a0 8e
    jmp short 0f877h                          ; eb f5
int15_handler_mouse:                         ; 0xff882 LB 0x5
    call 069a2h                               ; e8 1d 71
    jmp short 0f877h                          ; eb f0
int15_handler32:                             ; 0xff887 LB 0x9
    pushad                                    ; 66 60
    call 0627ch                               ; e8 f0 69
    popad                                     ; 66 61
    jmp short 0f878h                          ; eb e8
iret_modify_cf:                              ; 0xff890 LB 0x14
    jc short 0f89bh                           ; 72 09
    push bp                                   ; 55
    db  08bh, 0ech
    ; mov bp, sp                                ; 8b ec
    and byte [bp+006h], 0feh                  ; 80 66 06 fe
    pop bp                                    ; 5d
    iret                                      ; cf
    push bp                                   ; 55
    db  08bh, 0ech
    ; mov bp, sp                                ; 8b ec
    or byte [bp+006h], 001h                   ; 80 4e 06 01
    pop bp                                    ; 5d
    iret                                      ; cf
int74_handler:                               ; 0xff8a4 LB 0x2e
    sti                                       ; fb
    pushaw                                    ; 60
    push ES                                   ; 06
    push DS                                   ; 1e
    push strict byte 00000h                   ; 6a 00
    push strict byte 00000h                   ; 6a 00
    push strict byte 00000h                   ; 6a 00
    push strict byte 00000h                   ; 6a 00
    push strict byte 00000h                   ; 6a 00
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    call 068dch                               ; e8 24 70
    pop cx                                    ; 59
    jcxz 0f8c7h                               ; e3 0c
    push strict byte 00000h                   ; 6a 00
    pop DS                                    ; 1f
    push word [0040eh]                        ; ff 36 0e 04
    pop DS                                    ; 1f
    call far [word 00022h]                    ; ff 1e 22 00
    cli                                       ; fa
    call 0e03bh                               ; e8 70 e7
    add sp, strict byte 00008h                ; 83 c4 08
    pop DS                                    ; 1f
    pop ES                                    ; 07
    popaw                                     ; 61
    iret                                      ; cf
int76_handler:                               ; 0xff8d2 LB 0x19c
    push ax                                   ; 50
    push DS                                   ; 1e
    mov ax, strict word 00040h                ; b8 40 00
    mov ds, ax                                ; 8e d8
    mov byte [0008eh], 0ffh                   ; c6 06 8e 00 ff
    call 0e03bh                               ; e8 5a e7
    pop DS                                    ; 1f
    pop ax                                    ; 58
    iret                                      ; cf
    times 0x188 db 0
    db  'XM'
font8x8:                                     ; 0xffa6e LB 0x421
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 07eh, 081h, 0a5h, 081h, 0bdh, 099h, 081h, 07eh
    db  07eh, 0ffh, 0dbh, 0ffh, 0c3h, 0e7h, 0ffh, 07eh, 06ch, 0feh, 0feh, 0feh, 07ch, 038h, 010h, 000h
    db  010h, 038h, 07ch, 0feh, 07ch, 038h, 010h, 000h, 038h, 07ch, 038h, 0feh, 0feh, 07ch, 038h, 07ch
    db  010h, 010h, 038h, 07ch, 0feh, 07ch, 038h, 07ch, 000h, 000h, 018h, 03ch, 03ch, 018h, 000h, 000h
    db  0ffh, 0ffh, 0e7h, 0c3h, 0c3h, 0e7h, 0ffh, 0ffh, 000h, 03ch, 066h, 042h, 042h, 066h, 03ch, 000h
    db  0ffh, 0c3h, 099h, 0bdh, 0bdh, 099h, 0c3h, 0ffh, 00fh, 007h, 00fh, 07dh, 0cch, 0cch, 0cch, 078h
    db  03ch, 066h, 066h, 066h, 03ch, 018h, 07eh, 018h, 03fh, 033h, 03fh, 030h, 030h, 070h, 0f0h, 0e0h
    db  07fh, 063h, 07fh, 063h, 063h, 067h, 0e6h, 0c0h, 099h, 05ah, 03ch, 0e7h, 0e7h, 03ch, 05ah, 099h
    db  080h, 0e0h, 0f8h, 0feh, 0f8h, 0e0h, 080h, 000h, 002h, 00eh, 03eh, 0feh, 03eh, 00eh, 002h, 000h
    db  018h, 03ch, 07eh, 018h, 018h, 07eh, 03ch, 018h, 066h, 066h, 066h, 066h, 066h, 000h, 066h, 000h
    db  07fh, 0dbh, 0dbh, 07bh, 01bh, 01bh, 01bh, 000h, 03eh, 063h, 038h, 06ch, 06ch, 038h, 0cch, 078h
    db  000h, 000h, 000h, 000h, 07eh, 07eh, 07eh, 000h, 018h, 03ch, 07eh, 018h, 07eh, 03ch, 018h, 0ffh
    db  018h, 03ch, 07eh, 018h, 018h, 018h, 018h, 000h, 018h, 018h, 018h, 018h, 07eh, 03ch, 018h, 000h
    db  000h, 018h, 00ch, 0feh, 00ch, 018h, 000h, 000h, 000h, 030h, 060h, 0feh, 060h, 030h, 000h, 000h
    db  000h, 000h, 0c0h, 0c0h, 0c0h, 0feh, 000h, 000h, 000h, 024h, 066h, 0ffh, 066h, 024h, 000h, 000h
    db  000h, 018h, 03ch, 07eh, 0ffh, 0ffh, 000h, 000h, 000h, 0ffh, 0ffh, 07eh, 03ch, 018h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 030h, 078h, 078h, 030h, 030h, 000h, 030h, 000h
    db  06ch, 06ch, 06ch, 000h, 000h, 000h, 000h, 000h, 06ch, 06ch, 0feh, 06ch, 0feh, 06ch, 06ch, 000h
    db  030h, 07ch, 0c0h, 078h, 00ch, 0f8h, 030h, 000h, 000h, 0c6h, 0cch, 018h, 030h, 066h, 0c6h, 000h
    db  038h, 06ch, 038h, 076h, 0dch, 0cch, 076h, 000h, 060h, 060h, 0c0h, 000h, 000h, 000h, 000h, 000h
    db  018h, 030h, 060h, 060h, 060h, 030h, 018h, 000h, 060h, 030h, 018h, 018h, 018h, 030h, 060h, 000h
    db  000h, 066h, 03ch, 0ffh, 03ch, 066h, 000h, 000h, 000h, 030h, 030h, 0fch, 030h, 030h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 030h, 030h, 060h, 000h, 000h, 000h, 0fch, 000h, 000h, 000h, 000h
    db  000h, 000h, 000h, 000h, 000h, 030h, 030h, 000h, 006h, 00ch, 018h, 030h, 060h, 0c0h, 080h, 000h
    db  07ch, 0c6h, 0ceh, 0deh, 0f6h, 0e6h, 07ch, 000h, 030h, 070h, 030h, 030h, 030h, 030h, 0fch, 000h
    db  078h, 0cch, 00ch, 038h, 060h, 0cch, 0fch, 000h, 078h, 0cch, 00ch, 038h, 00ch, 0cch, 078h, 000h
    db  01ch, 03ch, 06ch, 0cch, 0feh, 00ch, 01eh, 000h, 0fch, 0c0h, 0f8h, 00ch, 00ch, 0cch, 078h, 000h
    db  038h, 060h, 0c0h, 0f8h, 0cch, 0cch, 078h, 000h, 0fch, 0cch, 00ch, 018h, 030h, 030h, 030h, 000h
    db  078h, 0cch, 0cch, 078h, 0cch, 0cch, 078h, 000h, 078h, 0cch, 0cch, 07ch, 00ch, 018h, 070h, 000h
    db  000h, 030h, 030h, 000h, 000h, 030h, 030h, 000h, 000h, 030h, 030h, 000h, 000h, 030h, 030h, 060h
    db  018h, 030h, 060h, 0c0h, 060h, 030h, 018h, 000h, 000h, 000h, 0fch, 000h, 000h, 0fch, 000h, 000h
    db  060h, 030h, 018h, 00ch, 018h, 030h, 060h, 000h, 078h, 0cch, 00ch, 018h, 030h, 000h, 030h, 000h
    db  07ch, 0c6h, 0deh, 0deh, 0deh, 0c0h, 078h, 000h, 030h, 078h, 0cch, 0cch, 0fch, 0cch, 0cch, 000h
    db  0fch, 066h, 066h, 07ch, 066h, 066h, 0fch, 000h, 03ch, 066h, 0c0h, 0c0h, 0c0h, 066h, 03ch, 000h
    db  0f8h, 06ch, 066h, 066h, 066h, 06ch, 0f8h, 000h, 0feh, 062h, 068h, 078h, 068h, 062h, 0feh, 000h
    db  0feh, 062h, 068h, 078h, 068h, 060h, 0f0h, 000h, 03ch, 066h, 0c0h, 0c0h, 0ceh, 066h, 03eh, 000h
    db  0cch, 0cch, 0cch, 0fch, 0cch, 0cch, 0cch, 000h, 078h, 030h, 030h, 030h, 030h, 030h, 078h, 000h
    db  01eh, 00ch, 00ch, 00ch, 0cch, 0cch, 078h, 000h, 0e6h, 066h, 06ch, 078h, 06ch, 066h, 0e6h, 000h
    db  0f0h, 060h, 060h, 060h, 062h, 066h, 0feh, 000h, 0c6h, 0eeh, 0feh, 0feh, 0d6h, 0c6h, 0c6h, 000h
    db  0c6h, 0e6h, 0f6h, 0deh, 0ceh, 0c6h, 0c6h, 000h, 038h, 06ch, 0c6h, 0c6h, 0c6h, 06ch, 038h, 000h
    db  0fch, 066h, 066h, 07ch, 060h, 060h, 0f0h, 000h, 078h, 0cch, 0cch, 0cch, 0dch, 078h, 01ch, 000h
    db  0fch, 066h, 066h, 07ch, 06ch, 066h, 0e6h, 000h, 078h, 0cch, 0e0h, 070h, 01ch, 0cch, 078h, 000h
    db  0fch, 0b4h, 030h, 030h, 030h, 030h, 078h, 000h, 0cch, 0cch, 0cch, 0cch, 0cch, 0cch, 0fch, 000h
    db  0cch, 0cch, 0cch, 0cch, 0cch, 078h, 030h, 000h, 0c6h, 0c6h, 0c6h, 0d6h, 0feh, 0eeh, 0c6h, 000h
    db  0c6h, 0c6h, 06ch, 038h, 038h, 06ch, 0c6h, 000h, 0cch, 0cch, 0cch, 078h, 030h, 030h, 078h, 000h
    db  0feh, 0c6h, 08ch, 018h, 032h, 066h, 0feh, 000h, 078h, 060h, 060h, 060h, 060h, 060h, 078h, 000h
    db  0c0h, 060h, 030h, 018h, 00ch, 006h, 002h, 000h, 078h, 018h, 018h, 018h, 018h, 018h, 078h, 000h
    db  010h, 038h, 06ch, 0c6h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 0ffh
    db  030h, 030h, 018h, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 078h, 00ch, 07ch, 0cch, 076h, 000h
    db  0e0h, 060h, 060h, 07ch, 066h, 066h, 0dch, 000h, 000h, 000h, 078h, 0cch, 0c0h, 0cch, 078h, 000h
    db  01ch, 00ch, 00ch, 07ch, 0cch, 0cch, 076h, 000h, 000h, 000h, 078h, 0cch, 0fch, 0c0h, 078h, 000h
    db  038h, 06ch, 060h, 0f0h, 060h, 060h, 0f0h, 000h, 000h, 000h, 076h, 0cch, 0cch, 07ch, 00ch, 0f8h
    db  0e0h, 060h, 06ch, 076h, 066h, 066h, 0e6h, 000h, 030h, 000h, 070h, 030h, 030h, 030h, 078h, 000h
    db  00ch, 000h, 00ch, 00ch, 00ch, 0cch, 0cch, 078h, 0e0h, 060h, 066h, 06ch, 078h, 06ch, 0e6h, 000h
    db  070h, 030h, 030h, 030h, 030h, 030h, 078h, 000h, 000h, 000h, 0cch, 0feh, 0feh, 0d6h, 0c6h, 000h
    db  000h, 000h, 0f8h, 0cch, 0cch, 0cch, 0cch, 000h, 000h, 000h, 078h, 0cch, 0cch, 0cch, 078h, 000h
    db  000h, 000h, 0dch, 066h, 066h, 07ch, 060h, 0f0h, 000h, 000h, 076h, 0cch, 0cch, 07ch, 00ch, 01eh
    db  000h, 000h, 0dch, 076h, 066h, 060h, 0f0h, 000h, 000h, 000h, 07ch, 0c0h, 078h, 00ch, 0f8h, 000h
    db  010h, 030h, 07ch, 030h, 030h, 034h, 018h, 000h, 000h, 000h, 0cch, 0cch, 0cch, 0cch, 076h, 000h
    db  000h, 000h, 0cch, 0cch, 0cch, 078h, 030h, 000h, 000h, 000h, 0c6h, 0d6h, 0feh, 0feh, 06ch, 000h
    db  000h, 000h, 0c6h, 06ch, 038h, 06ch, 0c6h, 000h, 000h, 000h, 0cch, 0cch, 0cch, 07ch, 00ch, 0f8h
    db  000h, 000h, 0fch, 098h, 030h, 064h, 0fch, 000h, 01ch, 030h, 030h, 0e0h, 030h, 030h, 01ch, 000h
    db  018h, 018h, 018h, 000h, 018h, 018h, 018h, 000h, 0e0h, 030h, 030h, 01ch, 030h, 030h, 0e0h, 000h
    db  076h, 0dch, 000h, 000h, 000h, 000h, 000h, 000h, 000h, 010h, 038h, 06ch, 0c6h, 0c6h, 0feh, 000h
    db  080h, 0fch, 0b1h, 075h, 00fh, 006h, 01eh, 00eh, 01fh, 0fch, 066h, 060h, 0e8h, 0abh, 08ah, 066h
    db  061h, 01fh, 007h, 0cfh, 006h, 01eh, 060h, 00eh, 01fh, 0fch, 0e8h, 0f9h, 067h, 061h, 01fh, 007h
    db  0cfh
int70_handler:                               ; 0xffe8f LB 0x16
    push ES                                   ; 06
    push DS                                   ; 1e
    pushaw                                    ; 60
    push CS                                   ; 0e
    pop DS                                    ; 1f
    cld                                       ; fc
    call 065c9h                               ; e8 31 67
    popaw                                     ; 61
    pop DS                                    ; 1f
    pop ES                                    ; 07
    iret                                      ; cf
    add byte [bx+si], al                      ; 00 00
    add byte [bx+si], al                      ; 00 00
    add byte [bx+si], al                      ; 00 00
    add byte [bx+si+04dh], bl                 ; 00 58 4d
int08_handler:                               ; 0xffea5 LB 0xae
    sti                                       ; fb
    push eax                                  ; 66 50
    push DS                                   ; 1e
    db  033h, 0c0h
    ; xor ax, ax                                ; 33 c0
    mov ds, ax                                ; 8e d8
    mov AL, byte [00440h]                     ; a0 40 04
    db  00ah, 0c0h
    ; or al, al                                 ; 0a c0
    je short 0febdh                           ; 74 09
    push dx                                   ; 52
    mov dx, 003f2h                            ; ba f2 03
    in AL, DX                                 ; ec
    and AL, strict byte 0cfh                  ; 24 cf
    out DX, AL                                ; ee
    pop dx                                    ; 5a
    mov eax, dword [0046ch]                   ; 66 a1 6c 04
    inc eax                                   ; 66 40
    cmp eax, strict dword 0001800b0h          ; 66 3d b0 00 18 00
    jc short 0fed2h                           ; 72 07
    db  066h, 033h, 0c0h
    ; xor eax, eax                              ; 66 33 c0
    inc byte [00470h]                         ; fe 06 70 04
    mov dword [0046ch], eax                   ; 66 a3 6c 04
    int 01ch                                  ; cd 1c
    cli                                       ; fa
    call 0e03fh                               ; e8 63 e1
    pop DS                                    ; 1f
    pop eax                                   ; 66 58
    iret                                      ; cf
    times 0x11 db 0
    pop ax                                    ; 58
    dec bp                                    ; 4d
    times 0xb db 0
    pop ax                                    ; 58
    dec bp                                    ; 4d
    dec di                                    ; 4f
    jc short 0ff64h                           ; 72 61
    arpl word [si+065h], bp                   ; 63 6c 65
    and byte [bp+04dh], dl                    ; 20 56 4d
    and byte [bp+069h], dl                    ; 20 56 69
    jc short 0ff82h                           ; 72 74
    jne short 0ff71h                          ; 75 61
    insb                                      ; 6c
    inc dx                                    ; 42
    outsw                                     ; 6f
    js short 0ff35h                           ; 78 20
    inc dx                                    ; 42
    dec cx                                    ; 49
    dec di                                    ; 4f
    push bx                                   ; 53
    times 0x38 db 0
    db  'XM'
dummy_iret:                                  ; 0xfff53 LB 0x9d
    iret                                      ; cf
    iret                                      ; cf
    mov ax, ax                                ; 89 c0
    mov ax, ax                                ; 89 c0
    mov ax, ax                                ; 89 c0
    mov ax, ax                                ; 89 c0
    mov ax, ax                                ; 89 c0
    cld                                       ; fc
    pop di                                    ; 5f
    push bx                                   ; 53
    dec bp                                    ; 4d
    pop di                                    ; 5f
    jnl short 0ff85h                          ; 7d 1f
    add al, byte [di]                         ; 02 05
    inc word [bx+si]                          ; ff 00
    add byte [bx+si], al                      ; 00 00
    add byte [bx+si], al                      ; 00 00
    add byte [bx+si], al                      ; 00 00
    pop di                                    ; 5f
    inc sp                                    ; 44
    dec bp                                    ; 4d
    dec cx                                    ; 49
    pop di                                    ; 5f
    sbb AL, strict byte 000h                  ; 1c 00
    add byte [bx+si], al                      ; 00 00
    adc byte [00900h], cl                     ; 10 0e 00 09
    add byte [di], ah                         ; 00 25
    times 0x6f db 0
    db  'XM'
cpu_reset:                                   ; 0xffff0 LB 0x10
    jmp far 0f000h:0e05bh                     ; ea 5b e0 00 f0
    db  030h, 036h, 02fh, 032h, 033h, 02fh, 039h, 039h, 000h, 0fch, 092h
