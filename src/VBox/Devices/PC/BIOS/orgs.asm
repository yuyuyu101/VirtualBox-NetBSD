;;
;; Copyright (C) 2006-2011 Oracle Corporation
;;
;; This file is part of VirtualBox Open Source Edition (OSE), as
;; available from http://www.virtualbox.org. This file is free software;
;; you can redistribute it and/or modify it under the terms of the GNU
;; General Public License (GPL) as published by the Free Software
;; Foundation, in version 2 as it comes in the "COPYING" file of the
;; VirtualBox OSE distribution. VirtualBox OSE is distributed in the
;; hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
;; --------------------------------------------------------------------
;;
;; This code is based on:
;;
;;  ROM BIOS for use with Bochs/Plex86/QEMU emulation environment
;;
;;  Copyright (C) 2002  MandrakeSoft S.A.
;;
;;    MandrakeSoft S.A.
;;    43, rue d'Aboukir
;;    75002 Paris - France
;;    http://www.linux-mandrake.com/
;;    http://www.mandrakesoft.com/
;;
;;  This library is free software; you can redistribute it and/or
;;  modify it under the terms of the GNU Lesser General Public
;;  License as published by the Free Software Foundation; either
;;  version 2 of the License, or (at your option) any later version.
;;
;;  This library is distributed in the hope that it will be useful,
;;  but WITHOUT ANY WARRANTY; without even the implied warranty of
;;  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;;  Lesser General Public License for more details.
;;
;;  You should have received a copy of the GNU Lesser General Public
;;  License along with this library; if not, write to the Free Software
;;  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
;;
;;


; Oracle LGPL Disclaimer: For the avoidance of doubt, except that if any license choice
; other than GPL or LGPL is available it will apply instead, Oracle elects to use only
; the Lesser General Public License version 2.1 (LGPLv2) at this time for any software where
; a choice of LGPL license versions is made available with the language indicating
; that LGPLv2 or any later version may be used, or where a choice of which version
; of the LGPL is applied is otherwise unspecified.

EBDA_SEG	equ	09FC0h		; starts at 639K
EBDA_SIZE	equ	1		; 1K
BASE_MEM_IN_K	equ	(640 - EBDA_SIZE)

CMOS_ADDR	equ	070h
CMOS_DATA	equ	071h


PIC_CMD_EOI	equ	020h
PIC_MASTER	equ	020h
PIC_SLAVE	equ	0A0h

BIOS_FIX_BASE	equ	0E000h

SYS_MODEL_ID	equ	0FCh		; PC/AT
SYS_SUBMODEL_ID	equ	0
BIOS_REVISION	equ	1

BIOS_BUILD_DATE	equ	'06/23/99'
BIOS_COPYRIGHT	equ	'Oracle VM VirtualBox BIOS'

BX_ROMBIOS32		equ	0
BX_CALL_INT15_4F	equ	1

;; Set a fixed BIOS location, with a marker for verification
BIOSORG		macro	addr
		org	addr - BIOS_FIX_BASE - 2
		db	'XM'
		endm

;; Set an interrupt vector (not very efficient if multiple vectors are
;; programmed in one go)
SET_INT_VECTOR	macro	vec, segm, offs
		mov	ax, offs
		mov	ds:[vec*4], ax
		mov	ax, segm
		mov	ds:[vec*4+2], ax
endm

; Set up an environment C code expects. DS must point to the BIOS segment
; and the direction flag must be cleared(!)
C_SETUP		macro
		push	cs
		pop	ds
		cld
endm

;; External function in separate modules
extrn		_dummy_isr_function:near
extrn		_log_bios_start:near
extrn		_nmi_handler_msg:near
extrn		_int18_panic_msg:near
extrn		_int09_function:near
extrn		_int13_diskette_function:near
extrn		_int13_eltorito:near
extrn		_int13_cdemu:near
extrn		_int13_cdrom:near
extrn		_cdemu_isactive:near
extrn		_cdemu_emulated_drive:near
extrn		_int13_harddisk:near
extrn		_int13_harddisk_ext:near
extrn		_int14_function:near
extrn		_int15_function:near
extrn		_int15_function_mouse:near
extrn		_int15_function32:near
extrn		_int16_function:near
extrn		_int17_function:near
extrn		_int19_function:near
extrn		_int1a_function:near
extrn		_pci16_function:near
extrn		_int70_function:near
extrn		_int74_function:near
extrn		_apm_function:near
extrn		_ata_init:near
extrn		_ahci_init:near
extrn		_scsi_init:near
extrn		_ata_detect:near
extrn		_cdemu_init:near
extrn		_keyboard_init:near
extrn		_print_bios_banner:near


;; Symbols referenced from C code
public		_diskette_param_table
public		_pmode_IDT
public		_rmode_IDT
public		post
public		eoi_both_pics
public		rtc_post

;; Additional publics for easier disassembly and debugging
ifndef DEBUG
 DEBUG	equ	1
endif
ifdef		DEBUG

public		int08_handler
public		int0e_handler
public		int11_handler
public		int12_handler
public		int13_handler
public		int13_relocated
public		int15_handler
public		int17_handler
public		int19_handler
public		int19_relocated
public		dummy_iret
public		nmi
public		rom_fdpt
public		cpu_reset
public		normal_post
public		eoi_jmp_post
public		eoi_master_pic
public		ebda_post
public		hard_drive_post
public		int13_legacy
public		int70_handler
public		int75_handler
public		int15_handler32
public		int15_handler_mouse
public		iret_modify_cf
public		rom_scan
public		rom_checksum
public		init_pic
public		floppy_post
public		int13_out
public		int13_disk
public		int13_notfloppy
public		int13_legacy
public		int13_noeltorito
public		int1c_handler
public		int10_handler
public		int74_handler
public		int76_handler
public		detect_parport
public		detect_serial
public		font8x8

endif

;; NOTE: The last 8K of the ROM BIOS are peppered with fixed locations which
;; must be retained for compatibility. As a consequence, some of the space is
;; going to be wasted, but the gaps should be filled with miscellaneous code
;; and data when possible.

.286p

BIOSSEG		segment	'CODE'
		assume	cs:BIOSSEG

;;
;; Start of fixed code - eoi_jmp_post is kept here to allow short jumps.
;;
		BIOSORG	0E030h
eoi_jmp_post:
		call	eoi_both_pics
		xor	ax, ax
		mov	ds, ax
		jmp	dword ptr ds:[0467h]

eoi_both_pics:
		mov	al, PIC_CMD_EOI
		out	PIC_SLAVE, al
eoi_master_pic:
		mov	al, PIC_CMD_EOI
		out	PIC_MASTER, al
		ret

;; --------------------------------------------------------
;; POST entry point
;; --------------------------------------------------------
		BIOSORG	0E05Bh
post:
		xor	ax, ax

		;; reset the DMA controllers
		out	00Dh, al
		out	0DAh, al

		;; then initialize the DMA controllers
		mov	al, 0C0h
		out	0D6h, al	; enable channel 4 cascade
		mov	al, 0
		out	0D4h, al	; unmask channel 4

		;; read the CMOS shutdown status
		mov	al, 0Fh
		out	CMOS_ADDR, al
		in	al, CMOS_DATA

		;; save status
		mov	bl, al

		;; reset the shutdown status in CMOS
		mov	al, 0Fh
		out	CMOS_ADDR, al
		mov	al, 0
		out	CMOS_DATA, al

		;; examine the shutdown status code
		mov	al, bl
		cmp	al, 0
		jz	normal_post
		cmp	al, 0Dh
		jae	normal_post
		cmp	al, 9
		je	normal_post	;; TODO: really?!

		;; 05h = EOI + jump through 40:67
		cmp	al, 5
		je	eoi_jmp_post

		;; any other shutdown status values are ignored
		;; OpenSolaris sets the status to 0Ah in some cases?
		jmp	normal_post


		;; routine to write the pointer in DX:AX to memory starting
		;; at DS:BX (repeat CX times)
		;; - modifies BX, CX
set_int_vects	proc	near

		mov	[bx], ax
		mov	[bx+2], dx
		add	bx, 4
		loop	set_int_vects
		ret

set_int_vects	endp

normal_post:
		;; shutdown code 0: normal startup
		cli
		;; Set up the stack top at 0:7800h. The stack should not be
		;; located above 0:7C00h; that conflicts with PXE, which
		;; considers anything above that address to be fair game.
		;; The traditional locations are 30:100 (PC) or 0:400 (PC/AT).
		mov	ax, 7800h
		mov	sp, ax
		xor	ax, ax
		mov	ds, ax
		mov	ss, ax

		;; clear the bottom of memory except for the word at 40:72
		;; TODO:  Why not clear all of it? What's the point?
		mov	es, ax
		xor	di, di
		cld
		mov	cx, 0472h / 2
	rep	stosw
		inc	di
		inc	di
		mov	cx, (1000h - 0472h - 2) / 2
	rep	stosw

		;; clear the remaining base memory except for the top
		;; of the EBDA (the MP table is planted there)
		xor	bx, bx
memory_zero_loop:
		add	bx, 1000h
		cmp	bx, 9000h
		jae	memory_cleared
		mov	es, bx
		xor	di, di
		mov	cx, 8000h	; 32K words
	rep	stosw
		jmp	memory_zero_loop
memory_cleared:
		mov	es, bx
		xor	di, di
		mov	cx, 7FF8h	; all but the last 16 bytes
	rep	stosw
		xor	bx, bx


		C_SETUP
		call	_log_bios_start

		call	pmode_setup

		;; set all interrupts in 00h-5Fh range to default handler
		xor	bx, bx
		mov	ds, bx
		mov	cx, 60h		; leave the rest as zeros
		mov	ax, dummy_iret
		mov	dx, BIOSSEG
		call	set_int_vects

		;; also set 68h-77h to default handler; note that the
		;; 60h-67h range must contain zeros for certain programs
		;; to function correctly
		mov	bx, 68h * 4
		mov	cx, 10h
		call	set_int_vects
		
		;; base memory in K to 40:13
		mov	ax, BASE_MEM_IN_K
		mov	ds:[413h], ax

		;; manufacturing test at 40:12
		;; zeroed out above

		;; set up various service vectors
		;; TODO: This should use the table at FEF3h instead
		SET_INT_VECTOR 11h, BIOSSEG, int11_handler
		SET_INT_VECTOR 12h, BIOSSEG, int12_handler
		SET_INT_VECTOR 15h, BIOSSEG, int15_handler
		SET_INT_VECTOR 17h, BIOSSEG, int17_handler
		SET_INT_VECTOR 18h, BIOSSEG, int18_handler
		SET_INT_VECTOR 19h, BIOSSEG, int19_handler
		SET_INT_VECTOR 1Ch, BIOSSEG, int1c_handler

		call	ebda_post

		;; PIT setup
		SET_INT_VECTOR 08h, BIOSSEG, int08_handler
		mov	al, 34h		; timer 0, binary, 16-bit, mode 2
		out	43h, al
		mov	al, 0		; max count -> ~18.2 Hz
		out	40h, al
		out	40h, al

		;; keyboard setup
		SET_INT_VECTOR 09h, BIOSSEG, int09_handler
		SET_INT_VECTOR 16h, BIOSSEG, int16_handler

		xor	ax, ax
		mov	ds, ax
		;; TODO: What's the point? The BDA is zeroed already?!
		mov	ds:[417h], al	; keyboard shift flags, set 1
		mov	ds:[418h], al	; keyboard shift flags, set 2
		mov	ds:[419h], al	; keyboard Alt-numpad work area
		mov	ds:[471h], al	; keyboard Ctrl-Break flag
		mov	ds:[497h], al	; keyboard status flags 4
		mov	al, 10h
		mov	ds:[496h], al	; keyboard status flags 3

		mov	bx, 1Eh
		mov	ds:[41Ah], bx	; keyboard buffer head
		mov	ds:[41Ch], bx	; keyboard buffer tail
		mov	ds:[480h], bx	; keyboard buffer start
		mov	bx, 3Eh
		mov	ds:[482h], bx	; keyboard buffer end

		push	ds
		C_SETUP
		call	_keyboard_init
		pop	ds


		;; store CMOS equipment byte in BDA
		mov	al, 14h
		out	CMOS_ADDR, al
		in	al, CMOS_DATA
		mov	ds:[410h], al

		;; parallel setup
		SET_INT_VECTOR 0Fh, BIOSSEG, dummy_iret
		xor	ax, ax
		mov	ds, ax
		xor	bx, bx
		mov	cl, 14h		; timeout value
		mov	dx, 378h	; parallel port 1
		call	detect_parport
		mov	dx, 278h	; parallel port 2
		call	detect_parport
		shl	bx, 0Eh
		mov	ax, ds:[410h]	; equipment word
		and	ax, 3FFFh
		or	ax, bx		; set number of parallel ports
		mov	ds:[410h], ax	; store in BDA

		;; Serial setup
		SET_INT_VECTOR 0Bh, BIOSSEG, dummy_isr
		SET_INT_VECTOR 0Ch, BIOSSEG, dummy_isr
		SET_INT_VECTOR 14h, BIOSSEG, int14_handler
		xor	bx, bx
		mov	cl, 0Ah		; timeout value
		mov	dx, 3F8h	; first serial address
		call	detect_serial
		mov	dx, 2F8h	; second serial address
		call	detect_serial
		mov	dx, 3E8h	; third serial address
		call	detect_serial
		mov	dx, 2E8h	; fourth serial address
		call	detect_serial
		shl	bx, 9
		mov	ax, ds:[410h]	; equipment word
		and	ax, 0F1FFh	; bits 9-11 determine serial ports
		or	ax, bx
		mov	ds:[410h], ax

		;; CMOS RTC
		SET_INT_VECTOR 1Ah, BIOSSEG, int1a_handler
		SET_INT_VECTOR 4Ah, BIOSSEG, dummy_iret	; TODO: redundant?
		SET_INT_VECTOR 70h, BIOSSEG, int70_handler
		;; BIOS DATA AREA 4CEh ???
		call	rtc_post

		;; PS/2 mouse setup
		SET_INT_VECTOR 74h, BIOSSEG, int74_handler

		;; IRQ 13h (FPU exception) setup
		SET_INT_VECTOR 75h, BIOSSEG, int75_handler

		;; Video setup
		SET_INT_VECTOR 10h, BIOSSEG, int10_handler

		call	init_pic

		call	pcibios_init_iomem_bases
		call	pcibios_init_irqs

		call	rom_scan

		C_SETUP
		;; ATA/ATAPI driver setup
		call	_ata_init
		call	_ata_detect

ifdef VBOX_WITH_AHCI
		; AHCI driver setup
		call	_ahci_init
endif

ifdef VBOX_WITH_SCSI
		; SCSI driver setup
		call	_scsi_init
endif

		;; floppy setup
		call	floppy_post

		;; hard drive setup
		call	hard_drive_post

		C_SETUP			; in case assembly code changed things
		call	_print_bios_banner

		;; El Torito floppy/hard disk emulation
		call	_cdemu_init

		; TODO: what's the point of enabling interrupts here??
		sti			; enable interrupts
		int	19h
		;; does not return here
		sti
wait_forever:
		hlt
		jmp	wait_forever
		cli
		hlt


;; --------------------------------------------------------
;; NMI handler
;; --------------------------------------------------------
		BIOSORG	0E2C3h
nmi:
		C_SETUP
		call	_nmi_handler_msg
		iret

int75_handler:
		out	0F0h, al	; clear IRQ13
		call	eoi_both_pics
		int	2		; emulate legacy NMI
		iret


hard_drive_post	proc	near

		;; TODO Why? And what about secondary controllers?
		mov	al, 0Ah		; disable IRQ 14
		mov	dx, 03F6h
		out	dx, al

		xor	ax, ax
		mov	ds, ax
		;; TODO: Didn't we just clear the entire EBDA?
		mov	ds:[474h], al	; last HD operation status
		mov	ds:[477h], al	; HD port offset (XT only???)
		mov	ds:[48Ch], al	; HD status register
		mov	ds:[48Dh], al	; HD error register
		mov	ds:[48Eh], al	; HD task complete flag
		mov	al, 0C0h
		mov	ds:[476h], al	; HD control byte
		;; set up hard disk interrupt vectors
		SET_INT_VECTOR 13h, BIOSSEG, int13_handler
		SET_INT_VECTOR 76h, BIOSSEG, int76_handler
		;; INT 41h/46h: hard disk 0/1 dpt
		; TODO: This should be done from the code which
		; builds the DPTs?
		SET_INT_VECTOR 41h, EBDA_SEG, 3Dh
		SET_INT_VECTOR 46h, EBDA_SEG, 4Dh
		ret

hard_drive_post	endp


;; --------------------------------------------------------
;; INT 13h handler - Disk services
;; --------------------------------------------------------
		BIOSORG	0E3FEh

int13_handler:
		jmp	int13_relocated


;; --------------------------------------------------------
;; Fixed Disk Parameter Table
;; --------------------------------------------------------
;;		BIOSORG	0E401h - fixed wrt preceding

rom_fdpt:

;; --------------------------------------------------------
;; INT 19h handler - Boot load service
;; --------------------------------------------------------
		BIOSORG	0E6F2h

int19_handler:
		jmp	int19_relocated



;; --------------------------------------------------------
;; System BIOS Configuration Table
;; --------------------------------------------------------
;;		BIOSORG	0E6F5h - fixed wrt preceding
; must match BIOS_CONFIG_TABLE
bios_cfg_table:
		dw	9	; table size in bytes
		db	SYS_MODEL_ID
		db	SYS_SUBMODEL_ID
		db	BIOS_REVISION
		; Feature byte 1
		; b7: 1=DMA channel 3 used by hard disk
		; b6: 1=2 interrupt controllers present
		; b5: 1=RTC present
		; b4: 1=BIOS calls int 15h/4Fh for every key
		; b3: 1=wait for extern event supported (Int 15h/41h)
		; b2: 1=extended BIOS data area used
		; b1: 0=AT or ESDI bus, 1=MicroChannel
		; b0: 1=Dual bus (MicroChannel + ISA)
ifdef BX_CALL_INT15_4F
		db	74h; or USE_EBDA
else
		db	64h; or USE_EBDA
endif
		; Feature byte 2
		; b7: 1=32-bit DMA supported
		; b6: 1=int16h, function 9 supported
		; b5: 1=int15h/C6h (get POS data) supported
		; b4: 1=int15h/C7h (get mem map info) supported
		; b3: 1=int15h/C8h (en/dis CPU) supported
		; b2: 1=non-8042 kb controller
		; b1: 1=data streaming supported
		; b0: reserved
		db	40h
		; Feature byte 3
		; b7: not used
		; b6: reserved
		; b5: reserved
		; b4: POST supports ROM-to-RAM enable/disable
		; b3: SCSI on system board
		; b2: info panel installed
		; b1: Initial Machine Load (IML) system - BIOS on disk
		; b0: SCSI supported in IML
		db	0
		; Feature byte 4
		; b7: IBM private
		; b6: EEPROM present
		; b5-3: ABIOS presence (011 = not supported)
		; b2: private
		; b1: memory split above 16Mb supported
		; b0: POSTEXT directly supported by POST
		db	0
		; Feature byte 5 (IBM)
		; b1: enhanced mouse
		; b0: flash EPROM
		db	0


;; --------------------------------------------------------
;; Baud Rate Generator Table
;; --------------------------------------------------------
		BIOSORG	0E729h


;; --------------------------------------------------------
;; INT 14h handler -  Serial Communication Service
;; --------------------------------------------------------
		BIOSORG	0E739h
int14_handler:
		push	ds
		push	es
		pusha
		C_SETUP
		call	_int14_function
		popa
		pop	es
		pop	ds
		iret



;;
;; Handler for unexpected hardware interrupts
;;
dummy_isr:
		push	ds
		push	es
		pusha
		C_SETUP
		call	_dummy_isr_function
		popa
		pop	es
		pop	ds
		iret


rom_checksum 	proc	near
		push	ax
ifdef CHECKSUM_ROMS
		push	bx
		push	cx
		xor	ax, ax
		xor	bx, bx
		xor	cx, cx
		mov	ch, ds:[2]
		shl	cx, 1
checksum_loop:
		add	al, [bx]
		inc	bx
		loop	checksum_loop
		and	al, 0FFh	; set flags
		pop	cx
		pop	bx
else
		xor	al, al
endif
		pop	ax
		ret
rom_checksum	endp


;;
;; ROM scan - scan for valid ROMs and initialize them
;;
rom_scan:
		mov	cx, 0C000h	; start at C000
rom_scan_loop:
		mov	ds, cx
		mov	ax, 4		; scan in 2K increments
		cmp	word ptr ds:[0], 0AA55h	; look for signature
		jne	rom_scan_increment

		call	rom_checksum
		jnz	rom_scan_increment

		mov	al, ds:[2]	; set increment to ROM length
		test	al, 3
		jz	block_count_rounded

		and	al, 0FCh	; round up
		add	al, 4		; to nearest 2K
block_count_rounded:
		xor	bx, bx
		mov	ds, bx
		push	ax
		push	cx		; push segment...
		push	3		; ...and offset of ROM entry
		mov	bp, sp
		call	dword ptr [bp]	; call ROM init routine
		cli			; in case ROM enabled interrupts
		add	sp, 2		; get rid of offset
		pop	cx		; restore registers
		pop	ax
rom_scan_increment:
		shl	ax, 5		; convert to 16-byte increments
		add	cx, ax
		cmp	cx, 0E800h	; must encompass VBOX_LANBOOT_SEG!
		jbe	rom_scan_loop

		xor	ax, ax		; DS back to zero
		mov	ds, ax
		ret

init_pic	proc	near

		mov	al, 11h		; send init commands
		out	PIC_MASTER, al
		out	PIC_SLAVE, al
		mov	al, 08h		; base 08h
		out	PIC_MASTER+1, al
		mov	al, 70h		; base 70h
		out	PIC_SLAVE+1, al
		mov	al, 04h		; master PIC
		out	PIC_MASTER+1, al
		mov	al, 02h		; slave PIC
		out	PIC_SLAVE+1, al
		mov	al, 01h
		out	PIC_MASTER+1, al
		out	PIC_SLAVE+1, al
		mov	al, 0B8h	; unmask IRQs 0/1/2/6
		out	PIC_MASTER+1, al
		mov	al, 08Fh
		out	PIC_SLAVE+1, al	; unmask IRQs 12/13/14
		ret

init_pic	endp

ebda_post	proc	near

		SET_INT_VECTOR 0Dh, BIOSSEG, dummy_isr	; IRQ 5
		SET_INT_VECTOR 0Fh, BIOSSEG, dummy_isr	; IRQ 7
		SET_INT_VECTOR 72h, BIOSSEG, dummy_isr	; IRQ 11
		SET_INT_VECTOR 77h, BIOSSEG, dummy_isr	; IRQ 15

		mov	ax, EBDA_SEG
		mov	ds, ax
		mov	byte ptr ds:[0], EBDA_SIZE
		;; store EBDA seg in 40:0E
		xor	ax, ax
		mov	ds, ax
		mov	word ptr ds:[40Eh], EBDA_SEG
		ret

ebda_post	endp



;; --------------------------------------------------------
;; INT 16h handler - Keyboard service
;; --------------------------------------------------------
		BIOSORG	0E82Eh
int16_handler:
		sti
		push	es
		push	ds
		pusha

		cmp	ah, 0
		je	int16_F00

		cmp	ah, 10h
		je	int16_F00

		C_SETUP
		call	_int16_function
		popa
		pop	ds
		pop	es
		iret

int16_F00:
		mov	bx, 40h		; TODO: why 40h here and 0 elsewhere?
		mov	ds, bx
int16_wait_for_key:
		cli
		mov	bx, ds:[1Ah]
		cmp	bx, ds:[1Ch]
		jne	int16_key_found
		sti
		nop
; TODO: review/enable?
if 0
		push	ax
		mov	ax, 9002h
		int	15h
		pop	ax
endif
		jmp	int16_wait_for_key

int16_key_found:
		C_SETUP
		call	_int16_function
		popa
		pop	ds
		pop	es
; TODO: review/enable? If so, flags should be restored here?
if 0
		push	ax
		mov	ax, 9202h
		int	15h
		pop	ax
endif
		iret


;; Quick and dirty protected mode entry/exit routines
include pmode.inc

;; Initialization code which needs to run in protected mode (LAPIC etc.)
include pmsetup.inc


KBDC_DISABLE	EQU	0ADh
KBDC_ENABLE	EQU	0AEh
KBC_CMD		EQU	64h
KBC_DATA	EQU	60h

;; --------------------------------------------------------
;; INT 09h handler - Keyboard ISR (IRQ 1)
;; --------------------------------------------------------
		BIOSORG	0E987h
int09_handler:
		cli			; TODO: why? they're off already!
		push	ax
		mov	al, KBDC_DISABLE
		out	KBC_CMD, al

		mov	al, 0Bh
		out	PIC_MASTER, al
		in	al, PIC_MASTER
		and	al, 2
		jz	int09_finish

		in	al, KBC_DATA
		push	ds
		pusha
		cld			; Before INT 15h (and any C code)
ifdef BX_CALL_INT15_4F
		mov	ah, 4Fh
		stc
		int	15h		; keyboard intercept
		jnc	int09_done
endif
		sti			; Only after calling INT 15h

		;; check for extended key
		cmp	al, 0E0h
		jne	int09_check_pause
		xor	ax, ax
		mov	ds, ax
		mov	al, ds:[496h]	; mf2_state |= 0x02
		or	al, 2		; TODO: why not RMW?
		mov	ds:[496h], al
		jmp	int09_done

int09_check_pause:
		cmp	al, 0E1h	; pause key?
		jne	int09_process_key
		xor	ax, ax
		mov	ds, ax		; TODO: haven't we just done that??
		mov	al, ds:[496h]
		or	al, 1
		mov	ds:[496h], al	; TODO: why not RMW?
		jmp	int09_done

int09_process_key:
		push	es
		C_SETUP
		call	_int09_function
		pop	es

int09_done:
		popa
		pop	ds
		cli
		call	eoi_master_pic

int09_finish:
		mov	al, KBDC_ENABLE
		out	KBC_CMD, al
		pop	ax
		iret


;; --------------------------------------------------------
;; INT 13h handler - Diskette service
;; --------------------------------------------------------
		BIOSORG	0EC59h
int13_diskette:
		jmp	int13_noeltorito



;; --------------------------------------------------------
;; INT 13h handler - Disk service
;; --------------------------------------------------------
int13_relocated:
		;; check for an El-Torito function
		cmp	ah, 4Ah
		jb	int13_not_eltorito

		cmp	ah, 4Dh
		ja	int13_not_eltorito

		pusha
		push	es
		push	ds
		C_SETUP			; TODO: setup C envrionment only once?
		push	int13_out	; simulate a call
		jmp	_int13_eltorito	; ELDX not used

int13_not_eltorito:
		push	es
		push	ax		; TODO: better register save/restore
		push	bx
		push	cx
		push	dx

		;; check if emulation is active
		call	_cdemu_isactive
		cmp	al, 0
		je	int13_cdemu_inactive

		;; check if access to the emulated drive
		call	_cdemu_emulated_drive
		pop	dx		; recover dx (destroyed by C code)
		push	dx
		cmp	al, dl		; INT 13h on emulated drive
		jne	int13_nocdemu

		pop	dx
		pop	cx
		pop	bx
		pop	ax
		pop	es

		pusha
		push	es
		push	ds
		C_SETUP			; TODO: setup environment only once?

		push	int13_out	; simulate a call
		jmp	_int13_cdemu	; ELDX not used

int13_nocdemu:
		and	dl, 0E0h	; mask to get device class
		cmp	al, dl
		jne	int13_cdemu_inactive

		pop	dx
		pop	cx
		pop	bx
		pop	ax
		pop	es

		push	ax
		push	cx
		push	dx
		push	bx

		dec	dl		; real drive is dl - 1
		jmp	int13_legacy

int13_cdemu_inactive:
		pop	dx
		pop	cx
		pop	bx
		pop	ax
		pop	es

int13_noeltorito:
		push	ax
		push	cx
		push	dx
		push	bx
int13_legacy:
		push	dx		; push eltorito dx in place of sp
		push	bp
		push	si
		push	di
		push	es
		push	ds
		C_SETUP			; TODO: setup environment only once?

		;; now the registers can be restored with
		;; pop ds; pop es; popa; iret
		test	dl, 80h		; non-removable?
		jnz	int13_notfloppy

		push	int13_out	; simulate a near call
		jmp	_int13_diskette_function

int13_notfloppy:
		cmp	dl, 0E0h
		jb	int13_notcdrom

		;; ebx may be modified, save here
		;; TODO: check/review 32-bit register use
		.386
		shr	ebx, 16
		push	bx
		call	_int13_cdrom
		pop	bx
		shl	ebx, 16
		.286

		jmp	int13_out

int13_notcdrom:
int13_disk:
		cmp	ah,40h
		ja	int13x
		call	_int13_harddisk
		jmp	int13_out

int13x:
		call	_int13_harddisk_ext

int13_out:
		pop	ds
		pop	es
		popa
		iret



; parallel port detection: port in dx, index in bx, timeout in cl
detect_parport	proc	near

		push	dx
		inc	dx
		inc	dx
		in	al, dx
		and	al, 0DFh	; clear input mode
		out	dx, al
		pop	dx
		mov	al, 0AAh
		out	dx, al
		in	al, dx
		cmp	al, 0AAh
		jne	no_parport

		push	bx
		shl	bx, 1
		mov	[bx+408h], dx	; parallel I/O address
		pop	bx
		mov	[bx+478h], cl	; parallel printer timeout
		inc	bx
no_parport:
		ret

detect_parport	endp

; setial port detection: port in dx, index in bx, timeout in cl
detect_serial	proc	near

		push	dx
		inc	dx
		mov	al, 2
		out	dx, al
		in	al, dx
		cmp	al, 2
		jne	no_serial

		inc	dx
		in	al, dx
		cmp	al, 2
		jne	no_serial

		dec	dx
		xor	al, al
		pop	dx
		push	bx
		shl	bx, 1
		mov	[bx+400h], dx	; serial I/O address
		pop	bx
		mov	[bx+47Ch], cl	; serial timeout
		inc	bx
		ret

no_serial:
		pop	dx
		ret

detect_serial	endp


;;
;; POST: Floppy drive
;;
floppy_post	proc	near

		xor	ax, ax
		mov	ds, ax

		;; TODO: This code is really stupid. Zeroing the BDA byte
		;; by byte is dumb, and it's been already zeroed elsewhere!
		mov	al, 0
		mov	ds:[43Eh], al	; drive 0/1 uncalibrated, no IRQ
		mov	ds:[43Fh], al	; motor status
		mov	ds:[440h], al	; motor timeout counter
		mov	ds:[441h], al	; controller status return code
		mov	ds:[442h], al	; hd/floppy ctlr status register
		mov	ds:[443h], al	; controller status register 1
		mov	ds:[444h], al	; controller status register 2
		mov	ds:[445h], al	; cylinder number
		mov	ds:[446h], al	; head number
		mov	ds:[447h], al	; sector number
		mov	ds:[448h], al	; bytes written

		mov	ds:[48Bh], al	; configuration data

		mov	al, 10h		; floppy drive type
		out	CMOS_ADDR, al
		in	al, CMOS_DATA
		mov	ah, al		; save drive type byte

look_drive0:
		; TODO: pre-init bl to reduce jumps
		shr	al, 4		; drive 0 in high nibble
		jz	f0_missing	; jump if no drive
		mov	bl, 7		; drv0 determined, multi-rate, chgline
		jmp	look_drive1

f0_missing:
		mov	bl, 0		; no drive 0

look_drive1:
		mov	al, ah		; restore CMOS data
		and	al, 0Fh		; drive 1 in low nibble
		jz	f1_missing
		or	bl, 70h		; drv1 determined, multi-rate, chgline
f1_missing:
		mov	ds:[48Fh], bl	; store in BDA

		;; TODO: See above. Dumb *and* redundant!
		mov	al, 0
		mov	ds:[490h], al	; drv0 media state
		mov	ds:[491h], al	; drv1 media state
		mov	ds:[492h], al	; drv0 operational state
		mov	ds:[493h], al	; drv1 operational state
		mov	ds:[494h], al	; drv0 current cylinder
		mov	ds:[495h], al	; drv1 current cylinder

		mov	al, 2
		out	0Ah, al		; unmask DMA channel 2

		SET_INT_VECTOR 1Eh, BIOSSEG, _diskette_param_table
		SET_INT_VECTOR 40h, BIOSSEG, int13_diskette
		SET_INT_VECTOR 0Eh, BIOSSEG, int0e_handler	; IRQ 6

		ret

floppy_post	endp


bcd_to_bin	proc	near

		;; in : AL in packed BCD format
		;; out: AL in binary, AH always 0
		shl	ax, 4
		shr	al, 4
		aad
		ret

bcd_to_bin	endp

rtc_post	proc	near

		.386
		;; get RTC seconds
		xor	eax, eax
		mov	al, 0
		out	CMOS_ADDR, al
		in	al, CMOS_DATA	; RTC seconds, in BCD
		call	bcd_to_bin	; eax now has seconds in binary
		mov	edx, 18206507
		mul	edx
		mov	ebx, 1000000
		xor	edx, edx
		div	ebx
		mov	ecx, eax	; total ticks in ecx

		;; get RTC minutes
		xor	eax, eax
		mov	al, 2
		out	CMOS_ADDR, al
		in	al, CMOS_DATA	; RTC minutes, in BCD
		call	bcd_to_bin	; eax now has minutes in binary
		mov	edx, 10923904
		mul	edx
		mov	ebx, 10000
		xor	edx, edx
		div	ebx
		add	ecx, eax	; add to total ticks

		;; get RTC hours
		xor	eax, eax
		mov	al, 4
		out	CMOS_ADDR, al
		in 	al, CMOS_DATA	; RTC hours, in BCD
		call	bcd_to_bin	; eax now has hours in binary
		mov	edx, 65543427
		mul	edx
		mov	ebx, 1000
		xor	edx, edx
		div	ebx
		add	ecx, eax	; add to total ticks

		mov	ds:[46Ch], ecx	; timer tick count
		xor	al, al		; TODO: redundant?
		mov	ds:[470h], al	; rollover flag
		.286
		ret

rtc_post	endp



;; --------------------------------------------------------
;; INT 0Eh handler - Diskette IRQ 6 ISR
;; --------------------------------------------------------
		BIOSORG	0EF57h
int0e_handler:
		push	ax
		push	dx
		mov	dx, 3F4h
		in	al, dx
		and	al, 0C0h
		cmp	al, 0C0h
		je	int0e_normal
		mov	dx, 3F5h
		mov	al, 08h		; sense interrupt
		out	dx, al
int0e_loop1:
		mov	dx, 3F4h	; TODO: move out of the loop?
		in	al, dx
		and	al, 0C0h
		cmp	al, 0C0h
		jne	int0e_loop1

int0e_loop2:
		mov	dx, 3F5h	; TODO: inc/dec dx instead
		in	al, dx
		mov	dx, 3F4h
		in	al, dx
		and	al, 0C0h
		cmp	al, 0C0h
		je	int0e_loop2

int0e_normal:
		push	ds
		xor	ax, ax
		mov	ds, ax
		call	eoi_master_pic
		; indicate that an interrupt occurred
		or	byte ptr ds:[43Eh], 80h
		pop	ds
		pop	dx
		pop	ax
		iret


;; --------------------------------------------------------
;; Diskette Parameter Table
;; --------------------------------------------------------
		BIOSORG	0EFC7h
_diskette_param_table:
		db	0AFh
		db	2		; HLT=1, DMA mode
		db	025h
		db	2
		db	18		; SPT (good for 1.44MB media)
		db	01Bh
		db	0FFh
		db	06Ch
		db	0F6h		; format filler
		db	15
		db	8



;; --------------------------------------------------------
;; INT 17h handler - Printer service
;; --------------------------------------------------------
;;		BIOSORG	0EFD2h - fixed WRT preceding code

		jmp	int17_handler	; NT floppy boot workaround
					; see @bugref{6481}
int17_handler:
		push	ds
		push	es
		pusha
		C_SETUP
		call	_int17_function
		popa
		pop	es
		pop	ds
		iret



;; Protected mode IDT descriptor
;;
;; The limit is 0 to cause a shutdown if an exception occurs
;; in protected mode. TODO: Is that what we really want?
;;
;; Set base to F0000 to correspond to beginning of BIOS,
;; in case an IDT is defined later.

_pmode_IDT:
		dw	0		; limit 15:0
		dw	0		; base  15:0
		dw	0Fh		; base  23:16


;; Real mode IDT descriptor
;;
;; Set to typical real-mode values.
;; base  = 000000
;; limit =   03ff

_rmode_IDT:
		dw	3FFh		; limit 15:00
		dw	0		; base  15:00
		dw	0		; base  23:16


;;
;; INT 1Ch
;;
;; TODO: Why does this need a special handler?
int1c_handler:	;; user timer tick
		iret



;; --------------------------------------------------------
;; INT 10h functions 0-Fh entry point
;; --------------------------------------------------------
		BIOSORG 0F045h
i10f0f_entry:
		iret


;; --------------------------------------------------------
;; INT 10h handler - MDA/CGA video
;; --------------------------------------------------------
		BIOSORG 0F065h
int10_handler:
		;; do nothing - assumes VGA
		iret


;; --------------------------------------------------------
;; MDA/CGA Video Parameter Table (INT 1Dh)
;; --------------------------------------------------------
		BIOSORG 0F0A4h
mdacga_vpt:


;;
;; INT 18h - boot failure
;;
int18_handler:
		C_SETUP
		call	_int18_panic_msg
		;; TODO: handle failure better?
		hlt
		iret

;;
;; INT 19h - boot service - relocated
;;
int19_relocated:
; If an already booted OS calls int 0x19 to reboot, it is not sufficient
; just to try booting from the configured drives. All BIOS variables and
; interrupt vectors need to be reset, otherwise strange things may happen.
; The approach used is faking a warm reboot (which just skips showing the
; logo), which is a bit more than what we need, but hey, it's fast.
		mov	bp, sp
		mov	ax, [bp+2]	; TODO: redundant? address via sp?
		cmp	ax, BIOSSEG	; check caller's segment
		jz	bios_initiated_boot

		xor	ax, ax
		mov	ds, ax
		mov	ax, 1234h
		mov	ds:[472], ax
		jmp	post

bios_initiated_boot:
		;; The C worker function returns the boot drive in bl and
		;; the boot segment in ax. In case of failure, the boot
		;; segment will be zero.
		C_SETUP			; TODO: Here? Now?
		push	bp
		mov	bp, sp

		;; 1st boot device
		mov	ax, 1
		push	ax
		call	_int19_function
		inc	sp
		inc	sp
		test	ax, ax		; if 0, try next device
		jnz	boot_setup

		;; 2nd boot device
		mov	ax, 2
		push	ax
		call	_int19_function
		inc	sp
		inc	sp
		test	ax, ax		; if 0, try next device
		jnz	boot_setup

		; 3rd boot device
		mov	ax, 3
		push	3
		call	_int19_function
		inc	sp
		inc	sp
		test	ax, ax		; if 0, try next device
		jnz	boot_setup

		; 4th boot device
		mov	ax, 4
		push	ax
		call	_int19_function
		inc	sp
		inc	sp
		test	ax, ax		; if 0, invoke INT 18h
		jz	int18_handler

boot_setup:
; TODO: the drive should be in dl already??
;;		mov	dl, bl		; tell guest OS what boot drive is
		.386	; NB: We're getting garbage into high eax bits
		shl	eax, 4		; convert seg to ip
		mov	[bp+2], ax	; set ip

		shr	eax, 4		; get cs back
		.286
		and	ax, BIOSSEG	; remove what went in ip
		mov	[bp+4], ax	; set cs
		xor	ax, ax
		mov	ds, ax
		mov	es, ax
		mov	[bp], ax	; TODO: what's this?!
		mov	ax, 0AA55h	; set ok flag ; TODO: and this?

		pop	bp		; TODO: why'd we just zero it??
		iret			; beam me up scotty

;; PCI BIOS

include pcibios.inc
include pirq.inc


;; --------------------------------------------------------
;; INT 12h handler - Memory size
;; --------------------------------------------------------
		BIOSORG	0F841h
int12_handler:
		;; Don't touch - fixed size!
		sti
		push	ds
		mov	ax, 40h
		mov	ds, ax
		mov	ax, ds:[13h]
		pop	ds
		iret


;; --------------------------------------------------------
;; INT 11h handler - Equipment list service
;; --------------------------------------------------------
;;		BIOSORG	0F84Dh - fixed wrt preceding code
int11_handler:
		;; Don't touch - fixed size!
		sti
		push	ds
		mov	ax, 40h
		mov	ds, ax
		mov	ax, ds:[10h]
		pop	ds
		iret


;; --------------------------------------------------------
;; INT 15h handler - System services
;; --------------------------------------------------------
;;		BIOSORG	0F859h - fixed wrt preceding code
int15_handler:
		pushf
		push	ds
		push	es
		C_SETUP
		cmp	ah, 86h
		je	int15_handler32
		cmp	ah, 0E8h
		je	int15_handler32
		pusha
		cmp	ah, 53h		; APM function?
		je	apm_call
		cmp	ah, 0C2h	; PS/2 mouse function?
		je	int15_handler_mouse

		call	_int15_function
int15_handler_popa_ret:
		popa
int15_handler32_ret:
		pop	es
		pop	ds
		popf
		jmp	iret_modify_cf

apm_call:
		call	_apm_function
		jmp	int15_handler_popa_ret

int15_handler_mouse:
		call	_int15_function_mouse
		jmp	int15_handler_popa_ret

int15_handler32:
		;; need to save/restore 32-bit registers
		.386
		pushad
		call	_int15_function32
		popad
		.286
		jmp	int15_handler32_ret

;;
;; Perform an IRET but retain the current carry flag value
;;
iret_modify_cf:
		jc	carry_set
		push	bp
		mov	bp, sp
		and	byte ptr [bp + 6], 0FEh
		pop	bp
		iret
carry_set:
		push	bp
		mov	bp, sp
		or	byte ptr [bp + 6], 1
		pop	bp
		iret

;;
;; INT 74h handler - PS/2 mouse (IRQ 12)
;;
int74_handler	proc

		sti
		pusha
		push	es
		push	ds
		push	0		; placeholder for status
		push	0		; placeholder for X
		push	0		; placeholder for Y
		push	0		; placeholder for Z
		push	0		; placeholder for make_far_call bool
		C_SETUP
		call	_int74_function
		pop	cx		; pop make_far_call flag
		jcxz	int74_done

		;; make far call to EBDA:0022
		push	0
		pop	ds
		push	ds:[40Eh]
		pop	ds
		call	far ptr ds:[22h]
int74_done:
		cli
		call	eoi_both_pics
		add	sp, 8		; remove status, X, Y, Z
		pop	ds
		pop	es
		popa
		iret

int74_handler	endp

int76_handler	proc

		;; record completion in BIOS task complete flag
		push	ax
		push	ds
		mov	ax, 40h
		mov	ds, ax
		mov	byte ptr ds:[8Eh], 0FFh
		call	eoi_both_pics
		pop	ds
		pop	ax
		iret

int76_handler	endp

;; --------------------------------------------------------
;; 8x8 font (first 128 characters)
;; --------------------------------------------------------
		BIOSORG	0FA6Eh
include font8x8.inc


;; --------------------------------------------------------
;; INT 1Ah handler - Time of the day + PCI BIOS
;; --------------------------------------------------------
;;		BIOSORG	0FE6Eh - fixed wrt preceding table
int1a_handler:
		cmp	ah, 0B1h
		jne	int1a_normal

		push	es
		push	ds
		C_SETUP
		.386
		pushad
		call	_pci16_function
		popad
		.286
		pop	ds
		pop	es
		iret

int1a_normal:
		push	es
		push	ds
		pusha
		C_SETUP
int1a_callfunction:
		call	_int1a_function
		popa
		pop	ds
		pop	es
		iret


;;
;; IRQ 8 handler (RTC)
;;
int70_handler:
		push	es
		push	ds
		pusha
		C_SETUP
		call	_int70_function
		popa
		pop	ds
		pop	es
		iret


;; --------------------------------------------------------
;; Timer tick - IRQ 0 handler
;; --------------------------------------------------------
		BIOSORG	0FEA5h
int08_handler:
		.386
		sti
		push	eax
		push	ds
		xor	ax, ax
		mov	ds, ax

		;; time to turn off floppy driv motor(s)?
		mov	al, ds:[440h]
		or	al, al
		jz	int08_floppy_off
		;; turn motor(s) off
		push	dx
		mov	dx, 03F2h
		in	al, dx
		and	al, 0CFh
		out	dx, al
		pop	dx

int08_floppy_off:
		mov	eax, ds:[46Ch]	; get ticks dword
		inc	eax

		;; compare eax to one day's worth of ticks (at 18.2 Hz)
		cmp	eax, 1800B0h
		jb	int08_store_ticks
		;; there has been a midnight rollover
		xor	eax, eax
		inc	byte ptr ds:[470h]	; increment rollover flag

int08_store_ticks:
		mov	ds:[46Ch], eax
		int	1Ch		; call the user timer handler
		cli
		call	eoi_master_pic
		pop	ds
		pop	eax
		.286
		iret


;; --------------------------------------------------------
;; Initial interrupt vector offsets for POST
;; --------------------------------------------------------
		BIOSORG	0FEF3h
vector_table:



;; --------------------------------------------------------
;; BIOS copyright string
;; --------------------------------------------------------
		BIOSORG	0FF00h
bios_string:
		db	BIOS_COPYRIGHT


;; --------------------------------------------------------
;; IRET - default interrupt handler
;; --------------------------------------------------------
		BIOSORG	0FF53h

dummy_iret:
		iret


;; --------------------------------------------------------
;; INT 05h - Print Screen service
;; --------------------------------------------------------
;;		BIOSORG	0FF54h - fixed wrt preceding
int05_handler:
		;; Not implemented
		iret

include smidmi.inc

;; --------------------------------------------------------
;; Processor reset entry point
;; --------------------------------------------------------
		BIOSORG	0FFF0h
cpu_reset:
		;; This is where the CPU starts executing after a reset
		jmp	far ptr post

		;; BIOS build date
		db	BIOS_BUILD_DATE
		db	0			; padding
		;; System model ID
		db	SYS_MODEL_ID
		;; Checksum byte
		db	0FFh


BIOSSEG		ends

		end

