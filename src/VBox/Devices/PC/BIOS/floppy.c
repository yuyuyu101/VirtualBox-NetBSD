/*
 * Copyright (C) 2006-2011 Oracle Corporation
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
 *  ROM BIOS for use with Bochs/Plex86/QEMU emulation environment
 *
 *  Copyright (C) 2002  MandrakeSoft S.A.
 *
 *    MandrakeSoft S.A.
 *    43, rue d'Aboukir
 *    75002 Paris - France
 *    http://www.linux-mandrake.com/
 *    http://www.mandrakesoft.com/
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 */


#include <stdint.h>
#include "inlines.h"
#include "biosint.h"

//////////////////////
// FLOPPY functions //
//////////////////////

void set_diskette_ret_status(uint8_t value)
{
    write_byte(0x0040, 0x0041, value);
}

void set_diskette_current_cyl(uint8_t drive, uint8_t cyl)
{
    if (drive > 1)
        BX_PANIC("set_diskette_current_cyl: drive > 1\n");
    write_byte(0x0040, 0x0094+drive, cyl);
}

#if 1 //BX_SUPPORT_FLOPPY

#if DEBUG_INT13_FL
#  define BX_DEBUG_INT13_FL(...) BX_DEBUG(__VA_ARGS__)
#else
#  define BX_DEBUG_INT13_FL(...)
#endif

#define BX_FLOPPY_ON_CNT    37          /* 2 seconds */

extern  int     diskette_param_table;   /* At a fixed location. */

void floppy_reset_controller(void)
{
    uint8_t     val8;
    
    // Reset controller
    val8 = inb(0x03f2);
    outb(0x03f2, val8 & ~0x04);
    outb(0x03f2, val8 | 0x04);
    
    // Wait for controller to come out of reset
    do {
        val8 = inb(0x3f4);
    } while ( (val8 & 0xc0) != 0x80 );
}

void floppy_prepare_controller(uint16_t drive)
{
    uint8_t     val8, dor, prev_reset;
    
    // set 40:3e bit 7 to 0
    val8 = read_byte(0x0040, 0x003e);
    val8 &= 0x7f;
    write_byte(0x0040, 0x003e, val8);
    
    // turn on motor of selected drive, DMA & int enabled, normal operation
    prev_reset = inb(0x03f2) & 0x04;
    if (drive)
        dor = 0x20;
    else
        dor = 0x10;
        dor |= 0x0c;
        dor |= drive;
        outb(0x03f2, dor);
    
    // reset the disk motor timeout value of INT 08
    write_byte(0x40,0x40, BX_FLOPPY_ON_CNT);
    
    // program data rate
    val8 = read_byte(0x0040, 0x008b);
    val8 >>= 6;
    outb(0x03f7, val8);
    
    // wait for drive readiness
    do {
        val8 = inb(0x3f4);
    } while ( (val8 & 0xc0) != 0x80 );
    
    if (prev_reset == 0) {
        // turn on interrupts
        int_enable();
        // wait on 40:3e bit 7 to become 1
        do {
            val8 = read_byte(0x0040, 0x003e);
        } while ( (val8 & 0x80) == 0 );
        val8 &= 0x7f;
        int_disable();
        write_byte(0x0040, 0x003e, val8);
    }
}

bx_bool floppy_media_known(uint16_t drive)
{
    uint8_t     val8;
    uint16_t    media_state_offset;
    
    val8 = read_byte(0x0040, 0x003e); // diskette recal status
    if (drive)
        val8 >>= 1;
    val8 &= 0x01;
    if (val8 == 0)
        return 0;
    
    media_state_offset = 0x0090;
    if (drive)
        media_state_offset += 1;
    
    val8 = read_byte(0x0040, media_state_offset);
    val8 = (val8 >> 4) & 0x01;
    if (val8 == 0)
        return 0;
    
    // checks passed, return KNOWN
    return 1;
}

bx_bool floppy_read_id(uint16_t drive)
{
    uint8_t     val8;
    uint8_t     return_status[7];
    int         i;
    
    floppy_prepare_controller(drive);
    
    // send Read ID command (2 bytes) to controller
    outb(0x03f5, 0x4a);  // 4a: Read ID (MFM)
    outb(0x03f5, drive); // 0=drive0, 1=drive1, head always 0
    
    // turn on interrupts
    int_enable();
    
    // wait on 40:3e bit 7 to become 1
    do {
        val8 = (read_byte(0x0040, 0x003e) & 0x80);
    } while ( val8 == 0 );
    
    val8 = 0; // separate asm from while() loop
    // turn off interrupts
    int_disable();
    
    // read 7 return status bytes from controller
    for (i = 0; i < 7; ++i) {
        return_status[i] = inb(0x3f5);
    }
    
    if ( (return_status[0] & 0xc0) != 0 )
        return 0;
    else
        return 1;
}

bx_bool floppy_drive_recal(uint16_t drive)
{
    uint8_t     val8;
    uint16_t    curr_cyl_offset;
    
    floppy_prepare_controller(drive);
    
    // send Recalibrate command (2 bytes) to controller
    outb(0x03f5, 0x07);  // 07: Recalibrate
    outb(0x03f5, drive); // 0=drive0, 1=drive1
    
    // turn on interrupts
    int_enable();
    
    // wait on 40:3e bit 7 to become 1
    do {
        val8 = (read_byte(0x0040, 0x003e) & 0x80);
    } while ( val8 == 0 );
    
    val8 = 0; // separate asm from while() loop
    // turn off interrupts
    int_disable();
    
    // set 40:3e bit 7 to 0, and calibrated bit
    val8 = read_byte(0x0040, 0x003e);
    val8 &= 0x7f;
    if (drive) {
        val8 |= 0x02; // Drive 1 calibrated
        curr_cyl_offset = 0x0095;
    } else {
        val8 |= 0x01; // Drive 0 calibrated
        curr_cyl_offset = 0x0094;
    }
    write_byte(0x0040, 0x003e, val8);
    write_byte(0x0040, curr_cyl_offset, 0); // current cylinder is 0
    
    return 1;
}


bx_bool floppy_media_sense(uint16_t drive)
{
    bx_bool     retval;
    uint16_t    media_state_offset;
    uint8_t     drive_type, config_data, media_state;
    
    if (floppy_drive_recal(drive) == 0)
        return 0;
    
    // Try the diskette data rates in the following order:
    // 1 Mbps -> 500 Kbps -> 300 Kbps -> 250 Kbps
    // The 1 Mbps rate is only tried for 2.88M drives.
    
    // ** config_data **
    // Bitfields for diskette media control:
    // Bit(s)  Description (Table M0028)
    //  7-6  last data rate set by controller
    //        00=500kbps, 01=300kbps, 10=250kbps, 11=1Mbps
    //  5-4  last diskette drive step rate selected
    //        00=0Ch, 01=0Dh, 10=0Eh, 11=0Ah
    //  3-2  {data rate at start of operation}
    //  1-0  reserved
    
    // ** media_state **
    // Bitfields for diskette drive media state:
    // Bit(s)  Description (Table M0030)
    //  7-6  data rate
    //    00=500kbps, 01=300kbps, 10=250kbps, 11=1Mbps
    //  5  double stepping required (e.g. 360kB in 1.2MB)
    //  4  media type established
    //  3  drive capable of supporting 4MB media
    //  2-0  on exit from BIOS, contains
    //    000 trying 360kB in 360kB
    //    001 trying 360kB in 1.2MB
    //    010 trying 1.2MB in 1.2MB
    //    011 360kB in 360kB established
    //    100 360kB in 1.2MB established
    //    101 1.2MB in 1.2MB established
    //    110 reserved
    //    111 all other formats/drives

    // @todo: break out drive type determination
    drive_type = inb_cmos(0x10);
    if (drive == 0)
        drive_type >>= 4;
    else
        drive_type &= 0x0f;
    if ( drive_type == 1 ) {
        // 360K 5.25" drive
        config_data = 0x00; // 0000 0000
        media_state = 0x15; // 0001 0101
        retval = 1;
    }
    else if ( drive_type == 2 ) {
        // 1.2 MB 5.25" drive
        config_data = 0x00; // 0000 0000
        media_state = 0x35; // 0011 0101   // need double stepping??? (bit 5)
        retval = 1;
    }
    else if ( drive_type == 3 ) {
        // 720K 3.5" drive
        config_data = 0x00; // 0000 0000 ???
        media_state = 0x17; // 0001 0111
        retval = 1;
    }
    else if ( drive_type == 4 ) {
        // 1.44 MB 3.5" drive
        config_data = 0x00; // 0000 0000
        media_state = 0x17; // 0001 0111
        retval = 1;
    }
    else if ( drive_type == 5 ) {
        // 2.88 MB 3.5" drive
        config_data = 0xCC; // 1100 1100
        media_state = 0xD7; // 1101 0111
        retval = 1;
    }
    // Extended floppy size uses special cmos setting
    else if ( drive_type == 6 ) {
        // 160k 5.25" drive
        config_data = 0x00; // 0000 0000
        media_state = 0x27; // 0010 0111
        retval = 1;
    }
    else if ( drive_type == 7 ) {
        // 180k 5.25" drive
        config_data = 0x00; // 0000 0000
        media_state = 0x27; // 0010 0111
        retval = 1;
    }
    else if ( drive_type == 8 ) {
        // 320k 5.25" drive
        config_data = 0x00; // 0000 0000
        media_state = 0x27; // 0010 0111
        retval = 1;
    }
    else {
        // not recognized
        config_data = 0x00; // 0000 0000
        media_state = 0x00; // 0000 0000
        retval = 0;
    }
    
    write_byte(0x0040, 0x008B, config_data);
    while (!floppy_read_id(drive)) {
        if ((config_data & 0xC0) == 0x80) {
            // If even 250 Kbps failed, we can't do much
            break;
        }
        switch (config_data & 0xC0) {
        case 0xC0:  // 1 Mbps
            config_data = config_data & 0x3F | 0x00;
            break;
        case 0x00:  // 500 Kbps
            config_data = config_data & 0x3F | 0x40;
            break;
        case 0x40:  // 300 Kbps
            config_data = config_data & 0x3F | 0x80;
            break;
        }
        write_byte(0x0040, 0x008B, config_data);
    }
    
    if (drive == 0)
        media_state_offset = 0x0090;
    else
        media_state_offset = 0x0091;
    write_byte(0x0040, 0x008B, config_data);
    write_byte(0x0040, media_state_offset, media_state);
    
    return retval;
}


bx_bool floppy_drive_exists(uint16_t drive)
{
    uint8_t     drive_type;
    
    // check CMOS to see if drive exists
    // @todo: break out drive type determination
    drive_type = inb_cmos(0x10);
    if (drive == 0)
        drive_type >>= 4;
    else
        drive_type &= 0x0f;
    return drive_type != 0;
}

//@todo: put in a header
#define AX      r.gr.u.r16.ax
#define BX      r.gr.u.r16.bx
#define CX      r.gr.u.r16.cx
#define DX      r.gr.u.r16.dx
#define SI      r.gr.u.r16.si
#define DI      r.gr.u.r16.di
#define BP      r.gr.u.r16.bp
#define ELDX    r.gr.u.r16.sp
#define DS      r.ds
#define ES      r.es
#define FLAGS   r.ra.flags.u.r16.flags

void BIOSCALL int13_diskette_function(disk_regs_t r)
{
    uint8_t     drive, num_sectors, track, sector, head;
    uint16_t    base_address, base_count, base_es;
    uint8_t     page, mode_register, val8;
    uint8_t     return_status[7];
    uint8_t     drive_type, num_floppies, ah;
    uint16_t    last_addr;
    int         i;
    
    BX_DEBUG_INT13_FL("%s: AX=%04x BX=%04x CX=%04x DX=%04x ES=%04x\n", __func__, AX, BX, CX, DX, ES);
    
    ah = GET_AH();
    
    switch ( ah ) {
    case 0x00: // diskette controller reset
        BX_DEBUG_INT13_FL("floppy f00\n");
        drive = GET_ELDL();
        if (drive > 1) {
            SET_AH(1); // invalid param
            set_diskette_ret_status(1);
            SET_CF();
            return;
        }
        // @todo: break out drive type determination
        drive_type = inb_cmos(0x10);
        if (drive == 0)
            drive_type >>= 4;
        else
            drive_type &= 0x0f;
        if (drive_type == 0) {
            SET_AH(0x80); // drive not responding
            set_diskette_ret_status(0x80);
            SET_CF();
            return;
        }
        
        // force re-calibration etc.
        write_byte(0x0040, 0x003e, 0);
        
        SET_AH(0);
        set_diskette_ret_status(0);
        CLEAR_CF(); // successful
        set_diskette_current_cyl(drive, 0); // current cylinder
        return;
    
    case 0x01: // Read Diskette Status
        CLEAR_CF();
        val8 = read_byte(0x0000, 0x0441);
        SET_AH(val8);
        if (val8) {
            SET_CF();
        }
        return;
    
    case 0x02: // Read Diskette Sectors
    case 0x03: // Write Diskette Sectors
    case 0x04: // Verify Diskette Sectors
        num_sectors = GET_AL();
        track       = GET_CH();
        sector      = GET_CL();
        head        = GET_DH();
        drive       = GET_ELDL();
        
        if ( (drive > 1) || (head > 1) ||
         (num_sectors == 0) || (num_sectors > 72) ) {
            BX_INFO("%s: drive>1 || head>1 ...\n", __func__);
            SET_AH(1);
            set_diskette_ret_status(1);
            SET_AL(0); // no sectors read
            SET_CF(); // error occurred
            return;
        }
        
        // see if drive exists
        if (floppy_drive_exists(drive) == 0) {
            SET_AH(0x80); // not responding
            set_diskette_ret_status(0x80);
            SET_AL(0); // no sectors read
            SET_CF(); // error occurred
            return;
        }
        
        // see if media in drive, and type is known
        if (floppy_media_known(drive) == 0) {
            if (floppy_media_sense(drive) == 0) {
                SET_AH(0x0C); // Media type not found
                set_diskette_ret_status(0x0C);
                SET_AL(0); // no sectors read
                SET_CF(); // error occurred
                return;
            }
        }
        
        if (ah == 0x02) {
            // Read Diskette Sectors
            
            //-----------------------------------
            // set up DMA controller for transfer
            //-----------------------------------
            
            // es:bx = pointer to where to place information from diskette
            // port 04: DMA-1 base and current address, channel 2
            // port 05: DMA-1 base and current count, channel 2
            // @todo: merge/factor out pointer normalization
            page = (ES >> 12);              // upper 4 bits
            base_es = (ES << 4);            // lower 16bits contributed by ES
            base_address = base_es + BX;    // lower 16 bits of address
                                            // contributed by ES:BX
            if ( base_address < base_es ) {
                // in case of carry, adjust page by 1
                page++;
            }
            base_count = (num_sectors * 512) - 1;
            
            // check for 64K boundary overrun
            last_addr = base_address + base_count;
            if (last_addr < base_address) {
                SET_AH(0x09);
                set_diskette_ret_status(0x09);
                SET_AL(0); // no sectors read
                SET_CF(); // error occurred
                return;
            }
            
            BX_DEBUG_INT13_FL("masking DMA-1 c2\n");
            outb(0x000a, 0x06);
            
            BX_DEBUG_INT13_FL("clear flip-flop\n");
            outb(0x000c, 0x00); // clear flip-flop
            outb(0x0004, base_address);
            outb(0x0004, base_address>>8);
            BX_DEBUG_INT13_FL("clear flip-flop\n");
            outb(0x000c, 0x00); // clear flip-flop
            outb(0x0005, base_count);
            outb(0x0005, base_count>>8);
            BX_DEBUG_INT13_FL("xfer buf at %x:%x\n", page, base_address);
            
            // port 0b: DMA-1 Mode Register
            mode_register = 0x46; // single mode, increment, autoinit disable,
              // transfer type=write, channel 2
            BX_DEBUG_INT13_FL("setting mode register\n");
            outb(0x000b, mode_register);
            
            BX_DEBUG_INT13_FL("setting page register\n");
            // port 81: DMA-1 Page Register, channel 2
            outb(0x0081, page);
            
            BX_DEBUG_INT13_FL("unmask chan 2\n");
            outb(0x000a, 0x02); // unmask channel 2
            
            BX_DEBUG_INT13_FL("unmasking DMA-1 c2\n");
            outb(0x000a, 0x02);
            
            //--------------------------------------
            // set up floppy controller for transfer
            //--------------------------------------
            floppy_prepare_controller(drive);
            
            // send read-normal-data command (9 bytes) to controller
            outb(0x03f5, 0xe6); // e6: read normal data
            outb(0x03f5, (head << 2) | drive); // HD DR1 DR2
            outb(0x03f5, track);
            outb(0x03f5, head);
            outb(0x03f5, sector);
            outb(0x03f5, 2); // 512 byte sector size
            outb(0x03f5, sector + num_sectors - 1); // last sector to read on track
            outb(0x03f5, 0); // Gap length
            outb(0x03f5, 0xff); // Gap length
            
            // turn on interrupts
            int_enable();
            
            // wait on 40:3e bit 7 to become 1
            do {
                    val8 = read_byte(0x0040, 0x0040);
                if (val8 == 0) {
                    floppy_reset_controller();
                    SET_AH(0x80); // drive not ready (timeout)
                    set_diskette_ret_status(0x80);
                    SET_AL(0); // no sectors read
                    SET_CF(); // error occurred
                    return;
                }
                val8 = (read_byte(0x0040, 0x003e) & 0x80);
            } while ( val8 == 0 );
            
            val8 = 0; // separate asm from while() loop
            // turn off interrupts
            int_disable();
            
            // set 40:3e bit 7 to 0
            val8 = read_byte(0x0040, 0x003e);
            val8 &= 0x7f;
            write_byte(0x0040, 0x003e, val8);
            
            // check port 3f4 for accessibility to status bytes
            val8 = inb(0x3f4);
            if ( (val8 & 0xc0) != 0xc0 )
                BX_PANIC("%s: ctrl not ready\n", __func__);

            // read 7 return status bytes from controller and store in BDA
            for (i = 0; i < 7; ++i) {
                return_status[i] = inb(0x3f5);
                write_byte(0x0040, 0x0042 + i, return_status[i]);
            }
            
            if ( (return_status[0] & 0xc0) != 0 ) {
                SET_AH(0x20);
                set_diskette_ret_status(0x20);
                SET_AL(0); // no sectors read
                SET_CF(); // error occurred
                return;
            }

#ifdef DMA_WORKAROUND
            rep_movsw(ES :> BX, ES :> BX, num_sectors * 512 / 2);
#endif
            // ??? should track be new val from return_status[3] ?
            set_diskette_current_cyl(drive, track);
            // AL = number of sectors read (same value as passed)
            SET_AH(0x00); // success
            CLEAR_CF();   // success
            return;
        } else if (ah == 0x03) {
            // Write Diskette Sectors
            
            //-----------------------------------
            // set up DMA controller for transfer
            //-----------------------------------
            
            // es:bx = pointer to where to place information from diskette
            // port 04: DMA-1 base and current address, channel 2
            // port 05: DMA-1 base and current count, channel 2
            // @todo: merge/factor out pointer normalization
            page = (ES >> 12);              // upper 4 bits
            base_es = (ES << 4);            // lower 16bits contributed by ES
            base_address = base_es + BX;    // lower 16 bits of address
                                            // contributed by ES:BX
            if ( base_address < base_es ) {
                // in case of carry, adjust page by 1
                page++;
            }
            base_count = (num_sectors * 512) - 1;
            
            // check for 64K boundary overrun
            last_addr = base_address + base_count;
            if (last_addr < base_address) {
                SET_AH(0x09);
                set_diskette_ret_status(0x09);
                SET_AL(0); // no sectors read
                SET_CF(); // error occurred
                return;
            }
            
            BX_DEBUG_INT13_FL("masking DMA-1 c2\n");
            outb(0x000a, 0x06);
            
            outb(0x000c, 0x00); // clear flip-flop
            outb(0x0004, base_address);
            outb(0x0004, base_address>>8);
            outb(0x000c, 0x00); // clear flip-flop
            outb(0x0005, base_count);
            outb(0x0005, base_count>>8);
            BX_DEBUG_INT13_FL("xfer buf at %x:%x\n", page, base_address);
            
            // port 0b: DMA-1 Mode Register
            mode_register = 0x4a; // single mode, increment, autoinit disable,
              // transfer type=read, channel 2
            outb(0x000b, mode_register);
            
            // port 81: DMA-1 Page Register, channel 2
            outb(0x0081, page);
            
            BX_DEBUG_INT13_FL("unmasking DMA-1 c2\n");
            outb(0x000a, 0x02);
            
            //--------------------------------------
            // set up floppy controller for transfer
            //--------------------------------------
            floppy_prepare_controller(drive);
            
            // send write-normal-data command (9 bytes) to controller
            outb(0x03f5, 0xc5); // c5: write normal data
            outb(0x03f5, (head << 2) | drive); // HD DR1 DR2
            outb(0x03f5, track);
            outb(0x03f5, head);
            outb(0x03f5, sector);
            outb(0x03f5, 2); // 512 byte sector size
            outb(0x03f5, sector + num_sectors - 1); // last sector to write on track
            outb(0x03f5, 0); // Gap length
            outb(0x03f5, 0xff); // Gap length
            
            // turn on interrupts
            int_enable();
            
            // wait on 40:3e bit 7 to become 1
            do {
                val8 = read_byte(0x0040, 0x0040);
                if (val8 == 0) {
                    floppy_reset_controller();
                    SET_AH(0x80); // drive not ready (timeout)
                    set_diskette_ret_status(0x80);
                    SET_AL(0); // no sectors written
                    SET_CF(); // error occurred
                    return;
                }
                val8 = (read_byte(0x0040, 0x003e) & 0x80);
            } while ( val8 == 0 );
            
            val8 = 0; // separate asm from while() loop @todo: why??
            // turn off interrupts
            int_disable();
            
            // set 40:3e bit 7 to 0
            val8 = read_byte(0x0040, 0x003e);
            val8 &= 0x7f;
            write_byte(0x0040, 0x003e, val8);
            
            // check port 3f4 for accessibility to status bytes
            val8 = inb(0x3f4);
            if ( (val8 & 0xc0) != 0xc0 )
                BX_PANIC("%s: ctrl not ready\n", __func__);
            
            // read 7 return status bytes from controller and store in BDA
            for (i = 0; i < 7; ++i) {
                return_status[i] = inb(0x3f5);
                write_byte(0x0040, 0x0042 + i, return_status[i]);
            }
            
            if ( (return_status[0] & 0xc0) != 0 ) {
                if ( (return_status[1] & 0x02) != 0 ) {
                    // diskette not writable.
                    // AH=status code=0x03 (tried to write on write-protected disk)
                    // AL=number of sectors written=0
                    AX = 0x0300;
                    SET_CF();
                    return;
                } else {
                    BX_PANIC("%s: read error\n", __func__);
                }
            }
            
            // ??? should track be new val from return_status[3] ?
            set_diskette_current_cyl(drive, track);
            // AL = number of sectors read (same value as passed)
            SET_AH(0x00); // success
            CLEAR_CF();   // success
            return;
        } else {  // if (ah == 0x04)
            // Verify Diskette Sectors
            
            // ??? should track be new val from return_status[3] ?
            set_diskette_current_cyl(drive, track);
            // AL = number of sectors verified (same value as passed)
            CLEAR_CF();   // success
            SET_AH(0x00); // success
            return;
        }
        break;
    
    case 0x05: // format diskette track
        BX_DEBUG_INT13_FL("floppy f05\n");
        
        num_sectors = GET_AL();
        track       = GET_CH();
        head        = GET_DH();
        drive       = GET_ELDL();
        
        if ((drive > 1) || (head > 1) || (track > 79) ||
         (num_sectors == 0) || (num_sectors > 18)) {
            SET_AH(1);
            set_diskette_ret_status(1);
            SET_CF(); // error occurred
        }
        
        // see if drive exists
        if (floppy_drive_exists(drive) == 0) {
            SET_AH(0x80); // drive not responding
            set_diskette_ret_status(0x80);
            SET_CF(); // error occurred
            return;
        }
        
        // see if media in drive, and type is known
        if (floppy_media_known(drive) == 0) {
            if (floppy_media_sense(drive) == 0) {
                SET_AH(0x0C); // Media type not found
                set_diskette_ret_status(0x0C);
                SET_AL(0); // no sectors read
                SET_CF(); // error occurred
            return;
            }
        }
        
        // set up DMA controller for transfer
        // @todo: merge/factor out pointer normalization
        page = (ES >> 12);              // upper 4 bits
        base_es = (ES << 4);            // lower 16bits contributed by ES
        base_address = base_es + BX;    // lower 16 bits of address
                                        // contributed by ES:BX
        if ( base_address < base_es ) {
            // in case of carry, adjust page by 1
            page++;
        }
        base_count = (num_sectors * 4) - 1;
        
        // check for 64K boundary overrun
        last_addr = base_address + base_count;
        if (last_addr < base_address) {
            SET_AH(0x09);
            set_diskette_ret_status(0x09);
            SET_AL(0); // no sectors read
            SET_CF(); // error occurred
            return;
        }
        
        outb(0x000a, 0x06);
        outb(0x000c, 0x00); // clear flip-flop
        outb(0x0004, base_address);
        outb(0x0004, base_address>>8);
        outb(0x000c, 0x00); // clear flip-flop
        outb(0x0005, base_count);
        outb(0x0005, base_count>>8);
        mode_register = 0x4a; // single mode, increment, autoinit disable,
        // transfer type=read, channel 2
        outb(0x000b, mode_register);
        // port 81: DMA-1 Page Register, channel 2
        outb(0x0081, page);
        outb(0x000a, 0x02);
        
        // set up floppy controller for transfer
        floppy_prepare_controller(drive);
        
        // send format-track command (6 bytes) to controller
        outb(0x03f5, 0x4d); // 4d: format track
        outb(0x03f5, (head << 2) | drive); // HD DR1 DR2
        outb(0x03f5, 2); // 512 byte sector size
        outb(0x03f5, num_sectors); // number of sectors per track
        outb(0x03f5, 0); // Gap length
        outb(0x03f5, 0xf6); // Fill byte
        // turn on interrupts
        int_enable();
        
        // wait on 40:3e bit 7 to become 1
        do {
            val8 = read_byte(0x0040, 0x0040);
            if (val8 == 0) {
                floppy_reset_controller();
                SET_AH(0x80); // drive not ready (timeout)
                set_diskette_ret_status(0x80);
                SET_CF(); // error occurred
                return;
            }
            val8 = (read_byte(0x0040, 0x003e) & 0x80);
        } while ( val8 == 0 );
        
        val8 = 0; // separate asm from while() loop
        // turn off interrupts
        int_disable();
        // set 40:3e bit 7 to 0
        val8 = read_byte(0x0040, 0x003e);
        val8 &= 0x7f;
        write_byte(0x0040, 0x003e, val8);
        // check port 3f4 for accessibility to status bytes
        val8 = inb(0x3f4);
        if ( (val8 & 0xc0) != 0xc0 )
            BX_PANIC("%s: ctrl not ready\n", __func__);
        
        // read 7 return status bytes from controller and store in BDA
        for (i = 0; i < 7; ++i) {
            return_status[i] = inb(0x3f5);
            write_byte(0x0040, 0x0042 + i, return_status[i]);
        }
        
        if ( (return_status[0] & 0xc0) != 0 ) {
            if ( (return_status[1] & 0x02) != 0 ) {
                // diskette not writable.
                // AH=status code=0x03 (tried to write on write-protected disk)
                // AL=number of sectors written=0
                AX = 0x0300;
                SET_CF();
                return;
            } else {
                BX_PANIC("%s: write error\n", __func__);
            }
        }
        
        SET_AH(0);
        set_diskette_ret_status(0);
        set_diskette_current_cyl(drive, 0);
        CLEAR_CF(); // successful
        return;
    
    
    case 0x08: // read diskette drive parameters
        BX_DEBUG_INT13_FL("floppy f08\n");
        drive = GET_ELDL();
        
        if (drive > 1) {
            AX = 0;
            BX = 0;
            CX = 0;
            DX = 0;
            ES = 0;
            DI = 0;
            SET_DL(num_floppies);
            SET_CF();
            return;
        }
        
        // @todo: break out drive type determination
        drive_type = inb_cmos(0x10);
        num_floppies = 0;
        if (drive_type & 0xf0)
            num_floppies++;
        if (drive_type & 0x0f)
            num_floppies++;
        
        if (drive == 0)
            drive_type >>= 4;
        else
            drive_type &= 0x0f;
        
        SET_BH(0);
        SET_BL(drive_type);
        SET_AH(0);
        SET_AL(0);
        SET_DL(num_floppies);
        
        switch (drive_type) {
        case 0: // none
            CX = 0;
            SET_DH(0);      // max head #
            break;
        
        case 1: // 360KB, 5.25"
            CX = 0x2709;    // 40 tracks, 9 sectors
            SET_DH(1);      // max head #
            break;
        
        case 2: // 1.2MB, 5.25"
            CX = 0x4f0f;    // 80 tracks, 15 sectors
            SET_DH(1);      // max head #
            break;
        
        case 3: // 720KB, 3.5"
            CX = 0x4f09;    // 80 tracks, 9 sectors
            SET_DH(1);      // max head #
            break;
        
        case 4: // 1.44MB, 3.5"
            CX = 0x4f12;    // 80 tracks, 18 sectors
            SET_DH(1);      // max head #
            break;
        
        case 5: // 2.88MB, 3.5"
            CX = 0x4f24;    // 80 tracks, 36 sectors
            SET_DH(1);      // max head #
            break;
        
        case 6: // 160k, 5.25"
            CX = 0x2708;    // 40 tracks, 8 sectors
            SET_DH(0);      // max head #
            break;
        
        case 7: // 180k, 5.25"
            CX = 0x2709;    // 40 tracks, 9 sectors
            SET_DH(0);      // max head #
            break;
        
        case 8: // 320k, 5.25"
            CX = 0x2708;    // 40 tracks, 8 sectors
            SET_DH(1);      // max head #
            break;
        
        default: // ?
            BX_PANIC("%s: bad floppy type\n", __func__);
        }
        
        /* set es & di to point to 11 byte diskette param table in ROM */
        ES = 0xF000;    // @todo: any way to make this relocatable?
        DI = (uint16_t)&diskette_param_table;
        CLEAR_CF(); // success
        /* disk status not changed upon success */
        return;
        
    case 0x15: // read diskette drive type
        BX_DEBUG_INT13_FL("floppy f15\n");
        drive = GET_ELDL();
        if (drive > 1) {
            SET_AH(0); // only 2 drives supported
            // set_diskette_ret_status here ???
            SET_CF();
            return;
        }
        // @todo: break out drive type determination
        drive_type = inb_cmos(0x10);
        if (drive == 0)
            drive_type >>= 4;
        else
            drive_type &= 0x0f;
        CLEAR_CF(); // successful, not present
        if (drive_type==0) {
            SET_AH(0); // drive not present
        }
        else {
            SET_AH(1); // drive present, does not support change line
        }
        
        return;
    
    case 0x16: // get diskette change line status
        BX_DEBUG_INT13_FL("floppy f16\n");
        drive = GET_ELDL();
        if (drive > 1) {
            SET_AH(0x01); // invalid drive
            set_diskette_ret_status(0x01);
            SET_CF();
        return;
        }
        
        SET_AH(0x06); // change line not supported
        set_diskette_ret_status(0x06);
        SET_CF();
        return;
        
    case 0x17: // set diskette type for format(old)
        BX_DEBUG_INT13_FL("floppy f17\n");
        /* not used for 1.44M floppies */
        SET_AH(0x01); // not supported
        set_diskette_ret_status(1); /* not supported */
        SET_CF();
        return;
    
    case 0x18: // set diskette type for format(new)
        BX_DEBUG_INT13_FL("floppy f18\n");
        SET_AH(0x01); // do later
        set_diskette_ret_status(1);
        SET_CF();
        return;
    
    default:
        BX_INFO("%s: unsupported AH=%02x\n", __func__, GET_AH());
    
        // if ( (ah==0x20) || ((ah>=0x41) && (ah<=0x49)) || (ah==0x4e) ) {
        SET_AH(0x01); // ???
        set_diskette_ret_status(1);
        SET_CF();
        return;
        //   }
    }
}

#else  // #if BX_SUPPORT_FLOPPY

void BIOSCALL int13_diskette_function(disk_regs_t r)
{
    uint8_t  val8;
    
    switch ( GET_AH() ) {
    
    case 0x01: // Read Diskette Status
        CLEAR_CF();
        val8 = read_byte(0x0000, 0x0441);
        SET_AH(val8);
        if (val8) {
            SET_CF();
        }
        return;
    
    default:
        SET_CF();
        write_byte(0x0000, 0x0441, 0x01);
        SET_AH(0x01);
    }
}

#endif  // #if BX_SUPPORT_FLOPPY

#if 0
void determine_floppy_media(uint16_t drive)
{
    uint8_t     val8, DOR, ctrl_info;
    
    ctrl_info = read_byte(0x0040, 0x008F);
    if (drive==1)
        ctrl_info >>= 4;
    else
        ctrl_info &= 0x0f;

#if 0
    if (drive == 0) {
        DOR = 0x1c; // DOR: drive0 motor on, DMA&int enabled, normal op, drive select 0
    }
    else {
        DOR = 0x2d; // DOR: drive1 motor on, DMA&int enabled, normal op, drive select 1
    }
#endif

    if ( (ctrl_info & 0x04) != 0x04 ) {
        // Drive not determined means no drive exists, done.
        return;
    }

#if 0
    // check Main Status Register for readiness
    val8 = inb(0x03f4) & 0x80; // Main Status Register
    if (val8 != 0x80)
    BX_PANIC("d_f_m: MRQ bit not set\n");
    
    // change line
    
    // existing BDA values
    
    // turn on drive motor
    outb(0x03f2, DOR); // Digital Output Register
    //
#endif
    BX_PANIC("d_f_m: OK so far\n");
}
#endif

