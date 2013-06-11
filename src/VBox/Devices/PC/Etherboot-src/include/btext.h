/*
 * This file describes the structure passed from the BootX application
 * (for MacOS) when it is used to boot Linux.
 *
 * Written by Benjamin Herrenschmidt.
 *  
 * Move to LinuxBIOS by LYH  yhlu@tyan.com
 *
 */


#ifndef _BTEXT_H__
#define _BTEXT_H__

#if 1
#define u32 unsigned int
#define u16 unsigned short
#define u8 unsigned char
#endif

/* Here are the boot informations that are passed to the bootstrap
 * Note that the kernel arguments and the device tree are appended
 * at the end of this structure. */
typedef struct boot_infos
{

    /* NEW (vers. 2) this holds the current _logical_ base addr of
       the frame buffer (for use by early boot message) */
    u8*       logicalDisplayBase;


    /* Some infos about the current MacOS display */
    u32       dispDeviceRect[4];       /* left,top,right,bottom */
    u32       dispDeviceDepth;         /* (8, 16 or 32) */
    u8*       dispDeviceBase;          /* base address (physical) */
    u32       dispDeviceRowBytes;      /* rowbytes (in bytes) */
    u32       dispDeviceColorsOffset;  /* Colormap (8 bits only) or 0 (*) */


    /* The framebuffer size (optional, currently 0) */
    u32       frameBufferSize;         /* Represents a max size, can be 0. */


} boot_infos_t;

/* (*) The format of the colormap is 256 * 3 * 2 bytes. Each color index is represented
 * by 3 short words containing a 16 bits (unsigned) color component.
 * Later versions may contain the gamma table for direct-color devices here.
 */
#define BOOTX_COLORTABLE_SIZE    (256UL*3UL*2UL)


/*
 * Definitions for using the procedures in btext.c.
 *
 * Benjamin Herrenschmidt <benh@kernel.crashing.org>
 */

extern void btext_clearscreen(void);

extern boot_infos_t disp_bi;
extern u32 boot_text_mapped;

void btext_setup_display(u32 width, u32 height, u32 depth, u32 pitch,
			 unsigned long address);
void map_boot_text(void);

void btext_drawchar(char c);
void btext_drawstring(const char *str);
void btext_drawhex(u32 v);

void btext_putc(int c);

void btext_init(void);

#endif /* _BTEXT_H */
