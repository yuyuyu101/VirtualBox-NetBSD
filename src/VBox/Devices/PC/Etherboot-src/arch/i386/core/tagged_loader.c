#include "realmode.h"
#include "segoff.h"

struct segheader
{
	unsigned char length;
	unsigned char vendortag;
	unsigned char reserved;
	unsigned char flags;
	unsigned long loadaddr;
	unsigned long imglength;
	unsigned long memlength;
};

struct imgheader
{
	unsigned long magic;
	unsigned long length;			/* and flags */
	union
	{
		segoff_t segoff;
		unsigned long location;
	} u;
	unsigned long execaddr;
};

/* Keep all context about loaded image in one place */
static struct tagged_context
{
	struct imgheader	img;			/* copy of header */
	unsigned long		linlocation;		/* addr of header */
	unsigned long		last0, last1;		/* of prev segment */
	unsigned long		segaddr, seglen;	/* of current segment */
	unsigned char		segflags;
	unsigned char		first;
	unsigned long		curaddr;
} tctx;

#define	TAGGED_PROGRAM_RETURNS	(tctx.img.length & 0x00000100)	/* bit 8 */
#define	LINEAR_EXEC_ADDR	(tctx.img.length & 0x80000000)	/* bit 31 */

static sector_t tagged_download(unsigned char *data, unsigned int len, int eof);
void xstart16 (unsigned long execaddr, segoff_t location,
	       void *bootp);

static inline os_download_t tagged_probe(unsigned char *data, unsigned int len)
{
	struct segheader	*sh;
	unsigned long loc;
	if (*((uint32_t *)data) != 0x1B031336L) {
		return 0;
	}
	printf("(NBI)");
	/* If we don't have enough data give up */
	if (len < 512)
		return dead_download;
	/* Zero all context info */
	memset(&tctx, 0, sizeof(tctx));
	/* Copy first 4 longwords */
	memcpy(&tctx.img, data, sizeof(tctx.img));
	/* Memory location where we are supposed to save it */
	tctx.segaddr = tctx.linlocation = 
		((tctx.img.u.segoff.segment) << 4) + tctx.img.u.segoff.offset;
	if (!prep_segment(tctx.segaddr, tctx.segaddr + 512, tctx.segaddr + 512,
			  0, 512)) {
		return dead_download;
	}
	/* Now verify the segments we are about to load */
	loc = 512;
	for(sh = (struct segheader *)(data
				      + ((tctx.img.length & 0x0F) << 2)
				      + ((tctx.img.length & 0xF0) >> 2) ); 
		(sh->length > 0) && ((unsigned char *)sh < data + 512); 
		sh = (struct segheader *)((unsigned char *)sh
					  + ((sh->length & 0x0f) << 2) + ((sh->length & 0xf0) >> 2)) ) {
		if (!prep_segment(
			sh->loadaddr,
			sh->loadaddr + sh->imglength,
			sh->loadaddr + sh->imglength,
			loc, loc + sh->imglength)) {
			return dead_download;
		}
		loc = loc + sh->imglength;
		if (sh->flags & 0x04) 
			break;
	}
	if (!(sh->flags & 0x04))
		return dead_download;
	/* Grab a copy */
	memcpy(phys_to_virt(tctx.segaddr), data, 512);
	/* Advance to first segment descriptor */
	tctx.segaddr += ((tctx.img.length & 0x0F) << 2)
		+ ((tctx.img.length & 0xF0) >> 2);
	/* Remember to skip the first 512 data bytes */
	tctx.first = 1;
	
	return tagged_download;

}
static sector_t tagged_download(unsigned char *data, unsigned int len, int eof)
{
	int	i;

	if (tctx.first) {
		tctx.first = 0;
		if (len > 512) {
			len -= 512;
			data += 512;
			/* and fall through to deal with rest of block */
		} else 
			return 0;
	}
	for (;;) {
		if (len == 0) /* Detect truncated files */
			eof = 0;
		while (tctx.seglen == 0) {
			struct segheader	sh;
			if (tctx.segflags & 0x04) {
				done(1);
				if (LINEAR_EXEC_ADDR) {
					int result;
					/* no gateA20_unset for PM call */
					result = xstart32(tctx.img.execaddr,
						virt_to_phys(&loaderinfo),
						tctx.linlocation,
						virt_to_phys(&bootp_data));
					printf("Secondary program returned %d\n",
						result);
					if (!TAGGED_PROGRAM_RETURNS) {
						/* We shouldn't have returned */
						result = -2;
					}
					if (result == 0)
						result = -2;
					longjmp(restart_etherboot, result);
						
				} else {
					gateA20_unset();
					xstart16(tctx.img.execaddr, 
						 tctx.img.u.segoff,
						 &bootp_data);
					longjmp(restart_etherboot, -2);
				}
			}
			sh = *((struct segheader *)phys_to_virt(tctx.segaddr));
			tctx.seglen = sh.imglength;
			if ((tctx.segflags = sh.flags & 0x03) == 0)
				tctx.curaddr = sh.loadaddr;
			else if (tctx.segflags == 0x01)
				tctx.curaddr = tctx.last1 + sh.loadaddr;
			else if (tctx.segflags == 0x02)
				tctx.curaddr = (Address)(meminfo.memsize * 1024L
						    + 0x100000L)
					- sh.loadaddr;
			else
				tctx.curaddr = tctx.last0 - sh.loadaddr;
			tctx.last1 = (tctx.last0 = tctx.curaddr) + sh.memlength;
			tctx.segflags = sh.flags;
			tctx.segaddr += ((sh.length & 0x0F) << 2)
				+ ((sh.length & 0xF0) >> 2);
			/* Avoid lock-up */
			if ( sh.length == 0 ) longjmp(restart_etherboot, -2); 
		}
		if ((len <= 0) && !eof)
			break;
		i = (tctx.seglen > len) ? len : tctx.seglen;
		memcpy(phys_to_virt(tctx.curaddr), data, i);
		tctx.seglen -= i;
		tctx.curaddr += i;
		len -= i;
		data += i;
	} 
	return 0;
}

void xstart16 (unsigned long execaddr, segoff_t location,
	       void *bootp) {
	struct {
		segoff_t execaddr;
		segoff_t location;
		segoff_t bootp;
	} PACKED in_stack;

	/* AFAICT, execaddr is actually already a segment:offset */
	*((unsigned long *)&in_stack.execaddr) = execaddr;
	in_stack.location = location;
	in_stack.bootp.segment = SEGMENT(bootp);
	in_stack.bootp.offset = OFFSET(bootp);

	RM_FRAGMENT(rm_xstart16,
		"popl %eax\n\t"	/* Calculated lcall */
		"pushw %cs\n\t" 
		"call 1f\n1:\tpopw %bp\n\t" 
		"leaw (2f-1b)(%bp), %bx\n\t" 
		"pushw %bx\n\t" 
		"pushl %eax\n\t"
		"lret\n2:\n\t"
	);
       
	real_call ( rm_xstart16, &in_stack, NULL );
}
