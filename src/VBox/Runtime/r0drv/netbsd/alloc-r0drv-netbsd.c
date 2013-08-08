/*******************************************************************************
*   Header Files                                                               *
*******************************************************************************/
#include "the-netbsd-kernel.h"
#include "internal/iprt.h"
#include <iprt/mem.h>

#include <iprt/assert.h>
#include <iprt/err.h>
#include <iprt/param.h>

#include "r0drv/alloc-r0drv.h"


/*******************************************************************************
*   Global Variables                                                           *
*******************************************************************************/
/* These two statements will define two globals and add initializers
   and destructors that will be called at load/unload time (I think). */
MALLOC_DEFINE(M_IPRTHEAP, "iprtheap", "IPRT - heap");
MALLOC_DEFINE(M_IPRTCONT, "iprtcont", "IPRT - contiguous");


DECLHIDDEN(int) rtR0MemAllocEx(size_t cb, uint32_t fFlags, PRTMEMHDR *ppHdr)
{
    size_t      cbAllocated = cb;
    PRTMEMHDR   pHdr        = NULL;

    {
        pHdr = (PRTMEMHDR)malloc(cb + sizeof(RTMEMHDR), M_IPRTHEAP,
                                 fFlags & RTMEMHDR_FLAG_ZEROED ? M_NOWAIT | M_ZERO : M_NOWAIT);
    }

    if (RT_UNLIKELY(!pHdr))
        return VERR_NO_MEMORY;

    pHdr->u32Magic   = RTMEMHDR_MAGIC;
    pHdr->fFlags     = fFlags;
    pHdr->cb         = cbAllocated;
    pHdr->cbReq      = cb;

    *ppHdr = pHdr;
    return VINF_SUCCESS;
}


DECLHIDDEN(void) rtR0MemFree(PRTMEMHDR pHdr)
{
    pHdr->u32Magic += 1;

    free(pHdr, M_IPRTHEAP);
}
