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

RTR0DECL(void) RTMemContFree(void *pv, size_t cb)
{
    if (pv)
    {
        AssertMsg(!((uintptr_t)pv & PAGE_OFFSET_MASK), ("pv=%p\n", pv));

    }
}

RTR0DECL(void *) RTMemContAlloc(PRTCCPHYS pPhys, size_t cb)
{
    /*
     * Validate input.
     */
    AssertPtr(pPhys);
    Assert(cb > 0);

    size_t cPages = atop(cb);
    int error;
    struct uvm_object *vm;
    struct vm_page *page;

    vm = uao_create(cb, 0);
    page = uvm_pagealloc(vm, 0, NULL, 0);

    /*
     * Get the physical address from the first page.
     */
    const paddr_t pa = VM_PAGE_TO_PHYS(page);
    *pPhys = pa;

    /*
     * We need to return a direct-mapped VA for the pa.
     */
    return (void*)pa;

}
