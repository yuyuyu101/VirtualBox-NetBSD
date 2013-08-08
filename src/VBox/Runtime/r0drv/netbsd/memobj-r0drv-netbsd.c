/* $Id: memobj-r0drv-freebsd.c $ */
/** @file
 * IPRT - Ring-0 Memory Objects, NetBSD.
 */

/*******************************************************************************
*   Header Files                                                               *
*******************************************************************************/
#include "the-netbsd-kernel.h"

#include <iprt/memobj.h>
#include <iprt/mem.h>
#include <iprt/err.h>
#include <iprt/assert.h>
#include <iprt/log.h>
#include <iprt/param.h>
#include <iprt/process.h>
#include "internal/memobj.h"


/*******************************************************************************
*   Structures and Typedefs                                                    *
*******************************************************************************/
/**
 * The NetBSD version of the memory object structure.
 */
typedef struct RTR0MEMOBJNETBSD
{
    /** The core structure. */
    RTR0MEMOBJINTERNAL  Core;
    int                 is_uvm;
    size_t              size;
    struct pglist       pglist;
} RTR0MEMOBJNETBSD, *PRTR0MEMOBJNETBSD;


MALLOC_DEFINE(M_IPRTMOBJ, "iprtmobj", "IPRT - R0MemObj");

typedef struct vm_map* vm_map_t;

/**
 * Gets the virtual memory map the specified object is mapped into.
 *
 * @returns VM map handle on success, NULL if no map.
 * @param   pMem                The memory object.
 */
static vm_map_t rtR0MemObjNetBSDGetMap(PRTR0MEMOBJINTERNAL pMem)
{
    switch (pMem->enmType)
    {
        case RTR0MEMOBJTYPE_PAGE:
        case RTR0MEMOBJTYPE_LOW:
        case RTR0MEMOBJTYPE_CONT:
            return kernel_map;

        case RTR0MEMOBJTYPE_PHYS:
        case RTR0MEMOBJTYPE_PHYS_NC:
            return NULL; /* pretend these have no mapping atm. */

        case RTR0MEMOBJTYPE_LOCK:
            return pMem->u.Lock.R0Process == NIL_RTR0PROCESS
                ? kernel_map
                : &((struct proc *)pMem->u.Lock.R0Process)->p_vmspace->vm_map;

        case RTR0MEMOBJTYPE_RES_VIRT:
            return pMem->u.ResVirt.R0Process == NIL_RTR0PROCESS
                ? kernel_map
                : &((struct proc *)pMem->u.ResVirt.R0Process)->p_vmspace->vm_map;

        case RTR0MEMOBJTYPE_MAPPING:
            return pMem->u.Mapping.R0Process == NIL_RTR0PROCESS
                ? kernel_map
                : &((struct proc *)pMem->u.Mapping.R0Process)->p_vmspace->vm_map;

        default:
            return NULL;
    }
}


DECLHIDDEN(int) rtR0MemObjNativeFree(RTR0MEMOBJ pMem)
{
    PRTR0MEMOBJNETBSD pMemNetBSD = (PRTR0MEMOBJNETBSD)pMem;
    int rc;

    switch (pMemNetBSD->Core.enmType)
    {
        case RTR0MEMOBJTYPE_PAGE:
        {
            kmem_free(pMemNetBSD->Core.pv, pMemNetBSD->size);
            break;
        }
        case RTR0MEMOBJTYPE_LOW:
        case RTR0MEMOBJTYPE_CONT:
        case RTR0MEMOBJTYPE_LOCK:
        case RTR0MEMOBJTYPE_RES_VIRT:
        case RTR0MEMOBJTYPE_MAPPING:
        case RTR0MEMOBJTYPE_PHYS:
        case RTR0MEMOBJTYPE_PHYS_NC:
        {
            
            break;
        }

        default:
            AssertMsgFailed(("enmType=%d\n", pMemNetBSD->Core.enmType));
            return VERR_INTERNAL_ERROR;
    }

    return VINF_SUCCESS;
}


void *rtR0MemObjNetBSDContigPhysAllocHelper(struct pglist *pglist, size_t cPages, paddr_t VmPhysAddrHigh)
{
    int error;

    error = uvm_pglistalloc(cPages * PAGE_SIZE, 0, VmPhysAddrHigh, PAGE_SIZE, 0, pglist, 1, 1);
    if (error)
        return NULL;

    /*
     * Get the physical address from the first page.
     */
    const struct vm_page * const pg = TAILQ_FIRST(pglist);
    KASSERT(pg != NULL);
    const paddr_t pa = VM_PAGE_TO_PHYS(pg);

    /*
     * We need to return a direct-mapped VA for the pa.
     */
    return (void*)pa;
}

static void *rtR0MemObjNetBSDPhysAllocHelper(struct pglist *pglist, size_t cPages, paddr_t VmPhysAddrHigh, bool fContiguous, int rcNoMem)
{
    //if (fContiguous)
    {
        void *mem = rtR0MemObjNetBSDContigPhysAllocHelper(pglist, cPages, VmPhysAddrHigh);
        if (mem)
            return mem;
        return NULL;
    }
}

static void *rtR0MemObjNetBSDAllocHelper(PRTR0MEMOBJNETBSD pMemNetBSD, bool fExecutable,
                                         paddr_t VmPhysAddrHigh, bool fContiguous, int rcNoMem)
{
    size_t      cPages = atop(pMemNetBSD->Core.cb);
    void        *rc;

    rc = rtR0MemObjNetBSDPhysAllocHelper(&pMemNetBSD->pglist, cPages, VmPhysAddrHigh, fContiguous, rcNoMem);
    if (rc)
    {
        /* Store start address */
        pMemNetBSD->Core.pv = rc;
        pMemNetBSD->is_uvm = 1;
        return VINF_SUCCESS;
    }
    return NULL;
}

DECLHIDDEN(int) rtR0MemObjNativeAllocPage(PPRTR0MEMOBJINTERNAL ppMem, size_t cb, bool fExecutable)
{
    PRTR0MEMOBJNETBSD pMemNetBSD = (PRTR0MEMOBJNETBSD)rtR0MemObjNew(sizeof(*pMemNetBSD),
                                                                       RTR0MEMOBJTYPE_PAGE, NULL, cb);
    if (!pMemNetBSD)
        return VERR_NO_MEMORY;

    void *pvMem = kmem_alloc(cb, KM_SLEEP);
    if (RT_UNLIKELY(!pvMem))
    {
        rtR0MemObjDelete(&pMemNetBSD->Core);
        return VERR_NO_PAGE_MEMORY;
    }

    pMemNetBSD->Core.pv = pvMem;
    pMemNetBSD->is_uvm = 0;
    pMemNetBSD->size = cb;
    *ppMem = &pMemNetBSD->Core;
    return VINF_SUCCESS;
}


DECLHIDDEN(int) rtR0MemObjNativeAllocLow(PPRTR0MEMOBJINTERNAL ppMem, size_t cb, bool fExecutable)
{
    PRTR0MEMOBJNETBSD pMemNetBSD = (PRTR0MEMOBJNETBSD)rtR0MemObjNew(sizeof(*pMemNetBSD),
                                                                       RTR0MEMOBJTYPE_LOW, NULL, cb);
    if (!pMemNetBSD)
        return VERR_NO_MEMORY;

    void *rc = rtR0MemObjNetBSDAllocHelper(pMemNetBSD, fExecutable, _4G - 1, false, VERR_NO_LOW_MEMORY);
    if (!rc)
    {
        rtR0MemObjDelete(&pMemNetBSD->Core);
        return VERR_NO_MEMORY;
    }

    *ppMem = &pMemNetBSD->Core;
    return VINF_SUCCESS;
}


DECLHIDDEN(int) rtR0MemObjNativeAllocCont(PPRTR0MEMOBJINTERNAL ppMem, size_t cb, bool fExecutable)
{
    PRTR0MEMOBJNETBSD pMemNetBSD = (PRTR0MEMOBJNETBSD)rtR0MemObjNew(sizeof(*pMemNetBSD),
                                                                       RTR0MEMOBJTYPE_CONT, NULL, cb);
    if (!pMemNetBSD)
        return VERR_NO_MEMORY;

    void *rc = rtR0MemObjNetBSDAllocHelper(pMemNetBSD, fExecutable, _4G - 1, true, VERR_NO_CONT_MEMORY);
    if (!rc)
    {
        rtR0MemObjDelete(&pMemNetBSD->Core);
        return VERR_NO_MEMORY;
    }

    pMemNetBSD->Core.u.Cont.Phys = pMemNetBSD->Core.pv;
    *ppMem = &pMemNetBSD->Core;
    return VINF_SUCCESS;
}


static int rtR0MemObjNetBSDAllocPhysPages(PPRTR0MEMOBJINTERNAL ppMem, RTR0MEMOBJTYPE enmType,
                                           size_t cb,
                                           RTHCPHYS PhysHighest, size_t uAlignment,
                                           bool fContiguous, int rcNoMem)
{
    uint32_t   cPages = atop(cb);
    paddr_t VmPhysAddrHigh;

    /* create the object. */
    PRTR0MEMOBJNETBSD pMemNetBSD = (PRTR0MEMOBJNETBSD)rtR0MemObjNew(sizeof(*pMemNetBSD),
                                                                       enmType, NULL, cb);
    if (!pMemNetBSD)
        return VERR_NO_MEMORY;

    if (PhysHighest != NIL_RTHCPHYS)
        VmPhysAddrHigh = PhysHighest;
    else
        VmPhysAddrHigh = ~(paddr_t)0;

    void *rc = rtR0MemObjNetBSDPhysAllocHelper(&pMemNetBSD->pglist, cPages, VmPhysAddrHigh, fContiguous, rcNoMem);
    if (rc)
    {
        if (fContiguous)
        {
            Assert(enmType == RTR0MEMOBJTYPE_PHYS);
            pMemNetBSD->Core.u.Phys.PhysBase = rc;
            pMemNetBSD->Core.u.Phys.fAllocated = true;
        }
        *ppMem = &pMemNetBSD->Core;
    }
    else
    {
        rtR0MemObjDelete(&pMemNetBSD->Core);
    }

    return VINF_SUCCESS;
}


DECLHIDDEN(int) rtR0MemObjNativeAllocPhys(PPRTR0MEMOBJINTERNAL ppMem, size_t cb, RTHCPHYS PhysHighest, size_t uAlignment)
{
    return rtR0MemObjNetBSDAllocPhysPages(ppMem, RTR0MEMOBJTYPE_PHYS, cb, PhysHighest, uAlignment, true, VERR_NO_MEMORY);
}


DECLHIDDEN(int) rtR0MemObjNativeAllocPhysNC(PPRTR0MEMOBJINTERNAL ppMem, size_t cb, RTHCPHYS PhysHighest)
{
    return rtR0MemObjNetBSDAllocPhysPages(ppMem, RTR0MEMOBJTYPE_PHYS_NC, cb, PhysHighest, PAGE_SIZE, false, VERR_NO_PHYS_MEMORY);
}


DECLHIDDEN(int) rtR0MemObjNativeEnterPhys(PPRTR0MEMOBJINTERNAL ppMem, RTHCPHYS Phys, size_t cb, uint32_t uCachePolicy)
{
    AssertReturn(uCachePolicy == RTMEM_CACHE_POLICY_DONT_CARE, VERR_NOT_SUPPORTED);

    /* create the object. */
    PRTR0MEMOBJNETBSD pMemNetBSD = (PRTR0MEMOBJNETBSD)rtR0MemObjNew(sizeof(*pMemNetBSD), RTR0MEMOBJTYPE_PHYS, NULL, cb);
    if (!pMemNetBSD)
        return VERR_NO_MEMORY;

    /* there is no allocation here, it needs to be mapped somewhere first. */
    pMemNetBSD->Core.u.Phys.fAllocated = false;
    pMemNetBSD->Core.u.Phys.PhysBase = Phys;
    pMemNetBSD->Core.u.Phys.uCachePolicy = uCachePolicy;
    *ppMem = &pMemNetBSD->Core;
    return VINF_SUCCESS;
}


DECLHIDDEN(int) rtR0MemObjNativeLockUser(PPRTR0MEMOBJINTERNAL ppMem, RTR3PTR R3Ptr, size_t cb, uint32_t fAccess, RTR0PROCESS R0Process)
{
    return VERR_NOT_SUPPORTED;
}


DECLHIDDEN(int) rtR0MemObjNativeLockKernel(PPRTR0MEMOBJINTERNAL ppMem, void *pv, size_t cb, uint32_t fAccess)
{
    return VERR_NOT_SUPPORTED;
}

DECLHIDDEN(int) rtR0MemObjNativeReserveKernel(PPRTR0MEMOBJINTERNAL ppMem, void *pvFixed, size_t cb, size_t uAlignment)
{
    return VERR_NOT_SUPPORTED;
}


DECLHIDDEN(int) rtR0MemObjNativeReserveUser(PPRTR0MEMOBJINTERNAL ppMem, RTR3PTR R3PtrFixed, size_t cb, size_t uAlignment, RTR0PROCESS R0Process)
{
    return VERR_NOT_SUPPORTED;
}


DECLHIDDEN(int) rtR0MemObjNativeMapKernel(PPRTR0MEMOBJINTERNAL ppMem, RTR0MEMOBJ pMemToMap, void *pvFixed, size_t uAlignment,
                                          unsigned fProt, size_t offSub, size_t cbSub)
{
    return VERR_NOT_SUPPORTED;
}


DECLHIDDEN(int) rtR0MemObjNativeMapUser(PPRTR0MEMOBJINTERNAL ppMem, RTR0MEMOBJ pMemToMap, RTR3PTR R3PtrFixed, size_t uAlignment,
                                        unsigned fProt, RTR0PROCESS R0Process)
{
    return VERR_NOT_SUPPORTED;
}


DECLHIDDEN(int) rtR0MemObjNativeProtect(PRTR0MEMOBJINTERNAL pMem, size_t offSub, size_t cbSub, uint32_t fProt)
{
    vm_prot_t          ProtectionFlags = 0;
    voff_t        AddrStart       = (uintptr_t)pMem->pv + offSub;
    vm_map_t           pVmMap          = rtR0MemObjNetBSDGetMap(pMem);

    if (!pVmMap)
        return VERR_NOT_SUPPORTED;

    if ((fProt & RTMEM_PROT_NONE) == RTMEM_PROT_NONE)
        ProtectionFlags = VM_PROT_NONE;
    if ((fProt & RTMEM_PROT_READ) == RTMEM_PROT_READ)
        ProtectionFlags |= VM_PROT_READ;
    if ((fProt & RTMEM_PROT_WRITE) == RTMEM_PROT_WRITE)
        ProtectionFlags |= VM_PROT_WRITE;
    if ((fProt & RTMEM_PROT_EXEC) == RTMEM_PROT_EXEC)
        ProtectionFlags |= VM_PROT_EXECUTE;

    int error = uvm_vslock(pVmMap, AddrStart, cbSub, ProtectionFlags);
    if (!error)
        return VINF_SUCCESS;

    return VERR_NOT_SUPPORTED;
}


DECLHIDDEN(RTHCPHYS) rtR0MemObjNativeGetPagePhysAddr(PRTR0MEMOBJINTERNAL pMem, size_t iPage)
{
    PRTR0MEMOBJNETBSD pMemNetBSD = (PRTR0MEMOBJNETBSD)pMem;

    switch (pMemNetBSD->Core.enmType)
    {
        case RTR0MEMOBJTYPE_LOCK:
        case RTR0MEMOBJTYPE_MAPPING:
        case RTR0MEMOBJTYPE_PAGE:
        {
            return pMemNetBSD->Core.pv;
        }

        case RTR0MEMOBJTYPE_LOW:
        case RTR0MEMOBJTYPE_PHYS_NC:
        case RTR0MEMOBJTYPE_PHYS:
            return pMemNetBSD->Core.u.Cont.Phys + ptoa(iPage);

        case RTR0MEMOBJTYPE_CONT:
            return pMemNetBSD->Core.u.Phys.PhysBase + ptoa(iPage);

        case RTR0MEMOBJTYPE_RES_VIRT:
        default:
            return NIL_RTHCPHYS;
    }
}
