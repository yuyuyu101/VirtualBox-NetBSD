/** @file
 * Base class for an host-guest service.
 */

/*
 * Copyright (C) 2011-2012 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 *
 * The contents of this file may alternatively be used under the terms
 * of the Common Development and Distribution License Version 1.0
 * (CDDL) only, as it comes in the "COPYING.CDDL" file of the
 * VirtualBox OSE distribution, in which case the provisions of the
 * CDDL are applicable instead of those of the GPL.
 *
 * You may elect to license modified versions of this file under the
 * terms and conditions of either the GPL or the CDDL or both.
 */


#ifndef ___VBox_HostService_Service_h
#define ___VBox_HostService_Service_h

#include <VBox/log.h>
#include <VBox/hgcmsvc.h>
#include <iprt/assert.h>
#include <iprt/alloc.h>
#include <iprt/cpp/utils.h>

#include <memory>  /* for auto_ptr */

namespace HGCM
{

class Message
{
public:
    Message(uint32_t uMsg, uint32_t cParms, VBOXHGCMSVCPARM aParms[])
      : m_uMsg(0)
      , m_cParms(0)
      , m_paParms(0)
    {
        setData(uMsg, cParms, aParms);
    }
    ~Message()
    {
        cleanup();
    }

    uint32_t message() const { return m_uMsg; }
    uint32_t paramsCount() const { return m_cParms; }

    int getData(uint32_t uMsg, uint32_t cParms, VBOXHGCMSVCPARM aParms[]) const
    {
        if (m_uMsg != uMsg)
        {
            LogFlowFunc(("Message type does not match (%u (buffer), %u (guest))\n",
                         m_uMsg, uMsg));
            return VERR_INVALID_PARAMETER;
        }
        if (m_cParms != cParms)
        {
            LogFlowFunc(("Parameter count does not match (%u (buffer), %u (guest))\n",
                         m_cParms, cParms));
            return VERR_INVALID_PARAMETER;
        }

        int rc = copyParms(cParms, m_paParms, &aParms[0], false /* fCreatePtrs */);

//        if (RT_FAILURE(rc))
//            cleanup(aParms);
        return rc;
    }
    int setData(uint32_t uMsg, uint32_t cParms, VBOXHGCMSVCPARM aParms[])
    {
        AssertReturn(cParms < 256, VERR_INVALID_PARAMETER);
        AssertPtrNullReturn(aParms, VERR_INVALID_PARAMETER);

        /* Cleanup old messages. */
        cleanup();

        m_uMsg = uMsg;
        m_cParms = cParms;

        if (cParms > 0)
        {
            m_paParms = (VBOXHGCMSVCPARM*)RTMemAllocZ(sizeof(VBOXHGCMSVCPARM) * m_cParms);
            if (!m_paParms)
                return VERR_NO_MEMORY;
        }

        int rc = copyParms(cParms, &aParms[0], m_paParms, true /* fCreatePtrs */);

        if (RT_FAILURE(rc))
            cleanup();

        return rc;
    }

    int getParmU32Info(uint32_t iParm, uint32_t *pu32Info) const
    {
        AssertPtrNullReturn(pu32Info, VERR_INVALID_PARAMETER);
        AssertReturn(iParm < m_cParms, VERR_INVALID_PARAMETER);
        AssertReturn(m_paParms[iParm].type == VBOX_HGCM_SVC_PARM_32BIT, VERR_INVALID_PARAMETER);

        *pu32Info = m_paParms[iParm].u.uint32;

        return VINF_SUCCESS;
    }
    int getParmU64Info(uint32_t iParm, uint64_t *pu64Info) const
    {
        AssertPtrNullReturn(pu64Info, VERR_INVALID_PARAMETER);
        AssertReturn(iParm < m_cParms, VERR_INVALID_PARAMETER);
        AssertReturn(m_paParms[iParm].type == VBOX_HGCM_SVC_PARM_64BIT, VERR_INVALID_PARAMETER);

        *pu64Info = m_paParms[iParm].u.uint64;

        return VINF_SUCCESS;
    }
    int getParmPtrInfo(uint32_t iParm, void **ppvAddr, uint32_t *pcSize) const
    {
        AssertPtrNullReturn(ppvAddr, VERR_INVALID_PARAMETER);
        AssertPtrNullReturn(pcSize, VERR_INVALID_PARAMETER);
        AssertReturn(iParm < m_cParms, VERR_INVALID_PARAMETER);
        AssertReturn(m_paParms[iParm].type == VBOX_HGCM_SVC_PARM_PTR, VERR_INVALID_PARAMETER);

        *ppvAddr = m_paParms[iParm].u.pointer.addr;
        *pcSize = m_paParms[iParm].u.pointer.size;

        return VINF_SUCCESS;
    }

    int copyParms(uint32_t cParms, PVBOXHGCMSVCPARM paParmsSrc, PVBOXHGCMSVCPARM paParmsDst, bool fCreatePtrs) const
    {
        int rc = VINF_SUCCESS;
        for (uint32_t i = 0; i < cParms; ++i)
        {
            paParmsDst[i].type = paParmsSrc[i].type;
            switch (paParmsSrc[i].type)
            {
                case VBOX_HGCM_SVC_PARM_32BIT:
                {
                    paParmsDst[i].u.uint32 = paParmsSrc[i].u.uint32;
                    break;
                }
                case VBOX_HGCM_SVC_PARM_64BIT:
                {
                    paParmsDst[i].u.uint64 = paParmsSrc[i].u.uint64;
                    break;
                }
                case VBOX_HGCM_SVC_PARM_PTR:
                {
                    /* Do we have to recreate the memory? */
                    if (fCreatePtrs)
                    {
                        /* Yes, do so. */
                        paParmsDst[i].u.pointer.size = paParmsSrc[i].u.pointer.size;
                        if (paParmsDst[i].u.pointer.size > 0)
                        {
                            paParmsDst[i].u.pointer.addr = RTMemAlloc(paParmsDst[i].u.pointer.size);
                            if (!paParmsDst[i].u.pointer.addr)
                            {
                                rc = VERR_NO_MEMORY;
                                break;
                            }
                        }
                    }
                    else
                    {
                        /* No, but we have to check if there is enough room. */
                        if (paParmsDst[i].u.pointer.size < paParmsSrc[i].u.pointer.size)
                            rc = VERR_BUFFER_OVERFLOW;
                    }
                    if (   paParmsDst[i].u.pointer.addr
                        && paParmsSrc[i].u.pointer.size > 0
                        && paParmsDst[i].u.pointer.size > 0)
                        memcpy(paParmsDst[i].u.pointer.addr,
                               paParmsSrc[i].u.pointer.addr,
                               RT_MIN(paParmsDst[i].u.pointer.size, paParmsSrc[i].u.pointer.size));
                    break;
                }
                default:
                {
                    AssertMsgFailed(("Unknown HGCM type %u\n", paParmsSrc[i].type));
                    rc = VERR_INVALID_PARAMETER;
                    break;
                }
            }
            if (RT_FAILURE(rc))
                break;
        }
        return rc;
    }

    void cleanup()
    {
        if (m_paParms)
        {
            for (uint32_t i = 0; i < m_cParms; ++i)
            {
                switch (m_paParms[i].type)
                {
                    case VBOX_HGCM_SVC_PARM_PTR:
                        if (m_paParms[i].u.pointer.size)
                            RTMemFree(m_paParms[i].u.pointer.addr);
                        break;
                }
            }
            RTMemFree(m_paParms);
            m_paParms = 0;
        }
        m_cParms = 0;
        m_uMsg = 0;
    }

protected:
    uint32_t m_uMsg;
    uint32_t m_cParms;
    PVBOXHGCMSVCPARM m_paParms;
};

class Client
{
public:
    Client(uint32_t uClientId, VBOXHGCMCALLHANDLE hHandle, uint32_t uMsg, uint32_t cParms, VBOXHGCMSVCPARM aParms[])
      : m_uClientId(uClientId)
      , m_hHandle(hHandle)
      , m_uMsg(uMsg)
      , m_cParms(cParms)
      , m_paParms(aParms) {}

    VBOXHGCMCALLHANDLE handle() const { return m_hHandle; }
    uint32_t message() const { return m_uMsg; }
    uint32_t clientId() const { return m_uClientId; }

    int addMessageInfo(uint32_t uMsg, uint32_t cParms)
    {
        if (m_cParms != 3)
            return VERR_INVALID_PARAMETER;

        m_paParms[0].setUInt32(uMsg);
        m_paParms[1].setUInt32(cParms);

        return VINF_SUCCESS;
    }
    int addMessageInfo(const Message *pMessage)
    {
        if (m_cParms != 3)
            return VERR_INVALID_PARAMETER;

        m_paParms[0].setUInt32(pMessage->message());
        m_paParms[1].setUInt32(pMessage->paramsCount());

        return VINF_SUCCESS;
    }
    int addMessage(const Message *pMessage)
    {
        return pMessage->getData(m_uMsg, m_cParms, m_paParms);
    }
private:
    uint32_t m_uClientId;
    VBOXHGCMCALLHANDLE m_hHandle;
    uint32_t m_uMsg;
    uint32_t m_cParms;
    PVBOXHGCMSVCPARM m_paParms;
};

template <class T>
class AbstractService: public RTCNonCopyable
{
public:
    /**
     * @copydoc VBOXHGCMSVCLOAD
     */
    static DECLCALLBACK(int) svcLoad(VBOXHGCMSVCFNTABLE *pTable)
    {
        LogFlowFunc(("ptable = %p\n", pTable));
        int rc = VINF_SUCCESS;

        if (!VALID_PTR(pTable))
            rc = VERR_INVALID_PARAMETER;
        else
        {
            LogFlowFunc(("ptable->cbSize = %d, ptable->u32Version = 0x%08X\n", pTable->cbSize, pTable->u32Version));

            if (   pTable->cbSize != sizeof (VBOXHGCMSVCFNTABLE)
                || pTable->u32Version != VBOX_HGCM_SVC_VERSION)
                rc = VERR_VERSION_MISMATCH;
            else
            {
                std::auto_ptr<AbstractService> apService;
                /* No exceptions may propagate outside. */
                try
                {
                    apService = std::auto_ptr<AbstractService>(new T(pTable->pHelpers));
                } catch (int rcThrown)
                {
                    rc = rcThrown;
                } catch (...)
                {
                    rc = VERR_UNRESOLVED_ERROR;
                }

                if (RT_SUCCESS(rc))
                {
                    /*
                     * We don't need an additional client data area on the host,
                     * because we're a class which can have members for that :-).
                     */
                    pTable->cbClient = 0;

                    /* These functions are mandatory */
                    pTable->pfnUnload             = svcUnload;
                    pTable->pfnConnect            = svcConnect;
                    pTable->pfnDisconnect         = svcDisconnect;
                    pTable->pfnCall               = svcCall;
                    /* Clear obligatory functions. */
                    pTable->pfnHostCall           = NULL;
                    pTable->pfnSaveState          = NULL;
                    pTable->pfnLoadState          = NULL;
                    pTable->pfnRegisterExtension  = NULL;

                    /* Let the service itself initialize. */
                    rc = apService->init(pTable);

                    /* Only on success stop the auto release of the auto_ptr. */
                    if (RT_SUCCESS(rc))
                        pTable->pvService = apService.release();
                }
            }
        }

        LogFlowFunc(("returning %Rrc\n", rc));
        return rc;
    }
    virtual ~AbstractService() {};

protected:
    explicit AbstractService(PVBOXHGCMSVCHELPERS pHelpers)
        : m_pHelpers(pHelpers)
        , m_pfnHostCallback(NULL)
        , m_pvHostData(NULL)
    {}
    virtual int  init(VBOXHGCMSVCFNTABLE *ptable) { return VINF_SUCCESS; }
    virtual int  uninit() { return VINF_SUCCESS; }
    virtual int  clientConnect(uint32_t u32ClientID, void *pvClient) = 0;
    virtual int  clientDisconnect(uint32_t u32ClientID, void *pvClient) = 0;
    virtual void guestCall(VBOXHGCMCALLHANDLE callHandle, uint32_t u32ClientID, void *pvClient, uint32_t eFunction, uint32_t cParms, VBOXHGCMSVCPARM paParms[]) = 0;
    virtual int  hostCall(uint32_t eFunction, uint32_t cParms, VBOXHGCMSVCPARM paParms[]) { return VINF_SUCCESS; }

    /** Type definition for use in callback functions. */
    typedef AbstractService SELF;
    /** HGCM helper functions. */
    PVBOXHGCMSVCHELPERS m_pHelpers;
    /*
     * Callback function supplied by the host for notification of updates
     * to properties.
     */
    PFNHGCMSVCEXT m_pfnHostCallback;
    /** User data pointer to be supplied to the host callback function. */
    void *m_pvHostData;

    /**
     * @copydoc VBOXHGCMSVCHELPERS::pfnUnload
     * Simply deletes the service object
     */
    static DECLCALLBACK(int) svcUnload(void *pvService)
    {
        AssertLogRelReturn(VALID_PTR(pvService), VERR_INVALID_PARAMETER);
        SELF *pSelf = reinterpret_cast<SELF *>(pvService);
        int rc = pSelf->uninit();
        AssertRC(rc);
        if (RT_SUCCESS(rc))
            delete pSelf;
        return rc;
    }

    /**
     * @copydoc VBOXHGCMSVCHELPERS::pfnConnect
     * Stub implementation of pfnConnect and pfnDisconnect.
     */
    static DECLCALLBACK(int) svcConnect(void *pvService,
                                        uint32_t u32ClientID,
                                        void *pvClient)
    {
        AssertLogRelReturn(VALID_PTR(pvService), VERR_INVALID_PARAMETER);
        LogFlowFunc(("pvService=%p, u32ClientID=%u, pvClient=%p\n", pvService, u32ClientID, pvClient));
        SELF *pSelf = reinterpret_cast<SELF *>(pvService);
        int rc = pSelf->clientConnect(u32ClientID, pvClient);
        LogFlowFunc(("rc=%Rrc\n", rc));
        return rc;
    }

    /**
     * @copydoc VBOXHGCMSVCHELPERS::pfnConnect
     * Stub implementation of pfnConnect and pfnDisconnect.
     */
    static DECLCALLBACK(int) svcDisconnect(void *pvService,
                                           uint32_t u32ClientID,
                                           void *pvClient)
    {
        AssertLogRelReturn(VALID_PTR(pvService), VERR_INVALID_PARAMETER);
        LogFlowFunc(("pvService=%p, u32ClientID=%u, pvClient=%p\n", pvService, u32ClientID, pvClient));
        SELF *pSelf = reinterpret_cast<SELF *>(pvService);
        int rc = pSelf->clientDisconnect(u32ClientID, pvClient);
        LogFlowFunc(("rc=%Rrc\n", rc));
        return rc;
    }

    /**
     * @copydoc VBOXHGCMSVCHELPERS::pfnCall
     * Wraps to the call member function
     */
    static DECLCALLBACK(void) svcCall(void * pvService,
                                      VBOXHGCMCALLHANDLE callHandle,
                                      uint32_t u32ClientID,
                                      void *pvClient,
                                      uint32_t u32Function,
                                      uint32_t cParms,
                                      VBOXHGCMSVCPARM paParms[])
    {
        AssertLogRelReturnVoid(VALID_PTR(pvService));
        LogFlowFunc(("pvService=%p, callHandle=%p, u32ClientID=%u, pvClient=%p, u32Function=%u, cParms=%u, paParms=%p\n", pvService, callHandle, u32ClientID, pvClient, u32Function, cParms, paParms));
        SELF *pSelf = reinterpret_cast<SELF *>(pvService);
        pSelf->guestCall(callHandle, u32ClientID, pvClient, u32Function, cParms, paParms);
        LogFlowFunc(("returning\n"));
    }

    /**
     * @copydoc VBOXHGCMSVCHELPERS::pfnHostCall
     * Wraps to the hostCall member function
     */
    static DECLCALLBACK(int) svcHostCall(void *pvService,
                                         uint32_t u32Function,
                                         uint32_t cParms,
                                         VBOXHGCMSVCPARM paParms[])
    {
        AssertLogRelReturn(VALID_PTR(pvService), VERR_INVALID_PARAMETER);
        LogFlowFunc(("pvService=%p, u32Function=%u, cParms=%u, paParms=%p\n", pvService, u32Function, cParms, paParms));
        SELF *pSelf = reinterpret_cast<SELF *>(pvService);
        int rc = pSelf->hostCall(u32Function, cParms, paParms);
        LogFlowFunc(("rc=%Rrc\n", rc));
        return rc;
    }

    /**
     * @copydoc VBOXHGCMSVCHELPERS::pfnRegisterExtension
     * Installs a host callback for notifications of property changes.
     */
    static DECLCALLBACK(int) svcRegisterExtension(void *pvService,
                                                  PFNHGCMSVCEXT pfnExtension,
                                                  void *pvExtension)
    {
        AssertLogRelReturn(VALID_PTR(pvService), VERR_INVALID_PARAMETER);
        LogFlowFunc(("pvService=%p, pfnExtension=%p, pvExtention=%p\n", pvService, pfnExtension, pvExtension));
        SELF *pSelf = reinterpret_cast<SELF *>(pvService);
        pSelf->m_pfnHostCallback = pfnExtension;
        pSelf->m_pvHostData = pvExtension;
        return VINF_SUCCESS;
    }
};

}

#endif /* !___VBox_HostService_Service_h */

