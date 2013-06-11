/* $Id: MediumImpl.cpp $ */
/** @file
 * VirtualBox COM class implementation
 */

/*
 * Copyright (C) 2008-2013 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#include "MediumImpl.h"
#include "ProgressImpl.h"
#include "SystemPropertiesImpl.h"
#include "VirtualBoxImpl.h"

#include "AutoCaller.h"
#include "Logging.h"

#include <VBox/com/array.h>
#include "VBox/com/MultiResult.h"
#include "VBox/com/ErrorInfo.h"

#include <VBox/err.h>
#include <VBox/settings.h>

#include <iprt/param.h>
#include <iprt/path.h>
#include <iprt/file.h>
#include <iprt/tcp.h>
#include <iprt/cpp/utils.h>

#include <VBox/vd.h>

#include <algorithm>
#include <list>

typedef std::list<Guid> GuidList;

////////////////////////////////////////////////////////////////////////////////
//
// Medium data definition
//
////////////////////////////////////////////////////////////////////////////////

/** Describes how a machine refers to this medium. */
struct BackRef
{
    /** Equality predicate for stdc++. */
    struct EqualsTo : public std::unary_function <BackRef, bool>
    {
        explicit EqualsTo(const Guid &aMachineId) : machineId(aMachineId) {}

        bool operator()(const argument_type &aThat) const
        {
            return aThat.machineId == machineId;
        }

        const Guid machineId;
    };

    BackRef(const Guid &aMachineId,
            const Guid &aSnapshotId = Guid::Empty)
        : machineId(aMachineId),
          fInCurState(aSnapshotId.isEmpty())
    {
        if (!aSnapshotId.isEmpty())
            llSnapshotIds.push_back(aSnapshotId);
    }

    Guid machineId;
    bool fInCurState : 1;
    GuidList llSnapshotIds;
};

typedef std::list<BackRef> BackRefList;

struct Medium::Data
{
    Data()
        : pVirtualBox(NULL),
          state(MediumState_NotCreated),
          variant(MediumVariant_Standard),
          size(0),
          readers(0),
          preLockState(MediumState_NotCreated),
          queryInfoSem(LOCKCLASS_MEDIUMQUERY),
          queryInfoRunning(false),
          type(MediumType_Normal),
          devType(DeviceType_HardDisk),
          logicalSize(0),
          hddOpenMode(OpenReadWrite),
          autoReset(false),
          hostDrive(false),
          implicit(false),
          uOpenFlagsDef(VD_OPEN_FLAGS_IGNORE_FLUSH),
          numCreateDiffTasks(0),
          vdDiskIfaces(NULL),
          vdImageIfaces(NULL)
    { }

    /** weak VirtualBox parent */
    VirtualBox * const pVirtualBox;

    // pParent and llChildren are protected by VirtualBox::getMediaTreeLockHandle()
    ComObjPtr<Medium> pParent;
    MediaList llChildren;           // to add a child, just call push_back; to remove a child, call child->deparent() which does a lookup

    GuidList llRegistryIDs;         // media registries in which this medium is listed

    const Guid id;
    Utf8Str strDescription;
    MediumState_T state;
    MediumVariant_T variant;
    Utf8Str strLocationFull;
    uint64_t size;
    Utf8Str strLastAccessError;

    BackRefList backRefs;

    size_t readers;
    MediumState_T preLockState;

    /** Special synchronization for operations which must wait for
     * Medium::queryInfo in another thread to complete. Using a SemRW is
     * not quite ideal, but at least it is subject to the lock validator,
     * unlike the SemEventMulti which we had here for many years. Catching
     * possible deadlocks is more important than a tiny bit of efficiency. */
    RWLockHandle queryInfoSem;
    bool queryInfoRunning : 1;

    const Utf8Str strFormat;
    ComObjPtr<MediumFormat> formatObj;

    MediumType_T type;
    DeviceType_T devType;
    uint64_t logicalSize;

    HDDOpenMode hddOpenMode;

    bool autoReset : 1;

    /** New UUID to be set on the next Medium::queryInfo call. */
    const Guid uuidImage;
    /** New parent UUID to be set on the next Medium::queryInfo call. */
    const Guid uuidParentImage;

    bool hostDrive : 1;

    settings::StringsMap mapProperties;

    bool implicit : 1;

    /** Default flags passed to VDOpen(). */
    unsigned uOpenFlagsDef;

    uint32_t numCreateDiffTasks;

    Utf8Str vdError;        /*< Error remembered by the VD error callback. */

    VDINTERFACEERROR vdIfError;

    VDINTERFACECONFIG vdIfConfig;

    VDINTERFACETCPNET vdIfTcpNet;

    PVDINTERFACE vdDiskIfaces;
    PVDINTERFACE vdImageIfaces;
};

typedef struct VDSOCKETINT
{
    /** Socket handle. */
    RTSOCKET hSocket;
} VDSOCKETINT, *PVDSOCKETINT;

////////////////////////////////////////////////////////////////////////////////
//
// Globals
//
////////////////////////////////////////////////////////////////////////////////

/**
 * Medium::Task class for asynchronous operations.
 *
 * @note Instances of this class must be created using new() because the
 *       task thread function will delete them when the task is complete.
 *
 * @note The constructor of this class adds a caller on the managed Medium
 *       object which is automatically released upon destruction.
 */
class Medium::Task
{
public:
    Task(Medium *aMedium, Progress *aProgress)
        : mVDOperationIfaces(NULL),
          mMedium(aMedium),
          mMediumCaller(aMedium),
          mThread(NIL_RTTHREAD),
          mProgress(aProgress),
          mVirtualBoxCaller(NULL)
    {
        AssertReturnVoidStmt(aMedium, mRC = E_FAIL);
        mRC = mMediumCaller.rc();
        if (FAILED(mRC))
            return;

        /* Get strong VirtualBox reference, see below. */
        VirtualBox *pVirtualBox = aMedium->m->pVirtualBox;
        mVirtualBox = pVirtualBox;
        mVirtualBoxCaller.attach(pVirtualBox);
        mRC = mVirtualBoxCaller.rc();
        if (FAILED(mRC))
            return;

        /* Set up a per-operation progress interface, can be used freely (for
         * binary operations you can use it either on the source or target). */
        mVDIfProgress.pfnProgress = vdProgressCall;
        int vrc = VDInterfaceAdd(&mVDIfProgress.Core,
                                "Medium::Task::vdInterfaceProgress",
                                VDINTERFACETYPE_PROGRESS,
                                mProgress,
                                sizeof(VDINTERFACEPROGRESS),
                                &mVDOperationIfaces);
        AssertRC(vrc);
        if (RT_FAILURE(vrc))
            mRC = E_FAIL;
    }

    // Make all destructors virtual. Just in case.
    virtual ~Task()
    {}

    HRESULT rc() const { return mRC; }
    bool isOk() const { return SUCCEEDED(rc()); }

    static int fntMediumTask(RTTHREAD aThread, void *pvUser);

    bool isAsync() { return mThread != NIL_RTTHREAD; }

    PVDINTERFACE mVDOperationIfaces;

    const ComObjPtr<Medium> mMedium;
    AutoCaller mMediumCaller;

    friend HRESULT Medium::runNow(Medium::Task*);

protected:
    HRESULT mRC;
    RTTHREAD mThread;

private:
    virtual HRESULT handler() = 0;

    const ComObjPtr<Progress> mProgress;

    static DECLCALLBACK(int) vdProgressCall(void *pvUser, unsigned uPercent);

    VDINTERFACEPROGRESS mVDIfProgress;

    /* Must have a strong VirtualBox reference during a task otherwise the
     * reference count might drop to 0 while a task is still running. This
     * would result in weird behavior, including deadlocks due to uninit and
     * locking order issues. The deadlock often is not detectable because the
     * uninit uses event semaphores which sabotages deadlock detection. */
    ComObjPtr<VirtualBox> mVirtualBox;
    AutoCaller mVirtualBoxCaller;
};

class Medium::CreateBaseTask : public Medium::Task
{
public:
    CreateBaseTask(Medium *aMedium,
                   Progress *aProgress,
                   uint64_t aSize,
                   MediumVariant_T aVariant)
        : Medium::Task(aMedium, aProgress),
          mSize(aSize),
          mVariant(aVariant)
    {}

    uint64_t mSize;
    MediumVariant_T mVariant;

private:
    virtual HRESULT handler();
};

class Medium::CreateDiffTask : public Medium::Task
{
public:
    CreateDiffTask(Medium *aMedium,
                   Progress *aProgress,
                   Medium *aTarget,
                   MediumVariant_T aVariant,
                   MediumLockList *aMediumLockList,
                   bool fKeepMediumLockList = false)
        : Medium::Task(aMedium, aProgress),
          mpMediumLockList(aMediumLockList),
          mTarget(aTarget),
          mVariant(aVariant),
          mTargetCaller(aTarget),
          mfKeepMediumLockList(fKeepMediumLockList)
    {
        AssertReturnVoidStmt(aTarget != NULL, mRC = E_FAIL);
        mRC = mTargetCaller.rc();
        if (FAILED(mRC))
            return;
    }

    ~CreateDiffTask()
    {
        if (!mfKeepMediumLockList && mpMediumLockList)
            delete mpMediumLockList;
    }

    MediumLockList *mpMediumLockList;

    const ComObjPtr<Medium> mTarget;
    MediumVariant_T mVariant;

private:
    virtual HRESULT handler();

    AutoCaller mTargetCaller;
    bool mfKeepMediumLockList;
};

class Medium::CloneTask : public Medium::Task
{
public:
    CloneTask(Medium *aMedium,
              Progress *aProgress,
              Medium *aTarget,
              MediumVariant_T aVariant,
              Medium *aParent,
              uint32_t idxSrcImageSame,
              uint32_t idxDstImageSame,
              MediumLockList *aSourceMediumLockList,
              MediumLockList *aTargetMediumLockList,
              bool fKeepSourceMediumLockList = false,
              bool fKeepTargetMediumLockList = false)
        : Medium::Task(aMedium, aProgress),
          mTarget(aTarget),
          mParent(aParent),
          mpSourceMediumLockList(aSourceMediumLockList),
          mpTargetMediumLockList(aTargetMediumLockList),
          mVariant(aVariant),
          midxSrcImageSame(idxSrcImageSame),
          midxDstImageSame(idxDstImageSame),
          mTargetCaller(aTarget),
          mParentCaller(aParent),
          mfKeepSourceMediumLockList(fKeepSourceMediumLockList),
          mfKeepTargetMediumLockList(fKeepTargetMediumLockList)
    {
        AssertReturnVoidStmt(aTarget != NULL, mRC = E_FAIL);
        mRC = mTargetCaller.rc();
        if (FAILED(mRC))
            return;
        /* aParent may be NULL */
        mRC = mParentCaller.rc();
        if (FAILED(mRC))
            return;
        AssertReturnVoidStmt(aSourceMediumLockList != NULL, mRC = E_FAIL);
        AssertReturnVoidStmt(aTargetMediumLockList != NULL, mRC = E_FAIL);
    }

    ~CloneTask()
    {
        if (!mfKeepSourceMediumLockList && mpSourceMediumLockList)
            delete mpSourceMediumLockList;
        if (!mfKeepTargetMediumLockList && mpTargetMediumLockList)
            delete mpTargetMediumLockList;
    }

    const ComObjPtr<Medium> mTarget;
    const ComObjPtr<Medium> mParent;
    MediumLockList *mpSourceMediumLockList;
    MediumLockList *mpTargetMediumLockList;
    MediumVariant_T mVariant;
    uint32_t midxSrcImageSame;
    uint32_t midxDstImageSame;

private:
    virtual HRESULT handler();

    AutoCaller mTargetCaller;
    AutoCaller mParentCaller;
    bool mfKeepSourceMediumLockList;
    bool mfKeepTargetMediumLockList;
};

class Medium::CompactTask : public Medium::Task
{
public:
    CompactTask(Medium *aMedium,
                Progress *aProgress,
                MediumLockList *aMediumLockList,
                bool fKeepMediumLockList = false)
        : Medium::Task(aMedium, aProgress),
          mpMediumLockList(aMediumLockList),
          mfKeepMediumLockList(fKeepMediumLockList)
    {
        AssertReturnVoidStmt(aMediumLockList != NULL, mRC = E_FAIL);
    }

    ~CompactTask()
    {
        if (!mfKeepMediumLockList && mpMediumLockList)
            delete mpMediumLockList;
    }

    MediumLockList *mpMediumLockList;

private:
    virtual HRESULT handler();

    bool mfKeepMediumLockList;
};

class Medium::ResizeTask : public Medium::Task
{
public:
    ResizeTask(Medium *aMedium,
               uint64_t aSize,
               Progress *aProgress,
               MediumLockList *aMediumLockList,
               bool fKeepMediumLockList = false)
        : Medium::Task(aMedium, aProgress),
          mSize(aSize),
          mpMediumLockList(aMediumLockList),
          mfKeepMediumLockList(fKeepMediumLockList)
    {
        AssertReturnVoidStmt(aMediumLockList != NULL, mRC = E_FAIL);
    }

    ~ResizeTask()
    {
        if (!mfKeepMediumLockList && mpMediumLockList)
            delete mpMediumLockList;
    }

    uint64_t        mSize;
    MediumLockList *mpMediumLockList;

private:
    virtual HRESULT handler();

    bool mfKeepMediumLockList;
};

class Medium::ResetTask : public Medium::Task
{
public:
    ResetTask(Medium *aMedium,
              Progress *aProgress,
              MediumLockList *aMediumLockList,
              bool fKeepMediumLockList = false)
        : Medium::Task(aMedium, aProgress),
          mpMediumLockList(aMediumLockList),
          mfKeepMediumLockList(fKeepMediumLockList)
    {}

    ~ResetTask()
    {
        if (!mfKeepMediumLockList && mpMediumLockList)
            delete mpMediumLockList;
    }

    MediumLockList *mpMediumLockList;

private:
    virtual HRESULT handler();

    bool mfKeepMediumLockList;
};

class Medium::DeleteTask : public Medium::Task
{
public:
    DeleteTask(Medium *aMedium,
               Progress *aProgress,
               MediumLockList *aMediumLockList,
               bool fKeepMediumLockList = false)
        : Medium::Task(aMedium, aProgress),
          mpMediumLockList(aMediumLockList),
          mfKeepMediumLockList(fKeepMediumLockList)
    {}

    ~DeleteTask()
    {
        if (!mfKeepMediumLockList && mpMediumLockList)
            delete mpMediumLockList;
    }

    MediumLockList *mpMediumLockList;

private:
    virtual HRESULT handler();

    bool mfKeepMediumLockList;
};

class Medium::MergeTask : public Medium::Task
{
public:
    MergeTask(Medium *aMedium,
              Medium *aTarget,
              bool fMergeForward,
              Medium *aParentForTarget,
              const MediaList &aChildrenToReparent,
              Progress *aProgress,
              MediumLockList *aMediumLockList,
              bool fKeepMediumLockList = false)
        : Medium::Task(aMedium, aProgress),
          mTarget(aTarget),
          mfMergeForward(fMergeForward),
          mParentForTarget(aParentForTarget),
          mChildrenToReparent(aChildrenToReparent),
          mpMediumLockList(aMediumLockList),
          mTargetCaller(aTarget),
          mParentForTargetCaller(aParentForTarget),
          mfChildrenCaller(false),
          mfKeepMediumLockList(fKeepMediumLockList)
    {
        AssertReturnVoidStmt(aMediumLockList != NULL, mRC = E_FAIL);
        for (MediaList::const_iterator it = mChildrenToReparent.begin();
             it != mChildrenToReparent.end();
             ++it)
        {
            HRESULT rc2 = (*it)->addCaller();
            if (FAILED(rc2))
            {
                mRC = E_FAIL;
                for (MediaList::const_iterator it2 = mChildrenToReparent.begin();
                     it2 != it;
                     --it2)
                {
                    (*it2)->releaseCaller();
                }
                return;
            }
        }
        mfChildrenCaller = true;
    }

    ~MergeTask()
    {
        if (!mfKeepMediumLockList && mpMediumLockList)
            delete mpMediumLockList;
        if (mfChildrenCaller)
        {
            for (MediaList::const_iterator it = mChildrenToReparent.begin();
                 it != mChildrenToReparent.end();
                 ++it)
            {
                (*it)->releaseCaller();
            }
        }
    }

    const ComObjPtr<Medium> mTarget;
    bool mfMergeForward;
    /* When mChildrenToReparent is empty then mParentForTarget is non-null.
     * In other words: they are used in different cases. */
    const ComObjPtr<Medium> mParentForTarget;
    MediaList mChildrenToReparent;
    MediumLockList *mpMediumLockList;

private:
    virtual HRESULT handler();

    AutoCaller mTargetCaller;
    AutoCaller mParentForTargetCaller;
    bool mfChildrenCaller;
    bool mfKeepMediumLockList;
};

class Medium::ExportTask : public Medium::Task
{
public:
    ExportTask(Medium *aMedium,
               Progress *aProgress,
               const char *aFilename,
               MediumFormat *aFormat,
               MediumVariant_T aVariant,
               VDINTERFACEIO *aVDImageIOIf,
               void *aVDImageIOUser,
               MediumLockList *aSourceMediumLockList,
               bool fKeepSourceMediumLockList = false)
        : Medium::Task(aMedium, aProgress),
          mpSourceMediumLockList(aSourceMediumLockList),
          mFilename(aFilename),
          mFormat(aFormat),
          mVariant(aVariant),
          mfKeepSourceMediumLockList(fKeepSourceMediumLockList)
    {
        AssertReturnVoidStmt(aSourceMediumLockList != NULL, mRC = E_FAIL);

        mVDImageIfaces = aMedium->m->vdImageIfaces;
        if (aVDImageIOIf)
        {
            int vrc = VDInterfaceAdd(&aVDImageIOIf->Core, "Medium::vdInterfaceIO",
                                     VDINTERFACETYPE_IO, aVDImageIOUser,
                                     sizeof(VDINTERFACEIO), &mVDImageIfaces);
            AssertRCReturnVoidStmt(vrc, mRC = E_FAIL);
        }
    }

    ~ExportTask()
    {
        if (!mfKeepSourceMediumLockList && mpSourceMediumLockList)
            delete mpSourceMediumLockList;
    }

    MediumLockList *mpSourceMediumLockList;
    Utf8Str mFilename;
    ComObjPtr<MediumFormat> mFormat;
    MediumVariant_T mVariant;
    PVDINTERFACE mVDImageIfaces;

private:
    virtual HRESULT handler();

    bool mfKeepSourceMediumLockList;
};

class Medium::ImportTask : public Medium::Task
{
public:
    ImportTask(Medium *aMedium,
               Progress *aProgress,
               const char *aFilename,
               MediumFormat *aFormat,
               MediumVariant_T aVariant,
               VDINTERFACEIO *aVDImageIOIf,
               void *aVDImageIOUser,
               Medium *aParent,
               MediumLockList *aTargetMediumLockList,
               bool fKeepTargetMediumLockList = false)
        : Medium::Task(aMedium, aProgress),
          mFilename(aFilename),
          mFormat(aFormat),
          mVariant(aVariant),
          mParent(aParent),
          mpTargetMediumLockList(aTargetMediumLockList),
          mParentCaller(aParent),
          mfKeepTargetMediumLockList(fKeepTargetMediumLockList)
    {
        AssertReturnVoidStmt(aTargetMediumLockList != NULL, mRC = E_FAIL);
        /* aParent may be NULL */
        mRC = mParentCaller.rc();
        if (FAILED(mRC))
            return;

        mVDImageIfaces = aMedium->m->vdImageIfaces;
        if (aVDImageIOIf)
        {
            int vrc = VDInterfaceAdd(&aVDImageIOIf->Core, "Medium::vdInterfaceIO",
                                     VDINTERFACETYPE_IO, aVDImageIOUser,
                                     sizeof(VDINTERFACEIO), &mVDImageIfaces);
            AssertRCReturnVoidStmt(vrc, mRC = E_FAIL);
        }
    }

    ~ImportTask()
    {
        if (!mfKeepTargetMediumLockList && mpTargetMediumLockList)
            delete mpTargetMediumLockList;
    }

    Utf8Str mFilename;
    ComObjPtr<MediumFormat> mFormat;
    MediumVariant_T mVariant;
    const ComObjPtr<Medium> mParent;
    MediumLockList *mpTargetMediumLockList;
    PVDINTERFACE mVDImageIfaces;

private:
    virtual HRESULT handler();

    AutoCaller mParentCaller;
    bool mfKeepTargetMediumLockList;
};

/**
 * Thread function for time-consuming medium tasks.
 *
 * @param pvUser    Pointer to the Medium::Task instance.
 */
/* static */
DECLCALLBACK(int) Medium::Task::fntMediumTask(RTTHREAD aThread, void *pvUser)
{
    LogFlowFuncEnter();
    AssertReturn(pvUser, (int)E_INVALIDARG);
    Medium::Task *pTask = static_cast<Medium::Task *>(pvUser);

    pTask->mThread = aThread;

    HRESULT rc = pTask->handler();

    /* complete the progress if run asynchronously */
    if (pTask->isAsync())
    {
        if (!pTask->mProgress.isNull())
            pTask->mProgress->notifyComplete(rc);
    }

    /* pTask is no longer needed, delete it. */
    delete pTask;

    LogFlowFunc(("rc=%Rhrc\n", rc));
    LogFlowFuncLeave();

    return (int)rc;
}

/**
 * PFNVDPROGRESS callback handler for Task operations.
 *
 * @param pvUser      Pointer to the Progress instance.
 * @param uPercent    Completion percentage (0-100).
 */
/*static*/
DECLCALLBACK(int) Medium::Task::vdProgressCall(void *pvUser, unsigned uPercent)
{
    Progress *that = static_cast<Progress *>(pvUser);

    if (that != NULL)
    {
        /* update the progress object, capping it at 99% as the final percent
         * is used for additional operations like setting the UUIDs and similar. */
        HRESULT rc = that->SetCurrentOperationProgress(uPercent * 99 / 100);
        if (FAILED(rc))
        {
            if (rc == E_FAIL)
                return VERR_CANCELLED;
            else
                return VERR_INVALID_STATE;
        }
    }

    return VINF_SUCCESS;
}

/**
 * Implementation code for the "create base" task.
 */
HRESULT Medium::CreateBaseTask::handler()
{
    return mMedium->taskCreateBaseHandler(*this);
}

/**
 * Implementation code for the "create diff" task.
 */
HRESULT Medium::CreateDiffTask::handler()
{
    return mMedium->taskCreateDiffHandler(*this);
}

/**
 * Implementation code for the "clone" task.
 */
HRESULT Medium::CloneTask::handler()
{
    return mMedium->taskCloneHandler(*this);
}

/**
 * Implementation code for the "compact" task.
 */
HRESULT Medium::CompactTask::handler()
{
    return mMedium->taskCompactHandler(*this);
}

/**
 * Implementation code for the "resize" task.
 */
HRESULT Medium::ResizeTask::handler()
{
    return mMedium->taskResizeHandler(*this);
}


/**
 * Implementation code for the "reset" task.
 */
HRESULT Medium::ResetTask::handler()
{
    return mMedium->taskResetHandler(*this);
}

/**
 * Implementation code for the "delete" task.
 */
HRESULT Medium::DeleteTask::handler()
{
    return mMedium->taskDeleteHandler(*this);
}

/**
 * Implementation code for the "merge" task.
 */
HRESULT Medium::MergeTask::handler()
{
    return mMedium->taskMergeHandler(*this);
}

/**
 * Implementation code for the "export" task.
 */
HRESULT Medium::ExportTask::handler()
{
    return mMedium->taskExportHandler(*this);
}

/**
 * Implementation code for the "import" task.
 */
HRESULT Medium::ImportTask::handler()
{
    return mMedium->taskImportHandler(*this);
}

////////////////////////////////////////////////////////////////////////////////
//
// Medium constructor / destructor
//
////////////////////////////////////////////////////////////////////////////////

DEFINE_EMPTY_CTOR_DTOR(Medium)

HRESULT Medium::FinalConstruct()
{
    m = new Data;

    /* Initialize the callbacks of the VD error interface */
    m->vdIfError.pfnError = vdErrorCall;
    m->vdIfError.pfnMessage = NULL;

    /* Initialize the callbacks of the VD config interface */
    m->vdIfConfig.pfnAreKeysValid = vdConfigAreKeysValid;
    m->vdIfConfig.pfnQuerySize = vdConfigQuerySize;
    m->vdIfConfig.pfnQuery = vdConfigQuery;

    /* Initialize the callbacks of the VD TCP interface (we always use the host
     * IP stack for now) */
    m->vdIfTcpNet.pfnSocketCreate = vdTcpSocketCreate;
    m->vdIfTcpNet.pfnSocketDestroy = vdTcpSocketDestroy;
    m->vdIfTcpNet.pfnClientConnect = vdTcpClientConnect;
    m->vdIfTcpNet.pfnClientClose = vdTcpClientClose;
    m->vdIfTcpNet.pfnIsClientConnected = vdTcpIsClientConnected;
    m->vdIfTcpNet.pfnSelectOne = vdTcpSelectOne;
    m->vdIfTcpNet.pfnRead = vdTcpRead;
    m->vdIfTcpNet.pfnWrite = vdTcpWrite;
    m->vdIfTcpNet.pfnSgWrite = vdTcpSgWrite;
    m->vdIfTcpNet.pfnFlush = vdTcpFlush;
    m->vdIfTcpNet.pfnSetSendCoalescing = vdTcpSetSendCoalescing;
    m->vdIfTcpNet.pfnGetLocalAddress = vdTcpGetLocalAddress;
    m->vdIfTcpNet.pfnGetPeerAddress = vdTcpGetPeerAddress;
    m->vdIfTcpNet.pfnSelectOneEx = NULL;
    m->vdIfTcpNet.pfnPoke = NULL;

    /* Initialize the per-disk interface chain (could be done more globally,
     * but it's not wasting much time or space so it's not worth it). */
    int vrc;
    vrc = VDInterfaceAdd(&m->vdIfError.Core,
                         "Medium::vdInterfaceError",
                         VDINTERFACETYPE_ERROR, this,
                         sizeof(VDINTERFACEERROR), &m->vdDiskIfaces);
    AssertRCReturn(vrc, E_FAIL);

    /* Initialize the per-image interface chain */
    vrc = VDInterfaceAdd(&m->vdIfConfig.Core,
                         "Medium::vdInterfaceConfig",
                         VDINTERFACETYPE_CONFIG, this,
                         sizeof(VDINTERFACECONFIG), &m->vdImageIfaces);
    AssertRCReturn(vrc, E_FAIL);

    vrc = VDInterfaceAdd(&m->vdIfTcpNet.Core,
                         "Medium::vdInterfaceTcpNet",
                         VDINTERFACETYPE_TCPNET, this,
                         sizeof(VDINTERFACETCPNET), &m->vdImageIfaces);
    AssertRCReturn(vrc, E_FAIL);

    return BaseFinalConstruct();
}

void Medium::FinalRelease()
{
    uninit();

    delete m;

    BaseFinalRelease();
}

/**
 * Initializes an empty hard disk object without creating or opening an associated
 * storage unit.
 *
 * This gets called by VirtualBox::CreateHardDisk() in which case uuidMachineRegistry
 * is empty since starting with VirtualBox 4.0, we no longer add opened media to a
 * registry automatically (this is deferred until the medium is attached to a machine).
 *
 * This also gets called when VirtualBox creates diff images; in this case uuidMachineRegistry
 * is set to the registry of the parent image to make sure they all end up in the same
 * file.
 *
 * For hard disks that don't have the MediumFormatCapabilities_CreateFixed or
 * MediumFormatCapabilities_CreateDynamic capability (and therefore cannot be created or deleted
 * with the means of VirtualBox) the associated storage unit is assumed to be
 * ready for use so the state of the hard disk object will be set to Created.
 *
 * @param aVirtualBox   VirtualBox object.
 * @param aFormat
 * @param aLocation     Storage unit location.
 * @param uuidMachineRegistry The registry to which this medium should be added (global registry UUID or machine UUID or empty if none).
 */
HRESULT Medium::init(VirtualBox *aVirtualBox,
                     const Utf8Str &aFormat,
                     const Utf8Str &aLocation,
                     const Guid &uuidMachineRegistry)
{
    AssertReturn(aVirtualBox != NULL, E_FAIL);
    AssertReturn(!aFormat.isEmpty(), E_FAIL);

    /* Enclose the state transition NotReady->InInit->Ready */
    AutoInitSpan autoInitSpan(this);
    AssertReturn(autoInitSpan.isOk(), E_FAIL);

    HRESULT rc = S_OK;

    unconst(m->pVirtualBox) = aVirtualBox;

    if (!uuidMachineRegistry.isEmpty())
        m->llRegistryIDs.push_back(uuidMachineRegistry);

    /* no storage yet */
    m->state = MediumState_NotCreated;

    /* cannot be a host drive */
    m->hostDrive = false;

    /* No storage unit is created yet, no need to call Medium::queryInfo */

    rc = setFormat(aFormat);
    if (FAILED(rc)) return rc;

    rc = setLocation(aLocation);
    if (FAILED(rc)) return rc;

    if (!(m->formatObj->getCapabilities() & (   MediumFormatCapabilities_CreateFixed
                                              | MediumFormatCapabilities_CreateDynamic))
       )
    {
        /* Storage for hard disks of this format can neither be explicitly
         * created by VirtualBox nor deleted, so we place the hard disk to
         * Inaccessible state here and also add it to the registry. The
         * state means that one has to use RefreshState() to update the
         * medium format specific fields. */
        m->state = MediumState_Inaccessible;
        // create new UUID
        unconst(m->id).create();

        AutoWriteLock treeLock(m->pVirtualBox->getMediaTreeLockHandle() COMMA_LOCKVAL_SRC_POS);
        ComObjPtr<Medium> pMedium;

        /*
         * Check whether the UUID is taken already and create a new one
         * if required.
         * Try this only a limited amount of times in case the PRNG is broken
         * in some way to prevent an endless loop.
         */
        for (unsigned i = 0; i < 5; i++)
        {
            bool fInUse;

            fInUse = m->pVirtualBox->isMediaUuidInUse(m->id, DeviceType_HardDisk);
            if (fInUse)
            {
                // create new UUID
                unconst(m->id).create();
            }
            else
                break;
        }

        rc = m->pVirtualBox->registerMedium(this, &pMedium, DeviceType_HardDisk);
        Assert(this == pMedium || FAILED(rc));
    }

    /* Confirm a successful initialization when it's the case */
    if (SUCCEEDED(rc))
        autoInitSpan.setSucceeded();

    return rc;
}

/**
 * Initializes the medium object by opening the storage unit at the specified
 * location. The enOpenMode parameter defines whether the medium will be opened
 * read/write or read-only.
 *
 * This gets called by VirtualBox::OpenMedium() and also by
 * Machine::AttachDevice() and createImplicitDiffs() when new diff
 * images are created.
 *
 * There is no registry for this case since starting with VirtualBox 4.0, we
 * no longer add opened media to a registry automatically (this is deferred
 * until the medium is attached to a machine).
 *
 * For hard disks, the UUID, format and the parent of this medium will be
 * determined when reading the medium storage unit. For DVD and floppy images,
 * which have no UUIDs in their storage units, new UUIDs are created.
 * If the detected or set parent is not known to VirtualBox, then this method
 * will fail.
 *
 * @param aVirtualBox   VirtualBox object.
 * @param aLocation     Storage unit location.
 * @param enOpenMode    Whether to open the medium read/write or read-only.
 * @param fForceNewUuid Whether a new UUID should be set to avoid duplicates.
 * @param aDeviceType   Device type of medium.
 */
HRESULT Medium::init(VirtualBox *aVirtualBox,
                     const Utf8Str &aLocation,
                     HDDOpenMode enOpenMode,
                     bool fForceNewUuid,
                     DeviceType_T aDeviceType)
{
    AssertReturn(aVirtualBox, E_INVALIDARG);
    AssertReturn(!aLocation.isEmpty(), E_INVALIDARG);

    HRESULT rc = S_OK;

    {
        /* Enclose the state transition NotReady->InInit->Ready */
        AutoInitSpan autoInitSpan(this);
        AssertReturn(autoInitSpan.isOk(), E_FAIL);

        unconst(m->pVirtualBox) = aVirtualBox;

        /* there must be a storage unit */
        m->state = MediumState_Created;

        /* remember device type for correct unregistering later */
        m->devType = aDeviceType;

        /* cannot be a host drive */
        m->hostDrive = false;

        /* remember the open mode (defaults to ReadWrite) */
        m->hddOpenMode = enOpenMode;

        if (aDeviceType == DeviceType_DVD)
            m->type = MediumType_Readonly;
        else if (aDeviceType == DeviceType_Floppy)
            m->type = MediumType_Writethrough;

        rc = setLocation(aLocation);
        if (FAILED(rc)) return rc;

        /* get all the information about the medium from the storage unit */
        if (fForceNewUuid)
            unconst(m->uuidImage).create();

        m->state = MediumState_Inaccessible;
        m->strLastAccessError = tr("Accessibility check was not yet performed");

        /* Confirm a successful initialization before the call to queryInfo.
         * Otherwise we can end up with a AutoCaller deadlock because the
         * medium becomes visible but is not marked as initialized. Causes
         * locking trouble (e.g. trying to save media registries) which is
         * hard to solve. */
        autoInitSpan.setSucceeded();
    }

    /* we're normal code from now on, no longer init */
    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc()))
        return autoCaller.rc();

    /* need to call queryInfo immediately to correctly place the medium in
     * the respective media tree and update other information such as uuid */
    rc = queryInfo(fForceNewUuid /* fSetImageId */, false /* fSetParentId */);
    if (SUCCEEDED(rc))
    {
        AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

        /* if the storage unit is not accessible, it's not acceptable for the
         * newly opened media so convert this into an error */
        if (m->state == MediumState_Inaccessible)
        {
            Assert(!m->strLastAccessError.isEmpty());
            rc = setError(E_FAIL, "%s", m->strLastAccessError.c_str());
            alock.release();
            autoCaller.release();
            uninit();
        }
        else
        {
            AssertStmt(!m->id.isEmpty(),
                       alock.release(); autoCaller.release(); uninit(); return E_FAIL);

            /* storage format must be detected by Medium::queryInfo if the
             * medium is accessible */
            AssertStmt(!m->strFormat.isEmpty(),
                       alock.release(); autoCaller.release(); uninit(); return E_FAIL);
        }
    }
    else
    {
        /* opening this image failed, mark the object as dead */
        autoCaller.release();
        uninit();
    }

    return rc;
}

/**
 * Initializes the medium object by loading its data from the given settings
 * node. In this mode, the medium will always be opened read/write.
 *
 * In this case, since we're loading from a registry, uuidMachineRegistry is
 * always set: it's either the global registry UUID or a machine UUID when
 * loading from a per-machine registry.
 *
 * @param aVirtualBox   VirtualBox object.
 * @param aParent       Parent medium disk or NULL for a root (base) medium.
 * @param aDeviceType   Device type of the medium.
 * @param uuidMachineRegistry The registry to which this medium should be added (global registry UUID or machine UUID).
 * @param aNode         Configuration settings.
 * @param strMachineFolder The machine folder with which to resolve relative paths; if empty, then we use the VirtualBox home directory
 *
 * @note Locks the medium tree for writing.
 */
HRESULT Medium::init(VirtualBox *aVirtualBox,
                     Medium *aParent,
                     DeviceType_T aDeviceType,
                     const Guid &uuidMachineRegistry,
                     const settings::Medium &data,
                     const Utf8Str &strMachineFolder)
{
    using namespace settings;

    AssertReturn(aVirtualBox, E_INVALIDARG);

    /* Enclose the state transition NotReady->InInit->Ready */
    AutoInitSpan autoInitSpan(this);
    AssertReturn(autoInitSpan.isOk(), E_FAIL);

    HRESULT rc = S_OK;

    unconst(m->pVirtualBox) = aVirtualBox;

    if (!uuidMachineRegistry.isEmpty())
        m->llRegistryIDs.push_back(uuidMachineRegistry);

    /* register with VirtualBox/parent early, since uninit() will
     * unconditionally unregister on failure */
    if (aParent)
    {
        // differencing medium: add to parent
        AutoWriteLock treeLock(m->pVirtualBox->getMediaTreeLockHandle() COMMA_LOCKVAL_SRC_POS);
        m->pParent = aParent;
        aParent->m->llChildren.push_back(this);
    }

    /* see below why we don't call Medium::queryInfo (and therefore treat
     * the medium as inaccessible for now */
    m->state = MediumState_Inaccessible;
    m->strLastAccessError = tr("Accessibility check was not yet performed");

    /* required */
    unconst(m->id) = data.uuid;

    /* assume not a host drive */
    m->hostDrive = false;

    /* optional */
    m->strDescription = data.strDescription;

    /* required */
    if (aDeviceType == DeviceType_HardDisk)
    {
        AssertReturn(!data.strFormat.isEmpty(), E_FAIL);
        rc = setFormat(data.strFormat);
        if (FAILED(rc)) return rc;
    }
    else
    {
        /// @todo handle host drive settings here as well?
        if (!data.strFormat.isEmpty())
            rc = setFormat(data.strFormat);
        else
            rc = setFormat("RAW");
        if (FAILED(rc)) return rc;
    }

    /* optional, only for diffs, default is false; we can only auto-reset
     * diff media so they must have a parent */
    if (aParent != NULL)
        m->autoReset = data.fAutoReset;
    else
        m->autoReset = false;

    /* properties (after setting the format as it populates the map). Note that
     * if some properties are not supported but present in the settings file,
     * they will still be read and accessible (for possible backward
     * compatibility; we can also clean them up from the XML upon next
     * XML format version change if we wish) */
    for (settings::StringsMap::const_iterator it = data.properties.begin();
         it != data.properties.end();
         ++it)
    {
        const Utf8Str &name = it->first;
        const Utf8Str &value = it->second;
        m->mapProperties[name] = value;
    }

    /* try to decrypt an optional iSCSI initiator secret */
    settings::StringsMap::const_iterator itCph = data.properties.find("InitiatorSecretEncrypted");
    if (   itCph != data.properties.end()
        && !itCph->second.isEmpty())
    {
        Utf8Str strPlaintext;
        int vrc = m->pVirtualBox->decryptSetting(&strPlaintext, itCph->second);
        if (RT_SUCCESS(vrc))
            m->mapProperties["InitiatorSecret"] = strPlaintext;
    }

    Utf8Str strFull;
    if (m->formatObj->getCapabilities() & MediumFormatCapabilities_File)
    {
        // compose full path of the medium, if it's not fully qualified...
        // slightly convoluted logic here. If the caller has given us a
        // machine folder, then a relative path will be relative to that:
        if (    !strMachineFolder.isEmpty()
             && !RTPathStartsWithRoot(data.strLocation.c_str())
           )
        {
            strFull = strMachineFolder;
            strFull += RTPATH_SLASH;
            strFull += data.strLocation;
        }
        else
        {
            // Otherwise use the old VirtualBox "make absolute path" logic:
            rc = m->pVirtualBox->calculateFullPath(data.strLocation, strFull);
            if (FAILED(rc)) return rc;
        }
    }
    else
        strFull = data.strLocation;

    rc = setLocation(strFull);
    if (FAILED(rc)) return rc;

    if (aDeviceType == DeviceType_HardDisk)
    {
        /* type is only for base hard disks */
        if (m->pParent.isNull())
            m->type = data.hdType;
    }
    else if (aDeviceType == DeviceType_DVD)
        m->type = MediumType_Readonly;
    else
        m->type = MediumType_Writethrough;

    /* remember device type for correct unregistering later */
    m->devType = aDeviceType;

    LogFlowThisFunc(("m->strLocationFull='%s', m->strFormat=%s, m->id={%RTuuid}\n",
                     m->strLocationFull.c_str(), m->strFormat.c_str(), m->id.raw()));

    /* Don't call Medium::queryInfo for registered media to prevent the calling
     * thread (i.e. the VirtualBox server startup thread) from an unexpected
     * freeze but mark it as initially inaccessible instead. The vital UUID,
     * location and format properties are read from the registry file above; to
     * get the actual state and the rest of the data, the user will have to call
     * COMGETTER(State). */

    AutoWriteLock treeLock(aVirtualBox->getMediaTreeLockHandle() COMMA_LOCKVAL_SRC_POS);

    /* load all children */
    for (settings::MediaList::const_iterator it = data.llChildren.begin();
         it != data.llChildren.end();
         ++it)
    {
        const settings::Medium &med = *it;

        ComObjPtr<Medium> pHD;
        pHD.createObject();
        rc = pHD->init(aVirtualBox,
                       this,            // parent
                       aDeviceType,
                       uuidMachineRegistry,
                       med,               // child data
                       strMachineFolder);
        if (FAILED(rc)) break;

        rc = m->pVirtualBox->registerMedium(pHD, &pHD, DeviceType_HardDisk);
        if (FAILED(rc)) break;
    }

    /* Confirm a successful initialization when it's the case */
    if (SUCCEEDED(rc))
        autoInitSpan.setSucceeded();

    return rc;
}

/**
 * Initializes the medium object by providing the host drive information.
 * Not used for anything but the host floppy/host DVD case.
 *
 * There is no registry for this case.
 *
 * @param aVirtualBox   VirtualBox object.
 * @param aDeviceType   Device type of the medium.
 * @param aLocation     Location of the host drive.
 * @param aDescription  Comment for this host drive.
 *
 * @note Locks VirtualBox lock for writing.
 */
HRESULT Medium::init(VirtualBox *aVirtualBox,
                     DeviceType_T aDeviceType,
                     const Utf8Str &aLocation,
                     const Utf8Str &aDescription /* = Utf8Str::Empty */)
{
    ComAssertRet(aDeviceType == DeviceType_DVD || aDeviceType == DeviceType_Floppy, E_INVALIDARG);
    ComAssertRet(!aLocation.isEmpty(), E_INVALIDARG);

    /* Enclose the state transition NotReady->InInit->Ready */
    AutoInitSpan autoInitSpan(this);
    AssertReturn(autoInitSpan.isOk(), E_FAIL);

    unconst(m->pVirtualBox) = aVirtualBox;

    // We do not store host drives in VirtualBox.xml or anywhere else, so if we want
    // host drives to be identifiable by UUID and not give the drive a different UUID
    // every time VirtualBox starts, we need to fake a reproducible UUID here:
    RTUUID uuid;
    RTUuidClear(&uuid);
    if (aDeviceType == DeviceType_DVD)
        memcpy(&uuid.au8[0], "DVD", 3);
    else
        memcpy(&uuid.au8[0], "FD", 2);
    /* use device name, adjusted to the end of uuid, shortened if necessary */
    size_t lenLocation = aLocation.length();
    if (lenLocation > 12)
        memcpy(&uuid.au8[4], aLocation.c_str() + (lenLocation - 12), 12);
    else
        memcpy(&uuid.au8[4 + 12 - lenLocation], aLocation.c_str(), lenLocation);
    unconst(m->id) = uuid;

    if (aDeviceType == DeviceType_DVD)
        m->type = MediumType_Readonly;
    else
        m->type = MediumType_Writethrough;
    m->devType = aDeviceType;
    m->state = MediumState_Created;
    m->hostDrive = true;
    HRESULT rc = setFormat("RAW");
    if (FAILED(rc)) return rc;
    rc = setLocation(aLocation);
    if (FAILED(rc)) return rc;
    m->strDescription = aDescription;

    autoInitSpan.setSucceeded();
    return S_OK;
}

/**
 * Uninitializes the instance.
 *
 * Called either from FinalRelease() or by the parent when it gets destroyed.
 *
 * @note All children of this medium get uninitialized by calling their
 *       uninit() methods.
 */
void Medium::uninit()
{
    /* Enclose the state transition Ready->InUninit->NotReady */
    AutoUninitSpan autoUninitSpan(this);
    if (autoUninitSpan.uninitDone())
        return;

    if (!m->formatObj.isNull())
    {
        /* remove the caller reference we added in setFormat() */
        m->formatObj->releaseCaller();
        m->formatObj.setNull();
    }

    if (m->state == MediumState_Deleting)
    {
        /* This medium has been already deleted (directly or as part of a
         * merge).  Reparenting has already been done. */
        Assert(m->pParent.isNull());
    }
    else
    {
        MediaList::iterator it;
        for (it = m->llChildren.begin();
            it != m->llChildren.end();
            ++it)
        {
            Medium *pChild = *it;
            pChild->m->pParent.setNull();
            pChild->uninit();
        }
        m->llChildren.clear();          // this unsets all the ComPtrs and probably calls delete

        if (m->pParent)
        {
            // this is a differencing disk: then remove it from the parent's children list
            deparent();
        }
    }

    unconst(m->pVirtualBox) = NULL;
}

/**
 * Internal helper that removes "this" from the list of children of its
 * parent. Used in uninit() and other places when reparenting is necessary.
 *
 * The caller must hold the medium tree lock!
 */
void Medium::deparent()
{
    MediaList &llParent = m->pParent->m->llChildren;
    for (MediaList::iterator it = llParent.begin();
         it != llParent.end();
         ++it)
    {
        Medium *pParentsChild = *it;
        if (this == pParentsChild)
        {
            llParent.erase(it);
            break;
        }
    }
    m->pParent.setNull();
}

/**
 * Internal helper that removes "this" from the list of children of its
 * parent. Used in uninit() and other places when reparenting is necessary.
 *
 * The caller must hold the medium tree lock!
 */
void Medium::setParent(const ComObjPtr<Medium> &pParent)
{
    m->pParent = pParent;
    if (pParent)
        pParent->m->llChildren.push_back(this);
}


////////////////////////////////////////////////////////////////////////////////
//
// IMedium public methods
//
////////////////////////////////////////////////////////////////////////////////

STDMETHODIMP Medium::COMGETTER(Id)(BSTR *aId)
{
    CheckComArgOutPointerValid(aId);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    m->id.toUtf16().cloneTo(aId);

    return S_OK;
}

STDMETHODIMP Medium::COMGETTER(Description)(BSTR *aDescription)
{
    CheckComArgOutPointerValid(aDescription);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    m->strDescription.cloneTo(aDescription);

    return S_OK;
}

STDMETHODIMP Medium::COMSETTER(Description)(IN_BSTR aDescription)
{
    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

//     AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);

    /// @todo update m->description and save the global registry (and local
    /// registries of portable VMs referring to this medium), this will also
    /// require to add the mRegistered flag to data

    NOREF(aDescription);

    ReturnComNotImplemented();
}

STDMETHODIMP Medium::COMGETTER(State)(MediumState_T *aState)
{
    CheckComArgOutPointerValid(aState);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);
    *aState = m->state;

    return S_OK;
}

STDMETHODIMP Medium::COMGETTER(Variant)(ULONG *aVariant)
{
    CheckComArgOutPointerValid(aVariant);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);
    *aVariant = m->variant;

    return S_OK;
}


STDMETHODIMP Medium::COMGETTER(Location)(BSTR *aLocation)
{
    CheckComArgOutPointerValid(aLocation);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    m->strLocationFull.cloneTo(aLocation);

    return S_OK;
}

STDMETHODIMP Medium::COMSETTER(Location)(IN_BSTR aLocation)
{
    CheckComArgStrNotEmptyOrNull(aLocation);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);

    /// @todo NEWMEDIA for file names, add the default extension if no extension
    /// is present (using the information from the VD backend which also implies
    /// that one more parameter should be passed to setLocation() requesting
    /// that functionality since it is only allowed when called from this method

    /// @todo NEWMEDIA rename the file and set m->location on success, then save
    /// the global registry (and local registries of portable VMs referring to
    /// this medium), this will also require to add the mRegistered flag to data

    ReturnComNotImplemented();
}

STDMETHODIMP Medium::COMGETTER(Name)(BSTR *aName)
{
    CheckComArgOutPointerValid(aName);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    getName().cloneTo(aName);

    return S_OK;
}

STDMETHODIMP Medium::COMGETTER(DeviceType)(DeviceType_T *aDeviceType)
{
    CheckComArgOutPointerValid(aDeviceType);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    *aDeviceType = m->devType;

    return S_OK;
}

STDMETHODIMP Medium::COMGETTER(HostDrive)(BOOL *aHostDrive)
{
    CheckComArgOutPointerValid(aHostDrive);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    *aHostDrive = m->hostDrive;

    return S_OK;
}

STDMETHODIMP Medium::COMGETTER(Size)(LONG64 *aSize)
{
    CheckComArgOutPointerValid(aSize);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    *aSize = m->size;

    return S_OK;
}

STDMETHODIMP Medium::COMGETTER(Format)(BSTR *aFormat)
{
    CheckComArgOutPointerValid(aFormat);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    /* no need to lock, m->strFormat is const */
    m->strFormat.cloneTo(aFormat);

    return S_OK;
}

STDMETHODIMP Medium::COMGETTER(MediumFormat)(IMediumFormat **aMediumFormat)
{
    CheckComArgOutPointerValid(aMediumFormat);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    /* no need to lock, m->formatObj is const */
    m->formatObj.queryInterfaceTo(aMediumFormat);

    return S_OK;
}

STDMETHODIMP Medium::COMGETTER(Type)(MediumType_T *aType)
{
    CheckComArgOutPointerValid(aType);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    *aType = m->type;

    return S_OK;
}

STDMETHODIMP Medium::COMSETTER(Type)(MediumType_T aType)
{
    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    // we access mParent and members
    AutoWriteLock treeLock(m->pVirtualBox->getMediaTreeLockHandle() COMMA_LOCKVAL_SRC_POS);
    AutoWriteLock mlock(this COMMA_LOCKVAL_SRC_POS);

    switch (m->state)
    {
        case MediumState_Created:
        case MediumState_Inaccessible:
            break;
        default:
            return setStateError();
    }

    if (m->type == aType)
    {
        /* Nothing to do */
        return S_OK;
    }

    DeviceType_T devType = getDeviceType();
    // DVD media can only be readonly.
    if (devType == DeviceType_DVD && aType != MediumType_Readonly)
        return setError(VBOX_E_INVALID_OBJECT_STATE,
                        tr("Cannot change the type of DVD medium '%s'"),
                        m->strLocationFull.c_str());
    // Floppy media can only be writethrough or readonly.
    if (   devType == DeviceType_Floppy
        && aType != MediumType_Writethrough
        && aType != MediumType_Readonly)
        return setError(VBOX_E_INVALID_OBJECT_STATE,
                        tr("Cannot change the type of floppy medium '%s'"),
                        m->strLocationFull.c_str());

    /* cannot change the type of a differencing medium */
    if (m->pParent)
        return setError(VBOX_E_INVALID_OBJECT_STATE,
                        tr("Cannot change the type of medium '%s' because it is a differencing medium"),
                        m->strLocationFull.c_str());

    /* Cannot change the type of a medium being in use by more than one VM.
     * If the change is to Immutable or MultiAttach then it must not be
     * directly attached to any VM, otherwise the assumptions about indirect
     * attachment elsewhere are violated and the VM becomes inaccessible.
     * Attaching an immutable medium triggers the diff creation, and this is
     * vital for the correct operation. */
    if (   m->backRefs.size() > 1
        || (   (   aType == MediumType_Immutable
                || aType == MediumType_MultiAttach)
            && m->backRefs.size() > 0))
        return setError(VBOX_E_INVALID_OBJECT_STATE,
                        tr("Cannot change the type of medium '%s' because it is attached to %d virtual machines"),
                        m->strLocationFull.c_str(), m->backRefs.size());

    switch (aType)
    {
        case MediumType_Normal:
        case MediumType_Immutable:
        case MediumType_MultiAttach:
        {
            /* normal can be easily converted to immutable and vice versa even
             * if they have children as long as they are not attached to any
             * machine themselves */
            break;
        }
        case MediumType_Writethrough:
        case MediumType_Shareable:
        case MediumType_Readonly:
        {
            /* cannot change to writethrough, shareable or readonly
             * if there are children */
            if (getChildren().size() != 0)
                return setError(VBOX_E_OBJECT_IN_USE,
                                tr("Cannot change type for medium '%s' since it has %d child media"),
                                m->strLocationFull.c_str(), getChildren().size());
            if (aType == MediumType_Shareable)
            {
                MediumVariant_T variant = getVariant();
                if (!(variant & MediumVariant_Fixed))
                    return setError(VBOX_E_INVALID_OBJECT_STATE,
                                    tr("Cannot change type for medium '%s' to 'Shareable' since it is a dynamic medium storage unit"),
                                    m->strLocationFull.c_str());
            }
            else if (aType == MediumType_Readonly && devType == DeviceType_HardDisk)
            {
                // Readonly hard disks are not allowed, this medium type is reserved for
                // DVDs and floppy images at the moment. Later we might allow readonly hard
                // disks, but that's extremely unusual and many guest OSes will have trouble.
                return setError(VBOX_E_INVALID_OBJECT_STATE,
                                tr("Cannot change type for medium '%s' to 'Readonly' since it is a hard disk"),
                                m->strLocationFull.c_str());
            }
            break;
        }
        default:
            AssertFailedReturn(E_FAIL);
    }

    if (aType == MediumType_MultiAttach)
    {
        // This type is new with VirtualBox 4.0 and therefore requires settings
        // version 1.11 in the settings backend. Unfortunately it is not enough to do
        // the usual routine in MachineConfigFile::bumpSettingsVersionIfNeeded() for
        // two reasons: The medium type is a property of the media registry tree, which
        // can reside in the global config file (for pre-4.0 media); we would therefore
        // possibly need to bump the global config version. We don't want to do that though
        // because that might make downgrading to pre-4.0 impossible.
        // As a result, we can only use these two new types if the medium is NOT in the
        // global registry:
        const Guid &uuidGlobalRegistry = m->pVirtualBox->getGlobalRegistryId();
        if (isInRegistry(uuidGlobalRegistry))
            return setError(VBOX_E_INVALID_OBJECT_STATE,
                            tr("Cannot change type for medium '%s': the media type 'MultiAttach' can only be used "
                               "on media registered with a machine that was created with VirtualBox 4.0 or later"),
                            m->strLocationFull.c_str());
    }

    m->type = aType;

    // save the settings
    mlock.release();
    treeLock.release();
    markRegistriesModified();
    m->pVirtualBox->saveModifiedRegistries();

    return S_OK;
}

STDMETHODIMP Medium::COMGETTER(AllowedTypes)(ComSafeArrayOut(MediumType_T, aAllowedTypes))
{
    CheckComArgOutSafeArrayPointerValid(aAllowedTypes);
    NOREF(aAllowedTypes);
#ifndef RT_OS_WINDOWS
    NOREF(aAllowedTypesSize);
#endif

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    ReturnComNotImplemented();
}

STDMETHODIMP Medium::COMGETTER(Parent)(IMedium **aParent)
{
    CheckComArgOutPointerValid(aParent);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    /* we access mParent */
    AutoReadLock treeLock(m->pVirtualBox->getMediaTreeLockHandle() COMMA_LOCKVAL_SRC_POS);

    m->pParent.queryInterfaceTo(aParent);

    return S_OK;
}

STDMETHODIMP Medium::COMGETTER(Children)(ComSafeArrayOut(IMedium *, aChildren))
{
    CheckComArgOutSafeArrayPointerValid(aChildren);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    /* we access children */
    AutoReadLock treeLock(m->pVirtualBox->getMediaTreeLockHandle() COMMA_LOCKVAL_SRC_POS);

    SafeIfaceArray<IMedium> children(this->getChildren());
    children.detachTo(ComSafeArrayOutArg(aChildren));

    return S_OK;
}

STDMETHODIMP Medium::COMGETTER(Base)(IMedium **aBase)
{
    CheckComArgOutPointerValid(aBase);

    /* base() will do callers/locking */

    getBase().queryInterfaceTo(aBase);

    return S_OK;
}

STDMETHODIMP Medium::COMGETTER(ReadOnly)(BOOL *aReadOnly)
{
    CheckComArgOutPointerValid(aReadOnly);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    /* isReadOnly() will do locking */

    *aReadOnly = isReadOnly();

    return S_OK;
}

STDMETHODIMP Medium::COMGETTER(LogicalSize)(LONG64 *aLogicalSize)
{
    CheckComArgOutPointerValid(aLogicalSize);

    {
        AutoCaller autoCaller(this);
        if (FAILED(autoCaller.rc())) return autoCaller.rc();

        /* we access mParent */
        AutoReadLock treeLock(m->pVirtualBox->getMediaTreeLockHandle() COMMA_LOCKVAL_SRC_POS);

        AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

        if (m->pParent.isNull())
        {
            *aLogicalSize = m->logicalSize;

            return S_OK;
        }
    }

    /* We assume that some backend may decide to return a meaningless value in
     * response to VDGetSize() for differencing media and therefore always
     * ask the base medium ourselves. */

    /* base() will do callers/locking */

    return getBase()->COMGETTER(LogicalSize)(aLogicalSize);
}

STDMETHODIMP Medium::COMGETTER(AutoReset)(BOOL *aAutoReset)
{
    CheckComArgOutPointerValid(aAutoReset);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    if (m->pParent.isNull())
        *aAutoReset = FALSE;
    else
        *aAutoReset = m->autoReset;

    return S_OK;
}

STDMETHODIMP Medium::COMSETTER(AutoReset)(BOOL aAutoReset)
{
    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoWriteLock mlock(this COMMA_LOCKVAL_SRC_POS);

    if (m->pParent.isNull())
        return setError(VBOX_E_NOT_SUPPORTED,
                        tr("Medium '%s' is not differencing"),
                        m->strLocationFull.c_str());

    if (m->autoReset != !!aAutoReset)
    {
        m->autoReset = !!aAutoReset;

        // save the settings
        mlock.release();
        markRegistriesModified();
        m->pVirtualBox->saveModifiedRegistries();
    }

    return S_OK;
}

STDMETHODIMP Medium::COMGETTER(LastAccessError)(BSTR *aLastAccessError)
{
    CheckComArgOutPointerValid(aLastAccessError);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    m->strLastAccessError.cloneTo(aLastAccessError);

    return S_OK;
}

STDMETHODIMP Medium::COMGETTER(MachineIds)(ComSafeArrayOut(BSTR,aMachineIds))
{
    CheckComArgOutSafeArrayPointerValid(aMachineIds);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    com::SafeArray<BSTR> machineIds;

    if (m->backRefs.size() != 0)
    {
        machineIds.reset(m->backRefs.size());

        size_t i = 0;
        for (BackRefList::const_iterator it = m->backRefs.begin();
             it != m->backRefs.end(); ++it, ++i)
        {
             it->machineId.toUtf16().detachTo(&machineIds[i]);
        }
    }

    machineIds.detachTo(ComSafeArrayOutArg(aMachineIds));

    return S_OK;
}

STDMETHODIMP Medium::SetIds(BOOL aSetImageId,
                            IN_BSTR aImageId,
                            BOOL aSetParentId,
                            IN_BSTR aParentId)
{
    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);

    switch (m->state)
    {
        case MediumState_Created:
            break;
        default:
            return setStateError();
    }

    Guid imageId, parentId;
    if (aSetImageId)
    {
        if (Bstr(aImageId).isEmpty())
            imageId.create();
        else
        {
            imageId = Guid(aImageId);
            if (imageId.isEmpty())
                return setError(E_INVALIDARG, tr("Argument %s is empty"), "aImageId");
        }
    }
    if (aSetParentId)
    {
        if (Bstr(aParentId).isEmpty())
            parentId.create();
        else
            parentId = Guid(aParentId);
    }

    unconst(m->uuidImage) = imageId;
    unconst(m->uuidParentImage) = parentId;

    // must not hold any locks before calling Medium::queryInfo
    alock.release();

    HRESULT rc = queryInfo(!!aSetImageId /* fSetImageId */,
                           !!aSetParentId /* fSetParentId */);

    return rc;
}

STDMETHODIMP Medium::RefreshState(MediumState_T *aState)
{
    CheckComArgOutPointerValid(aState);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    HRESULT rc = S_OK;

    switch (m->state)
    {
        case MediumState_Created:
        case MediumState_Inaccessible:
        case MediumState_LockedRead:
        {
            // must not hold any locks before calling Medium::queryInfo
            alock.release();

            rc = queryInfo(false /* fSetImageId */, false /* fSetParentId */);

            alock.acquire();
            break;
        }
        default:
            break;
    }

    *aState = m->state;

    return rc;
}

STDMETHODIMP Medium::GetSnapshotIds(IN_BSTR aMachineId,
                                    ComSafeArrayOut(BSTR, aSnapshotIds))
{
    CheckComArgExpr(aMachineId, Guid(aMachineId).isEmpty() == false);
    CheckComArgOutSafeArrayPointerValid(aSnapshotIds);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    com::SafeArray<BSTR> snapshotIds;

    Guid id(aMachineId);
    for (BackRefList::const_iterator it = m->backRefs.begin();
         it != m->backRefs.end(); ++it)
    {
        if (it->machineId == id)
        {
            size_t size = it->llSnapshotIds.size();

            /* if the medium is attached to the machine in the current state, we
             * return its ID as the first element of the array */
            if (it->fInCurState)
                ++size;

            if (size > 0)
            {
                snapshotIds.reset(size);

                size_t j = 0;
                if (it->fInCurState)
                    it->machineId.toUtf16().detachTo(&snapshotIds[j++]);

                for (GuidList::const_iterator jt = it->llSnapshotIds.begin();
                     jt != it->llSnapshotIds.end();
                     ++jt, ++j)
                {
                     (*jt).toUtf16().detachTo(&snapshotIds[j]);
                }
            }

            break;
        }
    }

    snapshotIds.detachTo(ComSafeArrayOutArg(aSnapshotIds));

    return S_OK;
}

/**
 * @note @a aState may be NULL if the state value is not needed (only for
 *       in-process calls).
 */
STDMETHODIMP Medium::LockRead(MediumState_T *aState)
{
    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    /* Must not hold the object lock, as we need control over it below. */
    Assert(!isWriteLockOnCurrentThread());
    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);

    /* Wait for a concurrently running Medium::queryInfo to complete. */
    if (m->queryInfoRunning)
    {
        /* Must not hold the media tree lock, as Medium::queryInfo needs this
         * lock and thus we would run into a deadlock here. */
        Assert(!m->pVirtualBox->getMediaTreeLockHandle().isWriteLockOnCurrentThread());
        while (m->queryInfoRunning)
        {
            alock.release();
            {
                AutoReadLock qlock(m->queryInfoSem COMMA_LOCKVAL_SRC_POS);
            }
            alock.acquire();
        }
    }

    /* return the current state before */
    if (aState)
        *aState = m->state;

    HRESULT rc = S_OK;

    switch (m->state)
    {
        case MediumState_Created:
        case MediumState_Inaccessible:
        case MediumState_LockedRead:
        {
            ++m->readers;

            ComAssertMsgBreak(m->readers != 0, ("Counter overflow"), rc = E_FAIL);

            /* Remember pre-lock state */
            if (m->state != MediumState_LockedRead)
                m->preLockState = m->state;

            LogFlowThisFunc(("Okay - prev state=%d readers=%d\n", m->state, m->readers));
            m->state = MediumState_LockedRead;

            break;
        }
        default:
        {
            LogFlowThisFunc(("Failing - state=%d\n", m->state));
            rc = setStateError();
            break;
        }
    }

    return rc;
}

/**
 * @note @a aState may be NULL if the state value is not needed (only for
 *       in-process calls).
 */
STDMETHODIMP Medium::UnlockRead(MediumState_T *aState)
{
    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);

    HRESULT rc = S_OK;

    switch (m->state)
    {
        case MediumState_LockedRead:
        {
            ComAssertMsgBreak(m->readers != 0, ("Counter underflow"), rc = E_FAIL);
            --m->readers;

            /* Reset the state after the last reader */
            if (m->readers == 0)
            {
                m->state = m->preLockState;
                /* There are cases where we inject the deleting state into
                 * a medium locked for reading. Make sure #unmarkForDeletion()
                 * gets the right state afterwards. */
                if (m->preLockState == MediumState_Deleting)
                    m->preLockState = MediumState_Created;
            }

            LogFlowThisFunc(("new state=%d\n", m->state));
            break;
        }
        default:
        {
            LogFlowThisFunc(("Failing - state=%d\n", m->state));
            rc = setError(VBOX_E_INVALID_OBJECT_STATE,
                          tr("Medium '%s' is not locked for reading"),
                          m->strLocationFull.c_str());
            break;
        }
    }

    /* return the current state after */
    if (aState)
        *aState = m->state;

    return rc;
}

/**
 * @note @a aState may be NULL if the state value is not needed (only for
 *       in-process calls).
 */
STDMETHODIMP Medium::LockWrite(MediumState_T *aState)
{
    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    /* Must not hold the object lock, as we need control over it below. */
    Assert(!isWriteLockOnCurrentThread());
    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);

    /* Wait for a concurrently running Medium::queryInfo to complete. */
    if (m->queryInfoRunning)
    {
        /* Must not hold the media tree lock, as Medium::queryInfo needs this
         * lock and thus we would run into a deadlock here. */
        Assert(!m->pVirtualBox->getMediaTreeLockHandle().isWriteLockOnCurrentThread());
        while (m->queryInfoRunning)
        {
            alock.release();
            {
                AutoReadLock qlock(m->queryInfoSem COMMA_LOCKVAL_SRC_POS);
            }
            alock.acquire();
        }
    }

    /* return the current state before */
    if (aState)
        *aState = m->state;

    HRESULT rc = S_OK;

    switch (m->state)
    {
        case MediumState_Created:
        case MediumState_Inaccessible:
        {
            m->preLockState = m->state;

            LogFlowThisFunc(("Okay - prev state=%d locationFull=%s\n", m->state, getLocationFull().c_str()));
            m->state = MediumState_LockedWrite;
            break;
        }
        default:
        {
            LogFlowThisFunc(("Failing - state=%d locationFull=%s\n", m->state, getLocationFull().c_str()));
            rc = setStateError();
            break;
        }
    }

    return rc;
}

/**
 * @note @a aState may be NULL if the state value is not needed (only for
 *       in-process calls).
 */
STDMETHODIMP Medium::UnlockWrite(MediumState_T *aState)
{
    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);

    HRESULT rc = S_OK;

    switch (m->state)
    {
        case MediumState_LockedWrite:
        {
            m->state = m->preLockState;
            /* There are cases where we inject the deleting state into
             * a medium locked for writing. Make sure #unmarkForDeletion()
             * gets the right state afterwards. */
            if (m->preLockState == MediumState_Deleting)
                m->preLockState = MediumState_Created;
            LogFlowThisFunc(("new state=%d locationFull=%s\n", m->state, getLocationFull().c_str()));
            break;
        }
        default:
        {
            LogFlowThisFunc(("Failing - state=%d locationFull=%s\n", m->state, getLocationFull().c_str()));
            rc = setError(VBOX_E_INVALID_OBJECT_STATE,
                          tr("Medium '%s' is not locked for writing"),
                          m->strLocationFull.c_str());
            break;
        }
    }

    /* return the current state after */
    if (aState)
        *aState = m->state;

    return rc;
}

STDMETHODIMP Medium::Close()
{
    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    // make a copy of VirtualBox pointer which gets nulled by uninit()
    ComObjPtr<VirtualBox> pVirtualBox(m->pVirtualBox);

    MultiResult mrc = close(autoCaller);

    pVirtualBox->saveModifiedRegistries();

    return mrc;
}

STDMETHODIMP Medium::GetProperty(IN_BSTR aName, BSTR *aValue)
{
    CheckComArgStrNotEmptyOrNull(aName);
    CheckComArgOutPointerValid(aValue);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    settings::StringsMap::const_iterator it = m->mapProperties.find(Utf8Str(aName));
    if (it == m->mapProperties.end())
        return setError(VBOX_E_OBJECT_NOT_FOUND,
                        tr("Property '%ls' does not exist"), aName);

    it->second.cloneTo(aValue);

    return S_OK;
}

STDMETHODIMP Medium::SetProperty(IN_BSTR aName, IN_BSTR aValue)
{
    CheckComArgStrNotEmptyOrNull(aName);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoWriteLock mlock(this COMMA_LOCKVAL_SRC_POS);

    switch (m->state)
    {
        case MediumState_Created:
        case MediumState_Inaccessible:
            break;
        default:
            return setStateError();
    }

    settings::StringsMap::iterator it = m->mapProperties.find(Utf8Str(aName));
    if (it == m->mapProperties.end())
        return setError(VBOX_E_OBJECT_NOT_FOUND,
                        tr("Property '%ls' does not exist"),
                        aName);

    it->second = aValue;

    // save the settings
    mlock.release();
    markRegistriesModified();
    m->pVirtualBox->saveModifiedRegistries();

    return S_OK;
}

STDMETHODIMP Medium::GetProperties(IN_BSTR aNames,
                                   ComSafeArrayOut(BSTR, aReturnNames),
                                   ComSafeArrayOut(BSTR, aReturnValues))
{
    CheckComArgOutSafeArrayPointerValid(aReturnNames);
    CheckComArgOutSafeArrayPointerValid(aReturnValues);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    /// @todo make use of aNames according to the documentation
    NOREF(aNames);

    com::SafeArray<BSTR> names(m->mapProperties.size());
    com::SafeArray<BSTR> values(m->mapProperties.size());
    size_t i = 0;

    for (settings::StringsMap::const_iterator it = m->mapProperties.begin();
         it != m->mapProperties.end();
         ++it)
    {
        it->first.cloneTo(&names[i]);
        it->second.cloneTo(&values[i]);
        ++i;
    }

    names.detachTo(ComSafeArrayOutArg(aReturnNames));
    values.detachTo(ComSafeArrayOutArg(aReturnValues));

    return S_OK;
}

STDMETHODIMP Medium::SetProperties(ComSafeArrayIn(IN_BSTR, aNames),
                                   ComSafeArrayIn(IN_BSTR, aValues))
{
    CheckComArgSafeArrayNotNull(aNames);
    CheckComArgSafeArrayNotNull(aValues);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoWriteLock mlock(this COMMA_LOCKVAL_SRC_POS);

    com::SafeArray<IN_BSTR> names(ComSafeArrayInArg(aNames));
    com::SafeArray<IN_BSTR> values(ComSafeArrayInArg(aValues));

    /* first pass: validate names */
    for (size_t i = 0;
         i < names.size();
         ++i)
    {
        if (m->mapProperties.find(Utf8Str(names[i])) == m->mapProperties.end())
            return setError(VBOX_E_OBJECT_NOT_FOUND,
                            tr("Property '%ls' does not exist"), names[i]);
    }

    /* second pass: assign */
    for (size_t i = 0;
         i < names.size();
         ++i)
    {
        settings::StringsMap::iterator it = m->mapProperties.find(Utf8Str(names[i]));
        AssertReturn(it != m->mapProperties.end(), E_FAIL);

        it->second = Utf8Str(values[i]);
    }

    // save the settings
    mlock.release();
    markRegistriesModified();
    m->pVirtualBox->saveModifiedRegistries();

    return S_OK;
}

STDMETHODIMP Medium::CreateBaseStorage(LONG64 aLogicalSize,
                                       ULONG aVariant,
                                       IProgress **aProgress)
{
    CheckComArgOutPointerValid(aProgress);
    if (aLogicalSize < 0)
        return setError(E_INVALIDARG, tr("The medium size argument (%lld) is negative"), aLogicalSize);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    HRESULT rc = S_OK;
    ComObjPtr <Progress> pProgress;
    Medium::Task *pTask = NULL;

    try
    {
        AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);

        aVariant = (MediumVariant_T)((unsigned)aVariant & (unsigned)~MediumVariant_Diff);
        if (    !(aVariant & MediumVariant_Fixed)
            &&  !(m->formatObj->getCapabilities() & MediumFormatCapabilities_CreateDynamic))
            throw setError(VBOX_E_NOT_SUPPORTED,
                           tr("Medium format '%s' does not support dynamic storage creation"),
                           m->strFormat.c_str());
        if (    (aVariant & MediumVariant_Fixed)
            &&  !(m->formatObj->getCapabilities() & MediumFormatCapabilities_CreateDynamic))
            throw setError(VBOX_E_NOT_SUPPORTED,
                           tr("Medium format '%s' does not support fixed storage creation"),
                           m->strFormat.c_str());

        if (m->state != MediumState_NotCreated)
            throw setStateError();

        pProgress.createObject();
        rc = pProgress->init(m->pVirtualBox,
                             static_cast<IMedium*>(this),
                             (aVariant & MediumVariant_Fixed)
                               ? BstrFmt(tr("Creating fixed medium storage unit '%s'"), m->strLocationFull.c_str()).raw()
                               : BstrFmt(tr("Creating dynamic medium storage unit '%s'"), m->strLocationFull.c_str()).raw(),
                             TRUE /* aCancelable */);
        if (FAILED(rc))
            throw rc;

        /* setup task object to carry out the operation asynchronously */
        pTask = new Medium::CreateBaseTask(this, pProgress, aLogicalSize,
                                           (MediumVariant_T)aVariant);
        rc = pTask->rc();
        AssertComRC(rc);
        if (FAILED(rc))
            throw rc;

        m->state = MediumState_Creating;
    }
    catch (HRESULT aRC) { rc = aRC; }

    if (SUCCEEDED(rc))
    {
        rc = startThread(pTask);

        if (SUCCEEDED(rc))
            pProgress.queryInterfaceTo(aProgress);
    }
    else if (pTask != NULL)
        delete pTask;

    return rc;
}

STDMETHODIMP Medium::DeleteStorage(IProgress **aProgress)
{
    CheckComArgOutPointerValid(aProgress);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    ComObjPtr<Progress> pProgress;

    MultiResult mrc = deleteStorage(&pProgress,
                                    false /* aWait */);
    /* Must save the registries in any case, since an entry was removed. */
    m->pVirtualBox->saveModifiedRegistries();

    if (SUCCEEDED(mrc))
        pProgress.queryInterfaceTo(aProgress);

    return mrc;
}

STDMETHODIMP Medium::CreateDiffStorage(IMedium *aTarget,
                                       ULONG aVariant,
                                       IProgress **aProgress)
{
    CheckComArgNotNull(aTarget);
    CheckComArgOutPointerValid(aProgress);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    ComObjPtr<Medium> diff = static_cast<Medium*>(aTarget);

    // locking: we need the tree lock first because we access parent pointers
    AutoMultiWriteLock3 alock(&m->pVirtualBox->getMediaTreeLockHandle(),
                              this->lockHandle(), diff->lockHandle() COMMA_LOCKVAL_SRC_POS);

    if (m->type == MediumType_Writethrough)
        return setError(VBOX_E_INVALID_OBJECT_STATE,
                        tr("Medium type of '%s' is Writethrough"),
                        m->strLocationFull.c_str());
    else if (m->type == MediumType_Shareable)
        return setError(VBOX_E_INVALID_OBJECT_STATE,
                        tr("Medium type of '%s' is Shareable"),
                        m->strLocationFull.c_str());
    else if (m->type == MediumType_Readonly)
        return setError(VBOX_E_INVALID_OBJECT_STATE,
                        tr("Medium type of '%s' is Readonly"),
                        m->strLocationFull.c_str());

    /* Apply the normal locking logic to the entire chain. */
    MediumLockList *pMediumLockList(new MediumLockList());
    alock.release();
    HRESULT rc = diff->createMediumLockList(true /* fFailIfInaccessible */,
                                            true /* fMediumLockWrite */,
                                            this,
                                            *pMediumLockList);
    alock.acquire();
    if (FAILED(rc))
    {
        delete pMediumLockList;
        return rc;
    }

    alock.release();
    rc = pMediumLockList->Lock();
    alock.acquire();
    if (FAILED(rc))
    {
        delete pMediumLockList;

        return setError(rc, tr("Could not lock medium when creating diff '%s'"),
                        diff->getLocationFull().c_str());
    }

    Guid parentMachineRegistry;
    if (getFirstRegistryMachineId(parentMachineRegistry))
    {
        /* since this medium has been just created it isn't associated yet */
        diff->m->llRegistryIDs.push_back(parentMachineRegistry);
        alock.release();
        diff->markRegistriesModified();
        alock.acquire();
    }

    alock.release();

    ComObjPtr <Progress> pProgress;

    rc = createDiffStorage(diff, (MediumVariant_T)aVariant, pMediumLockList,
                           &pProgress, false /* aWait */);
    if (FAILED(rc))
        delete pMediumLockList;
    else
        pProgress.queryInterfaceTo(aProgress);

    return rc;
}

STDMETHODIMP Medium::MergeTo(IMedium *aTarget, IProgress **aProgress)
{
    CheckComArgNotNull(aTarget);
    CheckComArgOutPointerValid(aProgress);
    ComAssertRet(aTarget != this, E_INVALIDARG);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    ComObjPtr<Medium> pTarget = static_cast<Medium*>(aTarget);

    bool fMergeForward = false;
    ComObjPtr<Medium> pParentForTarget;
    MediaList childrenToReparent;
    MediumLockList *pMediumLockList = NULL;

    HRESULT rc = S_OK;

    rc = prepareMergeTo(pTarget, NULL, NULL, true, fMergeForward,
                        pParentForTarget, childrenToReparent, pMediumLockList);
    if (FAILED(rc)) return rc;

    ComObjPtr <Progress> pProgress;

    rc = mergeTo(pTarget, fMergeForward, pParentForTarget, childrenToReparent,
                 pMediumLockList, &pProgress, false /* aWait */);
    if (FAILED(rc))
        cancelMergeTo(childrenToReparent, pMediumLockList);
    else
        pProgress.queryInterfaceTo(aProgress);

    return rc;
}

STDMETHODIMP Medium::CloneToBase(IMedium   *aTarget,
                                 ULONG     aVariant,
                                 IProgress **aProgress)
{
     int rc = S_OK;
     CheckComArgNotNull(aTarget);
     CheckComArgOutPointerValid(aProgress);
     rc =  CloneTo(aTarget, aVariant, NULL, aProgress);
     return rc;
}

STDMETHODIMP Medium::CloneTo(IMedium *aTarget,
                             ULONG aVariant,
                             IMedium *aParent,
                             IProgress **aProgress)
{
    CheckComArgNotNull(aTarget);
    CheckComArgOutPointerValid(aProgress);
    ComAssertRet(aTarget != this, E_INVALIDARG);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    ComObjPtr<Medium> pTarget = static_cast<Medium*>(aTarget);
    ComObjPtr<Medium> pParent;
    if (aParent)
        pParent = static_cast<Medium*>(aParent);

    HRESULT rc = S_OK;
    ComObjPtr<Progress> pProgress;
    Medium::Task *pTask = NULL;

    try
    {
        // locking: we need the tree lock first because we access parent pointers
        // and we need to write-lock the media involved
        uint32_t    cHandles    = 3;
        LockHandle* pHandles[4] = { &m->pVirtualBox->getMediaTreeLockHandle(),
                                    this->lockHandle(),
                                    pTarget->lockHandle() };
        /* Only add parent to the lock if it is not null */
        if (!pParent.isNull())
            pHandles[cHandles++] = pParent->lockHandle();
        AutoWriteLock alock(cHandles,
                            pHandles
                            COMMA_LOCKVAL_SRC_POS);

        if (    pTarget->m->state != MediumState_NotCreated
            &&  pTarget->m->state != MediumState_Created)
            throw pTarget->setStateError();

        /* Build the source lock list. */
        MediumLockList *pSourceMediumLockList(new MediumLockList());
        alock.release();
        rc = createMediumLockList(true /* fFailIfInaccessible */,
                                  false /* fMediumLockWrite */,
                                  NULL,
                                  *pSourceMediumLockList);
        alock.acquire();
        if (FAILED(rc))
        {
            delete pSourceMediumLockList;
            throw rc;
        }

        /* Build the target lock list (including the to-be parent chain). */
        MediumLockList *pTargetMediumLockList(new MediumLockList());
        alock.release();
        rc = pTarget->createMediumLockList(true /* fFailIfInaccessible */,
                                           true /* fMediumLockWrite */,
                                           pParent,
                                           *pTargetMediumLockList);
        alock.acquire();
        if (FAILED(rc))
        {
            delete pSourceMediumLockList;
            delete pTargetMediumLockList;
            throw rc;
        }

        alock.release();
        rc = pSourceMediumLockList->Lock();
        alock.acquire();
        if (FAILED(rc))
        {
            delete pSourceMediumLockList;
            delete pTargetMediumLockList;
            throw setError(rc,
                           tr("Failed to lock source media '%s'"),
                           getLocationFull().c_str());
        }
        alock.release();
        rc = pTargetMediumLockList->Lock();
        alock.acquire();
        if (FAILED(rc))
        {
            delete pSourceMediumLockList;
            delete pTargetMediumLockList;
            throw setError(rc,
                           tr("Failed to lock target media '%s'"),
                           pTarget->getLocationFull().c_str());
        }

        pProgress.createObject();
        rc = pProgress->init(m->pVirtualBox,
                             static_cast <IMedium *>(this),
                             BstrFmt(tr("Creating clone medium '%s'"), pTarget->m->strLocationFull.c_str()).raw(),
                             TRUE /* aCancelable */);
        if (FAILED(rc))
        {
            delete pSourceMediumLockList;
            delete pTargetMediumLockList;
            throw rc;
        }

        /* setup task object to carry out the operation asynchronously */
        pTask = new Medium::CloneTask(this, pProgress, pTarget,
                                      (MediumVariant_T)aVariant,
                                      pParent, UINT32_MAX, UINT32_MAX,
                                      pSourceMediumLockList, pTargetMediumLockList);
        rc = pTask->rc();
        AssertComRC(rc);
        if (FAILED(rc))
            throw rc;

        if (pTarget->m->state == MediumState_NotCreated)
            pTarget->m->state = MediumState_Creating;
    }
    catch (HRESULT aRC) { rc = aRC; }

    if (SUCCEEDED(rc))
    {
        rc = startThread(pTask);

        if (SUCCEEDED(rc))
            pProgress.queryInterfaceTo(aProgress);
    }
    else if (pTask != NULL)
        delete pTask;

    return rc;
}

STDMETHODIMP Medium::Compact(IProgress **aProgress)
{
    CheckComArgOutPointerValid(aProgress);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    HRESULT rc = S_OK;
    ComObjPtr <Progress> pProgress;
    Medium::Task *pTask = NULL;

    try
    {
        AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);

        /* Build the medium lock list. */
        MediumLockList *pMediumLockList(new MediumLockList());
        alock.release();
        rc = createMediumLockList(true /* fFailIfInaccessible */ ,
                                  true /* fMediumLockWrite */,
                                  NULL,
                                  *pMediumLockList);
        alock.acquire();
        if (FAILED(rc))
        {
            delete pMediumLockList;
            throw rc;
        }

        alock.release();
        rc = pMediumLockList->Lock();
        alock.acquire();
        if (FAILED(rc))
        {
            delete pMediumLockList;
            throw setError(rc,
                           tr("Failed to lock media when compacting '%s'"),
                           getLocationFull().c_str());
        }

        pProgress.createObject();
        rc = pProgress->init(m->pVirtualBox,
                             static_cast <IMedium *>(this),
                             BstrFmt(tr("Compacting medium '%s'"), m->strLocationFull.c_str()).raw(),
                             TRUE /* aCancelable */);
        if (FAILED(rc))
        {
            delete pMediumLockList;
            throw rc;
        }

        /* setup task object to carry out the operation asynchronously */
        pTask = new Medium::CompactTask(this, pProgress, pMediumLockList);
        rc = pTask->rc();
        AssertComRC(rc);
        if (FAILED(rc))
            throw rc;
    }
    catch (HRESULT aRC) { rc = aRC; }

    if (SUCCEEDED(rc))
    {
        rc = startThread(pTask);

        if (SUCCEEDED(rc))
            pProgress.queryInterfaceTo(aProgress);
    }
    else if (pTask != NULL)
        delete pTask;

    return rc;
}

STDMETHODIMP Medium::Resize(LONG64 aLogicalSize, IProgress **aProgress)
{
    CheckComArgOutPointerValid(aProgress);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    HRESULT rc = S_OK;
    ComObjPtr <Progress> pProgress;
    Medium::Task *pTask = NULL;

    try
    {
        AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);

        /* Build the medium lock list. */
        MediumLockList *pMediumLockList(new MediumLockList());
        alock.release();
        rc = createMediumLockList(true /* fFailIfInaccessible */ ,
                                  true /* fMediumLockWrite */,
                                  NULL,
                                  *pMediumLockList);
        alock.acquire();
        if (FAILED(rc))
        {
            delete pMediumLockList;
            throw rc;
        }

        alock.release();
        rc = pMediumLockList->Lock();
        alock.acquire();
        if (FAILED(rc))
        {
            delete pMediumLockList;
            throw setError(rc,
                           tr("Failed to lock media when compacting '%s'"),
                           getLocationFull().c_str());
        }

        pProgress.createObject();
        rc = pProgress->init(m->pVirtualBox,
                             static_cast <IMedium *>(this),
                             BstrFmt(tr("Compacting medium '%s'"), m->strLocationFull.c_str()).raw(),
                             TRUE /* aCancelable */);
        if (FAILED(rc))
        {
            delete pMediumLockList;
            throw rc;
        }

        /* setup task object to carry out the operation asynchronously */
        pTask = new Medium::ResizeTask(this, aLogicalSize, pProgress, pMediumLockList);
        rc = pTask->rc();
        AssertComRC(rc);
        if (FAILED(rc))
            throw rc;
    }
    catch (HRESULT aRC) { rc = aRC; }

    if (SUCCEEDED(rc))
    {
        rc = startThread(pTask);

        if (SUCCEEDED(rc))
            pProgress.queryInterfaceTo(aProgress);
    }
    else if (pTask != NULL)
        delete pTask;

    return rc;
}

STDMETHODIMP Medium::Reset(IProgress **aProgress)
{
    CheckComArgOutPointerValid(aProgress);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    HRESULT rc = S_OK;
    ComObjPtr <Progress> pProgress;
    Medium::Task *pTask = NULL;

    try
    {
        /* canClose() needs the tree lock */
        AutoMultiWriteLock2 multilock(&m->pVirtualBox->getMediaTreeLockHandle(),
                                      this->lockHandle()
                                      COMMA_LOCKVAL_SRC_POS);

        LogFlowThisFunc(("ENTER for medium %s\n", m->strLocationFull.c_str()));

        if (m->pParent.isNull())
            throw setError(VBOX_E_NOT_SUPPORTED,
                           tr("Medium type of '%s' is not differencing"),
                           m->strLocationFull.c_str());

        rc = canClose();
        if (FAILED(rc))
            throw rc;

        /* Build the medium lock list. */
        MediumLockList *pMediumLockList(new MediumLockList());
        multilock.release();
        rc = createMediumLockList(true /* fFailIfInaccessible */,
                                  true /* fMediumLockWrite */,
                                  NULL,
                                  *pMediumLockList);
        multilock.acquire();
        if (FAILED(rc))
        {
            delete pMediumLockList;
            throw rc;
        }

        multilock.release();
        rc = pMediumLockList->Lock();
        multilock.acquire();
        if (FAILED(rc))
        {
            delete pMediumLockList;
            throw setError(rc,
                           tr("Failed to lock media when resetting '%s'"),
                           getLocationFull().c_str());
        }

        pProgress.createObject();
        rc = pProgress->init(m->pVirtualBox,
                             static_cast<IMedium*>(this),
                             BstrFmt(tr("Resetting differencing medium '%s'"), m->strLocationFull.c_str()).raw(),
                             FALSE /* aCancelable */);
        if (FAILED(rc))
            throw rc;

        /* setup task object to carry out the operation asynchronously */
        pTask = new Medium::ResetTask(this, pProgress, pMediumLockList);
        rc = pTask->rc();
        AssertComRC(rc);
        if (FAILED(rc))
            throw rc;
    }
    catch (HRESULT aRC) { rc = aRC; }

    if (SUCCEEDED(rc))
    {
        rc = startThread(pTask);

        if (SUCCEEDED(rc))
            pProgress.queryInterfaceTo(aProgress);
    }
    else
    {
        /* Note: on success, the task will unlock this */
        {
            AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);
            HRESULT rc2 = UnlockWrite(NULL);
            AssertComRC(rc2);
        }
        if (pTask != NULL)
            delete pTask;
    }

    LogFlowThisFunc(("LEAVE, rc=%Rhrc\n", rc));

    return rc;
}

////////////////////////////////////////////////////////////////////////////////
//
// Medium public internal methods
//
////////////////////////////////////////////////////////////////////////////////

/**
 * Internal method to return the medium's parent medium. Must have caller + locking!
 * @return
 */
const ComObjPtr<Medium>& Medium::getParent() const
{
    return m->pParent;
}

/**
 * Internal method to return the medium's list of child media. Must have caller + locking!
 * @return
 */
const MediaList& Medium::getChildren() const
{
    return m->llChildren;
}

/**
 * Internal method to return the medium's GUID. Must have caller + locking!
 * @return
 */
const Guid& Medium::getId() const
{
    return m->id;
}

/**
 * Internal method to return the medium's state. Must have caller + locking!
 * @return
 */
MediumState_T Medium::getState() const
{
    return m->state;
}

/**
 * Internal method to return the medium's variant. Must have caller + locking!
 * @return
 */
MediumVariant_T Medium::getVariant() const
{
    return m->variant;
}

/**
 * Internal method which returns true if this medium represents a host drive.
 * @return
 */
bool Medium::isHostDrive() const
{
    return m->hostDrive;
}

/**
 * Internal method to return the medium's full location. Must have caller + locking!
 * @return
 */
const Utf8Str& Medium::getLocationFull() const
{
    return m->strLocationFull;
}

/**
 * Internal method to return the medium's format string. Must have caller + locking!
 * @return
 */
const Utf8Str& Medium::getFormat() const
{
    return m->strFormat;
}

/**
 * Internal method to return the medium's format object. Must have caller + locking!
 * @return
 */
const ComObjPtr<MediumFormat>& Medium::getMediumFormat() const
{
    return m->formatObj;
}

/**
 * Internal method that returns true if the medium is represented by a file on the host disk
 * (and not iSCSI or something).
 * @return
 */
bool Medium::isMediumFormatFile() const
{
    if (    m->formatObj
         && (m->formatObj->getCapabilities() & MediumFormatCapabilities_File)
       )
        return true;
    return false;
}

/**
 * Internal method to return the medium's size. Must have caller + locking!
 * @return
 */
uint64_t Medium::getSize() const
{
    return m->size;
}

/**
 * Returns the medium device type. Must have caller + locking!
 * @return
 */
DeviceType_T Medium::getDeviceType() const
{
    return m->devType;
}

/**
 * Returns the medium type. Must have caller + locking!
 * @return
 */
MediumType_T Medium::getType() const
{
    return m->type;
}

/**
 * Returns a short version of the location attribute.
 *
 * @note Must be called from under this object's read or write lock.
 */
Utf8Str Medium::getName()
{
    Utf8Str name = RTPathFilename(m->strLocationFull.c_str());
    return name;
}

/**
 * This adds the given UUID to the list of media registries in which this
 * medium should be registered. The UUID can either be a machine UUID,
 * to add a machine registry, or the global registry UUID as returned by
 * VirtualBox::getGlobalRegistryId().
 *
 * Note that for hard disks, this method does nothing if the medium is
 * already in another registry to avoid having hard disks in more than
 * one registry, which causes trouble with keeping diff images in sync.
 * See getFirstRegistryMachineId() for details.
 *
 * If fRecurse == true, then the media tree lock must be held for reading.
 *
 * @param id
 * @param fRecurse If true, recurses into child media to make sure the whole tree has registries in sync.
 * @return true if the registry was added; false if the given id was already on the list.
 */
bool Medium::addRegistry(const Guid& id, bool fRecurse)
{
    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc()))
        return false;
    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);

    bool fAdd = true;

    // hard disks cannot be in more than one registry
    if (   m->devType == DeviceType_HardDisk
        && m->llRegistryIDs.size() > 0)
        fAdd = false;

    // no need to add the UUID twice
    if (fAdd)
    {
        for (GuidList::const_iterator it = m->llRegistryIDs.begin();
             it != m->llRegistryIDs.end();
             ++it)
        {
            if ((*it) == id)
            {
                fAdd = false;
                break;
            }
        }
    }

    if (fAdd)
        m->llRegistryIDs.push_back(id);

    if (fRecurse)
    {
        // Get private list of children and release medium lock straight away.
        MediaList llChildren(m->llChildren);
        alock.release();

        for (MediaList::iterator it = llChildren.begin();
             it != llChildren.end();
             ++it)
        {
            Medium *pChild = *it;
            fAdd |= pChild->addRegistry(id, true);
        }
    }

    return fAdd;
}

/**
 * Removes the given UUID from the list of media registry UUIDs. Returns true
 * if found or false if not.
 *
 * If fRecurse == true, then the media tree lock must be held for reading.
 *
 * @param id
 * @param fRecurse If true, recurses into child media to make sure the whole tree has registries in sync.
 * @return
 */
bool Medium::removeRegistry(const Guid& id, bool fRecurse)
{
    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc()))
        return false;
    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);

    bool fRemove = false;

    for (GuidList::iterator it = m->llRegistryIDs.begin();
         it != m->llRegistryIDs.end();
         ++it)
    {
        if ((*it) == id)
        {
            m->llRegistryIDs.erase(it);
            fRemove = true;
            break;
        }
    }

    if (fRecurse)
    {
        // Get private list of children and release medium lock straight away.
        MediaList llChildren(m->llChildren);
        alock.release();

        for (MediaList::iterator it = llChildren.begin();
             it != llChildren.end();
             ++it)
        {
            Medium *pChild = *it;
            fRemove |= pChild->removeRegistry(id, true);
        }
    }

    return fRemove;
}

/**
 * Returns true if id is in the list of media registries for this medium.
 *
 * Must have caller + read locking!
 *
 * @param id
 * @return
 */
bool Medium::isInRegistry(const Guid& id)
{
    for (GuidList::const_iterator it = m->llRegistryIDs.begin();
         it != m->llRegistryIDs.end();
         ++it)
    {
        if (*it == id)
            return true;
    }

    return false;
}

/**
 * Internal method to return the medium's first registry machine (i.e. the machine in whose
 * machine XML this medium is listed).
 *
 * Every attached medium must now (4.0) reside in at least one media registry, which is identified
 * by a UUID. This is either a machine UUID if the machine is from 4.0 or newer, in which case
 * machines have their own media registries, or it is the pseudo-UUID of the VirtualBox
 * object if the machine is old and still needs the global registry in VirtualBox.xml.
 *
 * By definition, hard disks may only be in one media registry, in which all its children
 * will be stored as well. Otherwise we run into problems with having keep multiple registries
 * in sync. (This is the "cloned VM" case in which VM1 may link to the disks of VM2; in this
 * case, only VM2's registry is used for the disk in question.)
 *
 * If there is no medium registry, particularly if the medium has not been attached yet, this
 * does not modify uuid and returns false.
 *
 * ISOs and RAWs, by contrast, can be in more than one repository to make things easier for
 * the user.
 *
 * Must have caller + locking!
 *
 * @param uuid Receives first registry machine UUID, if available.
 * @return true if uuid was set.
 */
bool Medium::getFirstRegistryMachineId(Guid &uuid) const
{
    if (m->llRegistryIDs.size())
    {
        uuid = m->llRegistryIDs.front();
        return true;
    }
    return false;
}

/**
 * Marks all the registries in which this medium is registered as modified.
 */
void Medium::markRegistriesModified()
{
    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return;

    // Get local copy, as keeping the lock over VirtualBox::markRegistryModified
    // causes trouble with the lock order
    GuidList llRegistryIDs;
    {
        AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);
        llRegistryIDs = m->llRegistryIDs;
    }

    for (GuidList::const_iterator it = llRegistryIDs.begin();
         it != llRegistryIDs.end();
         ++it)
    {
        m->pVirtualBox->markRegistryModified(*it);
    }
}

/**
 * Adds the given machine and optionally the snapshot to the list of the objects
 * this medium is attached to.
 *
 * @param aMachineId    Machine ID.
 * @param aSnapshotId   Snapshot ID; when non-empty, adds a snapshot attachment.
 */
HRESULT Medium::addBackReference(const Guid &aMachineId,
                                 const Guid &aSnapshotId /*= Guid::Empty*/)
{
    AssertReturn(!aMachineId.isEmpty(), E_FAIL);

    LogFlowThisFunc(("ENTER, aMachineId: {%RTuuid}, aSnapshotId: {%RTuuid}\n", aMachineId.raw(), aSnapshotId.raw()));

    AutoCaller autoCaller(this);
    AssertComRCReturnRC(autoCaller.rc());

    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);

    switch (m->state)
    {
        case MediumState_Created:
        case MediumState_Inaccessible:
        case MediumState_LockedRead:
        case MediumState_LockedWrite:
            break;

        default:
            return setStateError();
    }

    if (m->numCreateDiffTasks > 0)
        return setError(VBOX_E_OBJECT_IN_USE,
                        tr("Cannot attach medium '%s' {%RTuuid}: %u differencing child media are being created"),
                        m->strLocationFull.c_str(),
                        m->id.raw(),
                        m->numCreateDiffTasks);

    BackRefList::iterator it = std::find_if(m->backRefs.begin(),
                                            m->backRefs.end(),
                                            BackRef::EqualsTo(aMachineId));
    if (it == m->backRefs.end())
    {
        BackRef ref(aMachineId, aSnapshotId);
        m->backRefs.push_back(ref);

        return S_OK;
    }

    // if the caller has not supplied a snapshot ID, then we're attaching
    // to a machine a medium which represents the machine's current state,
    // so set the flag
    if (aSnapshotId.isEmpty())
    {
        /* sanity: no duplicate attachments */
        if (it->fInCurState)
            return setError(VBOX_E_OBJECT_IN_USE,
                            tr("Cannot attach medium '%s' {%RTuuid}: medium is already associated with the current state of machine uuid {%RTuuid}!"),
                            m->strLocationFull.c_str(),
                            m->id.raw(),
                            aMachineId.raw());
        it->fInCurState = true;

        return S_OK;
    }

    // otherwise: a snapshot medium is being attached

    /* sanity: no duplicate attachments */
    for (GuidList::const_iterator jt = it->llSnapshotIds.begin();
         jt != it->llSnapshotIds.end();
         ++jt)
    {
        const Guid &idOldSnapshot = *jt;

        if (idOldSnapshot == aSnapshotId)
        {
#ifdef DEBUG
            dumpBackRefs();
#endif
            return setError(VBOX_E_OBJECT_IN_USE,
                            tr("Cannot attach medium '%s' {%RTuuid} from snapshot '%RTuuid': medium is already in use by this snapshot!"),
                            m->strLocationFull.c_str(),
                            m->id.raw(),
                            aSnapshotId.raw());
        }
    }

    it->llSnapshotIds.push_back(aSnapshotId);
    // Do not touch fInCurState, as the image may be attached to the current
    // state *and* a snapshot, otherwise we lose the current state association!

    LogFlowThisFuncLeave();

    return S_OK;
}

/**
 * Removes the given machine and optionally the snapshot from the list of the
 * objects this medium is attached to.
 *
 * @param aMachineId    Machine ID.
 * @param aSnapshotId   Snapshot ID; when non-empty, removes the snapshot
 *                      attachment.
 */
HRESULT Medium::removeBackReference(const Guid &aMachineId,
                                    const Guid &aSnapshotId /*= Guid::Empty*/)
{
    AssertReturn(!aMachineId.isEmpty(), E_FAIL);

    AutoCaller autoCaller(this);
    AssertComRCReturnRC(autoCaller.rc());

    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);

    BackRefList::iterator it =
        std::find_if(m->backRefs.begin(), m->backRefs.end(),
                     BackRef::EqualsTo(aMachineId));
    AssertReturn(it != m->backRefs.end(), E_FAIL);

    if (aSnapshotId.isEmpty())
    {
        /* remove the current state attachment */
        it->fInCurState = false;
    }
    else
    {
        /* remove the snapshot attachment */
        GuidList::iterator jt = std::find(it->llSnapshotIds.begin(),
                                          it->llSnapshotIds.end(),
                                          aSnapshotId);

        AssertReturn(jt != it->llSnapshotIds.end(), E_FAIL);
        it->llSnapshotIds.erase(jt);
    }

    /* if the backref becomes empty, remove it */
    if (it->fInCurState == false && it->llSnapshotIds.size() == 0)
        m->backRefs.erase(it);

    return S_OK;
}

/**
 * Internal method to return the medium's list of backrefs. Must have caller + locking!
 * @return
 */
const Guid* Medium::getFirstMachineBackrefId() const
{
    if (!m->backRefs.size())
        return NULL;

    return &m->backRefs.front().machineId;
}

/**
 * Internal method which returns a machine that either this medium or one of its children
 * is attached to. This is used for finding a replacement media registry when an existing
 * media registry is about to be deleted in VirtualBox::unregisterMachine().
 *
 * Must have caller + locking, *and* caller must hold the media tree lock!
 * @return
 */
const Guid* Medium::getAnyMachineBackref() const
{
    if (m->backRefs.size())
        return &m->backRefs.front().machineId;

    for (MediaList::iterator it = m->llChildren.begin();
         it != m->llChildren.end();
         ++it)
    {
        Medium *pChild = *it;
        // recurse for this child
        const Guid* puuid;
        if ((puuid = pChild->getAnyMachineBackref()))
            return puuid;
    }

    return NULL;
}

const Guid* Medium::getFirstMachineBackrefSnapshotId() const
{
    if (!m->backRefs.size())
        return NULL;

    const BackRef &ref = m->backRefs.front();
    if (!ref.llSnapshotIds.size())
        return NULL;

    return &ref.llSnapshotIds.front();
}

size_t Medium::getMachineBackRefCount() const
{
    return m->backRefs.size();
}

#ifdef DEBUG
/**
 * Debugging helper that gets called after VirtualBox initialization that writes all
 * machine backreferences to the debug log.
 */
void Medium::dumpBackRefs()
{
    AutoCaller autoCaller(this);
    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    LogFlowThisFunc(("Dumping backrefs for medium '%s':\n", m->strLocationFull.c_str()));

    for (BackRefList::iterator it2 = m->backRefs.begin();
         it2 != m->backRefs.end();
         ++it2)
    {
        const BackRef &ref = *it2;
        LogFlowThisFunc(("  Backref from machine {%RTuuid} (fInCurState: %d)\n", ref.machineId.raw(), ref.fInCurState));

        for (GuidList::const_iterator jt2 = it2->llSnapshotIds.begin();
             jt2 != it2->llSnapshotIds.end();
             ++jt2)
        {
            const Guid &id = *jt2;
            LogFlowThisFunc(("  Backref from snapshot {%RTuuid}\n", id.raw()));
        }
    }
}
#endif

/**
 * Checks if the given change of \a aOldPath to \a aNewPath affects the location
 * of this media and updates it if necessary to reflect the new location.
 *
 * @param aOldPath  Old path (full).
 * @param aNewPath  New path (full).
 *
 * @note Locks this object for writing.
 */
HRESULT Medium::updatePath(const Utf8Str &strOldPath, const Utf8Str &strNewPath)
{
    AssertReturn(!strOldPath.isEmpty(), E_FAIL);
    AssertReturn(!strNewPath.isEmpty(), E_FAIL);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);

    LogFlowThisFunc(("locationFull.before='%s'\n", m->strLocationFull.c_str()));

    const char *pcszMediumPath = m->strLocationFull.c_str();

    if (RTPathStartsWith(pcszMediumPath, strOldPath.c_str()))
    {
        Utf8Str newPath(strNewPath);
        newPath.append(pcszMediumPath + strOldPath.length());
        unconst(m->strLocationFull) = newPath;

        LogFlowThisFunc(("locationFull.after='%s'\n", m->strLocationFull.c_str()));
        // we changed something
        return S_OK;
    }

    // no change was necessary, signal error which the caller needs to interpret
    return VBOX_E_FILE_ERROR;
}

/**
 * Returns the base medium of the media chain this medium is part of.
 *
 * The base medium is found by walking up the parent-child relationship axis.
 * If the medium doesn't have a parent (i.e. it's a base medium), it
 * returns itself in response to this method.
 *
 * @param aLevel    Where to store the number of ancestors of this medium
 *                  (zero for the base), may be @c NULL.
 *
 * @note Locks medium tree for reading.
 */
ComObjPtr<Medium> Medium::getBase(uint32_t *aLevel /*= NULL*/)
{
    ComObjPtr<Medium> pBase;
    uint32_t level;

    AutoCaller autoCaller(this);
    AssertReturn(autoCaller.isOk(), pBase);

    /* we access mParent */
    AutoReadLock treeLock(m->pVirtualBox->getMediaTreeLockHandle() COMMA_LOCKVAL_SRC_POS);

    pBase = this;
    level = 0;

    if (m->pParent)
    {
        for (;;)
        {
            AutoCaller baseCaller(pBase);
            AssertReturn(baseCaller.isOk(), pBase);

            if (pBase->m->pParent.isNull())
                break;

            pBase = pBase->m->pParent;
            ++level;
        }
    }

    if (aLevel != NULL)
        *aLevel = level;

    return pBase;
}

/**
 * Returns @c true if this medium cannot be modified because it has
 * dependents (children) or is part of the snapshot. Related to the medium
 * type and posterity, not to the current media state.
 *
 * @note Locks this object and medium tree for reading.
 */
bool Medium::isReadOnly()
{
    AutoCaller autoCaller(this);
    AssertComRCReturn(autoCaller.rc(), false);

    /* we access children */
    AutoReadLock treeLock(m->pVirtualBox->getMediaTreeLockHandle() COMMA_LOCKVAL_SRC_POS);

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    switch (m->type)
    {
        case MediumType_Normal:
        {
            if (getChildren().size() != 0)
                return true;

            for (BackRefList::const_iterator it = m->backRefs.begin();
                 it != m->backRefs.end(); ++it)
                if (it->llSnapshotIds.size() != 0)
                    return true;

            if (m->variant & MediumVariant_VmdkStreamOptimized)
                return true;

            return false;
        }
        case MediumType_Immutable:
        case MediumType_MultiAttach:
            return true;
        case MediumType_Writethrough:
        case MediumType_Shareable:
        case MediumType_Readonly: /* explicit readonly media has no diffs */
            return false;
        default:
            break;
    }

    AssertFailedReturn(false);
}

/**
 * Internal method to return the medium's size. Must have caller + locking!
 * @return
 */
void Medium::updateId(const Guid &id)
{
    unconst(m->id) = id;
}

/**
 * Saves medium data by appending a new child node to the given
 * parent XML settings node.
 *
 * @param data      Settings struct to be updated.
 * @param strHardDiskFolder Folder for which paths should be relative.
 *
 * @note Locks this object, medium tree and children for reading.
 */
HRESULT Medium::saveSettings(settings::Medium &data,
                             const Utf8Str &strHardDiskFolder)
{
    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    /* we access mParent */
    AutoReadLock treeLock(m->pVirtualBox->getMediaTreeLockHandle() COMMA_LOCKVAL_SRC_POS);

    AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

    data.uuid = m->id;

    // make path relative if needed
    if (    !strHardDiskFolder.isEmpty()
         && RTPathStartsWith(m->strLocationFull.c_str(), strHardDiskFolder.c_str())
       )
        data.strLocation = m->strLocationFull.substr(strHardDiskFolder.length() + 1);
    else
        data.strLocation = m->strLocationFull;
    data.strFormat = m->strFormat;

    /* optional, only for diffs, default is false */
    if (m->pParent)
        data.fAutoReset = m->autoReset;
    else
        data.fAutoReset = false;

    /* optional */
    data.strDescription = m->strDescription;

    /* optional properties */
    data.properties.clear();

    /* handle iSCSI initiator secrets transparently */
    bool fHaveInitiatorSecretEncrypted = false;
    Utf8Str strCiphertext;
    settings::StringsMap::const_iterator itPln = m->mapProperties.find("InitiatorSecret");
    if (   itPln != m->mapProperties.end()
        && !itPln->second.isEmpty())
    {
        /* Encrypt the plain secret. If that does not work (i.e. no or wrong settings key
         * specified), just use the encrypted secret (if there is any). */
        int rc = m->pVirtualBox->encryptSetting(itPln->second, &strCiphertext);
        if (RT_SUCCESS(rc))
            fHaveInitiatorSecretEncrypted = true;
    }
    for (settings::StringsMap::const_iterator it = m->mapProperties.begin();
         it != m->mapProperties.end();
         ++it)
    {
        /* only save properties that have non-default values */
        if (!it->second.isEmpty())
        {
            const Utf8Str &name = it->first;
            const Utf8Str &value = it->second;
            /* do NOT store the plain InitiatorSecret */
            if (   !fHaveInitiatorSecretEncrypted
                || !name.equals("InitiatorSecret"))
                data.properties[name] = value;
        }
    }
    if (fHaveInitiatorSecretEncrypted)
        data.properties["InitiatorSecretEncrypted"] = strCiphertext;

    /* only for base media */
    if (m->pParent.isNull())
        data.hdType = m->type;

    /* save all children */
    for (MediaList::const_iterator it = getChildren().begin();
         it != getChildren().end();
         ++it)
    {
        settings::Medium med;
        HRESULT rc = (*it)->saveSettings(med, strHardDiskFolder);
        AssertComRCReturnRC(rc);
        data.llChildren.push_back(med);
    }

    return S_OK;
}

/**
 * Constructs a medium lock list for this medium. The lock is not taken.
 *
 * @note Caller MUST NOT hold the media tree or medium lock.
 *
 * @param fFailIfInaccessible If true, this fails with an error if a medium is inaccessible. If false,
 *          inaccessible media are silently skipped and not locked (i.e. their state remains "Inaccessible");
 *          this is necessary for a VM's removable media VM startup for which we do not want to fail.
 * @param fMediumLockWrite  Whether to associate a write lock with this medium.
 * @param pToBeParent       Medium which will become the parent of this medium.
 * @param mediumLockList    Where to store the resulting list.
 */
HRESULT Medium::createMediumLockList(bool fFailIfInaccessible,
                                     bool fMediumLockWrite,
                                     Medium *pToBeParent,
                                     MediumLockList &mediumLockList)
{
    Assert(!m->pVirtualBox->getMediaTreeLockHandle().isWriteLockOnCurrentThread());
    Assert(!isWriteLockOnCurrentThread());

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    HRESULT rc = S_OK;

    /* paranoid sanity checking if the medium has a to-be parent medium */
    if (pToBeParent)
    {
        AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);
        ComAssertRet(getParent().isNull(), E_FAIL);
        ComAssertRet(getChildren().size() == 0, E_FAIL);
    }

    ErrorInfoKeeper eik;
    MultiResult mrc(S_OK);

    ComObjPtr<Medium> pMedium = this;
    while (!pMedium.isNull())
    {
        AutoReadLock alock(pMedium COMMA_LOCKVAL_SRC_POS);

        /* Accessibility check must be first, otherwise locking interferes
         * with getting the medium state. Lock lists are not created for
         * fun, and thus getting the medium status is no luxury. */
        MediumState_T mediumState = pMedium->getState();
        if (mediumState == MediumState_Inaccessible)
        {
            alock.release();
            rc = pMedium->queryInfo(false /* fSetImageId */, false /* fSetParentId */);
            alock.acquire();
            if (FAILED(rc)) return rc;

            mediumState = pMedium->getState();
            if (mediumState == MediumState_Inaccessible)
            {
                // ignore inaccessible ISO media and silently return S_OK,
                // otherwise VM startup (esp. restore) may fail without good reason
                if (!fFailIfInaccessible)
                    return S_OK;

                // otherwise report an error
                Bstr error;
                rc = pMedium->COMGETTER(LastAccessError)(error.asOutParam());
                if (FAILED(rc)) return rc;

                /* collect multiple errors */
                eik.restore();
                Assert(!error.isEmpty());
                mrc = setError(E_FAIL,
                               "%ls",
                               error.raw());
                    // error message will be something like
                    // "Could not open the medium ... VD: error VERR_FILE_NOT_FOUND opening image file ... (VERR_FILE_NOT_FOUND).
                eik.fetch();
            }
        }

        if (pMedium == this)
            mediumLockList.Prepend(pMedium, fMediumLockWrite);
        else
            mediumLockList.Prepend(pMedium, false);

        pMedium = pMedium->getParent();
        if (pMedium.isNull() && pToBeParent)
        {
            pMedium = pToBeParent;
            pToBeParent = NULL;
        }
    }

    return mrc;
}

/**
 * Creates a new differencing storage unit using the format of the given target
 * medium and the location. Note that @c aTarget must be NotCreated.
 *
 * The @a aMediumLockList parameter contains the associated medium lock list,
 * which must be in locked state. If @a aWait is @c true then the caller is
 * responsible for unlocking.
 *
 * If @a aProgress is not NULL but the object it points to is @c null then a
 * new progress object will be created and assigned to @a *aProgress on
 * success, otherwise the existing progress object is used. If @a aProgress is
 * NULL, then no progress object is created/used at all.
 *
 * When @a aWait is @c false, this method will create a thread to perform the
 * create operation asynchronously and will return immediately. Otherwise, it
 * will perform the operation on the calling thread and will not return to the
 * caller until the operation is completed. Note that @a aProgress cannot be
 * NULL when @a aWait is @c false (this method will assert in this case).
 *
 * @param aTarget           Target medium.
 * @param aVariant          Precise medium variant to create.
 * @param aMediumLockList   List of media which should be locked.
 * @param aProgress         Where to find/store a Progress object to track
 *                          operation completion.
 * @param aWait             @c true if this method should block instead of
 *                          creating an asynchronous thread.
 *
 * @note Locks this object and @a aTarget for writing.
 */
HRESULT Medium::createDiffStorage(ComObjPtr<Medium> &aTarget,
                                  MediumVariant_T aVariant,
                                  MediumLockList *aMediumLockList,
                                  ComObjPtr<Progress> *aProgress,
                                  bool aWait)
{
    AssertReturn(!aTarget.isNull(), E_FAIL);
    AssertReturn(aMediumLockList, E_FAIL);
    AssertReturn(aProgress != NULL || aWait == true, E_FAIL);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoCaller targetCaller(aTarget);
    if (FAILED(targetCaller.rc())) return targetCaller.rc();

    HRESULT rc = S_OK;
    ComObjPtr<Progress> pProgress;
    Medium::Task *pTask = NULL;

    try
    {
        AutoMultiWriteLock2 alock(this, aTarget COMMA_LOCKVAL_SRC_POS);

        ComAssertThrow(   m->type != MediumType_Writethrough
                       && m->type != MediumType_Shareable
                       && m->type != MediumType_Readonly, E_FAIL);
        ComAssertThrow(m->state == MediumState_LockedRead, E_FAIL);

        if (aTarget->m->state != MediumState_NotCreated)
            throw aTarget->setStateError();

        /* Check that the medium is not attached to the current state of
         * any VM referring to it. */
        for (BackRefList::const_iterator it = m->backRefs.begin();
             it != m->backRefs.end();
             ++it)
        {
            if (it->fInCurState)
            {
                /* Note: when a VM snapshot is being taken, all normal media
                 * attached to the VM in the current state will be, as an
                 * exception, also associated with the snapshot which is about
                 * to create (see SnapshotMachine::init()) before deassociating
                 * them from the current state (which takes place only on
                 * success in Machine::fixupHardDisks()), so that the size of
                 * snapshotIds will be 1 in this case. The extra condition is
                 * used to filter out this legal situation. */
                if (it->llSnapshotIds.size() == 0)
                    throw setError(VBOX_E_INVALID_OBJECT_STATE,
                                   tr("Medium '%s' is attached to a virtual machine with UUID {%RTuuid}. No differencing media based on it may be created until it is detached"),
                                   m->strLocationFull.c_str(), it->machineId.raw());

                Assert(it->llSnapshotIds.size() == 1);
            }
        }

        if (aProgress != NULL)
        {
            /* use the existing progress object... */
            pProgress = *aProgress;

            /* ...but create a new one if it is null */
            if (pProgress.isNull())
            {
                pProgress.createObject();
                rc = pProgress->init(m->pVirtualBox,
                                     static_cast<IMedium*>(this),
                                     BstrFmt(tr("Creating differencing medium storage unit '%s'"), aTarget->m->strLocationFull.c_str()).raw(),
                                     TRUE /* aCancelable */);
                if (FAILED(rc))
                    throw rc;
            }
        }

        /* setup task object to carry out the operation sync/async */
        pTask = new Medium::CreateDiffTask(this, pProgress, aTarget, aVariant,
                                           aMediumLockList,
                                           aWait /* fKeepMediumLockList */);
        rc = pTask->rc();
        AssertComRC(rc);
        if (FAILED(rc))
             throw rc;

        /* register a task (it will deregister itself when done) */
        ++m->numCreateDiffTasks;
        Assert(m->numCreateDiffTasks != 0); /* overflow? */

        aTarget->m->state = MediumState_Creating;
    }
    catch (HRESULT aRC) { rc = aRC; }

    if (SUCCEEDED(rc))
    {
        if (aWait)
            rc = runNow(pTask);
        else
            rc = startThread(pTask);

        if (SUCCEEDED(rc) && aProgress != NULL)
            *aProgress = pProgress;
    }
    else if (pTask != NULL)
        delete pTask;

    return rc;
}

/**
 * Returns a preferred format for differencing media.
 */
Utf8Str Medium::getPreferredDiffFormat()
{
    AutoCaller autoCaller(this);
    AssertComRCReturn(autoCaller.rc(), Utf8Str::Empty);

    /* check that our own format supports diffs */
    if (!(m->formatObj->getCapabilities() & MediumFormatCapabilities_Differencing))
    {
        /* use the default format if not */
        Utf8Str tmp;
        m->pVirtualBox->getDefaultHardDiskFormat(tmp);
        return tmp;
    }

    /* m->strFormat is const, no need to lock */
    return m->strFormat;
}

/**
 * Implementation for the public Medium::Close() with the exception of calling
 * VirtualBox::saveRegistries(), in case someone wants to call this for several
 * media.
 *
 * After this returns with success, uninit() has been called on the medium, and
 * the object is no longer usable ("not ready" state).
 *
 * @param autoCaller AutoCaller instance which must have been created on the caller's stack for this medium. This gets released here
 *                   upon which the Medium instance gets uninitialized.
 * @return
 */
HRESULT Medium::close(AutoCaller &autoCaller)
{
    // we're accessing parent/child and backrefs, so lock the tree first, then ourselves
    AutoMultiWriteLock2 multilock(&m->pVirtualBox->getMediaTreeLockHandle(),
                                  this->lockHandle()
                                  COMMA_LOCKVAL_SRC_POS);

    LogFlowFunc(("ENTER for %s\n", getLocationFull().c_str()));

    bool wasCreated = true;

    switch (m->state)
    {
        case MediumState_NotCreated:
            wasCreated = false;
            break;
        case MediumState_Created:
        case MediumState_Inaccessible:
            break;
        default:
            return setStateError();
    }

    if (m->backRefs.size() != 0)
        return setError(VBOX_E_OBJECT_IN_USE,
                        tr("Medium '%s' cannot be closed because it is still attached to %d virtual machines"),
                        m->strLocationFull.c_str(), m->backRefs.size());

    // perform extra media-dependent close checks
    HRESULT rc = canClose();
    if (FAILED(rc)) return rc;

    if (wasCreated)
    {
        // remove from the list of known media before performing actual
        // uninitialization (to keep the media registry consistent on
        // failure to do so)
        rc = unregisterWithVirtualBox();
        if (FAILED(rc)) return rc;

        multilock.release();
        markRegistriesModified();
        // Release the AutoCalleri now, as otherwise uninit() will simply hang.
        // Needs to be done before saving the registry, as otherwise there
        // may be a deadlock with someone else closing this object while we're
        // in saveModifiedRegistries(), which needs the media tree lock, which
        // the other thread holds until after uninit() below.
        /// @todo redesign the locking here, as holding the locks over uninit causes lock order trouble which the lock validator can't detect
        autoCaller.release();
        m->pVirtualBox->saveModifiedRegistries();
        multilock.acquire();
    }
    else
    {
        // release the AutoCaller, as otherwise uninit() will simply hang
        autoCaller.release();
    }

    // Keep the locks held until after uninit, as otherwise the consistency
    // of the medium tree cannot be guaranteed.
    uninit();

    LogFlowFuncLeave();

    return rc;
}

/**
 * Deletes the medium storage unit.
 *
 * If @a aProgress is not NULL but the object it points to is @c null then a new
 * progress object will be created and assigned to @a *aProgress on success,
 * otherwise the existing progress object is used. If Progress is NULL, then no
 * progress object is created/used at all.
 *
 * When @a aWait is @c false, this method will create a thread to perform the
 * delete operation asynchronously and will return immediately. Otherwise, it
 * will perform the operation on the calling thread and will not return to the
 * caller until the operation is completed. Note that @a aProgress cannot be
 * NULL when @a aWait is @c false (this method will assert in this case).
 *
 * @param aProgress     Where to find/store a Progress object to track operation
 *                      completion.
 * @param aWait         @c true if this method should block instead of creating
 *                      an asynchronous thread.
 *
 * @note Locks mVirtualBox and this object for writing. Locks medium tree for
 *       writing.
 */
HRESULT Medium::deleteStorage(ComObjPtr<Progress> *aProgress,
                              bool aWait)
{
    AssertReturn(aProgress != NULL || aWait == true, E_FAIL);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    HRESULT rc = S_OK;
    ComObjPtr<Progress> pProgress;
    Medium::Task *pTask = NULL;

    try
    {
        /* we're accessing the media tree, and canClose() needs it too */
        AutoMultiWriteLock2 multilock(&m->pVirtualBox->getMediaTreeLockHandle(),
                                      this->lockHandle()
                                      COMMA_LOCKVAL_SRC_POS);
        LogFlowThisFunc(("aWait=%RTbool locationFull=%s\n", aWait, getLocationFull().c_str() ));

        if (    !(m->formatObj->getCapabilities() & (   MediumFormatCapabilities_CreateDynamic
                                                      | MediumFormatCapabilities_CreateFixed)))
            throw setError(VBOX_E_NOT_SUPPORTED,
                           tr("Medium format '%s' does not support storage deletion"),
                           m->strFormat.c_str());

        /* Note that we are fine with Inaccessible state too: a) for symmetry
         * with create calls and b) because it doesn't really harm to try, if
         * it is really inaccessible, the delete operation will fail anyway.
         * Accepting Inaccessible state is especially important because all
         * registered media are initially Inaccessible upon VBoxSVC startup
         * until COMGETTER(RefreshState) is called. Accept Deleting state
         * because some callers need to put the medium in this state early
         * to prevent races. */
        switch (m->state)
        {
            case MediumState_Created:
            case MediumState_Deleting:
            case MediumState_Inaccessible:
                break;
            default:
                throw setStateError();
        }

        if (m->backRefs.size() != 0)
        {
            Utf8Str strMachines;
            for (BackRefList::const_iterator it = m->backRefs.begin();
                it != m->backRefs.end();
                ++it)
            {
                const BackRef &b = *it;
                if (strMachines.length())
                    strMachines.append(", ");
                strMachines.append(b.machineId.toString().c_str());
            }
#ifdef DEBUG
            dumpBackRefs();
#endif
            throw setError(VBOX_E_OBJECT_IN_USE,
                           tr("Cannot delete storage: medium '%s' is still attached to the following %d virtual machine(s): %s"),
                           m->strLocationFull.c_str(),
                           m->backRefs.size(),
                           strMachines.c_str());
        }

        rc = canClose();
        if (FAILED(rc))
            throw rc;

        /* go to Deleting state, so that the medium is not actually locked */
        if (m->state != MediumState_Deleting)
        {
            rc = markForDeletion();
            if (FAILED(rc))
                throw rc;
        }

        /* Build the medium lock list. */
        MediumLockList *pMediumLockList(new MediumLockList());
        multilock.release();
        rc = createMediumLockList(true /* fFailIfInaccessible */,
                                  true /* fMediumLockWrite */,
                                  NULL,
                                  *pMediumLockList);
        multilock.acquire();
        if (FAILED(rc))
        {
            delete pMediumLockList;
            throw rc;
        }

        multilock.release();
        rc = pMediumLockList->Lock();
        multilock.acquire();
        if (FAILED(rc))
        {
            delete pMediumLockList;
            throw setError(rc,
                           tr("Failed to lock media when deleting '%s'"),
                           getLocationFull().c_str());
        }

        /* try to remove from the list of known media before performing
         * actual deletion (we favor the consistency of the media registry
         * which would have been broken if unregisterWithVirtualBox() failed
         * after we successfully deleted the storage) */
        rc = unregisterWithVirtualBox();
        if (FAILED(rc))
            throw rc;
        // no longer need lock
        multilock.release();
        markRegistriesModified();

        if (aProgress != NULL)
        {
            /* use the existing progress object... */
            pProgress = *aProgress;

            /* ...but create a new one if it is null */
            if (pProgress.isNull())
            {
                pProgress.createObject();
                rc = pProgress->init(m->pVirtualBox,
                                     static_cast<IMedium*>(this),
                                     BstrFmt(tr("Deleting medium storage unit '%s'"), m->strLocationFull.c_str()).raw(),
                                     FALSE /* aCancelable */);
                if (FAILED(rc))
                    throw rc;
            }
        }

        /* setup task object to carry out the operation sync/async */
        pTask = new Medium::DeleteTask(this, pProgress, pMediumLockList);
        rc = pTask->rc();
        AssertComRC(rc);
        if (FAILED(rc))
            throw rc;
    }
    catch (HRESULT aRC) { rc = aRC; }

    if (SUCCEEDED(rc))
    {
        if (aWait)
            rc = runNow(pTask);
        else
            rc = startThread(pTask);

        if (SUCCEEDED(rc) && aProgress != NULL)
            *aProgress = pProgress;

    }
    else
    {
        if (pTask)
            delete pTask;

        /* Undo deleting state if necessary. */
        AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);
        /* Make sure that any error signalled by unmarkForDeletion() is not
         * ending up in the error list (if the caller uses MultiResult). It
         * usually is spurious, as in most cases the medium hasn't been marked
         * for deletion when the error was thrown above. */
        ErrorInfoKeeper eik;
        unmarkForDeletion();
    }

    return rc;
}

/**
 * Mark a medium for deletion.
 *
 * @note Caller must hold the write lock on this medium!
 */
HRESULT Medium::markForDeletion()
{
    ComAssertRet(isWriteLockOnCurrentThread(), E_FAIL);
    switch (m->state)
    {
        case MediumState_Created:
        case MediumState_Inaccessible:
            m->preLockState = m->state;
            m->state = MediumState_Deleting;
            return S_OK;
        default:
            return setStateError();
    }
}

/**
 * Removes the "mark for deletion".
 *
 * @note Caller must hold the write lock on this medium!
 */
HRESULT Medium::unmarkForDeletion()
{
    ComAssertRet(isWriteLockOnCurrentThread(), E_FAIL);
    switch (m->state)
    {
        case MediumState_Deleting:
            m->state = m->preLockState;
            return S_OK;
        default:
            return setStateError();
    }
}

/**
 * Mark a medium for deletion which is in locked state.
 *
 * @note Caller must hold the write lock on this medium!
 */
HRESULT Medium::markLockedForDeletion()
{
    ComAssertRet(isWriteLockOnCurrentThread(), E_FAIL);
    if (   (   m->state == MediumState_LockedRead
            || m->state == MediumState_LockedWrite)
        && m->preLockState == MediumState_Created)
    {
        m->preLockState = MediumState_Deleting;
        return S_OK;
    }
    else
        return setStateError();
}

/**
 * Removes the "mark for deletion" for a medium in locked state.
 *
 * @note Caller must hold the write lock on this medium!
 */
HRESULT Medium::unmarkLockedForDeletion()
{
    ComAssertRet(isWriteLockOnCurrentThread(), E_FAIL);
    if (   (   m->state == MediumState_LockedRead
            || m->state == MediumState_LockedWrite)
        && m->preLockState == MediumState_Deleting)
    {
        m->preLockState = MediumState_Created;
        return S_OK;
    }
    else
        return setStateError();
}

/**
 * Queries the preferred merge direction from this to the other medium, i.e.
 * the one which requires the least amount of I/O and therefore time and
 * disk consumption.
 *
 * @returns Status code.
 * @retval  E_FAIL in case determining the merge direction fails for some reason,
 *          for example if getting the size of the media fails. There is no
 *          error set though and the caller is free to continue to find out
 *          what was going wrong later. Leaves fMergeForward unset.
 * @retval  VBOX_E_INVALID_OBJECT_STATE if both media are not related to each other
 *          An error is set.
 * @param pOther           The other medium to merge with.
 * @param fMergeForward    Resulting preferred merge direction (out).
 */
HRESULT Medium::queryPreferredMergeDirection(const ComObjPtr<Medium> &pOther,
                                             bool &fMergeForward)
{
    AssertReturn(pOther != NULL, E_FAIL);
    AssertReturn(pOther != this, E_FAIL);

    AutoCaller autoCaller(this);
    AssertComRCReturnRC(autoCaller.rc());

    AutoCaller otherCaller(pOther);
    AssertComRCReturnRC(otherCaller.rc());

    HRESULT rc = S_OK;
    bool fThisParent = false; /**<< Flag whether this medium is the parent of pOther. */

    try
    {
        // locking: we need the tree lock first because we access parent pointers
        AutoWriteLock treeLock(m->pVirtualBox->getMediaTreeLockHandle() COMMA_LOCKVAL_SRC_POS);

        /* more sanity checking and figuring out the current merge direction */
        ComObjPtr<Medium> pMedium = getParent();
        while (!pMedium.isNull() && pMedium != pOther)
            pMedium = pMedium->getParent();
        if (pMedium == pOther)
            fThisParent = false;
        else
        {
            pMedium = pOther->getParent();
            while (!pMedium.isNull() && pMedium != this)
                pMedium = pMedium->getParent();
            if (pMedium == this)
                fThisParent = true;
            else
            {
                Utf8Str tgtLoc;
                {
                    AutoReadLock alock(pOther COMMA_LOCKVAL_SRC_POS);
                    tgtLoc = pOther->getLocationFull();
                }

                AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);
                throw setError(VBOX_E_INVALID_OBJECT_STATE,
                               tr("Media '%s' and '%s' are unrelated"),
                               m->strLocationFull.c_str(), tgtLoc.c_str());
            }
        }

        /*
         * Figure out the preferred merge direction. The current way is to
         * get the current sizes of file based images and select the merge
         * direction depending on the size.
         *
         * Can't use the VD API to get current size here as the media might
         * be write locked by a running VM. Resort to RTFileQuerySize().
         */
        int vrc = VINF_SUCCESS;
        uint64_t cbMediumThis = 0;
        uint64_t cbMediumOther = 0;

        if (isMediumFormatFile() && pOther->isMediumFormatFile())
        {
            vrc = RTFileQuerySize(this->getLocationFull().c_str(), &cbMediumThis);
            if (RT_SUCCESS(vrc))
            {
                vrc = RTFileQuerySize(pOther->getLocationFull().c_str(),
                                      &cbMediumOther);
            }

            if (RT_FAILURE(vrc))
                rc = E_FAIL;
            else
            {
                /*
                 * Check which merge direction might be more optimal.
                 * This method is not bullet proof of course as there might
                 * be overlapping blocks in the images so the file size is
                 * not the best indicator but it is good enough for our purpose
                 * and everything else is too complicated, especially when the
                 * media are used by a running VM.
                 */
                bool fMergeIntoThis = cbMediumThis > cbMediumOther;
                fMergeForward = fMergeIntoThis ^ fThisParent;
            }
        }
    }
    catch (HRESULT aRC) { rc = aRC; }

    return rc;
}

/**
 * Prepares this (source) medium, target medium and all intermediate media
 * for the merge operation.
 *
 * This method is to be called prior to calling the #mergeTo() to perform
 * necessary consistency checks and place involved media to appropriate
 * states. If #mergeTo() is not called or fails, the state modifications
 * performed by this method must be undone by #cancelMergeTo().
 *
 * See #mergeTo() for more information about merging.
 *
 * @param pTarget       Target medium.
 * @param aMachineId    Allowed machine attachment. NULL means do not check.
 * @param aSnapshotId   Allowed snapshot attachment. NULL or empty UUID means
 *                      do not check.
 * @param fLockMedia    Flag whether to lock the medium lock list or not.
 *                      If set to false and the medium lock list locking fails
 *                      later you must call #cancelMergeTo().
 * @param fMergeForward Resulting merge direction (out).
 * @param pParentForTarget New parent for target medium after merge (out).
 * @param aChildrenToReparent List of children of the source which will have
 *                      to be reparented to the target after merge (out).
 * @param aMediumLockList Medium locking information (out).
 *
 * @note Locks medium tree for reading. Locks this object, aTarget and all
 *       intermediate media for writing.
 */
HRESULT Medium::prepareMergeTo(const ComObjPtr<Medium> &pTarget,
                               const Guid *aMachineId,
                               const Guid *aSnapshotId,
                               bool fLockMedia,
                               bool &fMergeForward,
                               ComObjPtr<Medium> &pParentForTarget,
                               MediaList &aChildrenToReparent,
                               MediumLockList * &aMediumLockList)
{
    AssertReturn(pTarget != NULL, E_FAIL);
    AssertReturn(pTarget != this, E_FAIL);

    AutoCaller autoCaller(this);
    AssertComRCReturnRC(autoCaller.rc());

    AutoCaller targetCaller(pTarget);
    AssertComRCReturnRC(targetCaller.rc());

    HRESULT rc = S_OK;
    fMergeForward = false;
    pParentForTarget.setNull();
    aChildrenToReparent.clear();
    Assert(aMediumLockList == NULL);
    aMediumLockList = NULL;

    try
    {
        // locking: we need the tree lock first because we access parent pointers
        AutoWriteLock treeLock(m->pVirtualBox->getMediaTreeLockHandle() COMMA_LOCKVAL_SRC_POS);

        /* more sanity checking and figuring out the merge direction */
        ComObjPtr<Medium> pMedium = getParent();
        while (!pMedium.isNull() && pMedium != pTarget)
            pMedium = pMedium->getParent();
        if (pMedium == pTarget)
            fMergeForward = false;
        else
        {
            pMedium = pTarget->getParent();
            while (!pMedium.isNull() && pMedium != this)
                pMedium = pMedium->getParent();
            if (pMedium == this)
                fMergeForward = true;
            else
            {
                Utf8Str tgtLoc;
                {
                    AutoReadLock alock(pTarget COMMA_LOCKVAL_SRC_POS);
                    tgtLoc = pTarget->getLocationFull();
                }

                AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);
                throw setError(VBOX_E_INVALID_OBJECT_STATE,
                               tr("Media '%s' and '%s' are unrelated"),
                               m->strLocationFull.c_str(), tgtLoc.c_str());
            }
        }

        /* Build the lock list. */
        aMediumLockList = new MediumLockList();
        treeLock.release();
        if (fMergeForward)
            rc = pTarget->createMediumLockList(true /* fFailIfInaccessible */,
                                               true /* fMediumLockWrite */,
                                               NULL,
                                               *aMediumLockList);
        else
            rc = createMediumLockList(true /* fFailIfInaccessible */,
                                      false /* fMediumLockWrite */,
                                      NULL,
                                      *aMediumLockList);
        treeLock.acquire();
        if (FAILED(rc))
            throw rc;

        /* Sanity checking, must be after lock list creation as it depends on
         * valid medium states. The medium objects must be accessible. Only
         * do this if immediate locking is requested, otherwise it fails when
         * we construct a medium lock list for an already running VM. Snapshot
         * deletion uses this to simplify its life. */
        if (fLockMedia)
        {
            {
                AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);
                if (m->state != MediumState_Created)
                    throw setStateError();
            }
            {
                AutoReadLock alock(pTarget COMMA_LOCKVAL_SRC_POS);
                if (pTarget->m->state != MediumState_Created)
                    throw pTarget->setStateError();
            }
        }

        /* check medium attachment and other sanity conditions */
        if (fMergeForward)
        {
            AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);
            if (getChildren().size() > 1)
            {
                throw setError(VBOX_E_INVALID_OBJECT_STATE,
                               tr("Medium '%s' involved in the merge operation has more than one child medium (%d)"),
                               m->strLocationFull.c_str(), getChildren().size());
            }
            /* One backreference is only allowed if the machine ID is not empty
             * and it matches the machine the medium is attached to (including
             * the snapshot ID if not empty). */
            if (   m->backRefs.size() != 0
                && (   !aMachineId
                    || m->backRefs.size() != 1
                    || aMachineId->isEmpty()
                    || *getFirstMachineBackrefId() != *aMachineId
                    || (   (!aSnapshotId || !aSnapshotId->isEmpty())
                        && *getFirstMachineBackrefSnapshotId() != *aSnapshotId)))
                throw setError(VBOX_E_OBJECT_IN_USE,
                               tr("Medium '%s' is attached to %d virtual machines"),
                               m->strLocationFull.c_str(), m->backRefs.size());
            if (m->type == MediumType_Immutable)
                throw setError(VBOX_E_INVALID_OBJECT_STATE,
                               tr("Medium '%s' is immutable"),
                               m->strLocationFull.c_str());
            if (m->type == MediumType_MultiAttach)
                throw setError(VBOX_E_INVALID_OBJECT_STATE,
                               tr("Medium '%s' is multi-attach"),
                               m->strLocationFull.c_str());
        }
        else
        {
            AutoReadLock alock(pTarget COMMA_LOCKVAL_SRC_POS);
            if (pTarget->getChildren().size() > 1)
            {
                throw setError(VBOX_E_OBJECT_IN_USE,
                               tr("Medium '%s' involved in the merge operation has more than one child medium (%d)"),
                               pTarget->m->strLocationFull.c_str(),
                               pTarget->getChildren().size());
            }
            if (pTarget->m->type == MediumType_Immutable)
                throw setError(VBOX_E_INVALID_OBJECT_STATE,
                               tr("Medium '%s' is immutable"),
                               pTarget->m->strLocationFull.c_str());
            if (pTarget->m->type == MediumType_MultiAttach)
                throw setError(VBOX_E_INVALID_OBJECT_STATE,
                               tr("Medium '%s' is multi-attach"),
                               pTarget->m->strLocationFull.c_str());
        }
        ComObjPtr<Medium> pLast(fMergeForward ? (Medium *)pTarget : this);
        ComObjPtr<Medium> pLastIntermediate = pLast->getParent();
        for (pLast = pLastIntermediate;
             !pLast.isNull() && pLast != pTarget && pLast != this;
             pLast = pLast->getParent())
        {
            AutoReadLock alock(pLast COMMA_LOCKVAL_SRC_POS);
            if (pLast->getChildren().size() > 1)
            {
                throw setError(VBOX_E_OBJECT_IN_USE,
                               tr("Medium '%s' involved in the merge operation has more than one child medium (%d)"),
                               pLast->m->strLocationFull.c_str(),
                               pLast->getChildren().size());
            }
            if (pLast->m->backRefs.size() != 0)
                throw setError(VBOX_E_OBJECT_IN_USE,
                               tr("Medium '%s' is attached to %d virtual machines"),
                               pLast->m->strLocationFull.c_str(),
                               pLast->m->backRefs.size());

        }

        /* Update medium states appropriately */
        {
            AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);

            if (m->state == MediumState_Created)
            {
                rc = markForDeletion();
                if (FAILED(rc))
                    throw rc;
            }
            else
            {
                if (fLockMedia)
                    throw setStateError();
                else if (   m->state == MediumState_LockedWrite
                         || m->state == MediumState_LockedRead)
                {
                    /* Either mark it for deletion in locked state or allow
                     * others to have done so. */
                    if (m->preLockState == MediumState_Created)
                        markLockedForDeletion();
                    else if (m->preLockState != MediumState_Deleting)
                        throw setStateError();
                }
                else
                    throw setStateError();
            }
        }

        if (fMergeForward)
        {
            /* we will need parent to reparent target */
            pParentForTarget = getParent();
        }
        else
        {
            /* we will need to reparent children of the source */
            for (MediaList::const_iterator it = getChildren().begin();
                 it != getChildren().end();
                 ++it)
            {
                pMedium = *it;
                if (fLockMedia)
                {
                    rc = pMedium->LockWrite(NULL);
                    if (FAILED(rc))
                        throw rc;
                }

                aChildrenToReparent.push_back(pMedium);
            }
        }
        for (pLast = pLastIntermediate;
             !pLast.isNull() && pLast != pTarget && pLast != this;
             pLast = pLast->getParent())
        {
            AutoWriteLock alock(pLast COMMA_LOCKVAL_SRC_POS);
            if (pLast->m->state == MediumState_Created)
            {
                rc = pLast->markForDeletion();
                if (FAILED(rc))
                    throw rc;
            }
            else
                throw pLast->setStateError();
        }

        /* Tweak the lock list in the backward merge case, as the target
         * isn't marked to be locked for writing yet. */
        if (!fMergeForward)
        {
            MediumLockList::Base::iterator lockListBegin =
                aMediumLockList->GetBegin();
            MediumLockList::Base::iterator lockListEnd =
                aMediumLockList->GetEnd();
            lockListEnd--;
            for (MediumLockList::Base::iterator it = lockListBegin;
                 it != lockListEnd;
                 ++it)
            {
                MediumLock &mediumLock = *it;
                if (mediumLock.GetMedium() == pTarget)
                {
                    HRESULT rc2 = mediumLock.UpdateLock(true);
                    AssertComRC(rc2);
                    break;
                }
            }
        }

        if (fLockMedia)
        {
            treeLock.release();
            rc = aMediumLockList->Lock();
            treeLock.acquire();
            if (FAILED(rc))
            {
                AutoReadLock alock(pTarget COMMA_LOCKVAL_SRC_POS);
                throw setError(rc,
                               tr("Failed to lock media when merging to '%s'"),
                               pTarget->getLocationFull().c_str());
            }
        }
    }
    catch (HRESULT aRC) { rc = aRC; }

    if (FAILED(rc))
    {
        delete aMediumLockList;
        aMediumLockList = NULL;
    }

    return rc;
}

/**
 * Merges this medium to the specified medium which must be either its
 * direct ancestor or descendant.
 *
 * Given this medium is SOURCE and the specified medium is TARGET, we will
 * get two variants of the merge operation:
 *
 *                forward merge
 *                ------------------------->
 *  [Extra] <- SOURCE <- Intermediate <- TARGET
 *  Any        Del       Del             LockWr
 *
 *
 *                            backward merge
 *                <-------------------------
 *             TARGET <- Intermediate <- SOURCE <- [Extra]
 *             LockWr    Del             Del       LockWr
 *
 * Each diagram shows the involved media on the media chain where
 * SOURCE and TARGET belong. Under each medium there is a state value which
 * the medium must have at a time of the mergeTo() call.
 *
 * The media in the square braces may be absent (e.g. when the forward
 * operation takes place and SOURCE is the base medium, or when the backward
 * merge operation takes place and TARGET is the last child in the chain) but if
 * they present they are involved too as shown.
 *
 * Neither the source medium nor intermediate media may be attached to
 * any VM directly or in the snapshot, otherwise this method will assert.
 *
 * The #prepareMergeTo() method must be called prior to this method to place all
 * involved to necessary states and perform other consistency checks.
 *
 * If @a aWait is @c true then this method will perform the operation on the
 * calling thread and will not return to the caller until the operation is
 * completed. When this method succeeds, all intermediate medium objects in
 * the chain will be uninitialized, the state of the target medium (and all
 * involved extra media) will be restored. @a aMediumLockList will not be
 * deleted, whether the operation is successful or not. The caller has to do
 * this if appropriate. Note that this (source) medium is not uninitialized
 * because of possible AutoCaller instances held by the caller of this method
 * on the current thread. It's therefore the responsibility of the caller to
 * call Medium::uninit() after releasing all callers.
 *
 * If @a aWait is @c false then this method will create a thread to perform the
 * operation asynchronously and will return immediately. If the operation
 * succeeds, the thread will uninitialize the source medium object and all
 * intermediate medium objects in the chain, reset the state of the target
 * medium (and all involved extra media) and delete @a aMediumLockList.
 * If the operation fails, the thread will only reset the states of all
 * involved media and delete @a aMediumLockList.
 *
 * When this method fails (regardless of the @a aWait mode), it is a caller's
 * responsibility to undo state changes and delete @a aMediumLockList using
 * #cancelMergeTo().
 *
 * If @a aProgress is not NULL but the object it points to is @c null then a new
 * progress object will be created and assigned to @a *aProgress on success,
 * otherwise the existing progress object is used. If Progress is NULL, then no
 * progress object is created/used at all. Note that @a aProgress cannot be
 * NULL when @a aWait is @c false (this method will assert in this case).
 *
 * @param pTarget       Target medium.
 * @param fMergeForward Merge direction.
 * @param pParentForTarget New parent for target medium after merge.
 * @param aChildrenToReparent List of children of the source which will have
 *                      to be reparented to the target after merge.
 * @param aMediumLockList Medium locking information.
 * @param aProgress     Where to find/store a Progress object to track operation
 *                      completion.
 * @param aWait         @c true if this method should block instead of creating
 *                      an asynchronous thread.
 *
 * @note Locks the tree lock for writing. Locks the media from the chain
 *       for writing.
 */
HRESULT Medium::mergeTo(const ComObjPtr<Medium> &pTarget,
                        bool fMergeForward,
                        const ComObjPtr<Medium> &pParentForTarget,
                        const MediaList &aChildrenToReparent,
                        MediumLockList *aMediumLockList,
                        ComObjPtr <Progress> *aProgress,
                        bool aWait)
{
    AssertReturn(pTarget != NULL, E_FAIL);
    AssertReturn(pTarget != this, E_FAIL);
    AssertReturn(aMediumLockList != NULL, E_FAIL);
    AssertReturn(aProgress != NULL || aWait == true, E_FAIL);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoCaller targetCaller(pTarget);
    AssertComRCReturnRC(targetCaller.rc());

    HRESULT rc = S_OK;
    ComObjPtr <Progress> pProgress;
    Medium::Task *pTask = NULL;

    try
    {
        if (aProgress != NULL)
        {
            /* use the existing progress object... */
            pProgress = *aProgress;

            /* ...but create a new one if it is null */
            if (pProgress.isNull())
            {
                Utf8Str tgtName;
                {
                    AutoReadLock alock(pTarget COMMA_LOCKVAL_SRC_POS);
                    tgtName = pTarget->getName();
                }

                AutoReadLock alock(this COMMA_LOCKVAL_SRC_POS);

                pProgress.createObject();
                rc = pProgress->init(m->pVirtualBox,
                                     static_cast<IMedium*>(this),
                                     BstrFmt(tr("Merging medium '%s' to '%s'"),
                                             getName().c_str(),
                                             tgtName.c_str()).raw(),
                                     TRUE /* aCancelable */);
                if (FAILED(rc))
                    throw rc;
            }
        }

        /* setup task object to carry out the operation sync/async */
        pTask = new Medium::MergeTask(this, pTarget, fMergeForward,
                                      pParentForTarget, aChildrenToReparent,
                                      pProgress, aMediumLockList,
                                      aWait /* fKeepMediumLockList */);
        rc = pTask->rc();
        AssertComRC(rc);
        if (FAILED(rc))
            throw rc;
    }
    catch (HRESULT aRC) { rc = aRC; }

    if (SUCCEEDED(rc))
    {
        if (aWait)
            rc = runNow(pTask);
        else
            rc = startThread(pTask);

        if (SUCCEEDED(rc) && aProgress != NULL)
            *aProgress = pProgress;
    }
    else if (pTask != NULL)
        delete pTask;

    return rc;
}

/**
 * Undoes what #prepareMergeTo() did. Must be called if #mergeTo() is not
 * called or fails. Frees memory occupied by @a aMediumLockList and unlocks
 * the medium objects in @a aChildrenToReparent.
 *
 * @param aChildrenToReparent List of children of the source which will have
 *                      to be reparented to the target after merge.
 * @param aMediumLockList Medium locking information.
 *
 * @note Locks the media from the chain for writing.
 */
void Medium::cancelMergeTo(const MediaList &aChildrenToReparent,
                           MediumLockList *aMediumLockList)
{
    AutoCaller autoCaller(this);
    AssertComRCReturnVoid(autoCaller.rc());

    AssertReturnVoid(aMediumLockList != NULL);

    /* Revert media marked for deletion to previous state. */
    HRESULT rc;
    MediumLockList::Base::const_iterator mediumListBegin =
        aMediumLockList->GetBegin();
    MediumLockList::Base::const_iterator mediumListEnd =
        aMediumLockList->GetEnd();
    for (MediumLockList::Base::const_iterator it = mediumListBegin;
         it != mediumListEnd;
         ++it)
    {
        const MediumLock &mediumLock = *it;
        const ComObjPtr<Medium> &pMedium = mediumLock.GetMedium();
        AutoWriteLock alock(pMedium COMMA_LOCKVAL_SRC_POS);

        if (pMedium->m->state == MediumState_Deleting)
        {
            rc = pMedium->unmarkForDeletion();
            AssertComRC(rc);
        }
    }

    /* the destructor will do the work */
    delete aMediumLockList;

    /* unlock the children which had to be reparented */
    for (MediaList::const_iterator it = aChildrenToReparent.begin();
         it != aChildrenToReparent.end();
         ++it)
    {
        const ComObjPtr<Medium> &pMedium = *it;

        AutoWriteLock alock(pMedium COMMA_LOCKVAL_SRC_POS);
        pMedium->UnlockWrite(NULL);
    }
}

/**
 * Fix the parent UUID of all children to point to this medium as their
 * parent.
 */
HRESULT Medium::fixParentUuidOfChildren(const MediaList &childrenToReparent)
{
    Assert(!isWriteLockOnCurrentThread());
    Assert(!m->pVirtualBox->getMediaTreeLockHandle().isWriteLockOnCurrentThread());
    MediumLockList mediumLockList;
    HRESULT rc = createMediumLockList(true /* fFailIfInaccessible */,
                                      false /* fMediumLockWrite */,
                                      this,
                                      mediumLockList);
    AssertComRCReturnRC(rc);

    try
    {
        PVBOXHDD hdd;
        int vrc = VDCreate(m->vdDiskIfaces, convertDeviceType(), &hdd);
        ComAssertRCThrow(vrc, E_FAIL);

        try
        {
            MediumLockList::Base::iterator lockListBegin =
                mediumLockList.GetBegin();
            MediumLockList::Base::iterator lockListEnd =
                mediumLockList.GetEnd();
            for (MediumLockList::Base::iterator it = lockListBegin;
                 it != lockListEnd;
                 ++it)
            {
                MediumLock &mediumLock = *it;
                const ComObjPtr<Medium> &pMedium = mediumLock.GetMedium();
                AutoReadLock alock(pMedium COMMA_LOCKVAL_SRC_POS);

                // open the medium
                vrc = VDOpen(hdd,
                             pMedium->m->strFormat.c_str(),
                             pMedium->m->strLocationFull.c_str(),
                             VD_OPEN_FLAGS_READONLY | m->uOpenFlagsDef,
                             pMedium->m->vdImageIfaces);
                if (RT_FAILURE(vrc))
                    throw vrc;
            }

            for (MediaList::const_iterator it = childrenToReparent.begin();
                 it != childrenToReparent.end();
                 ++it)
            {
                /* VD_OPEN_FLAGS_INFO since UUID is wrong yet */
                vrc = VDOpen(hdd,
                             (*it)->m->strFormat.c_str(),
                             (*it)->m->strLocationFull.c_str(),
                             VD_OPEN_FLAGS_INFO | m->uOpenFlagsDef,
                             (*it)->m->vdImageIfaces);
                if (RT_FAILURE(vrc))
                    throw vrc;

                vrc = VDSetParentUuid(hdd, VD_LAST_IMAGE, m->id.raw());
                if (RT_FAILURE(vrc))
                    throw vrc;

                vrc = VDClose(hdd, false /* fDelete */);
                if (RT_FAILURE(vrc))
                    throw vrc;

                (*it)->UnlockWrite(NULL);
            }
        }
        catch (HRESULT aRC) { rc = aRC; }
        catch (int aVRC)
        {
            rc = setError(E_FAIL,
                          tr("Could not update medium UUID references to parent '%s' (%s)"),
                          m->strLocationFull.c_str(),
                          vdError(aVRC).c_str());
        }

        VDDestroy(hdd);
    }
    catch (HRESULT aRC) { rc = aRC; }

    return rc;
}

/**
 * Used by IAppliance to export disk images.
 *
 * @param aFilename             Filename to create (UTF8).
 * @param aFormat               Medium format for creating @a aFilename.
 * @param aVariant              Which exact image format variant to use
 *                              for the destination image.
 * @param aVDImageIOCallbacks   Pointer to the callback table for a
 *                              VDINTERFACEIO interface. May be NULL.
 * @param aVDImageIOUser        Opaque data for the callbacks.
 * @param aProgress             Progress object to use.
 * @return
 * @note The source format is defined by the Medium instance.
 */
HRESULT Medium::exportFile(const char *aFilename,
                           const ComObjPtr<MediumFormat> &aFormat,
                           MediumVariant_T aVariant,
                           PVDINTERFACEIO aVDImageIOIf, void *aVDImageIOUser,
                           const ComObjPtr<Progress> &aProgress)
{
    AssertPtrReturn(aFilename, E_INVALIDARG);
    AssertReturn(!aFormat.isNull(), E_INVALIDARG);
    AssertReturn(!aProgress.isNull(), E_INVALIDARG);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    HRESULT rc = S_OK;
    Medium::Task *pTask = NULL;

    try
    {
        // This needs no extra locks besides what is done in the called methods.

        /* Build the source lock list. */
        MediumLockList *pSourceMediumLockList(new MediumLockList());
        rc = createMediumLockList(true /* fFailIfInaccessible */,
                                  false /* fMediumLockWrite */,
                                  NULL,
                                  *pSourceMediumLockList);
        if (FAILED(rc))
        {
            delete pSourceMediumLockList;
            throw rc;
        }

        rc = pSourceMediumLockList->Lock();
        if (FAILED(rc))
        {
            delete pSourceMediumLockList;
            throw setError(rc,
                           tr("Failed to lock source media '%s'"),
                           getLocationFull().c_str());
        }

        /* setup task object to carry out the operation asynchronously */
        pTask = new Medium::ExportTask(this, aProgress, aFilename, aFormat,
                                       aVariant, aVDImageIOIf,
                                       aVDImageIOUser, pSourceMediumLockList);
        rc = pTask->rc();
        AssertComRC(rc);
        if (FAILED(rc))
            throw rc;
    }
    catch (HRESULT aRC) { rc = aRC; }

    if (SUCCEEDED(rc))
        rc = startThread(pTask);
    else if (pTask != NULL)
        delete pTask;

    return rc;
}

/**
 * Used by IAppliance to import disk images.
 *
 * @param aFilename             Filename to read (UTF8).
 * @param aFormat               Medium format for reading @a aFilename.
 * @param aVariant              Which exact image format variant to use
 *                              for the destination image.
 * @param aVDImageIOCallbacks   Pointer to the callback table for a
 *                              VDINTERFACEIO interface. May be NULL.
 * @param aVDImageIOUser        Opaque data for the callbacks.
 * @param aParent               Parent medium. May be NULL.
 * @param aProgress             Progress object to use.
 * @return
 * @note The destination format is defined by the Medium instance.
 */
HRESULT Medium::importFile(const char *aFilename,
                           const ComObjPtr<MediumFormat> &aFormat,
                           MediumVariant_T aVariant,
                           PVDINTERFACEIO aVDImageIOIf, void *aVDImageIOUser,
                           const ComObjPtr<Medium> &aParent,
                           const ComObjPtr<Progress> &aProgress)
{
    AssertPtrReturn(aFilename, E_INVALIDARG);
    AssertReturn(!aFormat.isNull(), E_INVALIDARG);
    AssertReturn(!aProgress.isNull(), E_INVALIDARG);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    HRESULT rc = S_OK;
    Medium::Task *pTask = NULL;

    try
    {
        // locking: we need the tree lock first because we access parent pointers
        // and we need to write-lock the media involved
        uint32_t    cHandles    = 2;
        LockHandle* pHandles[3] = { &m->pVirtualBox->getMediaTreeLockHandle(),
                                    this->lockHandle() };
        /* Only add parent to the lock if it is not null */
        if (!aParent.isNull())
            pHandles[cHandles++] = aParent->lockHandle();
        AutoWriteLock alock(cHandles,
                            pHandles
                            COMMA_LOCKVAL_SRC_POS);

        if (   m->state != MediumState_NotCreated
            && m->state != MediumState_Created)
            throw setStateError();

        /* Build the target lock list. */
        MediumLockList *pTargetMediumLockList(new MediumLockList());
        alock.release();
        rc = createMediumLockList(true /* fFailIfInaccessible */,
                                  true /* fMediumLockWrite */,
                                  aParent,
                                  *pTargetMediumLockList);
        alock.acquire();
        if (FAILED(rc))
        {
            delete pTargetMediumLockList;
            throw rc;
        }

        alock.release();
        rc = pTargetMediumLockList->Lock();
        alock.acquire();
        if (FAILED(rc))
        {
            delete pTargetMediumLockList;
            throw setError(rc,
                           tr("Failed to lock target media '%s'"),
                           getLocationFull().c_str());
        }

        /* setup task object to carry out the operation asynchronously */
        pTask = new Medium::ImportTask(this, aProgress, aFilename, aFormat,
                                       aVariant, aVDImageIOIf,
                                       aVDImageIOUser, aParent,
                                       pTargetMediumLockList);
        rc = pTask->rc();
        AssertComRC(rc);
        if (FAILED(rc))
            throw rc;

        if (m->state == MediumState_NotCreated)
            m->state = MediumState_Creating;
    }
    catch (HRESULT aRC) { rc = aRC; }

    if (SUCCEEDED(rc))
        rc = startThread(pTask);
    else if (pTask != NULL)
        delete pTask;

    return rc;
}

/**
 * Internal version of the public CloneTo API which allows to enable certain
 * optimizations to improve speed during VM cloning.
 *
 * @param aTarget            Target medium
 * @param aVariant           Which exact image format variant to use
 *                           for the destination image.
 * @param aParent            Parent medium. May be NULL.
 * @param aProgress          Progress object to use.
 * @param idxSrcImageSame    The last image in the source chain which has the
 *                           same content as the given image in the destination
 *                           chain. Use UINT32_MAX to disable this optimization.
 * @param idxDstImageSame    The last image in the destination chain which has the
 *                           same content as the given image in the source chain.
 *                           Use UINT32_MAX to disable this optimization.
 * @return
 */
HRESULT Medium::cloneToEx(const ComObjPtr<Medium> &aTarget, ULONG aVariant,
                          const ComObjPtr<Medium> &aParent, IProgress **aProgress,
                          uint32_t idxSrcImageSame, uint32_t idxDstImageSame)
{
    CheckComArgNotNull(aTarget);
    CheckComArgOutPointerValid(aProgress);
    ComAssertRet(aTarget != this, E_INVALIDARG);

    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    HRESULT rc = S_OK;
    ComObjPtr<Progress> pProgress;
    Medium::Task *pTask = NULL;

    try
    {
        // locking: we need the tree lock first because we access parent pointers
        // and we need to write-lock the media involved
        uint32_t    cHandles    = 3;
        LockHandle* pHandles[4] = { &m->pVirtualBox->getMediaTreeLockHandle(),
                                    this->lockHandle(),
                                    aTarget->lockHandle() };
        /* Only add parent to the lock if it is not null */
        if (!aParent.isNull())
            pHandles[cHandles++] = aParent->lockHandle();
        AutoWriteLock alock(cHandles,
                            pHandles
                            COMMA_LOCKVAL_SRC_POS);

        if (    aTarget->m->state != MediumState_NotCreated
            &&  aTarget->m->state != MediumState_Created)
            throw aTarget->setStateError();

        /* Build the source lock list. */
        MediumLockList *pSourceMediumLockList(new MediumLockList());
        alock.release();
        rc = createMediumLockList(true /* fFailIfInaccessible */,
                                  false /* fMediumLockWrite */,
                                  NULL,
                                  *pSourceMediumLockList);
        alock.acquire();
        if (FAILED(rc))
        {
            delete pSourceMediumLockList;
            throw rc;
        }

        /* Build the target lock list (including the to-be parent chain). */
        MediumLockList *pTargetMediumLockList(new MediumLockList());
        alock.release();
        rc = aTarget->createMediumLockList(true /* fFailIfInaccessible */,
                                           true /* fMediumLockWrite */,
                                           aParent,
                                           *pTargetMediumLockList);
        alock.acquire();
        if (FAILED(rc))
        {
            delete pSourceMediumLockList;
            delete pTargetMediumLockList;
            throw rc;
        }

        alock.release();
        rc = pSourceMediumLockList->Lock();
        alock.acquire();
        if (FAILED(rc))
        {
            delete pSourceMediumLockList;
            delete pTargetMediumLockList;
            throw setError(rc,
                           tr("Failed to lock source media '%s'"),
                           getLocationFull().c_str());
        }
        alock.release();
        rc = pTargetMediumLockList->Lock();
        alock.acquire();
        if (FAILED(rc))
        {
            delete pSourceMediumLockList;
            delete pTargetMediumLockList;
            throw setError(rc,
                           tr("Failed to lock target media '%s'"),
                           aTarget->getLocationFull().c_str());
        }

        pProgress.createObject();
        rc = pProgress->init(m->pVirtualBox,
                             static_cast <IMedium *>(this),
                             BstrFmt(tr("Creating clone medium '%s'"), aTarget->m->strLocationFull.c_str()).raw(),
                             TRUE /* aCancelable */);
        if (FAILED(rc))
        {
            delete pSourceMediumLockList;
            delete pTargetMediumLockList;
            throw rc;
        }

        /* setup task object to carry out the operation asynchronously */
        pTask = new Medium::CloneTask(this, pProgress, aTarget,
                                      (MediumVariant_T)aVariant,
                                      aParent, idxSrcImageSame,
                                      idxDstImageSame, pSourceMediumLockList,
                                      pTargetMediumLockList);
        rc = pTask->rc();
        AssertComRC(rc);
        if (FAILED(rc))
            throw rc;

        if (aTarget->m->state == MediumState_NotCreated)
            aTarget->m->state = MediumState_Creating;
    }
    catch (HRESULT aRC) { rc = aRC; }

    if (SUCCEEDED(rc))
    {
        rc = startThread(pTask);

        if (SUCCEEDED(rc))
            pProgress.queryInterfaceTo(aProgress);
    }
    else if (pTask != NULL)
        delete pTask;

    return rc;
}

////////////////////////////////////////////////////////////////////////////////
//
// Private methods
//
////////////////////////////////////////////////////////////////////////////////

/**
 * Queries information from the medium.
 *
 * As a result of this call, the accessibility state and data members such as
 * size and description will be updated with the current information.
 *
 * @note This method may block during a system I/O call that checks storage
 *       accessibility.
 *
 * @note Caller MUST NOT hold the media tree or medium lock.
 *
 * @note Locks mParent for reading. Locks this object for writing.
 *
 * @param fSetImageId Whether to reset the UUID contained in the image file to the UUID in the medium instance data (see SetIDs())
 * @param fSetParentId Whether to reset the parent UUID contained in the image file to the parent UUID in the medium instance data (see SetIDs())
 * @return
 */
HRESULT Medium::queryInfo(bool fSetImageId, bool fSetParentId)
{
    Assert(!isWriteLockOnCurrentThread());
    AutoWriteLock alock(this COMMA_LOCKVAL_SRC_POS);

    if (   m->state != MediumState_Created
        && m->state != MediumState_Inaccessible
        && m->state != MediumState_LockedRead)
        return E_FAIL;

    HRESULT rc = S_OK;

    int vrc = VINF_SUCCESS;

    /* check if a blocking queryInfo() call is in progress on some other thread,
     * and wait for it to finish if so instead of querying data ourselves */
    if (m->queryInfoRunning)
    {
        Assert(   m->state == MediumState_LockedRead
               || m->state == MediumState_LockedWrite);

        while (m->queryInfoRunning)
        {
            alock.release();
            {
                AutoReadLock qlock(m->queryInfoSem COMMA_LOCKVAL_SRC_POS);
            }
            alock.acquire();
        }

        return S_OK;
    }

    bool success = false;
    Utf8Str lastAccessError;

    /* are we dealing with a new medium constructed using the existing
     * location? */
    bool isImport = m->id.isEmpty();
    unsigned uOpenFlags = VD_OPEN_FLAGS_INFO;

    /* Note that we don't use VD_OPEN_FLAGS_READONLY when opening new
     * media because that would prevent necessary modifications
     * when opening media of some third-party formats for the first
     * time in VirtualBox (such as VMDK for which VDOpen() needs to
     * generate an UUID if it is missing) */
    if (    m->hddOpenMode == OpenReadOnly
         || m->type == MediumType_Readonly
         || (!isImport && !fSetImageId && !fSetParentId)
       )
        uOpenFlags |= VD_OPEN_FLAGS_READONLY;

    /* Open shareable medium with the appropriate flags */
    if (m->type == MediumType_Shareable)
        uOpenFlags |= VD_OPEN_FLAGS_SHAREABLE;

    /* Lock the medium, which makes the behavior much more consistent */
    alock.release();
    if (uOpenFlags & (VD_OPEN_FLAGS_READONLY | VD_OPEN_FLAGS_SHAREABLE))
        rc = LockRead(NULL);
    else
        rc = LockWrite(NULL);
    if (FAILED(rc)) return rc;
    alock.acquire();

    /* Copies of the input state fields which are not read-only,
     * as we're dropping the lock. CAUTION: be extremely careful what
     * you do with the contents of this medium object, as you will
     * create races if there are concurrent changes. */
    Utf8Str format(m->strFormat);
    Utf8Str location(m->strLocationFull);
    ComObjPtr<MediumFormat> formatObj = m->formatObj;

    /* "Output" values which can't be set because the lock isn't held
     * at the time the values are determined. */
    Guid mediumId = m->id;
    uint64_t mediumSize = 0;
    uint64_t mediumLogicalSize = 0;

    /* Flag whether a base image has a non-zero parent UUID and thus
     * need repairing after it was closed again. */
    bool fRepairImageZeroParentUuid = false;

    /* release the object lock before a lengthy operation, and take the
     * opportunity to have a media tree lock, too, which isn't held initially */
    m->queryInfoRunning = true;
    alock.release();
    Assert(!isWriteLockOnCurrentThread());
    Assert(!m->pVirtualBox->getMediaTreeLockHandle().isWriteLockOnCurrentThread());
    AutoWriteLock treeLock(m->pVirtualBox->getMediaTreeLockHandle() COMMA_LOCKVAL_SRC_POS);
    treeLock.release();

    /* Note that taking the queryInfoSem after leaving the object lock above
     * can lead to short spinning of the loops waiting for queryInfo() to
     * complete. This is unavoidable since the other order causes a lock order
     * violation: here it would be requesting the object lock (at the beginning
     * of the method), then queryInfoSem, and below the other way round. */
    AutoWriteLock qlock(m->queryInfoSem COMMA_LOCKVAL_SRC_POS);

    try
    {
        /* skip accessibility checks for host drives */
        if (m->hostDrive)
        {
            success = true;
            throw S_OK;
        }

        PVBOXHDD hdd;
        vrc = VDCreate(m->vdDiskIfaces, convertDeviceType(), &hdd);
        ComAssertRCThrow(vrc, E_FAIL);

        try
        {
            /** @todo This kind of opening of media is assuming that diff
             * media can be opened as base media. Should be documented that
             * it must work for all medium format backends. */
            vrc = VDOpen(hdd,
                         format.c_str(),
                         location.c_str(),
                         uOpenFlags | m->uOpenFlagsDef,
                         m->vdImageIfaces);
            if (RT_FAILURE(vrc))
            {
                lastAccessError = Utf8StrFmt(tr("Could not open the medium '%s'%s"),
                                             location.c_str(), vdError(vrc).c_str());
                throw S_OK;
            }

            if (formatObj->getCapabilities() & MediumFormatCapabilities_Uuid)
            {
                /* Modify the UUIDs if necessary. The associated fields are
                 * not modified by other code, so no need to copy. */
                if (fSetImageId)
                {
                    alock.acquire();
                    vrc = VDSetUuid(hdd, 0, m->uuidImage.raw());
                    alock.release();
                    if (RT_FAILURE(vrc))
                    {
                        lastAccessError = Utf8StrFmt(tr("Could not update the UUID of medium '%s'%s"),
                                         location.c_str(), vdError(vrc).c_str());
                        throw S_OK;
                    }
                    mediumId = m->uuidImage;
                }
                if (fSetParentId)
                {
                    alock.acquire();
                    vrc = VDSetParentUuid(hdd, 0, m->uuidParentImage.raw());
                    alock.release();
                    if (RT_FAILURE(vrc))
                    {
                        lastAccessError = Utf8StrFmt(tr("Could not update the parent UUID of medium '%s'%s"),
                                         location.c_str(), vdError(vrc).c_str());
                        throw S_OK;
                    }
                }
                /* zap the information, these are no long-term members */
                alock.acquire();
                unconst(m->uuidImage).clear();
                unconst(m->uuidParentImage).clear();
                alock.release();

                /* check the UUID */
                RTUUID uuid;
                vrc = VDGetUuid(hdd, 0, &uuid);
                ComAssertRCThrow(vrc, E_FAIL);

                if (isImport)
                {
                    mediumId = uuid;

                    if (mediumId.isEmpty() && (m->hddOpenMode == OpenReadOnly))
                        // only when importing a VDMK that has no UUID, create one in memory
                        mediumId.create();
                }
                else
                {
                    Assert(!mediumId.isEmpty());

                    if (mediumId != uuid)
                    {
                        /** @todo r=klaus this always refers to VirtualBox.xml as the medium registry, even for new VMs */
                        lastAccessError = Utf8StrFmt(
                                tr("UUID {%RTuuid} of the medium '%s' does not match the value {%RTuuid} stored in the media registry ('%s')"),
                                &uuid,
                                location.c_str(),
                                mediumId.raw(),
                                m->pVirtualBox->settingsFilePath().c_str());
                        throw S_OK;
                    }
                }
            }
            else
            {
                /* the backend does not support storing UUIDs within the
                 * underlying storage so use what we store in XML */

                if (fSetImageId)
                {
                    /* set the UUID if an API client wants to change it */
                    alock.acquire();
                    mediumId = m->uuidImage;
                    alock.release();
                }
                else if (isImport)
                {
                    /* generate an UUID for an imported UUID-less medium */
                    mediumId.create();
                }
            }

            /* set the image uuid before the below parent uuid handling code
             * might place it somewhere in the media tree, so that the medium
             * UUID is valid at this point */
            alock.acquire();
            if (isImport || fSetImageId)
                unconst(m->id) = mediumId;
            alock.release();

            /* get the medium variant */
            unsigned uImageFlags;
            vrc = VDGetImageFlags(hdd, 0, &uImageFlags);
            ComAssertRCThrow(vrc, E_FAIL);
            alock.acquire();
            m->variant = (MediumVariant_T)uImageFlags;
            alock.release();

            /* check/get the parent uuid and update corresponding state */
            if (uImageFlags & VD_IMAGE_FLAGS_DIFF)
            {
                RTUUID parentId;
                vrc = VDGetParentUuid(hdd, 0, &parentId);
                ComAssertRCThrow(vrc, E_FAIL);

                /* streamOptimized VMDK images are only accepted as base
                 * images, as this allows automatic repair of OVF appliances.
                 * Since such images don't support random writes they will not
                 * be created for diff images. Only an overly smart user might
                 * manually create this case. Too bad for him. */
                if (   (isImport || fSetParentId)
                    && !(uImageFlags & VD_VMDK_IMAGE_FLAGS_STREAM_OPTIMIZED))
                {
                    /* the parent must be known to us. Note that we freely
                     * call locking methods of mVirtualBox and parent, as all
                     * relevant locks must be already held. There may be no
                     * concurrent access to the just opened medium on other
                     * threads yet (and init() will fail if this method reports
                     * MediumState_Inaccessible) */

                    ComObjPtr<Medium> pParent;
                    if (RTUuidIsNull(&parentId))
                        rc = VBOX_E_OBJECT_NOT_FOUND;
                    else
                        rc = m->pVirtualBox->findHardDiskById(Guid(parentId), false /* aSetError */, &pParent);
                    if (FAILED(rc))
                    {
                        if (fSetImageId && !fSetParentId)
                        {
                            /* If the image UUID gets changed for an existing
                             * image then the parent UUID can be stale. In such
                             * cases clear the parent information. The parent
                             * information may/will be re-set later if the
                             * API client wants to adjust a complete medium
                             * hierarchy one by one. */
                            rc = S_OK;
                            alock.acquire();
                            RTUuidClear(&parentId);
                            vrc = VDSetParentUuid(hdd, 0, &parentId);
                            alock.release();
                            ComAssertRCThrow(vrc, E_FAIL);
                        }
                        else
                        {
                            lastAccessError = Utf8StrFmt(tr("Parent medium with UUID {%RTuuid} of the medium '%s' is not found in the media registry ('%s')"),
                                                         &parentId, location.c_str(),
                                                         m->pVirtualBox->settingsFilePath().c_str());
                            throw S_OK;
                        }
                    }

                    /* we set mParent & children() */
                    treeLock.acquire();

                    if (m->pParent)
                        deparent();
                    setParent(pParent);

                    treeLock.release();
                }
                else
                {
                    /* we access mParent */
                    treeLock.acquire();

                    /* check that parent UUIDs match. Note that there's no need
                     * for the parent's AutoCaller (our lifetime is bound to
                     * it) */

                    if (m->pParent.isNull())
                    {
                        /* Due to a bug in VDCopy() in VirtualBox 3.0.0-3.0.14
                         * and 3.1.0-3.1.8 there are base images out there
                         * which have a non-zero parent UUID. No point in
                         * complaining about them, instead automatically
                         * repair the problem. Later we can bring back the
                         * error message, but we should wait until really
                         * most users have repaired their images, either with
                         * VBoxFixHdd or this way. */
#if 1
                        fRepairImageZeroParentUuid = true;
#else /* 0 */
                        lastAccessError = Utf8StrFmt(
                                tr("Medium type of '%s' is differencing but it is not associated with any parent medium in the media registry ('%s')"),
                                location.c_str(),
                                m->pVirtualBox->settingsFilePath().c_str());
                        treeLock.release();
                        throw S_OK;
#endif /* 0 */
                    }

                    {
                        AutoReadLock parentLock(m->pParent COMMA_LOCKVAL_SRC_POS);
                        if (   !fRepairImageZeroParentUuid
                            && m->pParent->getState() != MediumState_Inaccessible
                            && m->pParent->getId() != parentId)
                        {
                            /** @todo r=klaus this always refers to VirtualBox.xml as the medium registry, even for new VMs */
                            lastAccessError = Utf8StrFmt(
                                    tr("Parent UUID {%RTuuid} of the medium '%s' does not match UUID {%RTuuid} of its parent medium stored in the media registry ('%s')"),
                                    &parentId, location.c_str(),
                                    m->pParent->getId().raw(),
                                    m->pVirtualBox->settingsFilePath().c_str());
                            parentLock.release();
                            treeLock.release();
                            throw S_OK;
                        }
                    }

                    /// @todo NEWMEDIA what to do if the parent is not
                    /// accessible while the diff is? Probably nothing. The
                    /// real code will detect the mismatch anyway.

                    treeLock.release();
                }
            }

            mediumSize = VDGetFileSize(hdd, 0);
            mediumLogicalSize = VDGetSize(hdd, 0);

            success = true;
        }
        catch (HRESULT aRC)
        {
            rc = aRC;
        }

        vrc = VDDestroy(hdd);
        if (RT_FAILURE(vrc))
        {
            lastAccessError = Utf8StrFmt(tr("Could not update and close the medium '%s'%s"),
                                         location.c_str(), vdError(vrc).c_str());
            success = false;
            throw S_OK;
        }
    }
    catch (HRESULT aRC)
    {
        rc = aRC;
    }

    treeLock.acquire();
    alock.acquire();

    if (success)
    {
        m->size = mediumSize;
        m->logicalSize = mediumLogicalSize;
        m->strLastAccessError.setNull();
    }
    else
    {
        m->strLastAccessError = lastAccessError;
        LogWarningFunc(("'%s' is not accessible (error='%s', rc=%Rhrc, vrc=%Rrc)\n",
                        location.c_str(), m->strLastAccessError.c_str(),
                        rc, vrc));
    }

    /* unblock anyone waiting for the queryInfo results */
    qlock.release();
    m->queryInfoRunning = false;

    /* Set the proper state according to the result of the check */
    if (success)
        m->preLockState = MediumState_Created;
    else
        m->preLockState = MediumState_Inaccessible;

    HRESULT rc2;
    if (uOpenFlags & (VD_OPEN_FLAGS_READONLY | VD_OPEN_FLAGS_SHAREABLE))
        rc2 = UnlockRead(NULL);
    else
        rc2 = UnlockWrite(NULL);
    if (SUCCEEDED(rc) && FAILED(rc2))
        rc = rc2;
    if (FAILED(rc)) return rc;

    /* If this is a base image which incorrectly has a parent UUID set,
     * repair the image now by zeroing the parent UUID. This is only done
     * when we have structural information from a config file, on import
     * this is not possible. If someone would accidentally call openMedium
     * with a diff image before the base is registered this would destroy
     * the diff. Not acceptable. */
    if (fRepairImageZeroParentUuid)
    {
        rc = LockWrite(NULL);
        if (FAILED(rc)) return rc;

        alock.release();

        try
        {
            PVBOXHDD hdd;
            vrc = VDCreate(m->vdDiskIfaces, convertDeviceType(), &hdd);
            ComAssertRCThrow(vrc, E_FAIL);

            try
            {
                vrc = VDOpen(hdd,
                             format.c_str(),
                             location.c_str(),
                             (uOpenFlags & ~VD_OPEN_FLAGS_READONLY) | m->uOpenFlagsDef,
                             m->vdImageIfaces);
                if (RT_FAILURE(vrc))
                    throw S_OK;

                RTUUID zeroParentUuid;
                RTUuidClear(&zeroParentUuid);
                vrc = VDSetParentUuid(hdd, 0, &zeroParentUuid);
                ComAssertRCThrow(vrc, E_FAIL);
            }
            catch (HRESULT aRC)
            {
                rc = aRC;
            }

            VDDestroy(hdd);
        }
        catch (HRESULT aRC)
        {
            rc = aRC;
        }

        rc = UnlockWrite(NULL);
        if (SUCCEEDED(rc) && FAILED(rc2))
            rc = rc2;
        if (FAILED(rc)) return rc;
    }

    return rc;
}

/**
 * Performs extra checks if the medium can be closed and returns S_OK in
 * this case. Otherwise, returns a respective error message. Called by
 * Close() under the medium tree lock and the medium lock.
 *
 * @note Also reused by Medium::Reset().
 *
 * @note Caller must hold the media tree write lock!
 */
HRESULT Medium::canClose()
{
    Assert(m->pVirtualBox->getMediaTreeLockHandle().isWriteLockOnCurrentThread());

    if (getChildren().size() != 0)
        return setError(VBOX_E_OBJECT_IN_USE,
                        tr("Cannot close medium '%s' because it has %d child media"),
                        m->strLocationFull.c_str(), getChildren().size());

    return S_OK;
}

/**
 * Unregisters this medium with mVirtualBox. Called by close() under the medium tree lock.
 *
 * @note Caller must have locked the media tree lock for writing!
 */
HRESULT Medium::unregisterWithVirtualBox()
{
    /* Note that we need to de-associate ourselves from the parent to let
     * unregisterMedium() properly save the registry */

    /* we modify mParent and access children */
    Assert(m->pVirtualBox->getMediaTreeLockHandle().isWriteLockOnCurrentThread());

    Medium *pParentBackup = m->pParent;
    AssertReturn(getChildren().size() == 0, E_FAIL);
    if (m->pParent)
        deparent();

    HRESULT rc = m->pVirtualBox->unregisterMedium(this);
    if (FAILED(rc))
    {
        if (pParentBackup)
        {
            // re-associate with the parent as we are still relatives in the registry
            m->pParent = pParentBackup;
            m->pParent->m->llChildren.push_back(this);
        }
    }

    return rc;
}

/**
 * Like SetProperty but do not trigger a settings store. Only for internal use!
 */
HRESULT Medium::setPropertyDirect(const Utf8Str &aName, const Utf8Str &aValue)
{
    AutoCaller autoCaller(this);
    if (FAILED(autoCaller.rc())) return autoCaller.rc();

    AutoWriteLock mlock(this COMMA_LOCKVAL_SRC_POS);

    switch (m->state)
    {
        case MediumState_Created:
        case MediumState_Inaccessible:
            break;
        default:
            return setStateError();
    }

    m->mapProperties[aName] = aValue;

    return S_OK;
}

/**
 * Sets the extended error info according to the current media state.
 *
 * @note Must be called from under this object's write or read lock.
 */
HRESULT Medium::setStateError()
{
    HRESULT rc = E_FAIL;

    switch (m->state)
    {
        case MediumState_NotCreated:
        {
            rc = setError(VBOX_E_INVALID_OBJECT_STATE,
                          tr("Storage for the medium '%s' is not created"),
                          m->strLocationFull.c_str());
            break;
        }
        case MediumState_Created:
        {
            rc = setError(VBOX_E_INVALID_OBJECT_STATE,
                          tr("Storage for the medium '%s' is already created"),
                          m->strLocationFull.c_str());
            break;
        }
        case MediumState_LockedRead:
        {
            rc = setError(VBOX_E_INVALID_OBJECT_STATE,
                          tr("Medium '%s' is locked for reading by another task"),
                          m->strLocationFull.c_str());
            break;
        }
        case MediumState_LockedWrite:
        {
            rc = setError(VBOX_E_INVALID_OBJECT_STATE,
                          tr("Medium '%s' is locked for writing by another task"),
                          m->strLocationFull.c_str());
            break;
        }
        case MediumState_Inaccessible:
        {
            /* be in sync with Console::powerUpThread() */
            if (!m->strLastAccessError.isEmpty())
                rc = setError(VBOX_E_INVALID_OBJECT_STATE,
                              tr("Medium '%s' is not accessible. %s"),
                              m->strLocationFull.c_str(), m->strLastAccessError.c_str());
            else
                rc = setError(VBOX_E_INVALID_OBJECT_STATE,
                              tr("Medium '%s' is not accessible"),
                              m->strLocationFull.c_str());
            break;
        }
        case MediumState_Creating:
        {
            rc = setError(VBOX_E_INVALID_OBJECT_STATE,
                          tr("Storage for the medium '%s' is being created"),
                          m->strLocationFull.c_str());
            break;
        }
        case MediumState_Deleting:
        {
            rc = setError(VBOX_E_INVALID_OBJECT_STATE,
                          tr("Storage for the medium '%s' is being deleted"),
                          m->strLocationFull.c_str());
            break;
        }
        default:
        {
            AssertFailed();
            break;
        }
    }

    return rc;
}

/**
 * Sets the value of m->strLocationFull. The given location must be a fully
 * qualified path; relative paths are not supported here.
 *
 * As a special exception, if the specified location is a file path that ends with '/'
 * then the file name part will be generated by this method automatically in the format
 * '{<uuid>}.<ext>' where <uuid> is a fresh UUID that this method will generate
 * and assign to this medium, and <ext> is the default extension for this
 * medium's storage format. Note that this procedure requires the media state to
 * be NotCreated and will return a failure otherwise.
 *
 * @param aLocation Location of the storage unit. If the location is a FS-path,
 *                  then it can be relative to the VirtualBox home directory.
 * @param aFormat   Optional fallback format if it is an import and the format
 *                  cannot be determined.
 *
 * @note Must be called from under this object's write lock.
 */
HRESULT Medium::setLocation(const Utf8Str &aLocation,
                            const Utf8Str &aFormat /* = Utf8Str::Empty */)
{
    AssertReturn(!aLocation.isEmpty(), E_FAIL);

    AutoCaller autoCaller(this);
    AssertComRCReturnRC(autoCaller.rc());

    /* formatObj may be null only when initializing from an existing path and
     * no format is known yet */
    AssertReturn(    (!m->strFormat.isEmpty() && !m->formatObj.isNull())
                  || (    autoCaller.state() == InInit
                       && m->state != MediumState_NotCreated
                       && m->id.isEmpty()
                       && m->strFormat.isEmpty()
                       && m->formatObj.isNull()),
                 E_FAIL);

    /* are we dealing with a new medium constructed using the existing
     * location? */
    bool isImport = m->strFormat.isEmpty();

    if (   isImport
        || (   (m->formatObj->getCapabilities() & MediumFormatCapabilities_File)
            && !m->hostDrive))
    {
        Guid id;

        Utf8Str locationFull(aLocation);

        if (m->state == MediumState_NotCreated)
        {
            /* must be a file (formatObj must be already known) */
            Assert(m->formatObj->getCapabilities() & MediumFormatCapabilities_File);

            if (RTPathFilename(aLocation.c_str()) == NULL)
            {
                /* no file name is given (either an empty string or ends with a
                 * slash), generate a new UUID + file name if the state allows
                 * this */

                ComAssertMsgRet(!m->formatObj->getFileExtensions().empty(),
                                ("Must be at least one extension if it is MediumFormatCapabilities_File\n"),
                                E_FAIL);

                Utf8Str strExt = m->formatObj->getFileExtensions().front();
                ComAssertMsgRet(!strExt.isEmpty(),
                                ("Default extension must not be empty\n"),
                                E_FAIL);

                id.create();

                locationFull = Utf8StrFmt("%s{%RTuuid}.%s",
                                          aLocation.c_str(), id.raw(), strExt.c_str());
            }
        }

        // we must always have full paths now (if it refers to a file)
        if (   (   m->formatObj.isNull()
                || m->formatObj->getCapabilities() & MediumFormatCapabilities_File)
            && !RTPathStartsWithRoot(locationFull.c_str()))
            return setError(VBOX_E_FILE_ERROR,
                            tr("The given path '%s' is not fully qualified"),
                            locationFull.c_str());

        /* detect the backend from the storage unit if importing */
        if (isImport)
        {
            VDTYPE enmType = VDTYPE_INVALID;
            char *backendName = NULL;

            int vrc = VINF_SUCCESS;

            /* is it a file? */
            {
                RTFILE file;
                vrc = RTFileOpen(&file, locationFull.c_str(), RTFILE_O_READ | RTFILE_O_OPEN | RTFILE_O_DENY_NONE);
                if (RT_SUCCESS(vrc))
                    RTFileClose(file);
            }
            if (RT_SUCCESS(vrc))
            {
                vrc = VDGetFormat(NULL /* pVDIfsDisk */, NULL /* pVDIfsImage */,
                                  locationFull.c_str(), &backendName, &enmType);
            }
            else if (   vrc != VERR_FILE_NOT_FOUND
                     && vrc != VERR_PATH_NOT_FOUND
                     && vrc != VERR_ACCESS_DENIED
                     && locationFull != aLocation)
            {
                /* assume it's not a file, restore the original location */
                locationFull = aLocation;
                vrc = VDGetFormat(NULL /* pVDIfsDisk */, NULL /* pVDIfsImage */,
                                  locationFull.c_str(), &backendName, &enmType);
            }

            if (RT_FAILURE(vrc))
            {
                if (vrc == VERR_ACCESS_DENIED)
                    return setError(VBOX_E_FILE_ERROR,
                                    tr("Permission problem accessing the file for the medium '%s' (%Rrc)"),
                                    locationFull.c_str(), vrc);
                else if (vrc == VERR_FILE_NOT_FOUND || vrc == VERR_PATH_NOT_FOUND)
                    return setError(VBOX_E_FILE_ERROR,
                                    tr("Could not find file for the medium '%s' (%Rrc)"),
                                    locationFull.c_str(), vrc);
                else if (aFormat.isEmpty())
                    return setError(VBOX_E_IPRT_ERROR,
                                    tr("Could not get the storage format of the medium '%s' (%Rrc)"),
                                    locationFull.c_str(), vrc);
                else
                {
                    HRESULT rc = setFormat(aFormat);
                    /* setFormat() must not fail since we've just used the backend so
                     * the format object must be there */
                    AssertComRCReturnRC(rc);
                }
            }
            else if (   enmType == VDTYPE_INVALID
                     || m->devType != convertToDeviceType(enmType))
            {
                /*
                 * The user tried to use a image as a device which is not supported
                 * by the backend.
                 */
                return setError(E_FAIL,
                                tr("The medium '%s' can't be used as the requested device type"),
                                locationFull.c_str());
            }
            else
            {
                ComAssertRet(backendName != NULL && *backendName != '\0', E_FAIL);

                HRESULT rc = setFormat(backendName);
                RTStrFree(backendName);

                /* setFormat() must not fail since we've just used the backend so
                 * the format object must be there */
                AssertComRCReturnRC(rc);
            }
        }

        m->strLocationFull = locationFull;

        /* is it still a file? */
        if (    (m->formatObj->getCapabilities() & MediumFormatCapabilities_File)
             && (m->state == MediumState_NotCreated)
           )
            /* assign a new UUID (this UUID will be used when calling
             * VDCreateBase/VDCreateDiff as a wanted UUID). Note that we
             * also do that if we didn't generate it to make sure it is
             * either generated by us or reset to null */
            unconst(m->id) = id;
    }
    else
        m->strLocationFull = aLocation;

    return S_OK;
}

/**
 * Checks that the format ID is valid and sets it on success.
 *
 * Note that this method will caller-reference the format object on success!
 * This reference must be released somewhere to let the MediumFormat object be
 * uninitialized.
 *
 * @note Must be called from under this object's write lock.
 */
HRESULT Medium::setFormat(const Utf8Str &aFormat)
{
    /* get the format object first */
    {
        SystemProperties *pSysProps = m->pVirtualBox->getSystemProperties();
        AutoReadLock propsLock(pSysProps COMMA_LOCKVAL_SRC_POS);

        unconst(m->formatObj) = pSysProps->mediumFormat(aFormat);
        if (m->formatObj.isNull())
            return setError(E_INVALIDARG,
                            tr("Invalid medium storage format '%s'"),
                            aFormat.c_str());

        /* reference the format permanently to prevent its unexpected
         * uninitialization */
        HRESULT rc = m->formatObj->addCaller();
        AssertComRCReturnRC(rc);

        /* get properties (preinsert them as keys in the map). Note that the
         * map doesn't grow over the object life time since the set of
         * properties is meant to be constant. */

        Assert(m->mapProperties.empty());

        for (MediumFormat::PropertyList::const_iterator it = m->formatObj->getProperties().begin();
             it != m->formatObj->getProperties().end();
             ++it)
        {
            m->mapProperties.insert(std::make_pair(it->strName, Utf8Str::Empty));
        }
    }

    unconst(m->strFormat) = aFormat;

    return S_OK;
}

/**
 * Converts the Medium device type to the VD type.
 */
VDTYPE Medium::convertDeviceType()
{
    VDTYPE enmType;

    switch (m->devType)
    {
        case DeviceType_HardDisk:
            enmType = VDTYPE_HDD;
            break;
        case DeviceType_DVD:
            enmType = VDTYPE_DVD;
            break;
        case DeviceType_Floppy:
            enmType = VDTYPE_FLOPPY;
            break;
        default:
            ComAssertFailedRet(VDTYPE_INVALID);
    }

    return enmType;
}

/**
 * Converts from the VD type to the medium type.
 */
DeviceType_T Medium::convertToDeviceType(VDTYPE enmType)
{
    DeviceType_T devType;

    switch (enmType)
    {
        case VDTYPE_HDD:
            devType = DeviceType_HardDisk;
            break;
        case VDTYPE_DVD:
            devType = DeviceType_DVD;
            break;
        case VDTYPE_FLOPPY:
            devType = DeviceType_Floppy;
            break;
        default:
            ComAssertFailedRet(DeviceType_Null);
    }

    return devType;
}

/**
 * Returns the last error message collected by the vdErrorCall callback and
 * resets it.
 *
 * The error message is returned prepended with a dot and a space, like this:
 * <code>
 *   ". <error_text> (%Rrc)"
 * </code>
 * to make it easily appendable to a more general error message. The @c %Rrc
 * format string is given @a aVRC as an argument.
 *
 * If there is no last error message collected by vdErrorCall or if it is a
 * null or empty string, then this function returns the following text:
 * <code>
 *   " (%Rrc)"
 * </code>
 *
 * @note Doesn't do any object locking; it is assumed that the caller makes sure
 *       the callback isn't called by more than one thread at a time.
 *
 * @param aVRC  VBox error code to use when no error message is provided.
 */
Utf8Str Medium::vdError(int aVRC)
{
    Utf8Str error;

    if (m->vdError.isEmpty())
        error = Utf8StrFmt(" (%Rrc)", aVRC);
    else
        error = Utf8StrFmt(".\n%s", m->vdError.c_str());

    m->vdError.setNull();

    return error;
}

/**
 * Error message callback.
 *
 * Puts the reported error message to the m->vdError field.
 *
 * @note Doesn't do any object locking; it is assumed that the caller makes sure
 *       the callback isn't called by more than one thread at a time.
 *
 * @param   pvUser          The opaque data passed on container creation.
 * @param   rc              The VBox error code.
 * @param   RT_SRC_POS_DECL Use RT_SRC_POS.
 * @param   pszFormat       Error message format string.
 * @param   va              Error message arguments.
 */
/*static*/
DECLCALLBACK(void) Medium::vdErrorCall(void *pvUser, int rc, RT_SRC_POS_DECL,
                                       const char *pszFormat, va_list va)
{
    NOREF(pszFile); NOREF(iLine); NOREF(pszFunction); /* RT_SRC_POS_DECL */

    Medium *that = static_cast<Medium*>(pvUser);
    AssertReturnVoid(that != NULL);

    if (that->m->vdError.isEmpty())
        that->m->vdError =
            Utf8StrFmt("%s (%Rrc)", Utf8Str(pszFormat, va).c_str(), rc);
    else
        that->m->vdError =
            Utf8StrFmt("%s.\n%s (%Rrc)", that->m->vdError.c_str(),
                       Utf8Str(pszFormat, va).c_str(), rc);
}

/* static */
DECLCALLBACK(bool) Medium::vdConfigAreKeysValid(void *pvUser,
                                                const char * /* pszzValid */)
{
    Medium *that = static_cast<Medium*>(pvUser);
    AssertReturn(that != NULL, false);

    /* we always return true since the only keys we have are those found in
     * VDBACKENDINFO */
    return true;
}

/* static */
DECLCALLBACK(int) Medium::vdConfigQuerySize(void *pvUser,
                                            const char *pszName,
                                            size_t *pcbValue)
{
    AssertReturn(VALID_PTR(pcbValue), VERR_INVALID_POINTER);

    Medium *that = static_cast<Medium*>(pvUser);
    AssertReturn(that != NULL, VERR_GENERAL_FAILURE);

    settings::StringsMap::const_iterator it = that->m->mapProperties.find(Utf8Str(pszName));
    if (it == that->m->mapProperties.end())
        return VERR_CFGM_VALUE_NOT_FOUND;

    /* we interpret null values as "no value" in Medium */
    if (it->second.isEmpty())
        return VERR_CFGM_VALUE_NOT_FOUND;

    *pcbValue = it->second.length() + 1 /* include terminator */;

    return VINF_SUCCESS;
}

/* static */
DECLCALLBACK(int) Medium::vdConfigQuery(void *pvUser,
                                        const char *pszName,
                                        char *pszValue,
                                        size_t cchValue)
{
    AssertReturn(VALID_PTR(pszValue), VERR_INVALID_POINTER);

    Medium *that = static_cast<Medium*>(pvUser);
    AssertReturn(that != NULL, VERR_GENERAL_FAILURE);

    settings::StringsMap::const_iterator it = that->m->mapProperties.find(Utf8Str(pszName));
    if (it == that->m->mapProperties.end())
        return VERR_CFGM_VALUE_NOT_FOUND;

    /* we interpret null values as "no value" in Medium */
    if (it->second.isEmpty())
        return VERR_CFGM_VALUE_NOT_FOUND;

    const Utf8Str &value = it->second;
    if (value.length() >= cchValue)
        return VERR_CFGM_NOT_ENOUGH_SPACE;

    memcpy(pszValue, value.c_str(), value.length() + 1);

    return VINF_SUCCESS;
}

DECLCALLBACK(int) Medium::vdTcpSocketCreate(uint32_t fFlags, PVDSOCKET pSock)
{
    PVDSOCKETINT pSocketInt = NULL;

    if ((fFlags & VD_INTERFACETCPNET_CONNECT_EXTENDED_SELECT) != 0)
        return VERR_NOT_SUPPORTED;

    pSocketInt = (PVDSOCKETINT)RTMemAllocZ(sizeof(VDSOCKETINT));
    if (!pSocketInt)
        return VERR_NO_MEMORY;

    pSocketInt->hSocket = NIL_RTSOCKET;
    *pSock = pSocketInt;
    return VINF_SUCCESS;
}

DECLCALLBACK(int) Medium::vdTcpSocketDestroy(VDSOCKET Sock)
{
    PVDSOCKETINT pSocketInt = (PVDSOCKETINT)Sock;

    if (pSocketInt->hSocket != NIL_RTSOCKET)
        RTTcpClientCloseEx(pSocketInt->hSocket, false /*fGracefulShutdown*/);

    RTMemFree(pSocketInt);

    return VINF_SUCCESS;
}

DECLCALLBACK(int) Medium::vdTcpClientConnect(VDSOCKET Sock, const char *pszAddress, uint32_t uPort)
{
    PVDSOCKETINT pSocketInt = (PVDSOCKETINT)Sock;

    return RTTcpClientConnect(pszAddress, uPort, &pSocketInt->hSocket);
}

DECLCALLBACK(int) Medium::vdTcpClientClose(VDSOCKET Sock)
{
    int rc = VINF_SUCCESS;
    PVDSOCKETINT pSocketInt = (PVDSOCKETINT)Sock;

    rc = RTTcpClientCloseEx(pSocketInt->hSocket, false /*fGracefulShutdown*/);
    pSocketInt->hSocket = NIL_RTSOCKET;
    return rc;
}

DECLCALLBACK(bool) Medium::vdTcpIsClientConnected(VDSOCKET Sock)
{
    PVDSOCKETINT pSocketInt = (PVDSOCKETINT)Sock;
    return pSocketInt->hSocket != NIL_RTSOCKET;
}

DECLCALLBACK(int) Medium::vdTcpSelectOne(VDSOCKET Sock, RTMSINTERVAL cMillies)
{
    PVDSOCKETINT pSocketInt = (PVDSOCKETINT)Sock;
    return RTTcpSelectOne(pSocketInt->hSocket, cMillies);
}

DECLCALLBACK(int) Medium::vdTcpRead(VDSOCKET Sock, void *pvBuffer, size_t cbBuffer, size_t *pcbRead)
{
    PVDSOCKETINT pSocketInt = (PVDSOCKETINT)Sock;
    return RTTcpRead(pSocketInt->hSocket, pvBuffer, cbBuffer, pcbRead);
}

DECLCALLBACK(int) Medium::vdTcpWrite(VDSOCKET Sock, const void *pvBuffer, size_t cbBuffer)
{
    PVDSOCKETINT pSocketInt = (PVDSOCKETINT)Sock;
    return RTTcpWrite(pSocketInt->hSocket, pvBuffer, cbBuffer);
}

DECLCALLBACK(int) Medium::vdTcpSgWrite(VDSOCKET Sock, PCRTSGBUF pSgBuf)
{
    PVDSOCKETINT pSocketInt = (PVDSOCKETINT)Sock;
    return RTTcpSgWrite(pSocketInt->hSocket, pSgBuf);
}

DECLCALLBACK(int) Medium::vdTcpFlush(VDSOCKET Sock)
{
    PVDSOCKETINT pSocketInt = (PVDSOCKETINT)Sock;
    return RTTcpFlush(pSocketInt->hSocket);
}

DECLCALLBACK(int) Medium::vdTcpSetSendCoalescing(VDSOCKET Sock, bool fEnable)
{
    PVDSOCKETINT pSocketInt = (PVDSOCKETINT)Sock;
    return RTTcpSetSendCoalescing(pSocketInt->hSocket, fEnable);
}

DECLCALLBACK(int) Medium::vdTcpGetLocalAddress(VDSOCKET Sock, PRTNETADDR pAddr)
{
    PVDSOCKETINT pSocketInt = (PVDSOCKETINT)Sock;
    return RTTcpGetLocalAddress(pSocketInt->hSocket, pAddr);
}

DECLCALLBACK(int) Medium::vdTcpGetPeerAddress(VDSOCKET Sock, PRTNETADDR pAddr)
{
    PVDSOCKETINT pSocketInt = (PVDSOCKETINT)Sock;
    return RTTcpGetPeerAddress(pSocketInt->hSocket, pAddr);
}

/**
 * Starts a new thread driven by the appropriate Medium::Task::handler() method.
 *
 * @note When the task is executed by this method, IProgress::notifyComplete()
 *       is automatically called for the progress object associated with this
 *       task when the task is finished to signal the operation completion for
 *       other threads asynchronously waiting for it.
 */
HRESULT Medium::startThread(Medium::Task *pTask)
{
#ifdef VBOX_WITH_MAIN_LOCK_VALIDATION
    /* Extreme paranoia: The calling thread should not hold the medium
     * tree lock or any medium lock. Since there is no separate lock class
     * for medium objects be even more strict: no other object locks. */
    Assert(!AutoLockHoldsLocksInClass(LOCKCLASS_LISTOFMEDIA));
    Assert(!AutoLockHoldsLocksInClass(getLockingClass()));
#endif

    /// @todo use a more descriptive task name
    int vrc = RTThreadCreate(NULL, Medium::Task::fntMediumTask, pTask,
                             0, RTTHREADTYPE_MAIN_HEAVY_WORKER, 0,
                             "Medium::Task");
    if (RT_FAILURE(vrc))
    {
        delete pTask;
        return setError(E_FAIL, "Could not create Medium::Task thread (%Rrc)\n",  vrc);
    }

    return S_OK;
}

/**
 * Runs Medium::Task::handler() on the current thread instead of creating
 * a new one.
 *
 * This call implies that it is made on another temporary thread created for
 * some asynchronous task. Avoid calling it from a normal thread since the task
 * operations are potentially lengthy and will block the calling thread in this
 * case.
 *
 * @note When the task is executed by this method, IProgress::notifyComplete()
 *       is not called for the progress object associated with this task when
 *       the task is finished. Instead, the result of the operation is returned
 *       by this method directly and it's the caller's responsibility to
 *       complete the progress object in this case.
 */
HRESULT Medium::runNow(Medium::Task *pTask)
{
#ifdef VBOX_WITH_MAIN_LOCK_VALIDATION
    /* Extreme paranoia: The calling thread should not hold the medium
     * tree lock or any medium lock. Since there is no separate lock class
     * for medium objects be even more strict: no other object locks. */
    Assert(!AutoLockHoldsLocksInClass(LOCKCLASS_LISTOFMEDIA));
    Assert(!AutoLockHoldsLocksInClass(getLockingClass()));
#endif

    /* NIL_RTTHREAD indicates synchronous call. */
    return (HRESULT)Medium::Task::fntMediumTask(NIL_RTTHREAD, pTask);
}

/**
 * Implementation code for the "create base" task.
 *
 * This only gets started from Medium::CreateBaseStorage() and always runs
 * asynchronously. As a result, we always save the VirtualBox.xml file when
 * we're done here.
 *
 * @param task
 * @return
 */
HRESULT Medium::taskCreateBaseHandler(Medium::CreateBaseTask &task)
{
    HRESULT rc = S_OK;

    /* these parameters we need after creation */
    uint64_t size = 0, logicalSize = 0;
    MediumVariant_T variant = MediumVariant_Standard;
    bool fGenerateUuid = false;

    try
    {
        AutoWriteLock thisLock(this COMMA_LOCKVAL_SRC_POS);

        /* The object may request a specific UUID (through a special form of
        * the setLocation() argument). Otherwise we have to generate it */
        Guid id = m->id;
        fGenerateUuid = id.isEmpty();
        if (fGenerateUuid)
        {
            id.create();
            /* VirtualBox::registerMedium() will need UUID */
            unconst(m->id) = id;
        }

        Utf8Str format(m->strFormat);
        Utf8Str location(m->strLocationFull);
        uint64_t capabilities = m->formatObj->getCapabilities();
        ComAssertThrow(capabilities & (  MediumFormatCapabilities_CreateFixed
                                       | MediumFormatCapabilities_CreateDynamic), E_FAIL);
        Assert(m->state == MediumState_Creating);

        PVBOXHDD hdd;
        int vrc = VDCreate(m->vdDiskIfaces, convertDeviceType(), &hdd);
        ComAssertRCThrow(vrc, E_FAIL);

        /* unlock before the potentially lengthy operation */
        thisLock.release();

        try
        {
            /* ensure the directory exists */
            if (capabilities & MediumFormatCapabilities_File)
            {
                rc = VirtualBox::ensureFilePathExists(location, !(task.mVariant & MediumVariant_NoCreateDir) /* fCreate */);
                if (FAILED(rc))
                    throw rc;
            }

            VDGEOMETRY geo = { 0, 0, 0 }; /* auto-detect */

            vrc = VDCreateBase(hdd,
                               format.c_str(),
                               location.c_str(),
                               task.mSize,
                               task.mVariant & ~MediumVariant_NoCreateDir,
                               NULL,
                               &geo,
                               &geo,
                               id.raw(),
                               VD_OPEN_FLAGS_NORMAL | m->uOpenFlagsDef,
                               m->vdImageIfaces,
                               task.mVDOperationIfaces);
            if (RT_FAILURE(vrc))
                throw setError(VBOX_E_FILE_ERROR,
                               tr("Could not create the medium storage unit '%s'%s"),
                               location.c_str(), vdError(vrc).c_str());

            size = VDGetFileSize(hdd, 0);
            logicalSize = VDGetSize(hdd, 0);
            unsigned uImageFlags;
            vrc = VDGetImageFlags(hdd, 0, &uImageFlags);
            if (RT_SUCCESS(vrc))
                variant = (MediumVariant_T)uImageFlags;
        }
        catch (HRESULT aRC) { rc = aRC; }

        VDDestroy(hdd);
    }
    catch (HRESULT aRC) { rc = aRC; }

    if (SUCCEEDED(rc))
    {
        /* register with mVirtualBox as the last step and move to
         * Created state only on success (leaving an orphan file is
         * better than breaking media registry consistency) */
        AutoWriteLock treeLock(m->pVirtualBox->getMediaTreeLockHandle() COMMA_LOCKVAL_SRC_POS);
        ComObjPtr<Medium> pMedium;
        rc = m->pVirtualBox->registerMedium(this, &pMedium, DeviceType_HardDisk);
        Assert(this == pMedium);
    }

    // re-acquire the lock before changing state
    AutoWriteLock thisLock(this COMMA_LOCKVAL_SRC_POS);

    if (SUCCEEDED(rc))
    {
        m->state = MediumState_Created;

        m->size = size;
        m->logicalSize = logicalSize;
        m->variant = variant;

        thisLock.release();
        markRegistriesModified();
        if (task.isAsync())
        {
            // in asynchronous mode, save settings now
            m->pVirtualBox->saveModifiedRegistries();
        }
    }
    else
    {
        /* back to NotCreated on failure */
        m->state = MediumState_NotCreated;

        /* reset UUID to prevent it from being reused next time */
        if (fGenerateUuid)
            unconst(m->id).clear();
    }

    return rc;
}

/**
 * Implementation code for the "create diff" task.
 *
 * This task always gets started from Medium::createDiffStorage() and can run
 * synchronously or asynchronously depending on the "wait" parameter passed to
 * that function. If we run synchronously, the caller expects the medium
 * registry modification to be set before returning; otherwise (in asynchronous
 * mode), we save the settings ourselves.
 *
 * @param task
 * @return
 */
HRESULT Medium::taskCreateDiffHandler(Medium::CreateDiffTask &task)
{
    HRESULT rcTmp = S_OK;

    const ComObjPtr<Medium> &pTarget = task.mTarget;

    uint64_t size = 0, logicalSize = 0;
    MediumVariant_T variant = MediumVariant_Standard;
    bool fGenerateUuid = false;

    try
    {
        /* Lock both in {parent,child} order. */
        AutoMultiWriteLock2 mediaLock(this, pTarget COMMA_LOCKVAL_SRC_POS);

        /* The object may request a specific UUID (through a special form of
         * the setLocation() argument). Otherwise we have to generate it */
        Guid targetId = pTarget->m->id;
        fGenerateUuid = targetId.isEmpty();
        if (fGenerateUuid)
        {
            targetId.create();
            /* VirtualBox::registerMedium() will need UUID */
            unconst(pTarget->m->id) = targetId;
        }

        Guid id = m->id;

        Utf8Str targetFormat(pTarget->m->strFormat);
        Utf8Str targetLocation(pTarget->m->strLocationFull);
        uint64_t capabilities = pTarget->m->formatObj->getCapabilities();
        ComAssertThrow(capabilities & MediumFormatCapabilities_CreateDynamic, E_FAIL);

        Assert(pTarget->m->state == MediumState_Creating);
        Assert(m->state == MediumState_LockedRead);

        PVBOXHDD hdd;
        int vrc = VDCreate(m->vdDiskIfaces, convertDeviceType(), &hdd);
        ComAssertRCThrow(vrc, E_FAIL);

        /* the two media are now protected by their non-default states;
         * unlock the media before the potentially lengthy operation */
        mediaLock.release();

        try
        {
            /* Open all media in the target chain but the last. */
            MediumLockList::Base::const_iterator targetListBegin =
                task.mpMediumLockList->GetBegin();
            MediumLockList::Base::const_iterator targetListEnd =
                task.mpMediumLockList->GetEnd();
            for (MediumLockList::Base::const_iterator it = targetListBegin;
                 it != targetListEnd;
                 ++it)
            {
                const MediumLock &mediumLock = *it;
                const ComObjPtr<Medium> &pMedium = mediumLock.GetMedium();

                AutoReadLock alock(pMedium COMMA_LOCKVAL_SRC_POS);

                /* Skip over the target diff medium */
                if (pMedium->m->state == MediumState_Creating)
                    continue;

                /* sanity check */
                Assert(pMedium->m->state == MediumState_LockedRead);

                /* Open all media in appropriate mode. */
                vrc = VDOpen(hdd,
                             pMedium->m->strFormat.c_str(),
                             pMedium->m->strLocationFull.c_str(),
                             VD_OPEN_FLAGS_READONLY | m->uOpenFlagsDef,
                             pMedium->m->vdImageIfaces);
                if (RT_FAILURE(vrc))
                    throw setError(VBOX_E_FILE_ERROR,
                                   tr("Could not open the medium storage unit '%s'%s"),
                                   pMedium->m->strLocationFull.c_str(),
                                   vdError(vrc).c_str());
            }

            /* ensure the target directory exists */
            if (capabilities & MediumFormatCapabilities_File)
            {
                HRESULT rc = VirtualBox::ensureFilePathExists(targetLocation, !(task.mVariant & MediumVariant_NoCreateDir) /* fCreate */);
                if (FAILED(rc))
                    throw rc;
            }

            vrc = VDCreateDiff(hdd,
                               targetFormat.c_str(),
                               targetLocation.c_str(),
                               (task.mVariant & ~MediumVariant_NoCreateDir) | VD_IMAGE_FLAGS_DIFF,
                               NULL,
                               targetId.raw(),
                               id.raw(),
                               VD_OPEN_FLAGS_NORMAL | m->uOpenFlagsDef,
                               pTarget->m->vdImageIfaces,
                               task.mVDOperationIfaces);
            if (RT_FAILURE(vrc))
                throw setError(VBOX_E_FILE_ERROR,
                                tr("Could not create the differencing medium storage unit '%s'%s"),
                                targetLocation.c_str(), vdError(vrc).c_str());

            size = VDGetFileSize(hdd, VD_LAST_IMAGE);
            logicalSize = VDGetSize(hdd, VD_LAST_IMAGE);
            unsigned uImageFlags;
            vrc = VDGetImageFlags(hdd, 0, &uImageFlags);
            if (RT_SUCCESS(vrc))
                variant = (MediumVariant_T)uImageFlags;
        }
        catch (HRESULT aRC) { rcTmp = aRC; }

        VDDestroy(hdd);
    }
    catch (HRESULT aRC) { rcTmp = aRC; }

    MultiResult mrc(rcTmp);

    if (SUCCEEDED(mrc))
    {
        AutoWriteLock treeLock(m->pVirtualBox->getMediaTreeLockHandle() COMMA_LOCKVAL_SRC_POS);

        Assert(pTarget->m->pParent.isNull());

        /* associate the child with the parent */
        pTarget->m->pParent = this;
        m->llChildren.push_back(pTarget);

        /** @todo r=klaus neither target nor base() are locked,
            * potential race! */
        /* diffs for immutable media are auto-reset by default */
        pTarget->m->autoReset = (getBase()->m->type == MediumType_Immutable);

        /* register with mVirtualBox as the last step and move to
         * Created state only on success (leaving an orphan file is
         * better than breaking media registry consistency) */
        ComObjPtr<Medium> pMedium;
        mrc = m->pVirtualBox->registerMedium(pTarget, &pMedium, DeviceType_HardDisk);
        Assert(pTarget == pMedium);

        if (FAILED(mrc))
            /* break the parent association on failure to register */
            deparent();
    }

    AutoMultiWriteLock2 mediaLock(this, pTarget COMMA_LOCKVAL_SRC_POS);

    if (SUCCEEDED(mrc))
    {
        pTarget->m->state = MediumState_Created;

        pTarget->m->size = size;
        pTarget->m->logicalSize = logicalSize;
        pTarget->m->variant = variant;
    }
    else
    {
        /* back to NotCreated on failure */
        pTarget->m->state = MediumState_NotCreated;

        pTarget->m->autoReset = false;

        /* reset UUID to prevent it from being reused next time */
        if (fGenerateUuid)
            unconst(pTarget->m->id).clear();
    }

    // deregister the task registered in createDiffStorage()
    Assert(m->numCreateDiffTasks != 0);
    --m->numCreateDiffTasks;

    mediaLock.release();
    markRegistriesModified();
    if (task.isAsync())
    {
        // in asynchronous mode, save settings now
        m->pVirtualBox->saveModifiedRegistries();
    }

    /* Note that in sync mode, it's the caller's responsibility to
     * unlock the medium. */

    return mrc;
}

/**
 * Implementation code for the "merge" task.
 *
 * This task always gets started from Medium::mergeTo() and can run
 * synchronously or asynchronously depending on the "wait" parameter passed to
 * that function. If we run synchronously, the caller expects the medium
 * registry modification to be set before returning; otherwise (in asynchronous
 * mode), we save the settings ourselves.
 *
 * @param task
 * @return
 */
HRESULT Medium::taskMergeHandler(Medium::MergeTask &task)
{
    HRESULT rcTmp = S_OK;

    const ComObjPtr<Medium> &pTarget = task.mTarget;

    try
    {
        PVBOXHDD hdd;
        int vrc = VDCreate(m->vdDiskIfaces, convertDeviceType(), &hdd);
        ComAssertRCThrow(vrc, E_FAIL);

        try
        {
            // Similar code appears in SessionMachine::onlineMergeMedium, so
            // if you make any changes below check whether they are applicable
            // in that context as well.

            unsigned uTargetIdx = VD_LAST_IMAGE;
            unsigned uSourceIdx = VD_LAST_IMAGE;
            /* Open all media in the chain. */
            MediumLockList::Base::iterator lockListBegin =
                task.mpMediumLockList->GetBegin();
            MediumLockList::Base::iterator lockListEnd =
                task.mpMediumLockList->GetEnd();
            unsigned i = 0;
            for (MediumLockList::Base::iterator it = lockListBegin;
                 it != lockListEnd;
                 ++it)
            {
                MediumLock &mediumLock = *it;
                const ComObjPtr<Medium> &pMedium = mediumLock.GetMedium();

                if (pMedium == this)
                    uSourceIdx = i;
                else if (pMedium == pTarget)
                    uTargetIdx = i;

                AutoReadLock alock(pMedium COMMA_LOCKVAL_SRC_POS);

                /*
                 * complex sanity (sane complexity)
                 *
                 * The current medium must be in the Deleting (medium is merged)
                 * or LockedRead (parent medium) state if it is not the target.
                 * If it is the target it must be in the LockedWrite state.
                 */
                Assert(   (   pMedium != pTarget
                           && (   pMedium->m->state == MediumState_Deleting
                               || pMedium->m->state == MediumState_LockedRead))
                       || (   pMedium == pTarget
                           && pMedium->m->state == MediumState_LockedWrite));

                /*
                 * Medium must be the target, in the LockedRead state
                 * or Deleting state where it is not allowed to be attached
                 * to a virtual machine.
                 */
                Assert(   pMedium == pTarget
                       || pMedium->m->state == MediumState_LockedRead
                       || (   pMedium->m->backRefs.size() == 0
                           && pMedium->m->state == MediumState_Deleting));
                /* The source medium must be in Deleting state. */
                Assert(  pMedium != this
                       || pMedium->m->state == MediumState_Deleting);

                unsigned uOpenFlags = VD_OPEN_FLAGS_NORMAL;

                if (   pMedium->m->state == MediumState_LockedRead
                    || pMedium->m->state == MediumState_Deleting)
                    uOpenFlags = VD_OPEN_FLAGS_READONLY;
                if (pMedium->m->type == MediumType_Shareable)
                    uOpenFlags |= VD_OPEN_FLAGS_SHAREABLE;

                /* Open the medium */
                vrc = VDOpen(hdd,
                             pMedium->m->strFormat.c_str(),
                             pMedium->m->strLocationFull.c_str(),
                             uOpenFlags | m->uOpenFlagsDef,
                             pMedium->m->vdImageIfaces);
                if (RT_FAILURE(vrc))
                    throw vrc;

                i++;
            }

            ComAssertThrow(   uSourceIdx != VD_LAST_IMAGE
                           && uTargetIdx != VD_LAST_IMAGE, E_FAIL);

            vrc = VDMerge(hdd, uSourceIdx, uTargetIdx,
                          task.mVDOperationIfaces);
            if (RT_FAILURE(vrc))
                throw vrc;

            /* update parent UUIDs */
            if (!task.mfMergeForward)
            {
                /* we need to update UUIDs of all source's children
                 * which cannot be part of the container at once so
                 * add each one in there individually */
                if (task.mChildrenToReparent.size() > 0)
                {
                    for (MediaList::const_iterator it = task.mChildrenToReparent.begin();
                         it != task.mChildrenToReparent.end();
                         ++it)
                    {
                        /* VD_OPEN_FLAGS_INFO since UUID is wrong yet */
                        vrc = VDOpen(hdd,
                                     (*it)->m->strFormat.c_str(),
                                     (*it)->m->strLocationFull.c_str(),
                                     VD_OPEN_FLAGS_INFO | m->uOpenFlagsDef,
                                     (*it)->m->vdImageIfaces);
                        if (RT_FAILURE(vrc))
                            throw vrc;

                        vrc = VDSetParentUuid(hdd, VD_LAST_IMAGE,
                                              pTarget->m->id.raw());
                        if (RT_FAILURE(vrc))
                            throw vrc;

                        vrc = VDClose(hdd, false /* fDelete */);
                        if (RT_FAILURE(vrc))
                            throw vrc;

                        (*it)->UnlockWrite(NULL);
                    }
                }
            }
        }
        catch (HRESULT aRC) { rcTmp = aRC; }
        catch (int aVRC)
        {
            rcTmp = setError(VBOX_E_FILE_ERROR,
                             tr("Could not merge the medium '%s' to '%s'%s"),
                             m->strLocationFull.c_str(),
                             pTarget->m->strLocationFull.c_str(),
                             vdError(aVRC).c_str());
        }

        VDDestroy(hdd);
    }
    catch (HRESULT aRC) { rcTmp = aRC; }

    ErrorInfoKeeper eik;
    MultiResult mrc(rcTmp);
    HRESULT rc2;

    if (SUCCEEDED(mrc))
    {
        /* all media but the target were successfully deleted by
         * VDMerge; reparent the last one and uninitialize deleted media. */

        AutoWriteLock treeLock(m->pVirtualBox->getMediaTreeLockHandle() COMMA_LOCKVAL_SRC_POS);

        if (task.mfMergeForward)
        {
            /* first, unregister the target since it may become a base
             * medium which needs re-registration */
            rc2 = m->pVirtualBox->unregisterMedium(pTarget);
            AssertComRC(rc2);

            /* then, reparent it and disconnect the deleted branch at
             * both ends (chain->parent() is source's parent) */
            pTarget->deparent();
            pTarget->m->pParent = task.mParentForTarget;
            if (pTarget->m->pParent)
            {
                pTarget->m->pParent->m->llChildren.push_back(pTarget);
                deparent();
            }

            /* then, register again */
            ComObjPtr<Medium> pMedium;
            rc2 = m->pVirtualBox->registerMedium(pTarget, &pMedium,
                                                 DeviceType_HardDisk);
            AssertComRC(rc2);
        }
        else
        {
            Assert(pTarget->getChildren().size() == 1);
            Medium *targetChild = pTarget->getChildren().front();

            /* disconnect the deleted branch at the elder end */
            targetChild->deparent();

            /* reparent source's children and disconnect the deleted
             * branch at the younger end */
            if (task.mChildrenToReparent.size() > 0)
            {
                /* obey {parent,child} lock order */
                AutoWriteLock sourceLock(this COMMA_LOCKVAL_SRC_POS);

                for (MediaList::const_iterator it = task.mChildrenToReparent.begin();
                     it != task.mChildrenToReparent.end();
                     it++)
                {
                    Medium *pMedium = *it;
                    AutoWriteLock childLock(pMedium COMMA_LOCKVAL_SRC_POS);

                    pMedium->deparent();  // removes pMedium from source
                    pMedium->setParent(pTarget);
                }
            }
        }

        /* unregister and uninitialize all media removed by the merge */
        MediumLockList::Base::iterator lockListBegin =
            task.mpMediumLockList->GetBegin();
        MediumLockList::Base::iterator lockListEnd =
            task.mpMediumLockList->GetEnd();
        for (MediumLockList::Base::iterator it = lockListBegin;
             it != lockListEnd;
             )
        {
            MediumLock &mediumLock = *it;
            /* Create a real copy of the medium pointer, as the medium
             * lock deletion below would invalidate the referenced object. */
            const ComObjPtr<Medium> pMedium = mediumLock.GetMedium();

            /* The target and all media not merged (readonly) are skipped */
            if (   pMedium == pTarget
                || pMedium->m->state == MediumState_LockedRead)
            {
                ++it;
                continue;
            }

            rc2 = pMedium->m->pVirtualBox->unregisterMedium(pMedium);
            AssertComRC(rc2);

            /* now, uninitialize the deleted medium (note that
             * due to the Deleting state, uninit() will not touch
             * the parent-child relationship so we need to
             * uninitialize each disk individually) */

            /* note that the operation initiator medium (which is
             * normally also the source medium) is a special case
             * -- there is one more caller added by Task to it which
             * we must release. Also, if we are in sync mode, the
             * caller may still hold an AutoCaller instance for it
             * and therefore we cannot uninit() it (it's therefore
             * the caller's responsibility) */
            if (pMedium == this)
            {
                Assert(getChildren().size() == 0);
                Assert(m->backRefs.size() == 0);
                task.mMediumCaller.release();
            }

            /* Delete the medium lock list entry, which also releases the
             * caller added by MergeChain before uninit() and updates the
             * iterator to point to the right place. */
            rc2 = task.mpMediumLockList->RemoveByIterator(it);
            AssertComRC(rc2);

            if (task.isAsync() || pMedium != this)
                pMedium->uninit();
        }
    }

    markRegistriesModified();
    if (task.isAsync())
    {
        // in asynchronous mode, save settings now
        eik.restore();
        m->pVirtualBox->saveModifiedRegistries();
        eik.fetch();
    }

    if (FAILED(mrc))
    {
        /* Here we come if either VDMerge() failed (in which case we
         * assume that it tried to do everything to make a further
         * retry possible -- e.g. not deleted intermediate media
         * and so on) or VirtualBox::saveRegistries() failed (where we
         * should have the original tree but with intermediate storage
         * units deleted by VDMerge()). We have to only restore states
         * (through the MergeChain dtor) unless we are run synchronously
         * in which case it's the responsibility of the caller as stated
         * in the mergeTo() docs. The latter also implies that we
         * don't own the merge chain, so release it in this case. */
        if (task.isAsync())
        {
            Assert(task.mChildrenToReparent.size() == 0);
            cancelMergeTo(task.mChildrenToReparent, task.mpMediumLockList);
        }
    }

    return mrc;
}

/**
 * Implementation code for the "clone" task.
 *
 * This only gets started from Medium::CloneTo() and always runs asynchronously.
 * As a result, we always save the VirtualBox.xml file when we're done here.
 *
 * @param task
 * @return
 */
HRESULT Medium::taskCloneHandler(Medium::CloneTask &task)
{
    HRESULT rcTmp = S_OK;

    const ComObjPtr<Medium> &pTarget = task.mTarget;
    const ComObjPtr<Medium> &pParent = task.mParent;

    bool fCreatingTarget = false;

    uint64_t size = 0, logicalSize = 0;
    MediumVariant_T variant = MediumVariant_Standard;
    bool fGenerateUuid = false;

    try
    {
        /* Lock all in {parent,child} order. The lock is also used as a
         * signal from the task initiator (which releases it only after
         * RTThreadCreate()) that we can start the job. */
        AutoMultiWriteLock3 thisLock(this, pTarget, pParent COMMA_LOCKVAL_SRC_POS);

        fCreatingTarget = pTarget->m->state == MediumState_Creating;

        /* The object may request a specific UUID (through a special form of
         * the setLocation() argument). Otherwise we have to generate it */
        Guid targetId = pTarget->m->id;
        fGenerateUuid = targetId.isEmpty();
        if (fGenerateUuid)
        {
            targetId.create();
            /* VirtualBox::registerMedium() will need UUID */
            unconst(pTarget->m->id) = targetId;
        }

        PVBOXHDD hdd;
        int vrc = VDCreate(m->vdDiskIfaces, convertDeviceType(), &hdd);
        ComAssertRCThrow(vrc, E_FAIL);

        try
        {
            /* Open all media in the source chain. */
            MediumLockList::Base::const_iterator sourceListBegin =
                task.mpSourceMediumLockList->GetBegin();
            MediumLockList::Base::const_iterator sourceListEnd =
                task.mpSourceMediumLockList->GetEnd();
            for (MediumLockList::Base::const_iterator it = sourceListBegin;
                 it != sourceListEnd;
                 ++it)
            {
                const MediumLock &mediumLock = *it;
                const ComObjPtr<Medium> &pMedium = mediumLock.GetMedium();
                AutoReadLock alock(pMedium COMMA_LOCKVAL_SRC_POS);

                /* sanity check */
                Assert(pMedium->m->state == MediumState_LockedRead);

                /** Open all media in read-only mode. */
                vrc = VDOpen(hdd,
                             pMedium->m->strFormat.c_str(),
                             pMedium->m->strLocationFull.c_str(),
                             VD_OPEN_FLAGS_READONLY | m->uOpenFlagsDef,
                             pMedium->m->vdImageIfaces);
                if (RT_FAILURE(vrc))
                    throw setError(VBOX_E_FILE_ERROR,
                                   tr("Could not open the medium storage unit '%s'%s"),
                                   pMedium->m->strLocationFull.c_str(),
                                   vdError(vrc).c_str());
            }

            Utf8Str targetFormat(pTarget->m->strFormat);
            Utf8Str targetLocation(pTarget->m->strLocationFull);
            uint64_t capabilities = pTarget->m->formatObj->getCapabilities();

            Assert(   pTarget->m->state == MediumState_Creating
                   || pTarget->m->state == MediumState_LockedWrite);
            Assert(m->state == MediumState_LockedRead);
            Assert(   pParent.isNull()
                   || pParent->m->state == MediumState_LockedRead);

            /* unlock before the potentially lengthy operation */
            thisLock.release();

            /* ensure the target directory exists */
            if (capabilities & MediumFormatCapabilities_File)
            {
                HRESULT rc = VirtualBox::ensureFilePathExists(targetLocation, !(task.mVariant & MediumVariant_NoCreateDir) /* fCreate */);
                if (FAILED(rc))
                    throw rc;
            }

            PVBOXHDD targetHdd;
            vrc = VDCreate(m->vdDiskIfaces, convertDeviceType(), &targetHdd);
            ComAssertRCThrow(vrc, E_FAIL);

            try
            {
                /* Open all media in the target chain. */
                MediumLockList::Base::const_iterator targetListBegin =
                    task.mpTargetMediumLockList->GetBegin();
                MediumLockList::Base::const_iterator targetListEnd =
                    task.mpTargetMediumLockList->GetEnd();
                for (MediumLockList::Base::const_iterator it = targetListBegin;
                     it != targetListEnd;
                     ++it)
                {
                    const MediumLock &mediumLock = *it;
                    const ComObjPtr<Medium> &pMedium = mediumLock.GetMedium();

                    /* If the target medium is not created yet there's no
                     * reason to open it. */
                    if (pMedium == pTarget && fCreatingTarget)
                        continue;

                    AutoReadLock alock(pMedium COMMA_LOCKVAL_SRC_POS);

                    /* sanity check */
                    Assert(    pMedium->m->state == MediumState_LockedRead
                            || pMedium->m->state == MediumState_LockedWrite);

                    unsigned uOpenFlags = VD_OPEN_FLAGS_NORMAL;
                    if (pMedium->m->state != MediumState_LockedWrite)
                        uOpenFlags = VD_OPEN_FLAGS_READONLY;
                    if (pMedium->m->type == MediumType_Shareable)
                        uOpenFlags |= VD_OPEN_FLAGS_SHAREABLE;

                    /* Open all media in appropriate mode. */
                    vrc = VDOpen(targetHdd,
                                 pMedium->m->strFormat.c_str(),
                                 pMedium->m->strLocationFull.c_str(),
                                 uOpenFlags | m->uOpenFlagsDef,
                                 pMedium->m->vdImageIfaces);
                    if (RT_FAILURE(vrc))
                        throw setError(VBOX_E_FILE_ERROR,
                                       tr("Could not open the medium storage unit '%s'%s"),
                                       pMedium->m->strLocationFull.c_str(),
                                       vdError(vrc).c_str());
                }

                /** @todo r=klaus target isn't locked, race getting the state */
                if (task.midxSrcImageSame == UINT32_MAX)
                {
                    vrc = VDCopy(hdd,
                                 VD_LAST_IMAGE,
                                 targetHdd,
                                 targetFormat.c_str(),
                                 (fCreatingTarget) ? targetLocation.c_str() : (char *)NULL,
                                 false /* fMoveByRename */,
                                 0 /* cbSize */,
                                 task.mVariant & ~MediumVariant_NoCreateDir,
                                 targetId.raw(),
                                 VD_OPEN_FLAGS_NORMAL | m->uOpenFlagsDef,
                                 NULL /* pVDIfsOperation */,
                                 pTarget->m->vdImageIfaces,
                                 task.mVDOperationIfaces);
                }
                else
                {
                    vrc = VDCopyEx(hdd,
                                   VD_LAST_IMAGE,
                                   targetHdd,
                                   targetFormat.c_str(),
                                   (fCreatingTarget) ? targetLocation.c_str() : (char *)NULL,
                                   false /* fMoveByRename */,
                                   0 /* cbSize */,
                                   task.midxSrcImageSame,
                                   task.midxDstImageSame,
                                   task.mVariant & ~MediumVariant_NoCreateDir,
                                   targetId.raw(),
                                   VD_OPEN_FLAGS_NORMAL | m->uOpenFlagsDef,
                                   NULL /* pVDIfsOperation */,
                                   pTarget->m->vdImageIfaces,
                                   task.mVDOperationIfaces);
                }
                if (RT_FAILURE(vrc))
                    throw setError(VBOX_E_FILE_ERROR,
                                   tr("Could not create the clone medium '%s'%s"),
                                   targetLocation.c_str(), vdError(vrc).c_str());

                size = VDGetFileSize(targetHdd, VD_LAST_IMAGE);
                logicalSize = VDGetSize(targetHdd, VD_LAST_IMAGE);
                unsigned uImageFlags;
                vrc = VDGetImageFlags(targetHdd, 0, &uImageFlags);
                if (RT_SUCCESS(vrc))
                    variant = (MediumVariant_T)uImageFlags;
            }
            catch (HRESULT aRC) { rcTmp = aRC; }

            VDDestroy(targetHdd);
        }
        catch (HRESULT aRC) { rcTmp = aRC; }

        VDDestroy(hdd);
    }
    catch (HRESULT aRC) { rcTmp = aRC; }

    ErrorInfoKeeper eik;
    MultiResult mrc(rcTmp);

    /* Only do the parent changes for newly created media. */
    if (SUCCEEDED(mrc) && fCreatingTarget)
    {
        /* we set mParent & children() */
        AutoWriteLock alock2(m->pVirtualBox->getMediaTreeLockHandle() COMMA_LOCKVAL_SRC_POS);

        Assert(pTarget->m->pParent.isNull());

        if (pParent)
        {
            /* associate the clone with the parent and deassociate
             * from VirtualBox */
            pTarget->m->pParent = pParent;
            pParent->m->llChildren.push_back(pTarget);

            /* register with mVirtualBox as the last step and move to
             * Created state only on success (leaving an orphan file is
             * better than breaking media registry consistency) */
            eik.restore();
            ComObjPtr<Medium> pMedium;
            mrc = pParent->m->pVirtualBox->registerMedium(pTarget, &pMedium,
                                                          DeviceType_HardDisk);
            Assert(pTarget == pMedium);
            eik.fetch();

            if (FAILED(mrc))
                /* break parent association on failure to register */
                pTarget->deparent();     // removes target from parent
        }
        else
        {
            /* just register  */
            eik.restore();
            ComObjPtr<Medium> pMedium;
            mrc = m->pVirtualBox->registerMedium(pTarget, &pMedium,
                                                 DeviceType_HardDisk);
            Assert(pTarget == pMedium);
            eik.fetch();
        }
    }

    if (fCreatingTarget)
    {
        AutoWriteLock mLock(pTarget COMMA_LOCKVAL_SRC_POS);

        if (SUCCEEDED(mrc))
        {
            pTarget->m->state = MediumState_Created;

            pTarget->m->size = size;
            pTarget->m->logicalSize = logicalSize;
            pTarget->m->variant = variant;
        }
        else
        {
            /* back to NotCreated on failure */
            pTarget->m->state = MediumState_NotCreated;

            /* reset UUID to prevent it from being reused next time */
            if (fGenerateUuid)
                unconst(pTarget->m->id).clear();
        }
    }

    // now, at the end of this task (always asynchronous), save the settings
    if (SUCCEEDED(mrc))
    {
        // save the settings
        markRegistriesModified();
        /* collect multiple errors */
        eik.restore();
        m->pVirtualBox->saveModifiedRegistries();
        eik.fetch();
    }

    /* Everything is explicitly unlocked when the task exits,
     * as the task destruction also destroys the source chain. */

    /* Make sure the source chain is released early. It could happen
     * that we get a deadlock in Appliance::Import when Medium::Close
     * is called & the source chain is released at the same time. */
    task.mpSourceMediumLockList->Clear();

    return mrc;
}

/**
 * Implementation code for the "delete" task.
 *
 * This task always gets started from Medium::deleteStorage() and can run
 * synchronously or asynchronously depending on the "wait" parameter passed to
 * that function.
 *
 * @param task
 * @return
 */
HRESULT Medium::taskDeleteHandler(Medium::DeleteTask &task)
{
    NOREF(task);
    HRESULT rc = S_OK;

    try
    {
        /* The lock is also used as a signal from the task initiator (which
         * releases it only after RTThreadCreate()) that we can start the job */
        AutoWriteLock thisLock(this COMMA_LOCKVAL_SRC_POS);

        PVBOXHDD hdd;
        int vrc = VDCreate(m->vdDiskIfaces, convertDeviceType(), &hdd);
        ComAssertRCThrow(vrc, E_FAIL);

        Utf8Str format(m->strFormat);
        Utf8Str location(m->strLocationFull);

        /* unlock before the potentially lengthy operation */
        Assert(m->state == MediumState_Deleting);
        thisLock.release();

        try
        {
            vrc = VDOpen(hdd,
                         format.c_str(),
                         location.c_str(),
                         VD_OPEN_FLAGS_READONLY | VD_OPEN_FLAGS_INFO | m->uOpenFlagsDef,
                         m->vdImageIfaces);
            if (RT_SUCCESS(vrc))
                vrc = VDClose(hdd, true /* fDelete */);

            if (RT_FAILURE(vrc))
                throw setError(VBOX_E_FILE_ERROR,
                               tr("Could not delete the medium storage unit '%s'%s"),
                               location.c_str(), vdError(vrc).c_str());

        }
        catch (HRESULT aRC) { rc = aRC; }

        VDDestroy(hdd);
    }
    catch (HRESULT aRC) { rc = aRC; }

    AutoWriteLock thisLock(this COMMA_LOCKVAL_SRC_POS);

    /* go to the NotCreated state even on failure since the storage
     * may have been already partially deleted and cannot be used any
     * more. One will be able to manually re-open the storage if really
     * needed to re-register it. */
    m->state = MediumState_NotCreated;

    /* Reset UUID to prevent Create* from reusing it again */
    unconst(m->id).clear();

    return rc;
}

/**
 * Implementation code for the "reset" task.
 *
 * This always gets started asynchronously from Medium::Reset().
 *
 * @param task
 * @return
 */
HRESULT Medium::taskResetHandler(Medium::ResetTask &task)
{
    HRESULT rc = S_OK;

    uint64_t size = 0, logicalSize = 0;
    MediumVariant_T variant = MediumVariant_Standard;

    try
    {
        /* The lock is also used as a signal from the task initiator (which
         * releases it only after RTThreadCreate()) that we can start the job */
        AutoWriteLock thisLock(this COMMA_LOCKVAL_SRC_POS);

        /// @todo Below we use a pair of delete/create operations to reset
        /// the diff contents but the most efficient way will of course be
        /// to add a VDResetDiff() API call

        PVBOXHDD hdd;
        int vrc = VDCreate(m->vdDiskIfaces, convertDeviceType(), &hdd);
        ComAssertRCThrow(vrc, E_FAIL);

        Guid id = m->id;
        Utf8Str format(m->strFormat);
        Utf8Str location(m->strLocationFull);

        Medium *pParent = m->pParent;
        Guid parentId = pParent->m->id;
        Utf8Str parentFormat(pParent->m->strFormat);
        Utf8Str parentLocation(pParent->m->strLocationFull);

        Assert(m->state == MediumState_LockedWrite);

        /* unlock before the potentially lengthy operation */
        thisLock.release();

        try
        {
            /* Open all media in the target chain but the last. */
            MediumLockList::Base::const_iterator targetListBegin =
                task.mpMediumLockList->GetBegin();
            MediumLockList::Base::const_iterator targetListEnd =
                task.mpMediumLockList->GetEnd();
            for (MediumLockList::Base::const_iterator it = targetListBegin;
                 it != targetListEnd;
                 ++it)
            {
                const MediumLock &mediumLock = *it;
                const ComObjPtr<Medium> &pMedium = mediumLock.GetMedium();

                AutoReadLock alock(pMedium COMMA_LOCKVAL_SRC_POS);

                /* sanity check, "this" is checked above */
                Assert(   pMedium == this
                       || pMedium->m->state == MediumState_LockedRead);

                /* Open all media in appropriate mode. */
                vrc = VDOpen(hdd,
                             pMedium->m->strFormat.c_str(),
                             pMedium->m->strLocationFull.c_str(),
                             VD_OPEN_FLAGS_READONLY | m->uOpenFlagsDef,
                             pMedium->m->vdImageIfaces);
                if (RT_FAILURE(vrc))
                    throw setError(VBOX_E_FILE_ERROR,
                                   tr("Could not open the medium storage unit '%s'%s"),
                                   pMedium->m->strLocationFull.c_str(),
                                   vdError(vrc).c_str());

                /* Done when we hit the media which should be reset */
                if (pMedium == this)
                    break;
            }

            /* first, delete the storage unit */
            vrc = VDClose(hdd, true /* fDelete */);
            if (RT_FAILURE(vrc))
                throw setError(VBOX_E_FILE_ERROR,
                               tr("Could not delete the medium storage unit '%s'%s"),
                               location.c_str(), vdError(vrc).c_str());

            /* next, create it again */
            vrc = VDOpen(hdd,
                         parentFormat.c_str(),
                         parentLocation.c_str(),
                         VD_OPEN_FLAGS_READONLY | VD_OPEN_FLAGS_INFO | m->uOpenFlagsDef,
                         m->vdImageIfaces);
            if (RT_FAILURE(vrc))
                throw setError(VBOX_E_FILE_ERROR,
                               tr("Could not open the medium storage unit '%s'%s"),
                               parentLocation.c_str(), vdError(vrc).c_str());

            vrc = VDCreateDiff(hdd,
                               format.c_str(),
                               location.c_str(),
                               /// @todo use the same medium variant as before
                               VD_IMAGE_FLAGS_NONE,
                               NULL,
                               id.raw(),
                               parentId.raw(),
                               VD_OPEN_FLAGS_NORMAL,
                               m->vdImageIfaces,
                               task.mVDOperationIfaces);
            if (RT_FAILURE(vrc))
                throw setError(VBOX_E_FILE_ERROR,
                               tr("Could not create the differencing medium storage unit '%s'%s"),
                               location.c_str(), vdError(vrc).c_str());

            size = VDGetFileSize(hdd, VD_LAST_IMAGE);
            logicalSize = VDGetSize(hdd, VD_LAST_IMAGE);
            unsigned uImageFlags;
            vrc = VDGetImageFlags(hdd, 0, &uImageFlags);
            if (RT_SUCCESS(vrc))
                variant = (MediumVariant_T)uImageFlags;
        }
        catch (HRESULT aRC) { rc = aRC; }

        VDDestroy(hdd);
    }
    catch (HRESULT aRC) { rc = aRC; }

    AutoWriteLock thisLock(this COMMA_LOCKVAL_SRC_POS);

    m->size = size;
    m->logicalSize = logicalSize;
    m->variant = variant;

    if (task.isAsync())
    {
        /* unlock ourselves when done */
        HRESULT rc2 = UnlockWrite(NULL);
        AssertComRC(rc2);
    }

    /* Note that in sync mode, it's the caller's responsibility to
     * unlock the medium. */

    return rc;
}

/**
 * Implementation code for the "compact" task.
 *
 * @param task
 * @return
 */
HRESULT Medium::taskCompactHandler(Medium::CompactTask &task)
{
    HRESULT rc = S_OK;

    /* Lock all in {parent,child} order. The lock is also used as a
     * signal from the task initiator (which releases it only after
     * RTThreadCreate()) that we can start the job. */
    AutoWriteLock thisLock(this COMMA_LOCKVAL_SRC_POS);

    try
    {
        PVBOXHDD hdd;
        int vrc = VDCreate(m->vdDiskIfaces, convertDeviceType(), &hdd);
        ComAssertRCThrow(vrc, E_FAIL);

        try
        {
            /* Open all media in the chain. */
            MediumLockList::Base::const_iterator mediumListBegin =
                task.mpMediumLockList->GetBegin();
            MediumLockList::Base::const_iterator mediumListEnd =
                task.mpMediumLockList->GetEnd();
            MediumLockList::Base::const_iterator mediumListLast =
                mediumListEnd;
            mediumListLast--;
            for (MediumLockList::Base::const_iterator it = mediumListBegin;
                 it != mediumListEnd;
                 ++it)
            {
                const MediumLock &mediumLock = *it;
                const ComObjPtr<Medium> &pMedium = mediumLock.GetMedium();
                AutoReadLock alock(pMedium COMMA_LOCKVAL_SRC_POS);

                /* sanity check */
                if (it == mediumListLast)
                    Assert(pMedium->m->state == MediumState_LockedWrite);
                else
                    Assert(pMedium->m->state == MediumState_LockedRead);

                /* Open all media but last in read-only mode. Do not handle
                 * shareable media, as compaction and sharing are mutually
                 * exclusive. */
                vrc = VDOpen(hdd,
                             pMedium->m->strFormat.c_str(),
                             pMedium->m->strLocationFull.c_str(),
                             m->uOpenFlagsDef | (it == mediumListLast) ? VD_OPEN_FLAGS_NORMAL : VD_OPEN_FLAGS_READONLY,
                             pMedium->m->vdImageIfaces);
                if (RT_FAILURE(vrc))
                    throw setError(VBOX_E_FILE_ERROR,
                                   tr("Could not open the medium storage unit '%s'%s"),
                                   pMedium->m->strLocationFull.c_str(),
                                   vdError(vrc).c_str());
            }

            Assert(m->state == MediumState_LockedWrite);

            Utf8Str location(m->strLocationFull);

            /* unlock before the potentially lengthy operation */
            thisLock.release();

            vrc = VDCompact(hdd, VD_LAST_IMAGE, task.mVDOperationIfaces);
            if (RT_FAILURE(vrc))
            {
                if (vrc == VERR_NOT_SUPPORTED)
                    throw setError(VBOX_E_NOT_SUPPORTED,
                                   tr("Compacting is not yet supported for medium '%s'"),
                                   location.c_str());
                else if (vrc == VERR_NOT_IMPLEMENTED)
                    throw setError(E_NOTIMPL,
                                   tr("Compacting is not implemented, medium '%s'"),
                                   location.c_str());
                else
                    throw setError(VBOX_E_FILE_ERROR,
                                   tr("Could not compact medium '%s'%s"),
                                   location.c_str(),
                                   vdError(vrc).c_str());
            }
        }
        catch (HRESULT aRC) { rc = aRC; }

        VDDestroy(hdd);
    }
    catch (HRESULT aRC) { rc = aRC; }

    /* Everything is explicitly unlocked when the task exits,
     * as the task destruction also destroys the media chain. */

    return rc;
}

/**
 * Implementation code for the "resize" task.
 *
 * @param task
 * @return
 */
HRESULT Medium::taskResizeHandler(Medium::ResizeTask &task)
{
    HRESULT rc = S_OK;

    /* Lock all in {parent,child} order. The lock is also used as a
     * signal from the task initiator (which releases it only after
     * RTThreadCreate()) that we can start the job. */
    AutoWriteLock thisLock(this COMMA_LOCKVAL_SRC_POS);

    try
    {
        PVBOXHDD hdd;
        int vrc = VDCreate(m->vdDiskIfaces, convertDeviceType(), &hdd);
        ComAssertRCThrow(vrc, E_FAIL);

        try
        {
            /* Open all media in the chain. */
            MediumLockList::Base::const_iterator mediumListBegin =
                task.mpMediumLockList->GetBegin();
            MediumLockList::Base::const_iterator mediumListEnd =
                task.mpMediumLockList->GetEnd();
            MediumLockList::Base::const_iterator mediumListLast =
                mediumListEnd;
            mediumListLast--;
            for (MediumLockList::Base::const_iterator it = mediumListBegin;
                 it != mediumListEnd;
                 ++it)
            {
                const MediumLock &mediumLock = *it;
                const ComObjPtr<Medium> &pMedium = mediumLock.GetMedium();
                AutoReadLock alock(pMedium COMMA_LOCKVAL_SRC_POS);

                /* sanity check */
                if (it == mediumListLast)
                    Assert(pMedium->m->state == MediumState_LockedWrite);
                else
                    Assert(pMedium->m->state == MediumState_LockedRead);

                /* Open all media but last in read-only mode. Do not handle
                 * shareable media, as compaction and sharing are mutually
                 * exclusive. */
                vrc = VDOpen(hdd,
                             pMedium->m->strFormat.c_str(),
                             pMedium->m->strLocationFull.c_str(),
                             m->uOpenFlagsDef | (it == mediumListLast) ? VD_OPEN_FLAGS_NORMAL : VD_OPEN_FLAGS_READONLY,
                             pMedium->m->vdImageIfaces);
                if (RT_FAILURE(vrc))
                    throw setError(VBOX_E_FILE_ERROR,
                                   tr("Could not open the medium storage unit '%s'%s"),
                                   pMedium->m->strLocationFull.c_str(),
                                   vdError(vrc).c_str());
            }

            Assert(m->state == MediumState_LockedWrite);

            Utf8Str location(m->strLocationFull);

            /* unlock before the potentially lengthy operation */
            thisLock.release();

            VDGEOMETRY geo = {0, 0, 0}; /* auto */
            vrc = VDResize(hdd, task.mSize, &geo, &geo, task.mVDOperationIfaces);
            if (RT_FAILURE(vrc))
            {
                if (vrc == VERR_NOT_SUPPORTED)
                    throw setError(VBOX_E_NOT_SUPPORTED,
                                   tr("Resizing to new size %llu is not yet supported for medium '%s'"),
                                   task.mSize, location.c_str());
                else if (vrc == VERR_NOT_IMPLEMENTED)
                    throw setError(E_NOTIMPL,
                                   tr("Resiting is not implemented, medium '%s'"),
                                   location.c_str());
                else
                    throw setError(VBOX_E_FILE_ERROR,
                                   tr("Could not resize medium '%s'%s"),
                                   location.c_str(),
                                   vdError(vrc).c_str());
            }
        }
        catch (HRESULT aRC) { rc = aRC; }

        VDDestroy(hdd);
    }
    catch (HRESULT aRC) { rc = aRC; }

    /* Everything is explicitly unlocked when the task exits,
     * as the task destruction also destroys the media chain. */

    return rc;
}

/**
 * Implementation code for the "export" task.
 *
 * This only gets started from Medium::exportFile() and always runs
 * asynchronously. It doesn't touch anything configuration related, so
 * we never save the VirtualBox.xml file here.
 *
 * @param task
 * @return
 */
HRESULT Medium::taskExportHandler(Medium::ExportTask &task)
{
    HRESULT rc = S_OK;

    try
    {
        /* Lock all in {parent,child} order. The lock is also used as a
         * signal from the task initiator (which releases it only after
         * RTThreadCreate()) that we can start the job. */
        AutoWriteLock thisLock(this COMMA_LOCKVAL_SRC_POS);

        PVBOXHDD hdd;
        int vrc = VDCreate(m->vdDiskIfaces, convertDeviceType(), &hdd);
        ComAssertRCThrow(vrc, E_FAIL);

        try
        {
            /* Open all media in the source chain. */
            MediumLockList::Base::const_iterator sourceListBegin =
                task.mpSourceMediumLockList->GetBegin();
            MediumLockList::Base::const_iterator sourceListEnd =
                task.mpSourceMediumLockList->GetEnd();
            for (MediumLockList::Base::const_iterator it = sourceListBegin;
                 it != sourceListEnd;
                 ++it)
            {
                const MediumLock &mediumLock = *it;
                const ComObjPtr<Medium> &pMedium = mediumLock.GetMedium();
                AutoReadLock alock(pMedium COMMA_LOCKVAL_SRC_POS);

                /* sanity check */
                Assert(pMedium->m->state == MediumState_LockedRead);

                /* Open all media in read-only mode. */
                vrc = VDOpen(hdd,
                             pMedium->m->strFormat.c_str(),
                             pMedium->m->strLocationFull.c_str(),
                             VD_OPEN_FLAGS_READONLY | m->uOpenFlagsDef,
                             pMedium->m->vdImageIfaces);
                if (RT_FAILURE(vrc))
                    throw setError(VBOX_E_FILE_ERROR,
                                   tr("Could not open the medium storage unit '%s'%s"),
                                   pMedium->m->strLocationFull.c_str(),
                                   vdError(vrc).c_str());
            }

            Utf8Str targetFormat(task.mFormat->getId());
            Utf8Str targetLocation(task.mFilename);
            uint64_t capabilities = task.mFormat->getCapabilities();

            Assert(m->state == MediumState_LockedRead);

            /* unlock before the potentially lengthy operation */
            thisLock.release();

            /* ensure the target directory exists */
            if (capabilities & MediumFormatCapabilities_File)
            {
                rc = VirtualBox::ensureFilePathExists(targetLocation, !(task.mVariant & MediumVariant_NoCreateDir) /* fCreate */);
                if (FAILED(rc))
                    throw rc;
            }

            PVBOXHDD targetHdd;
            vrc = VDCreate(m->vdDiskIfaces, convertDeviceType(), &targetHdd);
            ComAssertRCThrow(vrc, E_FAIL);

            try
            {
                vrc = VDCopy(hdd,
                             VD_LAST_IMAGE,
                             targetHdd,
                             targetFormat.c_str(),
                             targetLocation.c_str(),
                             false /* fMoveByRename */,
                             0 /* cbSize */,
                             task.mVariant & ~MediumVariant_NoCreateDir,
                             NULL /* pDstUuid */,
                             VD_OPEN_FLAGS_NORMAL | VD_OPEN_FLAGS_SEQUENTIAL,
                             NULL /* pVDIfsOperation */,
                             task.mVDImageIfaces,
                             task.mVDOperationIfaces);
                if (RT_FAILURE(vrc))
                    throw setError(VBOX_E_FILE_ERROR,
                                   tr("Could not create the clone medium '%s'%s"),
                                   targetLocation.c_str(), vdError(vrc).c_str());
            }
            catch (HRESULT aRC) { rc = aRC; }

            VDDestroy(targetHdd);
        }
        catch (HRESULT aRC) { rc = aRC; }

        VDDestroy(hdd);
    }
    catch (HRESULT aRC) { rc = aRC; }

    /* Everything is explicitly unlocked when the task exits,
     * as the task destruction also destroys the source chain. */

    /* Make sure the source chain is released early, otherwise it can
     * lead to deadlocks with concurrent IAppliance activities. */
    task.mpSourceMediumLockList->Clear();

    return rc;
}

/**
 * Implementation code for the "import" task.
 *
 * This only gets started from Medium::importFile() and always runs
 * asynchronously. It potentially touches the media registry, so we
 * always save the VirtualBox.xml file when we're done here.
 *
 * @param task
 * @return
 */
HRESULT Medium::taskImportHandler(Medium::ImportTask &task)
{
    HRESULT rcTmp = S_OK;

    const ComObjPtr<Medium> &pParent = task.mParent;

    bool fCreatingTarget = false;

    uint64_t size = 0, logicalSize = 0;
    MediumVariant_T variant = MediumVariant_Standard;
    bool fGenerateUuid = false;

    try
    {
        /* Lock all in {parent,child} order. The lock is also used as a
         * signal from the task initiator (which releases it only after
         * RTThreadCreate()) that we can start the job. */
        AutoMultiWriteLock2 thisLock(this, pParent COMMA_LOCKVAL_SRC_POS);

        fCreatingTarget = m->state == MediumState_Creating;

        /* The object may request a specific UUID (through a special form of
         * the setLocation() argument). Otherwise we have to generate it */
        Guid targetId = m->id;
        fGenerateUuid = targetId.isEmpty();
        if (fGenerateUuid)
        {
            targetId.create();
            /* VirtualBox::registerMedium() will need UUID */
            unconst(m->id) = targetId;
        }


        PVBOXHDD hdd;
        int vrc = VDCreate(m->vdDiskIfaces, convertDeviceType(), &hdd);
        ComAssertRCThrow(vrc, E_FAIL);

        try
        {
            /* Open source medium. */
            vrc = VDOpen(hdd,
                         task.mFormat->getId().c_str(),
                         task.mFilename.c_str(),
                         VD_OPEN_FLAGS_READONLY | VD_OPEN_FLAGS_SEQUENTIAL | m->uOpenFlagsDef,
                         task.mVDImageIfaces);
            if (RT_FAILURE(vrc))
                throw setError(VBOX_E_FILE_ERROR,
                               tr("Could not open the medium storage unit '%s'%s"),
                               task.mFilename.c_str(),
                               vdError(vrc).c_str());

            Utf8Str targetFormat(m->strFormat);
            Utf8Str targetLocation(m->strLocationFull);
            uint64_t capabilities = task.mFormat->getCapabilities();

            Assert(   m->state == MediumState_Creating
                   || m->state == MediumState_LockedWrite);
            Assert(   pParent.isNull()
                   || pParent->m->state == MediumState_LockedRead);

            /* unlock before the potentially lengthy operation */
            thisLock.release();

            /* ensure the target directory exists */
            if (capabilities & MediumFormatCapabilities_File)
            {
                HRESULT rc = VirtualBox::ensureFilePathExists(targetLocation, !(task.mVariant & MediumVariant_NoCreateDir) /* fCreate */);
                if (FAILED(rc))
                    throw rc;
            }

            PVBOXHDD targetHdd;
            vrc = VDCreate(m->vdDiskIfaces, convertDeviceType(), &targetHdd);
            ComAssertRCThrow(vrc, E_FAIL);

            try
            {
                /* Open all media in the target chain. */
                MediumLockList::Base::const_iterator targetListBegin =
                    task.mpTargetMediumLockList->GetBegin();
                MediumLockList::Base::const_iterator targetListEnd =
                    task.mpTargetMediumLockList->GetEnd();
                for (MediumLockList::Base::const_iterator it = targetListBegin;
                     it != targetListEnd;
                     ++it)
                {
                    const MediumLock &mediumLock = *it;
                    const ComObjPtr<Medium> &pMedium = mediumLock.GetMedium();

                    /* If the target medium is not created yet there's no
                     * reason to open it. */
                    if (pMedium == this && fCreatingTarget)
                        continue;

                    AutoReadLock alock(pMedium COMMA_LOCKVAL_SRC_POS);

                    /* sanity check */
                    Assert(    pMedium->m->state == MediumState_LockedRead
                            || pMedium->m->state == MediumState_LockedWrite);

                    unsigned uOpenFlags = VD_OPEN_FLAGS_NORMAL;
                    if (pMedium->m->state != MediumState_LockedWrite)
                        uOpenFlags = VD_OPEN_FLAGS_READONLY;
                    if (pMedium->m->type == MediumType_Shareable)
                        uOpenFlags |= VD_OPEN_FLAGS_SHAREABLE;

                    /* Open all media in appropriate mode. */
                    vrc = VDOpen(targetHdd,
                                 pMedium->m->strFormat.c_str(),
                                 pMedium->m->strLocationFull.c_str(),
                                 uOpenFlags | m->uOpenFlagsDef,
                                 pMedium->m->vdImageIfaces);
                    if (RT_FAILURE(vrc))
                        throw setError(VBOX_E_FILE_ERROR,
                                       tr("Could not open the medium storage unit '%s'%s"),
                                       pMedium->m->strLocationFull.c_str(),
                                       vdError(vrc).c_str());
                }

                /** @todo r=klaus target isn't locked, race getting the state */
                vrc = VDCopy(hdd,
                             VD_LAST_IMAGE,
                             targetHdd,
                             targetFormat.c_str(),
                             (fCreatingTarget) ? targetLocation.c_str() : (char *)NULL,
                             false /* fMoveByRename */,
                             0 /* cbSize */,
                             task.mVariant & ~MediumVariant_NoCreateDir,
                             targetId.raw(),
                             VD_OPEN_FLAGS_NORMAL,
                             NULL /* pVDIfsOperation */,
                             m->vdImageIfaces,
                             task.mVDOperationIfaces);
                if (RT_FAILURE(vrc))
                    throw setError(VBOX_E_FILE_ERROR,
                                   tr("Could not create the clone medium '%s'%s"),
                                   targetLocation.c_str(), vdError(vrc).c_str());

                size = VDGetFileSize(targetHdd, VD_LAST_IMAGE);
                logicalSize = VDGetSize(targetHdd, VD_LAST_IMAGE);
                unsigned uImageFlags;
                vrc = VDGetImageFlags(targetHdd, 0, &uImageFlags);
                if (RT_SUCCESS(vrc))
                    variant = (MediumVariant_T)uImageFlags;
            }
            catch (HRESULT aRC) { rcTmp = aRC; }

            VDDestroy(targetHdd);
        }
        catch (HRESULT aRC) { rcTmp = aRC; }

        VDDestroy(hdd);
    }
    catch (HRESULT aRC) { rcTmp = aRC; }

    ErrorInfoKeeper eik;
    MultiResult mrc(rcTmp);

    /* Only do the parent changes for newly created media. */
    if (SUCCEEDED(mrc) && fCreatingTarget)
    {
        /* we set mParent & children() */
        AutoWriteLock alock2(m->pVirtualBox->getMediaTreeLockHandle() COMMA_LOCKVAL_SRC_POS);

        Assert(m->pParent.isNull());

        if (pParent)
        {
            /* associate the clone with the parent and deassociate
             * from VirtualBox */
            m->pParent = pParent;
            pParent->m->llChildren.push_back(this);

            /* register with mVirtualBox as the last step and move to
             * Created state only on success (leaving an orphan file is
             * better than breaking media registry consistency) */
            eik.restore();
            ComObjPtr<Medium> pMedium;
            mrc = pParent->m->pVirtualBox->registerMedium(this, &pMedium,
                                                          DeviceType_HardDisk);
            Assert(this == pMedium);
            eik.fetch();

            if (FAILED(mrc))
                /* break parent association on failure to register */
                this->deparent();     // removes target from parent
        }
        else
        {
            /* just register  */
            eik.restore();
            ComObjPtr<Medium> pMedium;
            mrc = m->pVirtualBox->registerMedium(this, &pMedium, DeviceType_HardDisk);
            Assert(this == pMedium);
            eik.fetch();
        }
    }

    if (fCreatingTarget)
    {
        AutoWriteLock mLock(this COMMA_LOCKVAL_SRC_POS);

        if (SUCCEEDED(mrc))
        {
            m->state = MediumState_Created;

            m->size = size;
            m->logicalSize = logicalSize;
            m->variant = variant;
        }
        else
        {
            /* back to NotCreated on failure */
            m->state = MediumState_NotCreated;

            /* reset UUID to prevent it from being reused next time */
            if (fGenerateUuid)
                unconst(m->id).clear();
        }
    }

    // now, at the end of this task (always asynchronous), save the settings
    {
        // save the settings
        markRegistriesModified();
        /* collect multiple errors */
        eik.restore();
        m->pVirtualBox->saveModifiedRegistries();
        eik.fetch();
    }

    /* Everything is explicitly unlocked when the task exits,
     * as the task destruction also destroys the target chain. */

    /* Make sure the target chain is released early, otherwise it can
     * lead to deadlocks with concurrent IAppliance activities. */
    task.mpTargetMediumLockList->Clear();

    return mrc;
}

/* vi: set tabstop=4 shiftwidth=4 expandtab: */
