/** @file
 *
 * VBox frontends: Qt GUI ("VirtualBox"):
 * UIMedium class declaration
 */

/*
 * Copyright (C) 2009-2012 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#ifndef __UIMedium_h__
#define __UIMedium_h__

/* Qt includes: */
#include <QPixmap>
#include <QLinkedList>

/* GUI includes: */
#include "UIMediumDefs.h"

/* COM includes: */
#include "COMEnums.h"
#include "CMedium.h"

/* Other VBox includes: */
#include "iprt/cpp/utils.h"

/**
 * Cache used to override some attributes in the user-friendly "don't show diffs" mode.
 */
struct NoDiffsCache
{
    NoDiffsCache() : isSet (false), state (KMediumState_NotCreated) {}
    NoDiffsCache& operator= (const NoDiffsCache &aOther)
    {
        isSet = aOther.isSet;
        state = aOther.state;
        result = aOther.result;
        toolTip = aOther.toolTip;
        return *this;
    }

    bool isSet : 1;

    KMediumState state;
    COMResult result;
    QString toolTip;
};

/**
 * Media descriptor for the GUI.
 *
 * Maintains the results of the last state (accessibility) check and precomposes
 * string parameters such as location, size which can be used in various GUI
 * controls.
 *
 * Many getter methods take the boolean @a aNoDiffs argument. Unless explicitly
 * stated otherwise, this argument, when set to @c true, will cause the
 * corresponding property of this object's root medium to be returned instead of
 * its own one. This is useful when hard disk media is represented in the
 * user-friendly "don't show diffs" mode. For non-hard disk media, the value of
 * this argument is irrelevant because the root object for such medium is
 * the medium itself.
 *
 * Note that this class "abuses" the KMediumState_NotCreated state value to
 * indicate that the accessibility check of the given medium (see
 * #blockAndQueryState()) has not been done yet and therefore some parameters
 * such as #size() are meaningless because they can be read only from the
 * accessible medium. The real KMediumState_NotCreated state is not necessary
 * because this class is only used with created (existing) media.
 */
class UIMedium
{
public:

    /**
     * Creates a null medium descriptor which is not associated with any medium.
     * The state field is set to KMediumState_NotCreated.
     */
    UIMedium()
        : mType (UIMediumType_Invalid)
        , mState (KMediumState_NotCreated)
        , mIsReadOnly (false)
        , mIsUsedInSnapshots (false)
        , mParent (0) { refresh(); }

    /**
     * Creates a media descriptor associated with the given medium.
     *
     * The state field remain KMediumState_NotCreated until #blockAndQueryState()
     * is called. All precomposed strings are filled up by implicitly calling
     * #refresh(), see the #refresh() details for more info.
     *
     * One of the hardDisk, dvdImage, or floppyImage members is assigned from
     * aMedium according to aType. @a aParent must be always NULL for non-hard
     * disk media.
     */
    UIMedium (const CMedium &aMedium, UIMediumType aType, UIMedium *aParent = 0)
        : mMedium (aMedium)
        , mType (aType)
        , mState (KMediumState_NotCreated)
        , mIsReadOnly (false)
        , mIsUsedInSnapshots (false)
        , mParent (aParent) { refresh(); }

    /**
     * Similar to the other non-null constructor but sets the media state to
     * @a aState. Suitable when the media state is known such as right after
     * creation.
     */
    UIMedium (const CMedium &aMedium, UIMediumType aType, KMediumState aState)
        : mMedium (aMedium)
        , mType (aType)
        , mState (aState)
        , mIsReadOnly (false)
        , mIsUsedInSnapshots (false)
        , mParent (0) { refresh(); }

    UIMedium& operator= (const UIMedium &aOther);

    void blockAndQueryState();
    void refresh();

    const CMedium &medium() const { return mMedium; }

    UIMediumType type() const { return mType; }

    /**
     * Media state. In "don't show diffs" mode, this is the worst state (in
     * terms of inaccessibility) detected on the given hard disk chain.
     *
     * @param aNoDiffs  @c true to enable user-friendly "don't show diffs" mode.
     */
    KMediumState state (bool aNoDiffs = false) const
    {
        unconst (this)->checkNoDiffs (aNoDiffs);
        return aNoDiffs ? mNoDiffs.state : mState;
    }

    QString lastAccessError() const { return mLastAccessError; }

    /**
     * Result of the last blockAndQueryState() call. Will indicate an error and
     * contain a proper error info if the last state check fails. In "don't show
     * diffs" mode, this is the worst result (in terms of inaccessibility)
     * detected on the given hard disk chain.
     *
     * @param aNoDiffs  @c true to enable user-friendly "don't show diffs" mode.
     */
    const COMResult &result (bool aNoDiffs = false) const
    {
        unconst (this)->checkNoDiffs (aNoDiffs);
        return aNoDiffs ? mNoDiffs.result : mResult;
    }

    QString id() const { return mId; }
    QString name (bool aNoDiffs = false) const { return aNoDiffs ? root().mName : mName; }
    QString location (bool aNoDiffs = false) const { return aNoDiffs ? root().mLocation : mLocation; }

    QString size (bool aNoDiffs = false) const { return aNoDiffs ? root().mSize : mSize; }
    QString logicalSize (bool aNoDiffs = false) const { return aNoDiffs ? root().mLogicalSize : mLogicalSize; }

    QString hardDiskFormat (bool aNoDiffs = false) const { return aNoDiffs ? root().mHardDiskFormat : mHardDiskFormat; }
    QString hardDiskType (bool aNoDiffs = false) const { return aNoDiffs ? root().mHardDiskType : mHardDiskType; }

    QString storageDetails() const { return mStorageDetails; }

    QString usage (bool aNoDiffs = false) const { return aNoDiffs ? root().mUsage : mUsage; }
    QString tip() const { return mToolTip; }

    const NoDiffsCache& cache() const { return mNoDiffs; }

    /**
     * Returns @c true if this medium is read-only (either because it is
     * Immutable or because it has child hard disks). Read-only media can only
     * be attached indirectly.
     */
    bool isReadOnly() const { return mIsReadOnly; }

    /**
     * Returns @c true if this medium is attached to any VM (in the current
     * state or in a snapshot) in which case #usage() will contain a string with
     * comma-separated VM names (with snapshot names, if any, in parenthesis).
     */
    bool isUsed() const { return !mUsage.isNull(); }

    /**
     * Returns @c true if this medium is attached to any VM in any snapshot.
     */
    bool isUsedInSnapshots() const { return mIsUsedInSnapshots; }

    /**
     * Returns @c true if this medium corresponds to real host drive.
     */
    bool isHostDrive() const { return mIsHostDrive; }

    /**
     * Returns @c true if this medium is attached to the given machine in the current state.
     */
    bool isAttachedInCurStateTo (const QString &aMachineId) const { return mCurStateMachineIds.indexOf (aMachineId) >= 0; }

    /**
     * Returns a vector of IDs of all machines this medium is attached
     * to in their current state (i.e. excluding snapshots).
     */
    const QList <QString> &curStateMachineIds() const { return mCurStateMachineIds; }

    /**
     * Returns a parent medium. For non-hard disk media, this is always NULL.
     */
    UIMedium* parent() const { return mParent; }

    UIMedium& root() const;

    QString toolTip (bool aNoDiffs = false, bool aCheckRO = false, bool aNullAllowed = false) const;
    QPixmap icon (bool aNoDiffs = false, bool aCheckRO = false) const;

    /** Shortcut to <tt>#toolTip (aNoDiffs, true)</tt>. */
    QString toolTipCheckRO (bool aNoDiffs = false, bool aNullAllowed = false) const { return toolTip (aNoDiffs, true, aNullAllowed); }

    /** Shortcut to <tt>#icon (aNoDiffs, true)</tt>. */
    QPixmap iconCheckRO (bool aNoDiffs = false) const { return icon (aNoDiffs, true); }

    QString details (bool aNoDiffs = false, bool aPredictDiff = false, bool aUseHTML = false) const;

    /** Shortcut to <tt>#details (aNoDiffs, aPredictDiff, true)</tt>. */
    QString detailsHTML (bool aNoDiffs = false, bool aPredictDiff = false) const { return details (aNoDiffs, aPredictDiff, true); }

    /** Returns @c true if this media descriptor is a null object. */
    bool isNull() const { return mMedium.isNull(); }

private:

    void checkNoDiffs (bool aNoDiffs);

    CMedium mMedium;

    UIMediumType mType;

    KMediumState mState;
    QString mLastAccessError;
    COMResult mResult;

    QString mId;
    QString mName;
    QString mLocation;

    QString mSize;
    QString mLogicalSize;

    QString mHardDiskFormat;
    QString mHardDiskType;

    QString mStorageDetails;

    QString mUsage;
    QString mToolTip;

    bool mIsReadOnly        : 1;
    bool mIsUsedInSnapshots : 1;
    bool mIsHostDrive       : 1;

    QList <QString> mCurStateMachineIds;

    UIMedium *mParent;

    NoDiffsCache mNoDiffs;

    static QString mTable;
    static QString mRow;
};

typedef QLinkedList <UIMedium> VBoxMediaList;

#endif /* __UIMedium_h__ */

