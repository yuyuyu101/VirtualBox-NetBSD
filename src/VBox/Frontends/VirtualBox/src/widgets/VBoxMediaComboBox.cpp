/* $Id: VBoxMediaComboBox.cpp $ */
/** @file
 *
 * VBox frontends: Qt GUI ("VirtualBox"):
 * VBoxMediaComboBox class implementation
 */

/*
 * Copyright (C) 2006-2012 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#include "VBoxMediaComboBox.h"

#include <QFileInfo>
#include <QDir>
#include <QAbstractItemView>

VBoxMediaComboBox::VBoxMediaComboBox (QWidget *aParent)
    : QComboBox (aParent)
    , mType (UIMediumType_Invalid)
    , mLastId (QString::null)
    , mShowDiffs (false)
    , mShowNullItem (false)
    , mMachineId (QString::null)
{
    /* Setup the elide mode */
    view()->setTextElideMode (Qt::ElideRight);
    QSizePolicy sp1 (QSizePolicy::Ignored, QSizePolicy::Fixed, QSizePolicy::ComboBox);
    sp1.setHorizontalStretch (2);
    setSizePolicy (sp1);

    /* Setup enumeration handlers */
    connect (&vboxGlobal(), SIGNAL (mediumEnumStarted()),
             this, SLOT (mediumEnumStarted()));
    connect (&vboxGlobal(), SIGNAL (mediumEnumerated (const UIMedium &)),
             this, SLOT (mediumEnumerated (const UIMedium &)));

    /* Setup update handlers */
    connect (&vboxGlobal(), SIGNAL (mediumAdded (const UIMedium &)),
             this, SLOT (mediumAdded (const UIMedium &)));
    connect (&vboxGlobal(), SIGNAL (mediumUpdated (const UIMedium &)),
             this, SLOT (mediumUpdated (const UIMedium &)));
    connect (&vboxGlobal(), SIGNAL (mediumRemoved (UIMediumType, const QString &)),
             this, SLOT (mediumRemoved (UIMediumType, const QString &)));

    /* Setup other connections */
    connect (this, SIGNAL (activated (int)),
             this, SLOT (processActivated (int)));
    connect (view(), SIGNAL (entered (const QModelIndex&)),
             this, SLOT (processOnItem (const QModelIndex&)));
}

/**
 * Fills this combobox with the current media list.
 */
void VBoxMediaComboBox::refresh()
{
    /* Clearing lists */
    clear(), mMedia.clear();

    VBoxMediaList list (vboxGlobal().currentMediaList());
    foreach (UIMedium medium, list)
        mediumAdded (medium);

    /* If at least one real medium present, process null medium */
    if (count() > 1 && (!mShowNullItem || mType == UIMediumType_HardDisk))
    {
        removeItem (0);
        mMedia.erase (mMedia.begin());
    }

    /* Inform the interested parties about the possibly changed active item's
     * icon, text, tooltip etc. */
    emit activated (currentIndex());
}

/**
 * Requests the global media list enumeration and repopulates the list with its
 * results.
 */
void VBoxMediaComboBox::repopulate()
{
    if (!vboxGlobal().isMediaEnumerationStarted())
        vboxGlobal().startEnumeratingMedia();
    else
        refresh();
}

QString VBoxMediaComboBox::id (int aIndex /*= -1*/) const
{
    AssertReturn (aIndex == -1 ||
                  (aIndex >= 0 && aIndex < mMedia.size()),
                  QString::null);

    if (aIndex == -1) aIndex = currentIndex();
    return aIndex == -1 ? QString::null : mMedia [aIndex].id;
}

QString VBoxMediaComboBox::location (int aIndex /*= -1*/) const
{
    AssertReturn (aIndex == -1 ||
                  (aIndex >= 0 && aIndex < mMedia.size()),
                  QString::null);

    if (aIndex == -1) aIndex = currentIndex();
    return aIndex == -1 ? QString::null : mMedia [aIndex].location;
}

void VBoxMediaComboBox::setCurrentItem (const QString &aId)
{
    mLastId = aId;

    int index;
    if (findMediaIndex (aId, index))
    {
        /* Note that the media combobox may be not populated here yet, so we
         * don't assert */
        QComboBox::setCurrentIndex (index);
        emit activated (index);
    }
}

void VBoxMediaComboBox::setType (UIMediumType aType)
{
    mType = aType;
}

void VBoxMediaComboBox::setMachineId (const QString &aMachineId)
{
    mMachineId = aMachineId;
}

void VBoxMediaComboBox::setNullItemPresent (bool aNullItemPresent)
{
    mShowNullItem = aNullItemPresent;
}

/**
 * Enables or disables the "show diffs" mode.
 *
 * In disabled "show diffs" mode, this combobox will only include base hard
 * disks plus differencing hard disks that are attached to the associated
 * machine passed to the constructor in the current state (if any). Note
 * that for these differencing hard disks, the details of their base hard disks
 * are shown instead of their own details (human-friendly mode).
 *
 * In enabled "show diffs" mode, all hard disks, base and differencing, are
 * shown.
 *
 * Note that you must call #refresh() in order for this change to take effect.
 *
 * @param aShowDiffs    @c true to enable "show diffs" mode.
 */
void VBoxMediaComboBox::setShowDiffs (bool aShowDiffs)
{
    AssertReturnVoid (aShowDiffs == true || !mMachineId.isNull());

    mShowDiffs = aShowDiffs;
}


void VBoxMediaComboBox::mediumEnumStarted()
{
    refresh();
}

void VBoxMediaComboBox::mediumEnumerated (const UIMedium &aMedium)
{
    mediumUpdated (aMedium);
}

void VBoxMediaComboBox::mediumAdded (const UIMedium &aMedium)
{
    if (aMedium.isNull() || aMedium.type() == mType)
    {
        if (!mShowDiffs && aMedium.type() == UIMediumType_HardDisk)
        {
            if (aMedium.parent() != NULL)
            {
                /* In !mShowDiffs mode, we ignore all diffs except ones that are
                 * directly attached to the related VM in the current state */
                if (!aMedium.isAttachedInCurStateTo (mMachineId))
                    return;
            }
        }

        appendItem (aMedium);

        /* Activate the required item if there is any */
        if (aMedium.id() == mLastId)
            setCurrentItem (aMedium.id());
        /* Select last added item if there is no item selected */
        else if (currentText().isEmpty())
            QComboBox::setCurrentIndex (count() - 1);
    }
}

void VBoxMediaComboBox::mediumUpdated (const UIMedium &aMedium)
{
    if (aMedium.isNull() || aMedium.type() == mType)
    {
        int index;
        if (!findMediaIndex (aMedium.id(), index))
            return;

        replaceItem (index, aMedium);

        /* Emit the signal to ensure the parent dialog handles the change of
         * the selected item's data */
        emit activated (currentIndex());
    }
}

void VBoxMediaComboBox::mediumRemoved (UIMediumType aType,
                                       const QString &aId)
{
    if (mType != aType)
        return;

    int index;
    if (!findMediaIndex (aId, index))
        return;

    removeItem (index);
    mMedia.erase (mMedia.begin() + index);

    /* If no real medium left, add the null medium */
    if (count() == 0)
        mediumAdded (UIMedium());

    /* Emit the signal to ensure the parent dialog handles the change of
     * the selected item */
    emit activated (currentIndex());
}


void VBoxMediaComboBox::processActivated (int aIndex)
{
    AssertReturnVoid (aIndex >= 0 && aIndex < mMedia.size());

    mLastId = mMedia [aIndex].id;

    updateToolTip (aIndex);
}

void VBoxMediaComboBox::updateToolTip (int aIndex)
{
    /* Set the combobox tooltip */
    setToolTip (QString::null);
    if (aIndex >= 0 && aIndex < mMedia.size())
        setToolTip (mMedia [aIndex].toolTip);
}

void VBoxMediaComboBox::processOnItem (const QModelIndex &aIndex)
{
    /* Set the combobox item's tooltip */
    int index = aIndex.row();
    view()->viewport()->setToolTip (QString::null);
    view()->viewport()->setToolTip (mMedia [index].toolTip);
}


void VBoxMediaComboBox::appendItem (const UIMedium &aMedium)
{
    if (!mShowDiffs && aMedium.parent() != NULL)
    {
        /* We are adding the direct machine diff in !mShowDiffs mode. Since its
         * base hard disk has been already appended (enumerated before), we want
         * to replace the base with the diff to avoid showing both (both would
         * be labeled using the base filename and therefore look like
         * duplicates). Note though that these visual duplicates are still
         * possible in !mShowDiffs mode if the same base hard disk is attached*
         * to the VM through different diffs (this is why we don't assert
         * below on findMediaIndex() == true). However, this situation is
         * unavoidable so we accept it assuming that the user will switch to
           mShowDiffs mode if he needs clarity. */
        int index;
        if (findMediaIndex (aMedium.root().id(), index))
        {
            replaceItem (index, aMedium);
            return;
        }
    }

    mMedia.append (Medium (aMedium.id(), aMedium.location(),
                           aMedium.toolTipCheckRO (!mShowDiffs, mShowNullItem && mType != UIMediumType_HardDisk)));

    insertItem (count(), aMedium.iconCheckRO (!mShowDiffs),
                aMedium.details (!mShowDiffs));
}

void VBoxMediaComboBox::replaceItem (int aIndex, const UIMedium &aMedium)
{
    AssertReturnVoid (aIndex >= 0 && aIndex < mMedia.size());

    mMedia [aIndex].id = aMedium.id();
    mMedia [aIndex].location = aMedium.location();
    mMedia [aIndex].toolTip = aMedium.toolTipCheckRO (!mShowDiffs, mShowNullItem && mType != UIMediumType_HardDisk);

    setItemText (aIndex, aMedium.details (!mShowDiffs));
    setItemIcon (aIndex, aMedium.iconCheckRO (!mShowDiffs));

    if (aIndex == currentIndex())
        updateToolTip (aIndex);
}

/**
 * Searches for a medium with the given ID in mMedia and stores its index in @a
 * aIndex. Returns @c true if the media was found and @c false otherwise (@a
 * aIndex will be set to a value >= mMedia.size() in this case).
 *
 * @param aId       Media ID to search for.
 * @param aIndex    Where to store the found media index.
 */
bool VBoxMediaComboBox::findMediaIndex (const QString &aId, int &aIndex)
{
    aIndex = 0;

    for (; aIndex < mMedia.size(); ++ aIndex)
        if (mMedia [aIndex].id == aId)
            break;

    return aIndex < mMedia.size();
}

