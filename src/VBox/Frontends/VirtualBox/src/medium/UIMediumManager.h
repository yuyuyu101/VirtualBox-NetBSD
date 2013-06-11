/** @file
 *
 * VBox frontends: Qt4 GUI ("VirtualBox"):
 * UIMediumManager class declaration
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

#ifndef __UIMediumManager_h__
#define __UIMediumManager_h__

/* GUI includes */
#include "UIMediumManager.gen.h"
#include "QIMainDialog.h"
#include "QIWithRetranslateUI.h"
#include "VBoxMediaComboBox.h"

/* COM includes: */
#include "CMachine.h"

/* Forward declarations: */
class MediaItem;
class VBoxProgressBar;
class UIToolBar;

class UIMediumManager : public QIWithRetranslateUI2<QIMainDialog>,
                            public Ui::UIMediumManager
{
    Q_OBJECT;

    enum TabIndex { HDTab = 0, CDTab, FDTab };
    enum ItemAction { ItemAction_Added, ItemAction_Updated, ItemAction_Removed };
    enum Action { Action_Select, Action_Edit, Action_Copy, Action_Modify, Action_Remove, Action_Release };

public:

    UIMediumManager (QWidget *aParent = NULL,
                         Qt::WindowFlags aFlags = Qt::Dialog);
    ~UIMediumManager();

    void setup (UIMediumType aType, bool aDoSelect,
                bool aRefresh = true,
                const CMachine &aSessionMachine = CMachine(),
                const QString &aSelectId = QString::null,
                bool aShowDiffs = true,
                const QStringList &aUsedMediaIds = QStringList());

    static void showModeless (QWidget *aParent = NULL, bool aRefresh = true);

    QString selectedId() const;
    QString selectedLocation() const;

    bool showDiffs() const { return mShowDiffs; };
    bool inAttachMode() const { return !mSessionMachine.isNull(); };

public slots:

    void refreshAll();

protected:

    void retranslateUi();
    virtual void closeEvent (QCloseEvent *aEvent);
    virtual bool eventFilter (QObject *aObject, QEvent *aEvent);

private slots:

    void mediumAdded (const UIMedium &aMedium);
    void mediumUpdated (const UIMedium &aMedium);
    void mediumRemoved (UIMediumType aType, const QString &aId);

    void mediumEnumStarted();
    void mediumEnumerated (const UIMedium &aMedium);
    void mediumEnumFinished (const VBoxMediaList &aList);

    void doNewMedium();
    void doAddMedium();
    void doCopyMedium();
    void doModifyMedium();
    void doRemoveMedium();
    void doReleaseMedium();

    bool releaseMediumFrom (const UIMedium &aMedium, const QString &aMachineId);

    void processCurrentChanged (int index = -1);
    void processCurrentChanged (QTreeWidgetItem *aItem, QTreeWidgetItem *aPrevItem = 0);
    void processDoubleClick (QTreeWidgetItem *aItem, int aColumn);
    void showContextMenu (const QPoint &aPos);

    void machineStateChanged(QString strId, KMachineState state);

    void makeRequestForAdjustTable();
    void performTablesAdjustment();

private:

    QTreeWidget* treeWidget (UIMediumType aType) const;
    UIMediumType currentTreeWidgetType() const;
    QTreeWidget* currentTreeWidget() const;

    QTreeWidgetItem* selectedItem (const QTreeWidget *aTree) const;
    MediaItem* toMediaItem (QTreeWidgetItem *aItem) const;

    void setCurrentItem (QTreeWidget *aTree, QTreeWidgetItem *aItem);

    void addMediumToList (const QString &aLocation, UIMediumType aType);

    MediaItem* createHardDiskItem (QTreeWidget *aTree, const UIMedium &aMedium) const;

    void updateTabIcons (MediaItem *aItem, ItemAction aAction);

    MediaItem* searchItem (QTreeWidget *aTree, const QString &aId) const;

    bool checkMediumFor (MediaItem *aItem, Action aAction);

    bool checkDndUrls (const QList<QUrl> &aUrls) const;
    void addDndUrls (const QList<QUrl> &aUrls);

    void clearInfoPanes();
    void prepareToRefresh (int aTotal = 0);

    QString formatPaneText (const QString &aText, bool aCompact = true, const QString &aElipsis = "middle");

    /* Private member vars */
    /* Window status */
    bool mDoSelect;
    static UIMediumManager *mModelessDialog;
    VBoxProgressBar *mProgressBar;

    /* The global VirtualBox instance */
    CVirtualBox mVBox;

    /* Type if we are in the select modus */
    int mType;

    bool mShowDiffs : 1;
    bool mSetupMode : 1;

    /* Icon definitions */
    QIcon mHardDiskIcon;
    QIcon mDVDImageIcon;
    QIcon mFloppyImageIcon;

    /* Menu & Toolbar */
    QMenu       *mActionsContextMenu;
    QMenu       *mActionsMenu;
    UIToolBar *mToolBar;
    QAction     *mNewAction;
    QAction     *mAddAction;
    QAction     *mCopyAction;
    QAction     *mModifyAction;
    QAction     *mRemoveAction;
    QAction     *mReleaseAction;
    QAction     *mRefreshAction;

    /* Machine */
    CMachine mSessionMachine;
    QString  mSessionMachineId;
    bool mHardDisksInaccessible;
    bool mDVDImagesInaccessible;
    bool mFloppyImagesInaccessible;
    QString mHDSelectedId;
    QString mCDSelectedId;
    QString mFDSelectedId;
    QStringList mUsedMediaIds;
};

#endif /* __UIMediumManager_h__ */

