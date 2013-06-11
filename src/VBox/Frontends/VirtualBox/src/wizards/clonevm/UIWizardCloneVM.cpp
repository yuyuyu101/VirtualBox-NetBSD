/* $Id: UIWizardCloneVM.cpp $ */
/** @file
 *
 * VBox frontends: Qt4 GUI ("VirtualBox"):
 * UIWizardCloneVM class implementation
 */

/*
 * Copyright (C) 2011-2013 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

/* GUI includes: */
#include "UIWizardCloneVM.h"
#include "UIWizardCloneVMPageBasic1.h"
#include "UIWizardCloneVMPageBasic2.h"
#include "UIWizardCloneVMPageBasic3.h"
#include "UIWizardCloneVMPageExpert.h"
#include "VBoxGlobal.h"
#include "UIMessageCenter.h"

/* COM includes: */
#include "CConsole.h"

UIWizardCloneVM::UIWizardCloneVM(QWidget *pParent, const CMachine &machine, CSnapshot snapshot /* = CSnapshot() */)
    : UIWizard(pParent, UIWizardType_CloneVM)
    , m_machine(machine)
    , m_snapshot(snapshot)
{
#ifndef Q_WS_MAC
    /* Assign watermark: */
    assignWatermark(":/vmw_clone.png");
#else /* Q_WS_MAC */
    /* Assign background image: */
    assignBackground(":/vmw_clone_bg.png");
#endif /* Q_WS_MAC */
}

bool UIWizardCloneVM::cloneVM()
{
    /* Get clone name: */
    QString strName = field("cloneName").toString();
    /* Should we reinit mac status? */
    bool fReinitMACs = field("reinitMACs").toBool();
    /* Should we create linked clone? */
    bool fLinked = field("linkedClone").toBool();
    /* Get clone mode: */
    KCloneMode cloneMode = (mode() == UIWizardMode_Basic && page(Page3)) ||
                           (mode() == UIWizardMode_Expert && page(PageExpert)) ?
                           field("cloneMode").value<KCloneMode>() : KCloneMode_MachineState;

    /* Get VBox object: */
    CVirtualBox vbox = vboxGlobal().virtualBox();

    /* Prepare machine for cloning: */
    CMachine srcMachine = m_machine;

    /* If the user like to create a linked clone from the current machine, we
     * have to take a little bit more action. First we create an snapshot, so
     * that new differencing images on the source VM are created. Based on that
     * we could use the new snapshot machine for cloning. */
    if (fLinked && m_snapshot.isNull())
    {
        /* Open session: */
        CSession session = vboxGlobal().openSession(m_machine.GetId());
        if (session.isNull())
            return false;

        /* Prepare console: */
        CConsole console = session.GetConsole();

        /* Take the snapshot: */
        QString strSnapshotName = tr("Linked Base for %1 and %2").arg(m_machine.GetName()).arg(strName);
        CProgress progress = console.TakeSnapshot(strSnapshotName, "");

        if (console.isOk())
        {
            /* Show the "Taking Snapshot" progress dialog: */
            msgCenter().showModalProgressDialog(progress, m_machine.GetName(), ":/progress_snapshot_create_90px.png", this, true);

            if (!progress.isOk() || progress.GetResultCode() != 0)
            {
                msgCenter().cannotTakeSnapshot(progress);
                return false;
            }
        }
        else
        {
            msgCenter().cannotTakeSnapshot(console);
            return false;
        }

        /* Unlock machine finally: */
        session.UnlockMachine();

        /* Get the new snapshot and the snapshot machine. */
        const CSnapshot &newSnapshot = m_machine.FindSnapshot(strSnapshotName);
        if (newSnapshot.isNull())
        {
            msgCenter().cannotFindSnapshotByName(this, m_machine, strSnapshotName);
            return false;
        }
        srcMachine = newSnapshot.GetMachine();
    }

    /* Create a new machine object. */
    const QString &strSettingsFile = vbox.ComposeMachineFilename(strName, QString::null /**< @todo group support */, QString::null, QString::null);
    CMachine cloneMachine = vbox.CreateMachine(strSettingsFile, strName, QVector<QString>(), QString::null, QString::null);
    if (!vbox.isOk())
    {
        msgCenter().cannotCreateMachine(vbox, this);
        return false;
    }

    /* Add the keep all MACs option to the import settings when requested. */
    QVector<KCloneOptions> options;
    if (!fReinitMACs)
        options.append(KCloneOptions_KeepAllMACs);
    /* Linked clones requested? */
    if (fLinked)
        options.append(KCloneOptions_Link);

    /* Start cloning. */
    CProgress progress = srcMachine.CloneTo(cloneMachine, cloneMode, options);
    if (!srcMachine.isOk())
    {
        msgCenter().cannotCreateClone(srcMachine, this);
        return false;
    }

    /* Wait until done. */
    msgCenter().showModalProgressDialog(progress, windowTitle(), ":/progress_clone_90px.png", this, true);
    if (progress.GetCanceled())
        return false;
    if (!progress.isOk() || progress.GetResultCode() != 0)
    {
        msgCenter().cannotCreateClone(srcMachine, progress, this);
        return false;
    }

    /* Finally register the clone machine. */
    vbox.RegisterMachine(cloneMachine);
    if (!vbox.isOk())
    {
        msgCenter().cannotRegisterMachine(vbox, cloneMachine, this);
        return false;
    }

    return true;
}

void UIWizardCloneVM::retranslateUi()
{
    /* Call to base-class: */
    UIWizard::retranslateUi();

    /* Translate wizard: */
    setWindowTitle(tr("Clone Virtual Machine"));
    setButtonText(QWizard::FinishButton, tr("Clone"));
}

void UIWizardCloneVM::prepare()
{
    /* Create corresponding pages: */
    switch (mode())
    {
        case UIWizardMode_Basic:
        {
            setPage(Page1, new UIWizardCloneVMPageBasic1(m_machine.GetName()));
            setPage(Page2, new UIWizardCloneVMPageBasic2(m_snapshot.isNull()));
            if (m_machine.GetSnapshotCount() > 0)
                setPage(Page3, new UIWizardCloneVMPageBasic3(m_snapshot.isNull() ? false : m_snapshot.GetChildrenCount() > 0));
            break;
        }
        case UIWizardMode_Expert:
        {
            setPage(PageExpert, new UIWizardCloneVMPageExpert(m_machine.GetName(),
                                                              m_snapshot.isNull(),
                                                              m_snapshot.isNull() ? false : m_snapshot.GetChildrenCount() > 0));
            break;
        }
        default:
        {
            AssertMsgFailed(("Invalid mode: %d", mode()));
            break;
        }
    }
    /* Call to base-class: */
    UIWizard::prepare();
}

