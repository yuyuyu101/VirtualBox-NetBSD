/* $Id: UIWizardExportAppPageBasic3.cpp $ */
/** @file
 *
 * VBox frontends: Qt4 GUI ("VirtualBox"):
 * UIWizardExportAppPageBasic3 class implementation
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

/* Global includes: */
#include <QDir>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>

/* Local includes: */
#include "UIWizardExportAppPageBasic3.h"
#include "UIWizardExportApp.h"
#include "UIWizardExportAppDefs.h"
#include "VBoxGlobal.h"
#include "VBoxFilePathSelectorWidget.h"
#include "QIRichTextLabel.h"

UIWizardExportAppPage3::UIWizardExportAppPage3()
{
}

void UIWizardExportAppPage3::chooseDefaultSettings()
{
    /* Select default settings: */
#if 0
    m_pUsernameEditor->setText(vboxGlobal().virtualBox().GetExtraData(GUI_Export_Username));
    m_pHostnameEditor->setText(vboxGlobal().virtualBox().GetExtraData(GUI_Export_Hostname));
    m_pBucketEditor->setText(vboxGlobal().virtualBox().GetExtraData(GUI_Export_Bucket));
#else
    /* Do nothing for now... */
#endif
}

void UIWizardExportAppPage3::refreshCurrentSettings()
{
    /* Setup components for chosen storage-type: */
    StorageType storageType = fieldImp("storageType").value<StorageType>();
    switch (storageType)
    {
        case Filesystem:
        {
            m_pUsernameLabel->setVisible(false);
            m_pUsernameEditor->setVisible(false);
            m_pPasswordLabel->setVisible(false);
            m_pPasswordEditor->setVisible(false);
            m_pHostnameLabel->setVisible(false);
            m_pHostnameEditor->setVisible(false);
            m_pBucketLabel->setVisible(false);
            m_pBucketEditor->setVisible(false);
            m_pOVF09Checkbox->setVisible(true);
            m_pFileSelector->setChooserVisible(true);
            break;
        }
        case SunCloud:
        {
            m_pUsernameLabel->setVisible(true);
            m_pUsernameEditor->setVisible(true);
            m_pPasswordLabel->setVisible(true);
            m_pPasswordEditor->setVisible(true);
            m_pHostnameLabel->setVisible(false);
            m_pHostnameEditor->setVisible(false);
            m_pBucketLabel->setVisible(true);
            m_pBucketEditor->setVisible(true);
            m_pOVF09Checkbox->setVisible(false);
            m_pOVF09Checkbox->setChecked(false);
            m_pFileSelector->setChooserVisible(false);
            break;
        }
        case S3:
        {
            m_pUsernameLabel->setVisible(true);
            m_pUsernameEditor->setVisible(true);
            m_pPasswordLabel->setVisible(true);
            m_pPasswordEditor->setVisible(true);
            m_pHostnameLabel->setVisible(true);
            m_pHostnameEditor->setVisible(true);
            m_pBucketLabel->setVisible(true);
            m_pBucketEditor->setVisible(true);
            m_pOVF09Checkbox->setVisible(true);
            m_pFileSelector->setChooserVisible(false);
            break;
        }
    }

    /* Use the default filename: */
    QString strName = m_strDefaultApplianceName;
    /* If it is one VM only, we use the VM name as filename: */
    if (fieldImp("machineNames").toStringList().size() == 1)
        strName = fieldImp("machineNames").toStringList()[0];
    strName += ".ova";
    if (storageType == Filesystem)
        strName = QDir::toNativeSeparators(QString("%1/%2").arg(vboxGlobal().documentsPath()).arg(strName));
    m_pFileSelector->setPath(strName);
}

bool UIWizardExportAppPage3::isOVF09Selected() const
{
    return m_pOVF09Checkbox->isChecked();
}

void UIWizardExportAppPage3::setOVF09Selected(bool fChecked)
{
    m_pOVF09Checkbox->setChecked(fChecked);
}

bool UIWizardExportAppPage3::isManifestSelected() const
{
    return m_pManifestCheckbox->isChecked();
}

void UIWizardExportAppPage3::setManifestSelected(bool fChecked)
{
    m_pManifestCheckbox->setChecked(fChecked);
}

QString UIWizardExportAppPage3::username() const
{
    return m_pUsernameEditor->text();
}

void UIWizardExportAppPage3::setUserName(const QString &strUserName)
{
    m_pUsernameEditor->setText(strUserName);
}

QString UIWizardExportAppPage3::password() const
{
    return m_pPasswordEditor->text();
}

void UIWizardExportAppPage3::setPassword(const QString &strPassword)
{
    m_pPasswordEditor->setText(strPassword);
}

QString UIWizardExportAppPage3::hostname() const
{
    return m_pHostnameEditor->text();
}

void UIWizardExportAppPage3::setHostname(const QString &strHostname)
{
    m_pHostnameEditor->setText(strHostname);
}

QString UIWizardExportAppPage3::bucket() const
{
    return m_pBucketEditor->text();
}

void UIWizardExportAppPage3::setBucket(const QString &strBucket)
{
    m_pBucketEditor->setText(strBucket);
}

QString UIWizardExportAppPage3::path() const
{
    return m_pFileSelector->path();
}

void UIWizardExportAppPage3::setPath(const QString &strPath)
{
    m_pFileSelector->setPath(strPath);
}

UIWizardExportAppPageBasic3::UIWizardExportAppPageBasic3()
{
    /* Create widgets: */
    QVBoxLayout *pMainLayout = new QVBoxLayout(this);
    {
        m_pLabel = new QIRichTextLabel(this);
        QGridLayout *pSettingsLayout = new QGridLayout;
        {
            m_pUsernameEditor = new QLineEdit(this);
            m_pUsernameLabel = new QLabel(this);
            {
                m_pUsernameLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
                m_pUsernameLabel->setBuddy(m_pUsernameEditor);
            }
            m_pPasswordEditor = new QLineEdit(this);
            {
                m_pPasswordEditor->setEchoMode(QLineEdit::Password);
            }
            m_pPasswordLabel = new QLabel(this);
            {
                m_pPasswordLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
                m_pPasswordLabel->setBuddy(m_pPasswordEditor);
            }
            m_pHostnameEditor = new QLineEdit(this);
            m_pHostnameLabel = new QLabel(this);
            {
                m_pHostnameLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
                m_pHostnameLabel->setBuddy(m_pHostnameEditor);
            }
            m_pBucketEditor = new QLineEdit(this);
            m_pBucketLabel = new QLabel(this);
            {
                m_pBucketLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
                m_pBucketLabel->setBuddy(m_pBucketEditor);
            }
            m_pFileSelector = new VBoxEmptyFileSelector(this);
            {
                m_pFileSelector->setMode(VBoxFilePathSelectorWidget::Mode_File_Save);
                m_pFileSelector->setEditable(true);
                m_pFileSelector->setButtonPosition(VBoxEmptyFileSelector::RightPosition);
                m_pFileSelector->setDefaultSaveExt("ova");
            }
            m_pFileSelectorLabel = new QLabel(this);
            {
                m_pFileSelectorLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
                m_pFileSelectorLabel->setBuddy(m_pFileSelector);
            }
            pSettingsLayout->addWidget(m_pUsernameLabel, 0, 0);
            pSettingsLayout->addWidget(m_pUsernameEditor, 0, 1);
            pSettingsLayout->addWidget(m_pPasswordLabel, 1, 0);
            pSettingsLayout->addWidget(m_pPasswordEditor, 1, 1);
            pSettingsLayout->addWidget(m_pHostnameLabel, 2, 0);
            pSettingsLayout->addWidget(m_pHostnameEditor, 2, 1);
            pSettingsLayout->addWidget(m_pBucketLabel, 3, 0);
            pSettingsLayout->addWidget(m_pBucketEditor, 3, 1);
            pSettingsLayout->addWidget(m_pFileSelectorLabel, 4, 0);
            pSettingsLayout->addWidget(m_pFileSelector, 4, 1);
        }
        m_pOVF09Checkbox = new QCheckBox(this);
        m_pManifestCheckbox = new QCheckBox(this);
        pMainLayout->addWidget(m_pLabel);
        pMainLayout->addLayout(pSettingsLayout);
        pMainLayout->addWidget(m_pOVF09Checkbox);
        pMainLayout->addWidget(m_pManifestCheckbox);
        pMainLayout->addStretch();
        chooseDefaultSettings();
    }

    /* Setup connections: */
    connect(m_pUsernameEditor, SIGNAL(textChanged(const QString &)), this, SIGNAL(completeChanged()));
    connect(m_pPasswordEditor, SIGNAL(textChanged(const QString &)), this, SIGNAL(completeChanged()));
    connect(m_pHostnameEditor, SIGNAL(textChanged(const QString &)), this, SIGNAL(completeChanged()));
    connect(m_pBucketEditor, SIGNAL(textChanged(const QString &)), this, SIGNAL(completeChanged()));
    connect(m_pFileSelector, SIGNAL(pathChanged(const QString &)), this, SIGNAL(completeChanged()));

    /* Register fields: */
    registerField("OVF09Selected", this, "OVF09Selected");
    registerField("manifestSelected", this, "manifestSelected");
    registerField("username", this, "username");
    registerField("password", this, "password");
    registerField("hostname", this, "hostname");
    registerField("bucket", this, "bucket");
    registerField("path", this, "path");
}

void UIWizardExportAppPageBasic3::retranslateUi()
{
    /* Translate page: */
    setTitle(UIWizardExportApp::tr("Appliance settings"));

    /* Translate objects: */
    m_strDefaultApplianceName = UIWizardExportApp::tr("Appliance");
    /* Translate widgets: */
    m_pUsernameLabel->setText(UIWizardExportApp::tr("&Username:"));
    m_pPasswordLabel->setText(UIWizardExportApp::tr("&Password:"));
    m_pHostnameLabel->setText(UIWizardExportApp::tr("&Hostname:"));
    m_pBucketLabel->setText(UIWizardExportApp::tr("&Bucket:"));
    m_pFileSelectorLabel->setText(UIWizardExportApp::tr("&File:"));
    m_pFileSelector->setFileDialogTitle(UIWizardExportApp::tr("Please choose a virtual appliance file"));
    m_pFileSelector->setFileFilters(UIWizardExportApp::tr("Open Virtualization Format Archive (%1)").arg("*.ova") + ";;" +
                                    UIWizardExportApp::tr("Open Virtualization Format (%1)").arg("*.ovf"));
    m_pOVF09Checkbox->setToolTip(UIWizardExportApp::tr("Write in legacy OVF 0.9 format for compatibility with other virtualization products."));
    m_pOVF09Checkbox->setText(UIWizardExportApp::tr("&Write legacy OVF 0.9"));
    m_pManifestCheckbox->setToolTip(UIWizardExportApp::tr("Create a Manifest file for automatic data integrity checks on import."));
    m_pManifestCheckbox->setText(UIWizardExportApp::tr("Write &Manifest file"));

    /* Refresh current settings: */
    refreshCurrentSettings();
}

void UIWizardExportAppPageBasic3::initializePage()
{
    /* Translate page: */
    retranslateUi();
}

bool UIWizardExportAppPageBasic3::isComplete() const
{
    /* Initial result: */
    bool fResult = true;

    /* Check storage-type attributes: */
    if (fResult)
    {
        const QString &strFile = m_pFileSelector->path().toLower();
        fResult = VBoxGlobal::hasAllowedExtension(strFile, OVFFileExts);
        if (fResult)
        {
            StorageType storageType = field("storageType").value<StorageType>();
            switch (storageType)
            {
                case Filesystem:
                    break;
                case SunCloud:
                    fResult &= !m_pUsernameEditor->text().isEmpty() &&
                               !m_pPasswordEditor->text().isEmpty() &&
                               !m_pBucketEditor->text().isEmpty();
                    break;
                case S3:
                    fResult &= !m_pUsernameEditor->text().isEmpty() &&
                               !m_pPasswordEditor->text().isEmpty() &&
                               !m_pHostnameEditor->text().isEmpty() &&
                               !m_pBucketEditor->text().isEmpty();
                    break;
            }
        }
    }

    /* Return result: */
    return fResult;
}

void UIWizardExportAppPageBasic3::refreshCurrentSettings()
{
    /* Refresh base-class settings: */
    UIWizardExportAppPage3::refreshCurrentSettings();

    /* Setup components for chosen storage-type: */
    StorageType storageType = field("storageType").value<StorageType>();
    switch (storageType)
    {
        case Filesystem:
        {
            m_pLabel->setText(tr("<p>Please choose a filename to export the OVF/OVA to.</p>"
                                 "<p>If you use an <i>ova</i> extension, "
                                 "then all the files will be combined into one Open Virtualization Format Archive.</p>"
                                 "<p>If you use an <i>ovf</i> extension, "
                                 "several files will be written separately.</p>"
                                 "<p>Other extensions are not allowed.</p>"));
            m_pFileSelector->setFocus();
            break;
        }
        case SunCloud:
        {
            m_pLabel->setText(tr("Please complete the additional fields like the username, password "
                                 "and the bucket, and provide a filename for the OVF target."));
            m_pUsernameEditor->setFocus();
            break;
        }
        case S3:
        {
            m_pLabel->setText(tr("Please complete the additional fields like the username, password, "
                                 "hostname and the bucket, and provide a filename for the OVF target."));
            m_pUsernameEditor->setFocus();
            break;
        }
    }
}

