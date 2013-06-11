/* $Id: UIGlobalSettingsLanguage.cpp $ */
/** @file
 *
 * VBox frontends: Qt4 GUI ("VirtualBox"):
 * UIGlobalSettingsLanguage class implementation
 */

/*
 * Copyright (C) 2006-2011 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

/* Global includes */
#include <QDir>
#include <QHeaderView>
#include <QPainter>
#include <QTranslator>

#include <iprt/err.h>
#include <iprt/param.h>
#include <iprt/path.h>
#include <VBox/version.h>

/* Local includes */
#include "UIGlobalSettingsLanguage.h"
#include "VBoxGlobalSettings.h"
#include "VBoxGlobal.h"

extern const char *gVBoxLangSubDir;
extern const char *gVBoxLangFileBase;
extern const char *gVBoxLangFileExt;
extern const char *gVBoxLangIDRegExp;
extern const char *gVBoxBuiltInLangName;

/* Language item: */
class UILanguageItem : public QTreeWidgetItem
{
public:

    /* Language item type: */
    enum { UILanguageItemType = QTreeWidgetItem::UserType + 1 };

    /* Language item constructor: */
    UILanguageItem(QTreeWidget *pParent, const QTranslator &translator,
                   const QString &strId, bool fBuiltIn = false)
        : QTreeWidgetItem(pParent, UILanguageItemType), m_fBuiltIn(fBuiltIn)
    {
        Assert (!strId.isEmpty());

        /* Note: context/source/comment arguments below must match strings
         * used in VBoxGlobal::languageName() and friends (the latter are the
         * source of information for the lupdate tool that generates
         * translation files) */

        QString strNativeLanguage = tratra(translator, "@@@", "English", "Native language name");
        QString strNativeCountry = tratra(translator, "@@@", "--", "Native language country name "
                                                                   "(empty if this language is for all countries)");

        QString strEnglishLanguage = tratra(translator, "@@@", "English", "Language name, in English");
        QString strEnglishCountry = tratra(translator, "@@@", "--", "Language country name, in English "
                                                                    "(empty if native country name is empty)");

        QString strTranslatorsName = tratra(translator, "@@@", "Oracle Corporation", "Comma-separated list of translators");

        QString strItemName = strNativeLanguage;
        QString strLanguageName = strEnglishLanguage;

        if (!m_fBuiltIn)
        {
            if (strNativeCountry != "--")
                strItemName += " (" + strNativeCountry + ")";

            if (strEnglishCountry != "--")
                strLanguageName += " (" + strEnglishCountry + ")";

            if (strItemName != strLanguageName)
                strLanguageName = strItemName + " / " + strLanguageName;
        }
        else
        {
            strItemName += UIGlobalSettingsLanguage::tr(" (built-in)", "Language");
            strLanguageName += UIGlobalSettingsLanguage::tr(" (built-in)", "Language");
        }

        setText(0, strItemName);
        setText(1, strId);
        setText(2, strLanguageName);
        setText(3, strTranslatorsName);

        /* Current language appears in bold: */
        if (text(1) == VBoxGlobal::languageId())
        {
            QFont fnt = font(0);
            fnt.setBold(true);
            setFont(0, fnt);
        }
    }

    /* Constructs an item for an invalid language ID
     * (i.e. when a language file is missing or corrupt): */
    UILanguageItem(QTreeWidget *pParent, const QString &strId)
        : QTreeWidgetItem(pParent, UILanguageItemType), m_fBuiltIn(false)
    {
        Assert(!strId.isEmpty());

        setText(0, QString("<%1>").arg(strId));
        setText(1, strId);
        setText(2, UIGlobalSettingsLanguage::tr("<unavailable>", "Language"));
        setText(3, UIGlobalSettingsLanguage::tr("<unknown>", "Author(s)"));

        /* Invalid language appears in italic: */
        QFont fnt = font(0);
        fnt.setItalic(true);
        setFont(0, fnt);
    }

    /* Constructs an item for the default language ID
     * (column 1 will be set to QString::null): */
    UILanguageItem(QTreeWidget *pParent)
        : QTreeWidgetItem(pParent, UILanguageItemType), m_fBuiltIn(false)
    {
        setText(0, UIGlobalSettingsLanguage::tr("Default", "Language"));
        setText(1, QString::null);
        /* Empty strings of some reasonable length to prevent the info part
         * from being shrinked too much when the list wants to be wider */
        setText(2, "                ");
        setText(3, "                ");
    }

    bool isBuiltIn() const { return m_fBuiltIn; }

    bool operator<(const QTreeWidgetItem &other) const
    {
        QString thisId = text(1);
        QString thatId = other.text(1);
        if (thisId.isNull())
            return true;
        if (thatId.isNull())
            return false;
        if (m_fBuiltIn)
            return true;
        if (other.type() == UILanguageItemType && ((UILanguageItem*)&other)->m_fBuiltIn)
            return false;
        return QTreeWidgetItem::operator<(other);
    }

private:

    QString tratra(const QTranslator &translator, const char *pCtxt,
                   const char *pSrc, const char *pCmnt)
    {
        QString strMsg = translator.translate(pCtxt, pSrc, pCmnt);
        /* Return the source text if no translation is found: */
        if (strMsg.isEmpty())
            strMsg = QString(pSrc);
        return strMsg;
    }

    bool m_fBuiltIn : 1;
};

/* Language page constructor: */
UIGlobalSettingsLanguage::UIGlobalSettingsLanguage()
    : m_fIsLanguageChanged(false)
{
    /* Apply UI decorations: */
    Ui::UIGlobalSettingsLanguage::setupUi(this);

    /* Setup widgets: */
    m_pLanguageTree->header()->hide();
    m_pLanguageTree->hideColumn(1);
    m_pLanguageTree->hideColumn(2);
    m_pLanguageTree->hideColumn(3);

    /* Setup connections: */
    connect(m_pLanguageTree, SIGNAL(painted(QTreeWidgetItem*, QPainter*)),
            this, SLOT(sltLanguageItemPainted(QTreeWidgetItem*, QPainter*)));
    connect(m_pLanguageTree, SIGNAL(currentItemChanged(QTreeWidgetItem*, QTreeWidgetItem*)),
            this, SLOT(sltCurrentLanguageChanged(QTreeWidgetItem*)));

    /* Apply language settings: */
    retranslateUi();
}

/* Load data to cashe from corresponding external object(s),
 * this task COULD be performed in other than GUI thread: */
void UIGlobalSettingsLanguage::loadToCacheFrom(QVariant &data)
{
    /* Fetch data to properties & settings: */
    UISettingsPageGlobal::fetchData(data);

    /* Load to cache: */
    m_cache.m_strLanguageId = m_settings.languageId();

    /* Upload properties & settings to data: */
    UISettingsPageGlobal::uploadData(data);
}

/* Load data to corresponding widgets from cache,
 * this task SHOULD be performed in GUI thread only: */
void UIGlobalSettingsLanguage::getFromCache()
{
    /* Fetch from cache: */
    reload(m_cache.m_strLanguageId);
    m_pLanguageInfo->setFixedHeight(fontMetrics().height() * 4);
}

/* Save data from corresponding widgets to cache,
 * this task SHOULD be performed in GUI thread only: */
void UIGlobalSettingsLanguage::putToCache()
{
    /* Upload to cache: */
    QTreeWidgetItem *pCurrentItem = m_pLanguageTree->currentItem();
    Assert(pCurrentItem);
    if (pCurrentItem)
        m_cache.m_strLanguageId = pCurrentItem->text(1);
}

/* Save data from cache to corresponding external object(s),
 * this task COULD be performed in other than GUI thread: */
void UIGlobalSettingsLanguage::saveFromCacheTo(QVariant &data)
{
    /* Fetch data to properties & settings: */
    UISettingsPageGlobal::fetchData(data);

    /* Save from cache: */
    if (m_fIsLanguageChanged)
        m_settings.setLanguageId(m_cache.m_strLanguageId);

    /* Upload properties & settings to data: */
    UISettingsPageGlobal::uploadData(data);
}

/* Navigation stuff: */
void UIGlobalSettingsLanguage::setOrderAfter(QWidget *pWidget)
{
    setTabOrder(pWidget, m_pLanguageTree);
}

/* Translation stuff: */
void UIGlobalSettingsLanguage::retranslateUi()
{
    /* Translate uic generated strings: */
    Ui::UIGlobalSettingsLanguage::retranslateUi(this);

    /* Reload language tree: */
    reload(VBoxGlobal::languageId());
}

/* Reload language tree: */
void UIGlobalSettingsLanguage::reload(const QString &strLangId)
{
    /* Clear languages tree: */
    m_pLanguageTree->clear();

    /* Load languages tree: */
    char szNlsPath[RTPATH_MAX];
    int rc = RTPathAppPrivateNoArch(szNlsPath, sizeof(szNlsPath));
    AssertRC(rc);
    QString strNlsPath = QString(szNlsPath) + gVBoxLangSubDir;
    QDir nlsDir(strNlsPath);
    QStringList files = nlsDir.entryList(QStringList(QString("%1*%2").arg(gVBoxLangFileBase, gVBoxLangFileExt)), QDir::Files);

    QTranslator translator;
    /* Add the default language: */
    new UILanguageItem(m_pLanguageTree);
    /* Add the built-in language: */
    new UILanguageItem(m_pLanguageTree, translator, gVBoxBuiltInLangName, true /* built-in */);
    /* Add all existing languages */
    for (QStringList::Iterator it = files.begin(); it != files.end(); ++it)
    {
        QString strFileName = *it;
        QRegExp regExp(QString(gVBoxLangFileBase) + gVBoxLangIDRegExp);
        int iPos = regExp.indexIn(strFileName);
        if (iPos == -1)
            continue;

        /* Skip any English version, cause this is extra handled: */
        QString strLanguage = regExp.cap(2);
        if (strLanguage.toLower() == "en")
            continue;

        bool fLoadOk = translator.load(strFileName, strNlsPath);
        if (!fLoadOk)
            continue;

        new UILanguageItem(m_pLanguageTree, translator, regExp.cap(1));
    }

    /* Adjust selector list: */
#ifdef Q_WS_MAC
    int width = qMax(static_cast<QAbstractItemView*>(m_pLanguageTree)->sizeHintForColumn(0) +
                     2 * m_pLanguageTree->frameWidth() + QApplication::style()->pixelMetric(QStyle::PM_ScrollBarExtent),
                     220);
    m_pLanguageTree->setFixedWidth(width);
#else /* Q_WS_MAC */
    m_pLanguageTree->setMinimumWidth(static_cast<QAbstractItemView*>(m_pLanguageTree)->sizeHintForColumn(0) +
                                     2 * m_pLanguageTree->frameWidth() + QApplication::style()->pixelMetric(QStyle::PM_ScrollBarExtent));
#endif /* Q_WS_MAC */
    m_pLanguageTree->resizeColumnToContents(0);

    /* Search for necessary language: */
    QList<QTreeWidgetItem*> itemsList = m_pLanguageTree->findItems(strLangId, Qt::MatchExactly, 1);
    QTreeWidgetItem *pItem = itemsList.isEmpty() ? 0 : itemsList[0];
    if (!pItem)
    {
        /* Add an pItem for an invalid language to represent it in the list: */
        pItem = new UILanguageItem(m_pLanguageTree, strLangId);
        m_pLanguageTree->resizeColumnToContents(0);
    }
    Assert(pItem);
    if (pItem)
        m_pLanguageTree->setCurrentItem(pItem);

    m_pLanguageTree->sortItems(0, Qt::AscendingOrder);
    m_pLanguageTree->scrollToItem(pItem);
    m_fIsLanguageChanged = false;
}

/* Routine to paint language items: */
void UIGlobalSettingsLanguage::sltLanguageItemPainted(QTreeWidgetItem *pItem, QPainter *pPainter)
{
    if (pItem && pItem->type() == UILanguageItem::UILanguageItemType)
    {
        UILanguageItem *pLanguageItem = static_cast<UILanguageItem*>(pItem);
        if (pLanguageItem->isBuiltIn())
        {
            QRect rect = m_pLanguageTree->visualItemRect(pLanguageItem);
            pPainter->setPen(m_pLanguageTree->palette().color(QPalette::Mid));
            pPainter->drawLine(rect.x(), rect.y() + rect.height() - 1,
                               rect.x() + rect.width(), rect.y() + rect.height() - 1);
        }
    }
}

/* Slot to handle currently language change fact: */
void UIGlobalSettingsLanguage::sltCurrentLanguageChanged(QTreeWidgetItem *pItem)
{
    if (!pItem) return;

    /* Disable labels for the Default language item: */
    bool fEnabled = !pItem->text (1).isNull();

    m_pLanguageInfo->setEnabled(fEnabled);
    m_pLanguageInfo->setText(QString("<table>"
                             "<tr><td>%1&nbsp;</td><td>%2</td></tr>"
                             "<tr><td>%3&nbsp;</td><td>%4</td></tr>"
                             "</table>")
                             .arg(tr("Language:"))
                             .arg(pItem->text(2))
                             .arg(tr("Author(s):"))
                             .arg(pItem->text(3)));

    m_fIsLanguageChanged = true;
}

