/** @file
 *
 * VBox frontends: Qt4 GUI ("VirtualBox"):
 * UIGlobalSettingsDisplay class declaration
 */

/*
 * Copyright (C) 2012 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#ifndef __UIGlobalSettingsDisplay_h__
#define __UIGlobalSettingsDisplay_h__

/* Local includes: */
#include "UISettingsPage.h"
#include "UIGlobalSettingsDisplay.gen.h"

/* Global settings / Display page / Cache: */
struct UISettingsCacheGlobalDisplay
{
    QString m_strMaxGuestResolution;
};

/* Global settings / Display page: */
class UIGlobalSettingsDisplay : public UISettingsPageGlobal, public Ui::UIGlobalSettingsDisplay
{
    Q_OBJECT;

public:

    /* Constructor: */
    UIGlobalSettingsDisplay();

protected:

    /* Load data to cashe from corresponding external object(s),
     * this task COULD be performed in other than GUI thread: */
    void loadToCacheFrom(QVariant &data);
    /* Load data to corresponding widgets from cache,
     * this task SHOULD be performed in GUI thread only: */
    void getFromCache();

    /* Save data from corresponding widgets to cache,
     * this task SHOULD be performed in GUI thread only: */
    void putToCache();
    /* Save data from cache to corresponding external object(s),
     * this task COULD be performed in other than GUI thread: */
    void saveFromCacheTo(QVariant &data);

    /* Navigation stuff: */
    void setOrderAfter(QWidget *pWidget);

    /* Translation stuff: */
    void retranslateUi();

protected slots:

    /* Max resolution combo activation handler: */
    void sltMaxResolutionComboActivated();

private:

    /* Populate combo-box: */
    void populate();

    /* Cache: */
    UISettingsCacheGlobalDisplay m_cache;
};

#endif // __UIGlobalSettingsDisplay_h__

