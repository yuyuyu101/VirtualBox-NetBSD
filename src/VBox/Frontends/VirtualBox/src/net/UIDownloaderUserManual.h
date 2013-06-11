/** @file
 *
 * VBox frontends: Qt GUI ("VirtualBox"):
 * UIDownloaderUserManual class declaration
 */

/*
 * Copyright (C) 2010-2012 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#ifndef __UIDownloaderUserManual_h__
#define __UIDownloaderUserManual_h__

/* Local includes: */
#include "UIDownloader.h"

/* UIDownloader extension for background user-manual downloading. */
class UIDownloaderUserManual : public UIDownloader
{
    Q_OBJECT;

signals:

    /* Notifies listeners about downloading finished: */
    void sigDownloadFinished(const QString &strFile);

public:

    /* Static stuff: */
    static UIDownloaderUserManual* create();
    static UIDownloaderUserManual* current();

private:

    /* Constructor/destructor: */
    UIDownloaderUserManual();
    ~UIDownloaderUserManual();

    /* Virtual stuff reimplementations: */
    bool askForDownloadingConfirmation(QNetworkReply *pReply);
    void handleDownloadedObject(QNetworkReply *pReply);

    /* Variables: */
    static UIDownloaderUserManual *m_spInstance;
};

#endif // __UIDownloaderUserManual_h__

