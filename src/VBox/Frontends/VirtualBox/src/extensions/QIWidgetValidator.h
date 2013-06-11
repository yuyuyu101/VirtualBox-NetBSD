/** @file
 *
 * VBox frontends: Qt GUI ("VirtualBox"):
 * VirtualBox Qt extensions: QIWidgetValidator class declaration
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

#ifndef __QIWidgetValidator_h__
#define __QIWidgetValidator_h__

#include <limits.h>

/* Qt includes */
#include <QObject>
#include <QValidator>
#include <QList>
#include <QPointer>

class QIWidgetValidator : public QObject
{
    Q_OBJECT

public:

    QIWidgetValidator (QWidget *aWidget, QObject *aParent = 0);
    QIWidgetValidator (const QString &aCaption,
                       QWidget *aWidget, QObject *aParent = 0);
    ~QIWidgetValidator();

    QWidget *widget() const { return mWidget; }
    bool isValid() const;
    void rescan();

    void setCaption (const QString& aCaption) { mCaption = aCaption; }
    QString caption() const { return mCaption; }

    QString warningText() const;

    QString lastWarning() const { return mLastWarning; }
    void setLastWarning (const QString &aLastWarning) { mLastWarning = aLastWarning; }

    void setOtherValid (bool aValid) { mOtherValid = aValid; }
    bool isOtherValid() const { return mOtherValid; }

signals:

    void validityChanged (const QIWidgetValidator *aValidator);
    void isValidRequested (QIWidgetValidator *aValidator);

public slots:

    void revalidate() { doRevalidate(); }

private:

    QString mLastWarning;
    QString mCaption;
    QWidget *mWidget;
    bool mOtherValid;

    struct Watched
    {
        Watched()
            : state (QValidator::Acceptable) {}

        QPointer<QWidget> widget;
        QPointer<QWidget> buddy;
        QValidator::State state;
    };

    QList <Watched> mWatched;
    Watched mLastInvalid;

private slots:

    void doRevalidate() { emit validityChanged (this); }
};

class QIULongValidator : public QValidator
{
public:

    QIULongValidator (QObject *aParent)
        : QValidator (aParent)
        , mBottom (0), mTop (ULONG_MAX) {}

    QIULongValidator (ulong aMinimum, ulong aMaximum,
                      QObject *aParent)
        : QValidator (aParent)
        , mBottom (aMinimum), mTop (aMaximum) {}

    ~QIULongValidator() {}

    State validate (QString &aInput, int &aPos) const;
    void setBottom (ulong aBottom) { setRange (aBottom, mTop); }
    void setTop (ulong aTop) { setRange (mBottom, aTop); }
    void setRange (ulong aBottom, ulong aTop) { mBottom = aBottom; mTop = aTop; }
    ulong bottom() const { return mBottom; }
    ulong top() const { return mTop; }

private:

    ulong mBottom;
    ulong mTop;
};

#endif // __QIWidgetValidator_h__

