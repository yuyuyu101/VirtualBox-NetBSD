/** @file
 *
 * VBox frontends: Qt GUI ("VirtualBox"):
 * VirtualBox Qt extensions: QIStateIndicator class declaration
 */

/*
 * Copyright (C) 2006-2010 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 */

#ifndef __QIStateIndicator_h__
#define __QIStateIndicator_h__

/* Qt includes */
#include <QFrame>
#include <QHash>

class QIStateIndicator : public QFrame
{
    Q_OBJECT;

public:

    QIStateIndicator (QWidget *pParent = 0);
    QIStateIndicator (int aState);
    ~QIStateIndicator();

    virtual QSize sizeHint() const;

    int state () const { return mState; }

    QPixmap stateIcon (int aState) const;
    void setStateIcon (int aState, const QPixmap &aPixmap);

    virtual void updateAppearance() {}

public slots:

    virtual void setState (int aState);
    virtual void setState (bool aState) { setState ((int) aState); }

signals:

    void mouseDoubleClicked (QIStateIndicator *aIndicator,
                             QMouseEvent *aEv);
    void contextMenuRequested (QIStateIndicator *aIndicator,
                               QContextMenuEvent *aEv);

protected:

    virtual void paintEvent (QPaintEvent *aEv);
    virtual void drawContents (QPainter *aPainter);

#ifdef Q_WS_MAC
    virtual void mousePressEvent (QMouseEvent *aEv);
#endif
    virtual void mouseDoubleClickEvent (QMouseEvent *aEv);
    virtual void contextMenuEvent (QContextMenuEvent *aEv);

private:

    int mState;
    QSize mSize;

    struct Icon
    {
        Icon (const QPixmap &aPixmap)
            : pixmap (aPixmap)
            , bgPixmap (NULL) {}

        QPixmap pixmap;
        QPixmap cached;
        QColor bgColor;
        const QPixmap *bgPixmap;
        QPoint bgOff;
    };

    QHash <int, Icon *> mStateIcons;
};

#endif // __QIStateIndicator_h__

