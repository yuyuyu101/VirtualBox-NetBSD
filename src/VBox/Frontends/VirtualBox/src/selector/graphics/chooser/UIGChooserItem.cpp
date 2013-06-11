/* $Id: UIGChooserItem.cpp $ */
/** @file
 *
 * VBox frontends: Qt GUI ("VirtualBox"):
 * UIGChooserItem class definition
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

/* Qt includes: */
#include <QApplication>
#include <QStyle>
#include <QPainter>
#include <QGraphicsScene>
#include <QStyleOptionFocusRect>
#include <QGraphicsSceneMouseEvent>
#include <QStateMachine>
#include <QPropertyAnimation>
#include <QSignalTransition>

/* GUI includes: */
#include "UIGChooserItem.h"
#include "UIGChooserModel.h"
#include "UIGChooserItemGroup.h"
#include "UIGChooserItemMachine.h"

UIGChooserItem::UIGChooserItem(UIGChooserItem *pParent, bool fTemporary)
    : m_fRoot(!pParent)
    , m_fTemporary(fTemporary)
    , m_pParent(pParent)
    , m_iPreviousMinimumWidthHint(0)
    , m_iPreviousMinimumHeightHint(0)
    , m_dragTokenPlace(DragToken_Off)
    , m_fHovered(false)
    , m_pHighlightMachine(0)
    , m_pForwardAnimation(0)
    , m_pBackwardAnimation(0)
    , m_iAnimationDuration(400)
    , m_iDefaultDarkness(100)
    , m_iHighlightDarkness(90)
    , m_iAnimationDarkness(m_iDefaultDarkness)
    , m_iDragTokenDarkness(110)
{
    /* Basic item setup: */
    setOwnedByLayout(false);
    setAcceptDrops(true);
    setFocusPolicy(Qt::NoFocus);
    setFlag(QGraphicsItem::ItemIsSelectable, false);
    setAcceptHoverEvents(!isRoot());

    /* Non-root item? */
    if (!isRoot())
    {
        /* Create state machine: */
        m_pHighlightMachine = new QStateMachine(this);
        /* Create 'default' state: */
        QState *pStateDefault = new QState(m_pHighlightMachine);
        /* Create 'highlighted' state: */
        QState *pStateHighlighted = new QState(m_pHighlightMachine);

        /* Forward animation: */
        m_pForwardAnimation = new QPropertyAnimation(this, "animationDarkness", this);
        m_pForwardAnimation->setDuration(m_iAnimationDuration);
        m_pForwardAnimation->setStartValue(m_iDefaultDarkness);
        m_pForwardAnimation->setEndValue(m_iHighlightDarkness);

        /* Backward animation: */
        m_pBackwardAnimation = new QPropertyAnimation(this, "animationDarkness", this);
        m_pBackwardAnimation->setDuration(m_iAnimationDuration);
        m_pBackwardAnimation->setStartValue(m_iHighlightDarkness);
        m_pBackwardAnimation->setEndValue(m_iDefaultDarkness);

        /* Add state transitions: */
        QSignalTransition *pDefaultToHighlighted = pStateDefault->addTransition(this, SIGNAL(sigHoverEnter()), pStateHighlighted);
        pDefaultToHighlighted->addAnimation(m_pForwardAnimation);
        QSignalTransition *pHighlightedToDefault = pStateHighlighted->addTransition(this, SIGNAL(sigHoverLeave()), pStateDefault);
        pHighlightedToDefault->addAnimation(m_pBackwardAnimation);

        /* Initial state is 'default': */
        m_pHighlightMachine->setInitialState(pStateDefault);
        /* Start state-machine: */
        m_pHighlightMachine->start();
    }
}

UIGChooserItemGroup* UIGChooserItem::toGroupItem()
{
    UIGChooserItemGroup *pItem = qgraphicsitem_cast<UIGChooserItemGroup*>(this);
    AssertMsg(pItem, ("Trying to cast invalid item type to UIGChooserItemGroup!"));
    return pItem;
}

UIGChooserItemMachine* UIGChooserItem::toMachineItem()
{
    UIGChooserItemMachine *pItem = qgraphicsitem_cast<UIGChooserItemMachine*>(this);
    AssertMsg(pItem, ("Trying to cast invalid item type to UIGChooserItemMachine!"));
    return pItem;
}

UIGChooserModel* UIGChooserItem::model() const
{
    UIGChooserModel *pModel = qobject_cast<UIGChooserModel*>(QIGraphicsWidget::scene()->parent());
    AssertMsg(pModel, ("Incorrect graphics scene parent set!"));
    return pModel;
}

UIGChooserItem* UIGChooserItem::parentItem() const
{
    return m_pParent;
}

void UIGChooserItem::show()
{
    /* Call to base-class: */
    QIGraphicsWidget::show();
}

void UIGChooserItem::hide()
{
    /* Call to base-class: */
    QIGraphicsWidget::hide();
}

void UIGChooserItem::setRoot(bool fRoot)
{
    m_fRoot = fRoot;
    handleRootStatusChange();
}

bool UIGChooserItem::isRoot() const
{
    return m_fRoot;
}

bool UIGChooserItem::isHovered() const
{
    return m_fHovered;
}

void UIGChooserItem::setHovered(bool fHovered)
{
    m_fHovered = fHovered;
    if (m_fHovered)
        emit sigHoverEnter();
    else
        emit sigHoverLeave();
}

void UIGChooserItem::updateGeometry()
{
    /* Call to base-class: */
    QIGraphicsWidget::updateGeometry();

    /* Update parent's geometry: */
    if (parentItem())
        parentItem()->updateGeometry();

    /* Special handling for root-items: */
    if (isRoot())
    {
        /* Root-item should notify chooser-view if minimum-width-hint was changed: */
        int iMinimumWidthHint = minimumWidthHint();
        if (m_iPreviousMinimumWidthHint != iMinimumWidthHint)
        {
            /* Save new minimum-width-hint, notify listener: */
            m_iPreviousMinimumWidthHint = iMinimumWidthHint;
            emit sigMinimumWidthHintChanged(m_iPreviousMinimumWidthHint);
        }
        /* Root-item should notify chooser-view if minimum-height-hint was changed: */
        int iMinimumHeightHint = minimumHeightHint();
        if (m_iPreviousMinimumHeightHint != iMinimumHeightHint)
        {
            /* Save new minimum-height-hint, notify listener: */
            m_iPreviousMinimumHeightHint = iMinimumHeightHint;
            emit sigMinimumHeightHintChanged(m_iPreviousMinimumHeightHint);
        }
    }
}

void UIGChooserItem::makeSureItsVisible()
{
    /* If item is not visible: */
    if (!isVisible())
    {
        /* Get parrent item, assert if can't: */
        if (UIGChooserItemGroup *pParentItem = parentItem()->toGroupItem())
        {
            /* We should make parent visible: */
            pParentItem->makeSureItsVisible();
            /* And make sure its opened: */
            if (pParentItem->isClosed())
                pParentItem->open(false);
        }
    }
}

void UIGChooserItem::setDragTokenPlace(DragToken where)
{
    /* Something changed? */
    if (m_dragTokenPlace != where)
    {
        m_dragTokenPlace = where;
        update();
    }
}

DragToken UIGChooserItem::dragTokenPlace() const
{
    return m_dragTokenPlace;
}


bool UIGChooserItem::isTemporary() const
{
    return m_fTemporary;
}

void UIGChooserItem::hoverMoveEvent(QGraphicsSceneHoverEvent*)
{
    if (!m_fHovered)
    {
        m_fHovered = true;
        emit sigHoverEnter();
    }
}

void UIGChooserItem::hoverLeaveEvent(QGraphicsSceneHoverEvent*)
{
    if (m_fHovered)
    {
        m_fHovered = false;
        emit sigHoverLeave();
    }
}

void UIGChooserItem::mousePressEvent(QGraphicsSceneMouseEvent *pEvent)
{
    /* By default, non-moveable and non-selectable items
     * can't grab mouse-press events which is required
     * to grab further mouse-move events which we wants... */
    if (isRoot())
        pEvent->ignore();
    else
        pEvent->accept();
}

void UIGChooserItem::mouseMoveEvent(QGraphicsSceneMouseEvent *pEvent)
{
    /* Make sure item is really dragged: */
    if (QLineF(pEvent->screenPos(),
               pEvent->buttonDownScreenPos(Qt::LeftButton)).length() <
        QApplication::startDragDistance())
        return;

    /* Initialize dragging: */
    QDrag *pDrag = new QDrag(pEvent->widget());
    model()->setCurrentDragObject(pDrag);
    pDrag->setPixmap(toPixmap());
    pDrag->setMimeData(createMimeData());
    pDrag->exec(Qt::MoveAction | Qt::CopyAction, Qt::MoveAction);
}

void UIGChooserItem::dragMoveEvent(QGraphicsSceneDragDropEvent *pEvent)
{
    /* Make sure we are non-root: */
    if (!isRoot())
    {
        /* Allow drag tokens only for the same item type as current: */
        bool fAllowDragToken = false;
        if ((type() == UIGChooserItemType_Group &&
             pEvent->mimeData()->hasFormat(UIGChooserItemGroup::className())) ||
            (type() == UIGChooserItemType_Machine &&
             pEvent->mimeData()->hasFormat(UIGChooserItemMachine::className())))
            fAllowDragToken = true;
        /* Do we need a drag-token? */
        if (fAllowDragToken)
        {
            QPoint p = pEvent->pos().toPoint();
            if (p.y() < 10)
                setDragTokenPlace(DragToken_Up);
            else if (p.y() > minimumSizeHint().toSize().height() - 10)
                setDragTokenPlace(DragToken_Down);
            else
                setDragTokenPlace(DragToken_Off);
        }
    }
    /* Check if drop is allowed: */
    pEvent->setAccepted(isDropAllowed(pEvent, dragTokenPlace()));
}

void UIGChooserItem::dragLeaveEvent(QGraphicsSceneDragDropEvent*)
{
    resetDragToken();
}

void UIGChooserItem::dropEvent(QGraphicsSceneDragDropEvent *pEvent)
{
    /* Do we have token active? */
    switch (dragTokenPlace())
    {
        case DragToken_Off:
        {
            /* Its our drop, processing: */
            processDrop(pEvent);
            break;
        }
        default:
        {
            /* Its parent drop, passing: */
            parentItem()->processDrop(pEvent, this, dragTokenPlace());
            break;
        }
    }
}

/* static */
void UIGChooserItem::configurePainterShape(QPainter *pPainter,
                                           const QStyleOptionGraphicsItem *pOption,
                                           int iRadius)
{
    /* Rounded corners? */
    if (iRadius)
    {
        /* Setup clipping: */
        QPainterPath roundedPath;
        roundedPath.addRoundedRect(pOption->rect, iRadius, iRadius);
        pPainter->setClipPath(roundedPath);
    }
}

/* static */
void UIGChooserItem::paintFrameRect(QPainter *pPainter, const QRect &rect, bool fIsSelected, int iRadius)
{
    pPainter->save();
    QPalette pal = QApplication::palette();
    QColor base = pal.color(QPalette::Active, fIsSelected ? QPalette::Highlight : QPalette::Window);
    pPainter->setPen(base.darker(160));
    if (iRadius)
        pPainter->drawRoundedRect(rect, iRadius, iRadius);
    else
        pPainter->drawRect(rect);
    pPainter->restore();
}

/* static */
void UIGChooserItem::paintPixmap(QPainter *pPainter, const QRect &rect, const QPixmap &pixmap)
{
    pPainter->drawPixmap(rect, pixmap);
}

/* static */
void UIGChooserItem::paintText(QPainter *pPainter, QPoint point,
                               const QFont &font, QPaintDevice *pPaintDevice,
                               const QString &strText)
{
    /* Prepare variables: */
    QFontMetrics fm(font, pPaintDevice);
    point += QPoint(0, fm.ascent());

    /* Draw text: */
    pPainter->save();
    pPainter->setFont(font);
    pPainter->drawText(point, strText);
    pPainter->restore();
}

/* static */
QSize UIGChooserItem::textSize(const QFont &font, QPaintDevice *pPaintDevice, const QString &strText)
{
    /* Make sure text is not empty: */
    if (strText.isEmpty())
        return QSize(0, 0);

    /* Return text size, based on font-metrics: */
    QFontMetrics fm(font, pPaintDevice);
    return QSize(fm.width(strText), fm.height());
}

/* static */
int UIGChooserItem::textWidth(const QFont &font, QPaintDevice *pPaintDevice, int iCount)
{
    /* Return text width: */
    QFontMetrics fm(font, pPaintDevice);
    QString strString;
    strString.fill('_', iCount);
    return fm.width(strString);
}

/* static */
QString UIGChooserItem::compressText(const QFont &font, QPaintDevice *pPaintDevice, QString strText, int iWidth)
{
    /* Check if passed text is empty: */
    if (strText.isEmpty())
        return strText;

    /* Check if passed text feats maximum width: */
    QFontMetrics fm(font, pPaintDevice);
    if (fm.width(strText) <= iWidth)
        return strText;

    /* Truncate otherwise: */
    QString strEllipsis = QString("...");
    int iEllipsisWidth = fm.width(strEllipsis + " ");
    while (!strText.isEmpty() && fm.width(strText) + iEllipsisWidth > iWidth)
        strText.truncate(strText.size() - 1);
    return strText + strEllipsis;
}

UIGChooserItemMimeData::UIGChooserItemMimeData(UIGChooserItem *pItem)
    : m_pItem(pItem)
{
}

bool UIGChooserItemMimeData::hasFormat(const QString &strMimeType) const
{
    if (strMimeType == QString(m_pItem->metaObject()->className()))
        return true;
    return false;
}

UIGChooserItem* UIGChooserItemMimeData::item() const
{
    return m_pItem;
}

