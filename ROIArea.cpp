// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "ROIArea.h"

#include <QMouseEvent>
#include <QPainter>

ROIArea::ROIArea(QWidget *parent)
    : QWidget(parent)
{
    // 1. Create and store an initial empty base image
    QImage baseImage(size(), QImage::Format_ARGB32_Premultiplied);
    baseImage.fill(Qt::white); // or Qt::transparent

    overlayList.append(baseImage);     // owned storage
    QImage *ptr = &overlayList.last(); // pointer handle
    overlayListHandles.append(ptr);

    // 2. Set current image pointer
    setCurrentImage(ptr); // currentImagePtr = ptr;

    // 3. Widget setup
    setAttribute(Qt::WA_StaticContents);
    setFocusPolicy(Qt::StrongFocus);
    finalPolygon = QPolygonF();
}

bool ROIArea::openImage(const QString &fileName)
{
    QImage loadedImage;
    if (!gdalHandler.openSrcRaster(fileName)) {
        CPLErr errClass = CPLGetLastErrorType();
        int errNo = CPLGetLastErrorNo();
        const char *msg = CPLGetLastErrorMsg();
        qWarning() << "GDAL error [" << errNo << "/" << errClass << "]:" << msg;
        return false;
    }

    loadedImage = gdalHandler.toQImage();
    if (loadedImage.isNull()) {
        qWarning() << "Image was null after GDAL conversion";
        return false;
    }

    // Option 2: store by value, keep pointer handle
    overlayList.append(loadedImage);   // owns the image
    QImage *ptr = &overlayList.last(); // pointer to stored image

    overlayListHandles.append(ptr); // track handle stack
    setCurrentImage(ptr);           // currentImagePtr = ptr;

    modified = false;
    update();
    return true;
}

bool ROIArea::closeImage()
{
    // Reset drawing state
    modified = false;
    writing = false;
    haveStartPoint = false;
    showFinalPolygon = false;
    isPolygonDrawn = false;
    finalPolygon = QPolygonF();
    grahamScanner.clear();

    // Remove all existing overlays and handles
    overlayList.clear();
    overlayListHandles.clear();
    currentImagePtr = nullptr;

    // Create a new blank white image as background
    QImage blank(size(), QImage::Format_ARGB32_Premultiplied);
    blank.fill(Qt::white); // white background

    overlayList.append(blank);         // own it
    QImage *ptr = &overlayList.last(); // pointer handle
    overlayListHandles.append(ptr);
    currentImagePtr = ptr; // make it current

    update(); // repaint
    return true;
}
bool ROIArea::saveImage(const QString &fileName, const char *fileFormat)
{
    QImage visibleImage = getCurrentImage();
    resizeImage(&visibleImage, size());

    if (visibleImage.save(fileName, fileFormat)) {
        modified = false;
        return true;
    }
    return false;
}

void ROIArea::setPenColor(const QColor &newColor)

{
    myPenColor = newColor;
}

void ROIArea::clearImage()
{
    QImage toClearImage = getCurrentImage();
    toClearImage.fill(qRgb(255, 255, 255));
    modified = true;
    update();
}
void ROIArea::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Escape:
        //clearImage(); // clears image + update()
        showFinalPolygon = false;
        finalPolygon = QPolygonF();
        isPolygonDrawn = false;
        grahamScanner.clear();
        update(); // ensure widget repaints
        break;

    case Qt::Key_Space:
        if (!isPolygonDrawn) {
            writing = false;
            haveStartPoint = false;

            finalPolygon = grahamScanner.ComputeHull(); // must return QPolygonF
            showFinalPolygon = !finalPolygon.isEmpty()
                               && finalPolygon.size() >= 3; // only if it’s a real polygon
            isPolygonDrawn = showFinalPolygon;
            update(); // triggers paintEvent
        }
        break;

    default:
        QWidget::keyPressEvent(event);
        break;
    }
}
void ROIArea::mousePressEvent(QMouseEvent *event)
{
    const QPointF p = event->position();

    if (event->button() == Qt::LeftButton) {
        // If a final polygon is visible, start a new one on first left click
        if (showFinalPolygon) {
            showFinalPolygon = false;
            isPolygonDrawn = false;
            finalPolygon = QPolygonF();
            grahamScanner.clear(); // implement this to clear its point list
        }

        writing = true;
        grahamScanner.addPointToPolygon(p);
        drawPointTo(p);

        if (!haveStartPoint) {
            lastPoint = p;
            haveStartPoint = true;
        } else {
            drawLineTo(p);
        }

        update();
    } else if (event->button() == Qt::RightButton) {
        // stop drawing current poly, don’t clear hull
        writing = false;
        haveStartPoint = false;
        update();
    }

    QWidget::mousePressEvent(event);
}

void ROIArea::mouseMoveEvent(QMouseEvent *event)
{
    QWidget::mouseMoveEvent(event);
}

void ROIArea::mouseReleaseEvent(QMouseEvent *event)
{
    QWidget::mouseReleaseEvent(event);
}

void ROIArea::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    QRect r = event->rect();

    if (currentImagePtr && !currentImagePtr->isNull()) {
        painter.drawImage(r, *currentImagePtr, r);
    } else if (!overlayListHandles.isEmpty()) {
        QImage *top = overlayListHandles.last();
        if (top && !top->isNull())
            painter.drawImage(r, *top, r);
    } else {
        painter.fillRect(r, Qt::white);
    }
}

void ROIArea::resizeEvent(QResizeEvent *event)
{
    QImage image = getCurrentImage();
    if (width() > image.width() || height() > image.height()) {
        int newWidth = qMax(width() + 128, image.width());
        int newHeight = qMax(height() + 128, image.height());
        resizeImage(&image, QSize(newWidth, newHeight));
        update();
    }
    QWidget::resizeEvent(event);
}

void ROIArea::drawPointTo(const QPointF &endPoint)
{
    qWarning() << "Triggered drawPointTo()";

    // 1. Ensure there is a current image to draw on
    if (!currentImagePtr || currentImagePtr->isNull())
        return;

    QPainter painter(currentImagePtr);
    if (!painter.isActive())
        return;

    painter.setPen(QPen(myPenColor, penWidth, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.drawPoint(endPoint);

    modified = true;

    int rad = (penWidth / 2) + 2;
    update(QRect(endPoint.x() - rad, endPoint.y() - rad, 2 * rad + 1, 2 * rad + 1));
}

void ROIArea::drawLineTo(const QPointF &endPoint)
{
    qWarning() << "Triggered drawLineTo()";

    // 1. Ensure there is a current image to draw on
    if (!currentImagePtr || currentImagePtr->isNull())
        return;

    QPainter painter(currentImagePtr);
    if (!painter.isActive())
        return;

    painter.setPen(QPen(penColor(), penWidth / 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.drawLine(lastPoint, endPoint);

    int rad = (penWidth / 2) + 2;
    update(
        QRect(lastPoint.toPoint(), endPoint.toPoint()).normalized().adjusted(-rad, -rad, +rad, +rad));

    lastPoint = endPoint; // to chain segments
}

void ROIArea::resizeImage(QImage *image, const QSize &newSize)
{
    if (image->size() == newSize)
        return;

    QImage newImage(newSize, QImage::Format_RGB32);
    newImage.fill(qRgb(255, 255, 255));
    QPainter painter(&newImage);
    painter.drawImage(QPoint(0, 0), *image);
    *image = newImage;
}

void ROIArea::drawPolygon(const QPolygonF &polygon)
{
    // Ensure there is a current image to draw on
    if (!currentImagePtr || currentImagePtr->isNull())
        return;

    QPainter painter(currentImagePtr);
    if (!painter.isActive())
        return;

    painter.setRenderHint(QPainter::Antialiasing);

    // Pen for the outline
    QPen pen(Qt::blue);
    pen.setWidth(2);
    painter.setPen(pen);

    // Brush for the area (fill)
    QBrush brush(QColor(0, 0, 255, 80)); // semi‑transparent blue
    painter.setBrush(brush);

    painter.drawPolygon(polygon);

    update(); // repaint to show the change
}
void ROIArea::addOverlay(const QImage &baseImage)
{
    qWarning() << "Triggered addOverlay";

    if (baseImage.isNull() || baseImage.size().isEmpty())
        return;

    // 1. Start overlay as a copy of the base image, so pixels are visible
    QImage overlay = baseImage.convertToFormat(QImage::Format_ARGB32_Premultiplied);

    QPainter p(&overlay);
    if (!p.isActive())
        return;

    p.setRenderHint(QPainter::Antialiasing);

    // 2. Draw bright yellow text in the top-right corner
    p.setPen(Qt::yellow);
    QFont f = p.font();
    f.setPointSize(14);
    f.setBold(true);
    p.setFont(f);

    const QString text = QStringLiteral("ROI Layer");
    const int margin = 5;
    QRect rect = overlay.rect().adjusted(margin, margin, -margin, -margin);
    p.drawText(rect, Qt::AlignTop | Qt::AlignRight, text);

    // 3. Store overlay and update current image pointer (Option 2)
    overlayList.append(overlay);
    QImage *ptr = &overlayList.last();
    overlayListHandles.append(ptr);
    setCurrentImage(ptr);

    update();
}

void ROIArea::setCurrentImage(QImage *settingImage)
{
    currentImagePtr = settingImage;
}

const QImage &ROIArea::getCurrentImage() const
{
    static QImage empty;
    return currentImagePtr ? *currentImagePtr : empty;
}
