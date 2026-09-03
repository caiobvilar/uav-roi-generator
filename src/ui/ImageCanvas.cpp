// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "ui/ImageCanvas.h"
#include "io/GDALHandler.h"
#include "geometry/ConvexHull.h"
#include "geometry/PolygonGeometry.h"
#include "io/GeoJSON.h"
#include "ui/ColorPalette.h"
#include "ui/LabelPlacer.h"
#include "constants.h"
#include "domain/GeoValidation.h"

#include <QDateTime>
#include <QMouseEvent>
#include <QPainter>
#include <qcontainerfwd.h>

ImageCanvas::ImageCanvas(QWidget* parent)
    : QWidget(parent)
{
    this->setMinimumHeight(constants::kWidgetMinHeight);
    this->setMinimumWidth(constants::kWidgetMinWidth);

    openImagePair = qMakePair(QImage(), QString());
    // 1. Create and store an initial empty base image
    QImage baseImage(size(), QImage::Format_ARGB32_Premultiplied);
    baseImage.fill(Qt::transparent);
    addOverlay(baseImage, "Base Layer");
    // 3. Widget setup
    setAttribute(Qt::WA_StaticContents);
    setFocusPolicy(Qt::StrongFocus);
    m_finalPolygon = QPolygonF();
}

void
ImageCanvas::addOverlay(const QImage& baseImage, const QString& overlayLabel)
{
    qDebug() << "Triggered addOverlay";

    if (baseImage.isNull() || baseImage.size().isEmpty())
        return;

    // 1. Start overlay as a copy of the base image, so pixels are visible
    QImage overlay = baseImage.convertToFormat(QImage::Format_ARGB32_Premultiplied);

    QPainter painter(&overlay);
    if (!painter.isActive())
        return;

    painter.setRenderHint(QPainter::Antialiasing);


    painter.setPen(Qt::yellow);
    QFont font = painter.font();
    font.setPointSize(constants::kFontSizeLarge);
    font.setBold(true);
    painter.setFont(font);

    const QString text = overlayLabel;
    const int margin = constants::kLabelMarginLarge;
    QRect rect = overlay.rect().adjusted(margin, margin, -margin, -margin);
    painter.drawText(rect, Qt::AlignTop | Qt::AlignRight, text);


    overlayStack.push(qMakePair(overlay, text));
    canDrawOnImage = true; // allow drawing now
    emit StatusMessageChanged(tr("Drawing enabled: layer added"));
    update();
}

void
ImageCanvas::removeOverlay()
{
    qDebug() << Q_FUNC_INFO << "overlayList size =" << overlayStack.size();

    // Keep the base image (index 0) always; only remove if there's > 1
    if (overlayStack.size() == 1)
    {
        qDebug() << "OverlayStack empty!";
        return;
    }

    overlayStack.pop(); // Removes overlay that's on top.
    canDrawOnImage = (overlayStack.size() > 1);

    if (!canDrawOnImage)
    {
        // No more drawable layers: clear polygon overlay state
        showFinalPolygon = false;
        isPolygonDrawn = false;
        m_finalPolygon = QPolygonF();
        m_grahamPoints.clear();
        emit StatusMessageChanged(tr("Drawing disabled: add new layer"));
    }

    update();
}

void
ImageCanvas::cleanToOpenImage()
{
    cleanOverlayStack();
    addOverlay(openImagePair.first, "");
}

void
ImageCanvas::cleanOverlayStack()
{
    overlayStack.clear();
    QImage baseImage(size(), QImage::Format_ARGB32_Premultiplied);
    baseImage.fill(Qt::black);
    addOverlay(baseImage, "Base Layer");
}

QPair<QImage, QString>&
ImageCanvas::getOverlayStackTop()
{
    return overlayStack.top();
}

bool
ImageCanvas::openImage(const QString& fileName)
{
    QImage loadedImage;
    if (!gdalHandler.openSrcRaster(fileName))
    {
        CPLErr errClass = CPLGetLastErrorType();
        int errNo = CPLGetLastErrorNo();
        const char* msg = CPLGetLastErrorMsg();
        qDebug() << "GDAL error [" << errNo << "/" << errClass << "]:" << msg;
        return false;
    }

    loadedImage = gdalHandler.toQImage();
    if (loadedImage.isNull())
    {
        qDebug() << "Image was null after GDAL conversion";
        return false;
    }

    addOverlay(loadedImage, "");
    openImagePair = qMakePair(loadedImage, "");
    canDrawOnImage = false;
    emit StatusMessageChanged(tr("Drawing disabled: add new layer"));
    update();
    return true;
}

bool
ImageCanvas::closeImage()
{
    // Reset drawing state
    modified = false;
    writing = false;
    haveStartPoint = false;
    showFinalPolygon = false;
    isPolygonDrawn = false;
    m_finalPolygon = QPolygonF();
    m_grahamPoints.clear();

    // Create a new blank white image as background
    cleanOverlayStack();
    canDrawOnImage = false;
    emit StatusMessageChanged(tr("Drawing disabled: image closed"));
    update(); // repaint
    return true;
}

bool
ImageCanvas::saveImage(const QString& fileName, const char* fileFormat)
{
    QImage visibleImage = getOverlayStackTop().first;

    if (visibleImage.save(fileName, fileFormat))
    {
        modified = false;
        return true;
    }
    return false;
}

void
ImageCanvas::setPenColor(const QColor& newColor)

{
    myPenColor = newColor;
}

void
ImageCanvas::clearImage()
{
    QImage toClearImage = getOverlayStackTop().first;
    toClearImage.fill(Qt::black);
    modified = true;
    update();
}

void
ImageCanvas::keyPressEvent(QKeyEvent* event)
{
    switch (event->key())
    {
    case Qt::Key_Escape:
        showFinalPolygon = false;
        m_finalPolygon = QPolygonF();
        isPolygonDrawn = false;
        m_grahamPoints.clear();
        update(); // ensure widget repaints
        break;

    case Qt::Key_Space:
        if (!isPolygonDrawn)
        {
            writing = false;
            haveStartPoint = false;

            m_finalPolygon = geometry::convexHull(m_grahamPoints);                    // must return QPolygonF
            m_finalPolygon = snapPolygon(m_finalPolygon);                               // Snaps last point to the first
            showFinalPolygon = !m_finalPolygon.isEmpty() && m_finalPolygon.size() >= 3; // only if it’s a real polygon
            isPolygonDrawn = showFinalPolygon;
            qInfo() << "finalPolygon size =" << m_finalPolygon.size() << " showFinalPolygon =" << showFinalPolygon;
            drawPolygonOutline(m_finalPolygon); // image coords
            update();                         // triggers paintEvent
        }
        break;

    default:
        QWidget::keyPressEvent(event);
        break;
    }
}

void
ImageCanvas::mousePressEvent(QMouseEvent* event)
{
    QPointF pWidget = event->position();

    if (!canDrawOnImage)
    {
        QWidget::mousePressEvent(event);
        return;
    }

    if (event->button() == Qt::MiddleButton)
    {
        panning = true;
        lastPanPos = event->pos();
        setCursor(Qt::ClosedHandCursor);
        update();
        return;
    }

    // Convert to image coordinates for drawing and geometry
    QPointF p = toImageCoords(pWidget);

    if (event->button() == Qt::LeftButton)
    {
        if (showFinalPolygon)
        {
            showFinalPolygon = false;
            isPolygonDrawn = false;
            m_finalPolygon = QPolygonF();
            m_grahamPoints.clear();
        }

        writing = true;
        m_grahamPoints.append(p); // image coords
        drawPointTo(p);                     // image coords

        if (!haveStartPoint)
        {
            startPoint = p;
            lastPoint = p;
            haveStartPoint = true;
        }
        else
        {
            drawLineTo(p); // image coords
        }

        update();
    }
    else if (event->button() == Qt::RightButton && haveStartPoint)
    {
        QPointF snapped = startPoint;
        m_grahamPoints.append(snapped);
        drawLineTo(snapped);
        writing = false;
        haveStartPoint = false;
        update();
    }

    QWidget::mousePressEvent(event);
}

void
ImageCanvas::wheelEvent(QWheelEvent* event)
{
    constexpr qreal zoomStep = constants::kZoomStep; // fine control
    if (panning)
    {
        QWidget::wheelEvent(event);
        return;
    }
    if (event->angleDelta().y() > 0)
        zoomFactor *= zoomStep;
    else
        zoomFactor /= zoomStep;

    zoomFactor = qBound<qreal>(constants::kZoomMin, zoomFactor, constants::kZoomMax);
    update();
    QWidget::wheelEvent(event);
}

void
ImageCanvas::mouseMoveEvent(QMouseEvent* event)
{

    if (panning)
    {
        QPoint delta = event->pos() - lastPanPos;
        lastPanPos = event->pos();
        panOffset += delta; // pan in widget pixels
        update();
        return;
    }
    QWidget::mouseMoveEvent(event);
}

void
ImageCanvas::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::MiddleButton && panning)
    {
        panning = false;
        setCursor(Qt::ArrowCursor);
        update();
        return;
    }

    QWidget::mouseReleaseEvent(event);
}

void
ImageCanvas::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.fillRect(rect(), Qt::black);

    // Guarantee: Only the top overlay image is painted
    Q_ASSERT(!overlayStack.empty());
    if (overlayStack.empty())
        return;

    painter.save();
    painter.translate(panOffset);
    painter.scale(zoomFactor, zoomFactor);

    // Always paint only the top overlay image
    painter.drawImage(QPoint(0, 0), getOverlayStackTop().first);

    painter.restore();
}

void
ImageCanvas::drawPolygonOutline(const QPolygonF& polygon)
{
    cleanToOpenImage();
    QPolygonF toDrawPolygon;
    if (domain::isPolygonWGS84(polygon))
    {
        qInfo() << "Polygon was in WGS84";
        toDrawPolygon = gdalHandler.geoPolygonToPixels(polygon);
    }
    else
    {
        toDrawPolygon = polygon;
        qInfo() << "Polygon is in Pixel space";
    }
    QImage& image = getOverlayStackTop().first;
    QPainter painter(&image);
    if (!painter.isActive())
        return;
    painter.setRenderHint(QPainter::Antialiasing);

    // Analyze background color within the polygon's bounding box
    QRectF polyBounds = toDrawPolygon.boundingRect();
    QColor bgColor = color::analyzeBackgroundColor(image, polyBounds);
    QVector<QColor> palette = color::generateContrastingPalette(bgColor, 1);
    QColor outlineColor = palette.isEmpty() ? Qt::red : palette[0];

    QPen pen(outlineColor);
    pen.setWidthF(constants::kPenWidthDefault / zoomFactor);
    painter.setPen(pen);
    QBrush brush(QColor(outlineColor.red(), outlineColor.green(), outlineColor.blue(), constants::kAlphaRoiOutline));
    painter.setBrush(brush);

    painter.drawPolygon(toDrawPolygon); // image coords
    update();
}

void
ImageCanvas::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    update();
}

QPointF
ImageCanvas::toImageCoords(const QPointF& pWidget) const
{
    // Inverse of: translate(panOffset) + scale(zoomFactor, zoomFactor)
    QPointF p = pWidget;
    p -= panOffset;  // undo translation (widget pixels)
    p /= zoomFactor; // undo scaling (zoom)
    return p;        // image-space point
}

void
ImageCanvas::drawPointTo(const QPointF& endPoint)
{
    if (!canDrawOnImage)
        return;

    QPair<QImage, QString>& overlay = getOverlayStackTop();
    if (overlay.first.isNull())
        return;

    QPainter painter(&overlay.first);
    if (!painter.isActive())
        return;

    painter.setPen(QPen(myPenColor, penWidth, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.drawPoint(endPoint);

    modified = true;
    int rad = (penWidth / 2) + 2;
    update(QRect(endPoint.x() - rad, endPoint.y() - rad, 2 * rad + 1, 2 * rad + 1));
}

void
ImageCanvas::drawLineTo(const QPointF& endPoint)
{
    if (!canDrawOnImage)
        return;

    QPair<QImage, QString>& overlay = getOverlayStackTop();
    if (overlay.first.isNull())
        return;

    QPainter painter(&overlay.first);
    if (!painter.isActive())
        return;

    painter.setPen(QPen(penColor(), penWidth / 2.0, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    painter.drawLine(lastPoint, endPoint); // both in image coords

    int rad = (penWidth / 2) + 2;
    update(QRect(lastPoint.toPoint(), endPoint.toPoint()).normalized().adjusted(-rad, -rad, +rad, +rad));

    lastPoint = endPoint;
}

void
ImageCanvas::drawPolygon(const QPolygonF& polygon)
{
    if (!canDrawOnImage)
        return;

    // Store polygon in image coordinates
    m_finalPolygon = polygon;
    showFinalPolygon = (m_finalPolygon.size() >= 3);

    update(); // triggers paintEvent, which draws with zoom/pan
}

QPolygonF
ImageCanvas::snapPolygon(const QPolygonF& polygon)
{
    QPolygonF result = polygon;
    if (result.size() >= 2)
    {
        // make last vertex identical to first
        result[result.size() - 1] = result.first();
    }
    return result;
}

void
ImageCanvas::clearPolygon()
{
    // Reset all polygon-related state
    showFinalPolygon = false;
    isPolygonDrawn = false;
    m_finalPolygon = QPolygonF();

    // Clear drawing state
    writing = false;
    haveStartPoint = false;

    // Clear the Graham scan data
    m_grahamPoints.clear();

    removeOverlay();
    // Trigger repaint
    update();
}

QByteArray
ImageCanvas::exportPolygonGeoJSON() const
{
    if (m_finalPolygon.size() < 3)
    {
        qDebug() << "Your list of points has less than three elements = not a polygon.";
        return QByteArray(); // nothing to export
    }

    QPolygonF geoPolygon = gdalHandler.polygonToGeo(m_finalPolygon);

    // Ensure ring is closed (first == last)
    if (geoPolygon.first() != geoPolygon.last())
        geoPolygon << geoPolygon.first();

    // Create GeoJSON in the source CRS first
    QJsonArray ring;
    for (int i = 0; i < geoPolygon.size(); ++i)
    {
        const QPointF& p = geoPolygon.at(i);
        QJsonArray coord;
        coord.append(p.x()); // X (easting)
        coord.append(p.y()); // Y (northing)
        ring.append(coord);
    }

    QJsonArray coordinates;
    coordinates.append(ring);

    QJsonObject geom;
    geom["type"] = "Polygon";
    geom["coordinates"] = coordinates;

    QJsonObject properties;
    properties["timestamp"] = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);

    QJsonObject feature;
    feature["type"] = "Feature";
    feature["geometry"] = geom;
    feature["properties"] = properties;

    QJsonObject fc;
    fc["type"] = "FeatureCollection";
    fc["features"] = QJsonArray{feature};

    QJsonDocument srcDoc(fc);

    // Now reproject to WGS84 using the extracted GeoJSON IO layer
    QString srcWkt;
    GDALDataset* srcDS = gdalHandler.getDataset();
    if (srcDS)
    {
        const char* wkt = srcDS->GetProjectionRef();
        if (wkt)
            srcWkt = QString::fromUtf8(wkt);
    }
    QJsonDocument wgs84Doc = geo::reprojectToWgs84(srcDoc, srcWkt);

    if (wgs84Doc.isNull())
    {
        qWarning() << "Failed to reproject to WGS84, returning source coordinates";
        return srcDoc.toJson(QJsonDocument::Indented);
    }

    return wgs84Doc.toJson(QJsonDocument::Indented);
}

void
ImageCanvas::saveGEOJson(QByteArray& document)
{
    QString fileName =
        QFileDialog::getSaveFileName(this, tr("Save ROI as GeoJSON"), QString(), tr("GeoJSON (*.geojson *.json)"));

    if (!fileName.isEmpty())
    {
        QByteArray json = document;
        QFile f(fileName);
        if (f.open(QIODevice::WriteOnly | QIODevice::Truncate))
            f.write(json);

        f.close();
        qDebug() << "Written data to: " << fileName.toStdString();
    }
}

QList<QPointF>
ImageCanvas::openGeoJSONFilePoints(const QString& filename)
{
    return geo::importPolygon(filename);
}

void
ImageCanvas::drawGeoPolygonOnCurrentOverlay(const QList<QPointF>& geoPts)
{
    emit StatusMessageChanged(QString(Q_FUNC_INFO));
    if (overlayStack.empty())
    {
        qDebug() << "overlayList is empty.";
        return;
    }

    QImage& img = getOverlayStackTop().first;

    // Convert QList<QPointF> to QPolygonF in pixel space
    QPolygonF pixPoly = gdalHandler.geoPolygonToPixels(geoPts); // change overload accordingly

    qDebug() << "geoPts count =" << geoPts.size() << "pixPoly count =" << pixPoly.size();

    // Image size
    const int w = img.width();
    const int h = img.height();

    // Compute polygon bounding box
    if (pixPoly.isEmpty())
    {
        qDebug() << "pixPoly is empty";
        return;
    }

    qreal minX = pixPoly.first().x();
    qreal maxX = minX;
    qreal minY = pixPoly.first().y();
    qreal maxY = minY;

    for (int i = 1; i < pixPoly.size(); ++i)
    {
        const QPointF& p = pixPoly.at(i);
        minX = qMin(minX, p.x());
        maxX = qMax(maxX, p.x());
        minY = qMin(minY, p.y());
        maxY = qMax(maxY, p.y());
    }

    qDebug() << "Image size:" << w << "x" << h;
    qDebug() << "Polygon bbox: x[" << minX << "," << maxX << "] y[" << minY << "," << maxY << "]";

    if (maxX < 0 || maxY < 0 || minX >= w || minY >= h)
    {
        qDebug() << "Polygon completely outside image bounds -> not visible";
        return; // early‑out so you know it's off‑image
    }

    QPainter p(&img);
    if (!p.isActive())
    {
        qDebug() << "drawGeoPolygonOnImage: painter not active";
        return;
    }

    p.setRenderHint(QPainter::Antialiasing, true);
    QPen pen(Qt::green);
    pen.setWidth(constants::kPenWidthThick);
    p.setPen(pen);
    p.setBrush(Qt::NoBrush);
    p.drawPolygon(pixPoly);

    update(); // repaint widget
}

void
ImageCanvas::calculateMinimumAreaRectangle()
{
    if (m_finalPolygon.size() < 3)
        return;

    // Remove the duplicate closing point if it exists
    QPolygonF geoPoly = gdalHandler.polygonToGeo(m_finalPolygon);
    QList<QPointF> hull;
    for (int i = 0; i < geoPoly.size(); ++i)
    {
        hull.append(geoPoly[i]);
    }

    // If last point equals first point, remove it
    if (!hull.isEmpty() && hull.size() > 1)
    {
        if (hull.first() == hull.last())
        {
            hull.removeLast();
        }
    }

    // Compute MAR in WGS84/projected coordinates (meters)
    // The hull is already in geo coordinates from polygonToGeo() above
    ROIPolygonMinAreaRect = geometry::minimumAreaRectangle(hull);

    // Validate result is in WGS84
    if (!domain::isRotatedRectWGS84(ROIPolygonMinAreaRect))
    {
        qWarning() << "Warning: MAR origin doesn't look like WGS84 coordinates:" << ROIPolygonMinAreaRect.origin;
        emit StatusMessageChanged(tr("<font color='red'>[WARNING]: MAR origin doesn't look like WGS84 coordinates - "
                                     "check input polygon</font>"));
    }
    else
    {
        qInfo() << "MAR computed in WGS84 coordinates:";
        qInfo() << "  Origin:" << ROIPolygonMinAreaRect.origin;
        qInfo() << "  Dimensions (m): width=" << ROIPolygonMinAreaRect.width
                << " height=" << ROIPolygonMinAreaRect.height;
        emit StatusMessageChanged(tr("[INFO]: MAR computed in WGS84 - width=%1m, height=%2m")
                                      .arg(ROIPolygonMinAreaRect.width, 0, 'f', 2)
                                      .arg(ROIPolygonMinAreaRect.height, 0, 'f', 2));
    }
}

void
ImageCanvas::drawMinimumAreaRectangle()
{
    // Create a new overlay image based on the current top overlay
    addOverlay(getOverlayStackTop().first, "Min Area Rect Overlay");
    QImage& overlayImage = getOverlayStackTop().first;
    // Rectangle in image coordinates
    QPolygonF PixelMARPoly = geometry::rotatedRectToPolygon(ROIPolygonMinAreaRect);
    QPolygonF box = gdalHandler.geoPolygonToPixels(PixelMARPoly);
    QPainter overlayPainter(&overlayImage);
    if (!overlayPainter.isActive())
        return;

    // Analyze background color within the MAR region
    QRectF marBounds = box.boundingRect();
    QColor bgColor = color::analyzeBackgroundColor(overlayImage, marBounds);
    QVector<QColor> palette = color::generateContrastingPalette(bgColor, 1);
    QColor marColor = palette.isEmpty() ? Qt::yellow : palette[0];

    QPen pen;
    pen.setColor(marColor);
    pen.setWidthF(constants::kPenWidthDefault / zoomFactor); // keep thickness with zoom
    overlayPainter.setPen(pen);
    overlayPainter.setBrush(Qt::NoBrush);
    overlayPainter.setRenderHint(QPainter::Antialiasing, true);
    overlayPainter.drawPolygon(box);
    overlayPainter.end();

    // WGS84 message
    QString msgWGS = QString("MinAreaRect[WGS84]: origin=(%1, %2), width=%3, height=%4, angle=%5 rad")
                         .arg(ROIPolygonMinAreaRect.origin.x(), 0, 'f', 2)
                         .arg(ROIPolygonMinAreaRect.origin.y(), 0, 'f', 2)
                         .arg(ROIPolygonMinAreaRect.width, 0, 'f', 2)
                         .arg(ROIPolygonMinAreaRect.height, 0, 'f', 2)
                         .arg(ROIPolygonMinAreaRect.angle, 0, 'f', 4);
    emit StatusMessageChanged(msgWGS);

    // Pixel-space message: show all four corners
    QStringList pts;
    for (const QPointF& pt : box)
        pts << QString("(%1, %2)").arg(pt.x(), 0, 'f', 1).arg(pt.y(), 0, 'f', 1);
    QString msgPix = QString("MinAreaRect[PIXELS]: %1").arg(pts.join(", "));
    emit StatusMessageChanged(msgPix);

    update(); // request repaint so paintEvent draws it
}

void
ImageCanvas::showDecomposedROI(const QList<QPair<QPolygonF, QString>>& decomposed)
{
    QPolygonF pixelSpacePolygon = QPolygonF();

    cleanToOpenImage();
    addOverlay(getOverlayStackTop().first, "ROI Decomposition");

    QImage& overlayImage = getOverlayStackTop().first;
    QPainter painter(&overlayImage);
    if (!painter.isActive())
    {
        qInfo() << "drawGeoPolygonOnImage: painter not active";
        return;
    }

    painter.setRenderHint(QPainter::Antialiasing, true);

    // Analyze background and generate contrasting palette
    QColor bgColor = color::analyzeBackgroundColor(overlayImage);
    QVector<QColor> palette = color::generateContrastingPalette(bgColor, qMax(decomposed.size(), 16));

    qInfo() << "Background color analyzed:" << bgColor.name() << "Generated" << palette.size()
            << "contrasting colors";

    LabelPlacer placer(overlayImage.width(), overlayImage.height(), constants::kLabelMarginSmall,
                       constants::kLabelRotationDeg);

    int colorIdx = 0;
    for (const auto& polyPair : decomposed)
    {
        QColor color = palette[colorIdx % palette.size()];
        QPen pen(color);
        pen.setWidth(constants::kPenWidthMedium);
        painter.setPen(pen);

        // Semi-transparent fill to differentiate sub-ROIs
        QColor fillColor = color;
        fillColor.setAlpha(constants::kAlphaSubroiFill); // 50/255 ≈ 20% opacity
        painter.setBrush(fillColor);

        // Convert polygon to pixel space
        if (domain::isPolygonWGS84(polyPair.first))
        {
            pixelSpacePolygon = gdalHandler.geoPolygonToPixels(polyPair.first);
            qInfo() << "[INFO]: Sub-ROI polygon converted from WGS84 to Pixel Space.";
        }
        else
        {
            pixelSpacePolygon = polyPair.first;
            qInfo() << "[INFO]: Sub-ROI polygon already in Pixel Space.";
        }

        // Remove duplicate closing point if present (Qt drawPolygon auto-closes)
        if (pixelSpacePolygon.size() >= 2 && pixelSpacePolygon.first() == pixelSpacePolygon.last())
        {
            pixelSpacePolygon.removeLast();
        }

        painter.drawPolygon(pixelSpacePolygon);

        QFont font = painter.font();
        font.setPointSize(constants::kFontSizeLarge);
        font.setBold(true);
        painter.setFont(font);

        QString labelText = polyPair.second;
        QFontMetrics fm(font);

        QPointF labelPt = placer.place(pixelSpacePolygon.boundingRect(), labelText, fm);
        drawLabel(painter, labelText, color, bgColor, constants::kAlphaLabelBg, labelPt, fm,
                  constants::kLabelRotationDeg);

        colorIdx++;
    }
    painter.end();

    update();
    emit StatusMessageChanged(tr("Decomposed ROI polygons drawn on overlay."));
}

void
ImageCanvas::showWaypoints(const QList<QPair<Drone, QList<QPointF>>>& waypoints)
{
    if (waypoints.isEmpty())
    {
        qWarning() << "No waypoints to display. Run generateWaypointsPerDecomposedArea() first.";
        emit StatusMessageChanged(tr("<font color='red'>[WARNING]: No waypoints to display. Run "
                                     "generateWaypointsPerDecomposedArea() first.</font>"));
        return;
    }

    cleanToOpenImage();
    addOverlay(getOverlayStackTop().first, "Waypoints Overlay");

    QImage& overlayImage = getOverlayStackTop().first;
    QPainter painter(&overlayImage);
    if (!painter.isActive())
    {
        qWarning() << "Painter not active in showWaypoints";
        emit StatusMessageChanged(
            tr("<font color='red'>[CRITICAL]: Painter not active - cannot draw waypoints</font>"));
        return;
    }
    painter.setRenderHint(QPainter::Antialiasing, true);

    // Analyze background and generate contrasting palette
    QColor bgColor = color::analyzeBackgroundColor(overlayImage);
    QVector<QColor> palette = color::generateContrastingPalette(bgColor, qMax(waypoints.size(), 16));

    qInfo() << "Background color analyzed:" << bgColor.name()
            << "Luminance:" << (0.2126 * bgColor.redF() + 0.7152 * bgColor.greenF() + 0.0722 * bgColor.blueF())
            << "Generated" << palette.size() << "contrasting colors";

    LabelPlacer placer(overlayImage.width(), overlayImage.height(), constants::kLabelMarginSmall,
                       constants::kLabelRotationDeg);

    int colorIdx = 0;
    int totalWaypointsDrawn = 0;

    for (const auto& dronePair : waypoints)
    {
        const Drone& d = dronePair.first;
        const QList<QPointF>& waypoints = dronePair.second;

        qInfo() << "Processing drone" << d.id << "with" << waypoints.size() << "waypoints";

        if (waypoints.isEmpty())
        {
            qWarning() << "No waypoints for drone" << d.id;
            emit StatusMessageChanged(tr("<font color='orange'>[WARNING]: No waypoints for drone %1</font>").arg(d.id));
            colorIdx++;
            continue;
        }

        QColor color = palette[colorIdx % palette.size()];
        QPen pen(color);
        pen.setWidth(constants::kPenWidthMedium);
        painter.setPen(pen);
        painter.setBrush(color);

        // Convert all waypoints from geo to pixel coordinates at once
        QPolygonF geoWaypoints;
        for (const QPointF& wp : waypoints)
            geoWaypoints << wp;

        // Validate waypoints are in WGS84 before conversion
        if (!domain::isPolygonWGS84(geoWaypoints))
        {
            qCritical() << "Waypoints for drone" << d.id << "are not in WGS84 coordinates!";
            emit StatusMessageChanged(tr("<font color='red'>[CRITICAL]: Waypoints for drone %1 not in WGS84 - cannot "
                                         "convert to pixels</font>")
                                          .arg(d.id));
            colorIdx++;
            continue;
        }

        QPolygonF pixelWaypoints = gdalHandler.geoPolygonToPixels(geoWaypoints);

        qInfo() << "Converted" << geoWaypoints.size() << "geo waypoints to" << pixelWaypoints.size()
                << "pixel waypoints";

        if (pixelWaypoints.isEmpty())
        {
            qWarning() << "Pixel waypoints empty after conversion for drone" << d.id;
            emit StatusMessageChanged(
                tr("<font color='red'>[WARNING]: Conversion to pixels failed for drone %1</font>").arg(d.id));
            // Debug: print first few geo waypoints
            for (int i = 0; i < qMin(3, waypoints.size()); ++i)
                qInfo() << "  Geo waypoint" << i << ":" << waypoints[i];
            colorIdx++;
            continue;
        }

        // Debug: print first few pixel waypoints
        for (int i = 0; i < qMin(3, pixelWaypoints.size()); ++i)
            qInfo() << "  Pixel waypoint" << i << ":" << pixelWaypoints[i];

        // Calculate dot radius - use 1 pixel for dense waypoint grids
        // For very large images, cap at 2 pixels to remain visible
        qreal imageMinDim = qMin(overlayImage.width(), overlayImage.height());
        qreal dotRadius =
            qBound(constants::kDotRadiusMin, imageMinDim * constants::kDotRadiusScale, constants::kDotRadiusMax); // 0.05% of image, range [0.5, 2.0]

        // Draw lines connecting consecutive waypoints (w_1 -> w_2 -> ... -> w_n)
        QPen linePen(color);
        linePen.setWidthF(qMax(constants::kDotRadiusMin, dotRadius * 0.5)); // Line thinner than dots
        painter.setPen(linePen);
        painter.setBrush(Qt::NoBrush);
        for (int i = 0; i < pixelWaypoints.size() - 1; ++i)
        {
            painter.drawLine(pixelWaypoints[i], pixelWaypoints[i + 1]);
        }

        // Draw each waypoint as a tiny dot (on top of lines)
        painter.setPen(pen);
        painter.setBrush(color);
        for (const QPointF& pixelPt : pixelWaypoints)
        {
            painter.drawEllipse(pixelPt, dotRadius, dotRadius);
        }

        // Position label OUTSIDE the sub-ROI bounding box, within image bounds
        QFont font = painter.font();
        font.setPointSize(constants::kFontSizeSmall);
        font.setBold(true);
        painter.setFont(font);

        QString labelText = QString("Drone %1 (%2 pts)").arg(d.id).arg(pixelWaypoints.size());
        QFontMetrics fm(font);

        QPointF labelPt = placer.place(pixelWaypoints.boundingRect(), labelText, fm);
        drawLabel(painter, labelText, color, bgColor, constants::kAlphaLabelBg, labelPt, fm,
                  constants::kLabelRotationDeg);

        totalWaypointsDrawn += pixelWaypoints.size();
        qInfo() << "Drew" << pixelWaypoints.size() << "waypoints for drone" << d.id;
        colorIdx++;
    }

    painter.end();
    update();
    emit StatusMessageChanged(QString("[INFO]: Displayed %1 waypoints for %2 drones")
                                  .arg(totalWaypointsDrawn)
                                  .arg(waypoints.size()));
}

