// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "ROIArea.h"
#include "GDALHandler.h"
#include "io/GeoJSON.h"
#include "geometry/ConvexHull.h"
#include "geometry/PolygonGeometry.h"
#include "pathplanner.h"
#include "utils.h"

#include <QDateTime>
#include <QMouseEvent>
#include <QPainter>
#include <qcontainerfwd.h>

ROIArea::ROIArea(QWidget* parent) : QWidget(parent)
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
    finalPolygon = QPolygonF();
}

void
ROIArea::addOverlay(const QImage& baseImage, const QString& overlayLabel)
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
ROIArea::removeOverlay()
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
        finalPolygon = QPolygonF();
        m_grahamPoints.clear();
        emit StatusMessageChanged(tr("Drawing disabled: add new layer"));
    }

    update();
}

void
ROIArea::cleanToOpenImage()
{
    cleanOverlayStack();
    addOverlay(openImagePair.first, "");
}

void
ROIArea::cleanOverlayStack()
{
    overlayStack.clear();
    QImage baseImage(size(), QImage::Format_ARGB32_Premultiplied);
    baseImage.fill(Qt::black);
    addOverlay(baseImage, "Base Layer");
}

QPair<QImage, QString>&
ROIArea::getOverlayStackTop()
{
    return overlayStack.top();
}

bool
ROIArea::openImage(const QString& fileName)
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
ROIArea::closeImage()
{
    // Reset drawing state
    modified = false;
    writing = false;
    haveStartPoint = false;
    showFinalPolygon = false;
    isPolygonDrawn = false;
    finalPolygon = QPolygonF();
    m_grahamPoints.clear();

    // Create a new blank white image as background
    cleanOverlayStack();
    canDrawOnImage = false;
    emit StatusMessageChanged(tr("Drawing disabled: image closed"));
    update(); // repaint
    return true;
}

bool
ROIArea::saveImage(const QString& fileName, const char* fileFormat)
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
ROIArea::setPenColor(const QColor& newColor)

{
    myPenColor = newColor;
}

void
ROIArea::clearImage()
{
    QImage toClearImage = getOverlayStackTop().first;
    toClearImage.fill(Qt::black);
    modified = true;
    update();
}

void
ROIArea::keyPressEvent(QKeyEvent* event)
{
    switch (event->key())
    {
    case Qt::Key_Escape:
        showFinalPolygon = false;
        finalPolygon = QPolygonF();
        isPolygonDrawn = false;
        m_grahamPoints.clear();
        update(); // ensure widget repaints
        break;

    case Qt::Key_Space:
        if (!isPolygonDrawn)
        {
            writing = false;
            haveStartPoint = false;

            finalPolygon = geometry::convexHull(m_grahamPoints);                    // must return QPolygonF
            finalPolygon = snapPolygon(finalPolygon);                               // Snaps last point to the first
            showFinalPolygon = !finalPolygon.isEmpty() && finalPolygon.size() >= 3; // only if it’s a real polygon
            isPolygonDrawn = showFinalPolygon;
            qInfo() << "finalPolygon size =" << finalPolygon.size() << " showFinalPolygon =" << showFinalPolygon;
            drawPolygonOutline(finalPolygon); // image coords
            update();                         // triggers paintEvent
        }
        break;

    default:
        QWidget::keyPressEvent(event);
        break;
    }
}

void
ROIArea::mousePressEvent(QMouseEvent* event)
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
            finalPolygon = QPolygonF();
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
ROIArea::wheelEvent(QWheelEvent* event)
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
ROIArea::mouseMoveEvent(QMouseEvent* event)
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
ROIArea::mouseReleaseEvent(QMouseEvent* event)
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
ROIArea::paintEvent(QPaintEvent* event)
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
ROIArea::drawPolygonOutline(const QPolygonF& polygon)
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
    QColor bgColor = analyzeBackgroundColor(image, polyBounds);
    QVector<QColor> palette = generateContrastingPalette(bgColor, 1);
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
ROIArea::resizeEvent(QResizeEvent* event)
{
    QWidget::resizeEvent(event);
    update();
}

QPointF
ROIArea::toImageCoords(const QPointF& pWidget) const
{
    // Inverse of: translate(panOffset) + scale(zoomFactor, zoomFactor)
    QPointF p = pWidget;
    p -= panOffset;  // undo translation (widget pixels)
    p /= zoomFactor; // undo scaling (zoom)
    return p;        // image-space point
}

void
ROIArea::drawPointTo(const QPointF& endPoint)
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
ROIArea::drawLineTo(const QPointF& endPoint)
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
ROIArea::drawPolygon(const QPolygonF& polygon)
{
    if (!canDrawOnImage)
        return;

    // Store polygon in image coordinates
    finalPolygon = polygon;
    showFinalPolygon = (finalPolygon.size() >= 3);

    update(); // triggers paintEvent, which draws with zoom/pan
}

QPolygonF
ROIArea::snapPolygon(const QPolygonF& polygon)
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
ROIArea::clearPolygon()
{
    // Reset all polygon-related state
    showFinalPolygon = false;
    isPolygonDrawn = false;
    finalPolygon = QPolygonF();

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
ROIArea::exportPolygonGeoJSON() const
{
    if (finalPolygon.size() < 3)
    {
        qDebug() << "Your list of points has less than three elements = not a polygon.";
        return QByteArray(); // nothing to export
    }

    QPolygonF geoPolygon = gdalHandler.polygonToGeo(finalPolygon);

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
ROIArea::saveGEOJson(QByteArray& document)
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
ROIArea::openGeoJSONFilePoints(const QString& filename)
{
    qInfo() << Q_FUNC_INFO << "IS THIS BEING CALLED AT ALL?????";
    return geo::importPolygon(filename);
}

void
ROIArea::drawGeoPolygonOnCurrentOverlay(const QList<QPointF>& geoPts)
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

    // Optionally log first few vertices
    for (int i = 0; i < pixPoly.size() && i < 5; ++i)
        qDebug() << "pix" << i << pixPoly.at(i);
    for (int i = 0; i < pixPoly.size() && i < 3; ++i)
        qDebug() << "pix" << i << pixPoly.at(i);

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
ROIArea::calculateMinimumAreaRectangle()
{
    if (finalPolygon.size() < 3)
        return;

    // Remove the duplicate closing point if it exists
    QPolygonF geoPoly = gdalHandler.polygonToGeo(finalPolygon);
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
ROIArea::drawMinimumAreaRectangle()
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
    QColor bgColor = analyzeBackgroundColor(overlayImage, marBounds);
    QVector<QColor> palette = generateContrastingPalette(bgColor, 1);
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
ROIArea::openDroneFile(const QString& filename)
{
    QList<Drone> listOfDrones = domain::parseDrones(filename);
    pathPlanner.setDroneList(listOfDrones);
}

QList<Drone>
ROIArea::calculateDroneCapabilities()
{
    QList<Drone> drones = pathPlanner.getDroneList();

    if (drones.isEmpty())
    {
        qWarning() << "No drones loaded. Please load drone info first.";
        return drones;
    }

    domain::calcFlightAltitude(drones);
    domain::calcDroneCameraFootprint(drones);
    domain::calcMaximumForwardVelocity(drones);
    domain::calcDroneRelativeCapability(drones);

    pathPlanner.setDroneList(drones);
    return drones;
}

void
ROIArea::decomposeROI()
{
    // Use the current finalPolygon as the ROI
    if (finalPolygon.size() < 3)
    {
        qWarning() << "No valid ROI polygon to decompose.";
        return;
    }

    QList<Drone> drones = pathPlanner.getDroneList();
    if (drones.isEmpty())
    {
        qWarning() << "No drones loaded for decomposition.";
        return;
    }

    // Make a copy of the polygon and drone list for the decomposition
    QPolygonF roi = gdalHandler.polygonToGeo(finalPolygon);
    // Ensure polygon is in geo coordinates.
    QList<QPair<QPolygonF, QString>> decomposed = pathPlanner.decomposedROI(roi, drones, ROIPolygonMinAreaRect);

    // Emit a message for each decomposed polygon's vertices
    for (int i = 0; i < decomposed.size(); ++i)
    {
        const QPolygonF& poly = decomposed[i].first;
        QStringList pts;
        for (const QPointF& pt : poly)
            pts << QString("(%1, %2)").arg(pt.x(), 0, 'f', 2).arg(pt.y(), 0, 'f', 2);
        QString msg = QString("Decomposed[%1] DroneID=%2: %3").arg(i).arg(decomposed[i].second).arg(pts.join(", "));
        emit StatusMessageChanged(msg);
    }

    pathPlanner.setDecomposedROIs(decomposed);
    // You can now use 'decomposed' as needed, e.g., store, draw, or emit a signal
    qInfo() << "Decomposed ROI into" << decomposed.size() << "sub-polygons.";
}

void
ROIArea::showDecomposedROI()
{
    QList<Drone> drones = pathPlanner.getDroneList();
    if (drones.isEmpty())
    {
        qInfo() << "No drones loaded for decomposition.";
        return;
    }
    QPolygonF pixelSpacePolygon = QPolygonF();

    QList<QPair<QPolygonF, QString>> decomposed = pathPlanner.getDecomposedROIs();
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
    QColor bgColor = analyzeBackgroundColor(overlayImage);
    contrastingPalette = generateContrastingPalette(bgColor, qMax(decomposed.size(), 16));
    contrastingTextColor = getContrastingTextColor(bgColor);

    qInfo() << "Background color analyzed:" << bgColor.name() << "Generated" << contrastingPalette.size()
            << "contrasting colors";

    // Track placed label rectangles to avoid overlap
    QList<QRectF> placedLabels;

    int colorIdx = 0;
    for (const auto& polyPair : decomposed)
    {
        QColor color = contrastingPalette[colorIdx % contrastingPalette.size()];
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

        // Draw Drone ID near the centroid of the polygon with background
        QPointF centroid(0, 0);
        for (const QPointF& pt : pixelSpacePolygon)
            centroid += pt;
        if (!pixelSpacePolygon.isEmpty())
            centroid /= pixelSpacePolygon.size();

        QFont font = painter.font();
        font.setPointSize(constants::kFontSizeLarge);
        font.setBold(true);
        painter.setFont(font);

        QString labelText = polyPair.second;
        QFontMetrics fm(font);
        int textWidth = fm.horizontalAdvance(labelText);
        int textHeight = fm.height();

        // Calculate bounding box of the sub-ROI
        qreal roiMinX = pixelSpacePolygon.first().x(), roiMaxX = roiMinX;
        qreal roiMinY = pixelSpacePolygon.first().y(), roiMaxY = roiMinY;
        for (const QPointF& pt : pixelSpacePolygon)
        {
            roiMinX = qMin(roiMinX, pt.x());
            roiMaxX = qMax(roiMaxX, pt.x());
            roiMinY = qMin(roiMinY, pt.y());
            roiMaxY = qMax(roiMaxY, pt.y());
        }

        // Try multiple label positions OUTSIDE the sub-ROI, picking first one that fits in image
        const qreal margin = constants::kLabelMarginSmall;
        const int imgW = overlayImage.width();
        const int imgH = overlayImage.height();
        const qreal labelRotation = constants::kLabelRotationDeg; // Rotate labels to angle away from waypoints

        // Positions outside the sub-ROI bounding box
        QVector<QPointF> candidatePositions = {
            QPointF(roiMinX - margin, roiMinY - margin),                 // Above top-left (outside)
            QPointF(roiMaxX + margin, roiMinY - margin),                 // Above top-right (outside)
            QPointF(roiMinX - textWidth - margin, roiMinY + textHeight), // Left of top-left (outside)
            QPointF(roiMaxX + margin, roiMaxY),                          // Right of bottom-right (outside)
        };

        // Lambda to check if a label rect overlaps with any placed label
        auto overlapsPlacedLabels = [&](const QRectF& rect) -> bool {
            for (const QRectF& placed : placedLabels)
            {
                if (rect.intersects(placed))
                    return true;
            }
            return false;
        };

        // Lambda to check if position is valid (within bounds and no overlap)
        auto isValidPosition = [&](const QPointF& pt) -> bool {
            QRectF labelRect(pt.x() - 2, pt.y() - textHeight, textWidth + 4, textHeight + 4);
            return pt.x() >= 0 && pt.x() + textWidth <= imgW && pt.y() - textHeight >= 0 && pt.y() <= imgH &&
                   !overlapsPlacedLabels(labelRect);
        };

        QPointF labelPt = candidatePositions[colorIdx % candidatePositions.size()];
        bool foundValidPosition = false;

        // First pass: try all candidate positions
        for (const QPointF& candidate : candidatePositions)
        {
            if (isValidPosition(candidate))
            {
                labelPt = candidate;
                foundValidPosition = true;
                break;
            }
        }

        // Second pass: if no valid position, try vertical offsets to avoid overlap
        if (!foundValidPosition)
        {
            for (const QPointF& candidate : candidatePositions)
            {
                // Try shifting up/down in increments of textHeight
                for (int yOffset = 0; yOffset <= imgH; yOffset += textHeight + 5)
                {
                    QPointF shifted = candidate + QPointF(0, yOffset);
                    if (isValidPosition(shifted))
                    {
                        labelPt = shifted;
                        foundValidPosition = true;
                        break;
                    }
                    // Also try shifting up
                    shifted = candidate - QPointF(0, yOffset);
                    if (isValidPosition(shifted))
                    {
                        labelPt = shifted;
                        foundValidPosition = true;
                        break;
                    }
                }
                if (foundValidPosition)
                    break;
            }
        }

        // Final clamp to ensure label stays within image bounds
        labelPt.setX(qBound(2.0, labelPt.x(), qreal(imgW - textWidth - 2)));
        labelPt.setY(qBound(qreal(textHeight + 2), labelPt.y(), qreal(imgH - 2)));

        // Record this label's bounding rect (approximate for rotated text)
        QRectF labelRect(labelPt.x() - 2, labelPt.y() - textHeight - 2, textWidth + 4, textHeight + 4);
        placedLabels.append(labelRect);

        // Draw rotated label with background for visibility
        painter.save();
        painter.translate(labelPt);
        painter.rotate(labelRotation);

        // Draw background rect at rotated position
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(bgColor.red(), bgColor.green(), bgColor.blue(), constants::kAlphaLabelBg));
        painter.drawRect(QRectF(-2, -textHeight, textWidth + 4, textHeight + 4));

        // Draw text
        painter.setPen(color);
        painter.drawText(QPointF(0, 0), labelText);

        painter.restore();

        colorIdx++;
    }
    painter.end();

    update();
    emit StatusMessageChanged(tr("Decomposed ROI polygons drawn on overlay."));
}

void
ROIArea::generateWaypointsPerDecomposedArea()
{
    auto decomposedPairs = pathPlanner.getDecomposedROIs();
    if (decomposedPairs.isEmpty())
    {
        qWarning() << "No decomposed ROIs available for waypoint generation.";
        emit StatusMessageChanged(
            tr("<font color='red'>[WARNING]: No decomposed ROIs available for waypoint generation.</font>"));
        return;
    }

    auto drones = pathPlanner.getDroneList();
    if (drones.isEmpty())
    {
        qWarning() << "No drones available for waypoint generation.";
        emit StatusMessageChanged(
            tr("<font color='red'>[WARNING]: No drones available for waypoint generation.</font>"));
        return;
    }

    // === VALIDATE MAR IS IN WGS84 COORDINATES ===
    if (!domain::isRotatedRectWGS84(ROIPolygonMinAreaRect))
    {
        qCritical() << "ERROR: ROIPolygonMinAreaRect is not in WGS84 coordinates!";
        qCritical() << "  Origin:" << ROIPolygonMinAreaRect.origin;
        qCritical() << "  Call calculateMinimumAreaRectangle() first with a geo-converted polygon.";
        emit StatusMessageChanged(tr("<font color='red'>[CRITICAL]: MAR not in WGS84 coordinates! Call "
                                     "calculateMinimumAreaRectangle() first.</font>"));
        return;
    }

    allWaypointsPerDrone.clear();

    for (const auto& pair : decomposedPairs)
    {
        const QPolygonF& subROI = pair.first;
        const QString& droneId = pair.second;

        if (subROI.size() < 3)
        {
            qWarning() << "Sub-ROI for drone" << droneId << "has less than 3 vertices, skipping.";
            emit StatusMessageChanged(
                tr("<font color='red'>[WARNING]: Sub-ROI for drone %1 has &lt;3 vertices, skipping.</font>")
                    .arg(droneId));
            continue;
        }

        // Ensure polygon is in geo coordinates (WGS84)
        QPolygonF subROIWGS84 = subROI;
        if (!domain::isPolygonWGS84(subROI))
        {
            qInfo() << "Converting sub-ROI from pixel to WGS84 for drone" << droneId;
            emit StatusMessageChanged(tr("[INFO]: Converting sub-ROI to WGS84 coordinates for drone %1").arg(droneId));
            subROIWGS84 = gdalHandler.polygonToGeo(subROI);

            // Verify conversion succeeded
            if (!domain::isPolygonWGS84(subROIWGS84))
            {
                qCritical() << "ERROR: Failed to convert sub-ROI to WGS84 for drone" << droneId;
                emit StatusMessageChanged(
                    tr("<font color='red'>[CRITICAL]: Failed to convert sub-ROI to WGS84 for drone %1</font>")
                        .arg(droneId));
                continue;
            }
        }

        // Find the drone associated with this sub-ROI
        Drone associatedDrone;
        bool droneFound = false;
        uint32_t droneIdNum = droneId.toUInt();
        for (const auto& d : drones)
        {
            if (d.id == droneIdNum)
            {
                associatedDrone = d;
                droneFound = true;
                break;
            }
        }

        if (!droneFound)
        {
            qWarning() << "Drone with ID" << droneId << "not found in drone list, skipping sub-ROI.";
            emit StatusMessageChanged(
                tr("<font color='red'>[WARNING]: Drone ID %1 not found in drone list, skipping.</font>").arg(droneId));
            continue;
        }

        // === FOOTPRINT VALIDATION (already in SI units - meters) ===
        // All drone parameters are now in SI units from drones.json
        // Footprint values should be reasonable for drone imagery (1m - 500m typical)
        double footprintX_m = associatedDrone.max_x_footprint;
        double footprintY_m = associatedDrone.max_y_footprint;

        if (footprintX_m <= 0.0 || footprintY_m <= 0.0)
        {
            qCritical() << "Footprint values are zero or negative for drone" << associatedDrone.id;
            emit StatusMessageChanged(
                tr("<font color='red'>[CRITICAL]: Drone %1 has invalid footprint (X=%2m, Y=%3m)</font>")
                    .arg(associatedDrone.id)
                    .arg(footprintX_m, 0, 'f', 2)
                    .arg(footprintY_m, 0, 'f', 2));
            continue;
        }

        qInfo() << "Drone" << associatedDrone.id << "footprint (SI units):";
        qInfo() << "  L_x:" << footprintX_m << "m";
        qInfo() << "  L_y:" << footprintY_m << "m";

        // Sanity check: footprint in meters should be reasonable (1m - 500m typical for drones)
        if (footprintX_m < 1.0 || footprintX_m > 500.0 || footprintY_m < 1.0 || footprintY_m > 500.0)
        {
            qWarning() << "Warning: Footprint values seem unusual for drone" << associatedDrone.id;
            qWarning() << "  Expected range: 1-500 meters, got X=" << footprintX_m << " Y=" << footprintY_m;
            emit StatusMessageChanged(
                tr("<font color='orange'>[WARNING]: Drone %1 footprint unusual (X=%2m, Y=%3m) - expected 1-500m</font>")
                    .arg(associatedDrone.id)
                    .arg(footprintX_m, 0, 'f', 1)
                    .arg(footprintY_m, 0, 'f', 1));
        }

        // === COORDINATE SYSTEM VALIDATION before calling PathPlanner ===
        if (!domain::validateCoordinateSystemMatch(subROIWGS84, ROIPolygonMinAreaRect, "generateWaypointsPerDecomposedArea"))
        {
            emit StatusMessageChanged(tr("<font color='red'>[CRITICAL]: Coordinate system mismatch for drone %1 - "
                                         "sub-ROI and MAR must both be WGS84</font>")
                                          .arg(associatedDrone.id));
            continue;
        }

        if (!domain::validateFootprintMeters(footprintX_m, footprintY_m, "generateWaypointsPerDecomposedArea"))
        {
            emit StatusMessageChanged(
                tr("<font color='red'>[CRITICAL]: Unit mismatch - footprint not in meters for drone %1</font>")
                    .arg(associatedDrone.id));
            continue;
        }

        // Generate waypoints using MAR-based sweep pattern
        // All inputs are now in METERS / WGS84 projected coordinates
        QList<QPointF> waypoints =
            pathPlanner.computeWaypointsWithMAR(subROIWGS84, footprintX_m, footprintY_m, ROIPolygonMinAreaRect);

        if (waypoints.isEmpty())
        {
            emit StatusMessageChanged(
                tr("<font color='red'>[WARNING]: No waypoints generated for drone %1 - check input validation</font>")
                    .arg(associatedDrone.id));
        }
        else
        {
            emit StatusMessageChanged(
                tr("[INFO]: Drone %1 generated %2 waypoints").arg(associatedDrone.id).arg(waypoints.size()));
        }

        qInfo() << "Drone" << associatedDrone.id << "has" << waypoints.size() << "waypoints for sub-ROI";

        allWaypointsPerDrone.append(qMakePair(associatedDrone, waypoints));
    }

    emit StatusMessageChanged(QString("Generated waypoints for %1 sub-ROIs").arg(allWaypointsPerDrone.size()));
}

void
ROIArea::showWaypoints()
{
    if (allWaypointsPerDrone.isEmpty())
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
    QColor bgColor = analyzeBackgroundColor(overlayImage);
    contrastingPalette = generateContrastingPalette(bgColor, qMax(allWaypointsPerDrone.size(), 16));
    contrastingTextColor = getContrastingTextColor(bgColor);

    qInfo() << "Background color analyzed:" << bgColor.name()
            << "Luminance:" << (0.2126 * bgColor.redF() + 0.7152 * bgColor.greenF() + 0.0722 * bgColor.blueF())
            << "Generated" << contrastingPalette.size() << "contrasting colors";

    // Track placed label rectangles to avoid overlap
    QList<QRectF> placedLabels;

    int colorIdx = 0;
    int totalWaypointsDrawn = 0;

    for (const auto& dronePair : allWaypointsPerDrone)
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

        QColor color = contrastingPalette[colorIdx % contrastingPalette.size()];
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

        // Calculate bounding box of waypoints to position label outside
        qreal minX = pixelWaypoints.first().x(), maxX = minX;
        qreal minY = pixelWaypoints.first().y(), maxY = minY;
        for (const QPointF& pt : pixelWaypoints)
        {
            minX = qMin(minX, pt.x());
            maxX = qMax(maxX, pt.x());
            minY = qMin(minY, pt.y());
            maxY = qMax(maxY, pt.y());
        }

        // Position label OUTSIDE the sub-ROI bounding box, within image bounds
        QFont font = painter.font();
        font.setPointSize(constants::kFontSizeSmall);
        font.setBold(true);
        painter.setFont(font);

        QString labelText = QString("Drone %1 (%2 pts)").arg(d.id).arg(pixelWaypoints.size());
        QFontMetrics fm(font);
        int textWidth = fm.horizontalAdvance(labelText);
        int textHeight = fm.height();

        const qreal margin = constants::kLabelMarginSmall;
        const qreal labelRotation = constants::kLabelRotationDeg; // Rotate labels to angle away from waypoints
        const int imgW = overlayImage.width();
        const int imgH = overlayImage.height();

        // Positions OUTSIDE the sub-ROI bounding box (above and to the sides)
        QVector<QPointF> candidatePositions = {
            QPointF(minX - margin, minY - margin),                 // Above top-left (outside)
            QPointF(maxX + margin, minY - margin),                 // Above top-right (outside)
            QPointF(minX - textWidth - margin, minY + textHeight), // Left of top-left (outside)
            QPointF(maxX + margin, maxY),                          // Right of bottom-right (outside)
        };

        // Lambda to check if a label rect overlaps with any placed label
        auto overlapsPlacedLabels = [&](const QRectF& rect) -> bool {
            for (const QRectF& placed : placedLabels)
            {
                if (rect.intersects(placed))
                    return true;
            }
            return false;
        };

        // Lambda to check if position is valid (within bounds and no overlap)
        auto isValidPosition = [&](const QPointF& pt) -> bool {
            QRectF labelRect(pt.x() - 2, pt.y() - textHeight, textWidth + 4, textHeight + 4);
            return pt.x() >= 0 && pt.x() + textWidth <= imgW && pt.y() - textHeight >= 0 && pt.y() <= imgH &&
                   !overlapsPlacedLabels(labelRect);
        };

        QPointF labelPt = candidatePositions[colorIdx % candidatePositions.size()];
        bool foundValidPosition = false;

        // First pass: try all candidate positions
        for (const QPointF& candidate : candidatePositions)
        {
            if (isValidPosition(candidate))
            {
                labelPt = candidate;
                foundValidPosition = true;
                break;
            }
        }

        // Second pass: if no valid position, try vertical offsets to avoid overlap
        if (!foundValidPosition)
        {
            for (const QPointF& candidate : candidatePositions)
            {
                // Try shifting up/down in increments of textHeight
                for (int yOffset = 0; yOffset <= imgH; yOffset += textHeight + 5)
                {
                    QPointF shifted = candidate + QPointF(0, yOffset);
                    if (isValidPosition(shifted))
                    {
                        labelPt = shifted;
                        foundValidPosition = true;
                        break;
                    }
                    // Also try shifting up
                    shifted = candidate - QPointF(0, yOffset);
                    if (isValidPosition(shifted))
                    {
                        labelPt = shifted;
                        foundValidPosition = true;
                        break;
                    }
                }
                if (foundValidPosition)
                    break;
            }
        }

        // Final clamp to ensure label stays within image bounds
        labelPt.setX(qBound(2.0, labelPt.x(), qreal(imgW - textWidth - 2)));
        labelPt.setY(qBound(qreal(textHeight + 2), labelPt.y(), qreal(imgH - 2)));

        // Record this label's bounding rect (approximate for rotated text)
        QRectF labelRect(labelPt.x() - 2, labelPt.y() - textHeight - 2, textWidth + 4, textHeight + 4);
        placedLabels.append(labelRect);

        // Draw rotated label with background for visibility
        painter.save();
        painter.translate(labelPt);
        painter.rotate(labelRotation);

        // Draw background rect at rotated position
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor(bgColor.red(), bgColor.green(), bgColor.blue(), constants::kAlphaLabelBg));
        painter.drawRect(QRectF(-2, -textHeight, textWidth + 4, textHeight + 4));

        // Draw text
        painter.setPen(color);
        painter.drawText(QPointF(0, 0), labelText);

        painter.restore();

        totalWaypointsDrawn += pixelWaypoints.size();
        qInfo() << "Drew" << pixelWaypoints.size() << "waypoints for drone" << d.id;
        colorIdx++;
    }

    painter.end();
    update();
    emit StatusMessageChanged(QString("[INFO]: Displayed %1 waypoints for %2 drones")
                                  .arg(totalWaypointsDrawn)
                                  .arg(allWaypointsPerDrone.size()));
}

QColor
ROIArea::analyzeBackgroundColor(const QImage& image, const QRectF& region) const
{
    if (image.isNull())
        return QColor(128, 128, 128); // Default to gray if no image

    // Determine the sampling region
    QRect sampleRect;
    if (region.isValid() && !region.isEmpty())
    {
        sampleRect = region.toRect().intersected(image.rect());
    }
    else
    {
        sampleRect = image.rect();
    }

    if (sampleRect.isEmpty())
        return QColor(128, 128, 128);

    // Sample pixels at regular intervals for efficiency (don't need every pixel)
    const int sampleStep = qMax(1, qMin(sampleRect.width(), sampleRect.height()) / 50);
    qint64 totalR = 0, totalG = 0, totalB = 0;
    int sampleCount = 0;

    for (int y = sampleRect.top(); y < sampleRect.bottom(); y += sampleStep)
    {
        for (int x = sampleRect.left(); x < sampleRect.right(); x += sampleStep)
        {
            QColor pixelColor = image.pixelColor(x, y);
            totalR += pixelColor.red();
            totalG += pixelColor.green();
            totalB += pixelColor.blue();
            sampleCount++;
        }
    }

    if (sampleCount == 0)
        return QColor(128, 128, 128);

    return QColor(totalR / sampleCount, totalG / sampleCount, totalB / sampleCount);
}

QVector<QColor>
ROIArea::generateContrastingPalette(const QColor& backgroundColor, int numColors) const
{
    QVector<QColor> palette;
    palette.reserve(numColors);

    // Calculate background luminance (perceived brightness)
    // Using ITU-R BT.709 luminance formula
    double bgLuminance =
        0.2126 * backgroundColor.redF() + 0.7152 * backgroundColor.greenF() + 0.0722 * backgroundColor.blueF();

    // Get background HSV for smarter color selection
    int bgHue, bgSat, bgVal;
    backgroundColor.getHsv(&bgHue, &bgSat, &bgVal);

    // Determine if we need light or dark colors based on background
    bool needLightColors = bgLuminance < 0.5;

    // Target value (brightness) for generated colors
    int targetValue = needLightColors ? 255 : 200;
    int targetSaturation = 255; // High saturation for visibility

    // For very light backgrounds, use darker saturated colors
    if (bgLuminance > 0.7)
    {
        targetValue = constants::kAlphaLabelBg;
        targetSaturation = 255;
    }

    // Generate colors evenly distributed around the color wheel
    // Offset from background hue to avoid similar colors
    int hueOffset = (bgHue + constants::kHueOpposite) % constants::kHueFullCircle; // Start opposite to background

    for (int i = 0; i < numColors; ++i)
    {
        // Distribute hues evenly, starting from opposite of background
        int hue = (hueOffset + (i * constants::kHueFullCircle) / numColors) % constants::kHueFullCircle;

        // Avoid hues too close to the background hue (within 30 degrees)
        if (bgSat > 50) // Only if background has significant saturation
        {
            int hueDiff = qAbs(hue - bgHue);
            if (hueDiff > constants::kHueOpposite)
                hueDiff = constants::kHueFullCircle - hueDiff;
            if (hueDiff < 30)
            {
                hue = (hue + constants::kHueShiftAmount) % constants::kHueFullCircle; // Shift away from background
            }
        }

        // Vary saturation and value slightly for visual distinction
        int sat = targetSaturation - (i % 3) * 20;
        int val = targetValue - (i % 2) * 30;

        QColor color = QColor::fromHsv(hue, sat, val);

        // Final contrast check - ensure minimum contrast ratio
        double colorLuminance = 0.2126 * color.redF() + 0.7152 * color.greenF() + 0.0722 * color.blueF();
        double contrastRatio = (qMax(bgLuminance, colorLuminance) + 0.05) / (qMin(bgLuminance, colorLuminance) + 0.05);

        // If contrast is too low, adjust brightness
        if (contrastRatio < 3.0)
        {
            if (needLightColors)
                color = color.lighter(150);
            else
                color = color.darker(150);
        }

        palette.append(color);
    }

    return palette;
}

QColor
ROIArea::getContrastingTextColor(const QColor& backgroundColor) const
{
    // Calculate luminance
    double luminance =
        0.2126 * backgroundColor.redF() + 0.7152 * backgroundColor.greenF() + 0.0722 * backgroundColor.blueF();

    // Return white for dark backgrounds, black for light backgrounds
    // Using WCAG recommended threshold
    if (luminance < 0.5)
        return QColor(255, 255, 255); // White
    else
        return QColor(0, 0, 0); // Black
}
