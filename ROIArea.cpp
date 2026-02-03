// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "ROIArea.h"
#include "GDALHandler.h"
#include "pathplanner.h"
#include "utils.h"

#include <QDateTime>
#include <QMouseEvent>
#include <QPainter>
#include <qcontainerfwd.h>

ROIArea::ROIArea(QWidget* parent) : QWidget(parent), grahamScanner(this), pathPlanner(this)
{
    this->setMinimumHeight(650);
    this->setMinimumWidth(1000);

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
    font.setPointSize(14);
    font.setBold(true);
    painter.setFont(font);

    const QString text = overlayLabel;
    const int margin = 10;
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
        grahamScanner.clear();
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
    grahamScanner.clear();

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
        grahamScanner.clear();
        update(); // ensure widget repaints
        break;

    case Qt::Key_Space:
        if (!isPolygonDrawn)
        {
            writing = false;
            haveStartPoint = false;

            finalPolygon = grahamScanner.ComputeHull();                             // must return QPolygonF
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
            grahamScanner.clear();
        }

        writing = true;
        grahamScanner.addPointToPolygon(p); // image coords
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
        grahamScanner.addPointToPolygon(snapped);
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
    constexpr qreal zoomStep = 1.001; // fine control
    if (panning)
    {
        QWidget::wheelEvent(event);
        return;
    }
    if (event->angleDelta().y() > 0)
        zoomFactor *= zoomStep;
    else
        zoomFactor /= zoomStep;

    zoomFactor = qBound<qreal>(0.1, zoomFactor, 20.0);
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
    if (isPolygonWGS84(polygon))
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

    QPen pen(Qt::red);
    pen.setWidthF(2.0 / zoomFactor);
    painter.setPen(pen);
    QBrush brush(QColor(255, 0, 0, 80));
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

// This could be improved using QVector2F for dot products
// or maybe just to be more consistent with the rest of the math.
double
ROIArea::dot(const QPointF& a, const QPointF& b)
{
    return a.x() * b.x() + a.y() * b.y();
}

QPointF
ROIArea::perp(const QPointF& v)
{
    return QPointF(-v.y(), v.x());
}

// hull: convex, CCW, size >= 3
RotatedRect
ROIArea::minimumAreaRectangle(const QList<QPointF>& hull)
{
    RotatedRect best{};
    const int n = hull.size();
    if (n < 3)
        return best;

    double bestArea = std::numeric_limits<double>::infinity();

    for (int i = 0; i < n; ++i)
    {
        int i2 = (i + 1) % n;
        QPointF edge = hull[i2] - hull[i];
        double len = std::hypot(edge.x(), edge.y());
        if (len == 0.0)
            continue;

        // Orthonormal basis for this orientation
        QPointF ux(edge.x() / len, edge.y() / len);
        QPointF uy = perp(ux);

        // Project *all* hull points onto this basis
        double minX = std::numeric_limits<double>::infinity();
        double maxX = -std::numeric_limits<double>::infinity();
        double minY = std::numeric_limits<double>::infinity();
        double maxY = -std::numeric_limits<double>::infinity();

        for (int k = 0; k < n; ++k)
        {
            const QPointF& p = hull[k];

            // Project point onto the orthonormal basis (ux, uy)
            double px = dot(p, ux); // coordinate along ux
            double py = dot(p, uy); // coordinate along uy

            if (px < minX)
                minX = px;
            if (px > maxX)
                maxX = px;
            if (py < minY)
                minY = py;
            if (py > maxY)
                maxY = py;
        }

        double width = maxX - minX;
        double height = maxY - minY;
        if (width <= 0.0 || height <= 0.0)
            continue;

        double area = width * height;
        if (area < bestArea)
        {
            bestArea = area;

            best.ux = ux;
            best.uy = uy;
            best.width = width;
            best.height = height;
            best.angle = std::atan2(ux.y(), ux.x());

            // Origin at (minX, minY) in world/image coords
            best.origin = minX * ux + minY * uy;

            // Optional axis‑aligned bounding rect
            QPointF o = best.origin;
            QPointF c1 = o + width * ux;
            QPointF c2 = c1 + height * uy;
            QPointF c3 = o + height * uy;
            qreal minBx = std::min({o.x(), c1.x(), c2.x(), c3.x()});
            qreal maxBx = std::max({o.x(), c1.x(), c2.x(), c3.x()});
            qreal minBy = std::min({o.y(), c1.y(), c2.y(), c3.y()});
            qreal maxBy = std::max({o.y(), c1.y(), c2.y(), c3.y()});
            best.rect = QRectF(QPointF(minBx, minBy), QPointF(maxBx, maxBy));
        }
    }

    return best;
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
    grahamScanner.clear();

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

    // Now reproject to WGS84 using GDALHandler
    QJsonDocument wgs84Doc = gdalHandler.reprojectGeoJSONPolygon(srcDoc);

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

QByteArray
ROIArea::reprojectGeoJSONPolygon(const QByteArray& srcJson) const
{
    QJsonParseError err;
    QJsonDocument srcDoc = QJsonDocument::fromJson(srcJson, &err);
    if (err.error != QJsonParseError::NoError || !srcDoc.isObject())
        return QByteArray();

    QJsonObject fc = srcDoc.object();
    if (fc.value("type").toString() != QLatin1String("FeatureCollection"))
        return QByteArray();

    QJsonArray features = fc.value("features").toArray();
    if (features.isEmpty() || !features.at(0).isObject())
        return QByteArray();

    QJsonObject feature = features.at(0).toObject();
    QJsonObject geom = feature.value("geometry").toObject();
    if (geom.value("type").toString() != QLatin1String("Polygon"))
        return QByteArray();

    QJsonArray coords = geom.value("coordinates").toArray();
    if (coords.isEmpty() || !coords.at(0).isArray())
        return QByteArray();

    QJsonArray ring = coords.at(0).toArray(); // first linear ring

    // --- set up CRS transform: EPSG:31984 -> EPSG:4326 ---
    OGRSpatialReference srcSRS, dstSRS;
    if (srcSRS.importFromEPSG(31984) != OGRERR_NONE)
        return QByteArray();
    if (dstSRS.importFromEPSG(4326) != OGRERR_NONE)
        return QByteArray();

    OGRCoordinateTransformation* ct = OGRCreateCoordinateTransformation(&srcSRS, &dstSRS);
    if (!ct)
        return QByteArray();

    QJsonArray outRing;
    for (int i = 0; i < ring.size(); ++i)
    {
        QJsonArray c = ring.at(i).toArray();
        if (c.size() < 2)
        {
            outRing.append(c);
            continue;
        }

        double x = c.at(0).toDouble(); // easting (m)
        double y = c.at(1).toDouble(); // northing (m)
        double z = 0.0;

        if (!ct->Transform(1, &x, &y, &z))
        {
            outRing.append(c); // fallback: keep original
            continue;
        }

        QJsonArray outC;
        outC.append(x); // lon
        outC.append(y); // lat
        outRing.append(outC);
    }
    OCTDestroyCoordinateTransformation(ct);

    QJsonArray outCoords;
    outCoords.append(outRing);
    geom["coordinates"] = outCoords;
    feature["geometry"] = geom;

    // update CRS properties
    QJsonObject props = feature.value("properties").toObject();
    props["crs_authority"] = QLatin1String("EPSG");
    props["crs_code"] = 4326;
    props["crs_name"] = QLatin1String("WGS 84");
    feature["properties"] = props;

    features[0] = feature;
    fc["features"] = features;

    QJsonDocument outDoc(fc);
    return outDoc.toJson(QJsonDocument::Indented);
}

QList<QPointF>
ROIArea::openGeoJSONFilePoints(const QString& filename)
{
    qInfo() << Q_FUNC_INFO << "IS THIS BEING CALLED AT ALL?????";
    return gdalHandler.loadPolygonFromGeoJSON(filename);
}

void
ROIArea::drawGeoPolygonOnImage(QImage* img, const QList<QPointF>& geoPts)
{
    qInfo() << Q_FUNC_INFO << "IS THIS BEING CALLED AT ALL?????";
    if (!img || img->isNull())
    {
        qDebug() << "drawGeoPolygonOnImage: null image";
        return;
    }

    // Convert QList<QPointF> to QPolygonF in pixel space
    QPolygonF pixPoly = gdalHandler.geoPolygonToPixels(geoPts); // change overload accordingly

    qDebug() << "geoPts count =" << geoPts.size() << "pixPoly count =" << pixPoly.size();

    // Image size
    const int w = img->width();
    const int h = img->height();

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

    QPainter p(img);
    if (!p.isActive())
    {
        qDebug() << "drawGeoPolygonOnImage: painter not active";
        return;
    }

    p.setRenderHint(QPainter::Antialiasing, true);
    QPen pen(Qt::green);
    pen.setWidth(4);
    p.setPen(pen);
    p.setBrush(Qt::NoBrush);
    p.drawPolygon(pixPoly);

    update(); // repaint widget
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

    drawGeoPolygonOnImage(&getOverlayStackTop().first, geoPts);
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
    ROIPolygonMinAreaRect = minimumAreaRectangle(hull);

    // Validate result is in WGS84
    if (!isRotatedRectWGS84(ROIPolygonMinAreaRect))
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
    QPolygonF PixelMARPoly = rotatedRectToPolygon(ROIPolygonMinAreaRect);
    QPolygonF box = gdalHandler.geoPolygonToPixels(PixelMARPoly);
    QPainter overlayPainter(&overlayImage);
    if (!overlayPainter.isActive())
        return;

    QPen pen;
    pen.setColor(Qt::yellow);
    pen.setWidthF(2.0 / zoomFactor); // keep thickness with zoom
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

QPolygonF
ROIArea::rotatedRectToPolygon(const RotatedRect& r)
{
    QPolygonF poly;
    poly.reserve(4);

    const QPointF& o = r.origin;
    const QPointF& ux = r.ux;
    const QPointF& uy = r.uy;
    qreal w = r.width;
    qreal h = r.height;

    QPointF c0 = o;
    QPointF c1 = o + w * ux;
    QPointF c2 = c1 + h * uy;
    QPointF c3 = o + h * uy;

    poly << c0 << c1 << c2 << c3;
    return poly;
}

void
ROIArea::openDroneFile(const QString& filename)
{
    QList<drone> listOfDrones = pathPlanner.getDroneInfo(filename);
    pathPlanner.setDroneList(listOfDrones);
}

QList<drone>
ROIArea::calculateDroneCapabilities()
{
    QList<drone> drones = pathPlanner.getDroneList();

    if (drones.isEmpty())
    {
        qWarning() << "No drones loaded. Please load drone info first.";
        return drones;
    }

    pathPlanner.calcFlightAltitude(drones);
    pathPlanner.calcDroneCameraFootprint(drones);
    pathPlanner.calcMaximumForwardVelocity(drones);
    pathPlanner.calcDroneRelativeCapability(drones);

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

    QList<drone> drones = pathPlanner.getDroneList();
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
    QList<drone> drones = pathPlanner.getDroneList();
    if (drones.isEmpty())
    {
        qInfo() << "No drones loaded for decomposition.";
        return;
    }
    QPolygonF pixelSpacePolygon = QPolygonF();

    QList<QPair<QPolygonF, QString>> decomposed = pathPlanner.getDecomposedROIs();
    cleanToOpenImage();
    addOverlay(getOverlayStackTop().first, "ROI Decomposition");
    QPainter painter(&getOverlayStackTop().first);
    if (!painter.isActive())
    {
        qInfo() << "drawGeoPolygonOnImage: painter not active";
        return;
    }

    painter.setRenderHint(QPainter::Antialiasing, true);
    QVector<QColor> colors = {Qt::green,
                              Qt::blue,
                              Qt::cyan,
                              Qt::magenta,
                              Qt::yellow,
                              Qt::gray,
                              Qt::darkGreen,
                              Qt::darkBlue,
                              Qt::darkCyan,
                              Qt::darkMagenta,
                              QColor(255, 165, 0),
                              QColor(128, 0, 128),
                              QColor(0, 128, 128),
                              QColor(128, 128, 0),
                              QColor(0, 255, 127),
                              QColor(70, 130, 180)};

    int colorIdx = 0;
    for (const auto& polyPair : decomposed)
    {
        QPen pen(colors[colorIdx % colors.size()]);
        pen.setWidth(3);
        painter.setPen(pen);
        painter.setBrush(Qt::NoBrush);
        if (isPolygonWGS84(polyPair.first))
        {
            emit StatusMessageChanged(tr("[ERROR]: Polygon was in WGS84 Space | Converted to Pixel Space"));
            pixelSpacePolygon = gdalHandler.geoPolygonToPixels(polyPair.first);
            qInfo() << "[ERROR]: Polygon was in WGS84 Space | Converted to Pixel Space.";
            // Pixel-space message: show all four corners
            QStringList pts;
            for (const QPointF& pt : pixelSpacePolygon)
                pts << QString("(%1, %2)").arg(pt.x(), 0, 'f', 1).arg(pt.y(), 0, 'f', 1);
            QString msgPix = QString("polPyPair.first[PIXELS]: %1").arg(pts.join(", "));
            qInfo() << msgPix;
            emit StatusMessageChanged(msgPix);
        }
        else
        {

            emit StatusMessageChanged(tr("[DEBUG]: Polygon is in Pixel Space | Nothing done"));
            qInfo() << "[Debug]: Polygon was in pixel Space | Nothing done to it.";
        }
        painter.drawPolygon(pixelSpacePolygon);

        // Draw Drone ID near the centroid of the polygon
        QPointF centroid(0, 0);
        for (const QPointF& pt : polyPair.first)
            centroid += pt;
        if (!polyPair.first.isEmpty())
            centroid /= polyPair.first.size();

        QFont font = painter.font();
        font.setPointSize(14);
        font.setBold(true);
        painter.setFont(font);
        painter.setPen(Qt::white);
        painter.drawText(centroid, polyPair.second);

        colorIdx++;
    }
    painter.end();

    update();
    emit StatusMessageChanged(tr("Decomposed ROI polygons drawn on overlay."));
}

QList<QPointF>
ROIArea::generateSweepWaypoints(const QPolygonF& subROI, const drone& d, const RotatedRect& mar) const
{
    QList<QPointF> waypoints;
    if (subROI.size() < 3)
        return waypoints;

    // 1. Find the two sides of the bounding box that are parallel to the sweep direction
    // We'll use the MAR (minimum area rectangle) for this
    QPointF A1 = mar.origin;
    QPointF B1 = mar.origin + mar.height * mar.uy;
    QPointF A2 = mar.origin + mar.width * mar.ux;
    QPointF B2 = mar.origin + mar.width * mar.ux + mar.height * mar.uy;

    // For sweeping along the width (mar.ux), sides are (A1,B1) and (A2,B2)
    double L = QLineF(A1, B1).length(); // Length of side for waypoints
    double step = d.max_y_footprint * (1.0 - SIDE_OVERLAP);
    if (step <= 0)
        step = L / 10; // fallback

    int nWaypoints = std::max(2, int(std::ceil(L / step)) + 1);

    for (int l = 0; l < nWaypoints; ++l)
    {
        double theta = (nWaypoints == 1) ? 0.5 : double(l) / (nWaypoints - 1);
        // Equation (19): w_{1,l} = A1*(1-theta) + B1*theta
        QPointF w1 = A1 * (1.0 - theta) + B1 * theta;
        QPointF w2 = A2 * (1.0 - theta) + B2 * theta;

        // Interpolate between w1 and w2 to get sweep points
        double sweepLen = QLineF(w1, w2).length();
        double dotSpacing = d.max_y_footprint * (1.0 - FORWARD_OVERLAP);
        if (dotSpacing <= 0)
            dotSpacing = sweepLen / 10; // fallback

        int nDots = std::max(2, int(std::ceil(sweepLen / dotSpacing)) + 1);
        for (int k = 0; k < nDots; ++k)
        {
            double t = (nDots == 1) ? 0.5 : double(k) / (nDots - 1);
            QPointF pt = w1 * (1.0 - t) + w2 * t;
            if (subROI.containsPoint(pt, Qt::OddEvenFill))
                waypoints.append(pt);
        }
    }
    return waypoints;
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
    if (!isRotatedRectWGS84(ROIPolygonMinAreaRect))
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
        if (!isPolygonWGS84(subROI))
        {
            qInfo() << "Converting sub-ROI from pixel to WGS84 for drone" << droneId;
            emit StatusMessageChanged(tr("[INFO]: Converting sub-ROI to WGS84 coordinates for drone %1").arg(droneId));
            subROIWGS84 = gdalHandler.polygonToGeo(subROI);

            // Verify conversion succeeded
            if (!isPolygonWGS84(subROIWGS84))
            {
                qCritical() << "ERROR: Failed to convert sub-ROI to WGS84 for drone" << droneId;
                emit StatusMessageChanged(
                    tr("<font color='red'>[CRITICAL]: Failed to convert sub-ROI to WGS84 for drone %1</font>")
                        .arg(droneId));
                continue;
            }
        }

        // Find the drone associated with this sub-ROI
        drone associatedDrone;
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
        if (!validateCoordinateSystemMatch(subROIWGS84, ROIPolygonMinAreaRect, "generateWaypointsPerDecomposedArea"))
        {
            emit StatusMessageChanged(tr("<font color='red'>[CRITICAL]: Coordinate system mismatch for drone %1 - "
                                         "sub-ROI and MAR must both be WGS84</font>")
                                          .arg(associatedDrone.id));
            continue;
        }

        if (!validateFootprintMeters(footprintX_m, footprintY_m, "generateWaypointsPerDecomposedArea"))
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

    QPainter painter(&getOverlayStackTop().first);
    if (!painter.isActive())
    {
        qWarning() << "Painter not active in showWaypoints";
        emit StatusMessageChanged(
            tr("<font color='red'>[CRITICAL]: Painter not active - cannot draw waypoints</font>"));
        return;
    }
    painter.setRenderHint(QPainter::Antialiasing, true);

    QVector<QColor> colors = {Qt::red,  Qt::green,   Qt::blue,      Qt::magenta, Qt::yellow,
                              Qt::cyan, Qt::darkRed, Qt::darkGreen, Qt::darkBlue};
    int colorIdx = 0;
    int totalWaypointsDrawn = 0;

    for (const auto& dronePair : allWaypointsPerDrone)
    {
        const drone& d = dronePair.first;
        const QList<QPointF>& waypoints = dronePair.second;

        qInfo() << "Processing drone" << d.id << "with" << waypoints.size() << "waypoints";

        if (waypoints.isEmpty())
        {
            qWarning() << "No waypoints for drone" << d.id;
            emit StatusMessageChanged(tr("<font color='orange'>[WARNING]: No waypoints for drone %1</font>").arg(d.id));
            colorIdx++;
            continue;
        }

        QColor color = colors[colorIdx % colors.size()];
        QPen pen(color);
        pen.setWidth(3);
        painter.setPen(pen);
        painter.setBrush(color);

        // Convert all waypoints from geo to pixel coordinates at once
        QPolygonF geoWaypoints;
        for (const QPointF& wp : waypoints)
            geoWaypoints << wp;

        // Validate waypoints are in WGS84 before conversion
        if (!isPolygonWGS84(geoWaypoints))
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
        const QImage& overlayImage = getOverlayStackTop().first;
        qreal imageMinDim = qMin(overlayImage.width(), overlayImage.height());
        qreal dotRadius = qBound(0.5, imageMinDim * 0.0005, 2.0); // 0.05% of image, range [0.5, 2.0]

        // Draw lines connecting consecutive waypoints (w_1 -> w_2 -> ... -> w_n)
        QPen linePen(color);
        linePen.setWidthF(qMax(0.5, dotRadius * 0.5)); // Line thinner than dots
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

        // Draw drone ID label near the first waypoint
        QPointF labelPt = pixelWaypoints.first();
        QFont font = painter.font();
        font.setPointSize(14);
        font.setBold(true);
        painter.setFont(font);
        painter.setPen(Qt::white);
        painter.drawText(labelPt + QPointF(10, -10), QString("Drone %1 (%2 pts)").arg(d.id).arg(pixelWaypoints.size()));

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
