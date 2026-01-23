// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "ROIArea.h"
#include "pathplanner.h"

#include <QDateTime>
#include <QMouseEvent>
#include <QPainter>
#include <qcontainerfwd.h>

ROIArea::ROIArea(QWidget* parent) : QWidget(parent), grahamScanner(this), pathPlanner(this)
{
    this->setMinimumHeight(650);
    this->setMinimumWidth(1000);


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

    // 2. Draw bright yellow text in the top-right corner
    painter.setPen(Qt::yellow);
    QFont font = painter.font();
    font.setPointSize(14);
    font.setBold(true);
    painter.setFont(font);

    const QString text = overlayLabel;
    const int margin = 10;
    QRect rect = overlay.rect().adjusted(margin, margin, -margin, -margin);
    painter.drawText(rect, Qt::AlignTop | Qt::AlignRight, text);

    // 3. Store overlay and update current image pointer (Option 2)
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

    // Option 2: store by value, keep pointer handle
    addOverlay(loadedImage, "");
    canDrawOnImage = false; // precisa de overlay
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
        // clearImage(); // clears image + update()
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
            qDebug() << "finalPolygon size =" << finalPolygon.size() << " showFinalPolygon =" << showFinalPolygon;
            update(); // triggers paintEvent
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

    if (overlayStack.empty())
        return;

    painter.save();

    painter.translate(panOffset);
    painter.scale(zoomFactor, zoomFactor);

    painter.drawImage(QPoint(0, 0), getOverlayStackTop().first);

    if (showFinalPolygon && finalPolygon.size() >= 3)
    {
        painter.setRenderHint(QPainter::Antialiasing);

        QPen pen(Qt::red);
        pen.setWidthF(2.0 / zoomFactor);
        painter.setPen(pen);
        QBrush brush(QColor(255, 0, 0, 80));
        painter.setBrush(brush);

        painter.drawPolygon(finalPolygon);      // image coords
        drawMinimumAreaRectangle(painter, pen); // same coords
    }

    painter.restore();
}

void
ROIArea::resizeEvent(QResizeEvent* event)
{
    // Do NOT resize any QImage here: images keep their pixel size.
    // The widget will just scale how it draws them (via paintEvent).

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
            double px = dot(p, ux);
            double py = dot(p, uy);
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
    QList<QPointF> hull;
    for (int i = 0; i < finalPolygon.size(); ++i)
    {
        hull.append(finalPolygon[i]);
    }

    // If last point equals first point, remove it
    if (!hull.isEmpty() && hull.size() > 1)
    {
        if (hull.first() == hull.last())
        {
            hull.removeLast();
        }
    }

    ROIPolygonMinAreaRect = minimumAreaRectangle(hull); // image coords
    update();                                           // request repaint so paintEvent draws it
}

void
ROIArea::drawMinimumAreaRectangle(QPainter& painter, QPen& pen)
{
    // Rectangle in image coordinates
    QPolygonF box = rotatedRectToPolygon(ROIPolygonMinAreaRect);

    pen.setColor(Qt::yellow);
    pen.setWidthF(2.0 / zoomFactor); // keep thickness with zoom
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);

    painter.drawPolygon(box); // image-space polygon, zoom/pan already applied
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
    QPolygonF roi = finalPolygon;
    QList<QPolygonF> decomposed = pathPlanner.decomposedROI(roi, drones, ROIPolygonMinAreaRect);

    // You can now use 'decomposed' as needed, e.g., store, draw, or emit a signal
    qDebug() << "Decomposed ROI into" << decomposed.size() << "sub-polygons.";
}

void
ROIArea::showDecomposedROI()
{
    if (finalPolygon.size() < 3)
    {
        qWarning() << "No valid ROI polygon to show decomposition.";
        return;
    }

    QList<drone> drones = pathPlanner.getDroneList();
    if (drones.isEmpty())
    {
        qWarning() << "No drones loaded for decomposition.";
        return;
    }

    QPolygonF roi = finalPolygon;
    QList<QPolygonF> decomposed = pathPlanner.decomposedROI(roi, drones, ROIPolygonMinAreaRect);


    // Remove the current overlay with drawings and revert to base image
    removeOverlay();
    addOverlay(getOverlayStackTop().first, "ROI Decomposition");
    QPainter painter(&getOverlayStackTop().first);
    if (!painter.isActive())
    {
        qDebug() << "drawGeoPolygonOnImage: painter not active";
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
    for (const QPolygonF& poly : decomposed)
    {
        QPen pen(colors[colorIdx % colors.size()]);
        pen.setWidth(3);
        painter.setPen(pen);
        painter.setBrush(Qt::NoBrush);
        painter.drawPolygon(poly);
        colorIdx++;
    }
    painter.end();


    update();
    emit StatusMessageChanged(tr("Decomposed ROI polygons drawn on overlay."));
}
