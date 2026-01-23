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
            qDebug() << "finalPolygon size =" << finalPolygon.size() << " showFinalPolygon =" << showFinalPolygon;
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

    painter.drawPolygon(finalPolygon); // image coords
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
}

void

ROIArea::drawMinimumAreaRectangle()
{
    // Create a new overlay image based on the current top overlay
    addOverlay(getOverlayStackTop().first, "Min Area Rect Overlay");
    QImage& overlayImage = getOverlayStackTop().first;
    // Rectangle in image coordinates
    QPolygonF box = rotatedRectToPolygon(ROIPolygonMinAreaRect);
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
    QPolygonF roi = finalPolygon;
    QList<QPair<QPolygonF, QString>> decomposed = pathPlanner.decomposedROI(roi, drones, ROIPolygonMinAreaRect);
    pathPlanner.setDecomposedROIs(decomposed);
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
    QList<QPair<QPolygonF, QString>> decomposed = pathPlanner.decomposedROI(roi, drones, ROIPolygonMinAreaRect);
    cleanToOpenImage();
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
    for (const auto& polyPair : decomposed)
    {
        QPen pen(colors[colorIdx % colors.size()]);
        pen.setWidth(3);
        painter.setPen(pen);
        painter.setBrush(Qt::NoBrush);
        painter.drawPolygon(polyPair.first);

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
    // Parameters
    QList<QPointF> waypoints;
    if (subROI.size() < 3)
        return waypoints;

    // Sweep direction: along mar.uy (shorter side)
    const QPointF& origin = mar.origin;
    const QPointF& ux = mar.ux; // long side direction
    const QPointF& uy = mar.uy; // short side direction
    double width = mar.width;
    double height = mar.height;

    // Choose sweep axis: sweep along uy, lines parallel to ux
    double step = d.max_y_footprint * (1.0 - SIDE_OVERLAP);
    if (step <= 0.0)
        step = 1.0; // fallback
    int nSteps = std::max(1, int(std::ceil(height / step)));

    // For each sweep line, compute intersection with subROI
    bool zigzag = false;
    for (int i = 0; i < nSteps; ++i)
    {
        double offset = i * step;
        QPointF sweepStart = origin + offset * uy;
        QPointF sweepEnd = sweepStart + width * ux;

        // Build the sweep line
        QLineF sweepLine(sweepStart, sweepEnd);

        // Find intersection points with subROI edges
        QList<QPointF> intersections;
        for (int j = 0; j < subROI.size(); ++j)
        {
            QPointF p1 = subROI[j];
            QPointF p2 = subROI[(j + 1) % subROI.size()];
            QLineF edge(p1, p2);
            QPointF intersectPt;
            QLineF::IntersectionType type = sweepLine.intersects(edge, &intersectPt);
            if (type == QLineF::BoundedIntersection)
            {
                intersections.append(intersectPt);
            }
        }
        // Only even number of intersections expected for convex polygon
        if (intersections.size() >= 2)
        {
            // Sort by projection along ux
            std::sort(intersections.begin(), intersections.end(), [&](const QPointF& a, const QPointF& b) {
                return QPointF::dotProduct(a - sweepStart, ux) < QPointF::dotProduct(b - sweepStart, ux);
            });
            // Add waypoints for this sweep (start to end or end to start for zigzag)
            if (!zigzag)
                waypoints << intersections[0] << intersections[1];
            else
                waypoints << intersections[1] << intersections[0];
            zigzag = !zigzag;
        }
    }
    return waypoints;
}

void
ROIArea::showWaypoints()
{
    QList<QPair<QPolygonF, QString>> decomposedPairs = pathPlanner.getDecomposedROIs();
    QList<QPolygonF> decomposedROIs;
    for (const auto& pair : decomposedPairs)
    {
        decomposedROIs.append(pair.first);
    }
    QList<QPointF> listOfWaypoints;
    QList<drone> drones = pathPlanner.getDroneList();
    for (const drone& d : drones)
    {
        for (const QPolygonF& subROI : decomposedROIs)
        {
            QList<QPointF> waypoints = generateSweepWaypoints(subROI, d, ROIPolygonMinAreaRect);
            listOfWaypoints.append(waypoints);
        }
    }
    addOverlay(getOverlayStackTop().first, "Waypoints Overlay");
    QPainter painter(&getOverlayStackTop().first);
    if (!painter.isActive())
        return;
    painter.setRenderHint(QPainter::Antialiasing, true);
    QPen pen(Qt::blue);
    pen.setWidthF(2.0 / zoomFactor); // keep thickness with zoom
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);
    for (const QPointF& wp : listOfWaypoints)
    {
        painter.drawEllipse(wp, 3.0 / zoomFactor, 3.0 / zoomFactor); // small circle
    }
    painter.end();
    update(); // request repaint so paintEvent draws it
}