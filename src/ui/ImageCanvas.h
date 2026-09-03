// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef IMAGECANVAS_H
#define IMAGECANVAS_H

#include "io/GDALHandler.h"
#include "domain/RotatedRect.h"
#include "domain/Drone.h"

#include <QColor>
#include <QDebug>
#include <QFileDialog>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QKeyEvent>
#include <QList>
#include <QPoint>
#include <QStack>
#include <QWidget>
#include <qcontainerfwd.h>

class ImageCanvas : public QWidget
{
    Q_OBJECT

  public:
    ImageCanvas(QWidget* parent = nullptr);
    QByteArray
    exportPolygonGeoJSON() const;
    bool
    openImage(const QString& fileName);
    bool
    closeImage();
    bool
    saveImage(const QString& fileName, const char* fileFormat);
    void
    setPenColor(const QColor& newColor);

    bool
    isModified() const
    {
        return modified;
    }

    QColor
    penColor() const
    {
        return myPenColor;
    }

    QPair<QImage, QString>&
    getOverlayStackTop();
    void
    cleanOverlayStack();
    void
    clearPolygon();
    void
    addOverlay(const QImage&, const QString&);
    void
    removeOverlay();
    void
    cleanToOpenImage();
    void
    saveGEOJson(QByteArray& document);
    QList<QPointF>
    openGeoJSONFilePoints(const QString& filename);
    void
    drawGeoPolygonOnCurrentOverlay(const QList<QPointF>& geoPts);
    void
    drawMinimumAreaRectangle();

    void
    drawPolygonOutline(const QPolygonF& polygon);

    void
    setPolygonMinAreaRect(const RotatedRect& rect)
    {
        ROIPolygonMinAreaRect = rect;
    }

    inline QPolygonF
    getFinalPolygon()
    {
        return m_finalPolygon;
    }

    inline RotatedRect
    getROIPolygonMinAreaRect()
    {
        return ROIPolygonMinAreaRect;
    }

    void
    calculateMinimumAreaRectangle();
    void
    showDecomposedROI(const QList<QPair<QPolygonF, QString>>& decomposed);
    void
    showWaypoints(const QList<QPair<Drone, QList<QPointF>>>& waypoints);

    QPolygonF
    finalPolygonGeo() const
    {
        return gdalHandler.polygonToGeo(m_finalPolygon);
    }

  public slots:
    void
    clearImage();
  signals:
    void
    StatusMessageChanged(const QString& text);

  protected:
    void
    mousePressEvent(QMouseEvent* event) override;
    void
    mouseMoveEvent(QMouseEvent* event) override;
    void
    mouseReleaseEvent(QMouseEvent* event) override;
    void
    paintEvent(QPaintEvent* event) override;
    void
    resizeEvent(QResizeEvent* event) override;
    void
    keyPressEvent(QKeyEvent* event) override;
    void
    wheelEvent(QWheelEvent* event) override;


  private:
    QList<QPointF> m_grahamPoints;
    GDALHandler gdalHandler;
    QPointF
    toImageCoords(const QPointF& pWidget) const;
    void
    drawLineTo(const QPointF& endPoint);
    void
    drawPointTo(const QPointF& endPoint);
    void
    drawPolygon(const QPolygonF& polygon);
    QPolygonF
    snapPolygon(const QPolygonF& poly);
    bool modified = false;
    bool writing = false;
    bool haveStartPoint = false;
    bool showFinalPolygon = false;
    bool isPolygonDrawn = false;
    QPointF startPoint;
    int penWidth = 10;
    QColor myPenColor = Qt::blue;
    QStack<QPair<QImage, QString>> overlayStack;
    QPair<QImage, QString> openImagePair;
    QPointF lastPoint;
    QList<QPointF> pointList;
    QPolygonF m_finalPolygon;
    qreal zoomFactor = 1.0;                // 1.0 = 100%
    QPointF panOffset = QPointF(0.0, 0.0); // In pixel, image space.
    RotatedRect ROIPolygonMinAreaRect;
    QPoint lastPanPos;
    bool panning = false;
    bool canDrawOnImage = false;
};

#endif
