// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef ROIAREA_H
#define ROIAREA_H

#include "GDALHandler.h"
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
#include <pathplanner.h>
#include <qcontainerfwd.h>
#include <utils.h>

class ROIArea : public QWidget
{
    Q_OBJECT

  public:
    ROIArea(QWidget* parent = nullptr);
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
    setPolygonMinAreaRect(RotatedRect rect)
    {
        ROIPolygonMinAreaRect = rect;
    }

    inline QPolygonF
    getFinalPolygon()
    {
        return finalPolygon;
    }

    inline RotatedRect
    getROIPolygonMinAreaRect()
    {
        return ROIPolygonMinAreaRect;
    }

    void
    generateWaypointsPerDecomposedArea();
    void
    calculateMinimumAreaRectangle();
    void
    openDroneFile(const QString& filename);
    QList<Drone>
    calculateDroneCapabilities();
    void
    decomposeROI();
    void
    showDecomposedROI();

    PathPlanner&
    getPathPlanner()
    {
        return pathPlanner;
    }

    void
    showWaypoints();

    // Color analysis and contrast generation
    QColor
    analyzeBackgroundColor(const QImage& image, const QRectF& region = QRectF()) const;
    QVector<QColor>
    generateContrastingPalette(const QColor& backgroundColor, int numColors) const;
    QColor
    getContrastingTextColor(const QColor& backgroundColor) const;

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
    PathPlanner pathPlanner;
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
    QPolygonF finalPolygon;
    qreal zoomFactor = 1.0;                // 1.0 = 100%
    QPointF panOffset = QPointF(0.0, 0.0); // In pixel, image space.
    RotatedRect ROIPolygonMinAreaRect;
    QPoint lastPanPos;
    bool panning = false;
    bool canDrawOnImage = false;
    QList<QPair<Drone, QList<QPointF>>> allWaypointsPerDrone;

    // Cached contrasting color palette based on background analysis
    QVector<QColor> contrastingPalette;
    QColor contrastingTextColor;
};

#endif
