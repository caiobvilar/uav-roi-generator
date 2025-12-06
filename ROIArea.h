// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef ROIAREA_H
#define ROIAREA_H

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
#include <QWidget>
#include "GDALHandler.h"
#include "grahamscan.h"

struct RotatedRect
{
    QRectF rect;
    qreal angle;
    QPointF origin; // corner at (minX, minY) in image coords
    QPointF ux;     // unit vector along width (x‑axis of rect)
    QPointF uy;     // unit vector along height (y‑axis of rect)
    qreal width;
    qreal height;
    // orientation of the box in radians
};
class ROIArea : public QWidget
{
    Q_OBJECT

public:
    ROIArea(QWidget *parent = nullptr);
    QByteArray exportPolygonGeoJSON() const;
    bool openImage(const QString &fileName);
    bool closeImage();
    bool saveImage(const QString &fileName, const char *fileFormat);
    void setPenColor(const QColor &newColor);
    bool isModified() const { return modified; }
    QColor penColor() const { return myPenColor; }
    const QImage &getCurrentImage() const;
    RotatedRect minimumAreaRectangle(const QList<QPointF> &hull);
    void setCurrentImage(QImage *settingImage);
    void clearPolygon();
    void addOverlay(const QImage &);
    void removeOverlay();
    void saveGEOJson(QByteArray &document);
    QByteArray reprojectGeoJSONPolygon(const QByteArray &srcJson) const;
    QList<QPointF> openGeoJSONFilePoints(const QString &filename);
    void drawGeoPolygonOnImage(QImage *img, const QList<QPointF> &geoPts);
    void drawGeoPolygonOnCurrentOverlay(const QList<QPointF> &geoPts);
    void drawMinimumAreaRectangle(QPainter &painter, QPen &pen);
    void setPolygonMinAreaRect(RotatedRect rect) { ROIPolygonMinAreaRect = rect; }
    void calculateMinimumAreaRectangle();
public slots:
    void clearImage();
signals:
    void StatusMessageChanged(const QString &text);

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    GrahamScan grahamScanner;
    GDALHandler gdalHandler;
    QPointF toImageCoords(const QPointF &pWidget) const;
    void drawLineTo(const QPointF &endPoint);
    void drawPointTo(const QPointF &endPoint);
    void drawPolygon(const QPolygonF &polygon);
    QPolygonF rotatedRectToPolygon(const RotatedRect &r);
    QPolygonF snapPolygon(const QPolygonF &poly);
    double dot(const QPointF &a, const QPointF &b);
    QPointF perp(const QPointF &v);
    bool modified = false;
    bool writing = false;
    bool haveStartPoint = false;
    bool showFinalPolygon = false;
    bool isPolygonDrawn = false;
    QPointF startPoint;
    int penWidth = 10;
    QColor myPenColor = Qt::blue;
    QList<QImage> overlayList;
    QList<QImage *> overlayListHandles;
    QImage *currentImagePtr = nullptr;
    QPointF lastPoint;
    QList<QPointF> pointList;
    QPolygonF finalPolygon;
    qreal zoomFactor = 1.0;                //1.0 = 100%
    QPointF panOffset = QPointF(0.0, 0.0); // In pixel, image space.
    RotatedRect ROIPolygonMinAreaRect;
    QPoint lastPanPos;
    bool panning = false;
    bool canDrawOnImage = false;
};

#endif
