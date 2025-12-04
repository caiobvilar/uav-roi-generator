// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef ROIAREA_H
#define ROIAREA_H

#include <QColor>
#include <QImage>
#include <QKeyEvent>
#include <QList>
#include <QPoint>
#include <QWidget>
#include "GDALHandler.h"
#include "grahamscan.h"

class ROIArea : public QWidget
{
    Q_OBJECT

public:
    ROIArea(QWidget *parent = nullptr);

    bool openImage(const QString &fileName);
    bool closeImage();
    bool saveImage(const QString &fileName, const char *fileFormat);
    void setPenColor(const QColor &newColor);
    bool isModified() const { return modified; }
    QColor penColor() const { return myPenColor; }
    const QImage &getCurrentImage() const;
    void setCurrentImage(QImage *settingImage);
    void clearPolygon();
    void addOverlay(const QImage &);
public slots:
    void clearImage();

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;

private:
    GrahamScan grahamScanner;
    GDALHandler gdalHandler;
    void drawLineTo(const QPointF &endPoint);
    void drawPointTo(const QPointF &endPoint);
    void drawPolygon(const QPolygonF &polygon);
    QPolygonF snapPolygon(const QPolygonF &poly);
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
};

#endif
