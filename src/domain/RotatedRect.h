#ifndef DOMAIN_ROTATEDRECT_H
#define DOMAIN_ROTATEDRECT_H

#include <QPointF>
#include <QRectF>

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

#endif // DOMAIN_ROTATEDRECT_H
