#ifndef GEOMETRY_POLYGONGEOMETRY_H
#define GEOMETRY_POLYGONGEOMETRY_H

#include "domain/RotatedRect.h"
#include <QList>
#include <QPointF>
#include <QPolygonF>

namespace geometry {
double dot(const QPointF& a, const QPointF& b);
QPointF perp(const QPointF& v);
double shoelaceArea(const QPolygonF& polygon);
RotatedRect minimumAreaRectangle(const QList<QPointF>& hull);
QPolygonF rotatedRectToPolygon(const RotatedRect& r);
} // namespace geometry

#endif
