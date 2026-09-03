#ifndef GEOMETRY_CONVEXHULL_H
#define GEOMETRY_CONVEXHULL_H

#include <QList>
#include <QPointF>
#include <QPolygonF>

namespace geometry {
double cross(const QPointF& a, const QPointF& b, const QPointF& c);
double dist2(const QPointF& a, const QPointF& b);
QPolygonF convexHull(const QList<QPointF>& points);
} // namespace geometry

#endif
