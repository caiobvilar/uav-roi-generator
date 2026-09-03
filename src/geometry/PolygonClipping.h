#ifndef GEOMETRY_POLYGONCLIPPING_H
#define GEOMETRY_POLYGONCLIPPING_H

#include <QPolygonF>

namespace geometry {
QPolygonF sutherlandHodgmanClip(const QPolygonF& subject, const QPolygonF& clip);
} // namespace geometry

#endif
