#include "geometry/PolygonGeometry.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace geometry {

double dot(const QPointF& a, const QPointF& b) { return a.x() * b.x() + a.y() * b.y(); }
QPointF perp(const QPointF& v) { return QPointF(-v.y(), v.x()); }

double shoelaceArea(const QPolygonF& polygon)
{
    if (polygon.size() < 3)
        return 0.0;
    double area = 0.0;
    for (int i = 0; i < polygon.size(); ++i)
    {
        QPointF p1 = polygon[i];
        QPointF p2 = polygon[(i + 1) % polygon.size()];
        area += (p1.x() * p2.y() - p2.x() * p1.y());
    }
    return std::abs(area) / 2.0;
}

RotatedRect minimumAreaRectangle(const QList<QPointF>& hull)
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

        QPointF ux(edge.x() / len, edge.y() / len);
        QPointF uy = perp(ux);

        double minX = std::numeric_limits<double>::infinity();
        double maxX = -std::numeric_limits<double>::infinity();
        double minY = std::numeric_limits<double>::infinity();
        double maxY = -std::numeric_limits<double>::infinity();

        for (int k = 0; k < n; ++k)
        {
            const QPointF& p = hull[k];
            double px = dot(p, ux);
            double py = dot(p, uy);
            if (px < minX) minX = px;
            if (px > maxX) maxX = px;
            if (py < minY) minY = py;
            if (py > maxY) maxY = py;
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
            best.origin = minX * ux + minY * uy;

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

QPolygonF rotatedRectToPolygon(const RotatedRect& r)
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

} // namespace geometry
