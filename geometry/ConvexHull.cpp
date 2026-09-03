#include "geometry/ConvexHull.h"
#include <algorithm>

namespace geometry {

double cross(const QPointF& a, const QPointF& b, const QPointF& c)
{
    return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
}

double dist2(const QPointF& a, const QPointF& b)
{
    const double dx = a.x() - b.x();
    const double dy = a.y() - b.y();
    return dx * dx + dy * dy;
}

QPolygonF convexHull(const QList<QPointF>& points)
{
    QList<QPointF> pts = points;
    const int n = pts.size();
    if (n <= 1)
        return pts;

    int p0 = 0;
    for (int i = 1; i < n; ++i)
    {
        if (pts[i].y() < pts[p0].y()
            || (qFuzzyCompare(pts[i].y(), pts[p0].y()) && pts[i].x() < pts[p0].x()))
            p0 = i;
    }
    std::swap(pts[0], pts[p0]);
    const QPointF pivot = pts[0];

    std::sort(pts.begin() + 1, pts.end(), [&](const QPointF& a, const QPointF& b) {
        const double cr = cross(pivot, a, b);
        if (qFuzzyIsNull(cr))
            return dist2(pivot, a) < dist2(pivot, b);
        return cr > 0.0;
    });

    QList<QPointF> hull;
    hull.reserve(n);
    hull.push_back(pts[0]);
    if (n > 1)
        hull.push_back(pts[1]);

    for (int i = 2; i < n; ++i)
    {
        while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.last(), pts[i]) <= 0.0)
            hull.removeLast();
        hull.push_back(pts[i]);
    }
    if (hull.size() >= 3 && hull.first() != hull.last())
        hull.push_back(hull.first());

    return hull;
}

} // namespace geometry
