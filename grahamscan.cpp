#include "grahamscan.h"

GrahamScan::GrahamScan(QObject *parent)
    : QObject{parent}
{}

double GrahamScan::cross(const QPointF &a, const QPointF &b, const QPointF &c)
{
    return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
}
double GrahamScan::dist2(const QPointF &a, const QPointF &b)
{
    const double dx = a.x() - b.x();
    const double dy = a.y() - b.y();
    return dx * dx + dy * dy;
}
void GrahamScan::clear()
{
    this->polygon.clear();
}
QList<QPointF> GrahamScan::grahamScan(QList<QPointF> pts)
{
    const int n = pts.size();
    if (n <= 1)
        return pts;

    // 1) Find pivot: lowest y, then lowest x
    int p0 = 0;
    for (int i = 1; i < n; ++i) {
        if (pts[i].y() < pts[p0].y()
            || (qFuzzyCompare(pts[i].y(), pts[p0].y()) && pts[i].x() < pts[p0].x())) {
            p0 = i;
        }
    }
    std::swap(pts[0], pts[p0]);
    const QPointF pivot = pts[0];

    // 2) Sort by polar angle around pivot (CCW), farthest last if collinear
    std::sort(pts.begin() + 1, pts.end(), [&](const QPointF &a, const QPointF &b) {
        const double cr = cross(pivot, a, b);
        if (qFuzzyIsNull(cr)) // same angle
            return dist2(pivot, a) < dist2(pivot, b);
        return cr > 0.0; // a before b if left of b
    });

    // 3) Build hull
    QList<QPointF> hull;
    hull.reserve(n);
    hull.push_back(pts[0]);
    if (n > 1)
        hull.push_back(pts[1]);

    for (int i = 2; i < n; ++i) {
        while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.last(), pts[i]) <= 0.0) {
            hull.removeLast(); // pop non-left turns
        }
        hull.push_back(pts[i]);
    }
    // Ensure the polygon always ends where the first point is.
    if (hull.size() >= 3 && hull.first() != hull.last()) {
        hull.push_back(hull.first());
    }
    this->polygon = hull; // store computed hull
    return hull;
}

QPolygonF GrahamScan::ComputeHull()
{
    QPolygonF points = grahamScan(this->polygon);
    return points;
}

void GrahamScan::addPointToPolygon(QPointF point)
{
    this->polygon.append(point);
}
