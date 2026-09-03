#include "geometry/PolygonClipping.h"
#include <cmath>

namespace geometry {

QPolygonF sutherlandHodgmanClip(QPolygonF subject, const QPolygonF& clip)
{
    // Sutherland-Hodgman polygon clipping algorithm
    QPolygonF inputPoly = subject;
    QPolygonF outputPoly;

    int clipCount = clip.size();
    if (clipCount < 3 || inputPoly.size() < 3)
        return QPolygonF();

    // Helper lambda: inside test for edge (clip edge from clip)
    auto inside = [](const QPointF& p, const QPointF& edgeStart, const QPointF& edgeEnd) {
        // Returns true if p is on the left side of edge (edgeStart->edgeEnd)
        return ((edgeEnd.x() - edgeStart.x()) * (p.y() - edgeStart.y()) -
                (edgeEnd.y() - edgeStart.y()) * (p.x() - edgeStart.x())) >= 0.0;
    };

    // Helper lambda: compute intersection point of two lines (p1-p2 and q1-q2)
    auto computeIntersection = [](const QPointF& p1, const QPointF& p2, const QPointF& q1,
                                  const QPointF& q2) -> QPointF {
        double a1 = p2.y() - p1.y();
        double b1 = p1.x() - p2.x();
        double c1 = a1 * p1.x() + b1 * p1.y();

        double a2 = q2.y() - q1.y();
        double b2 = q1.x() - q2.x();
        double c2 = a2 * q1.x() + b2 * q1.y();

        double det = a1 * b2 - a2 * b1;
        if (std::fabs(det) < 1e-12)
            return p2; // Lines are parallel, return p2 as fallback

        double x = (b2 * c1 - b1 * c2) / det;
        double y = (a1 * c2 - a2 * c1) / det;
        return QPointF(x, y);
    };

    // For each edge of the clip polygon
    for (int i = 0; i < clipCount; ++i)
    {
        outputPoly.clear();
        QPointF clipEdgeStart = clip[i];
        QPointF clipEdgeEnd = clip[(i + 1) % clipCount];

        int inputCount = inputPoly.size();
        if (inputCount == 0)
            break;

        for (int j = 0; j < inputCount; ++j)
        {
            QPointF curr = inputPoly[j];
            QPointF prev = inputPoly[(j + inputCount - 1) % inputCount];
            bool currInside = inside(curr, clipEdgeStart, clipEdgeEnd);
            bool prevInside = inside(prev, clipEdgeStart, clipEdgeEnd);

            if (currInside)
            {
                if (!prevInside)
                {
                    // Edge enters the clip region: add intersection
                    QPointF intersect = computeIntersection(prev, curr, clipEdgeStart, clipEdgeEnd);
                    outputPoly << intersect;
                }
                // Add current point
                outputPoly << curr;
            }
            else if (prevInside)
            {
                // Edge exits the clip region: add intersection
                QPointF intersect = computeIntersection(prev, curr, clipEdgeStart, clipEdgeEnd);
                outputPoly << intersect;
            }
            // else: both outside, add nothing
        }
        inputPoly = outputPoly;
    }

    // Optionally, ensure closed polygon (Qt polygons are usually open, but close if needed)
    if (!inputPoly.isEmpty() && inputPoly.first() != inputPoly.last())
        inputPoly << inputPoly.first();

    return inputPoly;
}

} // namespace geometry
