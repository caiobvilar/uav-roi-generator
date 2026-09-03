#include "planning/StripDecomposition.h"

#include "constants.h"
#include "geometry/PolygonClipping.h"
#include "geometry/PolygonGeometry.h"

#include <QDebug>
#include <QVector>

// Helper: Create a rectangle polygon in MAR coordinates
static QPolygonF
makeRectPoly(const RotatedRect& mar, double start, double end)
{
    QPointF o = mar.origin + start * mar.ux;
    QPointF ux = mar.ux;
    QPointF uy = mar.uy;
    double w = end - start;
    double h = mar.height;
    QPolygonF poly;
    poly << o << o + w * ux << o + w * ux + h * uy << o + h * uy << o; // closed
    return poly;
}

QList<QPair<QPolygonF, QString>>
StripDecomposition::decompose(const QPolygonF& roi, const QList<Drone>& droneList, const RotatedRect& mar) const
{
    QList<QPair<QPolygonF, QString>> result;

    if (roi.size() < 3 || droneList.isEmpty() || mar.width <= 0.0 || mar.height <= 0.0)
        return result;

    // Calculate total capability sum
    double totalCap = 0.0;
    QVector<double> capabilities;
    for (const Drone& d : droneList)
    {
        capabilities.append(d.relative_capability_score);
        totalCap += d.relative_capability_score;
    }
    if (totalCap <= 0.0)
        return result;

    // Total area of ROI
    double totalArea = geometry::shoelaceArea(roi);

    // Decompose using direct vector math
    double start = 0.0;
    double cumCap = 0.0;
    for (int i = 0; i < capabilities.size(); ++i)
    {
        cumCap += capabilities[i];
        double targetArea = cumCap / totalCap * totalArea;

        // Binary search for 'end' along MAR width
        double left = start;
        double right = mar.width;
        double end = right;
        for (int iter = 0; iter < 30; ++iter) // 30 iterations for high precision
        {
            double mid = (left + right) / 2.0;
            QPolygonF divider = makeRectPoly(mar, start, mid);
            QPolygonF clipped = geometry::sutherlandHodgmanClip(roi, divider);
            double area = geometry::shoelaceArea(clipped);
            if (area < (targetArea - constants::kEpsilonSmall))
                left = mid;
            else
                right = mid;
        }
        end = (left + right) / 2.0;

        QPolygonF divider = makeRectPoly(mar, start, end);
        QPolygonF polygon = geometry::sutherlandHodgmanClip(roi, divider);
        qInfo() << "Divider for drone" << droneList[i].id << ":" << divider;
        qInfo() << "Clipped polygon for drone" << droneList[i].id << "vertices:" << polygon.size()
                << "area:" << geometry::shoelaceArea(polygon);
        result.append({polygon, QString::number(droneList[i].id)});
        start = end;
    }

    return result;
}
