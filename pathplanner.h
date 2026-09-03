#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QObject>
#include <qarraydatapointer.h>
#include <qcontainerfwd.h>
#include <qlist.h>
#include <qpolygon.h>
#include <utils.h>

class PathPlanner : public QObject
{
    Q_OBJECT
  public:
    explicit PathPlanner(QObject* parent = nullptr);
    QList<drone>
    getDroneInfo(const QString& filename);
    void
    calcDroneCameraFootprint(QList<drone>& droneList);
    void
    calcFlightAltitude(QList<drone>& droneList);
    void
    calcMaximumForwardVelocity(QList<drone>& droneList);
    void
    calcDroneRelativeCapability(QList<drone>& droneList);
    QList<QPair<QPolygonF, QString>>
    decomposedROI(QPolygonF& roi, QList<drone>& droneList, const RotatedRect& mar);
    double
    calculatePolygonArea(const QPolygonF& polygon);

    QPolygonF
    suth_hodgman_polygon_clipper(QPolygonF& divider_poly, QPolygonF& target_poly);

    void
    setDroneList(QList<drone> list)
    {
        this->droneList = list;
    }

    QList<drone>
    getDroneList()
    {
        return this->droneList;
    }

    void
    setDecomposedROIs(const QList<QPair<QPolygonF, QString>>& decompROIs);
    QList<QPair<QPolygonF, QString>>
    getDecomposedROIs() const;

    // Path planning algorithm - Compute waypoints using MAR-based sweep pattern
    // Uses the Minimum Area Rectangle to bound waypoints within the sub-ROI
    // Returns: list of waypoints that are guaranteed to be inside the polygon
    QList<QPointF>
    computeWaypointsWithMAR(const QPolygonF& area, double max_x_footprint, double max_y_footprint,
                            const RotatedRect& mar);

    QPolygonF RegionOfInterest;
    QList<drone> droneList;
    QList<QPair<QPolygonF, QString>> decomposedPolygons;
  signals:
};


#endif // PATHPLANNER_H