#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "domain/Drone.h"
#include "domain/RotatedRect.h"

#include <QObject>
#include <QPair>
#include <QPolygonF>
#include <QString>
#include <qlist.h>

class PathPlanner : public QObject
{
    Q_OBJECT
  public:
    explicit PathPlanner(QObject* parent = nullptr);
    QList<QPair<QPolygonF, QString>>
    decomposedROI(QPolygonF& roi, QList<Drone>& droneList, const RotatedRect& mar);

    void
    setDroneList(QList<Drone> list)
    {
        this->droneList = list;
    }

    QList<Drone>
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
    QList<Drone> droneList;
    QList<QPair<QPolygonF, QString>> decomposedPolygons;
  signals:
};


#endif // PATHPLANNER_H