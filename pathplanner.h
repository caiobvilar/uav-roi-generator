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

    QPolygonF
    makeDividerPoly(const RotatedRect& left, const RotatedRect& right);
    void
    setDecomposedROIs(const QList<QPair<QPolygonF, QString>>& decompROIs);
    QList<QPair<QPolygonF, QString>>
    getDecomposedROIs() const;

    double
    compute_partitioned_area(const double theta, QTransform& EF, const QTransform& AD, const QTransform& BC,
                             QPolygonF& target, double marHeight);
    QPolygonF
    getBoundingBox(const QTransform& AD, const QTransform& EF, double height);
    QTransform
    rectToTransform(const RotatedRect& rect);
    double
    binary_search(double cap, QTransform& EF, const QTransform& AD, const QTransform& BC, QPolygonF& target,
                  double marHeight);

    // Path planning algorithm - Find longest bounding line and its slope
    // Searches consecutive edges (bounding lines) l_{k,k+1} of the polygon
    // Returns: QPair of (indices pair, slope) where indices pair is (k, k+1)
    QPair<QPair<int, int>, double>
    findLongestBoundingLineWithSlope(const QPolygonF& area);
    // Path planning algorithm - Helper: Compute internal angle at vertex i
    // Returns angle in radians between edges (v_{i-1}, v_i) and (v_i, v_{i+1})
    double
    computeInternalAngle(const QPolygonF& area, int i);

    // Path planning algorithm - Compute distance with angle adjustment
    // Returns distance from v_i to v_{i+1}, adjusted if internal angle > π/2
    // Uses L_x (max_x_footprint) for the angle adjustment
    double
    computeDistanceWithAngleAdjustment(const QPolygonF& area, int i, double max_x_footprint);

    // Path planning algorithm - Main loop computing waypoints with angle adjustment
    // while (i != k): iterates from start vertex to longest line's starting vertex
    // Returns: list of waypoints along the scanning direction
    QList<QPointF>
    computeWaypointsLoop(const QPolygonF& area, int start_i, int k, double max_x_footprint, double max_y_footprint,
                         double slope);

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