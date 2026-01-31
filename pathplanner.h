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
    double
    binary_search(double relative_cap, QPolygonF pol);

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

  private:
    QPolygonF RegionOfInterest;
    QList<drone> droneList;
    QList<QPair<QPolygonF, QString>> decomposedPolygons;
  signals:
};


#endif // PATHPLANNER_H