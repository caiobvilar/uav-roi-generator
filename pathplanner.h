#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QObject>
#include <QPolygonF>
#include <cstdint>
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

  private:
    QPolygonF RegionOfInterest;
    QList<drone> droneList;
    QList<QPair<QPolygonF, QString>> decomposedPolygons;
  signals:
};

#endif // PATHPLANNER_H
