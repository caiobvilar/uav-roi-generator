#ifndef MISSION_MISSIONCONTROLLER_H
#define MISSION_MISSIONCONTROLLER_H

#include "domain/Drone.h"
#include "domain/RotatedRect.h"
#include "planning/PathPlanner.h"

#include <QObject>
#include <QPolygonF>
#include <QString>

class MissionController : public QObject
{
    Q_OBJECT

  public:
    explicit MissionController(QObject* parent = nullptr);

    QList<Drone>
    loadDrones(const QString& jsonFile);
    QList<Drone>
    calculateCapabilities();
    QList<QPair<QPolygonF, QString>>
    decompose(const QPolygonF& roiGeo, const RotatedRect& mar);
    QList<QPair<Drone, QList<QPointF>>>
    generateWaypoints(const RotatedRect& mar);

    QList<QPair<QPolygonF, QString>>
    decomposed() const;
    QList<QPair<Drone, QList<QPointF>>>
    waypoints() const;

  signals:
    void
    statusMessageChanged(const QString& text);

  private:
    PathPlanner m_pathPlanner;
    QList<Drone> m_drones;
    QList<QPair<QPolygonF, QString>> m_decomposed;
    QList<QPair<Drone, QList<QPointF>>> m_waypoints;
};

#endif // MISSION_MISSIONCONTROLLER_H
