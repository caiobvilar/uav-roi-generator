#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "domain/Drone.h"
#include "domain/RotatedRect.h"
#include "planning/IDecompositionStrategy.h"
#include "planning/IWaypointGenerator.h"

#include <QPair>
#include <QPolygonF>
#include <QString>
#include <qlist.h>

#include <memory>

class PathPlanner
{
  public:
    PathPlanner(std::unique_ptr<IDecompositionStrategy> dec,
                std::unique_ptr<IWaypointGenerator> gen);

    QList<QPair<QPolygonF, QString>>
    decompose(const QPolygonF& roi, const QList<Drone>& drones, const RotatedRect& mar) const;

    QList<QPointF>
    generateWaypoints(const QPolygonF& subRoi, const Drone& d, const RotatedRect& mar) const;

    void
    setDroneList(QList<Drone> list)
    {
        this->m_drones = list;
    }

    const QList<Drone>&
    getDroneList() const
    {
        return this->m_drones;
    }

    void
    setDecomposedROIs(const QList<QPair<QPolygonF, QString>>& decompROIs);

    const QList<QPair<QPolygonF, QString>>&
    getDecomposedROIs() const;

  private:
    std::unique_ptr<IDecompositionStrategy> m_dec;
    std::unique_ptr<IWaypointGenerator> m_gen;
    QList<Drone> m_drones;
    QList<QPair<QPolygonF, QString>> m_decomposed;
};

#endif // PATHPLANNER_H
