#ifndef PLANNING_IDECOMPOSITIONSTRATEGY_H
#define PLANNING_IDECOMPOSITIONSTRATEGY_H

#include "domain/Drone.h"
#include "domain/RotatedRect.h"

#include <QList>
#include <QPair>
#include <QPolygonF>
#include <QString>

class IDecompositionStrategy
{
  public:
    virtual ~IDecompositionStrategy() = default;
    virtual QList<QPair<QPolygonF, QString>>
    decompose(const QPolygonF&, const QList<Drone>&, const RotatedRect&) const = 0;
};

#endif // PLANNING_IDECOMPOSITIONSTRATEGY_H
