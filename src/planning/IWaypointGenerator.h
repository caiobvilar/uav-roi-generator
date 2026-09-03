#ifndef PLANNING_IWAYPOINTGENERATOR_H
#define PLANNING_IWAYPOINTGENERATOR_H

#include "domain/Drone.h"
#include "domain/RotatedRect.h"

#include <QList>
#include <QPointF>
#include <QPolygonF>

class IWaypointGenerator
{
  public:
    virtual ~IWaypointGenerator() = default;
    virtual QList<QPointF>
    generate(const QPolygonF&, const Drone&, const RotatedRect&) const = 0;
};

#endif // PLANNING_IWAYPOINTGENERATOR_H
