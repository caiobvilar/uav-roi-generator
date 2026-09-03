#ifndef PLANNING_BOUSTROPHEDONSWEEP_H
#define PLANNING_BOUSTROPHEDONSWEEP_H

#include "planning/IWaypointGenerator.h"

class BoustrophedonSweep : public IWaypointGenerator
{
  public:
    QList<QPointF>
    generate(const QPolygonF& area, const Drone& d, const RotatedRect& mar) const override;
};

#endif // PLANNING_BOUSTROPHEDONSWEEP_H
