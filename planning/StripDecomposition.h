#ifndef PLANNING_STRIPDECOMPOSITION_H
#define PLANNING_STRIPDECOMPOSITION_H

#include "planning/IDecompositionStrategy.h"

class StripDecomposition : public IDecompositionStrategy
{
  public:
    QList<QPair<QPolygonF, QString>>
    decompose(const QPolygonF& roi, const QList<Drone>& droneList, const RotatedRect& mar) const override;
};

#endif // PLANNING_STRIPDECOMPOSITION_H
