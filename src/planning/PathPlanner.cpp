#include "planning/PathPlanner.h"

#include <utility>

PathPlanner::PathPlanner(std::unique_ptr<IDecompositionStrategy> dec,
                         std::unique_ptr<IWaypointGenerator> gen)
    : m_dec(std::move(dec)), m_gen(std::move(gen))
{
}

QList<QPair<QPolygonF, QString>>
PathPlanner::decompose(const QPolygonF& roi, const QList<Drone>& drones, const RotatedRect& mar) const
{
    return m_dec->decompose(roi, drones, mar);
}

QList<QPointF>
PathPlanner::generateWaypoints(const QPolygonF& subRoi, const Drone& d, const RotatedRect& mar) const
{
    return m_gen->generate(subRoi, d, mar);
}

void
PathPlanner::setDecomposedROIs(const QList<QPair<QPolygonF, QString>>& decompROIs)
{
    this->m_decomposed = decompROIs;
}

QList<QPair<QPolygonF, QString>>
PathPlanner::getDecomposedROIs() const
{
    return this->m_decomposed;
}
