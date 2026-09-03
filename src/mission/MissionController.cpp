#include "mission/MissionController.h"

#include "domain/GeoValidation.h"
#include "planning/BoustrophedonSweep.h"
#include "planning/StripDecomposition.h"

#include <QDebug>
#include <QStringList>

MissionController::MissionController(QObject* parent)
    : QObject(parent),
      m_pathPlanner(std::make_unique<StripDecomposition>(), std::make_unique<BoustrophedonSweep>())
{
}

QList<Drone>
MissionController::loadDrones(const QString& jsonFile)
{
    QList<Drone> listOfDrones = domain::parseDrones(jsonFile);
    m_pathPlanner.setDroneList(listOfDrones);
    m_drones = listOfDrones;
    return listOfDrones;
}

QList<Drone>
MissionController::calculateCapabilities()
{
    QList<Drone> drones = m_pathPlanner.getDroneList();

    if (drones.isEmpty())
    {
        qWarning() << "No drones loaded. Please load drone info first.";
        return drones;
    }

    domain::calcFlightAltitude(drones);
    domain::calcDroneCameraFootprint(drones);
    domain::calcMaximumForwardVelocity(drones);
    domain::calcDroneRelativeCapability(drones);

    m_pathPlanner.setDroneList(drones);
    m_drones = drones;
    return drones;
}

QList<QPair<QPolygonF, QString>>
MissionController::decompose(const QPolygonF& roiGeo, const RotatedRect& mar)
{
    if (roiGeo.size() < 3)
    {
        qWarning() << "No valid ROI polygon to decompose.";
        return QList<QPair<QPolygonF, QString>>();
    }

    QList<Drone> drones = m_pathPlanner.getDroneList();
    if (drones.isEmpty())
    {
        qWarning() << "No drones loaded for decomposition.";
        return QList<QPair<QPolygonF, QString>>();
    }

    QList<QPair<QPolygonF, QString>> decomposed = m_pathPlanner.decompose(roiGeo, drones, mar);

    for (int i = 0; i < decomposed.size(); ++i)
    {
        const QPolygonF& poly = decomposed[i].first;
        QStringList pts;
        for (const QPointF& pt : poly)
            pts << QString("(%1, %2)").arg(pt.x(), 0, 'f', 2).arg(pt.y(), 0, 'f', 2);
        QString msg = QString("Decomposed[%1] DroneID=%2: %3").arg(i).arg(decomposed[i].second).arg(pts.join(", "));
        emit statusMessageChanged(msg);
    }

    m_decomposed = decomposed;
    qInfo() << "Decomposed ROI into" << decomposed.size() << "sub-polygons.";
    return decomposed;
}

QList<QPair<Drone, QList<QPointF>>>
MissionController::generateWaypoints(const RotatedRect& mar)
{
    auto decomposedPairs = m_decomposed;
    if (decomposedPairs.isEmpty())
    {
        qWarning() << "No decomposed ROIs available for waypoint generation.";
        emit statusMessageChanged(
            tr("<font color='red'>[WARNING]: No decomposed ROIs available for waypoint generation.</font>"));
        return m_waypoints;
    }

    auto drones = m_drones;
    if (drones.isEmpty())
    {
        qWarning() << "No drones available for waypoint generation.";
        emit statusMessageChanged(
            tr("<font color='red'>[WARNING]: No drones available for waypoint generation.</font>"));
        return m_waypoints;
    }

    // === VALIDATE MAR IS IN WGS84 COORDINATES ===
    if (!domain::isRotatedRectWGS84(mar))
    {
        qCritical() << "ERROR: ROIPolygonMinAreaRect is not in WGS84 coordinates!";
        qCritical() << "  Origin:" << mar.origin;
        qCritical() << "  Call calculateMinimumAreaRectangle() first with a geo-converted polygon.";
        emit statusMessageChanged(tr("<font color='red'>[CRITICAL]: MAR not in WGS84 coordinates! Call "
                                     "calculateMinimumAreaRectangle() first.</font>"));
        return m_waypoints;
    }

    m_waypoints.clear();

    for (const auto& pair : decomposedPairs)
    {
        const QPolygonF& subROI = pair.first;
        const QString& droneId = pair.second;

        if (subROI.size() < 3)
        {
            qWarning() << "Sub-ROI for drone" << droneId << "has less than 3 vertices, skipping.";
            emit statusMessageChanged(
                tr("<font color='red'>[WARNING]: Sub-ROI for drone %1 has &lt;3 vertices, skipping.</font>")
                    .arg(droneId));
            continue;
        }

        // Sub-ROIs from decompose() are already in geo coordinates (WGS84)
        if (!domain::isPolygonWGS84(subROI))
        {
            qWarning() << "Sub-ROI for drone" << droneId << "is not in WGS84 coordinates, skipping.";
            emit statusMessageChanged(
                tr("<font color='red'>[CRITICAL]: Sub-ROI for drone %1 not in WGS84 - skipping</font>")
                    .arg(droneId));
            continue;
        }
        QPolygonF subROIWGS84 = subROI;

        // Find the drone associated with this sub-ROI
        Drone associatedDrone;
        bool droneFound = false;
        uint32_t droneIdNum = droneId.toUInt();
        for (const auto& d : drones)
        {
            if (d.id == droneIdNum)
            {
                associatedDrone = d;
                droneFound = true;
                break;
            }
        }

        if (!droneFound)
        {
            qWarning() << "Drone with ID" << droneId << "not found in drone list, skipping sub-ROI.";
            emit statusMessageChanged(
                tr("<font color='red'>[WARNING]: Drone ID %1 not found in drone list, skipping.</font>").arg(droneId));
            continue;
        }

        // === FOOTPRINT VALIDATION (already in SI units - meters) ===
        // All drone parameters are now in SI units from drones.json
        // Footprint values should be reasonable for drone imagery (1m - 500m typical)
        double footprintX_m = associatedDrone.max_x_footprint;
        double footprintY_m = associatedDrone.max_y_footprint;

        if (footprintX_m <= 0.0 || footprintY_m <= 0.0)
        {
            qCritical() << "Footprint values are zero or negative for drone" << associatedDrone.id;
            emit statusMessageChanged(
                tr("<font color='red'>[CRITICAL]: Drone %1 has invalid footprint (X=%2m, Y=%3m)</font>")
                    .arg(associatedDrone.id)
                    .arg(footprintX_m, 0, 'f', 2)
                    .arg(footprintY_m, 0, 'f', 2));
            continue;
        }

        qInfo() << "Drone" << associatedDrone.id << "footprint (SI units):";
        qInfo() << "  L_x:" << footprintX_m << "m";
        qInfo() << "  L_y:" << footprintY_m << "m";

        // Sanity check: footprint in meters should be reasonable (1m - 500m typical for drones)
        if (footprintX_m < 1.0 || footprintX_m > 500.0 || footprintY_m < 1.0 || footprintY_m > 500.0)
        {
            qWarning() << "Warning: Footprint values seem unusual for drone" << associatedDrone.id;
            qWarning() << "  Expected range: 1-500 meters, got X=" << footprintX_m << " Y=" << footprintY_m;
            emit statusMessageChanged(
                tr("<font color='orange'>[WARNING]: Drone %1 footprint unusual (X=%2m, Y=%3m) - expected 1-500m</font>")
                    .arg(associatedDrone.id)
                    .arg(footprintX_m, 0, 'f', 1)
                    .arg(footprintY_m, 0, 'f', 1));
        }

        // === COORDINATE SYSTEM VALIDATION before calling PathPlanner ===
        if (!domain::validateCoordinateSystemMatch(subROIWGS84, mar, "generateWaypointsPerDecomposedArea"))
        {
            emit statusMessageChanged(tr("<font color='red'>[CRITICAL]: Coordinate system mismatch for drone %1 - "
                                         "sub-ROI and MAR must both be WGS84</font>")
                                          .arg(associatedDrone.id));
            continue;
        }

        if (!domain::validateFootprintMeters(footprintX_m, footprintY_m, "generateWaypointsPerDecomposedArea"))
        {
            emit statusMessageChanged(
                tr("<font color='red'>[CRITICAL]: Unit mismatch - footprint not in meters for drone %1</font>")
                    .arg(associatedDrone.id));
            continue;
        }

        // Generate waypoints using MAR-based sweep pattern
        // All inputs are now in METERS / WGS84 projected coordinates
        QList<QPointF> waypoints = m_pathPlanner.generateWaypoints(subROIWGS84, associatedDrone, mar);

        if (waypoints.isEmpty())
        {
            emit statusMessageChanged(
                tr("<font color='red'>[WARNING]: No waypoints generated for drone %1 - check input validation</font>")
                    .arg(associatedDrone.id));
        }
        else
        {
            emit statusMessageChanged(
                tr("[INFO]: Drone %1 generated %2 waypoints").arg(associatedDrone.id).arg(waypoints.size()));
        }

        qInfo() << "Drone" << associatedDrone.id << "has" << waypoints.size() << "waypoints for sub-ROI";

        m_waypoints.append(qMakePair(associatedDrone, waypoints));
    }

    emit statusMessageChanged(QString("Generated waypoints for %1 sub-ROIs").arg(m_waypoints.size()));
    return m_waypoints;
}

QList<Drone>
MissionController::drones() const
{
    return m_drones;
}

QList<QPair<QPolygonF, QString>>
MissionController::decomposed() const
{
    return m_decomposed;
}

QList<QPair<Drone, QList<QPointF>>>
MissionController::waypoints() const
{
    return m_waypoints;
}

PathPlanner&
MissionController::pathPlanner()
{
    return m_pathPlanner;
}
