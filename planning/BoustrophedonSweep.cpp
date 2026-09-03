#include "planning/BoustrophedonSweep.h"

#include "constants.h"
#include "domain/GeoValidation.h"

#include <QDebug>
#include <algorithm>
#include <limits>

// Path planning algorithm - Compute waypoints using MAR-based sweep pattern
// Uses the Minimum Area Rectangle to bound waypoints within the sub-ROI
// PRECONDITIONS:
//   - area: polygon in WGS84/projected coordinates (meters)
//   - d.max_x_footprint, d.max_y_footprint: in METERS (not cm!)
//   - mar: RotatedRect in WGS84/projected coordinates (meters)
QList<QPointF>
BoustrophedonSweep::generate(const QPolygonF& area, const Drone& d, const RotatedRect& mar) const
{
    QList<QPointF> waypoints;

    double max_x_footprint = d.max_x_footprint;
    double max_y_footprint = d.max_y_footprint;

    // === INPUT VALIDATION ===
    if (area.isEmpty() || area.size() < 3)
    {
        qWarning() << "Error: Invalid polygon for waypoint generation";
        return waypoints;
    }

    if (max_x_footprint <= 0.0 || max_y_footprint <= 0.0)
    {
        qWarning() << "Error: footprint values must be positive";
        return waypoints;
    }

    // === COORDINATE SYSTEM VALIDATION ===
    // Ensure polygon and MAR are in the same coordinate system (both WGS84)
    if (!domain::validateCoordinateSystemMatch(area, mar, "computeWaypointsWithMAR"))
    {
        qCritical() << "Aborting waypoint generation due to coordinate system mismatch";
        return waypoints;
    }

    // Ensure polygon is in WGS84 (geo coordinates), not pixel space
    if (!domain::isPolygonWGS84(area))
    {
        qCritical() << "Error: Polygon must be in WGS84/projected coordinates for waypoint calculation";
        qCritical() << "  First point:" << area.first() << "- appears to be in pixel space";
        return waypoints;
    }

    // === UNIT VALIDATION ===
    // Ensure footprint values are in meters (not centimeters)
    if (!domain::validateFootprintMeters(max_x_footprint, max_y_footprint, "computeWaypointsWithMAR"))
    {
        qCritical() << "Aborting waypoint generation due to unit mismatch";
        qCritical() << "  Hint: Convert cm to meters by dividing by 100";
        return waypoints;
    }

    // Validate MAR dimensions are reasonable for meters
    if (mar.width > constants::kMarSizeMax || mar.height > constants::kMarSizeMax)
    {
        qWarning() << "Warning: MAR dimensions seem very large (width=" << mar.width << ", height=" << mar.height
                   << "). Verify units are in meters.";
    }

    qInfo() << "=== computeWaypointsWithMAR: Input validation PASSED ===";
    qInfo() << "  Polygon: WGS84, " << area.size() << " vertices";
    qInfo() << "  Footprint (m): X=" << max_x_footprint << " Y=" << max_y_footprint;
    qInfo() << "  MAR origin:" << mar.origin << " width:" << mar.width << "m height:" << mar.height << "m";

    // Compute the sub-ROI's own minimum area rectangle
    // Find bounding box of the sub-ROI in the MAR coordinate system
    double minU = std::numeric_limits<double>::infinity();
    double maxU = -std::numeric_limits<double>::infinity();
    double minV = std::numeric_limits<double>::infinity();
    double maxV = -std::numeric_limits<double>::infinity();

    for (const QPointF& pt : area)
    {
        // Project point onto MAR coordinate system
        QPointF relPt = pt - mar.origin;
        double u = relPt.x() * mar.ux.x() + relPt.y() * mar.ux.y(); // along width
        double v = relPt.x() * mar.uy.x() + relPt.y() * mar.uy.y(); // along height

        minU = qMin(minU, u);
        maxU = qMax(maxU, u);
        minV = qMin(minV, v);
        maxV = qMax(maxV, v);
    }

    qInfo() << "Sub-ROI bounds in MAR coords: U[" << minU << "," << maxU << "] V[" << minV << "," << maxV << "]";

    // Spacing calculations
    double spacingX = max_x_footprint * (1.0 - constants::kForwardOverlap); // Along flight line
    double spacingY = max_y_footprint * (1.0 - constants::kSideOverlap);    // Between flight lines

    // Get sub-ROI dimensions
    double subRoiWidth = maxU - minU;
    double subRoiHeight = maxV - minV;

    // If spacing is larger than sub-ROI, adjust to fit at least a few waypoints
    if (spacingX <= 0 || spacingX > subRoiWidth)
        spacingX = qMax(subRoiWidth / 5.0, 1.0);
    if (spacingY <= 0 || spacingY > subRoiHeight)
        spacingY = qMax(subRoiHeight / 3.0, 1.0);

    // Offset from border - use half of spacing, not half of footprint
    // This ensures waypoints start close to the edge but with some margin
    double offsetX = spacingX / 2.0;
    double offsetY = spacingY / 2.0;

    // Adjusted bounds
    double startU = minU + offsetX;
    double endU = maxU - offsetX;
    double startV = minV + offsetY;
    double endV = maxV - offsetY;

    // Make sure we have valid range - if sub-ROI is very small, use center
    if (startU >= endU)
    {
        startU = (minU + maxU) / 2.0;
        endU = startU;
    }
    if (startV >= endV)
    {
        startV = (minV + maxV) / 2.0;
        endV = startV;
    }

    qInfo() << "Sub-ROI dimensions: width=" << subRoiWidth << " height=" << subRoiHeight;
    qInfo() << "Adjusted spacing: X=" << spacingX << " Y=" << spacingY;
    qInfo() << "Waypoint bounds: U[" << startU << "," << endU << "] V[" << startV << "," << endV << "]";

    // Generate waypoints in a boustrophedon (lawn-mower) pattern
    int lineNum = 0;
    for (double v = startV; v <= endV + 0.001; v += spacingY)
    {
        // Alternate direction for each line
        bool leftToRight = (lineNum % 2 == 0);

        if (leftToRight)
        {
            for (double u = startU; u <= endU + 0.001; u += spacingX)
            {
                // Convert MAR coordinates back to geo coordinates
                QPointF geoPoint = mar.origin + u * mar.ux + v * mar.uy;

                // Only include if inside the polygon
                if (area.containsPoint(geoPoint, Qt::OddEvenFill))
                {
                    waypoints.append(geoPoint);
                }
            }
        }
        else
        {
            for (double u = endU; u >= startU - 0.001; u -= spacingX)
            {
                // Convert MAR coordinates back to geo coordinates
                QPointF geoPoint = mar.origin + u * mar.ux + v * mar.uy;

                // Only include if inside the polygon
                if (area.containsPoint(geoPoint, Qt::OddEvenFill))
                {
                    waypoints.append(geoPoint);
                }
            }
        }
        lineNum++;
    }

    qInfo() << "Generated" << waypoints.size() << "waypoints within sub-ROI using MAR sweep";
    return waypoints;
}
