#include "pathplanner.h"
#include "utils.h"
#include <QFile>
#include <QIODevice>
#include <QList>
#include <QPolygonF>
#include <QVector>
#include <algorithm>

// Helper: Create a rectangle polygon in MAR coordinates
static QPolygonF
makeRectPoly(const RotatedRect& mar, double start, double end)
{
    QPointF o = mar.origin + start * mar.ux;
    QPointF ux = mar.ux;
    QPointF uy = mar.uy;
    double w = end - start;
    double h = mar.height;
    QPolygonF poly;
    poly << o << o + w * ux << o + w * ux + h * uy << o + h * uy << o; // closed
    return poly;
}

PathPlanner::PathPlanner(QObject* parent) : QObject{parent}
{
}

QList<drone>
PathPlanner::getDroneInfo(const QString& filename)
{
    QList<drone> results;

    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly))
    {
        qWarning() << "Failed to open file:" << filename;
        return results;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (!doc.isObject())
    {
        qWarning() << "Invalid JSON format";
        return results;
    }

    QJsonObject rootObj = doc.object();
    QJsonArray dronesArray = rootObj["drones"].toArray();

    for (int i = 0; i < dronesArray.size(); ++i)
    {
        QJsonObject droneObj = dronesArray[i].toObject();

        drone d;
        d.id = droneObj["id"].toString().split("_").last().toUInt();
        d.name = droneObj["name"].toString();

        // Battery info
        QJsonObject batteryObj = droneObj["battery"].toObject();
        d.battery_capacity = batteryObj["capacity"].toDouble();
        d.battery_current_capacity = batteryObj["current_capacity"].toString().toDouble();

        // Max velocity
        QJsonObject velocityObj = droneObj["max_velocity"].toObject();
        d.max_horizontal_velocity = velocityObj["horizontal"].toDouble();
        d.max_vertical_velocity = velocityObj["vertical"].toDouble();

        // Camera info
        QJsonObject cameraObj = droneObj["Camera"].toObject();
        d.camera_focal_length = cameraObj["focal_length"].toString().toDouble();
        d.camera_array_width = cameraObj["array_width"].toString().toDouble();
        d.camera_array_height = cameraObj["array_height"].toString().toDouble();
        d.camera_image_width = cameraObj["image_width"].toString().toDouble();
        d.camera_image_height = cameraObj["image_height"].toString().toDouble();
        d.camera_shutter_speed = cameraObj["shutter_speed"].toString().toDouble();

        results.append(d);
    }

    return results;
}

void
PathPlanner::calcFlightAltitude(QList<drone>& droneList)
{
    for (auto& d : droneList)
    {
        // GSD_x * f_l * i_x / l_x
        double h_x = (DESIRED_GSD * d.camera_focal_length * d.camera_image_width) / d.camera_array_width;

        // GSD_y * f_l * i_y / l_y
        double h_y = (DESIRED_GSD * d.camera_focal_length * d.camera_image_height) / d.camera_array_height;

        // h = min{h_x, h_y}
        d.ideal_flight_altitude = std::min(h_x, h_y);

        qDebug() << "Drone" << d.name << "- Ideal flight altitude:" << d.ideal_flight_altitude << "m";
    }
}

void
PathPlanner::calcDroneCameraFootprint(QList<drone>& droneList)
{
    for (auto& d : droneList)
    {
        // L_x = h * l_x / f_l
        d.max_x_footprint = (d.ideal_flight_altitude * d.camera_array_width) / d.camera_focal_length;

        // L_y = h * l_y / f_l
        d.max_y_footprint = (d.ideal_flight_altitude * d.camera_array_height) / d.camera_focal_length;

        qDebug() << "Drone" << d.name << "- Camera footprint:";
        qDebug() << "  L_x:" << d.max_x_footprint << "m";
        qDebug() << "  L_y:" << d.max_y_footprint << "m";
    }
}

void
PathPlanner::calcMaximumForwardVelocity(QList<drone>& droneList)
{
    for (auto& d : droneList)
    {
        // Calculate GSD (Ground Sample Distance) in the flight direction
        // GSD_y = L_y / image_height
        double gsd_y = d.max_y_footprint / d.camera_image_height;

        // Motion blur constraint: drone movement during exposure should be less than 1 GSD
        // V_max_blur = GSD_y / shutter_speed
        double maxVelocityBlur = gsd_y / d.camera_shutter_speed;

        // Overlap constraint: velocity to maintain forward overlap
        // V_max_overlap = L_y * (1 - O_f) / frame_interval
        // For now, we use the motion blur constraint as primary
        double calculatedVelocity = maxVelocityBlur;

        // Cap at drone's physical maximum horizontal velocity
        d.max_forward_velocity = qMin(calculatedVelocity, d.max_horizontal_velocity);

        qDebug() << "Drone" << d.name << "- GSD (y):" << gsd_y * 100.0 << "cm/px";
        qDebug() << "Drone" << d.name << "- Maximum forward velocity:" << d.max_forward_velocity << "m/s";
        if (calculatedVelocity > d.max_horizontal_velocity)
        {
            qDebug() << "  (capped from" << calculatedVelocity << "m/s to drone's max" << d.max_horizontal_velocity
                     << "m/s)";
        }
    }
}

void
PathPlanner::calcDroneRelativeCapability(QList<drone>& droneList)
{
    // Calculate capability for each drone: c_i = V_max^i * L_x^i
    double totalCapability = 0.0;

    for (auto& d : droneList)
    {
        d.relative_capability_score = d.max_forward_velocity * d.max_x_footprint;
        totalCapability += d.relative_capability_score;
    }

    // Calculate relative capability: c_hat_i = c_i / sum(c_j)
    if (totalCapability > 0.0)
    {
        for (auto& d : droneList)
        {
            d.relative_capability_score = d.relative_capability_score / totalCapability;
            qDebug() << "Drone" << d.name << "- Relative capability score:" << d.relative_capability_score;
        }
    }
    else
    {
        qWarning() << "Total capability is zero; cannot calculate relative capabilities";
    }
}

// Helper function to calculate polygon area using Shoelace formula
double
PathPlanner::calculatePolygonArea(const QPolygonF& polygon)
{
    if (polygon.size() < 3)
    {
        return 0.0;
    }

    double area = 0.0;
    for (int i = 0; i < polygon.size(); ++i)
    {
        QPointF p1 = polygon[i];
        QPointF p2 = polygon[(i + 1) % polygon.size()];
        area += (p1.x() * p2.y() - p2.x() * p1.y());
    }

    return qAbs(area) / 2.0;
}

void
PathPlanner::setDecomposedROIs(const QList<QPair<QPolygonF, QString>>& decompROIs)
{
    this->decomposedPolygons = decompROIs;
}

QList<QPair<QPolygonF, QString>>
PathPlanner::getDecomposedROIs() const
{
    return this->decomposedPolygons;
}

QPolygonF
PathPlanner::suth_hodgman_polygon_clipper(QPolygonF& divider_poly, QPolygonF& target_poly)
{
    // Sutherland-Hodgman polygon clipping algorithm
    QPolygonF inputPoly = target_poly;
    QPolygonF outputPoly;

    int dividerCount = divider_poly.size();
    if (dividerCount < 3 || inputPoly.size() < 3)
        return QPolygonF();

    // Helper lambda: inside test for edge (clip edge from divider_poly)
    auto inside = [](const QPointF& p, const QPointF& edgeStart, const QPointF& edgeEnd) {
        // Returns true if p is on the left side of edge (edgeStart->edgeEnd)
        return ((edgeEnd.x() - edgeStart.x()) * (p.y() - edgeStart.y()) -
                (edgeEnd.y() - edgeStart.y()) * (p.x() - edgeStart.x())) >= 0.0;
    };

    // Helper lambda: compute intersection point of two lines (p1-p2 and q1-q2)
    auto computeIntersection = [](const QPointF& p1, const QPointF& p2, const QPointF& q1,
                                  const QPointF& q2) -> QPointF {
        double a1 = p2.y() - p1.y();
        double b1 = p1.x() - p2.x();
        double c1 = a1 * p1.x() + b1 * p1.y();

        double a2 = q2.y() - q1.y();
        double b2 = q1.x() - q2.x();
        double c2 = a2 * q1.x() + b2 * q1.y();

        double det = a1 * b2 - a2 * b1;
        if (std::fabs(det) < EPSILON_TINY)
            return p2; // Lines are parallel, return p2 as fallback

        double x = (b2 * c1 - b1 * c2) / det;
        double y = (a1 * c2 - a2 * c1) / det;
        return QPointF(x, y);
    };

    // For each edge of the divider (clip) polygon
    for (int i = 0; i < dividerCount; ++i)
    {
        outputPoly.clear();
        QPointF clipEdgeStart = divider_poly[i];
        QPointF clipEdgeEnd = divider_poly[(i + 1) % dividerCount];

        int inputCount = inputPoly.size();
        if (inputCount == 0)
            break;

        for (int j = 0; j < inputCount; ++j)
        {
            QPointF curr = inputPoly[j];
            QPointF prev = inputPoly[(j + inputCount - 1) % inputCount];
            bool currInside = inside(curr, clipEdgeStart, clipEdgeEnd);
            bool prevInside = inside(prev, clipEdgeStart, clipEdgeEnd);

            if (currInside)
            {
                if (!prevInside)
                {
                    // Edge enters the clip region: add intersection
                    QPointF intersect = computeIntersection(prev, curr, clipEdgeStart, clipEdgeEnd);
                    outputPoly << intersect;
                }
                // Add current point
                outputPoly << curr;
            }
            else if (prevInside)
            {
                // Edge exits the clip region: add intersection
                QPointF intersect = computeIntersection(prev, curr, clipEdgeStart, clipEdgeEnd);
                outputPoly << intersect;
            }
            // else: both outside, add nothing
        }
        inputPoly = outputPoly;
    }

    // Optionally, ensure closed polygon (Qt polygons are usually open, but close if needed)
    if (!inputPoly.isEmpty() && inputPoly.first() != inputPoly.last())
        inputPoly << inputPoly.first();

    return inputPoly;
}

QList<QPair<QPolygonF, QString>>
PathPlanner::decomposedROI(QPolygonF& roi, QList<drone>& droneList, const RotatedRect& mar)
{
    QList<QPair<QPolygonF, QString>> result;

    if (roi.size() < 3 || droneList.isEmpty() || mar.width <= 0.0 || mar.height <= 0.0)
        return result;

    // Calculate total capability sum
    double totalCap = 0.0;
    QVector<double> capabilities;
    for (const drone& d : droneList)
    {
        capabilities.append(d.relative_capability_score);
        totalCap += d.relative_capability_score;
    }
    if (totalCap <= 0.0)
        return result;

    // Total area of ROI
    double totalArea = calculatePolygonArea(roi);

    // Decompose using direct vector math
    double start = 0.0;
    double cumCap = 0.0;
    for (int i = 0; i < capabilities.size(); ++i)
    {
        cumCap += capabilities[i];
        double targetArea = cumCap / totalCap * totalArea;

        // Binary search for 'end' along MAR width
        double left = start;
        double right = mar.width;
        double end = right;
        for (int iter = 0; iter < 30; ++iter) // 30 iterations for high precision
        {
            double mid = (left + right) / 2.0;
            QPolygonF divider = makeRectPoly(mar, start, mid);
            QPolygonF clipped = suth_hodgman_polygon_clipper(divider, roi);
            double area = calculatePolygonArea(clipped);
            if (area < (targetArea - EPSILON_SMALL))
                left = mid;
            else
                right = mid;
        }
        end = (left + right) / 2.0;

        QPolygonF divider = makeRectPoly(mar, start, end);
        QPolygonF polygon = suth_hodgman_polygon_clipper(divider, roi);
        qInfo() << "Divider for drone" << droneList[i].id << ":" << divider;
        qInfo() << "Clipped polygon for drone" << droneList[i].id << "vertices:" << polygon.size()
                << "area:" << calculatePolygonArea(polygon);
        result.append({polygon, QString::number(droneList[i].id)});
        start = end;
    }

    return result;
}

// Path planning algorithm - Compute waypoints using MAR-based sweep pattern
// Uses the Minimum Area Rectangle to bound waypoints within the sub-ROI
// PRECONDITIONS:
//   - area: polygon in WGS84/projected coordinates (meters)
//   - max_x_footprint, max_y_footprint: in METERS (not cm!)
//   - mar: RotatedRect in WGS84/projected coordinates (meters)
QList<QPointF>
PathPlanner::computeWaypointsWithMAR(const QPolygonF& area, double max_x_footprint, double max_y_footprint,
                                     const RotatedRect& mar)
{
    QList<QPointF> waypoints;

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
    if (!validateCoordinateSystemMatch(area, mar, "computeWaypointsWithMAR"))
    {
        qCritical() << "Aborting waypoint generation due to coordinate system mismatch";
        return waypoints;
    }

    // Ensure polygon is in WGS84 (geo coordinates), not pixel space
    if (!isPolygonWGS84(area))
    {
        qCritical() << "Error: Polygon must be in WGS84/projected coordinates for waypoint calculation";
        qCritical() << "  First point:" << area.first() << "- appears to be in pixel space";
        return waypoints;
    }

    // === UNIT VALIDATION ===
    // Ensure footprint values are in meters (not centimeters)
    if (!validateFootprintMeters(max_x_footprint, max_y_footprint, "computeWaypointsWithMAR"))
    {
        qCritical() << "Aborting waypoint generation due to unit mismatch";
        qCritical() << "  Hint: Convert cm to meters by dividing by 100";
        return waypoints;
    }

    // Validate MAR dimensions are reasonable for meters
    if (mar.width > MAR_SIZE_MAX || mar.height > MAR_SIZE_MAX)
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
    double spacingX = max_x_footprint * (1.0 - FORWARD_OVERLAP); // Along flight line
    double spacingY = max_y_footprint * (1.0 - SIDE_OVERLAP);    // Between flight lines

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