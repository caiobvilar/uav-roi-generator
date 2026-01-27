#include "pathplanner.h"
#include "ROIArea.h"
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

        qDebug() << "Drone" << d.name << "- Ideal flight altitude:" << d.ideal_flight_altitude << "cm";
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
        qDebug() << "  L_x:" << d.max_x_footprint << "cm";
        qDebug() << "  L_y:" << d.max_y_footprint << "cm";
    }
}

void
PathPlanner::calcMaximumForwardVelocity(QList<drone>& droneList)
{
    for (auto& d : droneList)
    {
        // V_max = L_y * (1 - O_f) / s_h
        d.max_forward_velocity = (d.max_y_footprint * (1.0 - FORWARD_OVERLAP)) / d.camera_shutter_speed;

        qDebug() << "Drone" << d.name << "- Maximum forward velocity:" << d.max_forward_velocity << "cm/s";
        qDebug() << "  (with" << (FORWARD_OVERLAP * 100) << "% forward overlap)";
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

QList<QPair<QPolygonF, QString>>
PathPlanner::decomposedROI(QPolygonF& roi, QList<drone>& droneList, const RotatedRect& mar)
{
    QList<QPair<QPolygonF, QString>> result;

    if (roi.size() < 3 || droneList.isEmpty() || mar.width <= 0.0 || mar.height <= 0.0)
        return result;

    double totalArea = calculatePolygonArea(roi);
    if (totalArea <= 0.0)
        return result;

    // Normalize capabilities to [0,1] sum
    double totalCap = 0.0;
    for (const drone& d : droneList)
        totalCap += d.relative_capability_score;
    if (totalCap <= 0.0)
        return result;

    QVector<double> relCaps(droneList.size());
    for (int i = 0; i < droneList.size(); ++i)
        relCaps[i] = droneList[i].relative_capability_score / totalCap;

    double cumTargetArea = 0.0; // Prefix target in original ROI area
    double prevCut = 0.0;

    for (int i = 0; i < droneList.size(); ++i)
    {
        const bool isLast = (i == droneList.size() - 1);

        if (isLast)
        {
            // Last drone: intersection of [prevCut, mar.width] with original roi
            QPolygonF finalRect = makeRectPoly(mar, prevCut, mar.width);
            QPolygonF lastSlice = finalRect.intersected(roi);
            if (lastSlice.size() >= 3)
                result.append({lastSlice, QString::number(droneList[i].id)});
            else
                result.append({QPolygonF(), QString::number(droneList[i].id)});
            break;
        }

        // Target for this slice: relCaps[i] * totalArea (in original ROI space)
        double targetSliceArea = relCaps[i] * totalArea;
        cumTargetArea += targetSliceArea;

        // Binary search for cut position where prefix_area ≈ cumTargetArea
        double low = prevCut, high = mar.width;
        QPolygonF bestSlice;
        double bestCut = prevCut;
        double bestPrefixArea = 0.0;

        for (int iter = 0; iter < 40; ++iter)
        {
            double mid = 0.5 * (low + high);

            // Prefix rect: [0, mid] along mar.ux
            QPolygonF prefixRect = makeRectPoly(mar, 0.0, mid);
            QPolygonF prefixIntersect = prefixRect.intersected(roi);
            double prefixArea = calculatePolygonArea(prefixIntersect);

            // Slice area for this drone: prefix[mid] - prefix[prevCut]
            double sliceArea = prefixArea - bestPrefixArea; // Incremental

            if (std::fabs(sliceArea - targetSliceArea) < std::fabs(calculatePolygonArea(bestSlice) - targetSliceArea) &&
                prefixIntersect.size() >= 3)
            {
                // Slice is [prevCut, mid]
                QPolygonF sliceRect = makeRectPoly(mar, prevCut, mid);
                bestSlice = sliceRect.intersected(roi);
                bestCut = mid;
                bestPrefixArea = prefixArea;
            }

            if (prefixArea < cumTargetArea)
                low = mid;
            else
                high = mid;
        }

        // Fallback if slice is invalid: proportional cut
        if (bestSlice.size() < 3)
        {
            double proportional = prevCut + relCaps[i] * (mar.width - prevCut);
            QPolygonF fallbackRect = makeRectPoly(mar, prevCut, proportional);
            bestSlice = fallbackRect.intersected(roi);
            bestCut = proportional;
        }

        // Assign (guaranteed valid or empty)
        result.append({bestSlice, QString::number(droneList[i].id)});

        // Advance cut (no subtraction needed!)
        prevCut = bestCut;
    }

    return result;
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