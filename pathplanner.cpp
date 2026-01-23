#include "pathplanner.h"
#include "ROIArea.h"
#include <QList>
#include <QPolygonF>
#include <QVector>
#include <algorithm>

#define DESIRED_GSD 2.0     // This asumes a desired ground sample distance of 2.0cm/pixel for precision agriculture
#define FORWARD_OVERLAP 0.8 // This assumes precision agriculture
#define SIDE_OVERLAP 0.75   // This also assumes precision agriculture

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

QList<QPolygonF>
PathPlanner::decomposedROI(QPolygonF& roi, QList<drone>& droneList, const RotatedRect& mar)
{
    QList<QPolygonF> result;
    if (roi.size() < 3 || droneList.isEmpty() || mar.width <= 0.0 || mar.height <= 0.0)
        return result;

    // 1. Normalize relative capabilities
    double totalCap = 0.0;
    for (const drone& d : droneList)
        totalCap += d.relative_capability_score;
    if (totalCap <= 0.0)
        return result;

    QVector<double> caps;
    for (const drone& d : droneList)
        caps.append(d.relative_capability_score / totalCap);

    // 2. Partition MAR along its long side (width)
    double start = 0.0;
    for (int i = 0; i < caps.size(); ++i)
    {
        double end = start + caps[i] * mar.width;
        QPolygonF rectPoly = makeRectPoly(mar, start, end);

        // 3. Clip rectangle to ROI (intersection)
        QPolygonF clipped = rectPoly.intersected(roi);
        if (clipped.size() >= 3)
            result.append(clipped);

        start = end;
    }
    return result;
}
