#include "domain/Drone.h"
#include "constants.h"

#include <QDebug>
#include <QFile>
#include <QIODevice>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QtGlobal>
#include <algorithm>

namespace domain {

QList<Drone>
parseDrones(const QString& filename)
{
    QList<Drone> results;

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

        Drone d;
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
calcFlightAltitude(QList<Drone>& droneList)
{
    for (auto& d : droneList)
    {
        // GSD_x * f_l * i_x / l_x
        double h_x =
            (constants::kDesiredGsd * d.camera_focal_length * d.camera_image_width) / d.camera_array_width;

        // GSD_y * f_l * i_y / l_y
        double h_y =
            (constants::kDesiredGsd * d.camera_focal_length * d.camera_image_height) / d.camera_array_height;

        // h = min{h_x, h_y}
        d.ideal_flight_altitude = std::min(h_x, h_y);

        qDebug() << "Drone" << d.name << "- Ideal flight altitude:" << d.ideal_flight_altitude << "m";
    }
}

void
calcDroneCameraFootprint(QList<Drone>& droneList)
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
calcMaximumForwardVelocity(QList<Drone>& droneList)
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
calcDroneRelativeCapability(QList<Drone>& droneList)
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

} // namespace domain
