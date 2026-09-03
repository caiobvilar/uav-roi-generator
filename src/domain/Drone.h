#ifndef DOMAIN_DRONE_H
#define DOMAIN_DRONE_H

#include <QList>
#include <QString>
#include <cstdint>

// All units follow SI: lengths in meters, time in seconds, velocity in m/s
struct Drone
{
    uint32_t id;  // Unique drone identifier
    QString name; // Human-readable name

    // Battery
    double battery_capacity = 0.0;         // [mAh] Total battery capacity
    double battery_current_capacity = 0.0; // [%] Current charge percentage (0-100)

    // Velocity limits
    double max_horizontal_velocity = 0.0; // [m/s] Maximum horizontal flight speed
    double max_vertical_velocity = 0.0;   // [m/s] Maximum vertical flight speed

    // Camera sensor parameters (from JSON, in SI units)
    double camera_focal_length = 0.0;  // [m] Lens focal length
    double camera_array_width = 0.0;   // [m] Sensor physical width
    double camera_array_height = 0.0;  // [m] Sensor physical height
    double camera_image_width = 0.0;   // [pixels] Image width in pixels
    double camera_image_height = 0.0;  // [pixels] Image height in pixels
    double camera_shutter_speed = 0.0; // [s] Shutter speed (exposure time)

    // Computed parameters (all in SI units)
    double max_forward_velocity = 0.0;      // [m/s] Max velocity for image overlap
    double max_x_footprint = 0.0;           // [m] Ground footprint width (L_x)
    double max_y_footprint = 0.0;           // [m] Ground footprint height (L_y)
    double ideal_flight_altitude = 0.0;     // [m] Optimal altitude for desired GSD
    double relative_capability_score = 0.0; // [m²/s] Relative coverage capability
};

namespace domain {

void calcFlightAltitude(QList<Drone>& droneList);
void calcDroneCameraFootprint(QList<Drone>& droneList);
void calcMaximumForwardVelocity(QList<Drone>& droneList);
void calcDroneRelativeCapability(QList<Drone>& droneList);
QList<Drone> parseDrones(const QString& jsonFile);

} // namespace domain

#endif // DOMAIN_DRONE_H
