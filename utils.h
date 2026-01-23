#ifndef UTILS_H_
#define UTILS_H_


#include <QPointF>
#include <QRectF>
#include <QString>

struct RotatedRect
{
    QRectF rect;
    qreal angle;
    QPointF origin; // corner at (minX, minY) in image coords
    QPointF ux;     // unit vector along width (x‑axis of rect)
    QPointF uy;     // unit vector along height (y‑axis of rect)
    qreal width;
    qreal height;
    // orientation of the box in radians
};

struct drone
{
    uint32_t id;
    QString name;
    double battery_capacity = 0.0;
    double battery_current_capacity = 0.0;
    double max_horizontal_velocity = 0.0;
    double max_vertical_velocity = 0.0;
    double camera_focal_length = 0.0;
    double camera_array_width = 0.0;
    double camera_array_height = 0.0;
    double camera_image_width = 0.0;
    double camera_image_height = 0.0;
    double camera_shutter_speed = 0.0;
    double max_forward_velocity = 0.0;
    double max_x_footprint = 0.0;
    double max_y_footprint = 0.0;
    double ideal_flight_altitude = 0.0;
    double relative_capability_score = 0.0;
};
#endif // UTILS_H