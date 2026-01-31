#ifndef UTILS_H_
#define UTILS_H_


#include <QPointF>
#include <QPolygonF>
#include <QRectF>
#include <QSize>
#include <QString>

#define DESIRED_GSD 2.0     // This asumes a desired ground sample distance of 2.0cm/pixel for precision agriculture
#define FORWARD_OVERLAP 0.8 // This assumes precision agriculture
#define SIDE_OVERLAP 0.75   // This also assumes precision agriculture

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

// Heuristic: WGS84 (projected meters) coordinates are typically large (e.g., > 10,000)
inline bool
isPolygonWGS84(const QPolygonF& poly)
{
    if (poly.isEmpty())
        return false;
    int countWGS84 = 0;
    for (const QPointF& pt : poly)
    {
        if (std::abs(pt.x()) > 10000 && std::abs(pt.y()) > 10000)
            countWGS84++;
    }
    return countWGS84 > poly.size() / 2;
}

// Checks if most points are within the given widget's pixel bounds
inline bool
isPolygonPixel(const QPolygonF& poly, int width, int height)
{
    if (poly.isEmpty())
        return false;
    int countPixel = 0;
    for (const QPointF& pt : poly)
    {
        if (pt.x() >= 0 && pt.x() < width && pt.y() >= 0 && pt.y() < height)
            countPixel++;
    }
    return countPixel > poly.size() / 2;
}

#endif // UTILS_H