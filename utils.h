#ifndef UTILS_H_
#define UTILS_H_


#include <QDebug>
#include <QPointF>
#include <QPolygonF>
#include <QRectF>
#include <QSize>
#include <QString>
#include <cmath>

// === SI UNITS USED THROUGHOUT ===
// All lengths in METERS, time in SECONDS, velocity in M/S
#define DESIRED_GSD 0.02    // Ground sample distance: 0.02 m/pixel (2 cm/pixel) for precision agriculture
#define FORWARD_OVERLAP 0.8 // Forward overlap ratio (20%) - spacing = footprint * 0.8
#define SIDE_OVERLAP 0.75   // Side overlap ratio (25%) - spacing = footprint * 0.75

// === WIDGET DIMENSIONS ===
#define WIDGET_MIN_HEIGHT 650 // Minimum widget height in pixels
#define WIDGET_MIN_WIDTH 1000 // Minimum widget width in pixels

// === FONT SIZES ===
#define FONT_SIZE_SMALL 12 // Small font point size for labels
#define FONT_SIZE_LARGE 14 // Large font point size for titles

// === PEN WIDTHS ===
#define PEN_WIDTH_DEFAULT 2.0 // Default pen width for lines
#define PEN_WIDTH_MEDIUM 3    // Medium pen width for outlines
#define PEN_WIDTH_THICK 4     // Thick pen width for emphasis

// === ALPHA/TRANSPARENCY VALUES ===
#define ALPHA_SUBROI_FILL 50 // Alpha for sub-ROI fill (50/255 ≈ 20% opacity)
#define ALPHA_ROI_OUTLINE 80 // Alpha for ROI outline brush
#define ALPHA_LABEL_BG 180   // Alpha for label background

// === ZOOM PARAMETERS ===
#define ZOOM_STEP 1.001 // Zoom step multiplier for fine control
#define ZOOM_MIN 0.1    // Minimum zoom factor
#define ZOOM_MAX 20.0   // Maximum zoom factor

// === LABEL RENDERING ===
#define LABEL_ROTATION_DEG -30.0 // Label rotation in degrees
#define LABEL_MARGIN_SMALL 5.0   // Small margin for label positioning
#define LABEL_MARGIN_LARGE 10    // Large margin for label positioning

// === WAYPOINT DOT RENDERING ===
#define DOT_RADIUS_MIN 0.5      // Minimum dot radius in pixels
#define DOT_RADIUS_MAX 2.0      // Maximum dot radius in pixels
#define DOT_RADIUS_SCALE 0.0005 // Scale factor for dot radius (0.05% of image)

// === EPSILON VALUES FOR FLOATING-POINT COMPARISONS ===
#define EPSILON_SMALL 1e-8 // Small epsilon for comparisons
#define EPSILON_TINY 1e-12 // Tiny epsilon for determinant checks

// === COLOR CONSTANTS ===
#define HUE_FULL_CIRCLE 360 // Full hue circle in degrees
#define HUE_OPPOSITE 180    // Opposite hue offset
#define HUE_SHIFT_AMOUNT 60 // Amount to shift hue to avoid background

// === VALIDATION THRESHOLDS ===
#define MAR_SIZE_MAX 100000 // Maximum MAR dimension for validation

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

// === DRONE STRUCTURE ===
// All units follow SI: lengths in meters, time in seconds, velocity in m/s
struct drone
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

// Check if a RotatedRect is in WGS84/projected coordinates (large values typical of UTM)
inline bool
isRotatedRectWGS84(const RotatedRect& rect)
{
    return std::abs(rect.origin.x()) > 10000 && std::abs(rect.origin.y()) > 10000;
}

// Check if a value is in meters (typical for geo calculations: 0.1m to 10km range)
// Returns false if value looks like centimeters (> 1000 for typical drone footprints)
inline bool
isValueInMeters(double value, const QString& context = "")
{
    // Typical drone footprints: 10-500 meters
    // If value > 1000, it's likely in centimeters (10m = 1000cm)
    if (value > 1000.0)
    {
        qWarning() << "Unit mismatch suspected:" << context << "value=" << value
                   << "looks like centimeters, expected meters";
        return false;
    }
    // If value < 0.1, it's likely in kilometers or invalid
    if (value < 0.1 && value > 0)
    {
        qWarning() << "Unit mismatch suspected:" << context << "value=" << value << "looks too small for meters";
        return false;
    }
    return value > 0;
}

// Validate that polygon and RotatedRect are in the same coordinate system
inline bool
validateCoordinateSystemMatch(const QPolygonF& poly, const RotatedRect& rect, const QString& context = "")
{
    bool polyIsWGS84 = isPolygonWGS84(poly);
    bool rectIsWGS84 = isRotatedRectWGS84(rect);

    if (polyIsWGS84 != rectIsWGS84)
    {
        qCritical() << "COORDINATE SYSTEM MISMATCH in" << context << ":"
                    << "Polygon is" << (polyIsWGS84 ? "WGS84" : "Pixel") << "but RotatedRect is"
                    << (rectIsWGS84 ? "WGS84" : "Pixel");
        return false;
    }
    return true;
}

// Validate footprint values are in expected range for meters
inline bool
validateFootprintMeters(double footprintX, double footprintY, const QString& context = "")
{
    bool xValid = isValueInMeters(footprintX, context + " footprintX");
    bool yValid = isValueInMeters(footprintY, context + " footprintY");

    if (!xValid || !yValid)
    {
        qCritical() << "UNIT VALIDATION FAILED in" << context << ":"
                    << "footprintX=" << footprintX << "footprintY=" << footprintY
                    << "Expected values in METERS (typical range: 10-500m)";
        return false;
    }
    return true;
}

#endif // UTILS_H