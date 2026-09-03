#include "domain/GeoValidation.h"

#include <QDebug>
#include <QPointF>
#include <cmath>

namespace domain {

// Heuristic: WGS84 (projected meters) coordinates are typically large (e.g., > 10,000)
bool
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
bool
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
bool
isRotatedRectWGS84(const RotatedRect& rect)
{
    return std::abs(rect.origin.x()) > 10000 && std::abs(rect.origin.y()) > 10000;
}

// Check if a value is in meters (typical for geo calculations: 0.1m to 10km range)
// Returns false if value looks like centimeters (> 1000 for typical drone footprints)
bool
isValueInMeters(double value, const QString& context)
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
bool
validateCoordinateSystemMatch(const QPolygonF& poly, const RotatedRect& rect, const QString& context)
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
bool
validateFootprintMeters(double footprintX, double footprintY, const QString& context)
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

} // namespace domain
