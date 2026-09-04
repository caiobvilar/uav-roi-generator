#ifndef DOMAIN_GEOVALIDATION_H
#define DOMAIN_GEOVALIDATION_H

#include "domain/RotatedRect.h"

#include <QPolygonF>
#include <QString>

namespace domain {

bool isPolygonWGS84(const QPolygonF& poly);
bool isRotatedRectWGS84(const RotatedRect& rect);
bool isValueInMeters(double value, const QString& context = "");
bool validateCoordinateSystemMatch(const QPolygonF& poly, const RotatedRect& rect, const QString& context = "");
bool validateFootprintMeters(double footprintX, double footprintY, const QString& context = "");

} // namespace domain

#endif // DOMAIN_GEOVALIDATION_H
