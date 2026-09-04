#ifndef GEO_JSON_H
#define GEO_JSON_H

#include <QJsonDocument>
#include <QList>
#include <QPointF>
#include <QString>

namespace geo
{

QList<QPointF>
importPolygon(const QString& path);

QJsonDocument
reprojectToWgs84(const QJsonDocument& srcDoc, const QString& srcWkt);

} // namespace geo

#endif // GEO_JSON_H
