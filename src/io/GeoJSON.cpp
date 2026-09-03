#include "io/GeoJSON.h"

#include <gdal_priv.h>
#include <ogr_spatialref.h>
#include <ogrsf_frmts.h>

#include <QDebug>
#include <QJsonArray>
#include <QJsonObject>

namespace geo
{

QList<QPointF>
importPolygon(const QString& path)
{
    QList<QPointF> pts;

    GDALDataset* poDS =
        static_cast<GDALDataset*>(GDALOpenEx(path.toUtf8().constData(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr));
    if (!poDS)
        return pts;

    OGRLayer* layer = poDS->GetLayer(0);
    OGRFeature* feat = layer->GetNextFeature();
    if (!feat)
    {
        GDALClose(poDS);
        return pts;
    }

    OGRGeometry* geom = feat->GetGeometryRef();
    if (!geom || wkbFlatten(geom->getGeometryType()) != wkbPolygon)
    {
        OGRFeature::DestroyFeature(feat);
        GDALClose(poDS);
        return pts;
    }

    auto* poly = geom->toPolygon();
    OGRLinearRing* ring = poly->getExteriorRing();
    int n = ring->getNumPoints();
    pts.reserve(n);
    for (int i = 0; i < n; ++i)
    {
        double lon = ring->getX(i);
        double lat = ring->getY(i);
        pts.emplace_back(lon, lat);
    }

    OGRFeature::DestroyFeature(feat);
    GDALClose(poDS);
    return pts;
}

QJsonDocument
reprojectToWgs84(const QJsonDocument& srcDoc, const QString& srcWkt)
{
    if (srcDoc.isNull() || !srcDoc.isObject())
        return QJsonDocument();

    QJsonObject fc = srcDoc.object();
    if (fc.value("type").toString() != QStringLiteral("FeatureCollection"))
        return QJsonDocument();

    QJsonArray features = fc.value("features").toArray();
    if (features.isEmpty() || !features.at(0).isObject())
        return QJsonDocument();

    QJsonObject feature = features.at(0).toObject();
    QJsonObject geom = feature.value("geometry").toObject();
    if (geom.value("type").toString() != QStringLiteral("Polygon"))
        return QJsonDocument();

    QJsonArray coords = geom.value("coordinates").toArray();
    if (coords.isEmpty() || !coords.at(0).isArray())
        return QJsonDocument();

    QJsonArray ring = coords.at(0).toArray(); // first linear ring

    // --- Detect source CRS ---
    OGRSpatialReference srcSRS, dstSRS;

    // Try to get CRS from the provided WKT (from the dataset)
    if (!srcWkt.isEmpty())
    {
        if (srcSRS.importFromWkt(srcWkt.toUtf8().constData()) != OGRERR_NONE)
        {
            qWarning() << "Failed to import CRS from dataset, trying properties...";
            srcSRS.Clear();
        }
    }

    // If no dataset CRS, try to get from GeoJSON properties
    if (srcSRS.IsEmpty())
    {
        QJsonObject props = feature.value("properties").toObject();

        // Try EPSG code from properties
        if (props.contains("crs_code"))
        {
            int epsgCode = props.value("crs_code").toInt();
            if (epsgCode > 0)
            {
                if (srcSRS.importFromEPSG(epsgCode) != OGRERR_NONE)
                {
                    qWarning() << "Failed to import EPSG:" << epsgCode;
                    srcSRS.Clear();
                }
            }
        }

        // Try CRS from top-level "crs" member (older GeoJSON spec)
        if (srcSRS.IsEmpty() && fc.contains("crs"))
        {
            QJsonObject crsObj = fc.value("crs").toObject();
            QJsonObject crsProps = crsObj.value("properties").toObject();
            QString crsName = crsProps.value("name").toString();

            if (!crsName.isEmpty())
            {
                if (srcSRS.SetFromUserInput(crsName.toUtf8().constData()) != OGRERR_NONE)
                {
                    qWarning() << "Failed to parse CRS:" << crsName;
                    srcSRS.Clear();
                }
            }
        }
    }

    // Default to SIRGAS 2000 / UTM zone 24S (EPSG:31984) if still no CRS found
    if (srcSRS.IsEmpty())
    {
        qWarning() << "No CRS found in GeoJSON or dataset, defaulting to EPSG:31984";
        if (srcSRS.importFromEPSG(31984) != OGRERR_NONE)
        {
            qWarning() << "Failed to set default EPSG:31984";
            return QJsonDocument();
        }
    }

    // Set destination to WGS84
    if (dstSRS.importFromEPSG(4326) != OGRERR_NONE)
    {
        qWarning() << "Failed to import EPSG:4326 (WGS84)";
        return QJsonDocument();
    }

    // Check if source is already WGS84
    if (srcSRS.IsSame(&dstSRS))
    {
        qDebug() << "Source CRS is already WGS84, no reprojection needed";
        return srcDoc; // Return as-is
    }

    OGRCoordinateTransformation* ct = OGRCreateCoordinateTransformation(&srcSRS, &dstSRS);
    if (!ct)
    {
        qWarning() << "Failed to create coordinate transformation";
        return QJsonDocument();
    }

    QJsonArray outRing;
    for (int i = 0; i < ring.size(); ++i)
    {
        QJsonArray c = ring.at(i).toArray();
        if (c.size() < 2)
        {
            outRing.append(c);
            continue;
        }

        double x = c.at(0).toDouble(); // easting (m) or lon
        double y = c.at(1).toDouble(); // northing (m) or lat
        double z = 0.0;

        if (!ct->Transform(1, &x, &y, &z))
        {
            qWarning() << "Transform failed for point" << i;
            outRing.append(c); // fallback: keep original
            continue;
        }

        // GDAL Transform with EPSG:4326 may return (lat, lon) in traditional axis order
        // GeoJSON requires [lon, lat], so I swap them here
        QJsonArray outC;
        outC.append(y); // lon (was in y after transform)
        outC.append(x); // lat (was in x after transform)
        outRing.append(outC);
    }
    OCTDestroyCoordinateTransformation(ct);

    QJsonArray outCoords;
    outCoords.append(outRing);
    geom["coordinates"] = outCoords;
    feature["geometry"] = geom;

    // update properties CRS info
    QJsonObject props = feature.value("properties").toObject();
    props["crs_authority"] = QStringLiteral("EPSG");
    props["crs_code"] = 4326;
    props["crs_name"] = QStringLiteral("WGS 84");
    feature["properties"] = props;

    features[0] = feature;
    fc["features"] = features;

    return QJsonDocument(fc);
}

} // namespace geo
