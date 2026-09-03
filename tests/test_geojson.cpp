#include <QtTest>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPair>
#include <QString>
#include <QVector>

#include <cmath>

#include <cpl_conv.h>
#include <gdal_priv.h>
#include <ogr_spatialref.h>

#include "io/GeoJSON.h"

static QString
wktFromEpsg(int epsg)
{
    OGRSpatialReference srs;
    if (srs.importFromEPSG(epsg) != OGRERR_NONE)
        return QString();
    char* wkt = nullptr;
    srs.exportToWkt(&wkt);
    QString out = QString::fromUtf8(wkt);
    CPLFree(wkt);
    return out;
}

static QJsonDocument
makePolygonDoc(const QVector<QPair<double, double>>& ring)
{
    QJsonArray coords;
    for (const auto& p : ring)
        coords.append(QJsonArray{p.first, p.second});
    QJsonArray ringArr;
    ringArr.append(coords);
    QJsonArray coordinates;
    coordinates.append(ringArr);
    QJsonObject geom;
    geom["type"] = "Polygon";
    geom["coordinates"] = coordinates;
    QJsonObject feature;
    feature["type"] = "Feature";
    feature["geometry"] = geom;
    feature["properties"] = QJsonObject{};
    QJsonArray features;
    features.append(feature);
    QJsonObject fc;
    fc["type"] = "FeatureCollection";
    fc["features"] = features;
    return QJsonDocument(fc);
}

class TestGeojson : public QObject
{
    Q_OBJECT
  private slots:
    void initTestCase();
    void null_doc_returns_empty();
    void already_wgs84_passthrough();
    void utm_to_wgs84_reprojects();
};

void
TestGeojson::initTestCase()
{
    GDALAllRegister();
}

void
TestGeojson::null_doc_returns_empty()
{
    QVERIFY(geo::reprojectToWgs84(QJsonDocument(), QString()).isNull());

    QJsonDocument nonFc(QJsonObject{{"type", "NotAFeatureCollection"}});
    QVERIFY(geo::reprojectToWgs84(nonFc, wktFromEpsg(31984)).isNull());
}

void
TestGeojson::already_wgs84_passthrough()
{
    QVector<QPair<double, double>> ring{{-40.0, -20.0}, {-39.9, -20.0}, {-39.9, -19.9}, {-40.0, -19.9}, {-40.0, -20.0}};
    QJsonDocument in = makePolygonDoc(ring);
    QJsonDocument out = geo::reprojectToWgs84(in, wktFromEpsg(4326));

    QVERIFY(!out.isNull());
    QCOMPARE(out.toJson(QJsonDocument::Compact), in.toJson(QJsonDocument::Compact));
}

void
TestGeojson::utm_to_wgs84_reprojects()
{
    QVector<QPair<double, double>> ring{{500000, 8000000},
                                        {500200, 8000000},
                                        {500200, 8000200},
                                        {500000, 8000200},
                                        {500000, 8000000}};
    QJsonDocument in = makePolygonDoc(ring);
    QJsonDocument out = geo::reprojectToWgs84(in, wktFromEpsg(31984));

    QVERIFY(!out.isNull());

    QJsonObject fc = out.object();
    QCOMPARE(fc.value("type").toString(), QString("FeatureCollection"));

    QJsonArray features = fc.value("features").toArray();
    QVERIFY(!features.isEmpty());

    QJsonObject feature = features.at(0).toObject();
    QJsonObject props = feature.value("properties").toObject();
    QCOMPARE(props.value("crs_code").toInt(), 4326);

    QJsonArray coords = feature.value("geometry").toObject().value("coordinates").toArray();
    QVERIFY(!coords.isEmpty());
    QJsonArray outRing = coords.at(0).toArray();

    for (const auto& cVal : outRing)
    {
        QJsonArray c = cVal.toArray();
        QVERIFY(c.size() >= 2);
        double lon = c.at(0).toDouble();
        double lat = c.at(1).toDouble();
        QVERIFY(std::isfinite(lon));
        QVERIFY(std::isfinite(lat));
        QVERIFY(lon >= -180.0 && lon <= 180.0);
        QVERIFY(lat >= -90.0 && lat <= 90.0);
    }
}

QTEST_GUILESS_MAIN(TestGeojson)
#include "test_geojson.moc"
