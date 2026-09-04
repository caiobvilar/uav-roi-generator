#ifndef GDALHANDLER_H
#define GDALHANDLER_H
#include <QDebug>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QList>
#include <QPointF>
#include <QRectF>
#include <QString>
#include <QtMath>
#include <cpl_conv.h> // CPLMalloc
#include <gdal_priv.h>

#define GEO_TRANSFORM_SIZE 6

class GDALHandler
{
  public:
    GDALHandler();
    ~GDALHandler();
    GDALHandler(const GDALHandler&) = delete;
    GDALHandler& operator=(const GDALHandler&) = delete;

    bool
    openSrcRaster(const QString& fileName);
    void
    closeRaster();
    QPointF
    pixelToGeo(const QPointF& pixel) const;
    QPolygonF
    polygonToGeo(const QPolygonF& poly) const;

    const GDALDataset*
    getDataset() const
    {
        return srcDataset;
    }

    QString
    getDataSetCRS(const GDALDataset* dataSet) const;

    QString
    getDataSetCRSInfo() const
    {
        return dataSetCRSInfo;
    }

    QImage
    toQImage() const;
    QPointF
    geoToPixel(const QPointF& geo) const;
    QPolygonF
    geoPolygonToPixels(const QList<QPointF>& geoPts) const;
    QPolygonF
    geoPolygonToPixels(const QPolygonF& geoPoly) const;

  private:
    GDALDataset* srcDataset = nullptr;
    double geoTransform[GEO_TRANSFORM_SIZE];
    double invGeoTransform[GEO_TRANSFORM_SIZE];
    bool hasInvGeoTransform = false;
    QString dataSetCRSInfo = QString("NaN");
};

#endif // GDALHANDLER_H
