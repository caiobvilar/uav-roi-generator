#include "io/GDALHandler.h"

#include <cstring>
#include <vector>

#include <ogr_spatialref.h>

GDALHandler::GDALHandler()
{
    GDALAllRegister();
}

bool
GDALHandler::openSrcRaster(const QString& fileName)
{
    closeRaster(); // in case something was open

    srcDataset = static_cast<GDALDataset*>(GDALOpen(fileName.toStdString().c_str(), GA_ReadOnly));
    if (!srcDataset)
        return false;
    if (srcDataset->GetGeoTransform(geoTransform) == CE_None)
    {
        hasInvGeoTransform = GDALInvGeoTransform(geoTransform, invGeoTransform);
    }
    else
    {
        qWarning() << "No georeferencing data from this dataset/image.";
    }
    this->dataSetCRSInfo = getDataSetCRS(srcDataset);
    return true;
}

GDALHandler::~GDALHandler()
{
    closeRaster();
}

void
GDALHandler::closeRaster()
{
    if (srcDataset)
    {
        GDALClose(srcDataset);
        srcDataset = nullptr;
    }
    hasInvGeoTransform = false;
}

QString
GDALHandler::getDataSetCRS(const GDALDataset* dataSet) const
{
    QString retCRS = QString("NaN");
    if (!dataSet)
    {
        return retCRS = "Error opening GDALDataset";
    }

    // For most raster datasets (including GeoTIFF)
    const char* wkt = dataSet->GetProjectionRef(); // or GetProjection()
    if (!wkt || std::strlen(wkt) == 0)
    {
        // Some formats store only a GeoTransform or nothing
        qWarning() << "Dataset has no projection.";
        return retCRS = "Dataset has no projection.";
    }

    OGRSpatialReference srs;
    if (srs.importFromWkt(wkt) != OGRERR_NONE)
    {
        qWarning() << "Failed to parse WKT projection.";
        return retCRS = "Failed to parse WKT projection.";
    }

    // Try to get an EPSG code if available
    const char* authName = srs.GetAuthorityName(nullptr);
    const char* authCode = srs.GetAuthorityCode(nullptr);

    if (authName && authCode)
    {
        retCRS = "CRS authority:" + QString(authName) + "code:" + QString(authCode);
    }

    char* prettyWkt = nullptr;
    srs.exportToPrettyWkt(&prettyWkt, FALSE);
    retCRS += "CRS WKT:\n" + QString(prettyWkt);
    CPLFree(prettyWkt);
    return retCRS;
}

QImage
GDALHandler::toQImage() const
{
    if (!srcDataset)
        return QImage();

    const int width = srcDataset->GetRasterXSize();
    const int height = srcDataset->GetRasterYSize();
    const int bands = srcDataset->GetRasterCount();
    if (bands < 3)
        return QImage();

    GDALRasterBand* rb = srcDataset->GetRasterBand(1);
    GDALRasterBand* gb = srcDataset->GetRasterBand(2);
    GDALRasterBand* bb = srcDataset->GetRasterBand(3);
    if (!rb || !gb || !bb)
        return QImage();

    GDALDataType dtR = rb->GetRasterDataType();
    GDALDataType dtG = gb->GetRasterDataType();
    GDALDataType dtB = bb->GetRasterDataType();

    qDebug() << "Raster types:" << dtR << dtG << dtB;

    // Accept only 8‑bit here
    if (!(dtR == GDT_Byte && dtG == GDT_Byte && dtB == GDT_Byte))
    {
        qWarning("GDALHandler::toQImage: unsupported band type");
        return QImage();
    }

    const int64_t pixelCount = static_cast<int64_t>(width) * static_cast<int64_t>(height);
    if (pixelCount <= 0)
        return QImage();
    std::vector<uint8_t> rBuf(static_cast<size_t>(pixelCount));
    std::vector<uint8_t> gBuf(static_cast<size_t>(pixelCount));
    std::vector<uint8_t> bBuf(static_cast<size_t>(pixelCount));

    if (rb->RasterIO(GF_Read, 0, 0, width, height, rBuf.data(), width, height, GDT_Byte, 0, 0) != CE_None)
        return QImage();
    if (gb->RasterIO(GF_Read, 0, 0, width, height, gBuf.data(), width, height, GDT_Byte, 0, 0) != CE_None)
        return QImage();
    if (bb->RasterIO(GF_Read, 0, 0, width, height, bBuf.data(), width, height, GDT_Byte, 0, 0) != CE_None)
        return QImage();

    QImage img(width, height, QImage::Format_RGB888);
    if (img.isNull())
        return QImage();

    for (int y = 0; y < height; ++y)
    {
        uchar* dst = img.scanLine(y);
        const int64_t rowOff = static_cast<int64_t>(y) * width;
        for (int x = 0; x < width; ++x)
        {
            const int64_t i = rowOff + x;
            dst[3 * x + 0] = rBuf[static_cast<size_t>(i)];
            dst[3 * x + 1] = gBuf[static_cast<size_t>(i)];
            dst[3 * x + 2] = bBuf[static_cast<size_t>(i)];
        }
    }

    return img;
}

QPointF
GDALHandler::pixelToGeo(const QPointF& pixel) const
{
    const double* GT = geoTransform; // get pointer or array

    double x = pixel.x();
    double y = pixel.y();
    // To understand the "+ 0.5" refer to
    // https://engineering.purdue.edu/RVL/blog/doku.php?id=blog%3A2018%3A1003_pixel_to_geodesic_coordinate_transformations_using_geotiffs
    double Xgeo = GT[0] + (x + 0.5) * GT[1] + (y + 0.5) * GT[2];
    double Ygeo = GT[3] + (x + 0.5) * GT[4] + (y + 0.5) * GT[5];

    return QPointF(Xgeo, Ygeo);
}

QPolygonF
GDALHandler::polygonToGeo(const QPolygonF& poly) const
{
    QPolygonF geoPoly;
    geoPoly.reserve(poly.size());
    for (const QPointF& p : poly)
        geoPoly << pixelToGeo(p);
    return geoPoly;
}

QPointF
GDALHandler::geoToPixel(const QPointF& geo) const
{
    // geo = (lon, lat) for EPSG:4326
    if (!hasInvGeoTransform)
        return QPointF();

    double Xgeo = geo.x();
    double Ygeo = geo.y();
    double px = invGeoTransform[0] + Xgeo * invGeoTransform[1] + Ygeo * invGeoTransform[2];
    double py = invGeoTransform[3] + Xgeo * invGeoTransform[4] + Ygeo * invGeoTransform[5];

    // shift to pixel centers to draw at centers
    px -= 0.5;
    py -= 0.5;

    return QPointF(px, py);
}

QPolygonF
GDALHandler::geoPolygonToPixels(const QList<QPointF>& geoPts) const
{
    QPolygonF pixPoly;
    if (!hasInvGeoTransform)
        return pixPoly;

    pixPoly.reserve(geoPts.size());

    for (int i = 0; i < geoPts.size(); ++i)
    {
        const QPointF& g = geoPts.at(i);
        double Xgeo = g.x();
        double Ygeo = g.y();

        double px = invGeoTransform[0] + Xgeo * invGeoTransform[1] + Ygeo * invGeoTransform[2];
        double py = invGeoTransform[3] + Xgeo * invGeoTransform[4] + Ygeo * invGeoTransform[5];

        pixPoly << QPointF(px, py);
    }
    return pixPoly;
}

QPolygonF
GDALHandler::geoPolygonToPixels(const QPolygonF& geoPoly) const
{
    QPolygonF pixPoly;
    if (!hasInvGeoTransform)
        return pixPoly;

    pixPoly.reserve(geoPoly.size());

    for (int i = 0; i < geoPoly.size(); ++i)
    {
        const QPointF& g = geoPoly.at(i);
        double Xgeo = g.x();
        double Ygeo = g.y();

        double px = invGeoTransform[0] + Xgeo * invGeoTransform[1] + Ygeo * invGeoTransform[2];
        double py = invGeoTransform[3] + Xgeo * invGeoTransform[4] + Ygeo * invGeoTransform[5];

        pixPoly << QPointF(px, py);
    }
    return pixPoly;
}
