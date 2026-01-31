#include "GDALHandler.h"

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
    if (srcDataset->GetGeoTransform(geoTransform) != CE_None)
    {
        qWarning() << "No georeferencing data from this dataset/image.";
    }
    this->dataSetCRSInfo = getDataSetCRS(srcDataset);
    return true;
}

void
GDALHandler::closeRaster()
{
    if (srcDataset)
    {
        GDALClose(srcDataset);
        srcDataset = nullptr;
    }
    if (destDataset)
    {
        GDALClose(destDataset);
        destDataset = nullptr;
    }
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

    const int pixelCount = width * height;
    std::vector<uint8_t> rBuf(pixelCount);
    std::vector<uint8_t> gBuf(pixelCount);
    std::vector<uint8_t> bBuf(pixelCount);

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
        int rowOff = y * width;
        for (int x = 0; x < width; ++x)
        {
            int i = rowOff + x;
            dst[3 * x + 0] = rBuf[i];
            dst[3 * x + 1] = gBuf[i];
            dst[3 * x + 2] = bBuf[i];
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

QJsonDocument
GDALHandler::reprojectGeoJSONPolygon(const QJsonDocument& srcDoc) const
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

    // Try to get CRS from the dataset (if raster is loaded)
    if (srcDataset)
    {
        const char* wkt = srcDataset->GetProjectionRef();
        if (wkt && std::strlen(wkt) > 0)
        {
            if (srcSRS.importFromWkt(wkt) != OGRERR_NONE)
            {
                qWarning() << "Failed to import CRS from dataset, trying properties...";
                srcSRS.Clear();
            }
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

QList<QPointF>
GDALHandler::loadPolygonFromGeoJSON(const QString& path)
{
    QList<QPointF> pts;

    GDALAllRegister();
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

QPointF
GDALHandler::geoToPixel(const QPointF& geo) const
{
    // geo = (lon, lat) for EPSG:4326
    double invGT[6];
    double gt[6];
    std::memcpy(gt, geoTransform, sizeof(gt));

    if (!GDALInvGeoTransform(gt, invGT))
    {
        return QPointF();
    }

    double Xgeo = geo.x();
    double Ygeo = geo.y();
    double px = invGT[0] + Xgeo * invGT[1] + Ygeo * invGT[2];
    double py = invGT[3] + Xgeo * invGT[4] + Ygeo * invGT[5];

    // shift to pixel centers to draw at centers
    px -= 0.5;
    py -= 0.5;

    return QPointF(px, py);
}

QPolygonF
GDALHandler::geoPolygonToPixels(const QList<QPointF>& geoPts) const
{
    QPolygonF pixPoly;
    pixPoly.reserve(geoPts.size());
    double invGT[6];
    double gt[6];
    // Fix for invaid conversion from const double* to double*
    std::memcpy(gt, geoTransform, sizeof(gt));

    if (!GDALInvGeoTransform(gt, invGT))
        return pixPoly;

    for (int i = 0; i < geoPts.size(); ++i)
    {
        const QPointF& g = geoPts.at(i);
        double Xgeo = g.x();
        double Ygeo = g.y();

        double px = invGT[0] + Xgeo * invGT[1] + Ygeo * invGT[2];
        double py = invGT[3] + Xgeo * invGT[4] + Ygeo * invGT[5];

        pixPoly << QPointF(px, py);
    }
    return pixPoly;
}

QPolygonF
GDALHandler::geoPolygonToPixels(const QPolygonF& geoPoly) const
{
    QPolygonF pixPoly;
    pixPoly.reserve(geoPoly.size());
    double invGT[6];
    double gt[6];
    std::memcpy(gt, geoTransform, sizeof(gt));

    if (!GDALInvGeoTransform(gt, invGT))
        return pixPoly;

    for (int i = 0; i < geoPoly.size(); ++i)
    {
        const QPointF& g = geoPoly.at(i);
        double Xgeo = g.x();
        double Ygeo = g.y();

        double px = invGT[0] + Xgeo * invGT[1] + Ygeo * invGT[2];
        double py = invGT[3] + Xgeo * invGT[4] + Ygeo * invGT[5];

        pixPoly << QPointF(px, py);
    }
    return pixPoly;
}