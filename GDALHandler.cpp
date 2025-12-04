#include "GDALHandler.h"

GDALHandler::GDALHandler()
{
    GDALAllRegister();
}

bool GDALHandler::openSrcRaster(const QString &fileName)
{
    closeRaster();  // in case something was open

    srcDataset = static_cast<GDALDataset *>(
        GDALOpen(fileName.toStdString().c_str(), GA_ReadOnly));
    if (!srcDataset)
        return false;

    return true;
}

void GDALHandler::closeRaster()
{
    if (srcDataset) {
        GDALClose(srcDataset);
        srcDataset = nullptr;
    }
    if (destDataset) {
        GDALClose(destDataset);
        destDataset = nullptr;
    }
}
QImage GDALHandler::toQImage() const
{
    if (!srcDataset)
        return QImage();

    const int width  = srcDataset->GetRasterXSize();
    const int height = srcDataset->GetRasterYSize();
    const int bands  = srcDataset->GetRasterCount();
    if (bands < 3)
        return QImage();

    GDALRasterBand *rb = srcDataset->GetRasterBand(1);
    GDALRasterBand *gb = srcDataset->GetRasterBand(2);
    GDALRasterBand *bb = srcDataset->GetRasterBand(3);
    if (!rb || !gb || !bb)
        return QImage();

    GDALDataType dtR = rb->GetRasterDataType();
    GDALDataType dtG = gb->GetRasterDataType();
    GDALDataType dtB = bb->GetRasterDataType();

    qDebug() << "Raster types:" << dtR << dtG << dtB;

    // Accept only 8‑bit here
    if (!(dtR == GDT_Byte && dtG == GDT_Byte && dtB == GDT_Byte)) {
        qWarning("GDALHandler::toQImage: unsupported band type");
        return QImage();
    }

    const int pixelCount = width * height;
    std::vector<uint8_t> rBuf(pixelCount);
    std::vector<uint8_t> gBuf(pixelCount);
    std::vector<uint8_t> bBuf(pixelCount);

    if (rb->RasterIO(GF_Read, 0, 0, width, height,
                     rBuf.data(), width, height, GDT_Byte, 0, 0) != CE_None)
        return QImage();
    if (gb->RasterIO(GF_Read, 0, 0, width, height,
                     gBuf.data(), width, height, GDT_Byte, 0, 0) != CE_None)
        return QImage();
    if (bb->RasterIO(GF_Read, 0, 0, width, height,
                     bBuf.data(), width, height, GDT_Byte, 0, 0) != CE_None)
        return QImage();

    QImage img(width, height, QImage::Format_RGB888);
    if (img.isNull())
        return QImage();

    for (int y = 0; y < height; ++y) {
        uchar *dst = img.scanLine(y);
        int rowOff = y * width;
        for (int x = 0; x < width; ++x) {
            int i = rowOff + x;
            dst[3*x + 0] = rBuf[i];
            dst[3*x + 1] = gBuf[i];
            dst[3*x + 2] = bBuf[i];
        }
    }

    return img;
}
