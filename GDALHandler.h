#ifndef GDALHANDLER_H
#define GDALHANDLER_H

#include <QString>
#include <QImage>
#include <QDebug>
#include <gdal_priv.h>
#include <cpl_conv.h> // CPLMalloc

class GDALHandler
{
public:
    GDALHandler();

    bool openSrcRaster(const QString &fileName);
    void closeRaster();

    GDALDataset *getDataset() const { return srcDataset; }

    // NEW: build a QImage from the current dataset (band 1 = grayscale)
    QImage toQImage() const;

private:
    GDALDataset *srcDataset = nullptr;
    GDALDataset *destDataset = nullptr;
};

#endif // GDALHANDLER_H
