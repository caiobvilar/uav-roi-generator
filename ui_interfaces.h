#ifndef UI_INTERFACES_H
#define UI_INTERFACES_H

#include "domain/RotatedRect.h"

#include <QByteArray>
#include <QPolygonF>
#include <QString>

class IImageDocument
{
  public:
    virtual ~IImageDocument() = default;
    virtual bool openImage(const QString&) = 0;
    virtual bool closeImage() = 0;
    virtual bool saveImage(const QString&, const char*) = 0;
};

class IRoiProvider
{
  public:
    virtual ~IRoiProvider() = default;
    virtual QPolygonF finalPolygon() const = 0;
    virtual RotatedRect mar() const = 0;
};

class IMissionExporter
{
  public:
    virtual ~IMissionExporter() = default;
    virtual QByteArray exportGeoJSON() const = 0;
};

#endif
