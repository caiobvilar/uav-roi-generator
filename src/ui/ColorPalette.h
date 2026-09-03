#ifndef UI_COLORPALETTE_H
#define UI_COLORPALETTE_H

#include <QColor>
#include <QImage>
#include <QRectF>
#include <QVector>

namespace color {

QColor analyzeBackgroundColor(const QImage& image, const QRectF& region = QRectF());
QVector<QColor> generateContrastingPalette(const QColor& backgroundColor, int numColors);

} // namespace color

#endif // UI_COLORPALETTE_H
