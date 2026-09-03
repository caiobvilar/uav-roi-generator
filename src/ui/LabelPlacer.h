#ifndef UI_LABELPLACER_H
#define UI_LABELPLACER_H

#include <QColor>
#include <QFontMetrics>
#include <QList>
#include <QPainter>
#include <QPointF>
#include <QRectF>
#include <QString>

class LabelPlacer
{
  public:
    LabelPlacer(qreal imageWidth, qreal imageHeight, qreal margin = 5.0, qreal rotationDeg = -30.0);
    QPointF place(const QRectF& bbox, const QString& text, const QFontMetrics& fm);
    void clear();
    qreal imageWidth() const;
    qreal imageHeight() const;

  private:
    QList<QPointF> candidates(const QRectF&, const QString&, const QFontMetrics&) const;
    bool overlapsPlaced(const QRectF&) const;
    bool isValid(const QPointF&, const QString&, const QFontMetrics&, QRectF*) const;
    qreal m_imageWidth;
    qreal m_imageHeight;
    qreal m_margin;
    qreal m_rotationDeg;
    QList<QRectF> m_placedLabels;
};

void drawLabel(QPainter& painter, const QString& text, const QColor& fg, const QColor& bg, int bgAlpha,
               const QPointF& anchor, const QFontMetrics& fm, qreal rotationDeg);

#endif // UI_LABELPLACER_H
