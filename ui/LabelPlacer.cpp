#include "ui/LabelPlacer.h"

#include <QVector>
#include <QtGlobal>

LabelPlacer::LabelPlacer(qreal imageWidth, qreal imageHeight, qreal margin, qreal rotationDeg)
    : m_imageWidth(imageWidth), m_imageHeight(imageHeight), m_margin(margin), m_rotationDeg(rotationDeg)
{
}

QPointF
LabelPlacer::place(const QRectF& bbox, const QFontMetrics& fm)
{
    const int textWidth = fm.maxWidth();
    const int textHeight = fm.height();
    const qreal imgW = m_imageWidth;
    const qreal imgH = m_imageHeight;

    const QList<QPointF> posCandidates = candidates(bbox, fm);

    QPointF labelPt = posCandidates.value(0);
    bool foundValidPosition = false;

    // First pass: try all candidate positions
    for (const QPointF& candidate : posCandidates)
    {
        if (isValid(candidate, fm, nullptr))
        {
            labelPt = candidate;
            foundValidPosition = true;
            break;
        }
    }

    // Second pass: if no valid position, try vertical offsets to avoid overlap
    if (!foundValidPosition)
    {
        for (const QPointF& candidate : posCandidates)
        {
            for (int yOffset = 0; yOffset <= imgH; yOffset += textHeight + 5)
            {
                QPointF shifted = candidate + QPointF(0, yOffset);
                if (isValid(shifted, fm, nullptr))
                {
                    labelPt = shifted;
                    foundValidPosition = true;
                    break;
                }
                shifted = candidate - QPointF(0, yOffset);
                if (isValid(shifted, fm, nullptr))
                {
                    labelPt = shifted;
                    foundValidPosition = true;
                    break;
                }
            }
            if (foundValidPosition)
                break;
        }
    }

    // Final clamp to ensure label stays within image bounds
    labelPt.setX(qBound(2.0, labelPt.x(), qreal(imgW - textWidth - 2)));
    labelPt.setY(qBound(qreal(textHeight + 2), labelPt.y(), qreal(imgH - 2)));

    // Record this label's bounding rect (approximate for rotated text)
    QRectF labelRect(labelPt.x() - 2, labelPt.y() - textHeight - 2, textWidth + 4, textHeight + 4);
    m_placedLabels.append(labelRect);

    return labelPt;
}

void
LabelPlacer::clear()
{
    m_placedLabels.clear();
}

qreal
LabelPlacer::imageWidth() const
{
    return m_imageWidth;
}

qreal
LabelPlacer::imageHeight() const
{
    return m_imageHeight;
}

QList<QPointF>
LabelPlacer::candidates(const QRectF& bbox, const QFontMetrics& fm) const
{
    const qreal margin = m_margin;
    const int textWidth = fm.maxWidth();
    const int textHeight = fm.height();

    const qreal roiMinX = bbox.left();
    const qreal roiMinY = bbox.top();
    const qreal roiMaxX = bbox.right();
    const qreal roiMaxY = bbox.bottom();

    return QList<QPointF>{QPointF(roiMinX - margin, roiMinY - margin),                 // Above top-left (outside)
                          QPointF(roiMaxX + margin, roiMinY - margin),                 // Above top-right (outside)
                          QPointF(roiMinX - textWidth - margin, roiMinY + textHeight), // Left of top-left (outside)
                          QPointF(roiMaxX + margin, roiMaxY)};                         // Right of bottom-right (outside)
}

bool
LabelPlacer::overlapsPlaced(const QRectF& rect) const
{
    for (const QRectF& placed : m_placedLabels)
    {
        if (rect.intersects(placed))
            return true;
    }
    return false;
}

bool
LabelPlacer::isValid(const QPointF& pt, const QFontMetrics& fm, QRectF* outRect) const
{
    const int textWidth = fm.maxWidth();
    const int textHeight = fm.height();

    QRectF labelRect(pt.x() - 2, pt.y() - textHeight, textWidth + 4, textHeight + 4);
    const bool valid = pt.x() >= 0 && pt.x() + textWidth <= m_imageWidth && pt.y() - textHeight >= 0 &&
                       pt.y() <= m_imageHeight && !overlapsPlaced(labelRect);

    if (outRect)
        *outRect = labelRect;

    return valid;
}

void
drawLabel(QPainter& painter, const QString& text, const QColor& fg, const QColor& bg, int bgAlpha,
          const QPointF& anchor, const QFontMetrics& fm, qreal rotationDeg)
{
    const int textWidth = fm.horizontalAdvance(text);
    const int textHeight = fm.height();

    painter.save();
    painter.translate(anchor);
    painter.rotate(rotationDeg);

    // Draw background rect at rotated position
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(bg.red(), bg.green(), bg.blue(), bgAlpha));
    painter.drawRect(QRectF(-2, -textHeight, textWidth + 4, textHeight + 4));

    // Draw text
    painter.setPen(fg);
    painter.drawText(QPointF(0, 0), text);

    painter.restore();
}
