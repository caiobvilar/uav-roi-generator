#include "ui/LabelPlacer.h"

#include <QVector>
#include <QtGlobal>
#include <QtMath>
#include <cmath>

LabelPlacer::LabelPlacer(qreal imageWidth, qreal imageHeight, qreal margin, qreal rotationDeg)
    : m_imageWidth(imageWidth), m_imageHeight(imageHeight), m_margin(margin), m_rotationDeg(rotationDeg)
{
}

QPointF
LabelPlacer::place(const QRectF& bbox, const QString& text, const QFontMetrics& fm)
{
    const int textWidth = fm.horizontalAdvance(text);
    const int textHeight = fm.height();
    const qreal imgW = m_imageWidth;
    const qreal imgH = m_imageHeight;

    const QList<QPointF> posCandidates = candidates(bbox, text, fm);

    QPointF labelPt = posCandidates.value(0);
    bool foundValidPosition = false;

    // First pass: try all candidate positions
    for (const QPointF& candidate : posCandidates)
    {
        if (isValid(candidate, text, fm))
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
                if (isValid(shifted, text, fm))
                {
                    labelPt = shifted;
                    foundValidPosition = true;
                    break;
                }
                shifted = candidate - QPointF(0, yOffset);
                if (isValid(shifted, text, fm))
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

    // Record this label's rotated axis-aligned bounding box
    m_placedLabels.append(rotatedFootprint(labelPt, textWidth, textHeight));

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
LabelPlacer::candidates(const QRectF& bbox, const QString& text, const QFontMetrics& fm) const
{
    const qreal margin = m_margin;
    const int textWidth = fm.horizontalAdvance(text);
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
LabelPlacer::isValid(const QPointF& pt, const QString& text, const QFontMetrics& fm) const
{
    const qreal textWidth = fm.horizontalAdvance(text);
    const qreal textHeight = fm.height();

    const QRectF labelRect = rotatedFootprint(pt, textWidth, textHeight);

    return labelRect.left() >= 0 && labelRect.top() >= 0 && labelRect.right() <= m_imageWidth &&
           labelRect.bottom() <= m_imageHeight && !overlapsPlaced(labelRect);
}

QRectF
LabelPlacer::rotatedFootprint(const QPointF& anchor, qreal textWidth, qreal textHeight) const
{
    const double theta = qDegreesToRadians(m_rotationDeg);
    const double c = std::cos(theta);
    const double s = std::sin(theta);

    const double halfW = (std::fabs(textWidth * c) + std::fabs(textHeight * s)) / 2.0;
    const double halfH = (std::fabs(textWidth * s) + std::fabs(textHeight * c)) / 2.0;

    // Center of the label's local rect (x in [0, textWidth], y in [-textHeight, 0])
    const double centerX = (textWidth / 2.0) * c - (-textHeight / 2.0) * s;
    const double centerY = (textWidth / 2.0) * s + (-textHeight / 2.0) * c;
    const QPointF center(anchor.x() + centerX, anchor.y() + centerY);

    return QRectF(center.x() - halfW, center.y() - halfH, 2.0 * halfW, 2.0 * halfH);
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
