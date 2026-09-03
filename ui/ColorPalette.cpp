#include "ui/ColorPalette.h"

#include "constants.h"

#include <QtGlobal>

namespace color {

QColor
analyzeBackgroundColor(const QImage& image, const QRectF& region)
{
    if (image.isNull())
        return QColor(128, 128, 128); // Default to gray if no image

    // Determine the sampling region
    QRect sampleRect;
    if (region.isValid() && !region.isEmpty())
    {
        sampleRect = region.toRect().intersected(image.rect());
    }
    else
    {
        sampleRect = image.rect();
    }

    if (sampleRect.isEmpty())
        return QColor(128, 128, 128);

    // Sample pixels at regular intervals for efficiency (don't need every pixel)
    const int sampleStep = qMax(1, qMin(sampleRect.width(), sampleRect.height()) / 50);
    qint64 totalR = 0, totalG = 0, totalB = 0;
    int sampleCount = 0;

    for (int y = sampleRect.top(); y < sampleRect.bottom(); y += sampleStep)
    {
        for (int x = sampleRect.left(); x < sampleRect.right(); x += sampleStep)
        {
            QColor pixelColor = image.pixelColor(x, y);
            totalR += pixelColor.red();
            totalG += pixelColor.green();
            totalB += pixelColor.blue();
            sampleCount++;
        }
    }

    if (sampleCount == 0)
        return QColor(128, 128, 128);

    return QColor(totalR / sampleCount, totalG / sampleCount, totalB / sampleCount);
}

QVector<QColor>
generateContrastingPalette(const QColor& backgroundColor, int numColors)
{
    QVector<QColor> palette;
    palette.reserve(numColors);

    // Calculate background luminance (perceived brightness)
    // Using ITU-R BT.709 luminance formula
    double bgLuminance =
        0.2126 * backgroundColor.redF() + 0.7152 * backgroundColor.greenF() + 0.0722 * backgroundColor.blueF();

    // Get background HSV for smarter color selection
    int bgHue, bgSat, bgVal;
    backgroundColor.getHsv(&bgHue, &bgSat, &bgVal);

    // Determine if we need light or dark colors based on background
    bool needLightColors = bgLuminance < 0.5;

    // Target value (brightness) for generated colors
    int targetValue = needLightColors ? 255 : 200;
    int targetSaturation = 255; // High saturation for visibility

    // For very light backgrounds, use darker saturated colors
    if (bgLuminance > 0.7)
    {
        targetValue = constants::kAlphaLabelBg;
        targetSaturation = 255;
    }

    // Generate colors evenly distributed around the color wheel
    // Offset from background hue to avoid similar colors
    int hueOffset = (bgHue + constants::kHueOpposite) % constants::kHueFullCircle; // Start opposite to background

    for (int i = 0; i < numColors; ++i)
    {
        // Distribute hues evenly, starting from opposite of background
        int hue = (hueOffset + (i * constants::kHueFullCircle) / numColors) % constants::kHueFullCircle;

        // Avoid hues too close to the background hue (within 30 degrees)
        if (bgSat > 50) // Only if background has significant saturation
        {
            int hueDiff = qAbs(hue - bgHue);
            if (hueDiff > constants::kHueOpposite)
                hueDiff = constants::kHueFullCircle - hueDiff;
            if (hueDiff < 30)
            {
                hue = (hue + constants::kHueShiftAmount) % constants::kHueFullCircle; // Shift away from background
            }
        }

        // Vary saturation and value slightly for visual distinction
        int sat = targetSaturation - (i % 3) * 20;
        int val = targetValue - (i % 2) * 30;

        QColor color = QColor::fromHsv(hue, sat, val);

        // Final contrast check - ensure minimum contrast ratio
        double colorLuminance = 0.2126 * color.redF() + 0.7152 * color.greenF() + 0.0722 * color.blueF();
        double contrastRatio = (qMax(bgLuminance, colorLuminance) + 0.05) / (qMin(bgLuminance, colorLuminance) + 0.05);

        // If contrast is too low, adjust brightness
        if (contrastRatio < 3.0)
        {
            if (needLightColors)
                color = color.lighter(150);
            else
                color = color.darker(150);
        }

        palette.append(color);
    }

    return palette;
}

QColor
getContrastingTextColor(const QColor& backgroundColor)
{
    // Calculate luminance
    double luminance =
        0.2126 * backgroundColor.redF() + 0.7152 * backgroundColor.greenF() + 0.0722 * backgroundColor.blueF();

    // Return white for dark backgrounds, black for light backgrounds
    // Using WCAG recommended threshold
    if (luminance < 0.5)
        return QColor(255, 255, 255); // White
    else
        return QColor(0, 0, 0); // Black
}

} // namespace color
