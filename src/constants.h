#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace constants {
inline constexpr double kDesiredGsd       = 0.02;
inline constexpr double kForwardOverlap   = 0.8;
inline constexpr double kSideOverlap      = 0.75;
inline constexpr int    kWidgetMinHeight  = 650;
inline constexpr int    kWidgetMinWidth   = 1000;
inline constexpr int    kFontSizeSmall    = 12;
inline constexpr int    kFontSizeLarge    = 14;
inline constexpr double kPenWidthDefault  = 2.0;
inline constexpr int    kPenWidthMedium   = 3;
inline constexpr int    kPenWidthThick    = 4;
inline constexpr int    kAlphaSubroiFill  = 50;
inline constexpr int    kAlphaRoiOutline  = 80;
inline constexpr int    kAlphaLabelBg     = 180;
inline constexpr int    kLightBackgroundValue = 180;
inline constexpr double kZoomStep         = 1.001;
inline constexpr double kZoomMin          = 0.1;
inline constexpr double kZoomMax          = 20.0;
inline constexpr double kLabelRotationDeg = -30.0;
inline constexpr double kLabelMarginSmall = 5.0;
inline constexpr int    kLabelMarginLarge = 10;
inline constexpr double kDotRadiusMin     = 0.5;
inline constexpr double kDotRadiusMax     = 2.0;
inline constexpr double kDotRadiusScale   = 0.0005;
inline constexpr double kEpsilonSmall     = 1e-8;
inline constexpr double kEpsilonTiny      = 1e-12;
inline constexpr int    kHueFullCircle    = 360;
inline constexpr int    kHueOpposite      = 180;
inline constexpr int    kHueShiftAmount   = 60;
inline constexpr int    kMarSizeMax       = 100000;
} // namespace constants

#endif
