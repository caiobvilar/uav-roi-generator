# ROIGenerator — S.O.L.I.D. Refactor & Code Quality Design

- **Date:** 2026-09-03
- **Status:** Draft (pending user review)
- **Scope:** Full codebase refactor (dead code, bugs, build, readability, performance, SOLID architecture, tests)

## 1. Context

`uav-roi-generator` is a Qt6 + GDAL desktop app (C++/CMake) that generates drone
coverage waypoints over a user-drawn ROI on a georeferenced image. The path
planning implements a published multi-UAV coverage-decomposition method.

Current shape: ~2,300 lines across 14 files. The codebase works but carries a
large amount of dead code, duplicated logic, a `#define`-heavy catch-all header,
and a monolithic `ROIArea` widget (~1,830 lines) that violates most of S.O.L.I.D.

This document is the design for a staged refactor that fixes quality,
performance, and readability issues while bringing the project into S.O.L.I.D.
compliance. It is a **plan document**, not code.

## 2. Goals

- Remove all dead code and orphan files.
- Fix correctness bugs (reprojection divergence, disabled logging, GDAL resource
  lifetime, non-portable `M_PI`).
- Clean the build system and enable warnings-as-errors.
- Replace `#define` constants with typed `constexpr`.
- Conform to S.O.L.I.D. (SRP, OCP, LSP, ISP, DIP).
- Add a unit-test harness (CTest) for the pure geometry layer.

## 3. Non-goals

- No functional/algorithm changes: the decomposition and boustrophedon math
  behavior is preserved exactly.
- No new features.
- No change to the drone JSON schema or exported GeoJSON format.

## 4. Staged migration plan

Execution order is chosen to de-risk: remove dead code first, extract pure
geometry early (so tests can lock behavior), then refactor upwards into UI and
strategy interfaces.

### Step 1 — Dead code removal

Remove approximately 450 lines of orphaned algorithms plus 3 orphan files.

| File | Removals |
|---|---|
| `pathplanner.cpp` | `compute_partitioned_area`, `binary_search`, `getBoundingBox`, `makeDividerPoly`, `rectToTransform`, `findLongestBoundingLineWithSlope`, `computeInternalAngle`, `computeDistanceWithAngleAdjustment`, `computeWaypointsLoop` |
| `pathplanner.h` | Declarations for all of the above |
| `ROIArea.cpp` | `generateSweepWaypoints`, `drawGeoPolygonOnImage` (logic folded into `drawGeoPolygonOnCurrentOverlay`), `reprojectGeoJSONPolygon` (duplicate of GDALHandler) |
| `ROIArea.h` | Declarations for `generateSweepWaypoints`, `reprojectGeoJSONPolygon` |
| `ui.h` / `ui.cpp` / `ui.ui` | Delete files (unreferenced `UI` class) |

The live call paths — `decomposedROI` (own inline binary search) and
`computeWaypointsWithMAR` — depend on none of the removed code.

### Step 2 — Bug fixes

1. **Single reprojection path.** `exportPolygonGeoJSON` uses only
   `GDALHandler::reprojectGeoJSONPolygon`. The GDAL handler is the source of
   truth for the lon/lat axis swap (GeoJSON requires `[lon, lat]`).
2. **Logging.** Remove `QLoggingCategory::setFilterRules("*.warning=true")` in
   `main.cpp` (it silently disables the pervasive `qInfo/qDebug` output).
3. **GDAL RAII.** Add `~GDALHandler() { closeRaster(); }`; delete copy
   constructor/assignment (raw pointer + C array member); remove unused
   `destDataset`.
4. **`M_PI` portability.** Replace with `std::numbers::pi` (C++20) or a local
   `constexpr double kPi`.

### Step 3 — Extract pure geometry + domain (pre-test)

New headers/impls with no Qt GUI dependency, enabling Step 8 tests:

- `geometry/ConvexHull.{h,cpp}` — `cross`, `dist2`, `grahamScan` (replaces
  `GrahamScan` QObject).
- `geometry/PolygonGeometry.{h,cpp}` — `dot`, `perp`, `shoelaceArea`,
  `minimumAreaRectangle`, `rotatedRectToPolygon`.
- `geometry/PolygonClipping.{h,cpp}` — `sutherlandHodgmanClip`.
- `domain/RotatedRect.h` — `RotatedRect` struct (moved from `utils.h`).
- `domain/GeoValidation.{h,cpp}` — `isPolygonWGS84`, `isValueInMeters`,
  `validateCoordinateSystemMatch`, `validateFootprintMeters`.

### Step 4 — Build, constants, conventions

- `CMakeLists.txt`: single `find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui
  Widgets)`; single `target_link_libraries(... Qt6::Core Qt6::Gui Qt6::Widgets
  GDAL::GDAL)`; move `qt_standard_project_setup()` above `qt_add_executable`;
  add `-Wall -Wextra -Wpedantic`; `enable_testing()`.
- `utils.h` → `constants::` namespace with typed `inline constexpr` (full list
  in §6). Structs `RotatedRect`, `drone` move to dedicated headers (Step 5); the
  unused `PathSegment` struct is removed as dead code in Step 1.
- Rename `struct drone` → `struct Drone` (all 12 usages).
- Harmonize math helpers to `std::` (drop `qAbs/qMin/qMax/qBound` where
  `<cmath>`/`<algorithm>` suffice).
- Remove `QObject` base from `GrahamScan` and `PathPlanner` (LSP). Update
  `ROIArea.cpp` constructor that passes `this` as parent.

### Step 5 — `io/` layer (SRP + bug fix tie-in)

- `io/GDALHandler.{h,cpp}` — raster open/close, geotransform, `pixelToGeo`,
  `geoToPixel`, `toQImage`, `getDataSetCRS`. Cache inverse geotransform (see §7).
- `io/GeoJSON.{h,cpp}` — `importPolygon`, `exportPolygon`, `reproject`
  (extracted from `GDALHandler::reprojectGeoJSONPolygon` and
  `loadPolygonFromGeoJSON`).
- `domain/Drone.{h,cpp}` — `struct Drone` + the physics calc functions
  (`calcFlightAltitude`, `calcDroneCameraFootprint`, `calcMaximumForwardVelocity`,
  `calcDroneRelativeCapability`) as pure functions over `QList<Drone>&`.

### Step 6 — `ui/` layer (SRP + ISP)

Split the monolith `ROIArea`:

- `ui/ImageCanvas.{h,cpp}` — QWidget: mouse/zoom/pan input, overlay stack,
  `paintEvent`, polygon drawing. Owns composition of `GDALHandler&`,
  `PathPlanner&` injected via constructor (DIP).
- `ui/LabelPlacer.{h,cpp}` — pure label placement (exact API in §5.1).
- `ui/ColorPalette.{h,cpp}` — `analyzeBackgroundColor`,
  `generateContrastingPalette`, `getContrastingTextColor`.

Expose minimal interfaces to `MainWindow` (ISP):

- `IImageDocument` — `open/close/save/export`.
- `IRoiProvider` — `finalPolygon`, `mar`.
- `IMissionExporter` — `exportGeoJSON`.

`MainWindow` depends on these interfaces, not the concrete widget.

### Step 7 — `planning/` layer (OCP + DIP)

- `planning/IDecompositionStrategy.h` — interface with
  `decompose(const QPolygonF&, const QList<Drone>&, const RotatedRect&) -> QList<QPair<QPolygonF,QString>>`.
- `planning/IWaypointGenerator.h` — interface with
  `generate(const QPolygonF&, const Drone&, const RotatedRect&) -> QList<QPointF>`.
- `planning/StripDecomposition.{h,cpp}` — current strip binary-search logic.
- `planning/BoustrophedonSweep.{h,cpp}` — current MAR sweep logic.
- `planning/PathPlanner.{h,cpp}` — orchestrator that **receives** the two
  strategies by injection (constructor or setters). No algorithm hardcoding.

New strategies are added without modifying `PathPlanner` (OCP).

### Step 8 — Tests (CTest)

- Add `Qt6::Test` and a `tests/` target linking `geometry/`, `domain/`,
  `planning/` objects.
- Cases: Graham scan (square, collinear, <3 pts), shoelace area (CCW/CW, known
  triangle), Sutherland–Hodgman (intersection/empty/contained),
  `minimumAreaRectangle` (rotated square/rectangle), `decomposedROI`
  (sum of areas ≈ total; proportions ≈ capabilities), `LabelPlacer` (non-overlap,
  in-bounds).

## 5. New component contracts

### 5.1 `LabelPlacer`

```cpp
class LabelPlacer {
public:
    LabelPlacer(qreal imageWidth, qreal imageHeight,
                qreal margin = 5.0, qreal rotationDeg = -30.0);
    QPointF place(const QRectF& bbox, const QFontMetrics& fm);
    void clear();
    qreal imageWidth()  const;
    qreal imageHeight() const;
private:
    QList<QPointF> candidates(const QRectF&, const QFontMetrics&) const;
    bool overlapsPlaced(const QRectF&) const;
    bool isValid(const QPointF&, const QFontMetrics&, QRectF*) const;
    qreal m_imageWidth, m_imageHeight, m_margin, m_rotationDeg;
    QList<QRectF> m_placedLabels;
};
```

```cpp
void drawLabel(QPainter&, const QString& text, const QColor& fg,
               const QColor& bg, int bgAlpha, const QPointF& anchor,
               const QFontMetrics& fm, qreal rotationDeg);
```

Replaces the duplicated label logic in `showDecomposedROI` and `showWaypoints`.

### 5.2 Planning interfaces

```cpp
class IDecompositionStrategy {
public:
    virtual ~IDecompositionStrategy() = default;
    virtual QList<QPair<QPolygonF, QString>>
    decompose(const QPolygonF& roi, const QList<Drone>&, const RotatedRect&) const = 0;
};

class IWaypointGenerator {
public:
    virtual ~IWaypointGenerator() = default;
    virtual QList<QPointF>
    generate(const QPolygonF& subRoi, const Drone&, const RotatedRect&) const = 0;
};
```

## 6. Full `constants::` list (replaces `utils.h` `#define`s)

```cpp
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
}
```

## 7. Performance notes

- Cache inverse geotransform once per `openSrcRaster`, reuse in `geoToPixel` and
  both `geoPolygonToPixels` overloads (remove per-call `GDALInvGeoTransform`).
- `toQImage()`: optional single-buffer RGB read; a micro-optimization, only if
  it does not hurt readability.
- Label placement consolidation also removes duplicate bounding-box scans.

## 8. Testing / verification

- `ctest` runs the geometry/planning suites (Step 8) after migration.
- Manual smoke: open a georeferenced image, draw ROI, load `artifacts/drones.json`,
  decompose, generate waypoints, export GeoJSON. Compare exported GeoJSON
  coordinates against pre-refactor output (should be byte-identical).

## 9. File inventory (target state)

```
CMakeLists.txt            (cleaned, + tests + warnings)
src/
  main.cpp
  geometry/{ConvexHull,PolygonGeometry,PolygonClipping}.{h,cpp}
  domain/{Drone,RotatedRect,GeoValidation}.{h,cpp}
  planning/{IDecompositionStrategy,IWaypointGenerator,StripDecomposition,
            BoustrophedonSweep,PathPlanner}.{h,cpp}
  io/{GDALHandler,GeoJSON}.{h,cpp}
  ui/{ImageCanvas,LabelPlacer,ColorPalette,MainWindow}.{h,cpp}
  ui/mainwindow.ui
  constants.h             (was utils.h)
tests/
  tests.cpp (or per-suite files)
```
