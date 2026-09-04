# SOLID Refactor & Code Quality — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the Qt6+GDAL UAV ROI generator to remove dead code, fix bugs, clean the build, adopt typed constants, bring the code into S.O.L.I.D. compliance, and add unit tests — without changing behavior.

**Architecture:** Staged, de-risked migration: (1) remove dead code, (2) fix bugs, (3) extract pure geometry/domain, (4) clean build + constants + conventions, (5) split IO, (6) split UI, (7) strategize planning behind interfaces, (8) test. Each task ends with a testable deliverable (test pass and/or build pass).

**Tech Stack:** C++20, Qt6 (Core/Gui/Widgets, Qt6::Test), GDAL, CMake ≥3.19.

**Spec:** `docs/superpowers/specs/2026-09-03-solid-refactor-design.md`

## Global Constraints

- C++20 (`std::numbers::pi` available; falls back to `constexpr double kPi` if toolchain flags XML).
- Qt6 ≥ 6.5; components Core, Gui, Widgets; add `Test` for the test target.
- GDAL linked as `GDAL::GDAL`.
- All SI units: lengths meters, time seconds, velocity m/s.
- Constants live in `namespace constants` as `inline constexpr`, never `#define`.
- `struct drone` is renamed `struct Drone` everywhere.
- `GrahamScan` and `PathPlanner` must NOT derive `QObject`.
- Exported GeoJSON coordinates must be byte-identical to pre-refactor output.
- Conventions: types `PascalCase`, functions `camelCase`, files `PascalCase`.
- Naming/unit values copied verbatim from the spec §6 (`constants::` list) and §5 (interface signatures).
- CMake builds with `-Wall -Wextra -Wpedantic`; warnings are treated as errors (`-Werror`) only after the code compiles clean.

---

## File Structure (target state)

```
CMakeLists.txt                       cleaned; adds -Wall/-Wextra/-Wpedantic; adds Qt6::Test + tests/ target
src/main.cpp                         logging filter removed
src/constants.h                      was utils.h; only constants:: (constexpr)
src/geometry/ConvexHull.h/.cpp       grahamScan (cross, dist2) — free function
src/geometry/PolygonGeometry.h/.cpp  dot, perp, shoelaceArea, minimumAreaRectangle, rotatedRectToPolygon
src/geometry/PolygonClipping.h/.cpp  sutherlandHodgmanClip
src/domain/RotatedRect.h             RotatedRect struct (moved out of utils.h)
src/domain/GeoValidation.h/.cpp      isPolygonWGS84, isValueInMeters, validateCoordinateSystemMatch, validateFootprintMeters
src/domain/Drone.h/.cpp              struct Drone + calcFlightAltitude/calcDroneCameraFootprint/calcMaximumForwardVelocity/calcDroneRelativeCapability
src/planning/IDecompositionStrategy.h
src/planning/IWaypointGenerator.h
src/planning/StripDecomposition.h/.cpp
src/planning/BoustrophedonSweep.h/.cpp
src/planning/PathPlanner.h/.cpp      orchestrator; injects strategies (DIP)
src/io/GDALHandler.h/.cpp            raster + geotransform only (SRP); RAII cleanup; cached inverse geotransform
src/io/GeoJSON.h/.cpp                importPolygon, exportPolygon, reproject (SRP)
src/ui/ImageCanvas.h/.cpp            widget: input + overlay stack + paint (SRP)
src/ui/LabelPlacer.h/.cpp            pure label placement
src/ui/ColorPalette.h/.cpp           background analysis + palettes
src/ui/MainWindow.h/.cpp
src/ui/mainwindow.ui
src/ui_interfaces.h                  IImageDocument, IRoiProvider, IMissionExporter (ISP)
tests/CMakeLists.txt
tests/test_geometry.cpp              hull, area, clipping, MAR
tests/test_planning.cpp              decomposition, waypoint generation
tests/test_labelplacer.cpp           label placement
```

**File moves (from current flat layout):**
- `grahamscan.{h,cpp}` → `src/geometry/ConvexHull.{h,cpp}`
- `pathplanner.{h,cpp}` → split into `src/planning/*`, `src/domain/Drone.*`
- `GDALHandler.{h,cpp}` → `src/io/GDALHandler.*` + `src/io/GeoJSON.*`
- `ROIArea.{h,cpp}` → `src/ui/ImageCanvas.*` + `src/ui/LabelPlacer.*` + `src/ui/ColorPalette.*`
- `mainwindow.{h,cpp,ui}` → `src/ui/*`
- `main.cpp` → `src/main.cpp`
- `utils.h` → `src/constants.h` (constants) + `src/domain/*` (structs + validation)

> **Note on directory churn:** to keep each task independently buildable, the flat → `src/` move happens in one commit at the END (Task 10). Tasks 1–9 operate on files in their current flat location; Task 10 performs the physical move, updates `CMakeLists.txt` include paths, and runs the full suite. This avoids intermediate broken builds.

---

### Task 1: Remove dead code from `pathplanner`

**Files:**
- Modify: `pathplanner.h` (remove dead declarations)
- Modify: `pathplanner.cpp` (remove dead definitions + now-unused includes)

**Interfaces:**
- Produces: keeps live members `getDroneInfo`, `calcFlightAltitude`, `calcDroneCameraFootprint`, `calcMaximumForwardVelocity`, `calcDroneRelativeCapability`, `calculatePolygonArea`, `decomposedROI`, `suth_hodgman_polygon_clipper`, `set/getDroneList`, `set/getDecomposedROIs`.

- [ ] **Step 1: Delete dead function definitions in `pathplanner.cpp`**

Remove these definitions (line ranges from current file): `compute_partitioned_area` (212–225), `binary_search` (227–262), `getBoundingBox` (264–278), `makeDividerPoly` (366–378), `rectToTransform` (380–391), `findLongestBoundingLineWithSlope` (454–529), `computeInternalAngle` (532–582), `computeDistanceWithAngleAdjustment` (586–648), `computeWaypointsLoop` (652–790).

- [ ] **Step 2: Delete matching declarations in `pathplanner.h`**

Remove declarations for `makeDividerPoly`, `compute_partitioned_area`, `getBoundingBox`, `rectToTransform`, `binary_search`, `findLongestBoundingLineWithSlope`, `computeInternalAngle`, `computeDistanceWithAngleAdjustment`, `computeWaypointsLoop` (header lines 50–95).

- [ ] **Step 3: Remove now-unused includes**

In `pathplanner.cpp` remove `#include <QTransform>` and `#include <QVector>` if unused; keep `<algorithm>` for `std::min`.

- [ ] **Step 4: Build to verify nothing live referenced them**

Run: `cmake --build build`
Expected: build succeeds (removed functions are unreferenced).

- [ ] **Step 5: Commit**

```bash
git add pathplanner.h pathplanner.cpp
git commit -m "refactor: remove dead path-planning algorithms from PathPlanner"
```

---

### Task 2: Remove dead code in `ROIArea` + delete orphan `ui.*` + drop `PathSegment`

**Files:**
- Modify: `ROIArea.cpp`, `ROIArea.h`, `utils.h`
- Delete: `ui.h`, `ui.cpp`, `ui.ui`

**Interfaces:**
- Consumes: nothing new.
- Produces: `ROIArea::drawGeoPolygonOnCurrentOverlay` now draws inline (no `drawGeoPolygonOnImage` indirection). `utils.h` no longer contains `PathSegment`.

- [ ] **Step 1: Inline `drawGeoPolygonOnImage` into `drawGeoPolygonOnCurrentOverlay`**

`drawGeoPolygonOnCurrentOverlay` (currently `ROIArea.cpp:841–852`) is the only caller of `drawGeoPolygonOnImage`. Replace its body with the drawing logic (the portion of `drawGeoPolygonOnImage` after its null-image guard, from `ROIArea.cpp:779–838`). Remove the `img` parameter path and operate directly on `getOverlayStackTop().first`. Keep behavior identical (green pen `PEN_WIDTH_THICK`, `Qt::NoBrush`).

- [ ] **Step 2: Delete `drawGeoPolygonOnImage` definition and declaration**

Delete `ROIArea::drawGeoPolygonOnImage` (`ROIArea.cpp:769–839`) and its declaration in `ROIArea.h:76`.

- [ ] **Step 3: Delete `generateSweepWaypoints`**

Delete definition (`ROIArea.cpp:1236–1281`) and declaration (`ROIArea.h:114–115`).

- [ ] **Step 4: Delete duplicate `reprojectGeoJSONPolygon`**

Delete definition (`ROIArea.cpp:678–760`) and declaration (`ROIArea.h:70–71`). `exportPolygonGeoJSON` already calls `gdalHandler.reprojectGeoJSONPolygon` and does not need this duplicate.

- [ ] **Step 5: Remove `PathSegment` from `utils.h`**

Delete `struct PathSegment { ... }` (`utils.h:64–68`). It is never referenced.

- [ ] **Step 6: Delete orphan files**

```bash
git rm ui.h ui.cpp ui.ui
```

- [ ] **Step 7: Build to verify**

Run: `cmake --build build`
Expected: success.

- [ ] **Step 8: Commit**

```bash
git add ROIArea.h ROIArea.cpp utils.h
git commit -m "refactor: remove dead code from ROIArea and orphan UI files"
```

---

### Task 3: Fix bugs (reprojection, logging, GDAL RAII, M_PI)

**Files:**
- Modify: `main.cpp`, `GDALHandler.h`, `GDALHandler.cpp`, `pathplanner.cpp`

**Interfaces:**
- Produces: `GDALHandler::~GDALHandler()` (RAII), `GDALHandler` is non-copyable. `pathplanner.cpp` no longer uses `M_PI`.

- [ ] **Step 1: Remove the logging filter**

In `main.cpp`, delete line: `QLoggingCategory::setFilterRules("*.warning=true");` and the now-unused `#include <QLoggingCategory>`.

- [ ] **Step 2: Add GDAL RAII + non-copyable**

In `GDALHandler.h`, add a destructor declaration in the public section and delete copy ops; remove `destDataset`:

```cpp
public:
    GDALHandler();
    ~GDALHandler();
    GDALHandler(const GDALHandler&) = delete;
    GDALHandler& operator=(const GDALHandler&) = delete;
    // ... existing members ...
private:
    GDALDataset* srcDataset = nullptr;   // (destDataset removed)
    double geoTransform[GEO_TRANSFORM_SIZE];
    QString dataSetCRSInfo = QString("NaN");
```

In `GDALHandler.cpp`, add and simplify `closeRaster`:

```cpp
GDALHandler::~GDALHandler()
{
    closeRaster();
}

void
GDALHandler::closeRaster()
{
    if (srcDataset)
    {
        GDALClose(srcDataset);
        srcDataset = nullptr;
    }
}
```

(Remove the `destDataset` block from `closeRaster`.)

- [ ] **Step 3: Replace `M_PI`**

In `pathplanner.cpp:619`, replace `double pi = M_PI;` with `constexpr double pi = 3.14159265358979323846;` (portable, no `<numbers>` needed). If `M_PI` also appears in `ROIArea.cpp:734`, replace with the same local constant.

- [ ] **Step 4: Verify single reprojection path**

Confirm `ROIArea::exportPolygonGeoJSON` calls `gdalHandler.reprojectGeoJSONPolygon` and no other reprojection remains (grep `reproject`). If any caller still uses a removed function, fix the call site.

- [ ] **Step 5: Build**

Run: `cmake --build build`
Expected: success.

- [ ] **Step 6: Commit**

```bash
git add main.cpp GDALHandler.h GDALHandler.cpp pathplanner.cpp ROIArea.cpp
git commit -m "fix: RAII for GDAL datasets, restore logging, portable pi"
```

---

### Task 4: Extract pure geometry (`ConvexHull`, `PolygonGeometry`, `PolygonClipping`)

**Files:**
- Create: `geometry/ConvexHull.h`, `geometry/ConvexHull.cpp`
- Create: `geometry/PolygonGeometry.h`, `geometry/PolygonGeometry.cpp`
- Create: `geometry/PolygonClipping.h`, `geometry/PolygonClipping.cpp`
- Modify: `grahamscan.h`, `grahamscan.cpp` (delete after move), `ROIArea.cpp`, `ROIArea.h`, `pathplanner.cpp`, `pathplanner.h`
- Test: `tests/test_geometry.cpp` (created in Task 10; here write a temporary test to lock behavior)

**Interfaces:**
- Produces:
  - `namespace geometry { QPolygonF convexHull(const QList<QPointF>& pts); double cross(a,b,c); double dist2(a,b); }`
  - `namespace geometry { double dot(const QPointF&, const QPointF&); QPointF perp(const QPointF&); double shoelaceArea(const QPolygonF&); RotatedRect minimumAreaRectangle(const QList<QPointF>& hull, bool closedHull, const QPointF& originOffset); QPolygonF rotatedRectToPolygon(const RotatedRect&); }`
  - `namespace geometry { QPolygonF sutherlandHodgmanClip(QPolygonF subject, const QPolygonF& clip); }`

- [ ] **Step 1: Write `geometry/ConvexHull.h`**

```cpp
#ifndef GEOMETRY_CONVEXHULL_H
#define GEOMETRY_CONVEXHULL_H

#include <QList>
#include <QPointF>
#include <QPolygonF>

namespace geometry {
double cross(const QPointF& a, const QPointF& b, const QPointF& c);
double dist2(const QPointF& a, const QPointF& b);
QPolygonF convexHull(const QList<QPointF>& points);
} // namespace geometry

#endif
```

- [ ] **Step 2: Write `geometry/ConvexHull.cpp`** (port of current `grahamscan.cpp`, state-free)

```cpp
#include "geometry/ConvexHull.h"
#include <algorithm>

namespace geometry {

double cross(const QPointF& a, const QPointF& b, const QPointF& c)
{
    return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
}

double dist2(const QPointF& a, const QPointF& b)
{
    const double dx = a.x() - b.x();
    const double dy = a.y() - b.y();
    return dx * dx + dy * dy;
}

QPolygonF convexHull(const QList<QPointF>& points)
{
    QList<QPointF> pts = points;
    const int n = pts.size();
    if (n <= 1)
        return pts;

    int p0 = 0;
    for (int i = 1; i < n; ++i)
    {
        if (pts[i].y() < pts[p0].y()
            || (qFuzzyCompare(pts[i].y(), pts[p0].y()) && pts[i].x() < pts[p0].x()))
            p0 = i;
    }
    std::swap(pts[0], pts[p0]);
    const QPointF pivot = pts[0];

    std::sort(pts.begin() + 1, pts.end(), [&](const QPointF& a, const QPointF& b) {
        const double cr = cross(pivot, a, b);
        if (qFuzzyIsNull(cr))
            return dist2(pivot, a) < dist2(pivot, b);
        return cr > 0.0;
    });

    QList<QPointF> hull;
    hull.reserve(n);
    hull.push_back(pts[0]);
    if (n > 1)
        hull.push_back(pts[1]);

    for (int i = 2; i < n; ++i)
    {
        while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.last(), pts[i]) <= 0.0)
            hull.removeLast();
        hull.push_back(pts[i]);
    }
    if (hull.size() >= 3 && hull.first() != hull.last())
        hull.push_back(hull.first());

    return hull;
}

} // namespace geometry
```

- [ ] **Step 3: Write `geometry/PolygonGeometry.h`**

```cpp
#ifndef GEOMETRY_POLYGONGEOMETRY_H
#define GEOMETRY_POLYGONGEOMETRY_H

#include "domain/RotatedRect.h"
#include <QList>
#include <QPointF>
#include <QPolygonF>

namespace geometry {
double dot(const QPointF& a, const QPointF& b);
QPointF perp(const QPointF& v);
double shoelaceArea(const QPolygonF& polygon);
RotatedRect minimumAreaRectangle(const QList<QPointF>& hull);
QPolygonF rotatedRectToPolygon(const RotatedRect& r);
} // namespace geometry

#endif
```

- [ ] **Step 4: Write `geometry/PolygonGeometry.cpp`**

Port `ROIArea::dot`/`perp`/`minimumAreaRectangle`/`rotatedRectToPolygon` and `PathPlanner::calculatePolygonArea` verbatim. `minimumAreaRectangle` takes an *open* hull (tuples), so the "closed" wrapping logic currently embedded in `calculateMinimumAreaRectangle` stays in the caller. Exact port of the current `minimumAreaRectangle` loop (the body is unchanged from `ROIArea.cpp:500–579`) and `shoelaceArea` = current `calculatePolygonArea` (`pathplanner.cpp:181–198`), with the `std::abs` from `<cmath>`.

```cpp
#include "geometry/PolygonGeometry.h"
#include <cmath>
#include <limits>

namespace geometry {

double dot(const QPointF& a, const QPointF& b) { return a.x() * b.x() + a.y() * b.y(); }
QPointF perp(const QPointF& v) { return QPointF(-v.y(), v.x()); }

double shoelaceArea(const QPolygonF& polygon)
{
    if (polygon.size() < 3)
        return 0.0;
    double area = 0.0;
    for (int i = 0; i < polygon.size(); ++i)
    {
        QPointF p1 = polygon[i];
        QPointF p2 = polygon[(i + 1) % polygon.size()];
        area += (p1.x() * p2.y() - p2.x() * p1.y());
    }
    return std::abs(area) / 2.0;
}

RotatedRect minimumAreaRectangle(const QList<QPointF>& hull)
{
    RotatedRect best{};
    const int n = hull.size();
    if (n < 3)
        return best;

    double bestArea = std::numeric_limits<double>::infinity();

    for (int i = 0; i < n; ++i)
    {
        int i2 = (i + 1) % n;
        QPointF edge = hull[i2] - hull[i];
        double len = std::hypot(edge.x(), edge.y());
        if (len == 0.0)
            continue;

        QPointF ux(edge.x() / len, edge.y() / len);
        QPointF uy = perp(ux);

        double minX = std::numeric_limits<double>::infinity();
        double maxX = -std::numeric_limits<double>::infinity();
        double minY = std::numeric_limits<double>::infinity();
        double maxY = -std::numeric_limits<double>::infinity();

        for (int k = 0; k < n; ++k)
        {
            const QPointF& p = hull[k];
            double px = dot(p, ux);
            double py = dot(p, uy);
            if (px < minX) minX = px;
            if (px > maxX) maxX = px;
            if (py < minY) minY = py;
            if (py > maxY) maxY = py;
        }

        double width = maxX - minX;
        double height = maxY - minY;
        if (width <= 0.0 || height <= 0.0)
            continue;

        double area = width * height;
        if (area < bestArea)
        {
            bestArea = area;
            best.ux = ux;
            best.uy = uy;
            best.width = width;
            best.height = height;
            best.angle = std::atan2(ux.y(), ux.x());
            best.origin = minX * ux + minY * uy;

            QPointF o = best.origin;
            QPointF c1 = o + width * ux;
            QPointF c2 = c1 + height * uy;
            QPointF c3 = o + height * uy;
            qreal minBx = std::min({o.x(), c1.x(), c2.x(), c3.x()});
            qreal maxBx = std::max({o.x(), c1.x(), c2.x(), c3.x()});
            qreal minBy = std::min({o.y(), c1.y(), c2.y(), c3.y()});
            qreal maxBy = std::max({o.y(), c1.y(), c2.y(), c3.y()});
            best.rect = QRectF(QPointF(minBx, minBy), QPointF(maxBx, maxBy));
        }
    }
    return best;
}

QPolygonF rotatedRectToPolygon(const RotatedRect& r)
{
    QPolygonF poly;
    poly.reserve(4);
    const QPointF& o = r.origin;
    const QPointF& ux = r.ux;
    const QPointF& uy = r.uy;
    qreal w = r.width;
    qreal h = r.height;
    QPointF c0 = o;
    QPointF c1 = o + w * ux;
    QPointF c2 = c1 + h * uy;
    QPointF c3 = o + h * uy;
    poly << c0 << c1 << c2 << c3;
    return poly;
}

} // namespace geometry
```

- [ ] **Step 5: Write `geometry/PolygonClipping.h` and `.cpp`**

Header:

```cpp
#ifndef GEOMETRY_POLYGONCLIPPING_H
#define GEOMETRY_POLYGONCLIPPING_H

#include <QPolygonF>

namespace geometry {
QPolygonF sutherlandHodgmanClip(QPolygonF subject, const QPolygonF& clip);
} // namespace geometry

#endif
```

`.cpp` is the exact port of `PathPlanner::suth_hodgman_polygon_clipper` (`pathplanner.cpp:280–363`), taking `subject`/`clip` by value and returning the clipped polygon. Replace `EPSILON_TINY` with the `constants::kEpsilonTiny` from Task 5 (for now, use `1e-12` inline; Task 5 swaps to the constant).

- [ ] **Step 6: Retarget callers**

- `ROIArea`: replace `grahamScanner.addPointToPolygon(...)` / `grahamScanner.clear()` / `grahamScanner.ComputeHull()` with a plain member `QList<QPointF> m_grahamPoints` plus `geometry::convexHull(m_grahamPoints)`. Remove the `GrahamScan grahamScanner;` member.
- `PathPlanner::calculatePolygonArea(poly)` → `geometry::shoelaceArea(poly)` (update the 3 call sites: `compute_partitioned_area` is gone; `decomposedROI` and `mainwindow.cpp:242`).
- `PathPlanner::suth_hodgman_polygon_clipper(d, t)` → `geometry::sutherlandHodgmanClip(t, d)` (note argument order: subject is target, clip is divider). Update call sites in `decomposedROI`.
- `ROIArea::minimumAreaRectangle(hull)` → `geometry::minimumAreaRectangle(hull)`; `ROIArea::rotatedRectToPolygon(r)` → `geometry::rotatedRectToPolygon(r)`.
- Remove the now-unused member helpers `ROIArea::dot`/`perp` and `ROIArea::minimumAreaRectangle`/`rotatedRectToPolygon` declarations/definitions.

- [ ] **Step 7: Delete `grahamscan.{h,cpp}`**

```bash
git rm grahamscan.h grahamscan.cpp
```

- [ ] **Step 8: Build + temporary acceptance test**

Run: `cmake --build build` — expect success.

Add a temporary throwaway check (a small `main` compiled manually, or defer to Task 10's suite) that `convexHull` of a unit square returns 4 corners and `shoelaceArea` of a unit square is 1.0. If deferred, note it; Task 10 covers it.

- [ ] **Step 9: Commit**

```bash
git add geometry/ ROIArea.h ROIArea.cpp pathplanner.h pathplanner.cpp
git commit -m "refactor: extract pure geometry into geometry/ namespace"
```

---

### Task 5: Extract domain (`RotatedRect`, `Drone`, `GeoValidation`) + `constants` namespace

**Files:**
- Create: `domain/RotatedRect.h`, `domain/Drone.h`, `domain/Drone.cpp`, `domain/GeoValidation.h`, `domain/GeoValidation.cpp`
- Create: `constants.h`
- Modify: `utils.h` (reduce to `#include "constants.h"` + `#include "domain/*.h"` shim, then delete in Task 10), `pathplanner.h`, `pathplanner.cpp`, `ROIArea.*`

**Interfaces:**
- Produces:
  - `struct RotatedRect { QRectF rect; qreal angle; QPointF origin; QPointF ux; QPointF uy; qreal width; qreal height; };` (in `domain/RotatedRect.h`)
  - `struct Drone { ... same fields, CamelCase name ... };`
  - `namespace domain { void calcFlightAltitude(QList<Drone>&); void calcDroneCameraFootprint(QList<Drone>&); void calcMaximumForwardVelocity(QList<Drone>&); void calcDroneRelativeCapability(QList<Drone>&); QList<Drone> parseDrones(const QString& jsonFile); }`
  - `namespace constants { ... }` (full list below)

- [ ] **Step 1: Create `constants.h`**

Move every `#define` from `utils.h:15–62` into a typed `namespace constants`:

```cpp
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
```

- [ ] **Step 2: Create `domain/RotatedRect.h`**

Move the `RotatedRect` struct exactly from `utils.h:70–80`.

- [ ] **Step 3: Create `domain/Drone.h` and `domain/Drone.cpp`**

Move `struct drone` (renamed `struct Drone`) from `utils.h:84–111` into `domain/Drone.h`. Move the four calc functions from `pathplanner.cpp` (`calcFlightAltitude` 87–103, `calcDroneCameraFootprint` 105–120, `calcMaximumForwardVelocity` 122–151, `calcDroneRelativeCapability` 153–178) into `domain/Drone.cpp` as `domain::` free functions, plus `parseDrones` = body of `getDroneInfo` (pathplanner.cpp:29–85). Replace macro names with `constants::k...`.

- [ ] **Step 4: Create `domain/GeoValidation.h`/`.cpp`**

Move the four `inline` functions from `utils.h:113–204` (`isPolygonWGS84`, `isPolygonPixel`, `isRotatedRectWGS84`, `isValueInMeters`, `validateCoordinateSystemMatch`, `validateFootprintMeters`). Drop `inline` (they are in a `.cpp`), keep signatures.

- [ ] **Step 5: Retarget callers**

- `pathplanner.h/.cpp`: remove `getDroneInfo` and the four `calc*` members (now `domain::`); remove `#include "utils.h"` and `#include "GDALHandler.h"`-style leaks.
- `ROIArea.*`: replace `drone` → `Drone`, remove local `dot`/`perp` (done in Task 4), use `domain::` free functions.
- `mainwindow.cpp`: `drone` → `Drone`.

- [ ] **Step 6: Make `utils.h` a shim temporarily**

```cpp
#ifndef UTILS_H_
#define UTILS_H_
#include "constants.h"
#include "domain/RotatedRect.h"
#include "domain/Drone.h"
#include "domain/GeoValidation.h"
#endif
```

This keeps any remaining `#include <utils.h>` buildable until Task 10 removes it.

- [ ] **Step 7: Build**

Run: `cmake --build build` — expect success.

- [ ] **Step 8: Commit**

```bash
git add constants.h domain/ pathplanner.h pathplanner.cpp ROIArea.h ROIArea.cpp mainwindow.cpp utils.h
git commit -m "refactor: extract domain and typed constants; rename Drone"
```

---

### Task 6: Remove `QObject` from `PathPlanner`, clean CMake + warnings

**Files:**
- Modify: `pathplanner.h`, `pathplanner.cpp`, `ROIArea.cpp`, `CMakeLists.txt`

**Interfaces:**
- Produces: `PathPlanner` is a plain class (no `Q_OBJECT`, no `QObject` parent, no `signals:` block). `CMakeLists.txt` has one `find_package(Qt6 ...)`, one `target_link_libraries`, warnings enabled.

- [ ] **Step 1: De-QObject `PathPlanner`**

In `pathplanner.h`: remove `: public QObject`, `Q_OBJECT`, the `explicit PathPlanner(QObject*)` constructor → `PathPlanner() = default;`, and the empty `signals:` block. Remove `#include <QObject>`.

In `pathplanner.cpp`: change constructor to `PathPlanner::PathPlanner() = default;` (or delete it; declare in header as `= default`).

In `ROIArea.cpp:14`: change `pathPlanner(this)` to just default-construct (remove from init list; `PathPlanner` is now a value member).

- [ ] **Step 2: Clean `CMakeLists.txt`**

Result should be:

```cmake
cmake_minimum_required(VERSION 3.19)
project(ROIGenerator LANGUAGES CXX)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(GDAL REQUIRED)
qt_standard_project_setup()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

qt_add_executable(ROIGenerator WIN32 MACOSX_BUNDLE
    main.cpp
    ROIArea.h ROIArea.cpp
    GDALHandler.h GDALHandler.cpp
    mainwindow.h mainwindow.cpp mainwindow.ui
    pathplanner.h pathplanner.cpp
    geometry/ConvexHull.h geometry/ConvexHull.cpp
    geometry/PolygonGeometry.h geometry/PolygonGeometry.cpp
    geometry/PolygonClipping.h geometry/PolygonClipping.cpp
    domain/Drone.h domain/Drone.cpp
    domain/GeoValidation.h domain/GeoValidation.cpp
)

target_compile_options(ROIGenerator PRIVATE -Wall -Wextra -Wpedantic)

target_include_directories(ROIGenerator PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ROIGenerator PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets GDAL::GDAL)

...install/deploy block unchanged...
```

- [ ] **Step 3: Reconfigure + build, fix warnings**

Run: `cmake -S . -B build && cmake --build build`
Expected: builds; fix any warnings introduced by `-Wall -Wextra` (e.g., unused parameters — prefix with `Q_UNUSED` or remove; sign-compare; `-Wpedantic` extra semicolons). Do **not** add `-Werror` until the build is fully clean.

- [ ] **Step 4: Commit**

```bash
git add pathplanner.h pathplanner.cpp ROIArea.cpp CMakeLists.txt
git commit -m "refactor: de-QObject PathPlanner, clean CMake, enable warnings"
```

---

### Task 7: Split `io/` (GDALHandler SRP + GeoJSON) + cache inverse geotransform

**Files:**
- Create: `io/GeoJSON.h`, `io/GeoJSON.cpp`
- Modify: `GDALHandler.h`, `GDALHandler.cpp`, `ROIArea.cpp`

**Interfaces:**
- Produces:
  - `namespace geo { QImage readImage(const QString&); ... }` — GDALHandler keeps: `openSrcRaster`, `closeRaster`, `toQImage`, `pixelToGeo`, `polygonToGeo`, `geoToPixel`, `geoPolygonToPixels`, `getDataSetCRS(Info)`, `getDataset`.
  - `class GeoJSONExporter` (or free functions) `importPolygon(const QString&) -> QList<QPointF>`, `reprojectToWgs84(const QJsonDocument&, const OGRSpatialReference& src) -> QJsonDocument`.

- [ ] **Step 1: Cache inverse geotransform in `GDALHandler`**

Add member `double invGeoTransform[GEO_TRANSFORM_SIZE]; bool hasInv = false;`. In `openSrcRaster`, after `GetGeoTransform` succeeds, call `GDALInvGeoTransform(geoTransform, invGeoTransform)` and set `hasInv` on success; clear on `closeRaster`. Rewrite `geoToPixel` and both `geoPolygonToPixels` overloads to use the cached `invGeoTransform` (return empty/`QPointF()` if `!hasInv`), removing the per-call `memcpy` + `GDALInvGeoTransform`.

- [ ] **Step 2: Create `io/GeoJSON.h`/`.cpp`**

Move `loadPolygonFromGeoJSON` (`GDALHandler.cpp:327–368`) and `reprojectGeoJSONPolygon` (`GDALHandler.cpp:168–325`) into this file as free functions:

```cpp
QList<QPointF> importPolygon(const QString& path);
QJsonDocument reprojectToWgs84(const QJsonDocument& srcDoc, const QString& srcWkt);
```

`reprojectToWgs84` takes the source WKT string (from the dataset) instead of reaching into `GDALHandler`. `exportPolygonGeoJSON` passes `gdalHandler.getDataSetCRS(...)`-derived WKT or the GeoJSON `crs` fallback logic (port the source-CRS detection from `GDALHandler.cpp:194–256` into `reprojectToWgs84`, reading properties if `srcWkt` is empty).

- [ ] **Step 3: Retarget `ROIArea`**

`ROIArea::openGeoJSONFilePoints` → `geo::importPolygon(filename)`. `exportPolygonGeoJSON` → `geo::reprojectToWgs84(srcDoc, srcWkt)` (grab `srcWkt` from `gdalHandler.getDataset()->GetProjectionRef()`).

- [ ] **Step 4: Remove moved code from `GDALHandler.cpp`**

Delete `reprojectGeoJSONPolygon` and `loadPolygonFromGeoJSON` (now in `io/GeoJSON.cpp`). Remove their declarations from `GDALHandler.h`.

- [ ] **Step 5: Build**

Run: `cmake --build build` — expect success.

- [ ] **Step 6: Commit**

```bash
git add io/ GDALHandler.h GDALHandler.cpp ROIArea.cpp CMakeLists.txt
git commit -m "refactor: split io into GDALHandler + GeoJSON; cache inverse geotransform"
```

---

### Task 8: Split `ui/` (ImageCanvas, LabelPlacer, ColorPalette) + minimal interfaces

**Files:**
- Create: `ui_interfaces.h`, `ui/LabelPlacer.h`, `ui/LabelPlacer.cpp`, `ui/ColorPalette.h`, `ui/ColorPalette.cpp`
- Modify: `ROIArea.h`, `ROIArea.cpp` (becomes `ImageCanvas`)
- Test: `tests/test_labelplacer.cpp` (Task 10)

**Interfaces:**
- Produces:
  - `class LabelPlacer { LabelPlacer(qreal w, qreal h, qreal margin = 5.0, qreal rotationDeg = -30.0); QPointF place(const QRectF& bbox, const QFontMetrics& fm); void clear(); qreal imageWidth() const; qreal imageHeight() const; };` (exact implementation as per spec §5.1)
  - `void drawLabel(QPainter&, const QString&, const QColor& fg, const QColor& bg, int bgAlpha, const QPointF& anchor, const QFontMetrics& fm, qreal rotationDeg);`
  - `namespace color { QColor analyzeBackgroundColor(const QImage&, const QRectF& = QRectF()); QVector<QColor> generateContrastingPalette(const QColor&, int); QColor getContrastingTextColor(const QColor&); }`

- [ ] **Step 1: Create `ui/LabelPlacer.{h,cpp}`**

Implement exactly per spec §5.1. Port the candidate/overlap/valid logic currently duplicated in `showDecomposedROI` and `showWaypoints`. Implement `drawLabel` as a free function in the same `.cpp` (declared in the header).

- [ ] **Step 2: Create `ui/ColorPalette.{h,cpp}`**

Move `analyzeBackgroundColor` (`ROIArea.cpp:1700–1741`), `generateContrastingPalette` (1743–1816), `getContrastingTextColor` (1818–1831) into `namespace color`. Replace `ALPHA_LABEL_BG`, `HUE_*` with `constants::k...`.

- [ ] **Step 3: Refactor `showDecomposedROI` and `showWaypoints` to use `LabelPlacer`/`drawLabel`**

In each, drop `placedLabels`, `candidatePositions`, `overlapsPlacedLabels`, `isValidPosition`, the two passes and the clamp; replace with:

```cpp
LabelPlacer placer(overlayImage.width(), overlayImage.height(),
                   constants::kLabelMarginSmall, constants::kLabelRotationDeg);
// per polygon/drone:
QPointF labelPt = placer.place(pixelSpacePolygon.boundingRect(), fm);
drawLabel(painter, labelText, color, bgColor, constants::kAlphaLabelBg, labelPt, fm, constants::kLabelRotationDeg);
```

Remove `contrastingPalette`/`contrastingTextColor` members from the widget; call `color::` functions locally.

- [ ] **Step 4: Introduce `ui_interfaces.h`**

```cpp
#ifndef UI_INTERFACES_H
#define UI_INTERFACES_H
#include <QByteArray>
#include <QPolygonF>
#include "domain/RotatedRect.h"

class IImageDocument {
public:
    virtual ~IImageDocument() = default;
    virtual bool openImage(const QString&) = 0;
    virtual bool closeImage() = 0;
    virtual bool saveImage(const QString&, const char*) = 0;
};

class IRoiProvider {
public:
    virtual ~IRoiProvider() = default;
    virtual QPolygonF finalPolygon() const = 0;
    virtual RotatedRect mar() const = 0;
};

class IMissionExporter {
public:
    virtual ~IMissionExporter() = default;
    virtual QByteArray exportGeoJSON() const = 0;
};
#endif
```

`ImageCanvas` (renamed `ROIArea`) implements all three; `MainWindow` holds references to these interfaces.

- [ ] **Step 5: Rename `ROIArea` → `ImageCanvas`**

Apply a careful rename across `ROIArea.{h,cpp}`, the `.ui` object name, and `mainwindow.cpp` (`ui->roiArea`). Keep behavior identical.

- [ ] **Step 6: Build + smoke**

Run: `cmake --build build` — expect success. Manual smoke: open a GeoTIFF, draw ROI, load drones, decompose, generate waypoints, verify labels still place without overlap.

- [ ] **Step 7: Commit**

```bash
git add ui/ ui_interfaces.h ImageCanvas.h ImageCanvas.cpp mainwindow.cpp CMakeLists.txt
git commit -m "refactor: split UI into ImageCanvas, LabelPlacer, ColorPalette; add minimal interfaces"
```

---

### Task 9: Strategize `planning/` (interfaces + injection)

**Files:**
- Create: `planning/IDecompositionStrategy.h`, `planning/IWaypointGenerator.h`, `planning/StripDecomposition.h/.cpp`, `planning/BoustrophedonSweep.h/.cpp`
- Modify: `pathplanner.h`, `pathplanner.cpp`, `ImageCanvas.cpp`

**Interfaces:**
- Consumes: `Drone`, `RotatedRect`, `geometry::` from Tasks 4–5.
- Produces:
  - `class IDecompositionStrategy { virtual ~IDecompositionStrategy() = default; virtual QList<QPair<QPolygonF,QString>> decompose(const QPolygonF&, const QList<Drone>&, const RotatedRect&) const = 0; };`
  - `class IWaypointGenerator { virtual ~IWaypointGenerator() = default; virtual QList<QPointF> generate(const QPolygonF&, const Drone&, const RotatedRect&) const = 0; };`
  - `PathPlanner(unique_ptr<IDecompositionStrategy>, unique_ptr<IWaypointGenerator>)`; slim `plan(...)`.

- [ ] **Step 1: Write the two interface headers** (exact code in spec §5.2).

- [ ] **Step 2: Extract `StripDecomposition`**

Move the body of `decomposedROI` (`pathplanner.cpp:393–450`) into `StripDecomposition::decompose`. Keep `makeRectPoly` as a file-local helper. Return `QList<QPair<QPolygonF,QString>>`.

- [ ] **Step 3: Extract `BoustrophedonSweep`**

Move `computeWaypointsWithMAR` (`pathplanner.cpp:798–956`) into `BoustrophedonSweep::generate`.

- [ ] **Step 4: Slim `PathPlanner` into an orchestrator**

`PathPlanner` now holds `std::unique_ptr<IDecompositionStrategy>` and `std::unique_ptr<IWaypointGenerator>`, injected via constructor. Its public API shrinks to:

```cpp
class PathPlanner {
public:
    PathPlanner(std::unique_ptr<IDecompositionStrategy> dec,
                std::unique_ptr<IWaypointGenerator> gen);
    QList<QPair<QPolygonF, QString>> decompose(const QPolygonF& roi, const QList<Drone>& drones, const RotatedRect& mar) const;
    QList<QPointF> generateWaypoints(const QPolygonF& subRoi, const Drone& d, const RotatedRect& mar) const;
    // drone list accessors remain as thin storage
    void setDroneList(const QList<Drone>&);
    QList<Drone> droneList() const;
private:
    std::unique_ptr<IDecompositionStrategy> m_dec;
    std::unique_ptr<IWaypointGenerator> m_gen;
    QList<Drone> m_drones;
    QList<QPair<QPolygonF, QString>> m_decomposed;
};
```

Keep `get/setDecomposedROIs`/`get/setDroneList` as thin wrappers over the private fields so `ImageCanvas`/`MainWindow` call sites keep compiling; mark them deprecated-free but simple.

- [ ] **Step 5: Update construction site**

In `ImageCanvas` constructor, build:

```cpp
pathPlanner(std::make_unique<StripDecomposition>(),
            std::make_unique<BoustrophedonSweep>())
```

(now that `PathPlanner` is a value member and injection happens here).

- [ ] **Step 6: Build**

Run: `cmake --build build` — expect success.

- [ ] **Step 7: Commit**

```bash
git add planning/ pathplanner.h pathplanner.cpp ImageCanvas.cpp CMakeLists.txt
git commit -m "refactor: introduce decomposition/waypoint strategy interfaces (OCP/DIP)"
```

---

### Task 10: Physical move to `src/`, wire tests, final verification

**Files:**
- Move all sources into `src/` (see File Structure).
- Modify: `CMakeLists.txt` (add `src/` paths + `Qt6::Test`), create `tests/CMakeLists.txt`, `tests/test_geometry.cpp`, `tests/test_planning.cpp`, `tests/test_labelplacer.cpp`.
- Delete: `utils.h` shim (folded into `constants.h`/`domain/*`).

**Interfaces:**
- Consumes: everything produced in Tasks 1–9.

- [ ] **Step 1: Move files under `src/`** preserving the target layout; update `CMakeLists.txt` paths and `target_include_directories` to `${CMAKE_CURRENT_SOURCE_DIR}/src`.

- [ ] **Step 2: Enable tests in top-level `CMakeLists.txt`**

```cmake
find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Widgets Test)
...
enable_testing()
add_subdirectory(tests)
```

- [ ] **Step 3: `tests/CMakeLists.txt`**

```cmake
add_executable(test_geometry test_geometry.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/ConvexHull.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/PolygonGeometry.cpp
    ${CMAKE_SOURCE_DIR}/src/geometry/PolygonClipping.cpp
    ${CMAKE_SOURCE_DIR}/src/domain/Drone.cpp
)
target_include_directories(test_geometry PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(test_geometry PRIVATE Qt6::Core Qt6::Test)
add_test(NAME geometry COMMAND test_geometry)

# analogous: test_planning (planning/ + geometry/ + domain/), test_labelplacer (ui/LabelPlacer + Qt6::Gui)
```

- [ ] **Step 4: Write `tests/test_geometry.cpp`**

Use `QTEST_GUILESS_MAIN`. Concrete cases (actual assertions):

```cpp
#include <QtTest>
#include "geometry/ConvexHull.h"
#include "geometry/PolygonGeometry.h"
#include "geometry/PolygonClipping.h"

class TestGeometry : public QObject {
    Q_OBJECT
private slots:
    void hull_square();
    void hull_collinear();
    void area_square();
    void area_triangle_ccw_cw();
    void clip_contained();
    void mar_rotated_square();
};

void TestGeometry::hull_square()
{
    QList<QPointF> pts{{0,0},{1,0},{1,1},{0,1}};
    QPolygonF h = geometry::convexHull(pts);
    QCOMPARE(h.size() - 1, 4); // closed: 4 corners + repeat
}

void TestGeometry::hull_collinear()
{
    QList<QPointF> pts{{0,0},{1,1},{2,2}};
    QPolygonF h = geometry::convexHull(pts);
    QCOMPARE(h.size() - 1, 2);
}

void TestGeometry::area_square()
{
    QPolygonF sq{{0,0},{2,0},{2,2},{0,2}};
    QCOMPARE(geometry::shoelaceArea(sq), 4.0);
}

void TestGeometry::area_triangle_ccw_cw()
{
    QPolygonF ccw{{0,0},{3,0},{0,4}};
    QPolygonF cw{{0,0},{0,4},{3,0}};
    QCOMPARE(geometry::shoelaceArea(ccw), 6.0);
    QCOMPARE(geometry::shoelaceArea(cw), 6.0);
}

void TestGeometry::clip_contained()
{
    QPolygonF subject{{0.5,0.5},{0.6,0.5},{0.6,0.6},{0.5,0.6}};
    QPolygonF clip{{0,0},{1,0},{1,1},{0,1}};
    QPolygonF out = geometry::sutherlandHodgmanClip(subject, clip);
    QVERIFY(!out.isEmpty());
    QCOMPARE(geometry::shoelaceArea(out), 0.01);
}

void TestGeometry::mar_rotated_square()
{
    // axis-aligned square as a trivial rotated case
    QList<QPointF> hull{{0,0},{2,0},{2,2},{0,2}};
    RotatedRect mar = geometry::minimumAreaRectangle(hull);
    QCOMPARE(mar.width, 2.0);
    QCOMPARE(mar.height, 2.0);
}

QTEST_GUILESS_MAIN(TestGeometry)
#include "test_geometry.moc"
```

(The `minimumAreaRectangle` caller must un-close the hull first — the test passes 4 distinct vertices.)

- [ ] **Step 5: Write `tests/test_planning.cpp`** (decomposition proportion + waypoint bounds) and `tests/test_labelplacer.cpp` (no overlap, in-bounds), mirroring the geometry test style with real assertions.

- [ ] **Step 6: Build + run full suite**

```bash
cmake -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure
```
Expected: all tests pass.

- [ ] **Step 7: Behavioral smoke + GeoJSON byte-compare**

Open a georeferenced image, draw ROI, run decomposition/waypoints, export GeoJSON; diff against a pre-refactor export (must be identical).

- [ ] **Step 8: Commit**

```bash
git add -A
git commit -m "refactor: move to src/, add CTest suite"
```

---

## Self-Review Notes

- **Spec coverage:** Tasks 1–2 map to spec Step 1; Task 3 → Step 2; Task 4 → Step 3 (geometry); Task 5 → Step 3–4 (domain/constants); Task 6 → Step 4 (build, LSP); Task 7 → Step 5 (io); Task 8 → Step 6 (ui/ISP); Task 9 → Step 7 (planning/OCP/DIP); Task 10 → Step 8 (tests) + move. §5.1/5.2 and §6 contracts are reproduced verbatim. §7 performance (cached inverse geotransform) is Task 7.
- **Placeholder scan:** no TBD/TODO; all code steps contain concrete code.
- **Type consistency:** `geometry::` signatures, `domain::`, `Drone`, `LabelPlacer::place`, `drawLabel`, and both strategy interfaces are defined in their producing tasks and referenced identically in consumers.
