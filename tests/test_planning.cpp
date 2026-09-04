#include <QtTest>
#include "planning/StripDecomposition.h"
#include "planning/BoustrophedonSweep.h"
#include "geometry/PolygonGeometry.h"
#include "domain/Drone.h"

class TestPlanning : public QObject {
    Q_OBJECT
private slots:
    void decomposition_equal_drones();
    void waypoints_inside_subroi();
};

void TestPlanning::decomposition_equal_drones()
{
    // Axis-aligned 100x100 square ROI with its own minimum-area rectangle.
    QPolygonF roi{{0,0},{100,0},{100,100},{0,100}};
    QList<QPointF> hull{{0,0},{100,0},{100,100},{0,100}};
    RotatedRect mar = geometry::minimumAreaRectangle(hull);

    Drone d1;
    d1.id = 1;
    d1.name = "Drone A";
    d1.relative_capability_score = 1.0;

    Drone d2;
    d2.id = 2;
    d2.name = "Drone B";
    d2.relative_capability_score = 1.0;

    QList<Drone> drones{d1, d2};

    StripDecomposition dec;
    auto result = dec.decompose(roi, drones, mar);

    QCOMPARE(result.size(), 2);

    const double totalArea = geometry::shoelaceArea(roi);
    double sum = 0.0;
    for (const auto& p : result)
        sum += geometry::shoelaceArea(p.first);

    QVERIFY(qAbs(sum - totalArea) < 1e-4);

    const double a0 = geometry::shoelaceArea(result[0].first);
    const double a1 = geometry::shoelaceArea(result[1].first);
    // Two equal-capability drones split the ROI roughly in half.
    QVERIFY(qAbs(a0 - totalArea / 2.0) < 1e-3);
    QVERIFY(qAbs(a1 - totalArea / 2.0) < 1e-3);
    QVERIFY(qAbs(a0 - a1) < 1e-3);
}

void TestPlanning::waypoints_inside_subroi()
{
    // A simple square in WGS84-like coordinates (magnitudes > 1e4) so the
    // coordinate-system / meter validation in BoustrophedonSweep passes.
    QPolygonF area{{20000,20000},{20100,20000},{20100,20100},{20000,20100}};
    QList<QPointF> hull{{20000,20000},{20100,20000},{20100,20100},{20000,20100}};
    RotatedRect mar = geometry::minimumAreaRectangle(hull);

    Drone d;
    d.id = 7;
    d.name = "Drone 7";
    d.max_x_footprint = 10.0;
    d.max_y_footprint = 10.0;

    BoustrophedonSweep sweep;
    QList<QPointF> waypoints = sweep.generate(area, d, mar);

    QVERIFY(!waypoints.isEmpty());
    // Every generated waypoint must lie inside the sub-ROI.
    for (const QPointF& wp : waypoints)
        QVERIFY(area.containsPoint(wp, Qt::OddEvenFill));
}

QTEST_GUILESS_MAIN(TestPlanning)
#include "test_planning.moc"
