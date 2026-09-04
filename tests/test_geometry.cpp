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
    // Degenerate hull of collinear points collapses to the two extreme
    // endpoints; size() is 2 (the repeat of the first vertex is only added
    // when the hull has 3+ distinct vertices, so no closing point here).
    QCOMPARE(h.size(), 2);
}

void TestGeometry::area_square()
{
    QPolygonF sq{{0,0},{2,0},{2,2},{0,2}};
    QVERIFY(qAbs(geometry::shoelaceArea(sq) - 4.0) < 1e-6);
}

void TestGeometry::area_triangle_ccw_cw()
{
    QPolygonF ccw{{0,0},{3,0},{0,4}};
    QPolygonF cw{{0,0},{0,4},{3,0}};
    QVERIFY(qAbs(geometry::shoelaceArea(ccw) - 6.0) < 1e-6);
    QVERIFY(qAbs(geometry::shoelaceArea(cw) - 6.0) < 1e-6);
}

void TestGeometry::clip_contained()
{
    QPolygonF subject{{0.5,0.5},{0.6,0.5},{0.6,0.6},{0.5,0.6}};
    QPolygonF clip{{0,0},{1,0},{1,1},{0,1}};
    QPolygonF out = geometry::sutherlandHodgmanClip(subject, clip);
    QVERIFY(!out.isEmpty());
    QVERIFY(qAbs(geometry::shoelaceArea(out) - 0.01) < 1e-6);
}

void TestGeometry::mar_rotated_square()
{
    // axis-aligned square as a trivial rotated case
    QList<QPointF> hull{{0,0},{2,0},{2,2},{0,2}};
    RotatedRect mar = geometry::minimumAreaRectangle(hull);
    QVERIFY(qAbs(mar.width - 2.0) < 1e-6);
    QVERIFY(qAbs(mar.height - 2.0) < 1e-6);
}

QTEST_GUILESS_MAIN(TestGeometry)
#include "test_geometry.moc"
