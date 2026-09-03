#include <QtTest>
#include <QFont>
#include <QFontMetrics>
#include <QGuiApplication>
#include "ui/LabelPlacer.h"

class TestLabelPlacer : public QObject {
    Q_OBJECT
private slots:
    void place_in_bounds();
    void place_no_overlap();
};

void TestLabelPlacer::place_in_bounds()
{
    LabelPlacer placer(1000.0, 800.0, 5.0, -30.0);
    QFont font("Sans", 10);
    QFontMetrics fm(font);

    QRectF bbox(100, 100, 200, 150);
    QPointF p = placer.place(bbox, "Drone 1 (123 pts)", fm);

    QVERIFY(p.x() >= 0.0 && p.x() <= 1000.0);
    QVERIFY(p.y() >= 0.0 && p.y() <= 800.0);
}

void TestLabelPlacer::place_no_overlap()
{
    LabelPlacer placer(1000.0, 800.0, 5.0, -30.0);
    QFont font("Sans", 10);
    QFontMetrics fm(font);

    const QString text = "Drone 1 (123 pts)";
    QRectF bbox(100, 100, 200, 150);

    QPointF p1 = placer.place(bbox, text, fm);
    QPointF p2 = placer.place(bbox, text, fm);

    const int tw = fm.horizontalAdvance(text);
    const int th = fm.height();

    // Reconstruct the recorded footprint the same way LabelPlacer does.
    QRectF r1(p1.x() - 2.0, p1.y() - th - 2.0, tw + 4.0, th + 4.0);
    QRectF r2(p2.x() - 2.0, p2.y() - th - 2.0, tw + 4.0, th + 4.0);

    // The second label must not land on the first one's footprint.
    QVERIFY(!r1.intersects(r2));
}

int
main(int argc, char** argv)
{
    // QFontMetrics (via QFontDatabase) requires a QGuiApplication instance.
    QGuiApplication app(argc, argv);
    TestLabelPlacer tc;
    QTEST_SET_MAIN_SOURCE_PATH
    return QTest::qExec(&tc, argc, argv);
}

#include "test_labelplacer.moc"
