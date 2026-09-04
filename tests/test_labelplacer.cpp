#include <QtTest>
#include <QFont>
#include <QFontMetrics>
#include <QGuiApplication>
#include <QtMath>
#include <cmath>
#include "ui/LabelPlacer.h"

// Reconstruct the rotated axis-aligned footprint the same way LabelPlacer does.
static QRectF
rotatedFootprint(const QPointF& anchor, qreal textWidth, qreal textHeight, qreal rotationDeg)
{
    const double theta = qDegreesToRadians(rotationDeg);
    const double c = std::cos(theta);
    const double s = std::sin(theta);

    const double halfW = (std::fabs(textWidth * c) + std::fabs(textHeight * s)) / 2.0;
    const double halfH = (std::fabs(textWidth * s) + std::fabs(textHeight * c)) / 2.0;

    const double centerX = (textWidth / 2.0) * c - (-textHeight / 2.0) * s;
    const double centerY = (textWidth / 2.0) * s + (-textHeight / 2.0) * c;
    const QPointF center(anchor.x() + centerX, anchor.y() + centerY);

    return QRectF(center.x() - halfW, center.y() - halfH, 2.0 * halfW, 2.0 * halfH);
}

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

    const qreal tw = fm.horizontalAdvance(text);
    const qreal th = fm.height();

    // Reconstruct the recorded footprint the same way LabelPlacer does.
    QRectF r1 = rotatedFootprint(p1, tw, th, -30.0);
    QRectF r2 = rotatedFootprint(p2, tw, th, -30.0);

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
