#ifndef GRAHAMSCAN_H
#define GRAHAMSCAN_H

#include <QList>
#include <QObject>
#include <QPointF>
#include <QPolygonF>
class GrahamScan : public QObject
{
    Q_OBJECT
public:
    explicit GrahamScan(QObject *parent = nullptr);
    // Cross product (b - a) x (c - a)
    double cross(const QPointF &a, const QPointF &b, const QPointF &c);
    // Squared distance, for tie-breaking collinear points
    double dist2(const QPointF &a, const QPointF &b);
    QList<QPointF> grahamScan(QList<QPointF> pts);
    QPolygonF ComputeHull();
    void addPointToPolygon(QPointF);
    void clear();

private:
    QList<QPointF> polygon;
signals:
};

#endif // GRAHAMSCAN_H
