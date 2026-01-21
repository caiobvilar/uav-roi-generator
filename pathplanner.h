#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <QObject>
#include <QPolygonF>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <cstdint>
#include <qarraydatapointer.h>
#include <qcontainerfwd.h>
#include <qlist.h>
#include <qpolygon.h>

struct drone
{
    uint32_t id;
    QString name;
    double battery_capacity = 0.0;
    double battery_current_capacity = 0.0;
    double max_horizontal_velocity = 0.0;
    double max_vertical_velocity = 0.0;
    double camera_focal_length = 0.0;
    double camera_array_width = 0.0;
    double camera_array_height = 0.0;
    double camera_image_width = 0.0;
    double camera_image_height = 0.0;
    double camera_shutter_speed = 0.0;
    double max_forward_velocity = 0.0;
    double max_x_footprint = 0.0;
    double max_y_footprint = 0.0;
    double ideal_flight_altitude = 0.0;
    double relative_capability_score = 0.0;
};

class PathPlanner : public QObject
{
    Q_OBJECT
public:
    explicit PathPlanner(QObject *parent = nullptr);
    QList<drone> getDroneInfo(const QString &filename);
    void calcDroneCameraFootprint(QList<drone>);
    void calcFlightAltitude(QList<drone>);
    void calcMaximumForwardVelocity(QList<drone>);
    void calcDroneRelativeCapability(QList<drone>);
    QList<QPolygonF> decomposedROI(QPolygonF &, QList<drone>&);
    double calculatePolygonArea(const QPolygonF &polygon);
private:
    QPolygonF RegionOfInterest;
    QList<drone> droneList;

signals:
};

#endif // PATHPLANNER_H
