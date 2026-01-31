#include "pathplanner.h"
#include "ROIArea.h"
#include <QList>
#include <QPolygonF>
#include <QVector>
#include <algorithm>

// Helper: Create a rectangle polygon in MAR coordinates
static QPolygonF
makeRectPoly(const RotatedRect& mar, double start, double end)
{
    QPointF o = mar.origin + start * mar.ux;
    QPointF ux = mar.ux;
    QPointF uy = mar.uy;
    double w = end - start;
    double h = mar.height;
    QPolygonF poly;
    poly << o << o + w * ux << o + w * ux + h * uy << o + h * uy << o; // closed
    return poly;
}

PathPlanner::PathPlanner(QObject* parent) : QObject{parent}
{
}

QList<drone>
PathPlanner::getDroneInfo(const QString& filename)
{
    QList<drone> results;

    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly))
    {
        qWarning() << "Failed to open file:" << filename;
        return results;
    }

    QByteArray data = file.readAll();
    file.close();

    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (!doc.isObject())
    {
        qWarning() << "Invalid JSON format";
        return results;
    }

    QJsonObject rootObj = doc.object();
    QJsonArray dronesArray = rootObj["drones"].toArray();

    for (int i = 0; i < dronesArray.size(); ++i)
    {
        QJsonObject droneObj = dronesArray[i].toObject();

        drone d;
        d.id = droneObj["id"].toString().split("_").last().toUInt();
        d.name = droneObj["name"].toString();

        // Battery info
        QJsonObject batteryObj = droneObj["battery"].toObject();
        d.battery_capacity = batteryObj["capacity"].toDouble();
        d.battery_current_capacity = batteryObj["current_capacity"].toString().toDouble();

        // Max velocity
        QJsonObject velocityObj = droneObj["max_velocity"].toObject();
        d.max_horizontal_velocity = velocityObj["horizontal"].toDouble();
        d.max_vertical_velocity = velocityObj["vertical"].toDouble();

        // Camera info
        QJsonObject cameraObj = droneObj["Camera"].toObject();
        d.camera_focal_length = cameraObj["focal_length"].toString().toDouble();
        d.camera_array_width = cameraObj["array_width"].toString().toDouble();
        d.camera_array_height = cameraObj["array_height"].toString().toDouble();
        d.camera_image_width = cameraObj["image_width"].toString().toDouble();
        d.camera_image_height = cameraObj["image_height"].toString().toDouble();
        d.camera_shutter_speed = cameraObj["shutter_speed"].toString().toDouble();

        results.append(d);
    }

    return results;
}

void
PathPlanner::calcFlightAltitude(QList<drone>& droneList)
{
    for (auto& d : droneList)
    {
        // GSD_x * f_l * i_x / l_x
        double h_x = (DESIRED_GSD * d.camera_focal_length * d.camera_image_width) / d.camera_array_width;

        // GSD_y * f_l * i_y / l_y
        double h_y = (DESIRED_GSD * d.camera_focal_length * d.camera_image_height) / d.camera_array_height;

        // h = min{h_x, h_y}
        d.ideal_flight_altitude = std::min(h_x, h_y);

        qDebug() << "Drone" << d.name << "- Ideal flight altitude:" << d.ideal_flight_altitude << "cm";
    }
}

void
PathPlanner::calcDroneCameraFootprint(QList<drone>& droneList)
{
    for (auto& d : droneList)
    {
        // L_x = h * l_x / f_l
        d.max_x_footprint = (d.ideal_flight_altitude * d.camera_array_width) / d.camera_focal_length;

        // L_y = h * l_y / f_l
        d.max_y_footprint = (d.ideal_flight_altitude * d.camera_array_height) / d.camera_focal_length;

        qDebug() << "Drone" << d.name << "- Camera footprint:";
        qDebug() << "  L_x:" << d.max_x_footprint << "cm";
        qDebug() << "  L_y:" << d.max_y_footprint << "cm";
    }
}

void
PathPlanner::calcMaximumForwardVelocity(QList<drone>& droneList)
{
    for (auto& d : droneList)
    {
        // V_max = L_y * (1 - O_f) / s_h
        d.max_forward_velocity = (d.max_y_footprint * (1.0 - FORWARD_OVERLAP)) / d.camera_shutter_speed;

        qDebug() << "Drone" << d.name << "- Maximum forward velocity:" << d.max_forward_velocity << "cm/s";
        qDebug() << "  (with" << (FORWARD_OVERLAP * 100) << "% forward overlap)";
    }
}

void
PathPlanner::calcDroneRelativeCapability(QList<drone>& droneList)
{
    // Calculate capability for each drone: c_i = V_max^i * L_x^i
    double totalCapability = 0.0;

    for (auto& d : droneList)
    {
        d.relative_capability_score = d.max_forward_velocity * d.max_x_footprint;
        totalCapability += d.relative_capability_score;
    }

    // Calculate relative capability: c_hat_i = c_i / sum(c_j)
    if (totalCapability > 0.0)
    {
        for (auto& d : droneList)
        {
            d.relative_capability_score = d.relative_capability_score / totalCapability;
            qDebug() << "Drone" << d.name << "- Relative capability score:" << d.relative_capability_score;
        }
    }
    else
    {
        qWarning() << "Total capability is zero; cannot calculate relative capabilities";
    }
}

// Helper function to calculate polygon area using Shoelace formula
double
PathPlanner::calculatePolygonArea(const QPolygonF& polygon)
{
    if (polygon.size() < 3)
    {
        return 0.0;
    }

    double area = 0.0;
    for (int i = 0; i < polygon.size(); ++i)
    {
        QPointF p1 = polygon[i];
        QPointF p2 = polygon[(i + 1) % polygon.size()];
        area += (p1.x() * p2.y() - p2.x() * p1.y());
    }

    return qAbs(area) / 2.0;
}

void
PathPlanner::setDecomposedROIs(const QList<QPair<QPolygonF, QString>>& decompROIs)
{
    this->decomposedPolygons = decompROIs;
}

QList<QPair<QPolygonF, QString>>
PathPlanner::getDecomposedROIs() const
{
    return this->decomposedPolygons;
}

double
PathPlanner::compute_partitioned_area(const double theta, QTransform& EF, const QTransform& AD, const QTransform& BC,
                                      QPolygonF& target, double marHeight)
{
    // Based off "EF = AD * (1 - theta) + theta * BC;"
    EF.setMatrix(AD.m11() * (1 - theta) + BC.m11() * theta, AD.m12() * (1 - theta) + BC.m12() * theta,
                 AD.m13() * (1 - theta) + BC.m13() * theta, AD.m21() * (1 - theta) + BC.m21() * theta,
                 AD.m22() * (1 - theta) + BC.m22() * theta, AD.m23() * (1 - theta) + BC.m23() * theta,
                 AD.m31() * (1 - theta) + BC.m31() * theta, AD.m32() * (1 - theta) + BC.m32() * theta,
                 AD.m33() * (1 - theta) + BC.m33() * theta);
    QPolygonF divider = getBoundingBox(AD, EF, marHeight);
    QPolygonF clipped = suth_hodgman_polygon_clipper(divider, target);
    return std::abs(calculatePolygonArea(clipped));
}

double
PathPlanner::binary_search(double cap, QTransform& EF, const QTransform& AD, const QTransform& BC, QPolygonF& target,
                           double marHeight)
{
    double start = 0.0;
    double RES = 0.00001;
    const int N = static_cast<int>(1.0 / RES);
    std::vector<double> ivec(N);
    std::generate(ivec.begin(), ivec.end(), [=]() mutable {
        start += RES;
        return start;
    });

    int l = 0;
    int r = N - 1;
    while (l <= r)
    {
        int m = l + (r - l) / 2;
        double pivot = ivec[m];
        double area = compute_partitioned_area(pivot, EF, AD, BC, target, marHeight);
        double diff = cap - area;
        if (std::abs(diff) < 1e-8)
        {
            return pivot;
        }
        else if (diff > 0)
        {
            l = m + 1;
        }
        else
        {
            r = m - 1;
        }
    }
    return ivec[std::max(0, r)];
}

QPolygonF
PathPlanner::getBoundingBox(const QTransform& AD, const QTransform& EF, double height)
{
    // AD and EF are transforms for the left and right divider lines.
    // height is the MAR height (in WGS84 units).
    QPolygonF bbox;
    // Bottom edge (y=0)
    QPointF p0 = AD.map(QPointF(0, 0));
    QPointF p1 = EF.map(QPointF(0, 0));
    // Top edge (y=height)
    QPointF p2 = EF.map(QPointF(0, height));
    QPointF p3 = AD.map(QPointF(0, height));
    bbox << p0 << p1 << p2 << p3 << p0; // closed
    return bbox;
}

QPolygonF
PathPlanner::suth_hodgman_polygon_clipper(QPolygonF& divider_poly, QPolygonF& target_poly)
{
    // Sutherland-Hodgman polygon clipping algorithm
    QPolygonF inputPoly = target_poly;
    QPolygonF outputPoly;

    int dividerCount = divider_poly.size();
    if (dividerCount < 3 || inputPoly.size() < 3)
        return QPolygonF();

    // Helper lambda: inside test for edge (clip edge from divider_poly)
    auto inside = [](const QPointF& p, const QPointF& edgeStart, const QPointF& edgeEnd) {
        // Returns true if p is on the left side of edge (edgeStart->edgeEnd)
        return ((edgeEnd.x() - edgeStart.x()) * (p.y() - edgeStart.y()) -
                (edgeEnd.y() - edgeStart.y()) * (p.x() - edgeStart.x())) >= 0.0;
    };

    // Helper lambda: compute intersection point of two lines (p1-p2 and q1-q2)
    auto computeIntersection = [](const QPointF& p1, const QPointF& p2, const QPointF& q1,
                                  const QPointF& q2) -> QPointF {
        double a1 = p2.y() - p1.y();
        double b1 = p1.x() - p2.x();
        double c1 = a1 * p1.x() + b1 * p1.y();

        double a2 = q2.y() - q1.y();
        double b2 = q1.x() - q2.x();
        double c2 = a2 * q1.x() + b2 * q1.y();

        double det = a1 * b2 - a2 * b1;
        if (std::fabs(det) < 1e-12)
            return p2; // Lines are parallel, return p2 as fallback

        double x = (b2 * c1 - b1 * c2) / det;
        double y = (a1 * c2 - a2 * c1) / det;
        return QPointF(x, y);
    };

    // For each edge of the divider (clip) polygon
    for (int i = 0; i < dividerCount; ++i)
    {
        outputPoly.clear();
        QPointF clipEdgeStart = divider_poly[i];
        QPointF clipEdgeEnd = divider_poly[(i + 1) % dividerCount];

        int inputCount = inputPoly.size();
        if (inputCount == 0)
            break;

        for (int j = 0; j < inputCount; ++j)
        {
            QPointF curr = inputPoly[j];
            QPointF prev = inputPoly[(j + inputCount - 1) % inputCount];
            bool currInside = inside(curr, clipEdgeStart, clipEdgeEnd);
            bool prevInside = inside(prev, clipEdgeStart, clipEdgeEnd);

            if (currInside)
            {
                if (!prevInside)
                {
                    // Edge enters the clip region: add intersection
                    QPointF intersect = computeIntersection(prev, curr, clipEdgeStart, clipEdgeEnd);
                    outputPoly << intersect;
                }
                // Add current point
                outputPoly << curr;
            }
            else if (prevInside)
            {
                // Edge exits the clip region: add intersection
                QPointF intersect = computeIntersection(prev, curr, clipEdgeStart, clipEdgeEnd);
                outputPoly << intersect;
            }
            // else: both outside, add nothing
        }
        inputPoly = outputPoly;
    }

    // Optionally, ensure closed polygon (Qt polygons are usually open, but close if needed)
    if (!inputPoly.isEmpty() && inputPoly.first() != inputPoly.last())
        inputPoly << inputPoly.first();

    return inputPoly;
}

QPolygonF
PathPlanner::makeDividerPoly(const RotatedRect& left, const RotatedRect& right)
{
    double h = left.height; // or right.height, should be the same
    QPolygonF poly;
    // Bottom edge (y=0)
    QPointF p0 = left.origin;
    QPointF p1 = right.origin;
    // Top edge (y=h)
    QPointF p2 = right.origin + h * right.uy;
    QPointF p3 = left.origin + h * left.uy;
    poly << p0 << p1 << p2 << p3 << p0; // closed
    return poly;
}

QTransform
PathPlanner::rectToTransform(const RotatedRect& rect)
{
    QTransform result;
    // Include translation (origin) in m13, m23
    result = QTransform(rect.ux.x(), rect.uy.x(), rect.origin.x(), // m11, m12, m13
                        rect.ux.y(), rect.uy.y(), rect.origin.y(), // m21, m22, m23
                        0, 0, 1);                                  // m31, m32, m33
    qInfo() << result.map(QPointF(0, 0)) << " =?= " << rect.origin;
    qInfo() << result.map(QPointF(0, rect.height)) << " =?= " << (rect.origin + rect.height * rect.uy);
    return result;
}

QList<QPair<QPolygonF, QString>>
PathPlanner::decomposedROI(QPolygonF& roi, QList<drone>& droneList, const RotatedRect& mar)
{
    QList<QPair<QPolygonF, QString>> result;

    if (roi.size() < 3 || droneList.isEmpty() || mar.width <= 0.0 || mar.height <= 0.0)
        return result;

    // Calculate total capability sum
    double totalCap = 0.0;
    QVector<double> capabilities;
    for (const drone& d : droneList)
    {
        capabilities.append(d.relative_capability_score);
        totalCap += d.relative_capability_score;
    }
    if (totalCap <= 0.0)
        return result;

    // Total area of ROI
    double totalArea = calculatePolygonArea(roi);

    // Decompose using direct vector math
    double start = 0.0;
    double cumCap = 0.0;
    for (int i = 0; i < capabilities.size(); ++i)
    {
        cumCap += capabilities[i];
        double targetArea = cumCap / totalCap * totalArea;

        // Binary search for 'end' along MAR width
        double left = start;
        double right = mar.width;
        double end = right;
        for (int iter = 0; iter < 30; ++iter) // 30 iterations for high precision
        {
            double mid = (left + right) / 2.0;
            QPolygonF divider = makeRectPoly(mar, start, mid);
            QPolygonF clipped = suth_hodgman_polygon_clipper(divider, roi);
            double area = calculatePolygonArea(clipped);
            if (area < (targetArea - 1e-8))
                left = mid;
            else
                right = mid;
        }
        end = (left + right) / 2.0;

        QPolygonF divider = makeRectPoly(mar, start, end);
        QPolygonF polygon = suth_hodgman_polygon_clipper(divider, roi);
        qInfo() << "Divider for drone" << droneList[i].id << ":" << divider;
        qInfo() << "Clipped polygon for drone" << droneList[i].id << "vertices:" << polygon.size()
                << "area:" << calculatePolygonArea(polygon);
        result.append({polygon, QString::number(droneList[i].id)});
        start = end;
    }

    return result;
}
