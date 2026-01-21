#include "pathplanner.h"
#include <qlist.h>
#include <qpolygon.h>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#define DESIRED_GSD 2.0 // This asumes a desired ground sample distance of 2.0cm/pixel for precision agriculture
#define FORWARD_OVERLAP 0.8 // This assumes precision agriculture
#define SIDE_OVERLAP 0.75 //This also assumes precision agriculture

PathPlanner::PathPlanner(QObject *parent)
    : QObject{parent}
{}

QList<drone> PathPlanner::getDroneInfo(const QString &filename)
{
    QList<drone> results;
    
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Failed to open file:" << filename;
        return results;
    }
    
    QByteArray data = file.readAll();
    file.close();
    
    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (!doc.isObject()) {
        qWarning() << "Invalid JSON format";
        return results;
    }
    
    QJsonObject rootObj = doc.object();
    QJsonArray dronesArray = rootObj["drones"].toArray();
    
    for (int i = 0; i < dronesArray.size(); ++i) {
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
void PathPlanner::calcFlightAltitude(QList<drone> droneList)
{
    for (auto &d : droneList) {
        // GSD_x * f_l * i_x / l_x
        double h_x = (DESIRED_GSD * d.camera_focal_length * d.camera_image_width) / d.camera_array_width;
        
        // GSD_y * f_l * i_y / l_y
        double h_y = (DESIRED_GSD * d.camera_focal_length * d.camera_image_height) / d.camera_array_height;
        
        // h = min{h_x, h_y}
        d.ideal_flight_altitude = std::min(h_x, h_y);
        
        qDebug() << "Drone" << d.name << "- Ideal flight altitude:" << d.ideal_flight_altitude << "cm";
    }
}

void PathPlanner::calcDroneCameraFootprint(QList<drone> droneList)
{
    for (auto &d : droneList) {
        // L_x = h * l_x / f_l
        d.max_x_footprint = (d.ideal_flight_altitude * d.camera_array_width) / d.camera_focal_length;
        
        // L_y = h * l_y / f_l
        d.max_y_footprint = (d.ideal_flight_altitude * d.camera_array_height) / d.camera_focal_length;
        
        qDebug() << "Drone" << d.name << "- Camera footprint:";
        qDebug() << "  L_x:" << d.max_x_footprint << "cm";
        qDebug() << "  L_y:" << d.max_y_footprint << "cm";
    }
}
void PathPlanner::calcMaximumForwardVelocity(QList<drone> droneList)
{
    for (auto &d : droneList) {
        // V_max = L_y * (1 - O_f) / s_h
        d.max_forward_velocity = (d.max_y_footprint * (1.0 - FORWARD_OVERLAP)) / d.camera_shutter_speed;
        
        qDebug() << "Drone" << d.name << "- Maximum forward velocity:" << d.max_forward_velocity << "cm/s";
        qDebug() << "  (with" << (FORWARD_OVERLAP * 100) << "% forward overlap)";
    }
}

void PathPlanner::calcDroneRelativeCapability(QList<drone> droneList)
{
    // Calculate capability for each drone: c_i = V_max^i * L_x^i
    double totalCapability = 0.0;
    
    for (auto &d : droneList) {
        d.relative_capability_score = d.max_forward_velocity * d.max_x_footprint;
        totalCapability += d.relative_capability_score;
    }
    
    // Calculate relative capability: c_hat_i = c_i / sum(c_j)
    if (totalCapability > 0.0) {
        for (auto &d : droneList) {
            d.relative_capability_score = d.relative_capability_score / totalCapability;
            qDebug() << "Drone" << d.name << "- Relative capability score:" << d.relative_capability_score;
        }
    } else {
        qWarning() << "Total capability is zero; cannot calculate relative capabilities";
    }
}

// Helper function to calculate polygon area using Shoelace formula
double PathPlanner::calculatePolygonArea(const QPolygonF &polygon)
{
    if (polygon.size() < 3) {
        return 0.0;
    }
    
    double area = 0.0;
    for (int i = 0; i < polygon.size(); ++i) {
        QPointF p1 = polygon[i];
        QPointF p2 = polygon[(i + 1) % polygon.size()];
        area += (p1.x() * p2.y() - p2.x() * p1.y());
    }
    
    return qAbs(area) / 2.0;
}

QList<QPolygonF> PathPlanner::decomposedROI(QPolygonF &roi, QList<drone> &droneList)
{
    QList<QPolygonF> decomposedPolygons;
    
    if (droneList.isEmpty()) {
        qWarning() << "No drones provided for ROI decomposition";
        return decomposedPolygons;
    }
    
    // Calculate total ROI area using Shoelace formula
    double totalROIArea = calculatePolygonArea(roi);
    
    if (totalROIArea <= 0.0) {
        qWarning() << "Invalid ROI polygon area";
        return decomposedPolygons;
    }
    
    int numDrones = droneList.size();
    decomposedPolygons.resize(numDrones);
    
    // Binary search decomposition: split ROI based on relative capabilities
    // For each drone, assign a sub-polygon with area = relative_capability * total_area
    
    QPolygonF remainingROI = roi;
    
    for (int i = 0; i < numDrones - 1; ++i) {
        double targetArea = droneList[i].relative_capability_score * totalROIArea;
        
        // Binary search for the cut line that divides the polygon
        // Use a simple approach: cut along a horizontal or vertical line
        QRectF boundingBox = remainingROI.boundingRect();
        
        double minY = boundingBox.top();
        double maxY = boundingBox.bottom();
        double currentArea = 0.0;
        double cutY = minY;
        
        // Binary search to find the Y coordinate that gives us the target area
        double low = minY;
        double high = maxY;
        const double tolerance = 0.01; // 0.01 unit tolerance
        
        while ((high - low) > tolerance) {
            double mid = (low + high) / 2.0;
            
            // Create a line at y = mid and calculate the area below it
            QPolygonF bottomPart;
            
            for (int j = 0; j < remainingROI.size(); ++j) {
                QPointF p = remainingROI[j];
                if (p.y() <= mid) {
                    bottomPart.append(p);
                }
            }
            
            // Add intersection points at the cut line
            for (int j = 0; j < remainingROI.size(); ++j) {
                QPointF p1 = remainingROI[j];
                QPointF p2 = remainingROI[(j + 1) % remainingROI.size()];
                
                // Check if edge crosses the cut line
                if ((p1.y() <= mid && p2.y() > mid) || (p1.y() > mid && p2.y() <= mid)) {
                    double t = (mid - p1.y()) / (p2.y() - p1.y());
                    QPointF intersection(p1.x() + t * (p2.x() - p1.x()), mid);
                    bottomPart.append(intersection);
                }
            }
            
            currentArea = calculatePolygonArea(bottomPart);
            
            if (currentArea < targetArea) {
                low = mid;
            } else {
                high = mid;
            }
            cutY = mid;
        }
        
        // Create the sub-polygon for this drone by cutting at cutY
        QPolygonF dronePart;
        QPolygonF nextRemaining;
        
        for (int j = 0; j < remainingROI.size(); ++j) {
            QPointF p = remainingROI[j];
            if (p.y() <= cutY) {
                dronePart.append(p);
            } else {
                nextRemaining.append(p);
            }
        }
        
        // Add intersection points at the cut line
        for (int j = 0; j < remainingROI.size(); ++j) {
            QPointF p1 = remainingROI[j];
            QPointF p2 = remainingROI[(j + 1) % remainingROI.size()];
            
            // Check if edge crosses the cut line
            if ((p1.y() <= cutY && p2.y() > cutY) || (p1.y() > cutY && p2.y() <= cutY)) {
                double t = (cutY - p1.y()) / (p2.y() - p1.y());
                QPointF intersection(p1.x() + t * (p2.x() - p1.x()), cutY);
                dronePart.append(intersection);
                nextRemaining.append(intersection);
            }
        }
        
        decomposedPolygons[i] = dronePart;
        remainingROI = nextRemaining;
        
        qDebug() << "Drone" << droneList[i].name << "- Assigned sub-polygon area:"
                 << calculatePolygonArea(dronePart);
    }
    
    // Assign remaining polygon to the last drone
    decomposedPolygons[numDrones - 1] = remainingROI;
    qDebug() << "Drone" << droneList[numDrones - 1].name << "- Assigned sub-polygon area:"
             << calculatePolygonArea(remainingROI);
    
    qDebug() << "ROI decomposed into" << numDrones << "non-overlapping sub-polygons";
    
    return decomposedPolygons;
}