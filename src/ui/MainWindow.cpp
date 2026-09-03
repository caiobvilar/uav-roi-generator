#include "ui/MainWindow.h"
#include "geometry/PolygonGeometry.h"
#include "ui_mainwindow.h"
#include <QApplication>
#include <QCloseEvent>
#include <QColorDialog>
#include <QFileDialog>
#include <QImageWriter>
#include <QMenuBar>
#include <QMessageBox>
#include <QStandardItemModel>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui(std::make_unique<Ui::MainWindow>()), m_controller(this)
{
    ui->setupUi(this);
    setWindowTitle("ROIGenerator");

    connect(ui->actionQuit, &QAction::triggered, this, &MainWindow::close);
    connect(ui->actionOpen_File, &QAction::triggered, this, &MainWindow::onOpenFileTriggered);
    connect(ui->actionClose_File, &QAction::triggered, this, &MainWindow::onCloseFileTriggered);
    connect(ui->actionAdd_Layer_2, &QAction::triggered, this, &MainWindow::onAddLayerTriggered);
    connect(ui->actionRemove_Layer, &QAction::triggered, this, &MainWindow::onRemoveOverlayTriggered);
    connect(ui->actionSIRGAS2000_UTM24S, &QAction::triggered, this, &MainWindow::onSaveDrawnAreaTriggeredUTM24S);
    connect(ui->actionWGS84, &QAction::triggered, this, &MainWindow::onSaveDrawnAreaTriggeredWGS84);
    connect(ui->actionOpen_GeoJSON_File, &QAction::triggered, this, &MainWindow::onOpenGeoJSONFileTriggered);
    connect(ui->actionCalculate_Minimum_Area_Rectangle, &QAction::triggered, this,
            &MainWindow::onCalculateMinAreaRectTriggered);
    connect(ui->actionOpen_Drone_Info, &QAction::triggered, this, &MainWindow::onOpenDroneInfo);
    connect(ui->actionCalculate_Drone_Cap, &QAction::triggered, this, &MainWindow::onCalculateDroneCapabilities);
    connect(ui->actionDecompose_ROI, &QAction::triggered, this, &MainWindow::onDecomposeROI);
    connect(ui->actionShow_Decomposed_ROI, &QAction::triggered, this, &MainWindow::onShowDecomposedROI);
    connect(ui->actionWaypoints, &QAction::triggered, this, &MainWindow::onCalculateWaypoints);

    connect(&m_controller, &MissionController::statusMessageChanged, this,
            &MainWindow::on_roiArea_StatusMessageChanged);
}

MainWindow::~MainWindow() = default;

void
MainWindow::onOpenFileTriggered()
{
    QString fileName = QFileDialog::getOpenFileName(
        this, tr("Open image"), QString(), tr("Images (*.tif *.tiff *.png *.jpg *.jpeg *.bmp);;All files (*.*)"));

    if (!fileName.isEmpty())
        ui->roiArea->openImage(fileName);
}

void
MainWindow::onAddLayerTriggered()
{
    ui->roiArea->addOverlay(ui->roiArea->getOverlayStackTop().first, "Layer");
}

void
MainWindow::onRemoveOverlayTriggered()
{
    ui->roiArea->removeOverlay();
}

void
MainWindow::onCloseFileTriggered()
{
    ui->roiArea->closeImage();
}

void
MainWindow::onSaveDrawnAreaTriggeredUTM24S()
{
    // Remove this function or rename it to just save (it's now always WGS84)
    QByteArray geoJson = ui->roiArea->exportPolygonGeoJSON();
    ui->roiArea->saveGEOJson(geoJson);
}

void
MainWindow::onSaveDrawnAreaTriggeredWGS84()
{
    // This can now be the same as above, or you can remove one
    QByteArray geoJson = ui->roiArea->exportPolygonGeoJSON();
    ui->roiArea->saveGEOJson(geoJson);
}

void
MainWindow::onOpenGeoJSONFileTriggered()
{
    QString fileName =
        QFileDialog::getOpenFileName(this, tr("Open image"), QString(), tr("Json Documents (*.json);;All files (*.*)"));

    if (!fileName.isEmpty())
    {
        ui->roiArea->drawGeoPolygonOnCurrentOverlay(ui->roiArea->openGeoJSONFilePoints(fileName));
    }
}

void
MainWindow::onCalculateMinAreaRectTriggered()
{
    ui->roiArea->calculateMinimumAreaRectangle();
    ui->roiArea->drawMinimumAreaRectangle();
}

void
MainWindow::onOpenDroneInfo()
{
    QString fileName =
        QFileDialog::getOpenFileName(this, tr("Open Drone Info"), QString(), tr("JSON (*.json);;All files (*.*)"));

    if (!fileName.isEmpty())
        m_controller.loadDrones(fileName);
    onCalculateDroneCapabilities();
}

void
MainWindow::onCalculateDroneCapabilities()
{
    QList<Drone> drones = m_controller.calculateCapabilities();

    if (drones.isEmpty())
    {
        statusBar()->showMessage("No drones to display. Load drone info first.", 3000);
        return;
    }

    auto model = std::make_unique<QStandardItemModel>();
    model->setHorizontalHeaderLabels(QStringList() << "Property" << "Value");

    for (const Drone& d : drones)
    {
        // Create root item for this drone
        QStandardItem* droneItem = new QStandardItem(QString("%1 (ID: %2)").arg(d.name).arg(d.id));
        droneItem->setEditable(false);

        // Flight Altitude (SI: meters)
        QString altitudeStr = QString::number(d.ideal_flight_altitude, 'f', 2) + " m";
        QStandardItem* altitudeLabel = new QStandardItem("Ideal Flight Altitude");
        QStandardItem* altitudeValue = new QStandardItem(altitudeStr);
        altitudeLabel->setEditable(false);
        altitudeValue->setEditable(false);
        droneItem->appendRow(QList<QStandardItem*>() << altitudeLabel << altitudeValue);

        // Camera Footprint X (SI: meters)
        QString footprintXStr = QString::number(d.max_x_footprint, 'f', 2) + " m";
        QStandardItem* footprintXLabel = new QStandardItem("Camera Footprint X");
        QStandardItem* footprintXValue = new QStandardItem(footprintXStr);
        footprintXLabel->setEditable(false);
        footprintXValue->setEditable(false);
        droneItem->appendRow(QList<QStandardItem*>() << footprintXLabel << footprintXValue);

        // Camera Footprint Y (SI: meters)
        QString footprintYStr = QString::number(d.max_y_footprint, 'f', 2) + " m";
        QStandardItem* footprintYLabel = new QStandardItem("Camera Footprint Y");
        QStandardItem* footprintYValue = new QStandardItem(footprintYStr);
        footprintYLabel->setEditable(false);
        footprintYValue->setEditable(false);
        droneItem->appendRow(QList<QStandardItem*>() << footprintYLabel << footprintYValue);

        // Max Forward Velocity (SI: m/s)
        QString velocityStr = QString::number(d.max_forward_velocity, 'f', 2) + " m/s";
        QStandardItem* velocityLabel = new QStandardItem("Ideal Forward Velocity");
        QStandardItem* velocityValue = new QStandardItem(velocityStr);
        velocityLabel->setEditable(false);
        velocityValue->setEditable(false);
        droneItem->appendRow(QList<QStandardItem*>() << velocityLabel << velocityValue);

        // Relative Capability
        QStandardItem* capabilityLabel = new QStandardItem("Relative Capability");
        QStandardItem* capabilityValue = new QStandardItem(QString::number(d.relative_capability_score, 'f', 4));
        capabilityLabel->setEditable(false);
        capabilityValue->setEditable(false);
        droneItem->appendRow(QList<QStandardItem*>() << capabilityLabel << capabilityValue);

        // Battery
        QStandardItem* batteryLabel = new QStandardItem("Battery");
        QStandardItem* batteryValue = new QStandardItem(
            QString("%1 / %2 mAh").arg(d.battery_current_capacity, 0, 'f', 0).arg(d.battery_capacity, 0, 'f', 0));
        batteryLabel->setEditable(false);
        batteryValue->setEditable(false);
        droneItem->appendRow(QList<QStandardItem*>() << batteryLabel << batteryValue);

        // Add drone to model
        model->appendRow(droneItem);
    }

    // Set the new model - the treeView takes ownership and will delete old model
    ui->treeView->setModel(model.release());

    // Expand all items and resize columns
    ui->treeView->expandAll();
    ui->treeView->resizeColumnToContents(0);
    ui->treeView->resizeColumnToContents(1);
    ui->treeView->updateGeometry();
    ui->treeView->adjustSize();
    this->updateGeometry();
}

void
MainWindow::onDecomposeROI()
{
    m_controller.decompose(ui->roiArea->finalPolygonGeo(), ui->roiArea->getROIPolygonMinAreaRect());

    QList<QPair<QPolygonF, QString>> decomposed = m_controller.decomposed();

    int idx = 0;
    for (const auto& pair : decomposed)
    {
        const QPolygonF& poly = pair.first;
        const QString& droneId = pair.second;
        double area = geometry::shoelaceArea(poly);
        QString desc = QString("Polygon %1 | Drone ID: %2 | Vertices: %3 | Area: %4")
                           .arg(idx + 1)
                           .arg(droneId)
                           .arg(poly.size())
                           .arg(area, 0, 'f', 2);
        ui->listWidget->addItem(desc);
        idx++;
    }
}

void
MainWindow::onShowDecomposedROI()
{
    ui->roiArea->showDecomposedROI(m_controller.decomposed());
}

void
MainWindow::on_roiArea_StatusMessageChanged(const QString& text)
{
    // Optionally keep the status bar message
    statusBar()->showMessage(text);

    // Append the message to the listWidget
    ui->listWidget->addItem(text);

    // Optionally scroll to the bottom to show the latest message
    ui->listWidget->scrollToBottom();
}

void
MainWindow::onCalculateWaypoints()
{
    m_controller.generateWaypoints(ui->roiArea->getROIPolygonMinAreaRect());
    ui->roiArea->showWaypoints(m_controller.waypoints());
}
