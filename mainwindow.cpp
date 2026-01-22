#include "mainwindow.h"
#include "pathplanner.h"
#include "ui_mainwindow.h"
#include <QApplication>
#include <QCloseEvent>
#include <QColorDialog>
#include <QFileDialog>
#include <QImageWriter>
#include <QInputDialog>
#include <QMenuBar>
#include <QMessageBox>
#include <QStandardItemModel>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("ROIGenerator");

    connect(ui->actionQuit, &QAction::triggered, this, &MainWindow::close);
    connect(ui->actionOpen_File, &QAction::triggered, this, &MainWindow::onOpenFileTriggered);
    connect(ui->actionClose_File, &QAction::triggered, this, &MainWindow::onCloseFileTriggered);
    connect(ui->actionAdd_Layer_2, &QAction::triggered, this, &MainWindow::onAddLayerTriggered);
    connect(ui->actionRemove_Layer,
            &QAction::triggered,
            this,
            &MainWindow::onRemoveOverlayTriggered);
    connect(ui->actionSIRGAS2000_UTM24S,
            &QAction::triggered,
            this,
            &MainWindow::onSaveDrawnAreaTriggeredUTM24S);
    connect(ui->actionWGS84, &QAction::triggered, this, &MainWindow::onSaveDrawnAreaTriggeredWGS84);
    connect(ui->actionOpen_GeoJSON_File,
            &QAction::triggered,
            this,
            &MainWindow::onOpenGeoJSONFileTriggered);
    connect(ui->actionCalculate_Minimum_Area_Rectangle,
            &QAction::triggered,
            this,
            &MainWindow::onCalculateMinAreaRectTriggered);
    connect(ui->actionOpen_Drone_Info, &QAction::triggered, this, &MainWindow::onOpenDroneInfo);
    connect(ui->actionCalculate_Drone_Cap,
            &QAction::triggered,
            this,
            &MainWindow::onCalculateDroneCapabilities);
    connect(ui->actionDecompose_ROI, &QAction::triggered, this, &MainWindow::onDecomposeROI);
    connect(ui->actionShow_Decomposed_ROI,
            &QAction::triggered,
            this,
            &MainWindow::onShowDecomposedROI);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onOpenFileTriggered()
{
    QString fileName = QFileDialog::getOpenFileName(
        this,
        tr("Open image"),
        QString(),
        tr("Images (*.tif *.tiff *.png *.jpg *.jpeg *.bmp);;All files (*.*)"));

    if (!fileName.isEmpty())
        ui->roiArea->openImage(fileName);
}

void MainWindow::onAddLayerTriggered()
{
    ui->roiArea->addOverlay(ui->roiArea->getCurrentImage());
}

void MainWindow::onRemoveOverlayTriggered()
{
    ui->roiArea->removeOverlay();
}

void MainWindow::onCloseFileTriggered()
{
    ui->roiArea->closeImage();
}

void MainWindow::onSaveDrawnAreaTriggeredUTM24S()
{
    // Remove this function or rename it to just save (it's now always WGS84)
    QByteArray geoJson = ui->roiArea->exportPolygonGeoJSON();
    ui->roiArea->saveGEOJson(geoJson);
}

void MainWindow::onSaveDrawnAreaTriggeredWGS84()
{
    // This can now be the same as above, or you can remove one
    QByteArray geoJson = ui->roiArea->exportPolygonGeoJSON();
    ui->roiArea->saveGEOJson(geoJson);
}

void MainWindow::onOpenGeoJSONFileTriggered()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Open image"),
                                                    QString(),
                                                    tr("Json Documents (*.json);;All files (*.*)"));

    if (!fileName.isEmpty()) {
        ui->roiArea->drawGeoPolygonOnCurrentOverlay(ui->roiArea->openGeoJSONFilePoints(fileName));
    }
}

void MainWindow::onOpenGeoJSONAndDrawOnOverlayTriggered()
{
    QString fileName
        = QFileDialog::getOpenFileName(this,
                                       tr("Open GeoJSON polygon"),
                                       QString(),
                                       tr("GeoJSON (*.geojson *.json);;All files (*.*)"));
    if (fileName.isEmpty())
        return;

    QList<QPointF> pts = ui->roiArea->openGeoJSONFilePoints(fileName);
    if (pts.isEmpty())
        return;

    // Ensure an overlay exists and becomes current
    ui->roiArea->addOverlay(ui->roiArea->getCurrentImage());

    // Draw into the current overlay
    ui->roiArea->drawGeoPolygonOnCurrentOverlay(pts);
}

void MainWindow::onCalculateMinAreaRectTriggered()
{
    ui->roiArea->calculateMinimumAreaRectangle();
}

void MainWindow::onOpenDroneInfo()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Open Drone Info"),
                                                    QString(),
                                                    tr("JSON (*.json);;All files (*.*)"));

    if (!fileName.isEmpty())
        ui->roiArea->openDroneFile(fileName);
}

void MainWindow::onCalculateDroneCapabilities()
{
    QList<drone> drones = ui->roiArea->calculateDroneCapabilities();
    
    if (drones.isEmpty()) {
        statusBar()->showMessage("No drones to display. Load drone info first.", 3000);
        return;
    }
    
    QStandardItemModel *model = new QStandardItemModel();
    model->setHorizontalHeaderLabels(QStringList() << "Property" << "Value");
    
    for (const drone &d : drones) {
        // Create root item for this drone
        QStandardItem *droneItem = new QStandardItem(QString("%1 (ID: %2)").arg(d.name).arg(d.id));
        droneItem->setEditable(false);
        
        // Flight Altitude
        QString altitudeStr = d.ideal_flight_altitude > 100.0
            ? QString::number(d.ideal_flight_altitude / 100.0, 'f', 2) + " m"
            : QString::number(d.ideal_flight_altitude, 'f', 2) + " cm";
        QStandardItem *altitudeLabel = new QStandardItem("Flight Altitude");
        QStandardItem *altitudeValue = new QStandardItem(altitudeStr);
        altitudeLabel->setEditable(false);
        altitudeValue->setEditable(false);
        droneItem->appendRow(QList<QStandardItem*>() << altitudeLabel << altitudeValue);
        
        // Camera Footprint X
        QString footprintXStr = d.max_x_footprint > 100.0
            ? QString::number(d.max_x_footprint / 100.0, 'f', 2) + " m"
            : QString::number(d.max_x_footprint, 'f', 2) + " cm";
        QStandardItem *footprintXLabel = new QStandardItem("Camera Footprint X");
        QStandardItem *footprintXValue = new QStandardItem(footprintXStr);
        footprintXLabel->setEditable(false);
        footprintXValue->setEditable(false);
        droneItem->appendRow(QList<QStandardItem*>() << footprintXLabel << footprintXValue);
        
        // Camera Footprint Y
        QString footprintYStr = d.max_y_footprint > 100.0
            ? QString::number(d.max_y_footprint / 100.0, 'f', 2) + " m"
            : QString::number(d.max_y_footprint, 'f', 2) + " cm";
        QStandardItem *footprintYLabel = new QStandardItem("Camera Footprint Y");
        QStandardItem *footprintYValue = new QStandardItem(footprintYStr);
        footprintYLabel->setEditable(false);
        footprintYValue->setEditable(false);
        droneItem->appendRow(QList<QStandardItem*>() << footprintYLabel << footprintYValue);
        
        // Max Forward Velocity
        QString velocityStr = d.max_forward_velocity > 100.0
            ? QString::number(d.max_forward_velocity / 100.0, 'f', 2) + " m/s"
            : QString::number(d.max_forward_velocity, 'f', 2) + " cm/s";
        QStandardItem *velocityLabel = new QStandardItem("Max Forward Velocity");
        QStandardItem *velocityValue = new QStandardItem(velocityStr);
        velocityLabel->setEditable(false);
        velocityValue->setEditable(false);
        droneItem->appendRow(QList<QStandardItem*>() << velocityLabel << velocityValue);
        
        // Relative Capability
        QStandardItem *capabilityLabel = new QStandardItem("Relative Capability");
        QStandardItem *capabilityValue = new QStandardItem(QString::number(d.relative_capability_score, 'f', 4));
        capabilityLabel->setEditable(false);
        capabilityValue->setEditable(false);
        droneItem->appendRow(QList<QStandardItem*>() << capabilityLabel << capabilityValue);
        
        // Battery
        QStandardItem *batteryLabel = new QStandardItem("Battery");
        QStandardItem *batteryValue = new QStandardItem(
            QString("%1 / %2 mAh")
                .arg(d.battery_current_capacity, 0, 'f', 0)
                .arg(d.battery_capacity, 0, 'f', 0)
        );
        batteryLabel->setEditable(false);
        batteryValue->setEditable(false);
        droneItem->appendRow(QList<QStandardItem*>() << batteryLabel << batteryValue);
        
        // Add drone to model
        model->appendRow(droneItem);
    }
    
    // Set the new model - the treeView takes ownership and will delete old model
    QAbstractItemModel *oldModel = ui->treeView->model();
    ui->treeView->setModel(model);
    if (oldModel) {
        oldModel->deleteLater();
    }
    
    // Expand all items and resize columns
    ui->treeView->expandAll();
    ui->treeView->resizeColumnToContents(0);
    ui->treeView->resizeColumnToContents(1);
    ui->treeView->updateGeometry();
    ui->treeView->adjustSize();
    this->updateGeometry();
}

void MainWindow::onDecomposeROI()
{
    ui->roiArea->decomposeROI();
}
void MainWindow::onShowDecomposedROI()
{
    ui->roiArea->showDecomposedROI();
}

void MainWindow::on_roiArea_StatusMessageChanged(const QString &text)
{
    statusBar()->showMessage(text);
}
