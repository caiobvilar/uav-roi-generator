#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QApplication>
#include <QCloseEvent>
#include <QColorDialog>
#include <QFileDialog>
#include <QImageWriter>
#include <QInputDialog>
#include <QMenuBar>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("ROIGenerator");
    ui->verticalLayout->setAlignment(ui->roiArea, Qt::AlignCenter);
    connect(ui->roiArea, &ROIArea::StatusMessageChanged, this, &MainWindow::onStatusMessageChanged);
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
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::onStatusMessageChanged(const QString &text)
{
    ui->label->setText(text);
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
