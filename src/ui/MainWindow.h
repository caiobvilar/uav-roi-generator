#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "mission/MissionController.h"

#include <QMainWindow>
#include <memory>

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

  public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

  private slots:
    void
    onOpenFileTriggered();
    void
    onOpenGeoJSONFileTriggered();
    void
    onCloseFileTriggered();
    void
    onAddLayerTriggered();
    void
    onRemoveOverlayTriggered();

    void
    onSaveDrawnAreaTriggeredUTM24S();
    void
    onSaveDrawnAreaTriggeredWGS84();
    void
    onCalculateMinAreaRectTriggered();
    void
    onOpenDroneInfo();
    void
    onCalculateDroneCapabilities();
    void
    onDecomposeROI();
    void
    onShowDecomposedROI();
    void
    onCalculateWaypoints();

    // Auto-connected slot for ImageCanvas::StatusMessageChanged
    void
    on_roiArea_StatusMessageChanged(const QString& text);

  private:
    std::unique_ptr<Ui::MainWindow> ui;
    MissionController m_controller;
};

#endif // MAINWINDOW_H
