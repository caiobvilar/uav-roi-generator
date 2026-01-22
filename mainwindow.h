#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onOpenFileTriggered();
    void onOpenGeoJSONFileTriggered();
    void onCloseFileTriggered();
    void onAddLayerTriggered();
    void onRemoveOverlayTriggered();

    void onSaveDrawnAreaTriggeredUTM24S();
    void onSaveDrawnAreaTriggeredWGS84();
    void onOpenGeoJSONAndDrawOnOverlayTriggered();
    void onCalculateMinAreaRectTriggered();
    void onOpenDroneInfo();
    void onCalculateDroneCapabilities();
    void onDecomposeROI();
    void onShowDecomposedROI();
    
    // Auto-connected slot for ROIArea::StatusMessageChanged
    void on_roiArea_StatusMessageChanged(const QString &text);

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
