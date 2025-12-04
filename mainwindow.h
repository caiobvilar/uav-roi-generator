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

    void onStatusMessageChanged(const QString &text);
    void onSaveDrawnAreaTriggeredUTM24S();
    void onSaveDrawnAreaTriggeredWGS84();
    void onOpenGeoJSONAndDrawOnOverlayTriggered();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
