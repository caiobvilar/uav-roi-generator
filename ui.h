#ifndef UI_H
#define UI_H

#include <QMainWindow>

namespace Ui {
class UI;
}

class UI : public QMainWindow
{
    Q_OBJECT

public:
    explicit UI(QWidget *parent = nullptr);
    ~UI();

private:
    Ui::UI *ui;
};

#endif // UI_H
