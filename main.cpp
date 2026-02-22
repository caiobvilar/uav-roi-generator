#include "mainwindow.h"

#include <QApplication>
#include <QLoggingCategory>

int
main(int argc, char* argv[])
{
    QApplication a(argc, argv);
    QLoggingCategory::setFilterRules("*.warning=true");
    MainWindow w;
    w.show();
    return a.exec();
}
