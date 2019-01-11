#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    //w.setWindowTitle("Remote Sensing Monitoring System for Plantation Resources");
    w.setWindowTitle(u8"测试用");
    w.show();

    return a.exec();
}
