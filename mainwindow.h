#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <opencv2/opencv.hpp>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    Ui::MainWindow *ui;
    QStringList files;
    cv::Size board_size;
    cv::Size square_size;

public slots:
    void on_actionloadImage_triggered();

    void on_actionQuit_triggered();

    void on_listMenu_clicked(const QModelIndex &index);

    void on_actioncameraCalibration_triggered();

    void on_actionSURF_triggered();

    void on_actionsparseMatching_triggered();

private:

};

#endif // MAINWINDOW_H
