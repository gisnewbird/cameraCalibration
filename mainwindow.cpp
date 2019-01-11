#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "cameracalibration.h"


#include <QStringListModel>
#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <QDateTime>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <QDebug>
#include <iostream>
#include <stdlib.h>
#include "omp.h"
#include <ctime>
#include <iostream>
#include <fstream>
//#include <openMVG/image/image_io.hpp>
//#include <nonFree/sift/SIFT_describer.hpp>
//#include "openMVG/exif/exif_IO_EasyExif.hpp"
using namespace cv;
using namespace std;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QStatusBar * sBar = statusBar();//创建状态栏
    sBar->addWidget(new QLabel(u8"北京林业大学 郭正齐 20181212",this));//addWidget:从左往右添加
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionloadImage_triggered()
{
    //打开单个文件
//    QString filename = QFileDialog::getOpenFileName(this,"Open Image",QDir::currentPath(),"Image files (*.jpg *.JPG);;All files(*.*)");
//    if (!filename.isNull())
//    {
//        //用户选择了文件
//        // 处理文件
//        QMessageBox::information(this, u8"信息反馈", u8"选择图像成功", QMessageBox::Ok | QMessageBox::Cancel);
//    }
//    else
//        // 用户取消选择
//        QMessageBox::information(this, "Image", "No image", QMessageBox::Ok | QMessageBox::Cancel);
    //QString openFilesPath = QFileDialog::getOpenFileName(this,"Open Image",QDir::currentPath(),"Document files (*.jpg *.JPG);;All files(*.*)");
    //选定多个文件的文件打开对话框
     QFileDialog::Options options;
     options = QFileDialog::DontUseNativeDialog;
     QString selectedFilter;
     files = QFileDialog::getOpenFileNames(this,tr("QFileDialog::getOpenFileNames()"),"",tr("AImage files (*.jpg *.JPG);;All files(*.*)"),&selectedFilter,options);
     if (!files.empty())
     {
         //用户选择了文件
         // 处理文件
         //QMessageBox::information(this, "Image", "Has image", QMessageBox::Ok | QMessageBox::Cancel);
     }
     else
         // 用户取消选择
         QMessageBox::information(this, u8"信息反馈", u8"请选择图像", QMessageBox::Ok | QMessageBox::Cancel);
    QStringList fileNameList;
    for(int m = 0;m<files.count();m++){
        QString fileName = files[m];
        QStringList fileNameL = fileName.split("/");
        fileNameList<<fileNameL[fileNameL.count()-1];
    }
    ui->textLog->append(u8"已经加载了"+QString::number(fileNameList.count())+u8"张图");
    QStringListModel *model = new QStringListModel(fileNameList);
    ui->listMenu->setModel(model);
}

void MainWindow::on_actionQuit_triggered()
{
    close();
}

void MainWindow::on_listMenu_clicked(const QModelIndex &index)
{
    //ui->openGLWidget->update();
    QPainter painter;
    painter.begin(ui->openGLWidget);
    painter.setViewport(ui->openGLWidget->rect());
    QImage img;
    ui->textLog->append("Loading :"+files[index.row()]);
    img.load(files[index.row()]);
    painter.drawImage(QPoint(0, 0), img,ui->openGLWidget->rect());
    painter.end();
}
void MainWindow::on_actioncameraCalibration_triggered()
{
    clock_t myClock = clock();
    board_size = Size(ui->txtRow->text().toInt(), ui->txtCol->text().toInt());             /* 标定板上每行、列的角点数 */
    ui->textLog->append(u8"相机定标开始...");
    square_size = Size(ui->txtDistance->text().toInt(), ui->txtDistance->text().toInt());         /* 实际测量得到的标定板上每个棋盘格的大小 */
    cameraCalibration_triggered(files,square_size,board_size);
    myClock = clock();
    ui->textLog->append(QObject::tr(u8"用时:%1s").arg(myClock/1000));
    //输出相机参数
    ifstream fin("相机定标结果.txt");
    assert(fin.is_open());   //若失败,则输出错误消息,并终止程序运行
    string s;
    while(getline(fin,s))
    {
        if(s=="相机内参数矩阵："){
            getline(fin,s);
            QString neic = QString::fromStdString(s);
            ui->neican1->setText(neic.split(",")[0].split("[")[1]);
            ui->neican2->setText(neic.split(",")[1].split(" ")[1]);
            ui->neican3->setText((neic.split(",")[2].split(";")[0]).split(" ")[1]);
            getline(fin,s);
            neic = QString::fromStdString(s);
            ui->neican4->setText(neic.split(",")[0].split(" ")[1]);
            ui->neican5->setText(neic.split(",")[1].split(" ")[1]);
            ui->neican6->setText((neic.split(",")[2].split(";")[0]).split(" ")[1]);
            getline(fin,s);
            neic = QString::fromStdString(s);
            ui->neican7->setText(neic.split(",")[0].split(" ")[1]);
            ui->neican8->setText(neic.split(",")[1].split(" ")[1]);
            ui->neican9->setText((neic.split(",")[2].split("]")[0]).split(" ")[1]);
        }
        if(s=="畸变系数："){
            getline(fin,s);
            QString neic = QString::fromStdString(s);
            ui->jibian1->setText(neic.split(",")[0].split("[")[1]);
            ui->jibian2->setText(neic.split(",")[1].split(" ")[1]);
            ui->jibian3->setText(neic.split(",")[2].split(" ")[1]);
            ui->jibian4->setText(neic.split(",")[3].split(" ")[1]);
            ui->jibian5->setText((neic.split(",")[4].split("]")[0]).split(" ")[1]);
        }
    }
    ui->textLog->append(u8"相机定标完成!");
}
void MainWindow::on_actionSURF_triggered(){
    ui->textLog->append(u8"开始SURF检测");
    double begin = clock();
    SURF_triggered(files);
    double end = clock();
    ui->textLog->append(QObject::tr(u8"用时： %1 s").arg((end - begin) / CLOCKS_PER_SEC ));
}
//using namespace openMVG::image;
//using namespace openMVG::features;
//using namespace openMVG::sfm;
//using namespace openMVG::exif;
//using namespace openMVG::matching;
void MainWindow::on_actionsparseMatching_triggered(){
//    try
//    {
//        Mat3 K;//相机内参数矩阵

//        K(0, 0) = ui->neican1->text().toDouble();
//        K(0, 1) = ui->neican2->text().toDouble();
//        K(0, 2) = ui->neican3->text().toDouble();
//        K(1, 0) = ui->neican4->text().toDouble();
//        K(1, 1) = ui->neican5->text().toDouble();
//        K(1, 2) = ui->neican6->text().toDouble();
//        K(2, 0) = ui->neican7->text().toDouble();
//        K(2, 1) = ui->neican8->text().toDouble();
//        K(2, 2) = ui->neican9->text().toDouble();
//        std::vector<Vec3> vec_3DPoints;//生成的稀疏点云集合
//        std::vector<Vec3> vec_3DPointsRGB;//稀疏点云色彩集合
//        std::vector<Pose3> cameraPoses;//相机所处位置点集合
//        cameraPoses.resize(this->files.size());//与序列图像个数相同
//        cameraPoses[0] = Pose3(Mat3::Identity(), Vec3::Zero());//设置第一幅照片相机位置为(0,0,0)
//        cameraOWNposes.push_back(cameraPoses[0].center());
//        Mat34 P1, P2;
//        ui->textLog->append("\n" + QStringLiteral("开始进行立体恢复！"));
//        qApp->processEvents();
//        string mfilename;
//        //按顺序读取相邻像对
//        for (int img = 0; img < this->files.size() - 1; img++)
//        {
//            mfilename = this->files[img].toLocal8Bit().data();
//            const string jpg_filenameL = this->files[img].toLocal8Bit().data();
//            const string jpg_filenameR = this->files[img + 1].toLocal8Bit().data();
//            //读取左右相邻图像

//            Image<unsigned char> imageL, imageR;//存储灰度图
//            Image<RGBColor> rgb_imageL, rgb_imageR;//存储RGB图

//            ReadImage(jpg_filenameL.c_str(), &rgb_imageL);
//            ReadImage(jpg_filenameR.c_str(), &rgb_imageR);
//            ReadImage(jpg_filenameL.c_str(), &imageL);
//            ReadImage(jpg_filenameR.c_str(), &imageR);


//            SIFT_Regions* regionsL; SIFT_Regions* regionsR;
//            std::map<IndexT, std::unique_ptr<features::Regions> > regions_perImage;
//            PointFeatures featsL; PointFeatures featsR;
//            imageMatch* imgMatch = new imageMatch();
//            std::vector<IndMatch> vec_PutativeMatches =
//                    imgMatch->GetPutativeMatches(imageL, imageR,
//                                                 featsL, featsR, regions_perImage);

//            regionsL = dynamic_cast<SIFT_Regions*>(regions_perImage.at(0).get());
//            regionsR = dynamic_cast<SIFT_Regions*>(regions_perImage.at(1).get());

//            ui->textLog->append("\n" + QStringLiteral("第") + QString::number(img + 1) + QStringLiteral("对图像初始匹配")
//                                + QString::number(vec_PutativeMatches.size()) + QStringLiteral("个点"));
//            qApp->processEvents();



//            //图像精匹配
//            {
//                //相片匹配点对
//                openMVG::Mat xL(2, vec_PutativeMatches.size());
//                openMVG::Mat xR(2, vec_PutativeMatches.size());
//                for (size_t k = 0; k < vec_PutativeMatches.size(); ++k) {
//                    const PointFeature & imaL = featsL[vec_PutativeMatches[k].i_];
//                    const PointFeature & imaR = featsR[vec_PutativeMatches[k].j_];
//                    xL.col(k) = imaL.coords().cast<double>();
//                    xR.col(k) = imaR.coords().cast<double>();
//                }

//                //SFM 恢复相对相机外参数
//                std::pair<size_t, size_t> size_imaL(imageL.Width(), imageL.Height());
//                std::pair<size_t, size_t> size_imaR(imageR.Width(), imageR.Height());
//                sfm::RelativePose_Info relativePose_info;
//                //                    if (!sfm::robustRelativePose(K, K, xL, xR, relativePose_info, size_imaL, size_imaR, 256))
//                //                    {
//                //                        QMessageBox::information(this, QStringLiteral("提示"), QStringLiteral("图像精匹配出错！"));

//                //                        return;
//                //                    }

//                ui->textLog->append("\n" + QStringLiteral("第") + QString::number(img + 1) + QStringLiteral("对图像精匹配")
//                                    + QString::number(relativePose_info.vec_inliers.size()) + QStringLiteral("个点"));
//                qApp->processEvents();

//                // 绘制精匹配点对，输出svg图像
//                svg::svgDrawer svgStream(imageL.Width() + imageR.Width(), max(imageL.Height(), imageR.Height()));
//                svgStream.drawImage(jpg_filenameL, imageL.Width(), imageL.Height());
//                svgStream.drawImage(jpg_filenameR, imageR.Width(), imageR.Height(), imageL.Width());
//                for (size_t i = 0; i < relativePose_info.vec_inliers.size(); ++i) {
//                    const SIOPointFeature & LL = regionsL->Features()[vec_PutativeMatches[relativePose_info.vec_inliers[i]].i_];
//                    const SIOPointFeature & RR = regionsR->Features()[vec_PutativeMatches[relativePose_info.vec_inliers[i]].j_];
//                    const openMVG::Vec2f L = LL.coords();
//                    const openMVG::Vec2f R = RR.coords();
//                    svgStream.drawLine(L.x(), L.y(), R.x() + imageL.Width(), R.y(), svg::svgStyle().stroke("green", 2.0));
//                    svgStream.drawCircle(L.x(), L.y(), LL.scale(), svg::svgStyle().stroke("yellow", 2.0));
//                    svgStream.drawCircle(R.x() + imageL.Width(), R.y(), RR.scale(), svg::svgStyle().stroke("yellow", 2.0));
//                }
//                const std::string out_filename = QString::number(img + 1).toStdString() + ".svg";
//                std::ofstream svgFile(out_filename.c_str());
//                svgFile << svgStream.closeSvgFile().str();
//                svgFile.close();

//                //获取另一相机位置
//                cameraPoses[img + 1] = relativePose_info.relativePose;
//                cameraOWNposes.push_back(cameraPoses[img + 1].center());
//                //计算投影矩阵  3x4
//                P_From_KRt(K, cameraPoses[img].rotation(), cameraPoses[img].translation(), &P1);
//                P_From_KRt(K, cameraPoses[img + 1].rotation(), cameraPoses[img + 1].translation(), &P2);
//                Mat34 PLeft, PRight;//左右视图投影矩阵
//                PLeft = P1;
//                PRight = P2;
//                //this->F = F_from_P(P1, P2);

//                for (size_t i = 0; i < relativePose_info.vec_inliers.size(); ++i)
//                {
//                    const SIOPointFeature & LL = regionsL->Features()[vec_PutativeMatches[relativePose_info.vec_inliers[i]].i_];
//                    const SIOPointFeature & RR = regionsR->Features()[vec_PutativeMatches[relativePose_info.vec_inliers[i]].j_];
//                    //三角定位求解空间点坐标
//                    Vec3 X;
//                    openMVG::TriangulateDLT(P1, LL.coords().cast<double>(), P2, RR.coords().cast<double>(), &X);

//                    //投影点在相机位置后,没实现20181210
//                    //                        if (cameraPoses[img].depth(X) < 0 && cameraPoses[img + 1].depth(X) < 0)
//                    //                            continue;
//                    std::vector<Point2f> imgLpoints;//左视图匹配点
//                    std::vector<Point2f> imgRpoints;//右视图匹配点
//                    if (i<6)
//                    {
//                        imgLpoints.push_back(Point2f(LL.coords().x(), LL.coords().y()));
//                        imgRpoints.push_back(Point2f(RR.coords().x(), RR.coords().y()));
//                    }
//                    Vec3 mRGB;
//                    RGBColor colorL = rgb_imageL(LL.y(), LL.x());
//                    RGBColor colorR = rgb_imageR(RR.y(), RR.x());
//                    mRGB.x() = (colorL.r() + colorR.r()) / 2;
//                    mRGB.y() = (colorL.g() + colorR.g()) / 2;
//                    mRGB.z() = (colorL.b() + colorR.b()) / 2;
//                    vec_3DPointsRGB.push_back(mRGB);


//                    vec_3DPoints.emplace_back(X);
//                }
//            }
//        }

//        // 导出稀疏点云(相机位置点 + 对象三维点)
//        std::vector<Vec3> vec_camPos;
//        for (int m = 0; m < cameraPoses.size(); m++)
//        {
//            vec_camPos.push_back(cameraPoses[m].center());
//        }
//        //20181210
//        //            if (!exportToPcd(vec_3DPoints, vec_3DPointsRGB, vec_camPos, mfilename +".pcd"))
//        //            {
//        //                QMessageBox::information(this, QStringLiteral("提示"), QStringLiteral("输出稀疏点云出错！"));
//        //                return;
//        //            }
//        /*exportToPly(vec_3DPoints, vec_3DPointsRGB, vec_camPos, "cloud.ply");
//            */

//        ui->textLog->append("\n" + QStringLiteral("共提取")
//                            + QString::number(vec_3DPoints.size()) + QStringLiteral("个空间点"));
//        ui->textLog->append("\n" + QStringLiteral("左视图投影矩阵：") + "\n  "
//                            + QString::number(P1(0, 0)) + "," + QString::number(P1(0, 1)) + "," + QString::number(P1(0, 2)) + "," + QString::number(P1(0, 3)) + "\n  "
//                            + QString::number(P1(1, 0)) + "," + QString::number(P1(1, 1)) + "," + QString::number(P1(1, 2)) + "," + QString::number(P1(1, 3)) + "\n  "
//                            + QString::number(P1(2, 0)) + "," + QString::number(P1(2, 1)) + "," + QString::number(P1(2, 2)) + "," + QString::number(P1(2, 3)));
//        ui->textLog->append("\n" + QStringLiteral("右视图投影矩阵：") + "\n  "
//                            + QString::number(P2(0, 0)) + "," + QString::number(P2(0, 1)) + "," + QString::number(P2(0, 2)) + "," + QString::number(P2(0, 3)) + "\n  "
//                            + QString::number(P2(1, 0)) + "," + QString::number(P2(1, 1)) + "," + QString::number(P2(1, 2)) + "," + QString::number(P2(1, 3)) + "\n  "
//                            + QString::number(P2(2, 0)) + "," + QString::number(P2(2, 1)) + "," + QString::number(P2(2, 2)) + "," + QString::number(P2(2, 3)));

//        qApp->processEvents();
//    }
//    catch(Exception e){
//        QMessageBox::warning(this,"warning","idiot",
//                             QMessageBox::Ok,
//                             QMessageBox::Cancel);
//    }
}
