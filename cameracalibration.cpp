#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "cameracalibration.h"
#include <fstream>
#include <iostream>
using namespace std;
bool refineMatchesWithHomography(const std::vector<cv::KeyPoint>& queryKeypoints, const std::vector<cv::KeyPoint>& trainKeypoints,
                                 float reprojectionThreshold, std::vector<cv::DMatch>& matches, cv::Mat& homography);
void cameraCalibration_triggered(QStringList files,Size square_size,Size board_size)
{
    ofstream fout("相机定标结果.txt");  /* 保存标定结果的文件 */
    // 读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化

    int image_count = 0;  /* 图像数量 */
    Size image_size;      /* 图像的尺寸 */

    vector<Point2f> image_points_buf;         /* 缓存每幅图像上检测到的角点 */
    vector<vector<Point2f> > image_points_seq; /* 保存检测到的所有角点 */

    for (int m =0;m<files.count();m++)
    {
        image_count++;
        QString fileName = files[m];
        cv::Mat imageInput = imread(fileName.toStdString());
        // 读入第一张图片时获取图片大小
        if(image_count == 1)
        {
            image_size.width = imageInput.cols;
            image_size.height = imageInput.rows;
        }
        /* 提取角点 需要使用findChessboardCorners函数提取角点 */
        /********
      第一个参数Image，传入拍摄的棋盘图Mat图像，必须是8位的灰度或者彩色图像；(imageInput)
      第二个参数patternSize，每个棋盘图上内角点的行列数，一般情况下，行列数不要相同，便于后续标定程序识别标定板的方向；(board_size)
      第三个参数corners，用于存储检测到的内角点图像坐标位置，一般用元素是Point2f的向量来表示：vector<Point2f> image_points_buf;(image_points_buf)
      第四个参数flage：用于定义棋盘图上内角点查找的不同处理方式，有默认值(用了默认值)
        **********************/
        if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
        {
            //ui->textLog->append(u8"找不到角点!\n");  // 找不到角点
            exit(1);
        }
        else
        {
            cv::Mat view_gray;
            cvtColor(imageInput, view_gray, CV_RGB2GRAY);  // 转灰度图
            /* 亚像素精确化 */
            // image_points_buf 初始的角点坐标向量，同时作为亚像素坐标位置的输出
            // Size(5,5) 搜索窗口大小
            // （-1，-1）表示没有死区
            // TermCriteria 角点的迭代过程的终止条件, 可以为迭代次数和角点精度两者的组合
            cornerSubPix(view_gray, image_points_buf, Size(5,5), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 20, 0.1));
            image_points_seq.push_back(image_points_buf);  // 保存亚像素角点
            /* 在图像上显示角点位置 */
            drawChessboardCorners(view_gray, board_size, image_points_buf, false); // 用于在图片中标记角点
            //imshow("Camera Calibration", view_gray);       // 显示图片
            //waitKey(500); //暂停0.5S
        }
    }
    int CornerNum = board_size.width * board_size.height;  // 每张图片上总的角点数
    //-------------以下是摄像机标定------------------
    /*棋盘三维信息*/

    vector<vector<Point3f> > object_points;   /* 保存标定板上角点的三维坐标 */

    /*内外参数*/
    cv::Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));  /* 摄像机内参数矩阵 */
    vector<int> point_counts;   // 每幅图像中角点的数量
    cv::Mat distCoeffs=Mat(1, 5, CV_32FC1,Scalar::all(0));       /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
    vector<cv::Mat> tvecsMat;      /* 每幅图像的旋转向量 */
    vector<cv::Mat> rvecsMat;      /* 每幅图像的平移向量 */

    /* 初始化标定板上角点的三维坐标 */
    int i, j, t;
    {
        for (t=0; t<image_count; t++) //第几张图片
        {
            vector<Point3f> tempPointSet;
            for (i=0; i<board_size.height; i++)
            {
                for (j=0; j<board_size.width; j++)
                {
                    Point3f realPoint;
                    /* 假设标定板放在世界坐标系中z=0的平面上 */
                    realPoint.x = i * square_size.width;
                    realPoint.y = j * square_size.height;
                    realPoint.z = 0;
                    tempPointSet.push_back(realPoint);
                }
            }
            object_points.push_back(tempPointSet);
        }
    }

    /* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
    for (i=0; i<image_count; i++)
    {
        point_counts.push_back(board_size.width * board_size.height);
    }
    /* 开始标定 */
    // object_points 世界坐标系中的角点的三维坐标
    // image_points_seq 每一个内角点对应的图像坐标点
    // image_size 图像的像素尺寸大小
    // cameraMatrix 输出，内参矩阵
    // distCoeffs 输出，畸变系数
    // rvecsMat 输出，旋转向量
    // tvecsMat 输出，位移向量
    // 0 标定时所采用的算法
    calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
    //------------------------标定完成------------------------------------
    // -------------------对标定结果进行评价------------------------------
    double total_err = 0.0;         /* 所有图像的平均误差的总和 */
    double err = 0.0;               /* 每幅图像的平均误差 */
    vector<Point2f> image_points2;  /* 保存重新计算得到的投影点 */
    fout<<"每幅图像的标定误差：\n";

    //    omp_set_num_threads(4);//多线程并行
    //    #pragma omp parallel for
    for (i=0;i<image_count;i++)//第几张图片
    {

        vector<Point3f> tempPointSet = object_points[i];
        /* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
        projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
        /* 计算新的投影点和旧的投影点之间的误差*/
        vector<Point2f> tempImagePoint = image_points_seq[i];
        cv::Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
        cv::Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
        for (int j = 0 ; j < tempImagePoint.size(); j++)
        {
            image_points2Mat.at<Vec2f>(0,j) = Vec2f(image_points2[j].x, image_points2[j].y);
            tempImagePointMat.at<Vec2f>(0,j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
        }
        err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
        total_err += err/= point_counts[i];
        fout << "第" << i+1 << "张图像的误差是" << err<< "个像素" << endl;
    }
    fout << "总误差为" << total_err/image_count << "个像素" <<endl <<endl;
    //-------------------------评价完成---------------------------------------------
    //-----------------------保存定标结果-------------------------------------------
    cv::Mat rotation_matrix = cv::Mat(3,3,CV_32FC1, Scalar::all(0));  /* 保存每幅图像的旋转矩阵 */
    fout << "相机内参数矩阵：" << endl;
    fout << cameraMatrix << endl << endl;
    fout << "畸变系数：\n";
    fout << distCoeffs << endl << endl << endl;
    //    for (int i=0; i<image_count; i++)
    //    {
    //        fout << "第" << i+1 << "幅图像的旋转向量：" << endl;
    //        fout << tvecsMat[i] << endl;
    //        /* 将旋转向量转换为相对应的旋转矩阵 */
    //        Rodrigues(tvecsMat[i], rotation_matrix);
    //        fout << "第" << i+1 << "幅图像的旋转矩阵：" << endl;
    //        fout << rotation_matrix << endl;
    //        fout << "第" << i+1 << "幅图像的平移向量：" << endl;
    //        fout << rvecsMat[i] << endl << endl;
    //    }
    //    fout<<endl;
    //--------------------标定结果保存结束-------------------------------
    //---------------------------------查看标定效果——利用标定结果对棋盘图进行矫正-----
    //    Mat mapx = Mat(image_size, CV_32FC1);
    //    Mat mapy = Mat(image_size, CV_32FC1);
    //    Mat R = Mat::eye(3, 3, CV_32F);
    //    string imageFileName;
    //    std::stringstream StrStm;
    //    for (int i = 0 ; i != image_count ; i++)
    //    {
    //        initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);
    //        Mat imageSource = imread(filenames[i]);
    //        Mat newimage = imageSource.clone();
    //        remap(imageSource, newimage, mapx, mapy, INTER_LINEAR);
    //        StrStm.clear();
    //        imageFileName.clear();
    //        StrStm << i+1;
    //        StrStm >> imageFileName;
    //        imageFileName += "_d.jpg";
    //        imwrite(imageFileName, newimage);
    //    }
    fout.close();



}
void SURF_triggered(QStringList files)
{
    //    omp_set_num_threads(4);//多线程并行
    //#pragma omp parallel for

    for(int m =0;m<files.count()-1;m++){
        //        QString imgObjectName = files[m];
        //        QString imgSceneName = files[m+1];
        Mat imgObject = imread(files[m].toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
        Mat imgScene = imread(files[m+1].toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
        //        imshow("object",imgObject);
        //        imshow("scene",imgScene);
        if (!imgObject.data || !imgScene.data)
        {

            //ui->textLog->append( " --(!) Error reading images " );
        }

        //double begin = clock();

        ///-- Step 1: 使用SURF算子检测特征点
        int minHessian = 400;
        //SurfFeatureDetector detector(minHessian);//opencv 2.4.x
        Ptr<SURF> detector = SURF::create();//opencv 3.x.x
        vector<KeyPoint> keypointsObject, keypointsScene;
        //detector.detect(imgObject, keypointsObject);//opencv 2.4.x
        detector->detect(imgObject, keypointsObject);//opencv 3.x.x
        detector->detect(imgScene, keypointsScene);
        //        qDebug() << "object--number of keypoints: " << keypointsObject.size() << endl;
        //        qDebug() << "scene--number of keypoints: " << keypointsScene.size() << endl;

        ///-- Step 2: 使用SURF算子提取特征（计算特征向量）
        //SurfDescriptorExtractor extractor;//opencv 2.4.x
        Ptr<SURF> extractor = SURF::create();//opencv 3.x.x
        Mat descriptorsObject, descriptorsScene;
        extractor->compute(imgObject, keypointsObject, descriptorsObject);
        extractor->compute(imgScene, keypointsScene, descriptorsScene);

        ///-- Step 3: 使用FLANN法进行匹配
        FlannBasedMatcher matcher;
        vector< DMatch > allMatches;
        matcher.match(descriptorsObject, descriptorsScene, allMatches);
        //        qDebug() << "number of matches before filtering: " << allMatches.size() << endl;

        //-- 计算关键点间的最大最小距离
        double maxDist = 0;
        double minDist = 100;
        for (int i = 0; i < descriptorsObject.rows; i++)
        {
            double dist = allMatches[i].distance;
            if (dist < minDist)
                minDist = dist;
            if (dist > maxDist)
                maxDist = dist;
        }
        //        printf("	max dist : %f \n", maxDist);
        //        printf("	min dist : %f \n", minDist);

        //-- 过滤匹配点，保留好的匹配点（这里采用的标准：distance<3*minDist）
        vector< DMatch > goodMatches;
        for (int i = 0; i < descriptorsObject.rows; i++)
        {
            if (allMatches[i].distance < 2 * minDist)
                goodMatches.push_back(allMatches[i]);
        }
        //qDebug() << "number of matches after filtering: " << goodMatches.size() << endl;

        //-- 显示匹配结果
        Mat resultImg;
        drawMatches(imgObject, keypointsObject, imgScene, keypointsScene,
                    goodMatches, resultImg, Scalar::all(-1), Scalar::all(-1), vector<char>(),
                    DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS //不显示未匹配的点
                    );
        //-- 输出匹配点的对应关系
        //        for (int i = 0; i < goodMatches.size(); i++)
        //            ui->textLog->append(QObject::tr("good match %1: keypointsObject [%2]  -- keypointsScene [%3]\n").arg(i).arg(goodMatches[i].queryIdx).arg(goodMatches[i].trainIdx));

        ///-- Step 4: 使用findHomography找出相应的透视变换//获取满足Ratio Test的最小匹配的距离KNN算法
        vector<Point2f> object;
        vector<Point2f> scene;
        for (size_t i = 0; i < goodMatches.size(); i++)
        {
            //-- 从好的匹配中获取关键点: 匹配关系是关键点间具有的一 一对应关系，可以从匹配关系获得关键点的索引
            //-- e.g. 这里的goodMatches[i].queryIdx和goodMatches[i].trainIdx是匹配中一对关键点的索引
            object.push_back(keypointsObject[goodMatches[i].queryIdx].pt);
            scene.push_back(keypointsScene[goodMatches[i].trainIdx].pt);
        }
        Mat H = findHomography(object, scene, CV_RANSAC);

        ////一种新的透视变换判断算子
        //Mat H;
        refineMatchesWithHomography(keypointsObject, keypointsScene,2.0, allMatches, H);
        ///-- Step 5: 使用perspectiveTransform映射点群，在场景中获取目标位置
        std::vector<Point2f> objCorners(4);
        objCorners[0] = cvPoint(0, 0);
        objCorners[1] = cvPoint(imgObject.cols, 0);
        objCorners[2] = cvPoint(imgObject.cols, imgObject.rows);
        objCorners[3] = cvPoint(0, imgObject.rows);
        std::vector<Point2f> sceneCorners(4);
        perspectiveTransform(objCorners, sceneCorners, H);

        //-- 在被检测到的目标四个角之间划线;

        line(resultImg, sceneCorners[0] + Point2f(imgObject.cols, 0), sceneCorners[1] + Point2f(imgObject.cols, 0), Scalar(0, 255, 0), 4);
        line(resultImg, sceneCorners[1] + Point2f(imgObject.cols, 0), sceneCorners[2] + Point2f(imgObject.cols, 0), Scalar(0, 255, 0), 4);
        line(resultImg, sceneCorners[2] + Point2f(imgObject.cols, 0), sceneCorners[3] + Point2f(imgObject.cols, 0), Scalar(0, 255, 0), 4);
        line(resultImg, sceneCorners[3] + Point2f(imgObject.cols, 0), sceneCorners[0] + Point2f(imgObject.cols, 0), Scalar(0, 255, 0), 4);

        //-- 显示检测结果
        //imshow("detection result", resultImg);
        string resultName = (QObject::tr("result%1.jpg").arg(m+2)).toStdString();
        imwrite(resultName, resultImg);
        //double end = clock();
        //qDebug() << "\nSURF--elapsed time: " << (end - begin) / CLOCKS_PER_SEC * 1000 << " ms\n";
    }
}
//寻找源图与目标图像之间的透视变换https://blog.csdn.net/hust_bochu_xuchao/article/details/52153167
bool refineMatchesWithHomography(const std::vector<cv::KeyPoint>& queryKeypoints, const std::vector<cv::KeyPoint>& trainKeypoints,
                                 float reprojectionThreshold, std::vector<cv::DMatch>& matches, cv::Mat& homography)
{
    const int minNumberMatchesAllowed = 8;
    if (matches.size() < minNumberMatchesAllowed)
        return false;
    // Prepare data for cv::findHomography
    std::vector<cv::Point2f> srcPoints(matches.size());
    std::vector<cv::Point2f> dstPoints(matches.size());
    for (size_t i = 0; i < matches.size(); i++)
    {
        srcPoints[i] = trainKeypoints[matches[i].trainIdx].pt;
        dstPoints[i] = queryKeypoints[matches[i].queryIdx].pt;
        //srcPoints[i] = trainKeypoints[i].pt;
        //dstPoints[i] = queryKeypoints[i].pt;
    }
    // Find homography matrix and get inliers mask
    std::vector<unsigned char> inliersMask(srcPoints.size());
    homography = cv::findHomography(srcPoints, dstPoints, CV_FM_RANSAC, reprojectionThreshold, inliersMask);
    std::vector<cv::DMatch> inliers;
    //    omp_set_num_threads(4);//多线程并行
    //#pragma omp parallel for
    for (size_t i = 0; i<inliersMask.size(); i++)
    {
        if (inliersMask[i])
            inliers.push_back(matches[i]);
    }
    matches.swap(inliers);
    return matches.size() > minNumberMatchesAllowed;
}
//得到匹配点后，就可以使用OpenCV3.0中新加入的函数findEssentialMat()来求取本征矩阵了。得到本征矩阵后，再使用另一个函数对本征矩阵进行分解，并返回两相机之间的相对变换R和T。

bool find_transform(Mat& K, vector<Point2f>& p1, vector<Point2f>& p2, Mat& R, Mat& T, Mat& mask)
{
    //根据内参矩阵获取相机的焦距和光心坐标（主点坐标）
    double focal_length = 0.5*(K.at<double>(0) + K.at<double>(4));
    Point2d principle_point(K.at<double>(2), K.at<double>(5));

    //根据匹配点求取本征矩阵，使用RANSAC，进一步排除失配点
    Mat E = findEssentialMat(p1, p2, focal_length, principle_point, RANSAC, 0.999, 1.0, mask);
    if (E.empty()) return false;

    double feasible_count = countNonZero(mask);
    //        qDebug() << (int)feasible_count << " -in- " << p1.size() << endl;
    //对于RANSAC而言，outlier数量大于50%时，结果是不可靠的
    if (feasible_count <= 15 || (feasible_count / p1.size()) < 0.6)
        return false;

    //分解本征矩阵，获取相对变换
    int pass_count = recoverPose(E, p1, p2, R, T, focal_length, principle_point, mask);

    //同时位于两个相机前方的点的数量要足够大
    if (((double)pass_count) / feasible_count < 0.7)
        return false;

    return true;
}
