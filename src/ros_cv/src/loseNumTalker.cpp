//-----------------------------------
//可以实现基本功能，即螺钉松动检测
//问题：二值化阈值以及范围内像素数量需手动设置
//-----------------------------------
#include<opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <fstream>
#include <iostream>
#include <sstream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_cv/LosePointMsg.h"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

//全局函数声明
int  hough_circle_detection(Mat, int *);
int lose_position_find(Mat baseImage, Mat realImage, int circle_center[][2], int *p);    //松动螺钉位置

int main(int argc, char *argv[])
{
    int loseNum = 0;            //松动螺钉数量
    int circle_center[10][2];	//中心点的参数
    int *p = circle_center[0];

    // node init
    ros::init(argc, argv, "losenumtalker");

    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<ros_cv::LosePointMsg>("chatter1", 1000);
    ros::Rate loop_rate(0.5);

    while (ros::ok())
  {
    //img process
    
    //获得两幅图像
    Mat realImage = imread("/home/jier/Pictures/11.jpg");
    Mat baseImage = imread("/home/jier/Pictures/22.jpg");
    
    //返回松动螺钉数和圆心位置
    loseNum = lose_position_find(baseImage, realImage, circle_center, p);
    //输出
    //for(int i = 0; i < loseNum; i++)
    //   cout << "松动点圆心x = " << circle_center[i][0] << "  " << "y = " << circle_center[i][1] << endl;


    //publish 
    ros_cv::LosePointMsg msg;
    //std::stringstream ss;                               
    //ss << "hello world " << count;                          //缓存到ss
    msg.num = loseNum;
    msg.point1.x = circle_center[0][0];
    msg.point1.y = circle_center[0][1];
    msg.point2.x = circle_center[1][0];
    msg.point2.y = circle_center[1][1];
    //initrd.imgmsg.data = ss.str();                                    //赋值

    //ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    ros::spinOnce();                                        //接受回调函数
    loop_rate.sleep();                                      //休眠一段时间
  }
    return 0;
}

//hough圆检测子函数
int  hough_circle_detection(Mat src, int *p)
{
    Mat gray;
    cvtColor(src, gray, CV_BGR2GRAY);
    //高斯模糊平滑
    GaussianBlur(gray, gray, Size(5, 5), 3, 3);

    vector<Vec3f> circles;
    //霍夫变换
    HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, 30, 60, 17, 0, 50);

    //在原图中画出圆心和圆
    for (int a = 0; a < circles.size(); a++)
    {
        printf("x = %d,y = %d\n", cvRound(circles[a][0]), cvRound(circles[a][1]));
        p[2*a] = cvRound(circles[a][0]);
        p[2*a+1] = cvRound(circles[a][1]);
    }
    return circles.size();
}


//螺钉松动位置检测
//return 松动数量
//baseImage 基准图像
//realImage 实际图像
int lose_position_find(Mat baseImage, Mat realImage, int circle_center[][2], int *p)
{

    int center_row;//圆心行数
    int loseNum = 0;  //松动数量
    //imshow("realImage", realImage);
    //waitKey(30);

    //imshow("baseImage", baseImage);
    //waitKey(30);

    //--------------------
    //图像配准，找到不同处
    //--------------------
    //灰度图转换
    Mat image1, image2;
    cvtColor(realImage, image1, CV_RGB2GRAY);
    cvtColor(baseImage, image2, CV_RGB2GRAY);

    //提取特征点
    Ptr<SURF> surfDetector = SURF::create(800);         //海塞矩阵阈值  SurfFeatureDetector surfDetector(800);
    vector<KeyPoint> keyPoint1, keyPoint2;
    surfDetector->detect(image1, keyPoint1);
    surfDetector->detect(image2, keyPoint2);

    //特征点描述，为下边的特征点匹配做准备
    Ptr<SURF>SurfDescriptor = SURF::create();           //SurfDescriptorExtractor SurfDescriptor;
    Mat imageDesc1, imageDesc2;
    SurfDescriptor->compute(image1, keyPoint1, imageDesc1);
    SurfDescriptor->compute(image2, keyPoint2, imageDesc2);

    //获得匹配特征点，并提取最优配对
    FlannBasedMatcher matcher;
    vector<DMatch> matchePoints;
    matcher.match(imageDesc1, imageDesc2, matchePoints, Mat());
    sort(matchePoints.begin(), matchePoints.end());     //特征点排序

    //获取排在前N个的最优匹配特征点
    vector<Point2f> imagePoints1, imagePoints2;

    for (int i = 0; i<10; i++)
    {
        imagePoints1.push_back(keyPoint1[matchePoints[i].queryIdx].pt);
        imagePoints2.push_back(keyPoint2[matchePoints[i].trainIdx].pt);
    }

    //获取图像1到图像2的投影映射矩阵 尺寸为3*3
    Mat homo = findHomography(imagePoints1, imagePoints2, CV_RANSAC);
    cout << "变换矩阵为：\n" << homo << endl << endl; //输出映射矩阵
    double adjustValue = image1.cols;
    Mat adjustMat = (Mat_<double>(3, 3) << 1.0, 0, 35, 0, 1.0, 65, 0, 0, 1.0);
    cout << "调整矩阵为：\n" << adjustMat << endl << endl;
    cout << "调整后变换矩阵为：\n" << adjustMat*homo << endl;

    //图像配准
    Mat imageTransform1, imageTransform2;
    warpPerspective(realImage, imageTransform1, homo, Size(baseImage.cols, baseImage.rows));
    warpPerspective(realImage, imageTransform2, adjustMat*homo, Size(baseImage.cols*1.3, baseImage.rows*1.8));
    //imshow("直接经过透视矩阵变换", imageTransform1);

    //基准图像和配准后的图像作差得到差异图像
    Mat g_dstImage = Mat::zeros(baseImage.size(), baseImage.type());
    for (int y = 0; y<baseImage.rows; y++)
    {
        for (int x = 0; x < baseImage.cols; x++)
        {
            for (int c = 0; c < 3; c++)
            {
                g_dstImage.at<Vec3b>(y, x)[c] = saturate_cast<uchar>(baseImage.at<Vec3b>(y, x)[c] - imageTransform1.at<Vec3b>(y, x)[c]);
            }
        }
    }
    //对差异图像灰度化
    cvtColor(g_dstImage, g_dstImage, CV_BGR2GRAY);
    // 用 3x3内核来降噪
    blur(g_dstImage, g_dstImage, Size(2, 2));
    //差异图像二值化，阈值为第三个参数
    threshold(g_dstImage, g_dstImage, 85, 255, THRESH_BINARY);
    //imshow("差值图像", g_dstImage);

    //----------------
    //对原图进行hough圆检测，得到圆心坐标
    //----------------
    center_row = hough_circle_detection(baseImage, p);//hough圆检测

    int num;
    Mat label = Mat::zeros(center_row, 1, CV_8U);//螺钉标签，变化的螺钉标1，不变的为0
    //对二值化图像进行判断，圆心上下左右20个像素范围内，如果存在10个白点，则该圆心标签置1
    for (int i = 0; i < center_row; i++)
    {
        num = 0;
        for (int m = circle_center[i][1] - 20; m <= circle_center[i][1] + 20; m++)
        {
            uchar*data = g_dstImage.ptr<uchar>(m);
            for (int n = circle_center[i][0] - 20; n <= circle_center[i][0] + 20; n++)
            {
                if (data[n] == 255)
                {
                    num = num + 1;
                }
            }
        }
        if (num >= 8)
        {
            label.at<uchar>(i, 0) = 1;
        }
    }
    cout << "label" << label << endl << endl;
    
    //将最终变化的圆心写入TXT文件
    /*
    fstream file("test.txt", ios::out);	//打开一个文件
    for (int i = 0; i < center_row; i++)
    {
        if (label.at<uchar>(i, 0) == 1)
        {
            cout << "圆心x:" << circle_center[i][0] << endl;
            cout << "圆心y:" << circle_center[i][1] << endl;
            file << circle_center[i][0] << " " << circle_center[i][1];
            file << endl;
        }
    }
    file.close();
    */
    for (int i = 0; i < center_row; i++)
    {
        if (label.at<uchar>(i, 0) == 1)                     //若某螺钉松动
        {
            int midNum = circle_center[i][0];
            circle_center[loseNum][0] = midNum;
            
            midNum = circle_center[i][1];
            circle_center[loseNum][1] = midNum;
            loseNum++;
        }
    }
    /*for(int j = loseNum; j < center_row; j++)
    {
        circle_center[j][0] = 0;
        circle_center[j][1] = 0;
    }
    */
    return loseNum;
}
