#include <stdlib.h>
#include <iostream>
#include <string>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <boost/thread.hpp>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#define READIMAGE_ONLY
#define RIGHT 2
#define LEFT 1
#ifndef READIMAGE_ONLY
#endif

using namespace cv;
using namespace std;

int threshold_y = 330;
int step = 0;
int flag_step = 0;
double pi = 3.1415926;

int num = 0;

double pos_x = 0;
double pos_y = 0;
double pos_x_temp = 0;
double pos_y_temp = 0;
double angle = 0;
geometry_msgs::Twist msg;
int flag2 = 0;
int flag1 = 0;
int X0 = 336;
int num1 = 0;
int flag_cacu = 0;

int flag = 0;
int t = 0;
Mat door;
Mat door_TR;
Point2d point1(92, 343);
Point2d point2(309, 126);
Point2d point3(385, 126);
Point2d point4(638, 343);

double x_a;
double y_a;
double x_b;
double y_b;

double x_A;
double y_A;
double x_B;
double y_B;
double x_C;
double y_C;
double x_D;
double y_D;

double x_a_temp;
double y_a_temp;
double x_b_temp;
double y_b_temp;

double K = 90;
double xo = 247;
double yo = 363;

double L;
double theta;
double beta;

typedef struct center
{
    int x = 0;
    int y = 0;
} Center;

void color_thresh(Mat input);
void HSV_threshold(Mat H, Mat S, Mat V, Mat dst, int H_L, int H_H, int S_L, int S_H, int V_L, int V_H);
void color_detect(Mat input, Center *center_left, Center *center_right, int *Pixel_left, int *Pixel_right, int H_L, int H_H, int S_L, int S_H, int V_L, int V_H);
void imu_angle_callback(const std_msgs::Float32::ConstPtr &msg);
void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);
void imu_odom_callback(const std_msgs::Float32::ConstPtr &imu, const nav_msgs::Odometry::ConstPtr &odom);
bool pt_transform(Mat &img, Mat &dst, Point2d P1, Point2d P2, Point2d P3, Point2d P4);
void map_transform(int xa, int ya, int xb, int yb);
void calculate_motion_param(void);
void run(double x, double z)
{
    msg.linear.x = x;
    msg.angular.z = z;
}

int main(int argc, char **argv)
{
    ROS_DEBUG("vision_nav start");
    ros::init(argc, argv, "vision_nav_node"); //初始化ROS节点
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    ros::Subscriber sub_odom = n.subscribe("/odom", 1, odom_callback);
    ros::Subscriber sub_imu = n.subscribe("/imu_angle", 1, imu_angle_callback);
    ros::Rate loop_rate(1000);

    VideoCapture capture;
    capture.open(0);

    if (!capture.isOpened())
    {
        ROS_ERROR("unable to open camera");
        return -1;
    }

    Mat src;
    while (ros::ok())
    {
        capture.read(src);
        if (src.empty())
        {
            break;
        }
        else
        {
            imshow("src", src);
            waitKey(1);
            continue;
        }
        Rect rect(0, 0, 672, 376);
        //转HSV
        Mat src_HSV;
        //cvtColor(src, src_HSV, COLOR_RGB2HSV);

        //test picture
        //Mat test_src = src(rect);
        Mat test_src = imread("/home/dango/dashgo_ws/src/pass_door/door");
        Mat test_HSV;

        CvSize my_size;
        my_size.width = 700;
        my_size.height = 600;
        Mat test_src1;
        //cvtColor(test_src, test_HSV, COLOR_RGB2HSV);

        //色域分割，寻找阈值
        Mat color_cut = imread("/home/zdh/dashgo_ws/src/parking/rest_HSV.png");
        //color_thresh(color_cut);
        int H_L = 117;
        int H_H = 126;
        int S_L = 148;
        int S_H = 255;
        int V_L = 28;
        int V_H = 255;

        int deta_x = 0;
        double z = 0.0;
        double y = 0.0;

        //目标颜色检测,计算得到两个矩形框内的像素点个数，中心点坐标

        //结构体Center，包含x、y
        Center center_left, center_right;
        //矩形框内像素点个数
        int Pixel_left = 0;
        int Pixel_right = 0;

        //进行识别，并计算得中心点坐标和像素点个数
        Mat test_2333 = imread("/home/zdh/dashgo_ws/src/parking/2333.png");
        if (num1 >= 35)
        {
            flag1 = 1;
        }
        else
        {
            color_detect(test_src, &center_left, &center_right, &Pixel_left, &Pixel_right, H_L, H_H, S_L, S_H, V_L, V_H);
        }

        if (flag1 == 1 && flag_step == 0 && flag2 == 0)
        {
            color_detect(test_src, &center_left, &center_right, &Pixel_left, &Pixel_right, H_L, H_H, S_L, S_H, V_L, V_H);
            pt_transform(test_src, door_TR, point1, point2, point3, point4);
            cout << "x_a = " << x_a << endl;
            cout << "y_a = " << y_a << endl;
            cout << "x_b = " << x_b << endl;
            cout << "y_b = " << y_b << endl;

            flag2 = 1;
        }
        else if (flag1 == 1 && flag_step == 3)
        {
            color_detect(test_src, &center_left, &center_right, &Pixel_left, &Pixel_right, H_L, H_H, S_L, S_H, V_L, V_H);
        }

        //print出一次结果
        if (num1 == 36)
        {
            printf("center_left: (%d,%d) \n", center_left.x, center_left.y);
            printf("center_right: (%d,%d) \n", center_right.x, center_right.y);
            printf("number_left: %d \n", Pixel_left);
            printf("number_right:%d \n", Pixel_right);
        }

        msg.linear.x = 0;
        msg.angular.z = 0;

        //运动决策
        if (num1 >= 36)
        {
            //运动决策
            //定义执行时间

            //换算得到柱子的距离和相对角度
            double center_x = (center_left.x + center_right.x) / 2.0;
            double center_y = (center_left.y + center_right.y) / 2.0;
            //cout<<"center_x="<<center_x<<endl;
            //cout<<"center_y="<<center_y<<endl;
            map_transform(x_a, y_a, x_b, y_b);

            cout << "x_A = " << x_A << endl;
            cout << "y_A = " << y_A << endl;
            cout << "x_B = " << x_B << endl;
            cout << "y_B = " << y_B << endl;

            cout << "x_a = " << x_a << endl;
            cout << "y_a = " << y_a << endl;
            cout << "x_b = " << x_b << endl;
            cout << "y_b = " << y_b << endl;

            calculate_motion_param();
            cout << "L=" << L << endl;
            cout << "theta=" << theta << endl;
            cout << "beta=" << beta << endl;

            if (flag_step == 0)
            {
                //转到垂直中垂线
                /*
                if(flag_cacu == 0)
                {
                    calculate_motion_param();
                    flag_cacu = 1;
                }
                 */

                if (abs(angle + theta) > 0.05)
                {
                    //右转
                    if (theta > 0)
                    {
                        run(0, -abs(angle + theta));
                        pub.publish(msg);
                    }
                    //左转
                    if (theta < 0)
                    {
                        run(0, abs(angle + theta));
                        pub.publish(msg);
                    }
                }
                else
                {
                    flag_step = 1;
                }
            }

            if (flag_step == 1)
            {
                if (sqrt(pos_x * pos_x + pos_y * pos_y) <= L)
                {
                    run(0.3, 0);
                    pub.publish(msg);
                }
                else
                {
                    flag_step = 2;
                    sleep(1);
                }
                cout << "pos_x = " << pos_x << endl;
                cout << "pos_y = " << pos_y << endl;
            }

            if (flag_step == 2)
            {
                cout << "angle = " << angle << endl;
                cout << "beta = " << beta << endl;
                cout << "angle + beta = " << angle + beta << endl;
                if (abs(angle + beta) > 0.05)
                {
                    if (theta > 0)
                    {
                        run(0, abs(angle + beta));
                        pub.publish(msg);
                    }
                    else
                    {
                        run(0, -abs(angle + beta));
                        pub.publish(msg);
                    }
                }
                else
                {
                    flag_step = 3;
                    sleep(1);
                }
            }

            if (flag_step == 3)
            {
                cout << "distance = " << sqrt(pow(pos_y + x_D, 2) + pow(pos_x - y_D, 2)) << endl;
                cout << "pos_x = " << pos_x << endl;
                cout << "pos_y = " << pos_y << endl;
                cout << "x_D = " << x_D << endl;
                cout << "y_D = " << y_D << endl;

                if (sqrt(pow(pos_y + x_D, 2) + pow(pos_x - y_D, 2)) < 0.5 && flag == 0)
                {
                    pos_x_temp = pos_x;
                    pos_y_temp = pos_y;
                    flag = 1;
                }

                if (flag == 1)
                {
                    cout << "center_y=" << center_y << endl;
                    if (sqrt(pow(pos_x - pos_x_temp, 2) + pow(pos_y - pos_y_temp, 2)) > 1.5)
                    {
                        break;
                    }
                    run(0.2, 0);
                }
                else
                {
                    deta_x = X0 - center_x;
                    run(0.2, 0.005 * deta_x);
                }
                pub.publish(msg);
            }
        }

        waitKey(5);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void HSV_threshold(Mat H, Mat S, Mat V, Mat dst, int H_L, int H_H, int S_L, int S_H, int V_L, int V_H)
{
    int row = H.rows;
    int col = H.cols;
    int flag = 0;
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            int ele_H = H.at<uchar>(i, j);
            int ele_S = S.at<uchar>(i, j);
            int ele_V = V.at<uchar>(i, j);
            if ((ele_H > H_H || ele_H < H_L) || (ele_S > S_H || ele_S < S_L) || (ele_V > V_H || ele_V < V_L))
            {
                dst.at<uchar>(i, j) = 1;
            }
            else
            {
                dst.at<uchar>(i, j) = 255;
            }
            if (dst.at<uchar>(i, j) != 255)
            {
                dst.at<uchar>(i, j) = 0;
            }
        }
    }
    imshow("111111", dst);
}

void color_thresh(Mat input)
{
    int pos1 = 255;
    int pos2 = 0;
    int pos3 = 255;
    int pos4 = 0;
    int pos5 = 255;
    int pos6 = 0;
    int H_H = 255;
    int H_L = 0;
    int S_H = 255;
    int S_L = 0;
    int V_H = 255;
    int V_L = 0;
    Mat HSV[3];
    split(input, HSV);
    Mat output1 = HSV[0].clone();
    Mat output2 = HSV[1].clone();
    Mat output3 = HSV[2].clone();
    Mat res = HSV[0].clone();
    namedWindow("res");
    createTrackbar("H_H", "res", &pos1, 255);
    createTrackbar("H_L", "res", &pos2, 255);
    createTrackbar("S_H", "res", &pos3, 255);
    createTrackbar("S_L", "res", &pos4, 255);
    createTrackbar("V_H", "res", &pos5, 255);
    createTrackbar("V_L", "res", &pos6, 255);
    while (true)
    {
        H_H = getTrackbarPos("H_H", "res");
        H_L = getTrackbarPos("H_L", "res");
        S_H = getTrackbarPos("S_H", "res");
        S_L = getTrackbarPos("S_L", "res");
        V_H = getTrackbarPos("V_H", "res");
        V_L = getTrackbarPos("V_L", "res");
        HSV_threshold(HSV[0], HSV[1], HSV[2], res, H_L, H_H, S_L, S_H, V_L, V_H);
        CvSize my_size;
        my_size.width = 700;
        my_size.height = 600;
        Mat res1;
        resize(res, res1, my_size, CV_INTER_CUBIC);
        imshow("res", res1);
        waitKey(1);
    }
}

void color_detect(Mat input, Center *center_left, Center *center_right, int *Pixel_left, int *Pixel_right, int H_L, int H_H, int S_L, int S_H, int V_L, int V_H)
{
    Mat input_HSV;
    cvtColor(input, input_HSV, COLOR_RGB2HSV);
    Mat HSV[3];
    split(input_HSV, HSV);
    Mat H = HSV[0].clone();
    Mat S = HSV[1].clone();
    Mat V = HSV[2].clone();
    Mat res = HSV[0].clone();

    HSV_threshold(HSV[0], HSV[1], HSV[2], res, H_L, H_H, S_L, S_H, V_L, V_H);

    Mat S_temp = S.clone();
    Mat res_temp = res.clone();
    Mat res_canny;
    Canny(res, res_canny, 50, 150);

    vector< vector<Point> > contours_S;
    vector<Vec4i> hierarchy_S;
    if (num1 <= 30)
    {
        findContours(H, contours_S, hierarchy_S, CV_RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
    }
    else
    {
        findContours(res, contours_S, hierarchy_S, CV_RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
    }

    Mat S_Contours = Mat::zeros(S.size(), CV_8UC1);
    Mat Contours = Mat::zeros(S.size(), CV_8UC1);
    for (int i = 0; i < contours_S.size(); i++)
    {
        for (int j = 0; j < contours_S[i].size(); j++)
        {
            Point P = Point(contours_S[i][j].x, contours_S[i][j].y);
            Contours.at<uchar>(P) = 255;
        }
        drawContours(S_Contours, contours_S, i, Scalar(255), 1, 8, hierarchy_S);
    }
    num1 += 1;

    vector< vector<Point> > contours_poly(contours_S.size());
    vector<Rect> boundRect(contours_S.size());
    vector<Point2f> center(contours_S.size());
    vector<float> radius(contours_S.size());

    for (int i = 0; i < contours_S.size(); i++)
    {
        boundRect[i] = boundingRect(Mat(contours_S[i]));
    }

    for (int i = 0; i < contours_S.size(); i++)
    {
        rectangle(S_Contours, boundRect[i].tl(), boundRect[i].br(), Scalar(255), 1, 8, 0);
    }

    int index_second = 0;
    int index_max = 0;
    double Area_max = 0;
    double Area_second = 0;
    for (int i = 0; i < contours_S.size(); i++)
    {
        if (boundRect[i].area() > Area_max)
        {
            index_second = index_max;
            Area_second = Area_max;
            index_max = i;
            Area_max = boundRect[i].area();
        }
        else if (boundRect[i].area() > Area_second)
        {
            index_second = i;
            Area_second = boundRect[i].area();
        }
    }
    //rectangle(S_Contours,boundRect[index].tl(),boundRect[index].br(),Scalar(255),1,8,0);
    rectangle(input, boundRect[index_second].tl(), boundRect[index_second].br(), Scalar(255, 0, 255), 3, 8, 0);
    rectangle(input, boundRect[index_max].tl(), boundRect[index_max].br(), Scalar(255, 0, 255), 3, 8, 0);

    if (boundRect[index_second].tl().x < boundRect[index_max].tl().x)
    {
        center_left->x = (boundRect[index_second].tl().x + boundRect[index_second].br().x) / 2;
        center_left->y = (boundRect[index_second].tl().y + boundRect[index_second].br().y) / 2;
        center_right->x = (boundRect[index_max].tl().x + boundRect[index_max].br().x) / 2;
        center_right->y = (boundRect[index_max].tl().y + boundRect[index_max].br().y) / 2;
        *Pixel_left = boundRect[index_second].area();
        *Pixel_right = boundRect[index_max].area();
        x_a_temp = center_left->x;
        y_a_temp = boundRect[index_second].br().y;
        x_b_temp = center_right->x;
        y_b_temp = boundRect[index_max].br().y;
    }
    else
    {
        center_right->x = (boundRect[index_second].tl().x + boundRect[index_second].br().x) / 2;
        center_right->y = (boundRect[index_second].tl().y + boundRect[index_second].br().y) / 2;
        center_left->x = (boundRect[index_max].tl().x + boundRect[index_max].br().x) / 2;
        center_left->y = (boundRect[index_max].tl().y + boundRect[index_max].br().y) / 2;
        *Pixel_right = boundRect[index_second].area();
        *Pixel_left = boundRect[index_max].area();
        x_a_temp = center_left->x;
        y_a_temp = boundRect[index_max].br().y;
        x_b_temp = center_right->x;
        y_b_temp = boundRect[index_second].br().y;
    }

    imshow("result", input);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    pos_x = odom->pose.pose.position.x;
    pos_y = odom->pose.pose.position.y;
    //cout<<"pos_x = "<<pos_x<<endl;
    //cout<<"pos_y = "<<pos_y<<endl;
}

void imu_angle_callback(const std_msgs::Float32::ConstPtr &msg)
{
    angle = msg->data * 2.0;
}

bool pt_transform(Mat &img, Mat &dst, Point2d P1, Point2d P2, Point2d P3, Point2d P4)
{
    if (img.data)
    {
        Point2f corners[4];       //原图四个点
        Point2f corners_trans[4]; //逆透视图四个点

        //**车载场景图象的其他参数**//
        float roi_x0 = 0;
        float roi_y0 = 228;
        float ROI_HEIGHT = 30000;
        float ROI_WIDTH = 6000;
        //************************//

        corners[0] = P2;
        corners[1] = P3;
        corners[2] = P1;
        corners[3] = P4;

        //设定逆透视图的宽度
        float IPM_WIDTH = 500;
        float N = 7;
        //保证逆透视图的宽度大概为N个车头宽
        float sacale = (IPM_WIDTH / N) / ROI_WIDTH;
        float IPM_HEIGHT = ROI_HEIGHT * sacale;

        //逆透视图初始化
        dst = Mat::zeros(IPM_HEIGHT + 50, IPM_WIDTH, img.type());

        corners_trans[0] = Point2f(IPM_WIDTH / 2 - IPM_WIDTH / (2 * N), 0);          //P2
        corners_trans[1] = Point2f(IPM_WIDTH / 2 + IPM_WIDTH / (2 * N), 0);          //P3
        corners_trans[2] = Point2f(IPM_WIDTH / 2 - IPM_WIDTH / (2 * N), IPM_HEIGHT); //P1
        corners_trans[3] = Point2f(IPM_WIDTH / 2 + IPM_WIDTH / (2 * N), IPM_HEIGHT); //P4

        //计算原图到逆透视图和逆透视图到原图的变换矩阵
        Mat warpMatrix_src2ipm = getPerspectiveTransform(corners, corners_trans);
        warpPerspective(img, dst, warpMatrix_src2ipm, dst.size());
        double a11 = warpMatrix_src2ipm.at<double>(0, 0);
        double a12 = warpMatrix_src2ipm.at<double>(1, 0);
        double a13 = warpMatrix_src2ipm.at<double>(2, 0);
        double a21 = warpMatrix_src2ipm.at<double>(0, 1);
        double a22 = warpMatrix_src2ipm.at<double>(1, 1);
        double a23 = warpMatrix_src2ipm.at<double>(2, 1);
        double a31 = warpMatrix_src2ipm.at<double>(0, 2);
        double a32 = warpMatrix_src2ipm.at<double>(1, 2);
        double a33 = warpMatrix_src2ipm.at<double>(2, 2);
        x_a = (a11 * x_a_temp + a21 * y_a_temp + a31) / (a13 * x_a_temp + a23 * y_a_temp + a33);
        y_a = (a12 * x_a_temp + a22 * y_a_temp + a32) / (a13 * x_a_temp + a23 * y_a_temp + a33);
        x_b = (a11 * x_b_temp + a21 * y_b_temp + a31) / (a13 * x_b_temp + a23 * y_b_temp + a33);
        y_b = (a12 * x_b_temp + a22 * y_b_temp + a32) / (a13 * x_b_temp + a23 * y_b_temp + a33);
        //标出两组点
        for (int i = 0; i < 4; i++)
            circle(img, corners[i], 5, Scalar(0, 255, 255), 4);
        for (int i = 0; i < 4; i++)
            circle(dst, corners_trans[i], 5, Scalar(0, 255, 255), 4);

        imshow("img", img);
        imshow("dst", dst);
    }
    else
    {
        cout << "NO IMAGE!!!" << endl;
        return false;
    }

    return true;
}

void map_transform(int xa, int ya, int xb, int yb)
{
    x_A = (xa - xo) / K;
    x_B = (xb - xo) / K;
    y_A = (yo - ya) / K + 0.35;
    y_B = (yo - yb) / K + 0.35;
    x_D = (x_A + x_B) / 2.0;
    y_D = (y_A + y_B) / 2.0;
}

void calculate_motion_param(void)
{
    double alpha;
    alpha = (y_B * y_B - y_A * y_A + x_B * x_B - x_A * x_A) / (2 * ((y_B - y_A) * (y_B - y_A) + (x_B - x_A) * (x_B - x_A)));
    x_C = alpha * (x_B - x_A);
    y_C = alpha * (y_B - y_A);
    L = sqrt(x_C * x_C + y_C * y_C);
    theta = atan(x_C / y_C);
    beta = atan((y_A - y_B) / (x_B - x_A));
}
