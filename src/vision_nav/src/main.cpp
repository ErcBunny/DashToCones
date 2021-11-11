/*

 * WORK OF HITSZ AUTO 2018 student group, LIYUAN NO.10 FLOOR 17

 * idea and architecture by LINGXU CHEN and ZIXIAN ZHAO

 * code refinement and comments by YUEQIAN LIU

 * proofreading by HEMING LUO 

*/

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

using namespace cv;
using namespace std;

// ------------------------------------------ global variables and params ------------------------------------------
int threshold_y = 330;
int step = 0;
int state_machine = 0;
double pi = 3.1415926;

double pos_x = 0; // position of Dashgo from odom feedback, world frame
double pos_y = 0;
double pos_x_temp = 0;
double pos_y_temp = 0;
double angle = 0;

geometry_msgs::Twist msg;
int pt_flag = 0;
int initial_stage_pass_flag = 0;
int X0 = 336;
int current_step = 0;
int flag_cacu = 0;

int final_approach_flag = 0;
int t = 0;

double x_a_temp; // target position, pixel frame, before pt, obtained from color_detect()
double y_a_temp;
double x_b_temp;
double y_b_temp;

double x_a; // target position, pixel frame, after pt, obtained from perpective_transform()
double y_a;
double x_b;
double y_b;

double x_A; // target and reference position，world frame, obtained from get_target_coords()
double y_A;
double x_B;
double y_B;
double x_C;
double y_C;
double x_D;
double y_D;

double K = 90; // params for get_target_coords()
double xo = 247;
double yo = 363;

double L; // distance from initial position to mid-extension point
double theta;
double beta;

int delta_x = 0;
double z = 0.0;
double y = 0.0;

typedef struct center
{
    int x = 0;
    int y = 0;
} Center;

// ------------------------------------------ function declaration (deprecated in a large main.cpp) ------------------------------------------
void HSV_threshold(Mat H, Mat S, Mat V, Mat dst, int H_L, int H_H, int S_L, int S_H, int V_L, int V_H);
void color_detect(Mat input, Center *center_left, Center *center_right, int *Pixel_left, int *Pixel_right, int H_L, int H_H, int S_L, int S_H, int V_L, int V_H);
bool perspective_transform(Mat &img, Mat &dst, Point2d P1, Point2d P2, Point2d P3, Point2d P4);
void get_target_coords(int xa, int ya, int xb, int yb);

void calculate_motion_param(void);
void run(double x, double z);

void imu_angle_callback(const std_msgs::Float32::ConstPtr &msg);
void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);

int main(int argc, char **argv)
{
    // ------------------------------------------ init ROS middleware ------------------------------------------
    ROS_INFO("*** vision_nav start ***");
    ros::init(argc, argv, "vision_nav_node");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
    ros::Subscriber sub_odom = n.subscribe("/odom", 1, odom_callback);
    ros::Subscriber sub_imu = n.subscribe("/imu_angle", 1, imu_angle_callback);
    ros::Rate loop_rate(1000);

    // ------------------------------------------ init camera ------------------------------------------
    VideoCapture capture;
    capture.open(0);

    if (!capture.isOpened())
    {
        ROS_ERROR("unable to open camera");
        return -1;
    }

    // ------------------------------------------ main loop ------------------------------------------
    Mat src;
    while (ros::ok())
    {
        // ------------------------------------------ read img and crop for ROI ------------------------------------------
        capture.read(src);
        if (src.empty())
        {
            break;
        }
        Rect ROI(0, 0, 672, 376); // extract mono img from ZED
        Mat src_mono = src(ROI);

        Center center_left, center_right; // centre coord of rect box
        int Pixel_left = 0;               // pixel number in a rect box
        int Pixel_right = 0;

        int H_L = 117; // HSV thresh for bright orange cones
        int H_H = 126;
        int S_L = 148;
        int S_H = 255;
        int V_L = 28;
        int V_H = 255;

        // ------------------------------------------ skip color glitches ------------------------------------------
        if (current_step < 35)
        {
            ROS_INFO("*** initial detection ***");
            color_detect(src_mono, &center_left, &center_right, &Pixel_left, &Pixel_right, H_L, H_H, S_L, S_H, V_L, V_H);
        }
        else
        {
            initial_stage_pass_flag = 1;
        }

        // ------------------------------------------ detect and perspective transform ------------------------------------------
        if (initial_stage_pass_flag == 1 && state_machine == 0 && pt_flag == 0)
        {
            color_detect(src_mono, &center_left, &center_right, &Pixel_left, &Pixel_right, H_L, H_H, S_L, S_H, V_L, V_H);

            Mat door_TR;
            Point2d point1(92, 343);
            Point2d point2(309, 126);
            Point2d point3(385, 126);
            Point2d point4(638, 343);
            perspective_transform(src_mono, door_TR, point1, point2, point3, point4);

            ROS_INFO("*** perspective transform ***");
            ROS_INFO("x_a = %f", x_a);
            ROS_INFO("y_a = %f", y_a);
            ROS_INFO("x_b = %f", x_b);
            ROS_INFO("y_b = %f", y_b);

            pt_flag = 1;
        }
        // ------------------------------------------ visual feedback at the final approach state ------------------------------------------
        else if (initial_stage_pass_flag == 1 && state_machine == 3)
        {
            color_detect(src_mono, &center_left, &center_right, &Pixel_left, &Pixel_right, H_L, H_H, S_L, S_H, V_L, V_H);
        }

        // ------------------------------------------ print detection result ------------------------------------------
        if (current_step == 36)
        {
            ROS_INFO("*** rect info ***");
            ROS_INFO("center_left\t= (%d,%d)", center_left.x, center_left.y);
            ROS_INFO("center_right\t= (%d,%d)", center_right.x, center_right.y);
            ROS_INFO("number_left\t= %d", Pixel_left);
            ROS_INFO("number_right\t= %d", Pixel_right);
        }

        msg.linear.x = 0;
        msg.angular.z = 0;

        // ------------------------------------------ motion ------------------------------------------
        if (current_step >= 36)
        {

            // get distance and orientation relative to cones
            double center_x = (center_left.x + center_right.x) / 2.0;
            double center_y = (center_left.y + center_right.y) / 2.0;
            get_target_coords(x_a, y_a, x_b, y_b);

            ROS_INFO("*** MOTION ***");

            ROS_INFO("x_A = %f", x_A);
            ROS_INFO("y_A = %f", y_A);
            ROS_INFO("x_B = %f", x_B);
            ROS_INFO("y_B = %f", y_B);

            ROS_INFO("x_a = %f", x_a);
            ROS_INFO("y_a = %f", y_a);
            ROS_INFO("x_b = %f", x_b);
            ROS_INFO("y_b = %f", y_b);

            calculate_motion_param();
            ROS_INFO("L\t= %f", L);
            ROS_INFO("theta\t= %f", theta);
            ROS_INFO("beta\t= %f", beta);

            // turn towards mid-extension point
            if (state_machine == 0)
            {

                if (abs(angle + theta) > 0.05) // dead zone of angle deviation
                {
                    // turn right
                    if (theta > 0)
                    {
                        run(0, -abs(angle + theta));
                        pub.publish(msg);
                    }
                    // turn left
                    if (theta < 0)
                    {
                        run(0, abs(angle + theta));
                        pub.publish(msg);
                    }
                }
                else
                {
                    state_machine = 1;
                    sleep(1);
                }

                ROS_INFO("--- state machine 1: turn towards mid-extension point ---");

            }
            
            // go straight line to mid-extension point
            if (state_machine == 1)
            {
                if (sqrt(pos_x * pos_x + pos_y * pos_y) <= L + 0.25)
                {
                    run(0.3, 0);
                    pub.publish(msg);
                }
                else
                {
                    state_machine = 2;
                    sleep(1);
                }
                ROS_INFO("--- state machine 1: go straight line to mid-extension point ---");
                ROS_INFO("pos_x = %f", pos_x);
                ROS_INFO("pos_y = %f", pos_y);
            }

            // turn towards mid point between the cones
            if (state_machine == 2)
            {
                ROS_INFO("--- state machine 2: turn towards mid point between the cones ---");
                ROS_INFO("angle\t= %f", angle);
                ROS_INFO("beta\t= %f", beta);
                ROS_INFO("sum\t= %f", angle + beta);

                if (abs(angle + beta) > 0.1)
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
                    state_machine = 3;
                    sleep(1);
                }
            }
            
            // go through the cones with visual feedback
            if (state_machine == 3)
            {
                ROS_INFO("--- state machine 3: go through the cones with visual feedback ---");
                ROS_INFO("distance\t= %f", sqrt(pow(pos_y + x_D, 2) + pow(pos_x - y_D, 2)));
                ROS_INFO("pos_x\t= %f", pos_x);
                ROS_INFO("pos_y\t= %f", pos_y);
                ROS_INFO("x_D\t= %f", x_D);
                ROS_INFO("y_D\t= %f", y_D);

                if(final_approach_flag != 1)
                {
                    delta_x = X0 - center_x; // X0=336=center of the pixel frame, center_x is updated from color_detect()
                    run(0.2, 0.01 * delta_x); // angular velocity feedback
                }
                else // in final approach mode, go blind without feedback for 2m then stop
                {
                    if (sqrt(pow(pos_x - pos_x_temp, 2) + pow(pos_y - pos_y_temp, 2)) > 2)
                    {
                        break;
                    }
                    run(0.2, 0);
                }

                // if is close enough to the mid point D, enter final approach mode, the point of transition is marked as temp
                if (sqrt(pow(pos_y + x_D, 2) + pow(pos_x - y_D, 2)) < 0.5 && final_approach_flag == 0)
                {
                    pos_x_temp = pos_x;
                    pos_y_temp = pos_y;
                    final_approach_flag = 1;
                }

                pub.publish(msg);
            }
        }

        waitKey(1);
        ROS_INFO("-------------------- loop end --------------------");
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

// ------------------------------------------ function implementation below ------------------------------------------
void HSV_threshold(Mat H, Mat S, Mat V, Mat dst, int H_L, int H_H, int S_L, int S_H, int V_L, int V_H)
{
    int row = H.rows;
    int col = H.cols;
    int final_approach_flag = 0;
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
    imshow("binary", dst);
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
    if (current_step <= 30)
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
    current_step += 1;

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
    rectangle(input, boundRect[index_second].tl(), boundRect[index_second].br(), Scalar(0, 255, 0));
    rectangle(input, boundRect[index_max].tl(), boundRect[index_max].br(), Scalar(0, 255, 0));

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

    imshow("detection", input);
    waitKey(1);
}

bool perspective_transform(Mat &img, Mat &dst, Point2d P1, Point2d P2, Point2d P3, Point2d P4)
{
    if (img.data)
    {
        Point2f corners[4];       // 4 pts in src img
        Point2f corners_trans[4]; // 4 pts in inverse perspective map

        // params of the tranform (depend on robot hardware)
        float roi_x0 = 0;
        float roi_y0 = 228;
        float ROI_HEIGHT = 30000;
        float ROI_WIDTH = 6000;

        corners[0] = P2;
        corners[1] = P3;
        corners[2] = P1;
        corners[3] = P4;

        // set width of the inverse perpective img
        float IPM_WIDTH = 500;
        float N = 7;

        // assure that the IPM width is about N times the width of robot head width
        float scale = (IPM_WIDTH / N) / ROI_WIDTH;
        float IPM_HEIGHT = ROI_HEIGHT * scale;

        // init transform
        dst = Mat::zeros(IPM_HEIGHT + 50, IPM_WIDTH, img.type());

        corners_trans[0] = Point2f(IPM_WIDTH / 2 - IPM_WIDTH / (2 * N), 0);          //P2
        corners_trans[1] = Point2f(IPM_WIDTH / 2 + IPM_WIDTH / (2 * N), 0);          //P3
        corners_trans[2] = Point2f(IPM_WIDTH / 2 - IPM_WIDTH / (2 * N), IPM_HEIGHT); //P1
        corners_trans[3] = Point2f(IPM_WIDTH / 2 + IPM_WIDTH / (2 * N), IPM_HEIGHT); //P4

        // calculate transform matrix
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

        // mark the pts on img
        for (int i = 0; i < 4; i++)
            circle(img, corners[i], 5, Scalar(0, 255, 0), -1);
        for (int i = 0; i < 4; i++)
            circle(dst, corners_trans[i], 5, Scalar(0, 255, 0), -1);

        imshow("box & anchor", img);
        imshow("transform result", dst);
    }
    else
    {
        ROS_WARN("NO IMAGE");
        return false;
    }

    return true;
}

void get_target_coords(int xa, int ya, int xb, int yb)
{
    x_A = (xa - xo) / K;
    y_A = (yo - ya) / K + 0.35;

    x_B = (xb - xo) / K;
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

void run(double x, double z)
{
    msg.linear.x = x;
    msg.angular.z = z;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    pos_x = odom->pose.pose.position.x;
    pos_y = odom->pose.pose.position.y;
}

void imu_angle_callback(const std_msgs::Float32::ConstPtr &msg)
{
    angle = msg->data * 2.0;
}