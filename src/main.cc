#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "CoLocalSystem.h"
class ImageGrabber
{
  public:
    ImageGrabber(CoLocalSystem *coLocal) : mcoLocal(coLocal) {}
    void OpencvStereo(cv::Mat &Left, cv::Mat &Right);          //自己的
    cv::Mat OpencvStereo_Other(cv::Mat &Left, cv::Mat &Right); //第二架的
    void RosStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight);
    CoLocalSystem *mcoLocal;
    cv::Mat M1l, M2l, M1r, M2r;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_formation_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    std::string config_file;
    private_nh.param("config_file_location", config_file, std::string(""));
    std::string pic_path;
    private_nh.param("pic_file_location", pic_path,std::string(""));
    int use_ros_sub = 0;
    private_nh.param("use_ros_sub", use_ros_sub,0);

     cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
        std::cout << "Failed to open settings file at: " << config_file << endl;
        exit(-1);
    }
    CoLocalSystem coLocal(fsSettings);
    ImageGrabber igb(&coLocal);

    // Load settings related to stereo calibration
    {
        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, igb.M1l, igb.M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, igb.M1r, igb.M2r);
    }
    if (use_ros_sub)
    {
        message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
        message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
        message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
        sync.registerCallback(boost::bind(&ImageGrabber::RosStereo, &igb, _1, _2));
    }
    else
    {
        cv::Mat left0(cv::imread(pic_path+"left1.jpg", 0));
        cv::Mat right0(cv::imread(pic_path+"right1.jpg", 0));
        cout<<pic_path<<endl;
        igb.OpencvStereo(left0,right0);
    }

    while(1)
    {

    }
    return 0;
}
void ImageGrabber::RosStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat imLeft, imRight;
    cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
    cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
}

void ImageGrabber::OpencvStereo(cv::Mat &Left, cv::Mat &Right)
{
    //cv::remap(Left, Left, M1l, M2l, cv::INTER_LINEAR);
    //cv::remap(Right, Right, M1r, M2r, cv::INTER_LINEAR);
    mcoLocal->TrackStereo(Left,Right,0);
}
cv::Mat ImageGrabber::OpencvStereo_Other(cv::Mat &Left, cv::Mat &Right)
{
    cv::remap(Left, Left, M1l, M2l, cv::INTER_LINEAR);
    cv::remap(Right, Right, M1r, M2r, cv::INTER_LINEAR);
    mcoLocal->TrackStereo(Left,Right,1);
    return cv::Mat();
}