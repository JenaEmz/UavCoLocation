#include "ImageGrabber.h"

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