
#include "ImageGrabber.h"

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
        // 读第一个飞机
        cv::Mat left0(cv::imread(pic_path+"left0.jpg", 0));
        cv::Mat right0(cv::imread(pic_path+"right0.jpg", 0));
        cout << pic_path << endl;
        igb.OpencvStereo(left0,right0);
        // 读第二个飞机的图像
        cv::Mat left1(cv::imread(pic_path+"left1.jpg", 0));
        cv::Mat right1(cv::imread(pic_path+"right1.jpg", 0));
        cout<<pic_path<<endl;
        igb.OpencvStereo_Other(left1,right1);
    }


    // 这两个完了之后才能做匹配吧
    // 匹配是靠Track这个函数来做的吧
    Tracking* tracker = coLocal.GetTracker();

    while(1)
    {

    }
    return 0;
}