#pragma once
#include "ORBextractor.h"
#include "Frame.h"

#include <string>
#include <thread>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace std;
class Tracking
{
  private:
    ORBextractor *mpORBextractorLeft, *mpORBextractorRight;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;
    Frame mCurrentFrame;
    Frame mlastFrame;

  public:
    Tracking(const  cv::FileStorage& fSettings)
    {
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        //     |fx  0   cx|
        // K = |0   fy  cy|
        //     |0   0   1 |
        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = fx;
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;
        K.at<float>(1, 2) = cy;
        K.copyTo(mK);

        // 图像矫正系数
        // [k1 k2 p1 p2 k3]
        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        DistCoef.copyTo(mDistCoef);
        mbf = fSettings["Camera.bf"];

        // 每一帧提取的特征点数 1000
        int nFeatures = fSettings["ORBextractor.nFeatures"];
        // 图像建立金字塔时的变化尺度 1.2
        float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        // 尺度金字塔的层数 8
        int nLevels = fSettings["ORBextractor.nLevels"];
        // 提取fast特征点的默认阈值 20
        int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        // 如果默认阈值提取不出足够fast特征点，则使用最小阈值 8
        int fMinThFAST = fSettings["ORBextractor.minThFAST"];

        // tracking过程都会用到mpORBextractorLeft作为特征点提取器
        mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
        mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    }
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, int id);
};
