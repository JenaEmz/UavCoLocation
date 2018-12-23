#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <iostream>

#include "Tracking.h"

using namespace std;
class CoLocalSystem
{
private:
    cv::FileStorage fsSettings_;
    Tracking* mTracker;
public:
    Tracking* GetTracker();
    CoLocalSystem(const cv::FileStorage& fsSettings);
    ~CoLocalSystem();
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight,int id);
};

