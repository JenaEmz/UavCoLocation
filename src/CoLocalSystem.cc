#include "CoLocalSystem.h"

CoLocalSystem::CoLocalSystem(const cv::FileStorage& fsSettings)
{
    fsSettings_ = fsSettings;
    mTracker = new Tracking(fsSettings_);
}

cv::Mat CoLocalSystem::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight,int id)
{
    // 这两个完了之后才能做匹配吧
    // 匹配是靠Track这个函数来做的吧
    return mTracker->GrabImageStereo(imLeft,imRight,id);
}

CoLocalSystem::~CoLocalSystem()
{
    delete mTracker;
}
Tracking* CoLocalSystem::GetTracker(){
    return mTracker;
}