#include "CoLocalSystem.h"

CoLocalSystem::CoLocalSystem(const cv::FileStorage& fsSettings)
{
    fsSettings_ = fsSettings;
    Tracker_ = new Tracking(fsSettings_);
}

cv::Mat CoLocalSystem::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight,int id)
{
    return Tracker_->GrabImageStereo(imLeft,imRight,id);
}

CoLocalSystem::~CoLocalSystem()
{
    delete Tracker_;
}