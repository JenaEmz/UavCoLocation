#pragma once

#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "Frame.h"
#include "MapPoint.h"

#include "Map.h"
#include <string>
#include <thread>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;

class Map;

class Tracking
{
  private:
    ORBextractor *mpORBextractorLeft, *mpORBextractorRight;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;
    Frame mCurrentFrame;
    Frame mLastFrame;
    int mTrackFlag;
    float mThDepth;
    Map* mpMap;

    ORBVocabulary* mpVocabulary;

  public:
    void traceMap(Frame* frame);
    void StereoInitialization();
    Tracking(const cv::FileStorage& fSettings);
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, int id);

    bool TrackReferenceKeyFrame();
};
