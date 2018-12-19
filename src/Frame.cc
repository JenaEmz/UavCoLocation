#include "Frame.h"
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, ORBextractor *extractorLeft, ORBextractor *extractorRight, cv::Mat &K) : mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight)
{
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    //mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    thread threadLeft(&Frame::ExtractORB, this, 0, imLeft);
    thread threadRight(&Frame::ExtractORB, this, 1, imRight);
    threadLeft.join();
    threadRight.join();
    //debug
    {
        cv::Mat img, imgRight;
        imLeft.copyTo(img);
        imRight.copyTo(imgRight);
        std::for_each(mvKeys.begin(), mvKeys.end(), [&](cv::KeyPoint i) {
            cv::circle(img, i.pt, 4 * (i.octave + 1), cv::Scalar(0, 255, 0), 1);
        });
        std::for_each(mvKeysRight.begin(), mvKeysRight.end(), [&](cv::KeyPoint i) {
            cv::circle(imgRight, i.pt, 4 * (i.octave + 1), cv::Scalar(0, 255, 0), 1);
        });
        cv::imshow("left extractor", img);
        cv::imshow("right extractor", imgRight);
        cv::waitKey(1);
    }

    //debug end
    N = mvKeys.size();

    if (mvKeys.empty())
        return;

    mvuRight = vector<float>(N, -1);
    mvDepth = vector<float>(N, -1);

    //mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N, false);
}
void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if (flag == 0)
        (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors);
    else
        (*mpORBextractorRight)(im, cv::Mat(), mvKeysRight, mDescriptorsRight);
}
