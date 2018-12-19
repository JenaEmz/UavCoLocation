#include "Tracking.h"
cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight,int id)
{
    if(id == 0)//是自己的
    {
        mlastFrame = Frame(imRectLeft,imRectRight,mpORBextractorLeft,mpORBextractorRight,mK);
        return cv::Mat();
    }
    else
    {
        mCurrentFrame = Frame(imRectLeft,imRectRight,mpORBextractorLeft,mpORBextractorRight,mK);
        return cv::Mat();//此处应返回相对位置
    }
}