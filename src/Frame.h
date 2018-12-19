#pragma once

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

#include "ORBextractor.h"

#include <thread>

#include <opencv2/opencv.hpp>
using namespace std;
class Frame
{
private:
    void ExtractORB(int flag, const cv::Mat &im);
    ORBextractor *mpORBextractorLeft, *mpORBextractorRight;
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    // 对于双目，mvuRight存储了左目像素点在右目中的对应点的横坐标
    // mvDepth对应的深度
    // 单目摄像头，这两个容器中存的都是-1
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;
    cv::Mat mDescriptors, mDescriptorsRight;

    int N; //特征点个数

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    // Flag to identify outlier associations.
    // 观测不到Map中的3D点
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    // 坐标乘以mfGridElementWidthInv和mfGridElementHeightInv就可以确定在哪个格子
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    // 每个格子分配的特征点数，将图像分成格子，保证提取的特征点比较均匀
    // FRAME_GRID_ROWS 48
    // FRAME_GRID_COLS 64
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw; ///< 相机姿态 世界坐标系到相机坐标坐标系的变换矩阵

    // Current and Next Frame id.
    static long unsigned int nNextId; ///< Next Frame id.
    long unsigned int mnId;           ///< Current Frame id.

    // Scale pyramid info.
    int mnScaleLevels;      //图像提金字塔的层数
    float mfScaleFactor;    //图像提金字塔的尺度因子
    float mfLogScaleFactor; //
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    // 用于确定画格子时的边界
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;

public:
    Frame()
    {
    }
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, ORBextractor *extractorLeft, ORBextractor *extractorRight, cv::Mat &K);

    ~Frame()
    {
    }
};
