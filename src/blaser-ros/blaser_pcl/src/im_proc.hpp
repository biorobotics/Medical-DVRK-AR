#ifndef __IM_PROC__HPP__
#define __IM_PROC__HPP__

#include <opencv2/opencv.hpp>

void SimplestCB(const cv::Mat& in, cv::Mat& out, float percent);

std::string type2str(int type);

#endif /* __IM_PROC__HPP__ */