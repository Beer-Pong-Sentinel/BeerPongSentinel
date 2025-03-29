#ifndef IMAGEPROCESSING_H
#define IMAGEPROCESSING_H

#include <opencv2/opencv.hpp>
#include <string>

/*
 * Functions for ball detection using background subtraction.
 */

// Display text on an image at a specified location
cv::Mat DisplayText(cv::Mat image, const std::string& text, const cv::Point& org);

// Find the centroid of a thresholded binary image
std::vector<cv::Point> FindCentroids(const cv::Mat& thresholded_img);

// Draw a centroid point on the image
int DrawCentroid(cv::Mat& image, cv::Point& centroid);

// Subtract background from an image using a background subtractor model
cv::Mat SubtractBackground(const cv::Mat& image, cv::Ptr<cv::BackgroundSubtractor> backSub, cv::Mat& fgMask, cv::Mat& tmpGray, cv::Mat& kernel);

// BGSUB WITH CUDA
cv::Mat SubtractBackgroundCUDA(const cv::Mat& image,
                               cv::Ptr<cv::BackgroundSubtractor> d_backSub,
                               cv::cuda::GpuMat& d_fgMask,
                               cv::cuda::GpuMat& d_tmpGray,
                               cv::cuda::GpuMat& d_kernel);

std::pair<cv::Mat, cv::Point> getBGSubCentroid(
    const cv::Mat& image,
    cv::Ptr<cv::BackgroundSubtractor> d_backSub,
    cv::cuda::GpuMat& d_fgMask,
    cv::cuda::GpuMat& d_tmpGray,
    cv::cuda::GpuMat& d_kernel);

// Find the largest contour in a thresholded image and create an output image with only the largest contour
cv::Mat FindLargestContours(const cv::Mat& thresholdedImage, int numContours);

cv::Point FindLargestCentroid(const cv::Mat& thresholdedImage);

cv::Mat ApplyThreshold(const cv::Mat& inputImage, double thresholdValue, double maxValue, int thresholdType);

void ApplyThresholdConsecutively(const cv::Mat& input, cv::Mat& tmp, cv::Mat& output, const cv::Mat& background, double threshold);

void ApplyHSVThreshold(const cv::Mat& input, cv::Mat& tmp, cv::Mat& output, double minH, double maxH, double minS, double maxS, double minV, double maxV);

cv::Mat TestApplyHSVThreshold(const cv::Mat& input, double minH, double maxH, double minS, double maxS, double minV, double maxV);

void ApplyBGRThreshold(const cv::Mat& input, cv::Mat& output, double minR, double maxR, double minG, double maxG, double minB, double maxB);

void TestApplyBGRThreshold(const cv::Mat& input, cv::Mat& tmp, cv::Mat& output, double minH, double maxH, double minS, double maxS, double minV, double maxV);

void ApplyMotionThreshold(const cv::Mat& input, cv::Mat& tmp, cv::Mat& output, const cv::Mat& background, double threshold);

void ApplyMotionThresholdConsecutively(const cv::Mat& input, cv::Mat& tmp, cv::Mat& output, const cv::Mat& background, double threshold);

void ApplyMorphClosing(cv::Mat& input, cv::Mat& kernel);

cv::Point2f ComputeCentroid(const cv::Mat& input);

void DrawCentroidBinary(cv::Mat& image, const cv::Point2f& centroid, int radius = 5, cv::Scalar color = cv::Scalar(255), int thickness = -1);

#endif // IMAGEPROCESSING_H
