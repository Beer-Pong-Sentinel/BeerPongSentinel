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
cv::Mat SubtractBackground(const cv::Mat& image, cv::Ptr<cv::BackgroundSubtractor> backSub);

// Find the largest contour in a thresholded image and create an output image with only the largest contour
cv::Mat FindLargestContours(const cv::Mat& thresholdedImage, int numContours);

cv::Mat ApplyThreshold(const cv::Mat& inputImage, double thresholdValue, double maxValue, int thresholdType);

#endif // IMAGEPROCESSING_H
