#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

std::vector<std::string> getOrderedImagePaths(const std::string& directoryPath);

std::vector<cv::Mat> loadImages(const std::vector<std::string>& filePaths);

std::pair<std::vector<cv::Mat>, double> getCameraAndRTMatrices(std::string& dirCam1, std::string& dirCam2, int& rows, int& columns, double& worldScaling);

std::vector<cv::Mat> getProjectionMatrices(cv::Mat& cameraMatrix1, cv::Mat& cameraMatrix2, cv::Mat& R, cv::Mat& T);

cv::Mat triangulatePoint(const cv::Mat& P1, const cv::Mat& P2, const cv::Point2f& point1, const cv::Point2f& point2);

void calibrate(std::string& calDir, int& rows, int& columns, double& worldScaling);

#endif // CAMERACALIBRATION_H
