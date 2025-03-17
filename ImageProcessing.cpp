/*
Functions for ball detection using background subtraction. 
*/


#include <opencv2/opencv.hpp>
#include "ImageProcessing.h"
#include <QtCore/QDebug>

cv::Mat DisplayText(cv::Mat image, const std::string& text, const cv::Point& org) {
    int font = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.75;
    cv::Scalar color(255, 255, 255); // White color
    int thickness = 2;

    cv::putText(image, text, org, font, fontScale, color, thickness, cv::LINE_AA);
    return image;
}

std::vector<cv::Point> FindCentroids(const cv::Mat& thresholded_img) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    // Find contours in the thresholded image
    cv::findContours(thresholded_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> centroids;

    // Iterate through each contour and calculate its centroid
    for (const auto& contour : contours) {
        cv::Moments M = cv::moments(contour, true);

        // Check if the area (m00) is non-zero to avoid division by zero
        if (M.m00 != 0) {
            qDebug() << "Centroid found";
            int cX = static_cast<int>(M.m10 / M.m00);
            int cY = static_cast<int>(M.m01 / M.m00);
            centroids.emplace_back(cX, cY); // Add the centroid to the vector
        } else { // to make LED calibration more reliable (don't remove)
            qDebug() << "Found contour with zero area, using first edge point as centroid";
            int cX = contour[0].x;
            int cY = contour[0].y;
            centroids.emplace_back(cX, cY);
        }
    }

    return centroids; // Return the vector of centroids
}


int DrawCentroid(cv::Mat& image, cv::Point& centroid)
{
    // cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
    cv::circle(image, centroid, 5, cv::Scalar(0, 0, 255), -1);

    return 0;
}

cv::Mat SubtractBackground(const cv::Mat& image, cv::Ptr<cv::BackgroundSubtractor> backSub)
{
    int erode_kernel = 3;

    qDebug()<<"about to subtract bg";

    // Create the foreground mask
    cv::Mat fgMask;
    backSub->apply(image, fgMask);
    qDebug()<<"subtracted bg";

    // Remove shadows (threshold to binary, where values > 254 are set to 255)
    cv::threshold(fgMask, fgMask, 254, 255, cv::THRESH_BINARY);

    // Create kernel for erosion and dilation (morphological operations)
    cv::Mat kernel = cv::Mat::ones(erode_kernel, erode_kernel, CV_8U);

    // Apply erosion
    cv::Mat erodedMask;
    cv::erode(fgMask, erodedMask, kernel, cv::Point(-1, -1), 1);

    // Apply dilation
    cv::Mat dilatedMask;
    cv::dilate(erodedMask, dilatedMask, kernel, cv::Point(-1, -1), 1);

    // Return the new mask

    return dilatedMask;
}


cv::Mat FindLargestContours(const cv::Mat& thresholdedImage, int numContours) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    // Find contours in the thresholded image
    cv::findContours(thresholdedImage, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Store contours with their areas
    std::vector<std::pair<double, int>> contourAreas;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        contourAreas.emplace_back(area, static_cast<int>(i));
    }

    // Sort the contours by area in descending order
    std::sort(contourAreas.begin(), contourAreas.end(), [](const auto& a, const auto& b) {
        return a.first > b.first;
    });

    // Limit the number of contours to process
    numContours = std::min(numContours, static_cast<int>(contourAreas.size()));

    // Create a mask to hold the N largest contours
    cv::Mat largestContoursMask = cv::Mat::zeros(thresholdedImage.size(), CV_8U);

    for (int i = 0; i < numContours; i++) {
        int contourIndex = contourAreas[i].second;

        // Draw the current contour on the mask
        cv::drawContours(largestContoursMask, contours, contourIndex, cv::Scalar(255), cv::FILLED);
    }

    // Apply the mask to the original thresholded image
    cv::Mat result;
    cv::bitwise_and(thresholdedImage, largestContoursMask, result);

    return result; // Return the image with the N largest contours
}




cv::Mat ApplyThreshold(const cv::Mat& inputImage, double thresholdValue, double maxValue, int thresholdType)
{
    cv::Mat thresholdedImage;
    cv::threshold(inputImage, thresholdedImage, thresholdValue, maxValue, thresholdType);
    return thresholdedImage;
}
