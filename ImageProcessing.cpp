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
            // qDebug() << "Centroid found";
            int cX = static_cast<int>(M.m10 / M.m00);
            int cY = static_cast<int>(M.m01 / M.m00);
            centroids.emplace_back(cX, cY); // Add the centroid to the vector
        } else { // to make LED calibration more reliable (don't remove)
            // qDebug() << "Found contour with zero area, using first edge point as centroid";
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

cv::Mat SubtractBackground(const cv::Mat& image, cv::Ptr<cv::BackgroundSubtractor> backSub, cv::Mat& fgMask, cv::Mat& tmpGray, cv::Mat& kernel)
{
    cv::cvtColor(image, tmpGray, cv::COLOR_BGR2GRAY);

    // qDebug()<<"about to subtract bg";

    // Create the foreground mask
    backSub->apply(image, fgMask);
    // qDebug()<<"subtracted bg";

    // Remove shadows (threshold to binary, where values > 254 are set to 255)
    cv::threshold(fgMask, fgMask, 254, 255, cv::THRESH_BINARY);

    // Create kernel for erosion and dilation (morphological operations)

    // Apply erosion
    cv::erode(fgMask, fgMask, kernel, cv::Point(-1, -1), 1);

    // Apply dilation
    cv::dilate(fgMask, fgMask, kernel, cv::Point(-1, -1), 1);

    // Return the new mask

    return fgMask;
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
    cv::Mat grayscale;
    cv::cvtColor(inputImage, grayscale, cv::COLOR_BGR2GRAY);
    cv::Mat thresholdedImage;
    cv::threshold(grayscale, thresholdedImage, thresholdValue, maxValue, thresholdType);
    return thresholdedImage;
}

void ApplyHSVThreshold(const cv::Mat& input, cv::Mat& tmp, cv::Mat& output, 
                       double minH, double maxH, 
                       double minS, double maxS, 
                       double minV, double maxV) {
    // Convert the input image to HSV color space
    cv::cvtColor(input, tmp, cv::COLOR_BGR2HSV);

    if (maxH > 180) {
        // First mask for range [minH, 180]
        cv::Mat mask1, mask2;
        cv::inRange(tmp, cv::Scalar(minH, minS, minV), cv::Scalar(180, maxS, maxV), mask1);
        // Second mask for range [0, maxH - 180]
        cv::inRange(tmp, cv::Scalar(0, minS, minV), cv::Scalar(maxH - 180, maxS, maxV), mask2);
        // Combine the two masks
        cv::bitwise_or(mask1, mask2, output);
    } else {
        // Normal case (no wrap-around)
        cv::inRange(tmp, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV), output);
    }
}

cv::Mat TestApplyHSVThreshold(const cv::Mat& input, double minH, double maxH, double minS, double maxS, double minV, double maxV) {
    cv::Mat tmp, output;
    cv::cvtColor(input, tmp, cv::COLOR_BGR2HSV);
    cv::inRange(tmp, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV), output);
    return output;
}

void ApplyBGRThreshold(const cv::Mat& input, cv::Mat& output, double minB, double maxB, double minG, double maxG, double minR, double maxR) {
    // Use inRange to apply the thresholds directly in the RGB color space
    cv::inRange(input, cv::Scalar(minB, minG, minR), cv::Scalar(maxB, maxG, maxR), output);
}

void TestApplyBGRThreshold(const cv::Mat& input, cv::Mat& tmp, cv::Mat& output, double minH, double maxH, double minS, double maxS, double minV, double maxV) {
    cv::inRange(input, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV), output);
}

void ApplyThresholdConsecutively(const cv::Mat& input, cv::Mat& tmp, cv::Mat& output, const cv::Mat& background, double threshold) {
    cv::cvtColor(input, tmp, cv::COLOR_BGR2GRAY);
    cv::threshold(tmp, tmp, threshold, 255, cv::THRESH_BINARY);
    cv::bitwise_and(output, tmp, output);
}

void ApplyMotionThreshold(const cv::Mat& input, cv::Mat& tmp, cv::Mat& output, const cv::Mat& background, double threshold) {
    if (background.empty()) return;
    cv::cvtColor(input, tmp, cv::COLOR_BGR2GRAY);
    cv::absdiff(tmp, background, tmp);
    cv::threshold(tmp, output, threshold, 255, cv::THRESH_BINARY);
}

void ApplyMotionThresholdConsecutively(const cv::Mat& input, cv::Mat& tmp, cv::Mat& output, const cv::Mat& background, double threshold) {
    if (background.empty()) return;
    cv::cvtColor(input, tmp, cv::COLOR_BGR2GRAY);
    cv::absdiff(tmp, background, tmp);
    cv::threshold(tmp, tmp, threshold, 255, cv::THRESH_BINARY);
    cv::bitwise_and(output, tmp, output);
}

void ApplyMorphClosing(cv::Mat& input, cv::Mat& kernel) {
    cv::erode(input, input, kernel);
    cv::dilate(input, input, kernel);
}

cv::Point2f ComputeCentroid(const cv::Mat& input) {
    int sumX = 0, sumY = 0, count = 0;

    // Use a pointer-based approach for speed
    for (int y = 0; y < input.rows; ++y) {
        const uchar* rowPtr = input.ptr<uchar>(y);
        for (int x = 0; x < input.cols; ++x) {
            if (rowPtr[x] > 0) {  // White pixel (assuming binary image)
                sumX += x;
                sumY += y;
                count++;
            }
        }
    }

    if (count == 0) {
        return cv::Point2f(-1, -1);  // No white pixels found
    }

    return cv::Point2f(static_cast<float>(sumX) / count, static_cast<float>(sumY) / count);
}

void DrawCentroidBinary(cv::Mat& image, const cv::Point2f& centroid, int radius, cv::Scalar color, int thickness) {
    if (centroid.x >= 0 && centroid.y >= 0) {
        cv::circle(image, centroid, radius, color, thickness);
    }
}

