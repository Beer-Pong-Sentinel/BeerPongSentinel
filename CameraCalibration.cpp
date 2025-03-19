#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "json.hpp"
#include <QtCore/QDebug>

namespace fs = std::filesystem;

std::vector<std::string> getOrderedImagePaths(const std::string& directoryPath) {
    std::vector<std::string> filePaths;
    qDebug()<<"getting ordered image paths";

    // Iterate through the directory and collect paths with .jpeg extension
    for (const auto& entry : fs::directory_iterator(directoryPath)) {
        qDebug()<<"in for loop in getOrderedImagePaths";
        if (entry.is_regular_file() && entry.path().extension() == ".jpeg") {
            //CHANGE TO JPEG LATER
            filePaths.push_back(entry.path().string());
            qDebug()<<"added a path:";
            qDebug()<<entry.path().string();
        }
    }

    // Sort the vector based on numeric order of the filenames
    std::sort(filePaths.begin(), filePaths.end(), [](const std::string& a, const std::string& b) {
        int numA = std::stoi(fs::path(a).stem().string());
        int numB = std::stoi(fs::path(b).stem().string());
        return numA < numB;
    });

    return filePaths;
}

std::vector<cv::Mat> loadImages(const std::vector<std::string>& filePaths) {
    std::vector<cv::Mat> images;

    for (const auto& path : filePaths) {
        cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);  // Load the image in color

        if (img.empty()) {
            std::cerr << "Warning: Could not open or find the image at path: " << path << std::endl;
            continue;  // Skip if the image could not be loaded
        }

        images.push_back(img);  // Add the loaded image to the vector
    }

    return images;
}

std::pair<std::vector<cv::Mat>, double> getCameraAndRTMatrices(std::string& dirCam1, std::string& dirCam2, int& rows, int& columns, double& worldScaling){

    // Get images
    std::vector<std::string> imagePaths1 = getOrderedImagePaths(dirCam1);
    qDebug()<<"imagapaths1";
    qDebug()<<imagePaths1;
    std::vector<std::string> imagePaths2 = getOrderedImagePaths(dirCam2);
    qDebug()<<"imagapaths2";
    qDebug()<<imagePaths2;
    std::vector<cv::Mat> images1 = loadImages(imagePaths1);
    std::vector<cv::Mat> images2 = loadImages(imagePaths2);

    // Get image size
    cv::Size imageSize = images1[0].size();

    // Set chessboard corner search criterea
    cv::TermCriteria criteriaIndividual(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001);

    // Coordinates of squares in the checkerboard world space
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < columns; j++) {
            objp.emplace_back(j * worldScaling, i * worldScaling, 0);
        }
    }

    // Storage for detected corners and corresponding 3D points
    std::vector<std::vector<cv::Point3f>> objectPoints; // 3D points in world space
    std::vector<std::vector<cv::Point2f>> imagePoints1; // 2D points in image plane
    std::vector<std::vector<cv::Point2f>> imagePoints2;

    // Loop through each image to detect the checkerboard corners
    for (size_t i = 0; i < images1.size() && i < images2.size(); ++i) {

        cv::Mat frame1 = images1[i];
        cv::Mat frame2 = images2[i];

        // In case they're not already grayscale for some reason
        cv::Mat gray1;
        cv::cvtColor(frame1, gray1, cv::COLOR_BGR2GRAY);
        cv::Mat gray2;
        cv::cvtColor(frame2, gray2, cv::COLOR_BGR2GRAY);

        // Find the checkerboard corners
        std::vector<cv::Point2f> corners1;
        std::vector<cv::Point2f> corners2;
        bool ret1 = cv::findChessboardCorners(gray1, cv::Size(columns, rows), corners1);
        bool ret2 = cv::findChessboardCorners(gray2, cv::Size(columns, rows), corners2);

        if (ret1 && ret2) {
            // Refine the corner locations
            cv::cornerSubPix(gray1, corners1, cv::Size(11, 11), cv::Size(-1, -1), criteriaIndividual);
            cv::cornerSubPix(gray2, corners2, cv::Size(11, 11), cv::Size(-1, -1), criteriaIndividual);

            // Add object points and image points to the lists
            objectPoints.push_back(objp);
            imagePoints1.push_back(corners1);
            imagePoints2.push_back(corners2);
        }
        else{
            std::cout<<"Unable to locate corners for one or both images in the pair" << std::endl;
        }
    }

    // Output variables
    cv::Mat cameraMatrix1, cameraMatrix2, distortionCoeffs1, distortionCoeffs2;
    std::vector<cv::Mat> rotationVectors1, rotationVectors2, translationVectors1, translationVectors2;

    // Calibrate the camera
    double rms1 = cv::calibrateCamera(objectPoints,
                                     imagePoints1,
                                     imageSize,
                                     cameraMatrix1,
                                     distortionCoeffs1,
                                     rotationVectors1,
                                     translationVectors1);

    double rms2 = cv::calibrateCamera(objectPoints,
                                      imagePoints2,
                                      imageSize,
                                      cameraMatrix2,
                                      distortionCoeffs2,
                                      rotationVectors2,
                                      translationVectors2);

    cv::Mat R, T, E, F;
    int stereocalibrationFlags = cv::CALIB_FIX_INTRINSIC;
    cv::TermCriteria criteriaStereo(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 1e-4);

    double stereoProjectionError = cv::stereoCalibrate(
        objectPoints, imagePoints1, imagePoints2,
        cameraMatrix1, distortionCoeffs1, cameraMatrix2, distortionCoeffs2, imageSize,
        R, T, E, F, stereocalibrationFlags, criteriaStereo
        );

    qDebug()<<"projection error:";
    qDebug()<<stereoProjectionError;

    return {{cameraMatrix1, cameraMatrix2, R, T},stereoProjectionError};

}

std::vector<cv::Mat> getProjectionMatrices(cv::Mat& cameraMatrix1, cv::Mat& cameraMatrix2, cv::Mat& R, cv::Mat&T){

    // RT matrix for C1 is identity.
    cv::Mat RT1 = cv::Mat::eye(3, 4, CV_64F);  // 3x4 identity matrix for C1
    cv::Mat P1 = cameraMatrix1 * RT1;                   // Projection matrix for C1

    // RT matrix for C2 using R and T from stereo calibration
    cv::Mat RT2;
    cv::hconcat(R, T, RT2);                    // Concatenate R and T horizontally to form a 3x4 matrix
    cv::Mat P2 = cameraMatrix2 * RT2;                   // Projection matrix for C2

    // Output matrices for verification
    std::cout << "Projection matrix for camera 1 (P1):\n" << P1 << std::endl;
    std::cout << "Projection matrix for camera 2 (P2):\n" << P2 << std::endl;

    return { P1, P2 };

}

// Attempt to implement triangulation using OpenCV's triangulatePoints function, currently facing error
void triangulate(const std::vector<cv::Point2f>& points1, const std::vector<cv::Point2f>& points2,
                       const cv::Mat& P1, const cv::Mat& P2, std::vector<cv::Mat>& points3D) {
    // Check input sizes
    if (points1.size() != points2.size()) {
        throw std::runtime_error("Input point vectors must be of the same size.");
    }

    // Convert points to cv::Mat (2xN matrix)
    cv::Mat points1Mat(2, points1.size(), CV_64F);
    cv::Mat points2Mat(2, points2.size(), CV_64F);

    for (size_t i = 0; i < points1.size(); i++) {
        points1Mat.at<double>(0, i) = points1[i].x;
        points1Mat.at<double>(1, i) = points1[i].y;
        points2Mat.at<double>(0, i) = points2[i].x;
        points2Mat.at<double>(1, i) = points2[i].y;
    }

    // Triangulate points
    cv::Mat points4D;
    std::cout << "Camera 1 points: " << points1Mat << std::endl;
    std::cout << "Camera 2 points: " << points2Mat << std::endl;

    cv::triangulatePoints(P1, P2, points1Mat, points2Mat, points4D);

    std::cout << "Triangulation output: " << points4D << std::endl;
    // Convert homogeneous coordinates to 3D Cartesian coordinates and store as 1x3 matrices
    points3D.clear();
    for (int i = 0; i < points4D.cols; i++) {
        cv::Mat x = points4D.col(i);
        x /= x.at<double>(3); // Normalize by the last coordinate

        // Create a 1x3 matrix for the 3D point
        cv::Mat point3D = (cv::Mat_<double>(1, 3) << x.at<double>(0), x.at<double>(1), x.at<double>(2));
        points3D.push_back(point3D);
    }
}

cv::Mat triangulatePoint(const cv::Mat& P1, const cv::Mat& P2, const cv::Point2f& point1, const cv::Point2f& point2) {
    // Create matrix A based on the point correspondences and projection matrices
    cv::Mat A(4, 4, CV_64F);
    A.row(0) = point1.y * P1.row(2) - P1.row(1);
    A.row(1) = P1.row(0) - point1.x * P1.row(2);
    A.row(2) = point2.y * P2.row(2) - P2.row(1);
    A.row(3) = P2.row(0) - point2.x * P2.row(2);

    // Compute B = A^T * A
    cv::Mat B = A.t() * A;

    // Perform SVD on B
    cv::SVD svd(B, cv::SVD::FULL_UV);

    // Extract the last row of V^T, which corresponds to the smallest singular value
    cv::Mat V = svd.vt.t();
    cv::Mat triangulatedPoint = V.row(3).colRange(0, 3) / V.at<double>(3, 3);

    // Print the triangulated point for verification
    std::cout << "Triangulated point: " << triangulatedPoint << std::endl;

    return triangulatedPoint;
}

void calibrate(std::string& calDir, int& rows, int& columns, double& worldScaling) {
    std::string dirCam1 = "../../CameraCalibration/" + calDir + "/cam1";
    std::string dirCam2 = "../../CameraCalibration/" + calDir + "/cam2";

    qDebug()<<dirCam1;
    qDebug()<<dirCam2;


    std::pair<std::vector<cv::Mat>,double> camAndRTMatrices = getCameraAndRTMatrices(dirCam1, dirCam2, rows, columns, worldScaling);

    std::vector<cv::Mat> matrices = camAndRTMatrices.first;
    cv::Mat cameraMatrix1 = matrices[0];
    cv::Mat cameraMatrix2 = matrices[1];
    cv::Mat R = matrices[2];
    cv::Mat T = matrices[3];

    double stereoProjectionError = camAndRTMatrices.second;


    // Get projection matrices
    std::vector<cv::Mat> projectionMatrices = getProjectionMatrices(cameraMatrix1, cameraMatrix2, R, T);
    cv::Mat P1 = projectionMatrices[0];
    cv::Mat P2 = projectionMatrices[1];
    std::cout << "P1: " << P1 << std::endl;
    std::cout << "P2: " << P2 << std::endl;

    // Convert projection matrices to JSON format
    nlohmann::json calibrationData;

    // Helper lambda to convert cv::Mat to JSON array
    auto matToJsonArray = [](const cv::Mat& mat) {
        nlohmann::json jsonArray;
        for (int i = 0; i < mat.rows; ++i) {
            nlohmann::json row;
            for (int j = 0; j < mat.cols; ++j) {
                row.push_back(mat.at<double>(i, j));
            }
            jsonArray.push_back(row);
        }
        return jsonArray;
    };

    // Add matrices to the JSON object
    calibrationData["P1"] = matToJsonArray(P1);
    calibrationData["P2"] = matToJsonArray(P2);
    calibrationData["projection_error"] = stereoProjectionError;

    // Specify the output file path
    std::string outputPath = "../../CameraCalibration/" + calDir + "/" + calDir + ".json";

    // Write JSON data to file
    std::ofstream file(outputPath);
    if (file.is_open()) {
        file << calibrationData.dump(4);  // Pretty print with 4 spaces
        file.close();
        std::cout << "Calibration data saved to " << outputPath << std::endl;
    } else {
        std::cerr << "Error: Could not open file for writing: " << outputPath << std::endl;
    }
}






























