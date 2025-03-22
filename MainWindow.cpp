#include "MainWindow.h"
#include <QtCore/QDebug>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QFileInfoList>
#include <QtCore/QStringList>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QFileDialog>
#include <QtCore/QStandardPaths>
#include <QtCore/QFile>
#include "CameraStreamWidget.h"
#include "CameraCalibration.h"
#include "AltitudeControl.h"
#include "AzimuthControl.h"
#include <chrono>
#include <thread>
#include <gl/GL.h>
#include "json.hpp"
#include <fstream>
#include <QSerialPort>
#include "ImageProcessing.h"
#include <QtConcurrent>
#include <QFuture>
#include <QMutex>
#include <time.h>
#include <sstream>
//#include <QmlDebuggingEnabler>

using json = nlohmann::json;

//QQmlDebuggingEnabler enabler;

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent), captureThread(nullptr), serialPort(new QSerialPort(this)) {
    ui.setupUi(this);  // Sets up the UI and initializes the widgets
    qDebug()<<"setup ui";
    this->adjustSize();
    // showMaximized();


    // Serial Port

    // Configure the serial port (modify these settings as necessary for your device)

    serialPort->setPortName("COM4");

    if (!serialPort->open(QIODevice::ReadWrite)) {
        qDebug() << "Error: Failed to open serial port" << serialPort->portName();
    }

    qDebug()<< "set com3";
    serialPort->setBaudRate(QSerialPort::Baud115200);
    qDebug()<< "set baud rate";
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);

    qDebug()<<"set serial settings";


    connect(ui.selectCal3DFromSavedFile, &QRadioButton::toggled, this, &MainWindow::toggleCal3DType);
    connect(ui.selectNewCal3D, &QRadioButton::toggled, this, &MainWindow::toggleCal3DType);
    connect(ui.selectMoCamCalFromSavedFile, &QRadioButton::toggled, this, &MainWindow::toggleMotorCameraCalType);
    connect(ui.selectNewMoCamCal, &QRadioButton::toggled, this, &MainWindow::toggleMotorCameraCalType);
    connect(ui.savedCal3DFilesRefreshButton, &QPushButton::clicked, this, &MainWindow::updateSavedCameraCalibrationFilesComboBox);
    connect(ui.calibrateCal3DFromLoadedFile, &QPushButton::clicked, this, &MainWindow::calibrateCameraWithSelectedFile);
    connect(ui.newCal3DStartImageCaptureButton, &QPushButton::clicked, this, &MainWindow::newCameraCalibrationStartImageCapture);
    connect(ui.newCal3DSaveImagePairButton, &QPushButton::clicked, this, &MainWindow::newCameraCalibrationSaveImagePair);
    connect(ui.newCal3DStopImageCaptureButton, &QPushButton::clicked, this, &MainWindow::newCal3DStopImageCapture);
    // connect(ui.cancelNewCal3DButton, &QPushButton::clicked, this, &MainWindow::cancelNewCal3D);
    connect(ui.calibrateNewCal3DButton, &QPushButton::clicked, this, &MainWindow::calibrateCameras);
    connect(ui.startCameraCaptureThreadButton, &QPushButton::clicked, this, &MainWindow::startCapture);
    connect(ui.stopCameraCaptureThreadButton, &QPushButton::clicked, this, &MainWindow::stopCapture);
    //connect(ui.startMotorCameraCalibrationButton, &QPushButton::clicked, this, &MainWindow::calibrateMotorCamera);
    connect(ui.sendMotorPositionButton, &QPushButton::clicked, this, &MainWindow::sendMotorPositions);
    connect(ui.setProcessingButton, &QPushButton::clicked, this, &MainWindow::setProcesseing);
    connect(ui.calibrateSphericalButton, &QPushButton::clicked, this, &MainWindow::sphericalCalibration);
    connect(ui.targetAimButton, &QPushButton::clicked, this, &MainWindow::sphericalTest);
    connect(ui.setBackgroundImageButton, &QPushButton::clicked, this, &MainWindow::setBackgroundImage);

    connect(ui.fireButton, &QPushButton::clicked, this, &MainWindow::fire);


    connect(ui.initAlPushButton, &QPushButton::clicked, this, &MainWindow::initializeAltitude);
    connect(ui.initAzPushButton, &QPushButton::clicked, this, &MainWindow::initializeAzimuth);
    connect(ui.enableAzButton, &QPushButton::clicked, this, &MainWindow::enableAzimuth);
    connect(ui.disableAzButton, &QPushButton::clicked, this, &MainWindow::disableAzimuth);

    connect(ui.testMoveTimeButton, &QPushButton::clicked, this, &MainWindow::altitudeTimeTest);



    connect(ui.sendAltitudeButton, &QPushButton::clicked, this, &MainWindow::altitudeTest);
    connect(ui.sendAzimuthButton, &QPushButton::clicked, this, &MainWindow::azimuthTest);
    connect(this, &MainWindow::processedFramesReady, this, &MainWindow::onProcessedFramesReady);
    connect(ui.takePictureButton, &QPushButton::clicked, this, &MainWindow::saveProcessedFrames);

    connect(ui.enableAlButton, &QPushButton::clicked, this, &MainWindow::enableAltitude);
    connect(ui.disableAlButton, &QPushButton::clicked, this, &MainWindow::disableAltitude);

    connect(ui.homeMotorsButton, &QPushButton::clicked, this, &MainWindow::homeMotors);


    connect(ui.goToAlLimitButton, &QPushButton::clicked, this, &MainWindow::moveAlLimit);
    connect(ui.goToAzLimitButton, &QPushButton::clicked, this, &MainWindow::moveAzLimit);

    connect(ui.setLowerAzLimit, &QPushButton::clicked, this, &MainWindow::setAzimuthLowerLimit);
    connect(ui.setUpperAzLimit, &QPushButton::clicked, this, &MainWindow::setAzimuthUpperLimit);
    connect(ui.setLowerAlLimit, &QPushButton::clicked, this, &MainWindow::setAltitudeLowerLimit);
    connect(ui.setUpperAlLimit, &QPushButton::clicked, this, &MainWindow::setAltitudeUpperLimit);

    connect(ui.startMotorCameraCalibrationButton, &QPushButton::clicked, this, &MainWindow::sweepLookupTable);

    qDebug()<<"connected thread";

    updateSavedCameraCalibrationFilesComboBox();


}

MainWindow::~MainWindow() {
    // Cleanup if necessary
    stopCapture();
    disableAltitude();
    disableAzimuth();
}

void MainWindow::toggleCal3DType() {

    if (ui.selectCal3DFromSavedFile->isChecked()) {
        ui.newCal3DGroupBox->setEnabled(false);
        ui.loadCal3DFromSavedFileGroupBox->setEnabled(true);
    } else if (ui.selectNewCal3D->isChecked()) {
        ui.loadCal3DFromSavedFileGroupBox->setEnabled(false);
        ui.newCal3DGroupBox->setEnabled(true);
        resetNewCameraCalibration();
    }
}

void MainWindow::toggleMotorCameraCalType() {

    if (ui.selectMoCamCalFromSavedFile->isChecked()) {
        ui.newMoCamCalGroupbox->setEnabled(false);
        ui.loadMoCamCalGroupbox->setEnabled(true);
    } else if (ui.selectNewMoCamCal->isChecked()) {
        ui.loadMoCamCalGroupbox->setEnabled(false);
        ui.newMoCamCalGroupbox->setEnabled(true);
    }
}

// Helper function to get directories sorted by modification date
QStringList MainWindow::getDirectoriesSortedByDate(const QString &directoryPath) {
    QDir dir(directoryPath);

    // Set filter to only include directories and no other file types
    dir.setFilter(QDir::Dirs | QDir::NoDotAndDotDot);

    // Retrieve directory info list sorted by last modified time
    QFileInfoList dirList = dir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Time);

    // Extract the directory paths
    QStringList sortedDirectories;
    for (const QFileInfo &info : dirList) {
        sortedDirectories.append(info.fileName());  // Use fileName() to get the directory name
    }

    return sortedDirectories;
}

// Function to populate the combo box with directories sorted by date
void MainWindow::updateSavedCameraCalibrationFilesComboBox() {
    // Get the project directory path
    QString cal3DFilesDirectory = "../../CameraCalibration/";  // Replace with your main project folder path

    // Retrieve sorted directories
    QStringList sortedDirectories = getDirectoriesSortedByDate(cal3DFilesDirectory);

    // Clear the existing items in the combo box and populate it with the sorted list
    ui.savedCal3DFilesComboBox->clear();
    ui.savedCal3DFilesComboBox->addItems(sortedDirectories);
}

void MainWindow::indicateCameraCalibrationComplete(){

    ui.completedCameraCalibrationCheckBox->setChecked(true);
    ui.projectionErrorLabel->setText(QString::number(projectionError, 'f', 2) + " px");
}

void MainWindow::calibrateCameraWithSelectedFile() {
    // Get the selected directory from the combo box
    QString name = ui.savedCal3DFilesComboBox->currentText();
    if (name.isEmpty()) {
        qDebug() << "Calibration name is empty. Please enter a valid name.";
        return;
    }

    // Construct the path to the JSON file
    QString filePath = QString("../../CameraCalibration/%1/%1.json").arg(name);

    // Check if the file exists
    if (!QFile::exists(filePath)) {
        qDebug() << "File does not exist:" << filePath;
        return;
    }

    // Open the JSON file using an ifstream
    std::ifstream file(filePath.toStdString());
    if (!file.is_open()) {
        qDebug() << "Failed to open the JSON file:" << filePath;
        return;
    }

    // Parse the JSON file
    json j;
    file >> j;
    file.close();

    // Extract P1 and P2 matrices
    try {
        std::vector<std::vector<double>> P1_data = j.at("P1").get<std::vector<std::vector<double>>>();
        std::vector<std::vector<double>> P2_data = j.at("P2").get<std::vector<std::vector<double>>>();

        // Convert the extracted data to cv::Mat format
        P1 = cv::Mat(P1_data.size(), P1_data[0].size(), CV_64F);
        P2 = cv::Mat(P2_data.size(), P2_data[0].size(), CV_64F);
        for (int i = 0; i < P1.rows; ++i) {
            for (int j = 0; j < P1.cols; ++j) {
                P1.at<double>(i, j) = P1_data[i][j];
                P2.at<double>(i, j) = P2_data[i][j];
            }
        }

        // Extract projection error
        projectionError = j.at("projection_error").get<double>();

        qDebug() << "Successfully loaded P1, P2, and projectionError from" << filePath;
        qDebug() << "Projection error:" << projectionError;
        indicateCameraCalibrationComplete();
    } catch (json::exception &e) {
        qDebug() << "JSON parsing error:" << e.what();
    }
}

void MainWindow::resetNewCameraCalibration() {
    if (ui.newCal3DGroupBox->isEnabled()) {
        // Enable these widgets
        ui.newCal3DNameLineEdit->setEnabled(true);
        ui.newCal3DStartImageCaptureButton->setEnabled(true);

        // Disable these widgets
        ui.newCal3DStopImageCaptureButton->setEnabled(false);
        ui.newCal3DSaveImagePairButton->setEnabled(false);
        ui.calibrateNewCal3DButton->setEnabled(false);
        ui.newCal3DSaveToFileButton->setEnabled(false);
        // ui.cancelNewCal3DButton->setEnabled(false);
        ui.selectCal3DFromSavedFile->setEnabled(true);
        ui.chessRowsSpinBox->setEnabled(false);
        ui.chessColumnsSpinBox->setEnabled(false);
        ui.squareSideLengthSpinBox->setEnabled(false);

        ui.tabWidget->setEnabled(true);

        for(int i=0; i<ui.tabWidget->count(); ++i){
            ui.tabWidget->tabBar()->setTabEnabled(i,true);
        }

        //Reset image pair counter
        numberOfSavedImagePairs=0;

        // Update the label with the new count
        ui.numberOfSavedImagePairsLabel->setText(QString::number(numberOfSavedImagePairs));

        stopCapture();
    }
}

void MainWindow::newCameraCalibrationStartImageCapture() {

    processingType="thresh";

    numberOfSavedImagePairs = 0;
    // Update the label with the new count
    ui.numberOfSavedImagePairsLabel->setText(QString::number(numberOfSavedImagePairs));

    QString name = ui.newCal3DNameLineEdit->text().trimmed();

    // Check if name line edit is not empty
    if (name.isEmpty()) {
        QMessageBox::warning(this, "Invalid Name", "Please enter a name before starting image capture.");
        return;
    }

    // Disable the name line edit and start button, enable stop and save image pair buttons
    ui.newCal3DNameLineEdit->setEnabled(false);
    ui.newCal3DStartImageCaptureButton->setEnabled(false);
    ui.newCal3DStopImageCaptureButton->setEnabled(true);
    ui.newCal3DSaveImagePairButton->setEnabled(true);
    // ui.cancelNewCal3DButton->setEnabled(true);
    ui.selectCal3DFromSavedFile->setEnabled(false);
    ui.chessRowsSpinBox->setEnabled(true);
    ui.chessColumnsSpinBox->setEnabled(true);
    ui.squareSideLengthSpinBox->setEnabled(true);

    int currentTabIndex = ui.tabWidget->currentIndex();

    for(int i=0; i<ui.tabWidget->count(); ++i){

        if(i!=currentTabIndex){
            ui.tabWidget->tabBar()->setTabEnabled(i,false);
        }

    }

    // Create directories
    QString baseDir = "../../CameraCalibration/" + name;
    QDir().mkpath(baseDir);                // Creates calibration3d/NAME
    QDir().mkpath(baseDir + "/cam1");      // Creates calibration3d/NAME/left
    QDir().mkpath(baseDir + "/cam2");     // Creates calibration3d/NAME/right

    qDebug() << "Directories created for new calibration:" << baseDir;


    qDebug()<<"Starting camera stream";

    stopCapture();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    startCapture();

}

void MainWindow::newCameraCalibrationSaveImagePair() {
    if (!captureThread) {
        qDebug() << "Capture thread is not running. Cannot save image pairs.";
        return;
    }

    // Retrieve the latest frames from the capture thread
    cv::Mat leftImage = captureThread->getFrame1();
    cv::Mat rightImage = captureThread->getFrame2();

    if (leftImage.empty() || rightImage.empty()) {
        qDebug() << "One or both images are null. Skipping save.";
        return;
    }

    // Define directories and file paths for saving
    QString baseDir = "../../CameraCalibration/" + ui.newCal3DNameLineEdit->text().trimmed();
    QString leftDir = baseDir + "/cam1";
    QString rightDir = baseDir + "/cam2";

    // Ensure directories exist
    QDir().mkpath(leftDir);
    QDir().mkpath(rightDir);

    // Define file paths with incremental filenames
    QString leftImagePath = leftDir + "/" + QString::number(numberOfSavedImagePairs) + ".jpeg";
    QString rightImagePath = rightDir + "/" + QString::number(numberOfSavedImagePairs) + ".jpeg";

    // Save the images
    bool leftSaved = cv::imwrite(leftImagePath.toStdString(), leftImage);
    bool rightSaved = cv::imwrite(rightImagePath.toStdString(), rightImage);

    if (leftSaved && rightSaved) {
        // Increment the saved image pairs counter only if both saves are successful
        numberOfSavedImagePairs++;
        ui.numberOfSavedImagePairsLabel->setText(QString::number(numberOfSavedImagePairs));
        qDebug() << "Saved image pair number:" << numberOfSavedImagePairs;
        qDebug() << "Left image saved to:" << leftImagePath;
        qDebug() << "Right image saved to:" << rightImagePath;
    } else {
        qDebug() << "Error saving images. Left saved:" << leftSaved << ", Right saved:" << rightSaved;
    }
}


void MainWindow::newCal3DStopImageCapture() {
    // Enable the start capture button
    ui.newCal3DStartImageCaptureButton->setEnabled(true);

    // Disable the save image pair and stop capture buttons
    ui.newCal3DSaveImagePairButton->setEnabled(false);
    ui.newCal3DStopImageCaptureButton->setEnabled(false);

    ui.calibrateNewCal3DButton->setEnabled(true);


    stopCapture();

    qDebug() << "Image capture stopped";
}

void MainWindow::cancelNewCal3D() {
    // Get the name from the line edit
    QString name = ui.newCal3DNameLineEdit->text().trimmed();

    // Check if the name is valid (not empty)
    if (name.isEmpty()) {
        QMessageBox::warning(this, "Invalid Name", "Please enter a valid name in the line edit.");
        return;
    }

    // Define the target directory
    QString targetDirectory = "../../CameraCalibration/" + name;

    // Attempt to remove the directory
    QDir dir(targetDirectory);
    if (dir.exists()) {
        if (dir.removeRecursively()) {
            qDebug() << "Directory deleted:" << targetDirectory;
        } else {
            QMessageBox::warning(this, "Deletion Failed", "Failed to delete the directory.");
            qDebug() << "Failed to delete directory:" << targetDirectory;
            return;
        }
    } else {
        QMessageBox::information(this, "Directory Not Found", "The directory does not exist.");
        qDebug() << "Directory does not exist:" << targetDirectory;
    }

    // Reset the UI elements
    resetNewCameraCalibration();
}

void MainWindow::calibrateCameras() {
    // Get the name from the line edit
    QString name = ui.newCal3DNameLineEdit->text().trimmed();
    // Check if the name is valid (not empty)
    if (name.isEmpty()) {
        qDebug() << "Calibration name is empty. Please enter a valid name.";
        return;
    }
    else{
        std::string calDir = name.toStdString();

        int chessRows = ui.chessRowsSpinBox->value();
        int chessColumns = ui.chessColumnsSpinBox->value();
        double squareSideLength = ui.squareSideLengthSpinBox->value();

        calibrate(calDir, chessRows, chessColumns, squareSideLength);
        // Print calibration message to debug output
        qDebug() << "Calibrated" << name;
    }

    // Enable the "Save to File" button
    ui.newCal3DSaveToFileButton->setEnabled(true);
}

void MainWindow::setupCaptureThreadConnections() {
    if (!captureThread) return;

    connect(captureThread, &CameraCaptureThread::newFrameReady,
            this, &MainWindow::receiveAndProcessFrames);

    connect(captureThread, &CameraCaptureThread::stoppedCapture,
            ui.cameraStreamWidget, &CameraStreamWidget::setDefaultBlackFrame);
}


void MainWindow::startCapture() {

    if (!captureThread) {
        captureThread = new CameraCaptureThread(ui.cameraStreamWidget);
        qDebug() << "Started new thread";

        if (!captureThread->initializeCameras()) {
            qDebug() << "Cameras could not be initialized. Capture thread will not start.";
            delete captureThread;
            captureThread = nullptr;
            return;
        }

        // Set up the connections
        setupCaptureThreadConnections();
    }

    captureThread->startCapture();
    qDebug() << "Started capture";
    ui.startCameraCaptureThreadButton->setEnabled(false);
    ui.stopCameraCaptureThreadButton->setEnabled(true);
    qDebug() << "Disabled/enabled buttons";
}


void MainWindow::stopCapture() {

    if (captureThread) {
        captureThread->stopCapture();
        delete captureThread;
        captureThread = nullptr;
    }
    ui.stopCameraCaptureThreadButton->setEnabled(false);
    ui.startCameraCaptureThreadButton->setEnabled(true);
    ui.cameraStreamWidget->setDefaultBlackFrame();
}

void MainWindow::saveProcessedFrames() {
    if (!captureThread) {
        qDebug() << "Capture thread is not running. Cannot save frames.";
        return;
    }

    // Get the latest processed frames
    cv::Mat frame1, frame2;
    
    if (processingType == "None") {
        frame1 = captureThread->getFrame1().clone(); // Clone to ensure thread safety
        frame2 = captureThread->getFrame2().clone();
    } else {
        // If processing is enabled, try to get the latest processed frames
        frame1 = processedFrame1.clone();
        frame2 = processedFrame2.clone();
        
        // Fallback to raw frames if processed frames aren't available
        if (frame1.empty() || frame2.empty()) {
            frame1 = captureThread->getFrame1().clone();
            frame2 = captureThread->getFrame2().clone();
        }
    }

    if (frame1.empty() || frame2.empty()) {
        qDebug() << "One or both frames are empty. Cannot save.";
        return;
    }

    // Run the saving operation in a separate thread
    QtConcurrent::run([this, frame1, frame2]() {
        // Create a timestamp for unique filenames
        QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
        
        // Define directory for saving
        QString saveDir = "../../ProcessedFrames/";
        QDir().mkpath(saveDir);  // Create directory if it doesn't exist
        
        // Define file paths with timestamp
        QString frame1Path = saveDir + "frame1_" + timestamp + ".jpeg";
        QString frame2Path = saveDir + "frame2_" + timestamp + ".jpeg";
        
        // Save the frames
        bool frame1Saved = cv::imwrite(frame1Path.toStdString(), frame1);
        bool frame2Saved = cv::imwrite(frame2Path.toStdString(), frame2);
        
        if (frame1Saved && frame2Saved) {
            qDebug() << "Saved processed frames:";
            qDebug() << "Frame 1 saved to:" << frame1Path;
            qDebug() << "Frame 2 saved to:" << frame2Path;
        } else {
            qDebug() << "Error saving frames. Frame 1 saved:" << frame1Saved << ", Frame 2 saved:" << frame2Saved;
        }
    });
    
}

// THIS IS WHERE THE FRAME PROCESSING HAPPENS!!!!!!

void MainWindow::receiveAndProcessFrames(const cv::Mat &originalFrame1, const cv::Mat &originalFrame2) {


    if (processingType == "None") {
        // No processing; just update the frames
        QtConcurrent::run([this, originalFrame1, originalFrame2]() {
            ui.cameraStreamWidget->updateFrame(originalFrame1, originalFrame2, format);
        });
    } else if (processingType == "BGSub") {
        // Offload the processing to another thread
        // TODO: Figure out why BG Sub is so slow.
        QtConcurrent::run([this, originalFrame1, originalFrame2]() {
            // QElapsedTimer taskTimer;
            // taskTimer.start();
            
            // qint64 startTime1 = taskTimer.elapsed();
            // cv::Mat bgSub1 = SubtractBackground(originalFrame1, backSub, tmp1, tmpGray1, kernel);
            // qint64 elapsed1 = taskTimer.elapsed() - startTime1;
            
            // qDebug() << "Background subtraction timing:";   
            // qDebug() << "  First frame:" << elapsed1 << "ms";

            cv::Mat bgSub1 = SubtractBackground(originalFrame1, backSub, tmp1, tmpGray1, kernel);
            cv::Mat bgSub2 = SubtractBackground(originalFrame2, backSub, tmp2, tmpGray2, kernel);

            // Use Qt's signal-slot mechanism to update the GUI safely
            emit processedFramesReady(bgSub1, bgSub2);
        });


    }else if(processingType=="Thresh"){

        //Retrieve the current threshold value from the spinbox
        int thresholdValue = ui.processingThresholdingThresholdSpinBox->value();

        // Apply the threshold with the updated value
        cv::Mat thresholdedImage1 = ApplyThreshold(originalFrame1, thresholdValue, 255, cv::THRESH_BINARY);
        cv::Mat thresholdedImage2 = ApplyThreshold(originalFrame2, thresholdValue, 255, cv::THRESH_BINARY);

        // Made this do triangulation only once every second for readability and performance
        if (ui.processingThresholdingShowCentroidsCheckBox->isChecked() &&
            difftime(time(0), MainWindow::start)) {

            QtConcurrent::run([this, thresholdedImage1, thresholdedImage2]() {
                // Throttle processing
                MainWindow::start = time(0);

                std::vector<cv::Point> centroids1 = FindCentroids(thresholdedImage1);
                std::vector<cv::Point> centroids2 = FindCentroids(thresholdedImage2);

                qDebug() << "Image1 centroids count:" << static_cast<int>(centroids1.size());
                qDebug() << "Image2 centroids count:" << static_cast<int>(centroids2.size());

                // Case 1: No centroids found
                if (centroids1.empty() || centroids2.empty()) {
                    qDebug() << "No centroids found";
                }
                // Case 2: Exactly one centroid in each image: triangulate
                else if (centroids1.size() == 1 && centroids2.size() == 1) {
                    if (!P1.empty() && !P2.empty()) {
                        // cv::triangulatePoints expects 2xN matrices (of type CV_64F) for the points.
                        cv::Mat pts1 = (cv::Mat_<double>(2,1) << static_cast<double>(centroids1[0].x), static_cast<double>(centroids1[0].y));
                        cv::Mat pts2 = (cv::Mat_<double>(2,1) << static_cast<double>(centroids2[0].x), static_cast<double>(centroids2[0].y));

                        cv::Mat points4D;
                        cv::triangulatePoints(P1, P2, pts1, pts2, points4D);

                        // Convert from homogeneous coordinates (4×1) to Euclidean (3×1)
                        cv::Mat point3D = points4D.rowRange(0,3) / points4D.at<double>(3,0);
                        std::cout << "Triangulated 3D Point: " << point3D << std::endl;
                    } else {
                        qDebug() << "Projection matrices are empty, cannot triangulate.";
                    }
                }
                // Case 3: More than one centroid found: print all 2D locations
                else {
                    qDebug() << "Multiple centroids found in image1:";
                    for (const auto &pt : centroids1) {
                        qDebug() << "(" << pt.x << "," << pt.y << ")";
                    }
                    qDebug() << "Multiple centroids found in image2:";
                    for (const auto &pt : centroids2) {
                        qDebug() << "(" << pt.x << "," << pt.y << ")";
                    }
                }

                // Optionally, draw a red circle at the first centroid of each image (if available)
                if (!centroids1.empty())
                    cv::circle(thresholdedImage1, centroids1[0], 5, cv::Scalar(0, 0, 255), -1);
                if (!centroids2.empty())
                    cv::circle(thresholdedImage2, centroids2[0], 5, cv::Scalar(0, 0, 255), -1);

                emit processedFramesReady(thresholdedImage1, thresholdedImage2);
            });
        }


        emit processedFramesReady(thresholdedImage1, thresholdedImage2);
    } else if (processingType == "BD") {
        setBDValues();
        processImageCentroid(originalFrame1, originalFrame2, false);
    } else {
        emit processedFramesReady(originalFrame1, originalFrame2);
    }
}

void MainWindow::onProcessedFramesReady(const cv::Mat &processedFrame1, const cv::Mat &processedFrame2) {
    ui.cameraStreamWidget->updateFrame(processedFrame1, processedFrame2, format);
}



// // FRAME PROCESSING: this includes all the necessary processing for the different usecase.
// // OPENCV SHOULD ONLY BE USED IN THIS FUNCTION!!!
// void MainWindow::receiveAndProcessFrames(const cv::Mat &originalFrame1, const cv::Mat &originalFrame2) {

//     // if(performingMotorCameraCalibration){

//     //     // Retrieve the current threshold value from the spinbox
//     //     int thresholdValue = ui.thresholdSpinBox->value();

//     //     // Apply the threshold with the updated value
//     //     cv::Mat thresholdedImage1 = ApplyThreshold(originalFrame1, thresholdValue, 255, cv::THRESH_BINARY);
//     //     cv::Mat thresholdedImage2 = ApplyThreshold(originalFrame2, thresholdValue, 255, cv::THRESH_BINARY);

//     //     ui.cameraStreamWidget->updateFrame(thresholdedImage1, thresholdedImage2);

//     // }
//     // else{

//     //     ui.cameraStreamWidget->updateFrame(originalFrame1, originalFrame2);

//     // }

//     if(processingType=="None"){
//         ui.cameraStreamWidget->updateFrame(originalFrame1, originalFrame2);
//     }
//     else if(processingType=="BGSub"){

//         cv::Mat bgSub1 = SubtractBackground(originalFrame1,backSub);
//         cv::Mat bgSub2 = SubtractBackground(originalFrame2,backSub);

//         ui.cameraStreamWidget->updateFrame(bgSub1, bgSub2);

//     }
//     else{
//         ui.cameraStreamWidget->updateFrame(originalFrame1, originalFrame2);

//     }



// }

void MainWindow::setProcesseing() {

    if (ui.showProcessingNoneRadioButton->isChecked()) {
        processingType = "None";
    } else if (ui.showProcessingBGSubRadioButton->isChecked()) {
        //reset background subtractor
        backSub.dynamicCast<cv::BackgroundSubtractorMOG2>()->clear();
        processingType = "BGSub";
    } else if(ui.showProcessingThresholdingRadioButton->isChecked()){
        processingType="Thresh";
    } else if (ui.showProcessingBallDetectionRadioButton->isChecked()) {
        processingType = "BD";
    } else {
        processingType = "None"; // Default to "None" if neither checkbox is checked
    }

    if (processingType == "BD" || processingType == "BGSub" || processingType == "Thresh") {
        ui.setBackgroundImageButton->setEnabled(true);
        format = QImage::Format_Grayscale8;
    } else {
        ui.setBackgroundImageButton->setEnabled(false);
        format = QImage::Format_BGR888;
    }

    // Debug print or handle the processingType as needed
    qDebug() << "Processing Type set to:" << processingType;

    // Further logic to handle processingType
}

/**
 * @brief helper function for reordering the centroids for getLEDCoords
 * @param centroids
 * @return centroid locations in order of top, left, right
 */
std::vector<cv::Point> MainWindow::reorderCentroids(const std::vector<cv::Point>& centroids) {
    // Ensure the input has exactly 3 centroids
    if (centroids.size() != 3) {
        throw std::invalid_argument("The input vector must contain exactly 3 centroids.");
    }


    cv::Point left_centroid = centroids[0];
    for (const auto& centroid : centroids) {
        if (centroid.x < left_centroid.x) {
            left_centroid = centroid;
        }
    }

    // Step 2: Separate the remaining two centroids
    std::vector<cv::Point> remaining_centroids;
    for (const auto& centroid : centroids) {
        if (centroid != left_centroid) {
            remaining_centroids.push_back(centroid);
            if (remaining_centroids.size() > 2) {
                qDebug() << "Reordering error";
            }
        }
    }

    cv::Point top_centroid = remaining_centroids[0];
    cv::Point right_centroid = remaining_centroids[1];
    if (remaining_centroids[0].y > remaining_centroids[1].y) {
        std::swap(top_centroid, right_centroid);
    }

    // Step 4: Create the reordered list
    std::vector<cv::Point> reordered_centroids = {top_centroid, left_centroid, right_centroid};
    return reordered_centroids;
}

// Used to be from the old motor calibration - repurposed for spherical coordinate calibration
/**
 * @brief MainWindow::getLEDCoords
 * @return the 3D points of the spherical calibration LEDs in order of top, left, right
 */
std::vector<cv::Mat> MainWindow::getLEDCoords() {
    // Vector to store the result
    std::vector<cv::Mat> result;

    // Get the latest frames
    cv::Mat frame1 = captureThread->getFrame1();
    cv::Mat frame2 = captureThread->getFrame2();

    // Get the threshold value from the spinbox
    int thresholdValue = ui.processingThresholdingThresholdSpinBox->value();
    qDebug() << "Threshold used: " << thresholdValue << "\n";

    // Apply thresholding
    cv::Mat thresholdedImage1 = ApplyThreshold(frame1, thresholdValue, 255, cv::THRESH_BINARY);
    cv::imwrite("threshold1.jpg", thresholdedImage1);
    cv::Mat thresholdedImage2 = ApplyThreshold(frame2, thresholdValue, 255, cv::THRESH_BINARY);
    cv::imwrite("threshold2.jpg", thresholdedImage1);

    // Find the 3 largest contours
    cv::Mat largestContoursImage1 = FindLargestContours(thresholdedImage1, 3);
    cv::imwrite("contour1.jpg", largestContoursImage1);
    // qDebug() << "Camera 1 - found" <<
    cv::Mat largestContoursImage2 = FindLargestContours(thresholdedImage2, 3);
    cv::imwrite("contour2.jpg", largestContoursImage2);


    // Find centroids of the largest contours
    std::vector<cv::Point> centroids1 = FindCentroids(largestContoursImage1);
    std::vector<cv::Point> centroids2 = FindCentroids(largestContoursImage2);

    // Check if the number of centroids in camera 1 is exactly 3
    if (centroids1.size() != 3) {
        qDebug() << "Error: Camera 1 - Expected 3 centroids, but detected" << centroids1.size() << "centroid(s).";
        for (size_t i = 0; i < centroids1.size(); ++i) {
            qDebug() << "Camera 1 Centroid" << i + 1 << ": (" << centroids1[i].x << "," << centroids1[i].y << ")";
        }
        return result; // Return empty vector if the centroids are not valid
    }

    // Check if the number of centroids in camera 2 is exactly 3
    if (centroids2.size() != 3) {
        qDebug() << "Error: Camera 2 - Expected 3 centroids, but detected" << centroids2.size() << "centroid(s).";
        for (size_t i = 0; i < centroids2.size(); ++i) {
            qDebug() << "Camera 2 Centroid" << i + 1 << ": (" << centroids2[i].x << "," << centroids2[i].y << ")";
        }
        return result; // Return empty vector if the centroids are not valid
    }

    std::vector<cv::Point> reordered_centroids1 = MainWindow::reorderCentroids(centroids1);
    std::cout << reordered_centroids1 << std::endl;
    std::vector<cv::Point> reordered_centroids2 = MainWindow::reorderCentroids(centroids2);
    std::cout << reordered_centroids2 << std::endl;

    std::vector<cv::Point2f> point1vec;
    std::vector<cv::Point2f> point2vec;

    for (int i=0; i<3; i++) {
        cv::Point2f point1(reordered_centroids1[i].x, reordered_centroids1[i].y);
        cv::Point2f point2(reordered_centroids2[i].x, reordered_centroids2[i].y);
        point1vec.push_back(point1);
        point2vec.push_back(point2);

    }
    triangulate(point1vec, point2vec, P1, P2, result);

    return result;
}

void MainWindow::sphericalCalibration() {

    if(P1.empty() || P2.empty()) {
        qCritical() << "Cameras must be calibrated before doing spherical calibration";
        return;
    }

    std::vector<cv::Mat> LEDCoords = MainWindow::getLEDCoords();
    if (LEDCoords.empty()) return;
    cv::Mat topCoords = LEDCoords[0].reshape(1,3);
    std::cout << "Top: " << topCoords << std::endl;
    cv::Mat leftCoords = LEDCoords[1].reshape(1,3);
    std::cout << "Left: " << leftCoords << std::endl;
    cv::Mat rightCoords = LEDCoords[2].reshape(1,3);
    std::cout << "Right: " << rightCoords << std::endl;

    std::cout << "Difference between right and left coordinates: " << (rightCoords - leftCoords) << std::endl;
    std::cout << "Difference between top and right coordinates: " << (topCoords - rightCoords) << std::endl;

    std::cout << "x vector in inches: " << cv::norm(rightCoords - leftCoords)/25.4 << std::endl;

    std::cout << "y vector in inches: " << cv::norm(topCoords - rightCoords)/25.4 << std::endl;

    cv::Mat xDir = (rightCoords - leftCoords) / cv::norm(rightCoords - leftCoords);
    cv::Mat yDir = (topCoords - rightCoords) / cv::norm(topCoords - rightCoords);
    cv::Mat zDir = xDir.cross(yDir);

    try
    {
        sphericalOrigin = rightCoords + displacementRightLEDToOrigin;

    }
    catch (...)
    {
        qDebug() << "Error in spherical origin determination";
    }

    rotationMatrix = (cv::Mat_<double>(3,3) <<
                          xDir.at<double>(0,0), xDir.at<double>(1,0), xDir.at<double>(2,0),
                      yDir.at<double>(0,0), yDir.at<double>(1,0), yDir.at<double>(2,0),
                      zDir.at<double>(0,0), zDir.at<double>(1,0), zDir.at<double>(2,0)
                      );

    qDebug() << "Successfully completed spherical calibration";
}

void MainWindow::sphericalTest() {
    if (sphericalOrigin.empty() || rotationMatrix.empty()) {
        qCritical() << "Spherical calibration must be done first";
        return;
    }

    // Get the latest frames
    cv::Mat frame1 = captureThread->getFrame1();
    cv::Mat frame2 = captureThread->getFrame2();

    // Get the threshold value from the spinbox
    int thresholdValue = ui.thresholdSpinBox->value();

    // Apply thresholding
    cv::Mat thresholdedImage1 = ApplyThreshold(frame1, thresholdValue, 255, cv::THRESH_BINARY);
    cv::Mat thresholdedImage2 = ApplyThreshold(frame2, thresholdValue, 255, cv::THRESH_BINARY);

    // Find the largest contour
    cv::Mat largestContoursImage1 = FindLargestContours(thresholdedImage1, 1);
    cv::Mat largestContoursImage2 = FindLargestContours(thresholdedImage2, 1);

    // Find centroids of the largest contour
    std::vector<cv::Point> centroids1 = FindCentroids(largestContoursImage1);
    if (centroids1.size() != 1) {
        qDebug() << "No centroid found in camera 1";
        return;
    }
    std::vector<cv::Point> centroids2 = FindCentroids(largestContoursImage2);
    if (centroids2.size() != 1) {
        qDebug() << "No centroid found in camera 2";
        return;
    }

    // Find centroid in camera coordinates
    cv::Point2f point1(centroids1[0].x, centroids1[0].y);
    qDebug() << "Centroid 1: " << point1.x << ", " << point1.y;
    cv::Point2f point2(centroids2[0].x, centroids2[0].y);
    qDebug() << "Centroid 2: " << point2.x << ", " << point2.y;
    // cv::Mat targetCoordsCameraCoordinates = triangulatePoint(P1, P2, point1, point2);
    cv::Mat pts1 = (cv::Mat_<double>(2,1) << static_cast<double>(centroids1[0].x), static_cast<double>(centroids1[0].y));
    cv::Mat pts2 = (cv::Mat_<double>(2,1) << static_cast<double>(centroids2[0].x), static_cast<double>(centroids2[0].y));

    cv::Mat points4D;
    cv::triangulatePoints(P1, P2, pts1, pts2, points4D);

    // Convert from homogeneous coordinates (4×1) to Euclidean (3×1)
    cv::Mat targetCoordsCameraCoordinates = points4D.rowRange(0,3) / points4D.at<double>(3,0);


    cv::Mat targetCoordsSpherical = rotationMatrix * (targetCoordsCameraCoordinates - sphericalOrigin);
    double x = targetCoordsSpherical.at<double>(0,0);
    double y = targetCoordsSpherical.at<double>(1,0);
    double z = targetCoordsSpherical.at<double>(2,0);

    double normXY = std::sqrt(x * x + y * y);
    double phi = std::copysign(1.0, y) * std::acos(x / normXY);
    double optimalAzimuth = (phi - M_PI / 2.0) * 180.0 / M_PI;

    double normXYZ = std::sqrt(x * x + y * y + z * z);
    double theta = std::acos(z / normXYZ);
    double optimalAltitude = (M_PI / 2.0 - theta) * 180.0 / M_PI;

    qDebug() << "Target location in global coordinates: " << x << ", " << y << ", " << z << "\n";
    qDebug() << "Calculated azimuth: " << optimalAzimuth << "\n";
    qDebug() << "Calculated altitude: " << optimalAltitude << "\n";
    // TODO: move the motors to their desired angles

    // put the calculated angles into the spinboxes
    ui.aziValueSpinBox->setValue(optimalAzimuth);
    ui.altValueSpinBox->setValue(optimalAltitude);
    // send the angles to the motors
    sendMotorPositions();

}

#include <fstream> // Add this include for file operations

void MainWindow::altitudeTimeTest() {
    moveAltitudeMotor(altitudePointer, 22.5, 0);
    std::ofstream outFile("altitude_time_test_data_centered_jerk16.csv");
    outFile << "Angle,RpmLimit,TimeTaken\n"; // Write the header
    // std::vector<float> angles = {0.5, 2, 5, 10, 15, 20, 25, 30, 35, 40, 45};
    // for (float angle : angles) {
    for (float angle = 0; angle <= 46; angle += 5) {
        for (int rpmLimit = 10; rpmLimit <= 120; rpmLimit += 10) {
            double timeTaken = moveAltitudeMotor(altitudePointer, angle, rpmLimit);
            QThread::msleep(250);
            moveAltitudeMotor(altitudePointer, (float)22.5, 0);
            QThread::msleep(250);
            outFile << angle << "," << rpmLimit << "," << timeTaken << "\n"; // Write the data
            qDebug() << "Angle:" << angle << ", RpmLimit:" << rpmLimit << ", TimeTaken:" << timeTaken;
        }
    }

    outFile.close();
}


// ************** IGNORE FUNCTIONS BELOW *******************************

// The functions below use serial to communicate with the blue pill to
// control the original iteration of the motors.
// We're changing to a different motor setup, so these will (likely)
// no longer be relevant. But don't delete them just in case.


// void MainWindow::stopMotorCameraCalibration() {
//     qDebug() << "Stopping Motor-Camera calibration";

//     // Set the flag to signal the worker thread to stop
//     stopMotorCameraCalibrationFlag = true;

//     // Stop the camera capture
//     stopCapture();

//     // Wait for any ongoing calibration to complete
//     // (optional, only if you want to ensure the thread completes)
//     if (performingMotorCameraCalibration) {
//         qDebug() << "Waiting for calibration to stop...";
//     }
// }



void MainWindow::calibrateMotorCamera() {
    qDebug() << "Starting Motor-Camera calibration";

    startCapture();

    performingMotorCameraCalibration = true;

    // Reset the stop flag before starting calibration
    stopMotorCameraCalibrationFlag = false;

    // Run the entire calibration, including serial port opening, in a separate thread
    QFuture<void> future = QtConcurrent::run([this]() {
        QSerialPort localSerialPort; // Create a local serial port object for the thread
        localSerialPort.setPortName(serialPort->portName()); // Use the same port name
        localSerialPort.setBaudRate(serialPort->baudRate()); // Set the baud rate
        localSerialPort.setDataBits(serialPort->dataBits());
        localSerialPort.setParity(serialPort->parity());
        localSerialPort.setStopBits(serialPort->stopBits());
        localSerialPort.setFlowControl(serialPort->flowControl());

        // Attempt to open the serial port
        if (!localSerialPort.open(QIODevice::ReadWrite)) {
            QMetaObject::invokeMethod(this, [this]() {
                QMessageBox::critical(this, "Serial Port Error", "Cannot connect to Serial in the thread.");
                performingMotorCameraCalibration = false;
            });
            return;
        }

        // Perform the calibration loop
        for (quint16 aziValue = 100; aziValue <= 200 && !stopMotorCameraCalibrationFlag; aziValue += 20) {
            for (quint16 altValue = 100; altValue <= 200 && !stopMotorCameraCalibrationFlag; altValue += 20) {
                qDebug() << "Setting motor position: aziValue =" << aziValue << ", altValue =" << altValue;

                // Send the motor position commands via the local serial port
                queryMotorPosition(aziValue, altValue, localSerialPort);

                // Optional: Add a delay to ensure motor commands are not sent too rapidly
                QThread::msleep(1); // Adjust the delay as necessary

                // std::vector<cv::Point> LEDCoords = getLEDCoords(); // Continue with LED coordinate collection
            }
        }

        // Close the local serial port
        localSerialPort.close();

        // Reset the flag in the GUI thread
        QMetaObject::invokeMethod(this, [this]() {
            performingMotorCameraCalibration = false;
            qDebug() << "Motor-Camera calibration completed.";
        });
    });
}

void MainWindow::sendSerialMessage(QString baseMessage) {
    QString serialMessage = baseMessage + "\n";
    qint64 bytesWritten = serialPort->write(serialMessage.toUtf8());
    if (bytesWritten == -1) {
        qDebug() << "Error: Failed to write data to serial port.";
    } else {
        qDebug() << "Message sent successfully:" << serialMessage.trimmed();
    }
}

void MainWindow::fire() {
    sendSerialMessage("t");
}

void MainWindow::enableAzimuth() {
    sendSerialMessage("e");
}

void MainWindow::disableAzimuth() {
    sendSerialMessage("d");
}

void MainWindow::initializeAltitude() {
    altitudePointer = initializeAltitudeMotor();
}

void MainWindow::initializeAzimuth() {
    sendSerialMessage(QString::number(-1*azimuthPosition));
    azimuthPosition = 0;

}

void MainWindow::altitudeTest() {
    float absoluteAngle = ui.altValueSpinBox->value();
    int rpmLimit = ui.rpmLimitSpinBox->value();
    moveAltitudeMotor(altitudePointer, absoluteAngle, rpmLimit);
}

void MainWindow::enableAltitude(){

    enableAltitudeMotor(altitudePointer, true);

}

void MainWindow::disableAltitude(){

    enableAltitudeMotor(altitudePointer, false);

}

// -------- MOTOR - CAMERA CALIBRATION STUFF -------------------

void MainWindow::moveAlLimit()
{
    double altitudeLimit = ui.altitudeLimitSpinbox->value();
    qDebug() << "moving altitude to " <<altitudeLimit << "degrees";
    moveAltitudeMotor(altitudePointer, altitudeLimit, 0.0);
    QThread::msleep(250);

}


void MainWindow::moveAzLimit() {
    float absoluteAngle = ui.azimuthLimitSpinbox->value();
    int steps = static_cast<int>((absoluteAngle)/ 0.45) - azimuthPosition;
    sendSerialMessage(QString::number(steps));
    azimuthPosition += steps;
}


void MainWindow::setAzimuthLowerLimit(){
    azimuthCalLowerLimit = ui.azimuthLimitSpinbox->value();
    qDebug() << "Azimuth calibration lower limit set to: " << azimuthCalLowerLimit;
}

void MainWindow::setAzimuthUpperLimit(){
    azimuthCalUpperLimit = ui.azimuthLimitSpinbox->value();
    qDebug() << "Azimuth calibration upper limit set to: " << azimuthCalUpperLimit;
}

void MainWindow::setAltitudeLowerLimit(){
    altitudeCalLowerLimit = ui.altitudeLimitSpinbox->value();
    qDebug() << "Altitude calibration lower limit set to: " << altitudeCalLowerLimit;
}

void MainWindow::setAltitudeUpperLimit(){
    altitudeCalUpperLimit = ui.altitudeLimitSpinbox->value();
    qDebug() << "Altitude calibration upper limit set to: " << altitudeCalUpperLimit;
}


std::vector<double> MainWindow::linspace(double lower, double upper, int num_points) {
    std::vector<double> result;
    if (num_points <= 0)
        return result;
    if (num_points == 1) {
        result.push_back(lower);
        return result;
    }
    double step = (upper - lower) / (num_points - 1);
    for (int i = 0; i < num_points; ++i) {
        result.push_back(lower + i * step);
    }
    return result;
}



// Define a slot to perform each step of the sweep:
void MainWindow::performSweepStep() {
    if (currentAzIndex >= azimuthCalPositions.size()) {
        // Sweep is complete.
        sweepTimer->stop();
        sweepTimer->deleteLater();
        return;
    }

    // For this example, we assume you move azimuth only once per azimuth index.
    if (currentAlIndex == 0) {
        double az = azimuthCalPositions[currentAzIndex];
        qDebug() << "Moving azimuth to" << az << "degrees";
        int steps = static_cast<int>(az / 0.45) - azimuthPosition;
        sendSerialMessage(QString::number(steps));
        azimuthPosition += steps;
        // Let the event loop process events here naturally between timer ticks.
    }

    // Process altitude moves if needed:
    if (currentAlIndex < altitudeCalPositions.size()) {
        double al = altitudeCalPositions[currentAlIndex];
        qDebug() << "Moving altitude to" << al << "degrees";
        moveAltitudeMotor(altitudePointer, al, 0.0);
        currentAlIndex++;
    } else {
        // All altitude positions processed for the current azimuth; move to the next azimuth.
        currentAlIndex = 0;
        currentAzIndex++;
    }
}

void MainWindow::sweepLookupTable() {
    // Set up your positions and reset indices
    int numAzPoints = ui.numPointsAzimuthSpinbox->value();
    int numAlPoints = ui.numPointsAltitudeSpinbox->value();

    azimuthCalPositions = linspace(azimuthCalLowerLimit, azimuthCalUpperLimit, numAzPoints);
    altitudeCalPositions = linspace(altitudeCalLowerLimit, altitudeCalUpperLimit, numAlPoints);
    currentAzIndex = 0;
    currentAlIndex = 0;

    // Create and start a QTimer with an interval that suits your needs (e.g., 1000ms)
    sweepTimer = new QTimer(this);
    connect(sweepTimer, &QTimer::timeout, this, &MainWindow::performSweepStep);
    sweepTimer->start(1000); // interval in milliseconds
}




// -----------------------------------------------------------------


void MainWindow::azimuthTest() {
    float absoluteAngle = ui.aziValueSpinBox->value();
    int steps = static_cast<int>((absoluteAngle)/ 0.45) - azimuthPosition;
    sendSerialMessage(QString::number(steps));
    azimuthPosition += steps;
}


void MainWindow::sendMotorPositions() {
    azimuthTest();
    altitudeTest();
}



void MainWindow::homeMotors(){

    // if altitude not initialized
    //if(altitudePointer==nullptr){
    initializeAltitudeMotor();
    // sleep for 1 second
    QThread::msleep(250);
    moveAzimuthMotor(serialPort,0.0);
}

void MainWindow::queryMotorPosition(quint16 aziValue, quint16 altValue, QSerialPort &localSerialPort) {


    QByteArray command;
    command.append(static_cast<char>((aziValue >> 8) & 0xFF)); // Most significant byte
    command.append(static_cast<char>((aziValue) & 0xFF));
    command.append(static_cast<char>((altValue >> 8) & 0xFF));
    command.append(static_cast<char>(altValue & 0xFF));
    localSerialPort.write(command);

    // Wait for a response
    if (!localSerialPort.waitForReadyRead(2000)) { // Timeout set to 2000 ms (adjust as needed)
        QMessageBox::warning(this, "Serial Port Timeout", "No response received from the device within the timeout period.");
        return;
    }

    // Read all available data
    QByteArray response = localSerialPort.readAll();
    while (localSerialPort.waitForReadyRead(2000)) { // Keep reading if more data is available
        response += localSerialPort.readAll();
    }

    // Debug output to check the response
    if (response.size() == 4) {
        // Extract the first integer (X) from the first two bytes
        quint16 x = (static_cast<unsigned char>(response[0]) << 8) |
                    static_cast<unsigned char>(response[1]);

        // Extract the second integer (Y) from the last two bytes
        quint16 y = (static_cast<unsigned char>(response[2]) << 8) |
                    static_cast<unsigned char>(response[3]);

        qDebug() << "Received response as integers: " << x << "," << y;
    } else if (response.isEmpty()) {
        qDebug() << "Received an empty response.";
    } else {
        qDebug() << "Unexpected response size:" << response.size();
    }
}


void MainWindow::queryMotorPositionHardCode() {

    // Check if the serial port is already open
    if (!serialPort->isOpen()) {
        // Attempt to open the serial port
        if (!serialPort->open(QIODevice::ReadWrite)) {
            QMessageBox::critical(this, "Serial Port Error", "Cannot connect to Serial.");
            performingMotorCameraCalibration = false;
            return;
        }
    }


    quint16 aziValue = ui.aziValueSpinBox->value();
    quint16 altValue = ui.altValueSpinBox->value();
    QByteArray command;
    command.append(static_cast<char>((aziValue >> 8) & 0xFF)); // Most significant byte
    command.append(static_cast<char>((aziValue) & 0xFF));
    command.append(static_cast<char>((altValue >> 8) & 0xFF));
    command.append(static_cast<char>(altValue & 0xFF));
    serialPort->write(command);

    // Wait for a response
    if (!serialPort->waitForReadyRead(2000)) { // Timeout set to 2000 ms (adjust as needed)
        QMessageBox::warning(this, "Serial Port Timeout", "No response received from the device within the timeout period.");
        return;
    }

    // Read all available data
    QByteArray response = serialPort->readAll();
    while (serialPort->waitForReadyRead(2000)) { // Keep reading if more data is available
        response += serialPort->readAll();
    }

    // Debug output to check the response
    if (response.size() == 4) {
        // Extract the first integer (X) from the first two bytes
        quint16 x = (static_cast<unsigned char>(response[0]) << 8) |
                    static_cast<unsigned char>(response[1]);

        // Extract the second integer (Y) from the last two bytes
        quint16 y = (static_cast<unsigned char>(response[2]) << 8) |
                    static_cast<unsigned char>(response[3]);

        qDebug() << "Received response as integers: " << x << "," << y;
    } else if (response.isEmpty()) {
        qDebug() << "Received an empty response.";
    } else {
        qDebug() << "Unexpected response size:" << response.size();
    }
}

void MainWindow::setBackgroundImage() {
    if (processingType != "BD") return;

    cv::cvtColor(captureThread->getFrame1(), backgroundImage1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(captureThread->getFrame2(), backgroundImage2, cv::COLOR_BGR2GRAY);
    return;
}

void MainWindow::setBDValues() {
    motionEnabled = ui.processingMotionEnabled->isChecked();
    hsvEnabled = ui.processingHSVEnabled->isChecked();
    bgrEnabled = ui.processingBGREnabled->isChecked();
    morphEnabled = ui.processingMorphEnabled->isChecked();
    centroidEnabled = ui.processingCentroidEnabled->isChecked();
    drawEnabled = ui.processingDrawCentroidEnabled->isChecked();
    thresholdValue = ui.processingBDThresholdSpinBox->value();
    hMax = ui.processingHMax->value();
    hMin = ui.processingHMin->value();
    sMin = ui.processingSMin->value();
    sMax = ui.processingSMax->value();
    vMin = ui.processingVMin->value();
    vMax = ui.processingVMax->value();

    kernelSize = ui.processingMorphKernelSize->value();
    if (prevKernelSize != kernelSize) {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernelSize, kernelSize));
        prevKernelSize = kernelSize;
    } 

}

cv::Point3f MainWindow::processImageCentroid(const cv::Mat &originalFrame1, const cv::Mat &originalFrame2, bool timingEnabled) {
    setBDValues();
    if (originalFrame1.empty() || originalFrame2.empty()) {
        qDebug() << "One or both input frames are empty!";
        return cv::Point3f(-1, -1, -1);
    }
    if (timingEnabled) {
        // Use totalTimer when timing is enabled
        QtConcurrent::run([this, originalFrame1, originalFrame2]() {
            totalTimer->timeVoid([&]() {
                return processImages(originalFrame1, originalFrame2, true);
            });
        });
    } else {
        // Skip totalTimer when timing is not enabled
        QtConcurrent::run([this, originalFrame1, originalFrame2]() {
            return processImages(originalFrame1, originalFrame2, false);
        });
    }
}

cv::Point3f MainWindow::processImages(const cv::Mat &originalFrame1, const cv::Mat &originalFrame2, bool timingEnabled) {
    cv::Point3f point = cv::Point3f(-1, -1, -1);
    // Process image 1
    QFuture<void> futureOutput1 = QtConcurrent::run([&]() {
        processSingleImage(originalFrame1, output1, timingEnabled);
    });

    // Process image 2
    QFuture<void> futureOutput2 = QtConcurrent::run([&]() {
        processSingleImage(originalFrame2, output2, false);
    });

    // Wait for both outputs to be processed
    futureOutput1.waitForFinished();
    futureOutput2.waitForFinished();

    // Find centroids in the thresholded images
    std::vector<cv::Point> centroids1 = FindCentroids(output1);
    std::vector<cv::Point> centroids2 = FindCentroids(output2);

    if (drawEnabled && centroids1.size() == 1) DrawCentroidBinary(output1, centroids1[0]);
    if (drawEnabled && centroids2.size() == 1) DrawCentroidBinary(output2, centroids2[0]);

    if (centroids1.size() == 1 and centroids2.size() == 1) {
        // qDebug() << "Found singular centroid in both images";
        point = handleCentroids(centroids1, centroids2);
        qDebug() << "Centroid:" << point.x << point.y << point.z;
    }
    else {
        qDebug() << "No single centroid found";
    }

    processedFrame1 = output1.clone();
    processedFrame2 = output2.clone();  
    emit processedFramesReady(processedFrame1, processedFrame2);
    return point;
}

void MainWindow::processSingleImage(const cv::Mat &originalFrame, cv::Mat &output, bool timingEnabled) {
    if (timingEnabled) {
        // Apply HSV and BGR Thresholding with timers
        if (hsvEnabled || bgrEnabled) {
            if (bgrEnabled) {
                bgrTimer->timeVoid([&]() {
                    TestApplyBGRThreshold(originalFrame, tmp1, output, hMin, hMax, sMin, sMax, vMin, vMax);
                });
            }
            if (hsvEnabled) {
                hsvTimer->timeVoid([&]() {
                    ApplyHSVThreshold(originalFrame, tmp1, output, hMin, hMax, sMin, sMax, vMin, vMax);
                });
            }
        }

        // Apply Motion Thresholding with timers
        if (hsvEnabled && motionEnabled) {
            motionTimer->timeVoid([&]() {
                ApplyMotionThresholdConsecutively(originalFrame, tmpGray1, output, backgroundImage1, thresholdValue);
            });
        } else if (motionEnabled) {
            motionTimer->timeVoid([&]() {
                ApplyMotionThreshold(originalFrame, tmpGray1, output, backgroundImage1, thresholdValue);
            });
        }

        // Apply Morphological Closing with a timer
        if ((hsvEnabled || motionEnabled) && morphEnabled) {
            morphTimer->timeVoid([&]() {
                ApplyMorphClosing(output, kernel);
            });
        }
    } else {
        // If timers are not enabled, process normally (no timers)
        if (hsvEnabled || bgrEnabled) {
            if (bgrEnabled) {
                TestApplyBGRThreshold(originalFrame, tmp1, output, hMin, hMax, sMin, sMax, vMin, vMax);
            }
            if (hsvEnabled) {
                ApplyHSVThreshold(originalFrame, tmp1, output, hMin, hMax, sMin, sMax, vMin, vMax);
            }
        }

        if (hsvEnabled && motionEnabled) {
            ApplyMotionThresholdConsecutively(originalFrame, tmpGray1, output, backgroundImage1, thresholdValue);
        } else if (motionEnabled) {
            ApplyMotionThreshold(originalFrame, tmpGray1, output, backgroundImage1, thresholdValue);
        }

        if ((hsvEnabled || motionEnabled) && morphEnabled) {
            ApplyMorphClosing(output, kernel);
        }
    }
}

cv::Point3f MainWindow::handleCentroids(const std::vector<cv::Point> &centroids1, const std::vector<cv::Point> &centroids2) {
    if (centroids1.empty() || centroids2.empty()) {
        qDebug() << "No centroids found";
        return cv::Point3f(-1, -1, -1);
    }
    // Case 2: Exactly one centroid in each image: triangulate
    else if (centroids1.size() == 1 && centroids2.size() == 1) {
        if (!P1.empty() && !P2.empty()) {
            // Perform triangulation here
            cv::Mat pts1 = (cv::Mat_<double>(2,1) << static_cast<double>(centroids1[0].x), static_cast<double>(centroids1[0].y));
            cv::Mat pts2 = (cv::Mat_<double>(2,1) << static_cast<double>(centroids2[0].x), static_cast<double>(centroids2[0].y));

            cv::Mat points4D;
            cv::triangulatePoints(P1, P2, pts1, pts2, points4D);

            // Convert from homogeneous coordinates (4×1) to Euclidean (3×1)
            cv::Mat point3D = points4D.rowRange(0, 3) / points4D.at<double>(3, 0);
            // std::cout << "Triangulated 3D Point: " << point3D << std::endl;
            return cv::Point3f(point3D.at<double>(0), point3D.at<double>(1), point3D.at<double>(2));
        } else {
            qDebug() << "Projection matrices are empty, cannot triangulate.";
            return cv::Point3f(-1, -1, -1);
        }
    }
}
