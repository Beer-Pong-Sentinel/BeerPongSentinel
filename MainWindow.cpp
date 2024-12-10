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

using json = nlohmann::json;

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent), captureThread(nullptr), serialPort(new QSerialPort(this)) {
    ui.setupUi(this);  // Sets up the UI and initializes the widgets
    qDebug()<<"setup ui";
    this->adjustSize();
    // showMaximized();


    // Serial Port

    // Configure the serial port (modify these settings as necessary for your device)

    serialPort->setPortName("COM4");
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
    connect(ui.savedCal3DFilesRefreshButton, &QPushButton::clicked, this, &MainWindow::updateSavedCameraCalibrationFilesComboBox);
    connect(ui.calibrateCal3DFromLoadedFile, &QPushButton::clicked, this, &MainWindow::calibrateCameraWithSelectedFile);
    connect(ui.newCal3DStartImageCaptureButton, &QPushButton::clicked, this, &MainWindow::newCameraCalibrationStartImageCapture);
    connect(ui.newCal3DSaveImagePairButton, &QPushButton::clicked, this, &MainWindow::newCameraCalibrationSaveImagePair);
    connect(ui.newCal3DStopImageCaptureButton, &QPushButton::clicked, this, &MainWindow::newCal3DStopImageCapture);
    connect(ui.cancelNewCal3DButton, &QPushButton::clicked, this, &MainWindow::cancelNewCal3D);
    connect(ui.calibrateNewCal3DButton, &QPushButton::clicked, this, &MainWindow::calibrateCameras);
    connect(ui.startCameraCaptureThreadButton, &QPushButton::clicked, this, &MainWindow::startCapture);
    connect(ui.stopCameraCaptureThreadButton, &QPushButton::clicked, this, &MainWindow::stopCapture);
    connect(ui.startMotorCameraCalibrationButton, &QPushButton::clicked, this, &MainWindow::calibrateMotorCamera);
    connect(ui.sendMotorPositionButton, &QPushButton::clicked, this, &MainWindow::queryMotorPositionHardCode);



    qDebug()<<"connected thread";

    updateSavedCameraCalibrationFilesComboBox();


}

MainWindow::~MainWindow() {
    // Cleanup if necessary
    stopCapture();
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
        ui.cancelNewCal3DButton->setEnabled(false);
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
    ui.cancelNewCal3DButton->setEnabled(true);
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








// FRAME PROCESSING
void MainWindow::receiveAndProcessFrames(const cv::Mat &originalFrame1, const cv::Mat &originalFrame2) {


    if(performingMotorCameraCalibration){

        // Retrieve the current threshold value from the spinbox
        int thresholdValue = ui.thresholdSpinBox->value();

        // Apply the threshold with the updated value
        cv::Mat thresholdedImage1 = ApplyThreshold(originalFrame1, thresholdValue, 255, cv::THRESH_BINARY);
        cv::Mat thresholdedImage2 = ApplyThreshold(originalFrame2, thresholdValue, 255, cv::THRESH_BINARY);

        ui.cameraStreamWidget->updateFrame(thresholdedImage1, thresholdedImage2);

    }
    else{

        ui.cameraStreamWidget->updateFrame(originalFrame1, originalFrame2);

    }


}

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

                std::vector<cv::Point> LEDCoords = getLEDCoords(); // Continue with LED coordinate collection
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


std::vector<cv::Point> MainWindow::getLEDCoords() {
    // Vector to store the result
    std::vector<cv::Point> result;

    // Get the latest frames
    cv::Mat frame1 = captureThread->getFrame1();
    cv::Mat frame2 = captureThread->getFrame2();

    // Get the threshold value from the spinbox
    int thresholdValue = ui.thresholdSpinBox->value();

    // Apply thresholding
    cv::Mat thresholdedImage1 = ApplyThreshold(frame1, thresholdValue, 255, cv::THRESH_BINARY);
    cv::Mat thresholdedImage2 = ApplyThreshold(frame2, thresholdValue, 255, cv::THRESH_BINARY);

    // Find the 2 largest contours
    cv::Mat largestContoursImage1 = FindLargestContours(thresholdedImage1, 2);
    cv::Mat largestContoursImage2 = FindLargestContours(thresholdedImage2, 2);

    // Find centroids of the largest contours
    std::vector<cv::Point> centroids1 = FindCentroids(largestContoursImage1);
    std::vector<cv::Point> centroids2 = FindCentroids(largestContoursImage2);

    // Check if the number of centroids in camera 1 is exactly 2
    if (centroids1.size() != 2) {
        qDebug() << "Error: Camera 1 - Expected 2 centroids, but detected" << centroids1.size() << "centroid(s).";
        for (size_t i = 0; i < centroids1.size(); ++i) {
            qDebug() << "Camera 1 Centroid" << i + 1 << ": (" << centroids1[i].x << "," << centroids1[i].y << ")";
        }
        return result; // Return empty vector if the centroids are not valid
    }

    // Check if the number of centroids in camera 2 is exactly 2
    if (centroids2.size() != 2) {
        qDebug() << "Error: Camera 2 - Expected 2 centroids, but detected" << centroids2.size() << "centroid(s).";
        for (size_t i = 0; i < centroids2.size(); ++i) {
            qDebug() << "Camera 2 Centroid" << i + 1 << ": (" << centroids2[i].x << "," << centroids2[i].y << ")";
        }
        return result; // Return empty vector if the centroids are not valid
    }

    // Log detected centroids
    qDebug() << "Camera 1 Centroid 1: (" << centroids1[0].x << "," << centroids1[0].y << ")";
    qDebug() << "Camera 1 Centroid 2: (" << centroids1[1].x << "," << centroids1[1].y << ")";
    qDebug() << "Camera 2 Centroid 1: (" << centroids2[0].x << "," << centroids2[0].y << ")";
    qDebug() << "Camera 2 Centroid 2: (" << centroids2[1].x << "," << centroids2[1].y << ")";

    // Add the centroids to the result vector
    result.push_back(centroids1[0]); // Camera 1 Centroid 1
    result.push_back(centroids1[1]); // Camera 1 Centroid 2
    result.push_back(centroids2[0]); // Camera 2 Centroid 1
    result.push_back(centroids2[1]); // Camera 2 Centroid 2

    return result;
}




