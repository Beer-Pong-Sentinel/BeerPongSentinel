#include "CameraCaptureThread.h"
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
#include <cmath>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/LevenbergMarquardt>
#include <vector>
#include <cmath>
#include "YOLOInferenceWorker.h"
#include <QSerialPortInfo>
//#include <QmlDebuggingEnabler>

using json = nlohmann::json;

//QQmlDebuggingEnabler enabler;

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent), captureThread(nullptr), serialPort(new QSerialPort(this)) {
    ui.setupUi(this);  // Sets up the UI and initializes the widgets
    qDebug()<<"setup ui";
    this->adjustSize();
    // showMaximized();





    connect(ui.selectCal3DFromSavedFile, &QRadioButton::toggled, this, &MainWindow::toggleCal3DType);
    connect(ui.selectNewCal3D, &QRadioButton::toggled, this, &MainWindow::toggleCal3DType);

    connect(ui.selectMoCamCalFromSavedFile, &QRadioButton::toggled, this, &MainWindow::toggleMotorCameraCalType);
    connect(ui.selectNewMoCamCal, &QRadioButton::toggled, this, &MainWindow::toggleMotorCameraCalType);

    connect(ui.savedCal3DFilesRefreshButton, &QPushButton::clicked, this, &MainWindow::updateSavedCameraCalibrationFilesComboBox);
    connect(ui.savedMoCamFilesRefreshButton, &QPushButton::clicked, this, &MainWindow::updateSavedMotorCameraCalibrationFilesComboBox);


    connect(ui.calibrateCal3DFromLoadedFile, &QPushButton::clicked, this, &MainWindow::calibrateCameraWithSelectedFile);
    connect(ui.calibrateMoCamFromLoadedFile, &QPushButton::clicked, this, &MainWindow::calibrateMotorCameraWithSelectedFile);


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

    connect(ui.goToAlLimitButton, &QPushButton::clicked, this, &MainWindow::moveAlLimit);
    connect(ui.goToAzLimitButton, &QPushButton::clicked, this, &MainWindow::moveAzLimit);

    connect(ui.setLowerAzLimit, &QPushButton::clicked, this, &MainWindow::setAzimuthLowerLimit);
    connect(ui.setUpperAzLimit, &QPushButton::clicked, this, &MainWindow::setAzimuthUpperLimit);
    connect(ui.setLowerAlLimit, &QPushButton::clicked, this, &MainWindow::setAltitudeLowerLimit);
    connect(ui.setUpperAlLimit, &QPushButton::clicked, this, &MainWindow::setAltitudeUpperLimit);

    connect(ui.startMotorCameraCalibrationButton, &QPushButton::clicked, this, &MainWindow::sweepLookupTable);

    connect(ui.farSweepRadioButton, &QRadioButton::toggled, this, &MainWindow::toggleNearFarSweep);
    connect(ui.nearSweepRadioButton, &QRadioButton::toggled, this, &MainWindow::toggleNearFarSweep);


    connect(ui.calculateLookupTableButton, &QPushButton::clicked, this, &MainWindow::getLookupTable);

    connect(ui.numPointsAzimuthSpinbox, QOverload<int>::of(&QSpinBox::valueChanged),
            [this](int newValue) {
                ui.numAzInterpPointsSpinBox->setMinimum(newValue);
            });

    connect(ui.numPointsAltitudeSpinbox, QOverload<int>::of(&QSpinBox::valueChanged),
            [this](int newValue) {
                ui.numAlInterpPointsSpinBox->setMinimum(newValue);
            });


    connect(ui.aimButton, &QPushButton::clicked, this, &MainWindow::aimAtCentroid);
    connect(ui.mcAimButton, &QPushButton::clicked, this, &MainWindow::aimAtCentroid);
    //connect(ui.startContinuousAimButton, &QPushButton::clicked, this, &MainWindow::aimContinuous);
    //connect(ui.stopContinuousAimButton, &QPushButton::clicked, this, &MainWindow::stopContinuousAim);

    connect(ui.startCentroidPushButton, &QPushButton::clicked, this, &MainWindow::toggleCaptureCentroid);
    connect(ui.saveCentroidPushButton, &QPushButton::clicked, this, &MainWindow::saveCentroidListToJson);

    connect(ui.startPredictionPushButton, &QPushButton::clicked, this, &MainWindow::togglePrediction);

    connect(this, &MainWindow::predictionDataReady, this, &MainWindow::runPrediction);



    qDebug()<<"connected thread";

    updateSavedCameraCalibrationFilesComboBox();
    updateSavedMotorCameraCalibrationFilesComboBox();

    sweepData = nlohmann::json::object();
    sweepData["sweep"] = nlohmann::json::array();
    d_kernel.upload(kernel_cpu);

    try {
        yoloNet = cv::dnn::readNetFromONNX("../../Models/best.onnx");

        // Force GPU usage if available; otherwise, it will fall back to CPU.
        // Uncomment the following lines if you have CUDA built into OpenCV:
        yoloNet.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        yoloNet.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

        qDebug() << "YOLO model loaded successfully.";

        // Otherwise, you can use:
        // yoloNet.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
        // yoloNet.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    } catch (const cv::Exception &e) {
        qDebug() << "Error loading YOLO model:" << e.what();
    }

    yoloWorker = new YOLOInferenceWorker(yoloNet, yoloInputSize, confThreshold);
    yoloThread = new QThread(this);
    yoloWorker->moveToThread(yoloThread);
    connect(yoloThread, &QThread::started, yoloWorker, &YOLOInferenceWorker::processQueue);
    connect(yoloWorker, &YOLOInferenceWorker::inferenceComplete, this, &MainWindow::handleInferenceComplete);
    yoloThread->start();

    alConfigData = parseAltitudeConfig();
    azConfigData = parseAzimuthConfig();

    qDebug() << "Loaded motor configs";

    setMotorGUILimits();
    refreshMotorPorts();
    connect(ui.refreshPortsButton, &QPushButton::clicked, this, &MainWindow::refreshMotorPorts);
    connect(ui.connectMotorsButton, &QPushButton::clicked, this, &MainWindow::connectMotors);
    connect(ui.stopMotorCameraCalibrationButton, &QPushButton::clicked, this, &MainWindow::stopSweep);





}

MainWindow::~MainWindow() {
    // Cleanup if necessary
    stopCapture();
    disableAltitude();
    // not gonna work lol
    disableAzimuth();


}

void MainWindow::setMotorGUILimits(){

    ui.azimuthLimitSpinbox->setMinimum(azConfigData.minAzAngle);
    ui.azimuthLimitSpinbox->setMaximum(azConfigData.maxAzAngle);

    ui.altitudeLimitSpinbox->setMinimum(alConfigData.minAlAngle);
    ui.altitudeLimitSpinbox->setMaximum(alConfigData.maxAlAngle);

}

void MainWindow::refreshMotorPorts() {
    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();

    // Optionally reset variables (using -1 to indicate not found)
    altitudeCOMNum = -1;
    azimuthCOMNum = -1;

    // Optionally clear the combo boxes to remove previous entries
    ui.altitudePortComboBox->clear();
    ui.azimuthoPortComboBox->clear();

    for (const QSerialPortInfo &port : ports) {
        QString description = port.description().toLower();
        QString manufacturer = port.manufacturer().toLower();
        QString portName = port.portName();  // typically "COMxx"
        int comNumber = portName.remove("COM", Qt::CaseInsensitive).toInt();

        // Check for Teknic SC Hub port
        // We look for "teknic sc hub" or the combination of "teknic" and "hub"
        if (description.contains("teknic sc hub") ||
            (description.contains("teknic") && description.contains("hub")) ||
            manufacturer.contains("teknic")) {
            altitudeCOMNum = comNumber;
            qDebug() << "Found Teknic SC Hub device:";
            qDebug() << "  Description:" << port.description();
            qDebug() << "  Port:" << "COM" + QString::number(altitudeCOMNum);
            qDebug() << "  altitudeCOMNum:" << altitudeCOMNum;
            ui.altitudePortComboBox->addItem("COM" + QString::number(altitudeCOMNum));
        }
        // Check for a serial communication port (if not already identified as Teknic)
        else if (description.contains("serial") || manufacturer.contains("usb serial")) {
            azimuthCOMNum = comNumber;
            qDebug() << "Found Serial Communication device:";
            qDebug() << "  Description:" << port.description();
            qDebug() << "  Port:" << "COM" + QString::number(azimuthCOMNum);
            qDebug() << "  azimuthCOMNum:" << azimuthCOMNum;
            ui.azimuthoPortComboBox->addItem("COM" + QString::number(azimuthCOMNum));
        }
    }

    // If a COM port is not found for altitude, disable its combo box
    if (altitudeCOMNum == -1) {
        ui.altitudePortComboBox->setEnabled(false);
        qDebug() << "No Teknic SC Hub device found. Disabling altitudePortComboBox.";
    } else {
        ui.altitudePortComboBox->setEnabled(true);
        ui.connectMotorsButton->setEnabled(true);
    }

    // If a COM port is not found for azimuth, disable its combo box
    if (azimuthCOMNum == -1) {
        ui.azimuthoPortComboBox->setEnabled(false);
        qDebug() << "No Serial Communication device found. Disabling azimuthoPortComboBox.";
    } else {
        ui.azimuthoPortComboBox->setEnabled(true);
        ui.connectMotorsButton->setEnabled(true);
    }

    // if both are not available
    if (azimuthCOMNum == -1 && altitudeCOMNum== -1){

        ui.connectMotorsButton->setEnabled(false);

    }

}


void MainWindow::connectMotors(){

    // Connect azimuth first

    // Serial Port

    // Configure the serial port (modify these settings as necessary for your device)

    if (azimuthCOMNum!=-1){
        serialPort->setPortName("COM" + QString::number(azimuthCOMNum));

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


        // enable corresponding controls
        ui.mcEnableAzimuthButton->setEnabled(true);
        ui.mcDisableAzimuthButton->setEnabled(true);
        ui.mcAzimuthAngleSpinBox->setEnabled(true);
        ui.mcMoveAzimuthButton->setEnabled(true);

    }
    else{
        qDebug() << "No Serial port found so can't connect azimuth";
    }

    if (altitudeCOMNum!=-1){
        initializeAltitude(altitudeCOMNum);

        ui.mcEnableAltitudeButton->setEnabled(true);
        ui.mcDisableAltitudeButton->setEnabled(true);
        ui.mcAltitudeAngleSpinBox->setEnabled(true);
        ui.mcMoveAltitudeButton->setEnabled(true);

    } else{

        qDebug() << "No SC Hub found so can't connect altitude";
    }

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
        resetMotorCameraCalibration();
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

// void MainWindow::indicateCameraCalibrationComplete(){

//     ui.completedCameraCalibrationCheckBox->setChecked(true);
//     ui.projectionErrorLabel->setText(QString::number(projectionError, 'f', 2) + " px");
// }

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
        //indicateCameraCalibrationComplete();
        // TODO: Fix the checkLED command crashing calibration when cameras are not capturing.
        // checkLEDDistances();
    } catch (json::exception &e) {
        qDebug() << "JSON parsing error:" << e.what();
    }
}

void MainWindow::updateSavedMotorCameraCalibrationFilesComboBox() {
    // Get the project directory path
    QString cal3DFilesDirectory = "../../MotorCameraCalibration/";  // Replace with your main project folder path

    // Retrieve sorted directories
    QStringList sortedDirectories = getDirectoriesSortedByDate(cal3DFilesDirectory);

    // Clear the existing items in the combo box and populate it with the sorted list
    ui.savedMoCamFilesComboBox->clear();
    ui.savedMoCamFilesComboBox->addItems(sortedDirectories);
}

void MainWindow::calibrateMotorCameraWithSelectedFile() {
    // Get the selected directory from the combo box
    QString name = ui.savedMoCamFilesComboBox->currentText();
    if (name.isEmpty()) {
        qDebug() << "Calibration name is empty. Please enter a valid name.";
        return;
    }

    // Construct the path to the JSON file
    QString filePath = QString("../../MotorCameraCalibration/%1/interpolated_lookuptable.json").arg(name);

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
    nlohmann::json j;
    try {
        file >> j;
    } catch (const std::exception &e) {
        qDebug() << "Error parsing JSON:" << e.what();
        return;
    }
    file.close();

    // Load the interpolated lookup table from the JSON.
    if (!j.contains("lookuptable") || !j["lookuptable"].is_array()) {
        qDebug() << "Error: JSON file does not contain a valid 'lookuptable' array.";
        return;
    }

    std::vector<LookupEntry> loadedLookupTable;
    auto lookupArray = j["lookuptable"];
    for (const auto &entry : lookupArray) {
        // Expect each row to have exactly 8 values: [az, al, xo, yo, zo, ux, uy, uz]
        if (!entry.is_array() || entry.size() < 8) {
            qDebug() << "Warning: A lookup table entry does not have enough elements. Skipping.";
            continue;
        }
        LookupEntry lookupEntry;
        try {
            lookupEntry.az = entry[0].get<double>();
            lookupEntry.al = entry[1].get<double>();
            lookupEntry.xo = entry[2].get<double>();
            lookupEntry.yo = entry[3].get<double>();
            lookupEntry.zo = entry[4].get<double>();
            lookupEntry.ux = entry[5].get<double>();
            lookupEntry.uy = entry[6].get<double>();
            lookupEntry.uz = entry[7].get<double>();
        } catch (const std::exception &e) {
            qDebug() << "Error converting lookup entry:" << e.what();
            continue;
        }
        loadedLookupTable.push_back(lookupEntry);
    }

    // Save the loaded lookup table into the class variable.
    this->interpolatedLookupTable = loadedLookupTable;
    qDebug() << "Loaded interpolated lookup table with" << loadedLookupTable.size() << "entries.";

    // Print out the loaded lookup table entries.
    // for (const auto &entry : loadedLookupTable) {
    //     qDebug() << "Lookup Entry:";
    //     qDebug() << "  az:" << entry.az << ", al:" << entry.al;
    //     qDebug() << "  xo:" << entry.xo << ", yo:" << entry.yo << ", zo:" << entry.zo;
    //     qDebug() << "  ux:" << entry.ux << ", uy:" << entry.uy << ", uz:" << entry.uz;
    // }

}




void MainWindow::resetMotorCameraCalibration() {
    if (ui.newMoCamCalGroupbox->isEnabled()) {
        // Enable these widgets
        ui.newMoCamCalLineEdit->setEnabled(true);
        ui.startMotorCameraCalibrationButton->setEnabled(true);

        // Disable these widgets
        ui.stopMotorCameraCalibrationButton->setEnabled(false);
        ui.nearSweepDoneCheckBox->setEnabled(false);
        ui.farSweepDoneCheckBox->setEnabled(false);
        ui.calculateLookupTableButton->setEnabled(false);
        ui.setAimAlRPMLimitSpinBox->setEnabled(false);
        ui.numAzInterpPointsSpinBox->setEnabled(false);
        ui.numAlInterpPointsSpinBox->setEnabled(false);

        //uncheck sweep completed boxes
        ui.nearSweepRadioButton->setChecked(false);
        ui.farSweepDoneCheckBox->setChecked(false);

        // redo radio buttons
        ui.nearSweepRadioButton->setChecked(true);
        ui.farSweepRadioButton->setChecked(false);

        stopCapture();
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

    //processingType="thresh";

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

void MainWindow::receiveAndProcessFrames(const cv::Mat &originalFrame1, const cv::Mat &originalFrame2, double timestamp) {


    if (processingType == "None") {
        // No processing; just update the frames
        QtConcurrent::run([this, originalFrame1, originalFrame2]() {
            ui.cameraStreamWidget->updateFrame(originalFrame1.clone(), originalFrame2.clone(), format);
        });
    } else if (processingType == "BGSub") {
        QtConcurrent::run([this, originalFrame1, originalFrame2, timestamp]() {
            cv::Mat bgSub1, bgSub2;
            cv::Point centroid1, centroid2;
            // Measure the time for background subtraction and centroid detection
            totalTimer->timeVoid([&]() {
                auto result1 = getBGSubCentroid(originalFrame1, backSubCUDA, d_fgMask1, d_tmpGray1, d_kernel);
                bgSub1 = result1.first;
                centroid1 = result1.second;

                auto result2 = getBGSubCentroid(originalFrame2, backSubCUDA, d_fgMask2, d_tmpGray2, d_kernel);
                bgSub2 = result2.first;
                centroid2 = result2.second;
            });

            if (!P1.empty() && !P2.empty()) {

                if (centroid1.x != -1 && centroid1.y != -1){
                    // Triangulate the 3D point from the centroids
                    cv::Mat pts1 = (cv::Mat_<double>(2, 1) << static_cast<double>(centroid1.x), static_cast<double>(centroid1.y));
                    cv::Mat pts2 = (cv::Mat_<double>(2, 1) << static_cast<double>(centroid2.x), static_cast<double>(centroid2.y));

                    cv::Mat points4D;
                    cv::triangulatePoints(P1, P2, pts1, pts2, points4D);

                    cv::Mat point3D = points4D.rowRange(0, 3) / points4D.at<double>(3, 0);
                    cv::Point3f point3f(point3D.at<double>(0), point3D.at<double>(1), point3D.at<double>(2));
                    updateCentroid(point3f, timestamp);
                    //std::cout << "Triangulated 3D Point Thresholded: " << point3f.x << " " << point3f.y << " " << point3f.z << std::endl;
                } else {
                    qDebug() << "no ball centroid found";
                }
            } else {
                qDebug() << "Projection matrices are empty, cannot triangulate.";
            }

            // Convert the 1-channel foreground masks to 3-channel BGR images before drawing
            if (bgSub1.channels() == 1)
                cv::cvtColor(bgSub1, bgSub1, cv::COLOR_GRAY2BGR);
            if (bgSub2.channels() == 1)
                cv::cvtColor(bgSub2, bgSub2, cv::COLOR_GRAY2BGR);

            // Draw centroids in red if valid (red in BGR is (0,0,255))
            if (centroid1.x != -1 && centroid1.y != -1) {
                cv::circle(bgSub1, centroid1, 5, cv::Scalar(0, 0, 255), -1);
            }
            if (centroid2.x != -1 && centroid2.y != -1) {
                cv::circle(bgSub2, centroid2, 5, cv::Scalar(0, 0, 255), -1);
            }
            // Emit the processed frames to update the UI
            emit processedFramesReady(bgSub1, bgSub2);
        });
    }else if(processingType=="Thresh"){

        //qDebug() << "In Thresh";

        //Retrieve the current threshold value from the spinbox
        int thresholdValue = ui.processingThresholdingThresholdSpinBox->value();

        // Apply the threshold with the updated value
        cv::Mat thresholdedImage1 = ApplyThreshold(originalFrame1, thresholdValue, 255, cv::THRESH_BINARY);
        cv::Mat thresholdedImage2 = ApplyThreshold(originalFrame2, thresholdValue, 255, cv::THRESH_BINARY);

        // Made this do triangulation only once every second for readability and performance
        if (ui.processingThresholdingShowCentroidsCheckBox->isChecked() &&
            difftime(time(0), MainWindow::start)) {

            QtConcurrent::run([this, thresholdedImage1, thresholdedImage2, timestamp]() {
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
                        cv::Point3f point3f = cv::Point3f(point3D.at<double>(0), point3D.at<double>(1), point3D.at<double>(2));
                        updateCentroid(point3f, timestamp);
                        std::cout << "Triangulated 3D Point Thresholded: " << point3f.x <<" "<< point3f.y<< " " << point3f.z << std::endl;

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
        QtConcurrent::run([this, originalFrame1, originalFrame2, timestamp]() {
            updateCentroid(processImageCentroidCUDA(originalFrame1, originalFrame2, true), timestamp);
        });
        //qDebug() << "Centroid in receive and process frames: "<< motorCameraCalibrationCurrentCentroid.x << motorCameraCalibrationCurrentCentroid.y << motorCameraCalibrationCurrentCentroid.z ;
    }
    else if (processingType =="YOLO"){


        FramePair pair;
        pair.frame1 = originalFrame1.clone();
        pair.frame2 = originalFrame2.clone();
        yoloWorker->enqueueFramePair(pair, timestamp);


    }
    else {
        emit processedFramesReady(originalFrame1, originalFrame2);
    }
}

void MainWindow::onProcessedFramesReady(const cv::Mat &processedFrame1, const cv::Mat &processedFrame2) {
    ui.cameraStreamWidget->updateFrame(processedFrame1, processedFrame2, format);
}

// void MainWindow::handleInferenceComplete(const cv::Mat &processedFrame1, const cv::Mat &processedFrame2,
//                                          cv::Point centroid1, cv::Point centroid2, double timestamp)
// {
//     // Emit these frames to your widget.
//     emit processedFramesReady(processedFrame1, processedFrame2);

//     // Log the centroids received.
//     qDebug() << "Centroid for frame 1:" << centroid1.x << "," << centroid1.y;
//     qDebug() << "Centroid for frame 2:" << centroid2.x << "," << centroid2.y;

//     if (!P1.empty() && !P2.empty()) {

//         if (centroid1.x != -1 && centroid1.y != -1){
//             // Triangulate the 3D point from the centroids
//             cv::Mat pts1 = (cv::Mat_<double>(2, 1) << static_cast<double>(centroid1.x), static_cast<double>(centroid1.y));
//             cv::Mat pts2 = (cv::Mat_<double>(2, 1) << static_cast<double>(centroid2.x), static_cast<double>(centroid2.y));

//             cv::Mat points4D;
//             cv::triangulatePoints(P1, P2, pts1, pts2, points4D);

//             cv::Mat point3D = points4D.rowRange(0, 3) / points4D.at<double>(3, 0);
//             cv::Point3f point3f(point3D.at<double>(0), point3D.at<double>(1), point3D.at<double>(2));
//             updateCentroid(point3f, timestamp);
//             std::cout << "Triangulated 3D Point YOLO: " << point3f.x << " " << point3f.y << " " << point3f.z << std::endl;
//         } else {
//             qDebug() << "no ball centroid found";
//         }
//     } else {
//         qDebug() << "Projection matrices are empty, cannot triangulate.";
//     }


// }

void MainWindow::handleInferenceComplete(const cv::Mat &processedFrame1, const cv::Mat &processedFrame2,
                                         cv::Point centroid1, cv::Point centroid2, double timestamp)
{
    // Emit these frames to your widget.
    emit processedFramesReady(processedFrame1, processedFrame2);

    // Log the centroids received.
    qDebug() << "Centroid for frame 1:" << centroid1.x << "," << centroid1.y;
    qDebug() << "Centroid for frame 2:" << centroid2.x << "," << centroid2.y;

    if (!P1.empty() && !P2.empty()) {
        if (centroid1.x != -1 && centroid1.y != -1) {
            // Convert centroids from image coordinates (origin top-left, Y down)
            // to camera coordinates (origin bottom-left, Y up) for triangulation.
            int imageHeight1 = processedFrame1.rows; // assume both frames have the same height
            int imageHeight2 = processedFrame2.rows;
            cv::Point convertedCentroid1(centroid1.x, centroid1.y);
            cv::Point convertedCentroid2(centroid2.x, centroid2.y);

            // Prepare the points for triangulation using the converted centroids.
            cv::Mat pts1 = (cv::Mat_<double>(2, 1) << static_cast<double>(convertedCentroid1.x),
                            static_cast<double>(convertedCentroid1.y));
            cv::Mat pts2 = (cv::Mat_<double>(2, 1) << static_cast<double>(convertedCentroid2.x),
                            static_cast<double>(convertedCentroid2.y));

            cv::Mat points4D;
            cv::triangulatePoints(P1, P2, pts1, pts2, points4D);

            // Convert from homogeneous to Euclidean coordinates.
            cv::Mat point3D = points4D.rowRange(0, 3) / points4D.at<double>(3, 0);
            cv::Point3f point3f(point3D.at<double>(0),
                                point3D.at<double>(1),
                                point3D.at<double>(2));
            updateCentroid(point3f, timestamp);
            std::cout << "Triangulated 3D Point YOLO: "
                      << point3f.x << " " << point3f.y << " " << point3f.z << std::endl;
        } else {
            qDebug() << "No ball centroid found";
        }
    } else {
        qDebug() << "Projection matrices are empty, cannot triangulate.";
    }
}




void MainWindow::processYOLOFrame(cv::Mat &frame) {
    qDebug() << "Processing YOLO frame";

    // Create a blob from the image (normalize pixel values to [0,1])
    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1.0 / 255.0, yoloInputSize, cv::Scalar(0, 0, 0), true, false);
    qDebug() << "Blob created";

    {
        // Lock the mutex for thread-safe network usage.
        std::lock_guard<std::mutex> lock(yoloMutex);
        yoloNet.setInput(blob);
        qDebug() << "YOLO input set";

        // Forward pass to get detections.
        cv::Mat outputs = yoloNet.forward();
        qDebug() << "Forward pass complete";

        // Print output dimensions.
        QString outputInfo = "Outputs dims: " + QString::number(outputs.dims);
        for (int i = 0; i < outputs.dims; i++) {
            outputInfo += " " + QString::number(outputs.size[i]);
        }
        qDebug() << outputInfo;

        // The output is expected to be of shape (1, 5, 8400).
        // Reshape to a 2D matrix with one detection per row (8400 rows, 5 columns).
        cv::Mat detectionMat = outputs.reshape(1, outputs.size[1]); // now shape is (5, 8400)
        cv::transpose(detectionMat, detectionMat); // now shape is (8400, 5)
        qDebug() << "Detection matrix reshaped to:" << detectionMat.rows << "x" << detectionMat.cols;

        // Compute scaling factors from network input size (yoloInputSize) to original frame size.
        float x_factor = static_cast<float>(frame.cols) / yoloInputSize.width;
        float y_factor = static_cast<float>(frame.rows) / yoloInputSize.height;

        // Threshold for accepting detections.
        // modelScoreThreshold should be defined in your class (e.g., 0.5)
        std::vector<cv::Rect> boxes;
        std::vector<float> confidences;

        // Pointer to the detection data.
        float *data = reinterpret_cast<float*>(detectionMat.data);
        int numDetections = detectionMat.rows; // e.g., 8400

        for (int i = 0; i < numDetections; ++i) {
            // Each detection is in the form:
            // [center_x, center_y, width, height, confidence]
            float cx = data[0];
            float cy = data[1];
            float w  = data[2];
            float h  = data[3];
            float conf = data[4];

            if (conf > confThreshold) {
                confidences.push_back(conf);
                // Convert center-based coordinates to top-left corner.
                int left = static_cast<int>((cx - 0.5f * w) * x_factor);
                int top  = static_cast<int>((cy - 0.5f * h) * y_factor);
                int width = static_cast<int>(w * x_factor);
                int height = static_cast<int>(h * y_factor);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
            data += 5; // move to the next detection (5 elements per detection)
        }

        // Draw detection boxes and labels on the frame.
        for (size_t i = 0; i < boxes.size(); i++) {
            cv::rectangle(frame, boxes[i], cv::Scalar(0, 255, 0), 2);
            std::string label = cv::format("t: %.2f", confidences[i]);
            cv::putText(frame, label, boxes[i].tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }
        qDebug() << "Drew detection boxes";
    } // Mutex unlocks here
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
        backSub.dynamicCast<cv::BackgroundSubtractorMOG2>()->clear();
        processingType = "BD";
    } else if (ui.showProcessingYOLOButton->isChecked()){
        processingType="YOLO";
    }
    else {
        processingType = "None"; // Default to "None" if neither checkbox is checked
    }
    // excluded processingType == "BGSub" || from below if statement bc I want it to
    // be 3 channel so that we can see a red centroid on it
    if (processingType == "Thresh") {
        ui.setBackgroundImageButton->setEnabled(true);
        format = QImage::Format_Grayscale8;
    } else {
        // will also be color for YOLO
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


    // cv::Point left_centroid = centroids[0];
    // for (const auto& centroid : centroids) {
    //     if (centroid.x < left_centroid.x) {
    //         left_centroid = centroid;
    //     }
    // }

    // // Step 2: Separate the remaining two centroids
    // std::vector<cv::Point> remaining_centroids;
    // for (const auto& centroid : centroids) {
    //     if (centroid != left_centroid) {
    //         remaining_centroids.push_back(centroid);
    //         if (remaining_centroids.size() > 2) {
    //             qDebug() << "Reordering error";
    //         }
    //     }
    // }

    // cv::Point top_centroid = remaining_centroids[0];
    // cv::Point right_centroid = remaining_centroids[1];
    // if (remaining_centroids[0].y > remaining_centroids[1].y) {
    //     std::swap(top_centroid, right_centroid);
    // }

    cv::Point right_centroid = centroids[0];
    for (const auto& centroid : centroids) {
        if (centroid.x > right_centroid.x) {
            right_centroid = centroid;
        }
    }

    // Step 2: Separate the remaining two centroids
    std::vector<cv::Point> remaining_centroids;
    for (const auto& centroid : centroids) {
        if (centroid != right_centroid) {
            remaining_centroids.push_back(centroid);
            if (remaining_centroids.size() > 2) {
                qDebug() << "Reordering error";
            }
        }
    }

    cv::Point top_centroid = remaining_centroids[0];
    cv::Point left_centroid = remaining_centroids[1];
    if (remaining_centroids[0].y > remaining_centroids[1].y) {
        std::swap(top_centroid, left_centroid);
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

void MainWindow::checkLEDDistances() {
    std::vector<cv::Mat> LEDCoords = MainWindow::getLEDCoords();
    if (LEDCoords.empty()) {
        return;
    }
    cv::Mat topCoords = LEDCoords[0].reshape(1,3); // left top
    // std::cout << "Top: " << topCoords << std::endl;
    cv::Mat leftCoords = LEDCoords[1].reshape(1,3); // left bot
    // std::cout << "Left: " << leftCoords << std::endl;
    cv::Mat rightCoords = LEDCoords[2].reshape(1,3);
    // std::cout << "Right: " << rightCoords << std::endl;

    double horizontalDistance = cv::norm(rightCoords - topCoords);
    double verticalDistance = cv::norm(topCoords - leftCoords);
    double actualHorizontalDistance = 690; //749
    double actualVerticalDistance = 405; //440

    double horizontalError = std::abs(horizontalDistance - actualHorizontalDistance);
    double verticalError = std::abs(verticalDistance - actualVerticalDistance);

    qDebug() << "Measured horizontal distance: " << horizontalDistance << " mm, absolute error: " << horizontalError << " mm, relative error: " << horizontalError / actualHorizontalDistance * 100 << " %";
    qDebug() << "Measured vertical distance: " << verticalDistance << " mm, absolute error: " << verticalError << " mm, relative error: " << verticalError / actualVerticalDistance * 100 << " %";

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
    // DEBUG: print angle between x and y
    cv::Mat zDir = xDir.cross(yDir);


    sphericalOrigin = rightCoords + displacementRightLEDToOrigin; // DEBUG: print this


    rotationMatrix = (cv::Mat_<double>(3,3) <<
                          xDir.at<double>(0,0), xDir.at<double>(1,0), xDir.at<double>(2,0),
                      yDir.at<double>(0,0), yDir.at<double>(1,0), yDir.at<double>(2,0),
                      zDir.at<double>(0,0), zDir.at<double>(1,0), zDir.at<double>(2,0)
                      );

    qDebug() << "Successfully completed spherical calibration";
}

void MainWindow::sphericalTest() { // can try to test by pointing at one of the markers
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

void MainWindow::sendSerialMessage(QString baseMessage, bool getFeedback) {
    QString serialMessage = baseMessage + "\n";
    qint64 bytesWritten = serialPort->write(serialMessage.toUtf8());
    if (bytesWritten == -1) {
        qDebug() << "Error: Failed to write data to serial port.";
    } else {
        qDebug() << "Message sent successfully:" << serialMessage.trimmed();
    }

    if (getFeedback) {
        // Wait for up to 5000ms (5 seconds) for feedback
        if (serialPort->waitForReadyRead(5000)) {
            QByteArray feedbackData = serialPort->readAll();
            // If more data is expected, you could loop until no more data is available:
            while (serialPort->waitForReadyRead(100)) {
                feedbackData.append(serialPort->readAll());
            }
            QString feedback = QString::fromUtf8(feedbackData);
            qDebug() << "Received feedback:" << feedback.trimmed();
        } else {
            qDebug() << "No feedback received within timeout.";
        }
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

void MainWindow::initializeAltitude(int portNum) {
    altitudePointer = initializeAltitudeMotor(portNum);
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


void MainWindow::toggleNearFarSweep() {

    // we want to set the limiting angles for the near
    // sweep first.
    if (ui.nearSweepRadioButton->isChecked()) {
        // if near away the sweep settings are enabled
        ui.farSweepRadioButton->setChecked(false);
        ui.azimuthLimitSpinbox->setEnabled(true);
        ui.goToAzLimitButton->setEnabled(true);
        ui.setLowerAzLimit->setEnabled(true);
        ui.setUpperAzLimit->setEnabled(true);
        ui.numPointsAzimuthSpinbox->setEnabled(true);

        ui.altitudeLimitSpinbox->setEnabled(true);
        ui.goToAlLimitButton->setEnabled(true);
        ui.setLowerAlLimit->setEnabled(true);
        ui.setUpperAlLimit->setEnabled(true);
        ui.numPointsAltitudeSpinbox->setEnabled(true);
        ui.setSweepAlRPMLimitSpinBox->setEnabled(true);
    } else if (ui.farSweepRadioButton->isChecked()) {
        // if far sweep selected disable so you can't change
        ui.nearSweepRadioButton->setChecked(false);
        ui.azimuthLimitSpinbox->setEnabled(false);
        ui.goToAzLimitButton->setEnabled(false);
        ui.setLowerAzLimit->setEnabled(false);
        ui.setUpperAzLimit->setEnabled(false);
        ui.numPointsAzimuthSpinbox->setEnabled(false);

        ui.altitudeLimitSpinbox->setEnabled(false);
        ui.goToAlLimitButton->setEnabled(false);
        ui.setLowerAlLimit->setEnabled(false);
        ui.setUpperAlLimit->setEnabled(false);
        ui.numPointsAltitudeSpinbox->setEnabled(false);
        ui.setSweepAlRPMLimitSpinBox->setEnabled(false);
    }
}

void MainWindow::moveAlLimit()
{
    double altitudeLimit = ui.altitudeLimitSpinbox->value();
    sweepAlRPMLimit = ui.setSweepAlRPMLimitSpinBox->value();
    qDebug() << "moving altitude to " <<altitudeLimit << "degrees";
    moveAltitudeMotor(altitudePointer, altitudeLimit, sweepAlRPMLimit);
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

void MainWindow::performSweepStep() {
    // If we've finished all azimuth positions, the sweep is complete.
    if (currentAzIndex >= azimuthCalPositions.size()) {
        // Sweep complete: stop capture, reset UI, and save the JSON file.
        stopCapture();
        processingType = "None";
        format = QImage::Format_BGR888;
        qDebug() << "Sweep complete. Camera capture stopped.";

        // Create folder based on sweepName
        QString folderPath = QString("../../MotorCameraCalibration/") + sweepName;
        QDir dir;
        if (!dir.exists(folderPath)) {
            if (!dir.mkpath(folderPath)) {
                qDebug() << "Failed to create directory:" << folderPath;
                // Optionally handle the error (e.g., return or set an error flag)
            }
        }

        // Determine file path based on radio button selection.
        std::string filePath = "";
        if (ui.farSweepRadioButton->isChecked()){
            filePath = folderPath.toStdString() + "/far_sweep.json";
            ui.farSweepDoneCheckBox->setChecked(true);
        }
        else if (ui.nearSweepRadioButton->isChecked()){
            filePath = folderPath.toStdString() + "/near_sweep.json";
            ui.nearSweepDoneCheckBox->setChecked(true);
        }

        std::ofstream outFile(filePath);
        if (outFile.is_open()) {
            outFile << std::setw(4) << sweepData << std::endl;
            outFile.close();
            qDebug() << "Saved sweep data to" << QString::fromStdString(filePath);
        } else {
            qDebug() << "Error opening file for sweep data:" << QString::fromStdString(filePath);
        }
        sweepTimer->stop();

        if(ui.nearSweepDoneCheckBox->isChecked() && ui.farSweepDoneCheckBox->isChecked()){
            // if both sweeps have been performed, unlock the next buttons.
            ui.calculateLookupTableButton->setEnabled(true);
            ui.numAzInterpPointsSpinBox->setEnabled(true);
            ui.numAlInterpPointsSpinBox->setEnabled(true);
        }

        ui.startMotorCameraCalibrationButton->setEnabled(true);
        ui.stopMotorCameraCalibrationButton->setEnabled(false);

        sweepData.clear();

        return;
    }


    // For a new azimuth group, move the azimuth motor once.
    if (currentAlIndex == 0) {
        double az = azimuthCalPositions[currentAzIndex];
        qDebug() << "Moving azimuth to" << az << "degrees";
        int steps = static_cast<int>(az / 0.45) - azimuthPosition;
        sendSerialMessage(QString::number(steps));
        azimuthPosition += steps;
    }

    // If there are altitude positions left for the current azimuth:
    if (currentAlIndex < altitudeCalPositions.size()) {
        double al = altitudeCalPositions[currentAlIndex];
        qDebug() << "Moving altitude to" << al << "degrees";
        moveAltitudeMotor(altitudePointer, al, sweepAlRPMLimit);

        // Capture the current azimuth and altitude in local variables.
        double localAz = azimuthCalPositions[currentAzIndex];
        double localAl = al;
        currentAlIndex++;

        // Stop the main timer so it doesn't fire before our wait is over.
        sweepTimer->stop();
        // Wait 5 seconds for the motor to settle, then process the laser dot.
        QTimer::singleShot(1000, this, [this, localAz, localAl]() {
            qDebug() << "Motor settled. Processing image.";
            auto centroid = motorCameraCalibrationCurrentCentroid;
            qDebug() << "LASER DOT:" << centroid.x << centroid.y << centroid.z;

            // Append a row [az, al, x, y, z] using the local (captured) values.
            nlohmann::json row = { localAz, localAl, centroid.x, centroid.y, centroid.z };
            sweepData["sweep"].push_back(row);

            // Restart the main timer to continue the sweep.
            sweepTimer->start(2000);
        });
    } else {
        // Finished all altitude moves for the current azimuth; reset for the next azimuth.
        currentAlIndex = 0;
        currentAzIndex++;
    }
}




void MainWindow::sweepLookupTable() {

    sweepName = ui.newMoCamCalLineEdit->text().trimmed();

    if (sweepName.isEmpty()) {
        QMessageBox::warning(this, "Invalid Name", "Please enter a name before starting lookup table sweep.");
        return;
    }


    qDebug() << "Starting Motor-Camera Sweep";

    // disable start button
    ui.startMotorCameraCalibrationButton->setEnabled(false);
    // enable stop button
    ui.stopMotorCameraCalibrationButton->setEnabled(true);

    processingType = "Thresh";
    format = QImage::Format_Grayscale8;

    startCapture();

    // Set up your positions and reset indices
    int numAzPoints = ui.numPointsAzimuthSpinbox->value();
    int numAlPoints = ui.numPointsAltitudeSpinbox->value();

    azimuthCalPositions = linspace(azimuthCalLowerLimit, azimuthCalUpperLimit, numAzPoints);
    altitudeCalPositions = linspace(altitudeCalLowerLimit, altitudeCalUpperLimit, numAlPoints);
    currentAzIndex = 0;
    currentAlIndex = 0;
    sweepAlRPMLimit = ui.setSweepAlRPMLimitSpinBox->value();

    // Create and start a QTimer with an interval that suits your needs (e.g., 2000ms)
    sweepTimer = new QTimer(this);
    connect(sweepTimer, &QTimer::timeout, this, &MainWindow::performSweepStep);
    sweepTimer->start(2000); // interval in milliseconds

    qDebug() << "Motor-Camera Sweep started";
}

void MainWindow::stopSweep() {
    // Stop the timer if it exists and is active.
    if (sweepTimer && sweepTimer->isActive()) {
        sweepTimer->stop();
    }

    // Stop capture and reset processing type.
    stopCapture();
    processingType = "None";

    // Optionally reset indices for a future sweep.
    currentAzIndex = 0;
    currentAlIndex = 0;

    qDebug() << "Motor-Camera Sweep stopped by user.";

    resetMotorCameraCalibration();
}



#include <fstream>
#include <cmath>
#include <iomanip>
#include "json.hpp"  // nlohmann::json header

// Make sure these are declared in your MainWindow class if not already:
// std::vector<double> azimuthCalPositions;
// std::vector<double> altitudeCalPositions;
// int azimuthPosition;  // current azimuth motor position

void MainWindow::calculateLookupTable() {
    // Open the far and near JSON files



    QString folderPath = "../../MotorCameraCalibration/" + sweepName;
    std::ifstream farFile((folderPath + "/far_sweep.json").toStdString());
    std::ifstream nearFile((folderPath + "/near_sweep.json").toStdString());


    if (!farFile.is_open() || !nearFile.is_open()) {
        qDebug() << "Error: Could not open one of the JSON files.";
        return;
    }

    // Parse JSON data from the files
    nlohmann::json farJson, nearJson;
    try {
        farFile >> farJson;
        nearFile >> nearJson;
    } catch (const std::exception &e) {
        qDebug() << "Error parsing JSON:" << e.what();
        return;
    }

    // Extract the "sweep" arrays from both JSON objects
    auto farSweep = farJson["sweep"];
    auto nearSweep = nearJson["sweep"];

    // Verify that both files contain the same number of sweep points
    if (farSweep.size() != nearSweep.size()) {
        qDebug() << "Error: Mismatch in the number of sweep points between far and near files.";
        return;
    }

    // Prepare the lookup table JSON object
    nlohmann::json lookupTableJson;
    lookupTableJson["lookuptable"] = nlohmann::json::array();

    // For each row, calculate the unit vector from near point to far point.
    for (size_t i = 0; i < farSweep.size(); ++i) {
        // Each row is expected to be of the form: [az, al, x, y, z]
        double az = farSweep[i][0];
        double al = farSweep[i][1];

        // Far sweep coordinates
        double farX = farSweep[i][2];
        double farY = farSweep[i][3];
        double farZ = farSweep[i][4];

        // Near sweep coordinates
        double nearX = nearSweep[i][2];
        double nearY = nearSweep[i][3];
        double nearZ = nearSweep[i][4];

        // Compute the difference vector: far - near
        double diffX = farX - nearX;
        double diffY = farY - nearY;
        double diffZ = farZ - nearZ;

        // Calculate the Euclidean norm (magnitude) of the difference vector
        double norm = std::sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);
        double ux = 0.0, uy = 0.0, uz = 0.0;
        if (norm > 1e-8) {  // Avoid division by zero
            ux = diffX / norm;
            uy = diffY / norm;
            uz = diffZ / norm;
        }

        // Build a row with the following format:
        // [az, al, nearX, nearY, nearZ, ux, uy, uz]
        nlohmann::json row = { az, al, nearX, nearY, nearZ, ux, uy, uz };
        lookupTableJson["lookuptable"].push_back(row);
    }

    // Write the lookup table to "lookuptable.json"
    QString lookupTablePath = folderPath + "/lookuptable.json";
    std::ofstream outFile(lookupTablePath.toStdString());
    if (!outFile.is_open()) {
        qDebug() << "Error: Could not open lookuptable.json for writing.";
        return;
    }
    outFile << std::setw(4) << lookupTableJson;
    outFile.close();

    qDebug() << "Lookup table successfully calculated and saved to lookuptable.json";
}
#include <limits>
#include <fstream>
#include <iostream>
#include <vector>
#include <set>
#include <array>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include "json.hpp"  // nlohmann::json

// Helper function: Bilinear interpolation
double bilinearInterpolate(double x, double y,
                           double x1, double x2, double y1, double y2,
                           double Q11, double Q21, double Q12, double Q22)
{
    double denom = (x2 - x1) * (y2 - y1);
    if (std::abs(denom) < 1e-8) {
        return Q11; // Fallback if cell is degenerate.
    }
    double term1 = Q11 * (x2 - x) * (y2 - y);
    double term2 = Q21 * (x - x1) * (y2 - y);
    double term3 = Q12 * (x2 - x) * (y - y1);
    double term4 = Q22 * (x - x1) * (y - y1);
    return (term1 + term2 + term3 + term4) / denom;
}

void MainWindow::calculateInterpolatedLookupTable()
{

    int newAzCount = ui.numAzInterpPointsSpinBox->value();
    int newAlCount = ui.numAlInterpPointsSpinBox->value();

    QString folderPath = "../../MotorCameraCalibration/" + sweepName;

    // 1. Read the raw lookup table JSON file.
    std::ifstream inFile((folderPath + "/lookuptable.json").toStdString());
    if (!inFile.is_open()) {
        qDebug() << "Error: Could not open lookuptable.json";
        return;
    }
    nlohmann::json rawJson;
    try {
        inFile >> rawJson;
    } catch (const std::exception &e) {
        qDebug() << "Error parsing JSON:" << e.what();
        return;
    }
    inFile.close();

    // Expect the file to contain a key "lookuptable" with an array of rows.
    auto rawTable = rawJson["lookuptable"];
    size_t totalPoints = rawTable.size();
    if (totalPoints == 0) {
        qDebug() << "Error: Lookup table is empty.";
        return;
    }

    // 2. Extract the raw data and determine unique azimuth and altitude values.
    // Each row is assumed to be: [az, al, xo, yo, zo, ux, uy, uz]
    std::vector<std::vector<double>> rawData; // each inner vector will have 8 values.
    std::set<double> azSet, alSet;
    for (auto& row : rawTable) {
        std::vector<double> v;
        for (int i = 0; i < 8; i++) {
            v.push_back(row[i]);
        }
        rawData.push_back(v);
        azSet.insert(v[0]); // azimuth value
        alSet.insert(v[1]); // altitude value
    }
    // Create sorted vectors.
    std::vector<double> uniqueAz(azSet.begin(), azSet.end());
    std::vector<double> uniqueAl(alSet.begin(), alSet.end());
    size_t origAzCount = uniqueAz.size();
    size_t origAlCount = uniqueAl.size();

    // Optional: warn if the number of raw points doesn't match a regular grid.
    if (totalPoints != origAzCount * origAlCount) {
        qDebug() << "Warning: The raw lookup table does not form a regular grid.";
    }

    // 3. Arrange the 6 data columns (near point and unit vector) into a 2D grid.
    // We'll index grid[i][j] where i corresponds to uniqueAz and j to uniqueAl.
    // Each grid element is an array of 6 doubles: {xo, yo, zo, ux, uy, uz}.
    std::vector<std::vector<std::array<double, 6>>> grid(
        origAzCount, std::vector<std::array<double, 6>>(origAlCount));
    // For each raw row, find its position in the grid.
    for (const auto& row : rawData) {
        double az = row[0];
        double al = row[1];
        auto itAz = std::find(uniqueAz.begin(), uniqueAz.end(), az);
        auto itAl = std::find(uniqueAl.begin(), uniqueAl.end(), al);
        if (itAz == uniqueAz.end() || itAl == uniqueAl.end())
            continue;
        size_t i = std::distance(uniqueAz.begin(), itAz);
        size_t j = std::distance(uniqueAl.begin(), itAl);
        grid[i][j] = { row[2], row[3], row[4], row[5], row[6], row[7] };
    }

    // 4. Determine the min/max for azimuth and altitude.
    double azMin = uniqueAz.front();
    double azMax = uniqueAz.back();
    double alMin = uniqueAl.front();
    double alMax = uniqueAl.back();

    // 5. Create new uniformly spaced arrays for azimuth and altitude.
    std::vector<double> newAz(newAzCount);
    std::vector<double> newAl(newAlCount);
    for (int i = 0; i < newAzCount; i++) {
        newAz[i] = azMin + i * (azMax - azMin) / (newAzCount - 1);
    }
    for (int j = 0; j < newAlCount; j++) {
        newAl[j] = alMin + j * (alMax - alMin) / (newAlCount - 1);
    }

    // 6. Interpolate the 6 values for each new grid point using bilinear interpolation.
    // The new lookup table will have rows of the form:
    // [az, al, xo, yo, zo, ux, uy, uz]
    nlohmann::json interpJson;
    interpJson["lookuptable"] = nlohmann::json::array();

    // Also, create a local vector to hold the new lookup table entries.
    std::vector<LookupEntry> newLookupTable;
    newLookupTable.reserve(newAzCount * newAlCount);

    // For each new azimuth and altitude point:
    for (int i_new = 0; i_new < newAzCount; i_new++) {
        for (int j_new = 0; j_new < newAlCount; j_new++) {
            double az_query = newAz[i_new];
            double al_query = newAl[j_new];

            // Find the bounding indices in the original grid.
            size_t i_low = 0;
            while (i_low < uniqueAz.size() - 1 && uniqueAz[i_low + 1] <= az_query)
                i_low++;
            size_t i_high = (i_low < uniqueAz.size() - 1) ? i_low + 1 : i_low;

            size_t j_low = 0;
            while (j_low < uniqueAl.size() - 1 && uniqueAl[j_low + 1] <= al_query)
                j_low++;
            size_t j_high = (j_low < uniqueAl.size() - 1) ? j_low + 1 : j_low;

            // If the query lies exactly on a grid point, use that data directly.
            if (std::abs(uniqueAz[i_low] - az_query) < 1e-8 &&
                std::abs(uniqueAl[j_low] - al_query) < 1e-8) {
                std::array<double, 6> interpValues = grid[i_low][j_low];
                nlohmann::json row = { az_query, al_query,
                                      interpValues[0], interpValues[1], interpValues[2],
                                      interpValues[3], interpValues[4], interpValues[5] };
                interpJson["lookuptable"].push_back(row);

                // Save the lookup entry.
                LookupEntry entry;
                entry.az = az_query;
                entry.al = al_query;
                entry.xo = interpValues[0];
                entry.yo = interpValues[1];
                entry.zo = interpValues[2];
                entry.ux = interpValues[3];
                entry.uy = interpValues[4];
                entry.uz = interpValues[5];
                newLookupTable.push_back(entry);
                continue;
            }

            // Otherwise, perform bilinear interpolation for each component.
            double x1 = uniqueAz[i_low], x2 = uniqueAz[i_high];
            double y1 = uniqueAl[j_low], y2 = uniqueAl[j_high];

            std::array<double, 6> interpVals;
            for (int k = 0; k < 6; k++) {
                double Q11 = grid[i_low][j_low][k];
                double Q21 = grid[i_high][j_low][k];
                double Q12 = grid[i_low][j_high][k];
                double Q22 = grid[i_high][j_high][k];
                interpVals[k] = bilinearInterpolate(az_query, al_query,
                                                    x1, x2, y1, y2,
                                                    Q11, Q21, Q12, Q22);
            }

            nlohmann::json newRow = { az_query, al_query,
                                     interpVals[0], interpVals[1], interpVals[2],
                                     interpVals[3], interpVals[4], interpVals[5] };
            interpJson["lookuptable"].push_back(newRow);

            // Save the lookup entry.
            LookupEntry entry;
            entry.az = az_query;
            entry.al = al_query;
            entry.xo = interpVals[0];
            entry.yo = interpVals[1];
            entry.zo = interpVals[2];
            entry.ux = interpVals[3];
            entry.uy = interpVals[4];
            entry.uz = interpVals[5];
            newLookupTable.push_back(entry);
        }
    }

    // 7. Save the interpolated lookup table to a new JSON file.
    QString lookupTablePath = folderPath + "/interpolated_lookuptable.json";
    std::ofstream outFile(lookupTablePath.toStdString());
    if (!outFile.is_open()) {
        qDebug() << "Error: Could not open file for writing interpolated lookup table.";
        return;
    }
    outFile << std::setw(4) << interpJson;
    outFile.close();

    // 8. Save the new lookup table to the class variable.
    this->interpolatedLookupTable = newLookupTable;

    qDebug() << "Interpolated lookup table successfully saved to interpolated_lookuptable.json";
}


void MainWindow::getLookupTable(){
    calculateLookupTable();
    calculateInterpolatedLookupTable();

    ui.setAimAlRPMLimitSpinBox->setEnabled(true);
    ui.aimButton->setEnabled(true);
}

#include <limits>
#include <utility>  // for std::pair
#include <vector>
#include <algorithm> // for std::min and std::max

// Assume LookupEntry is defined as follows:
// struct LookupEntry {
//     double az;  // azimuth angle
//     double al;  // altitude angle
//     double xo, yo, zo; // near point (origin of the line)
//     double ux, uy, uz; // unit direction vector (points from near to far)
// };
// And that MainWindow has a member variable:
// std::vector<LookupEntry> interpolatedLookupTable;

std::pair<double, double> MainWindow::findFiringAngle(double x, double y, double z) {
    double minDistSq = std::numeric_limits<double>::max();
    double bestAz = 0.0;
    double bestAl = 0.0;

    // Iterate over all entries in the interpolated lookup table.
    for (const auto& entry : interpolatedLookupTable) {
        // Compute the vector from the near point (A) to the given point (P)
        double vx = x - entry.xo;
        double vy = y - entry.yo;
        double vz = z - entry.zo;

        // Dot product of (P-A) with the unit direction vector d
        double dot = vx * entry.ux + vy * entry.uy + vz * entry.uz;

        // Squared norm of (P-A)
        double vNormSq = vx * vx + vy * vy + vz * vz;

        // Squared perpendicular (normal) distance from the point to the line.
        double distSq = vNormSq - dot * dot;

        // qDebug() << "in lookup table loop";
        // qDebug() << "distSq: " << distSq;

        // Update best match if this distance is smaller than the current minimum.
        if (distSq < minDistSq) {

            minDistSq = distSq;

            // qDebug() << "good match found: " << minDistSq;
            // qDebug() << "vector: " << vx << vy << vz;
            // qDebug() << "dot: " << dot;
            // qDebug() << "vNormSq: " << vNormSq;
            bestAz = entry.az;
            bestAl = entry.al;
            // qDebug() << "Best angles: " <<bestAz << bestAl;
        }
    }

    bestAz = std::max(azConfigData.minAzAngle, std::min(bestAz, azConfigData.maxAzAngle));
    bestAl = std::max(alConfigData.minAlAngle, std::min(bestAl, alConfigData.maxAlAngle));

    return { bestAz, bestAl };
}


void MainWindow::aimAtCentroid(){

    qDebug() << "In aim at centroid";
    // Get the current centroid.
    cv::Point3f point = getCentroid();

    qDebug() << "got threadsafe centroid successfully";

    double x = point.x;
    double y = point.y;
    double z = point.z;

    qDebug() << "threadsafe centroid is " << x << y<< z;

    // Compute desired firing angles.
    std::pair<double, double> bestAngles = findFiringAngle(x, y, z);
    qDebug() << "Best az:" << bestAngles.first << "best al:" << bestAngles.second;

    // Calculate required steps for the azimuth motor.
    int azSteps = static_cast<int>(bestAngles.first / 0.45) - azimuthPosition;
    qDebug() << "Moving azimuth motor by" << azSteps << "steps.";
    sendSerialMessage(QString::number(azSteps));
    azimuthPosition += azSteps;  // Update current azimuth position.
    double aimRPMLimit = ui.setAimAlRPMLimitSpinBox->value();

    // Use a short delay to allow the azimuth move to start/completed before moving altitude.
    QTimer::singleShot(200, this, [this, bestAngles, aimRPMLimit]() {
        qDebug() << "Moving altitude motor to" << bestAngles.second << "degrees.";
        moveAltitudeMotor(altitudePointer, bestAngles.second, 1.0);
    });
}

void MainWindow::aimAtInterceptionPoint(cv::Point3f& point) {
    double x = point.x;
    double y = point.y;
    double z = point.z;

    qDebug() << "Intercepting at: " << x << y<< z;

    // Compute desired firing angles.
    std::pair<double, double> bestAngles = findFiringAngle(x, y, z);
    qDebug() << "Best az:" << bestAngles.first << "best al:" << bestAngles.second;

    // Calculate required steps for the azimuth motor.
    int azSteps = static_cast<int>(bestAngles.first / 0.45) - azimuthPosition;
    qDebug() << "Moving azimuth motor by" << azSteps << "steps.";
    sendSerialMessage(QString::number(azSteps));
    azimuthPosition += azSteps;  // Update current azimuth position.
    double aimRPMLimit = ui.setAimAlRPMLimitSpinBox->value();

    // Use a short delay to allow the azimuth move to start/completed before moving altitude.
    QTimer::singleShot(10, this, [this, bestAngles, aimRPMLimit]() {
        qDebug() << "Moving altitude motor to" << bestAngles.second << "degrees.";
        moveAltitudeMotor(altitudePointer, bestAngles.second, aimRPMLimit);
    });
}

// In MainWindow.cpp, add:
void MainWindow::aimContinuous(){
    // Create the timer if it hasn't been created yet.
    if (!continuousAimTimer) {
        continuousAimTimer = new QTimer(this);
        // Connect the timer's timeout signal to aimAtCentroid slot.
        connect(continuousAimTimer, &QTimer::timeout, this, &MainWindow::aimAtCentroid);
    }
    // Start the timer with a 1000 ms (1 second) interval.
    continuousAimTimer->start(100);
    qDebug() << "Continuous aiming started.";
}

void MainWindow::stopContinuousAim(){
    // Stop the timer if it's running.
    if (continuousAimTimer && continuousAimTimer->isActive()){
        continuousAimTimer->stop();
        qDebug() << "Continuous aiming stopped.";
    }
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
    //initializeAltitudeMotor();
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
        d_kernel.upload(kernel);  // Store the kernel on the GPU

        // Create filters once with the new kernel
        erodeFilter1 = cv::cuda::createMorphologyFilter(cv::MORPH_ERODE, CV_8UC1, kernel_cpu);
        dilateFilter1 = cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, CV_8UC1, kernel_cpu);
        erodeFilter2 = cv::cuda::createMorphologyFilter(cv::MORPH_ERODE, CV_8UC1, kernel_cpu);
        dilateFilter2 = cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, CV_8UC1, kernel_cpu);

        prevKernelSize = kernelSize;
    }

}
cv::Point3f MainWindow::processImageCentroid(const cv::Mat &originalFrame1, const cv::Mat &originalFrame2, bool timingEnabled) {
    setBDValues();

    if (originalFrame1.empty() || originalFrame2.empty()) {
        qDebug() << "One or both input frames are empty!";
        return cv::Point3f(-1, -1, -1);
    }

    // Future to store result
    QFuture<cv::Point3f> future;

    if (timingEnabled) {
        // Run with totalTimer and store result in future
        future = QtConcurrent::run([this, originalFrame1, originalFrame2]() {
            cv::Point3f point;
            totalTimer->timeVoid([&]() {
                point = processImages(originalFrame1, originalFrame2, true);
            });
            return point;
        });
    } else {
        // Run without timing and store result in future
        future = QtConcurrent::run([this, originalFrame1, originalFrame2]() {
            return processImages(originalFrame1, originalFrame2, false);
        });
    }

    // Wait for processImages to finish and get result
    future.waitForFinished();
    cv::Point3f result = future.result();

    //qDebug() << "Returning from processImageCentroid:" << result.x << result.y << result.z;
    return result;
}

cv::Point3f MainWindow::processImageCentroidCUDA(const cv::Mat &originalFrame1, const cv::Mat &originalFrame2, bool timingEnabled) {
    setBDValues();

    if (originalFrame1.empty() || originalFrame2.empty()) {
        qDebug() << "One or both input frames are empty!";
        return cv::Point3f(-1, -1, -1);
    }

    // Future to store result
    QFuture<cv::Point3f> future;

    if (timingEnabled) {
        cv::Point3f point;
        totalTimer->timeVoid([&]() {
            point = processImagesCUDA(originalFrame1, originalFrame2, true);
        });
        return point;
    } else {
        return processImagesCUDA(originalFrame1, originalFrame2, false);
    }
}

cv::Point3f MainWindow::processImages(const cv::Mat &originalFrame1, const cv::Mat &originalFrame2, bool timingEnabled) {
    cv::Point3f point = cv::Point3f(-1, -1, -1);  // Default to invalid point

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
    cv::Point2f centroid1 = cv::Point2f(-1, -1), centroid2 = cv::Point2f(-1, -1);
    if (centroidEnabled) {
        // centroid1 = ComputeCentroid(output1);
        // centroid2 = ComputeCentroid(output2);

        // std::vector<cv::Point> centroids1, centroids2;
        centroid1 = FindLargestCentroid(output1);
        centroid2 = FindLargestCentroid(output2);

    }

    // Special handling if either centroid is invalid (-1, -1)
    if (centroid1 == cv::Point2f(-1, -1) || centroid2 == cv::Point2f(-1, -1)) {
        // if (centroid1 == cv::Point2f(-1, -1) && centroid2 == cv::Point2f(-1, -1)) {
        //     qDebug() << "No centroids detected in either image";
        // } else {
        //     qDebug() << "Only one valid centroid detected";
        // }
    }
    else {
         // If centroids are enabled and valid, draw the centroids
        if (centroidEnabled && drawEnabled) {
            if (centroid1 != cv::Point2f(-1, -1)) {
                DrawCentroidBinary(output1, centroid1);
            }
            if (centroid2 != cv::Point2f(-1, -1)) {
                DrawCentroidBinary(output2, centroid2);
            }
        }

        // Handle centroids if both are valid
        if (centroid1 != cv::Point2f(-1, -1) && centroid2 != cv::Point2f(-1, -1)) {
            // Both centroids are valid
            point = handleCentroids(centroid1, centroid2);
        }
    }

    // Store the processed frames for further use
    processedFrame1 = output1.clone();
    processedFrame2 = output2.clone();
    emit processedFramesReady(processedFrame1, processedFrame2);

    return point;
}

cv::Point3f MainWindow::processImagesCUDA(const cv::Mat &originalFrame1, const cv::Mat &originalFrame2, bool timingEnabled) {
    format = QImage::Format_Grayscale8;
    cv::Point3f point = cv::Point3f(-1, -1, -1);  // Default to invalid point

    // Process image 1
    QFuture<void> futureOutput1 = QtConcurrent::run([&]() {
        processSingleImageCUDA(originalFrame1, output1, true, timingEnabled);
        if (centroidEnabled) {
            if (timingEnabled) {
                centroidTimer->timeVoid([&]() {
                    // centroid1 = FindCentroidCUDA(output1);
                    centroid1 = ComputeCentroid(output1);
                });
            } else {
                // centroid1 = FindCentroidCUDA(output1);
                centroid1 = ComputeCentroid(output1);
            }
        }
    });

    // Process image 2
    QFuture<void> futureOutput2 = QtConcurrent::run([&]() {
        processSingleImageCUDA(originalFrame2, output2, false, false);
        // if (centroidEnabled) centroid2 = FindCentroidCUDA(output2);
        if (centroidEnabled) centroid2 = ComputeCentroid(output2);
    });

    futureOutput1.waitForFinished();
    futureOutput2.waitForFinished();

    bool validCentroids = (centroid1 != cv::Point(-1, -1) && centroid2 != cv::Point(-1, -1));
    if (!validCentroids) {
        // if (centroid1 == cv::Point2f(-1, -1) && centroid2 == cv::Point2f(-1, -1)) {
        //     qDebug() << "No centroids detected in either image";
        // } else {
        //     qDebug() << "Only one valid centroid detected";
        // }
    } else {
        // qDebug() << "Centroids detected";
        // Handle centroids
        point = handleCentroids(centroid1, centroid2);
    }

    if (drawEnabled && validCentroids) {
        cv::cvtColor(output1, output1, cv::COLOR_GRAY2BGR);
        cv::cvtColor(output2, output2, cv::COLOR_GRAY2BGR);
        cv::circle(output1, centroid1, 5, cv::Scalar(0, 0, 255), -1);
        cv::circle(output2, centroid2, 5, cv::Scalar(0, 0, 255), -1);
        processedFrame1 = output1.clone();
        processedFrame2 = output2.clone();
        format = QImage::Format_BGR888;
    } else {
        processedFrame1 = output1.clone();
        processedFrame2 = output2.clone();
        format = QImage::Format_Grayscale8;
    }

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

        if (hsvEnabled || bgrEnabled && motionEnabled) {
            ApplyThresholdConsecutively(originalFrame, tmpGray1, output, backgroundImage1, thresholdValue);
        } else if (motionEnabled) {
            ApplyMotionThreshold(originalFrame, tmpGray1, output, backgroundImage1, thresholdValue);
        }

        if ((hsvEnabled || motionEnabled) && morphEnabled) {
            ApplyMorphClosing(output, kernel);
        }
    }
}

void MainWindow::processSingleImageCUDA(const cv::Mat &originalFrame, cv::Mat& output, bool first, bool timingEnabled) {
    cv::cuda::GpuMat d_originalFrame = UploadImage(originalFrame);
    if (timingEnabled) {
        // Apply HSV and BGR Thresholding with timers
        if (hsvEnabled || bgrEnabled) {
            // if (bgrEnabled) {
            //     bgrTimer->timeVoid([&]() {
            //         TestApplyBGRThreshold(originalFrame, tmp1, output, hMin, hMax, sMin, sMax, vMin, vMax);
            //     });
            // }
            if (hsvEnabled) {
                hsvTimer->timeVoid([&]() {
                    ApplyHSVThresholdCUDA(d_originalFrame, (first) ? d_tmp1 : d_tmp2, (first) ? d_output1 : d_output2, hMin, hMax, sMin, sMax, vMin, vMax);
                });
            }
            if (motionEnabled) {
                motionTimer->timeVoid([&]() {
                    ApplyMotionThresholdConsecutivelyCUDA(d_originalFrame, (first) ? d_output1 : d_output2, backSubCUDA, (first) ? d_fgMask1 : d_fgMask2, (first) ? d_tmpGray1 : d_tmpGray2);
                });
            }
        } else if (motionEnabled) {
            motionTimer->timeVoid([&]() {
                ApplyMotionThresholdCUDA(d_originalFrame, (first) ? d_output1 : d_output2, backSubCUDA, (first) ? d_fgMask1 : d_fgMask2, (first) ? d_tmpGray1 : d_tmpGray2);
            });
        }
        if (morphEnabled) {
            morphTimer->timeVoid([&]() {
                ApplyMorphologyCUDA((first) ? d_output1 : d_output2,
                                    (first) ? erodeFilter1 : erodeFilter2,
                                    (first) ? dilateFilter1 : dilateFilter2);
            });
        }
    } else {
        // If timers are not enabled, process normally (no timers)
        if (hsvEnabled || bgrEnabled) {
            // if (bgrEnabled) {
            //     TestApplyBGRThreshold(originalFrame, tmp1, output, hMin, hMax, sMin, sMax, vMin, vMax);
            // }
            if (hsvEnabled) {
                ApplyHSVThresholdCUDA(d_originalFrame, (first) ? d_tmp1 : d_tmp2, (first) ? d_output1 : d_output2, hMin, hMax, sMin, sMax, vMin, vMax);
            }
            if (motionEnabled) {
                ApplyMotionThresholdConsecutivelyCUDA(d_originalFrame, (first) ? d_output1 : d_output2, backSubCUDA, (first) ? d_fgMask1 : d_fgMask2, (first) ? d_tmpGray1 : d_tmpGray2);
            }
        } else if (motionEnabled) {
            ApplyMotionThresholdCUDA(d_originalFrame, (first) ? d_output1 : d_output2, backSubCUDA, (first) ? d_fgMask1 : d_fgMask2, (first) ? d_tmpGray1 : d_tmpGray2);
        }
        if (morphEnabled) {
            ApplyMorphologyCUDA((first) ? d_output1 : d_output2,
                                (first) ? erodeFilter1 : erodeFilter2,
                                (first) ? dilateFilter1 : dilateFilter2);
        }
    }
    (first) ? d_output1.download(output) : d_output2.download(output);
}

cv::Point3f MainWindow::handleCentroids(const cv::Point &centroid1, const cv::Point &centroid2) {
    // Perform triangulation here if both centroids are valid
    if (!P1.empty() && !P2.empty()) {
        // Prepare the points for triangulation
        cv::Mat pts1 = (cv::Mat_<double>(2, 1) << static_cast<double>(centroid1.x), static_cast<double>(centroid1.y));
        cv::Mat pts2 = (cv::Mat_<double>(2, 1) << static_cast<double>(centroid2.x), static_cast<double>(centroid2.y));

        cv::Mat points4D;
        cv::triangulatePoints(P1, P2, pts1, pts2, points4D);

        // Convert from homogeneous coordinates (4x1) to Euclidean (3x1)
        cv::Mat point3D = points4D.rowRange(0, 3) / points4D.at<double>(3, 0);

        // Return the triangulated 3D point
        return cv::Point3f(point3D.at<double>(0), point3D.at<double>(1), point3D.at<double>(2));
    } else {
        qDebug() << "Projection matrices are empty, cannot triangulate.";
        return cv::Point3f(-1, -1, -1);
    }
}

void MainWindow::updateCentroid(const cv::Point3f& newCentroid, double timestamp) {
    qDebug() << "New centroid: " << newCentroid.x << " " << newCentroid.y << " " << newCentroid.z;
    if (newCentroid == cv::Point3f(-1,-1,-1)) return;
    QMutexLocker locker(&centroidMutex);
    motorCameraCalibrationCurrentCentroid = newCentroid;
    if (capturingCentroids) {
        CentroidData data = {newCentroid.x, newCentroid.y, newCentroid.z, timestamp};
        centroidData.push_back(data);
        if (centroidData.size() == 1) frameTimer.start();
    }
    if (centroidData.size() >= ui.predictionFramesSpinBox->value() && !predicted) {
        predicted = true;
        std::vector<CentroidData> clone = centroidData;
        emit predictionDataReady(clone);
        qint64 frameTime = frameTimer.elapsed();
        qDebug() << "Time taken to collect frames: " << frameTime << "ms";
        frameTimer.restart();
    }
}

cv::Point3f MainWindow::getCentroid() {
    qDebug() << "in get centroid";
    QMutexLocker locker(&centroidMutex);
    qDebug() << "created locker";
    qDebug() << "reutrning centroid " << motorCameraCalibrationCurrentCentroid.x << " " << motorCameraCalibrationCurrentCentroid.y << " " << motorCameraCalibrationCurrentCentroid.z;
    return motorCameraCalibrationCurrentCentroid;  // Safe copy
}

void MainWindow::saveCentroidListToJson() {
    // Convert the vector to a JSON object
    QString filename = "../../CentroidData/" + ui.centroidLineEdit->text().trimmed() + ".json";
    nlohmann::json jsonData = centroidData;

    // Open the file and write the JSON data
    std::ofstream file(filename.toStdString());
    if (file.is_open()) {
        file << jsonData.dump(4);  // Pretty print with indentation of 4 spaces
        file.close();
        qDebug() << "Saved centroid data to " << filename;
    } else {
        qDebug() << "Error opening file for saving centroids.";
    }
    centroidData.clear();
}

void MainWindow::toggleCaptureCentroid() {
    capturingCentroids = !capturingCentroids;
    qDebug() << "Centroid capture set to " << capturingCentroids;
    if (capturingCentroids) {
        centroidData.clear();
        ui.startCentroidPushButton->setText("Stop Centroid Capture");
    } else {
        ui.startCentroidPushButton->setText("Start Centroid Capture");
    }
}


void to_json(nlohmann::json& j, const CentroidData& data) {
    j = nlohmann::json{{"x", data.x}, {"y", data.y}, {"z", data.z}, {"t", data.t}};
}

void MainWindow::togglePrediction() {
    QMutexLocker locker(&mutex);

    predicting = !predicting;
    qDebug() << "Predicting set to " << predicting;

    if (predicting) {
        centroidData.clear();
        predicted = false;
        ui.startPredictionPushButton->setText("Stop Prediction");
        if (!capturingCentroids) toggleCaptureCentroid();
    } else {
        ui.startPredictionPushButton->setText("Start Prediction");
        // Allow the process to finish naturally, but don't start new ones
    }
}

cv::Point3f MainWindow::runPrediction() {
    if (!predicting) return cv::Point3f(-1, -1, -1);

    QElapsedTimer timer;
    cv::Point3f point;


    timer.start();
    InitialConditions result = determineInitialConditions(centroidData, gravity_vector);
    point = getInterceptionPoint(result, ui.interceptionDelaySpinBox->value() / 1000.0);
    qint64 interceptionTime = timer.elapsed();
    qDebug() << "Prediction took:" << interceptionTime << "ms";

    timer.restart();
    aimAtInterceptionPoint(point);
    qint64 aimTime = timer.elapsed();
    qDebug() << "aimAtInterceptionPoint took:" << aimTime << "ms";

    return point;
}

InitialConditions MainWindow::determineInitialConditions(const std::vector<CentroidData>& centroid_data,
                                                         const std::vector<double>& gravity_vector) {
    x_vals.clear(); y_vals.clear(); z_vals.clear(); t_vals.clear();
    double t0 = centroid_data[0].t;
    for (CentroidData point : centroid_data) {
        x_vals.push_back(point.x / 1000);
        y_vals.push_back(point.y / 1000);
        z_vals.push_back(point.z / 1000);
        t_vals.push_back((point.t - t0) / 1000000);
    }

    double ax = gravity_vector[0];
    double ay = gravity_vector[1];
    double az = gravity_vector[2];

    Eigen::Vector2d x_params = fitTrajectory(t_vals, x_vals, ax);
    Eigen::Vector2d y_params = fitTrajectory(t_vals, y_vals, ay);
    Eigen::Vector2d z_params = fitTrajectory(t_vals, z_vals, az);

    InitialConditions result;
    result.initial_position = {x_params[0], y_params[0], z_params[0]};
    result.initial_velocity = {x_params[1], y_params[1], z_params[1]};
    result.acceleration = {ax, ay, az};

    qDebug() << "Initial Position: " << result.initial_position[0] << result.initial_position[1] << result.initial_position[2];
    qDebug() << "Initial Velocity: " << result.initial_velocity[0] << result.initial_velocity[1] << result.initial_velocity[2];
    qDebug() << "Acceleration" << result.acceleration[0] << result.acceleration[1] << result.acceleration[2];

    return result;
}

Eigen::Vector2d MainWindow::fitTrajectory(const std::vector<double>& t_vals, const std::vector<double>& coords, double a) {
    int n = t_vals.size();

    // Setup the matrix for least-squares (A * params = coords)
    Eigen::MatrixXd A(n, 2);  // Design matrix: 2 columns (x0, v0)
    Eigen::VectorXd b(n);     // Observed values

    for (int i = 0; i < n; ++i) {
        // Include the full model: x0 + v0*t + 0.5*a*t^2
        // We're solving for x0 and v0, with known a
        A(i, 0) = 1.0;                     // x0 term
        A(i, 1) = t_vals[i];               // v0 term (t)
        b(i) = coords[i] - 0.5 * a * t_vals[i] * t_vals[i]; // Subtract known acceleration term
    }

    // Initial parameter guess (similar to Python)
    Eigen::Vector2d initial_guess;
    if (n >= 2) {
        initial_guess << coords[0], (coords[1] - coords[0]) / (t_vals[1] - t_vals[0]);
    } else {
        initial_guess << coords[0], 0.0;
    }

    // Solve using least squares
    // Starting with initial guess for better matching with Python's curve_fit
    Eigen::Vector2d params = (A.transpose() * A).ldlt().solve(A.transpose() * b);

    // For more accuracy, you could implement Levenberg-Marquardt like curve_fit uses
    // but for this quadratic model, the linear least squares should give identical results
    // as long as the model formulation is correct

    return params;
}

cv::Point3f MainWindow::getInterceptionPoint(const InitialConditions& initial_conditions, double time) {
    float x = ball_model(time, initial_conditions.initial_position[0], initial_conditions.initial_velocity[0], initial_conditions.acceleration[0]);
    float y = ball_model(time, initial_conditions.initial_position[1], initial_conditions.initial_velocity[1], initial_conditions.acceleration[1]);
    float z = ball_model(time, initial_conditions.initial_position[2], initial_conditions.initial_velocity[2], initial_conditions.acceleration[2]);

    return cv::Point3f(x * 1000.0, y * 1000.0, z * 1000.0);
}
