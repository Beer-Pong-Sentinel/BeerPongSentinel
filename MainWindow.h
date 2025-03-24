#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"  // Include the generated UI header file (note: had to change the casing so that the generated file would have this casing which was expected by the UI editor_
#include <QtCore/QStringList>
#include "CameraStreamWidget.h"
#include <QtGui/QImage>
#include <opencv2/opencv.hpp>
#include <QSerialPort>
#include <QMutex>
#include <time.h>
#include "ProcessTimer.h"
#include "pubSysCls.h"
#include "json.hpp"


struct LookupEntry {
    double az;      // azimuth angle
    double al;      // altitude angle
    double xo, yo, zo; // near point (origin of the line)
    double ux, uy, uz; // unit direction vector (points from near to far)
};


    class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);  // Constructor
    time_t start = time(0);
    ~MainWindow();  // Destructor

//public slots:
    //void processMotorSettled();

private slots:
    void toggleCal3DType();
    void toggleMotorCameraCalType();
    void updateSavedCameraCalibrationFilesComboBox();
    void calibrateCameraWithSelectedFile();
    void resetNewCameraCalibration();
    void newCameraCalibrationStartImageCapture();
    void newCameraCalibrationSaveImagePair();          // Save image pair function
    void newCal3DStopImageCapture();       // Stop image capture function
    void cancelNewCal3D();
    void calibrateCameras();
    void stopCapture();
    void startCapture();
    void setProcesseing();
    void setBackgroundImage();
    void receiveAndProcessFrames(const cv::Mat &originalFrame1, const cv::Mat &originalFrame2);
    void indicateCameraCalibrationComplete();
    void setupCaptureThreadConnections();
    void onProcessedFramesReady(const cv::Mat &processedFrame1, const cv::Mat &processedFrame2);



signals:
    void processedFramesReady(const cv::Mat &processedFrame1, const cv::Mat &processedFrame2);

private:
    QStringList getDirectoriesSortedByDate(const QString &directoryPath);
    Ui::MainWindow ui;  // UI object to access widgets
    int numberOfSavedImagePairs = 0;
    CameraStreamWidget *cameraStreamWidget;
    CameraCaptureThread *captureThread = nullptr;
    QImage::Format format = QImage::Format_BGR888;
    cv::Mat P1;                      // Projection matrix for camera 1
    cv::Mat P2;                      // Projection matrix for camera 2
    double projectionError = 0.0;
    QSerialPort *serialPort;
    void sendSerialMessage(QString baseMessage);
    void calibrateMotorCamera();
    std::vector<cv::Mat> getLEDCoords();
    std::vector<cv::Point> reorderCentroids(const std::vector<cv::Point>& centroids);
    void sphericalCalibration();

    void sphericalTest();
    void queryMotorPosition(quint16 aziValue, quint16 altValue, QSerialPort &localSerialPort);
    void queryMotorPositionHardCode();
    void setBDValues();
    void saveProcessedFrames();
    void checkLEDDistances();

    //possible variants: None, thresh and bgsub
    QString  processingType="None";
    cv::Ptr<cv::BackgroundSubtractor> backSub =  cv::createBackgroundSubtractorMOG2(100, 16, false); 
    bool performingMotorCameraCalibration = false;
    std::atomic<bool> stopMotorCameraCalibrationFlag=false;

    // for spherical calibration

    cv::Mat displacementTopRightMountHoleToOrigin = (cv::Mat_<double>(3, 1) << -3.5, -3.5, 14.625) * 25.4;
    cv::Mat displacementTopRightBoardHoleToTopRightMountHole = (cv::Mat_<double>(3, 1) << -6, -12, 0) * 25.4;
    cv::Mat displacementRightLEDToTopRightBoardHole = (cv::Mat_<double>(3, 1) << 14, -842, 22); //ROUGH
    cv::Mat displacementRightLEDToOrigin = displacementRightLEDToTopRightBoardHole+displacementTopRightBoardHoleToTopRightMountHole+displacementTopRightMountHoleToOrigin;
    std::vector<double> linspace(double lower, double upper, int num_points);


    cv::Mat rotationMatrix;
    cv::Mat sphericalOrigin;

    // triggering
    void fire();

    // for altitude motor
    sFnd::INode* altitudePointer;
    void initializeAltitude();
    void altitudeTest();
    void altitudeTimeTest();
    void enableAltitude();
    void disableAltitude();
    double altitudeCalLowerLimit = 0.0;
    double altitudeCalUpperLimit = 5.0;
    std::vector<double> altitudeCalPositions;


    // for azimuth motor
    void initializeAzimuth();
    void azimuthTest();
    void enableAzimuth();
    void disableAzimuth();
    int azimuthPosition = 0;
    double azimuthCalLowerLimit = -10.0;
    double azimuthCalUpperLimit = 8.0;
    std::vector<double> azimuthCalPositions;


    // motor camera cal
    void sendMotorPositions();
    void homeMotors();

    void moveAzLimit();
    void moveAlLimit();

    void setAzimuthLowerLimit();
    void setAzimuthUpperLimit();

    void setAltitudeLowerLimit();
    void setAltitudeUpperLimit();

    void sweepLookupTable();

    void toggleNearFarSweep();


    // In your header, add state variables:
    int currentAzIndex = 0;
    int currentAlIndex = 0;
    QTimer *sweepTimer = nullptr;
    void performSweepStep();
    void calculateLookupTable();

    cv::Point3f motorCameraCalibrationCurrentCentroid = cv::Point3f(-1,-1,-1);

    void calculateInterpolatedLookupTable(int newAzCount, int newAlCount);

    std::vector<LookupEntry> interpolatedLookupTable;

    void aimAtCentroid();

    double lastAz = 0.0;
    double lastAl = 0.0;
    nlohmann::json sweepData;

    std::pair<double, double> findFiringAngle(double x, double y, double z);




    // for ball detection
    int thresholdValue, hMax, hMin, sMax, sMin, vMax, vMin;
    bool hsvEnabled = true, bgrEnabled = true, motionEnabled = true, morphEnabled = true, centroidEnabled = true, drawEnabled = false;
    int kernelSize = 5, prevKernelSize = -1;
    cv::Mat backgroundImage1, backgroundImage2;
    cv::Mat tmp1, tmp2;
    cv::Mat tmpGray1, tmpGray2;
    cv::Mat output1, output2;
    cv::Mat kernel;
    cv::Mat processedFrame1, processedFrame2;

    ProcessTimer* hsvTimer = new ProcessTimer("HSV Threshold", 200, 5000, this);
    ProcessTimer* bgrTimer = new ProcessTimer("HSV Threshold", 200, 5000, this);
    ProcessTimer* motionTimer = new ProcessTimer("Motion Threshold", 200, 5000, this);
    ProcessTimer* morphTimer = new ProcessTimer("Morph Close", 200, 5000, this);
    ProcessTimer* centroidTimer = new ProcessTimer("Centroid", 200, 5000, this);
    ProcessTimer* totalTimer = new ProcessTimer("Total Processing", 200, 5000, this);

    cv::Point3f processImageCentroid(const cv::Mat &originalFrame1, const cv::Mat &originalFrame2, bool timingEnabled);
    cv::Point3f processImages(const cv::Mat &originalFrame1, const cv::Mat &originalFrame2, bool timingEnabled);
    void processSingleImage(const cv::Mat &originalFrame, cv::Mat &output, bool timingEnabled);
    cv::Point3f handleCentroids(const std::vector<cv::Point> &centroids1, const std::vector<cv::Point> &centroids2);
};

#endif // MAINWINDOW_H
