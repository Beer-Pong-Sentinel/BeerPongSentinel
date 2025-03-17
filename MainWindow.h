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
#include "pubSysCls.h"  


    class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);  // Constructor
    time_t start = time(0);
    ~MainWindow();  // Destructor

private slots:
    void toggleCal3DType();
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

    //possible variants: None, thresh and bgsub
    QString  processingType="None";
    cv::Ptr<cv::BackgroundSubtractor> backSub = cv::createBackgroundSubtractorMOG2();
    bool performingMotorCameraCalibration = false;
    std::atomic<bool> stopMotorCameraCalibrationFlag=false;

    // for spherical calibration
    cv::Mat displacementFromLEDPlaneToOrigin = (cv::Mat_<float>(3, 1) << 0, 0, 10.0); // CHANGE THIS
    cv::Mat rotationMatrix;
    cv::Mat sphericalOrigin;

    // triggering
    void fire();

    // for altitude motor
    sFnd::INode* altitudePointer;
    void initializeAltitude();
    void altitudeTest();
    void altitudeTimeTest();

    // for azimuth motor
    void initializeAzimuth();
    void azimuthTest();
    void enableAzimuth();
    void disableAzimuth();
    int azimuthPosition = 0;


    void sendMotorPositions();


};

#endif // MAINWINDOW_H
