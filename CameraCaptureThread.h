#ifndef CAMERACAPTURETHREAD_H
#define CAMERACAPTURETHREAD_H

#include <QtCore/QThread>
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>
#include <QtGui/QImage>
#include <QtCore/QMutex>
#include <opencv2/opencv.hpp>


class CameraStreamWidget;

class CameraCaptureThread : public QThread {
    Q_OBJECT

public:
    explicit CameraCaptureThread(CameraStreamWidget *parent);
    ~CameraCaptureThread();
    void run() override;
    bool initializeCameras();
    void startCapture();
    void stopCapture();
    bool isCameraAvailable() const;
    cv::Mat getFrame1();
    cv::Mat getFrame2();

signals:
    void newFrameReady(const cv::Mat &frame1, const cv::Mat &frame2, double timestamp);
    void stoppedCapture();

private:
    CameraStreamWidget *widget;  // Pointer to the widget to update with frames
    Spinnaker::CameraPtr camera1;
    Spinnaker::CameraPtr camera2;
    bool running = true;
    bool cameraAvailable = false;
    Spinnaker::SystemPtr system;
    Spinnaker::CameraList camList; 
    mutable QMutex frameMutex;  // Mutex to protect frame access
    cv::Mat frame1;  // Latest frame from camera 1
    cv::Mat frame2;  // Latest frame from camera 2
    double timestamp; // timestamp of frame 1 in ms
};

#endif // CAMERACAPTURETHREAD_H
