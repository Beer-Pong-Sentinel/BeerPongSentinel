#ifndef CAMERASTREAMWIDGET_H
#define CAMERASTREAMWIDGET_H

#include <QtOpenGLWidgets/QOpenGLWidget>
#include <QtGui/QOpenGLFunctions>
#include <opencv2/opencv.hpp>
#include <QtGui/QImage>
#include <QtCore/QMutex>
#include <QtCore/QThread>
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include "CameraCaptureThread.h"

class CameraCaptureThread;

class CameraStreamWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    explicit CameraStreamWidget(QWidget *parent = nullptr);
    ~CameraStreamWidget();
    void setDefaultBlackFrame();

public slots:

    void updateFrame(const cv::Mat &frame1, const cv::Mat &frame2);  // Slot to receive and display frames

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;


private:
    friend class CameraCaptureThread;
    QImage currentFrame1;  // Store the latest frame from camera 1
    QImage currentFrame2; 
    QImage fullFrame;
    QMutex frameMutex;
    bool cameraFound = false;
    int frameCounter = 0;
    int displayInterval = 5;
    int processingType=0; //0 is none, 1 is thresholding, 2 is BGSub
       


    CameraCaptureThread *captureThread;  // Thread for capturing frames
};

#endif // CAMERASTREAMWIDGET_H
