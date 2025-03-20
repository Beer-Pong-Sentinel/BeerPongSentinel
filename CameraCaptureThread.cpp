#include "CameraStreamWidget.h"
#include <opencv2/opencv.hpp>
#include <QtGui/QImage>
#include <QtCore/QMutexLocker>
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>
#include <chrono>
#include <thread>
#include <QtGui/QPainter>
#include "CameraSetup.h"

CameraCaptureThread::CameraCaptureThread(CameraStreamWidget *parent)
    : QThread(parent), widget(parent), system(nullptr), camera1(nullptr), camera2(nullptr), running(false), cameraAvailable(false) {
    // No heavy initialization here; we defer that to initializeCameras()
    //system = Spinnaker::System::GetInstance();
}

bool CameraCaptureThread::initializeCameras() {
    // Clear previous camera list and system if they were previously allocated
    qDebug()<<"In initialize cameras";
    if (system) {
        camList.Clear();
        qDebug()<<"cleared camlist";
        system->ReleaseInstance();
        qDebug()<<"released system instance";
    }

    system = Spinnaker::System::GetInstance();
    qDebug()<<"created new system instance";
    camList = system->GetCameras();
    qDebug()<<"created new camlist";

    if (camList.GetSize() >= 2) {
        camera1 = camList.GetByIndex(0);
        camera2 = camList.GetByIndex(1);

        camera1->Init();
        qDebug()<<"init camera1";
        INodeMap& nodeMap1 = camera1->GetNodeMap();
        qDebug()<<"got camera1 nodepam";
        ConfigureCameraSettings(nodeMap1);


        qDebug()<<"configured camera1 settings";
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        qDebug()<<"waited 500ms";
        camera2->Init();
        qDebug()<<"init camera2";        
        INodeMap& nodeMap2 = camera2->GetNodeMap();
        qDebug()<<"got camera2 nodemap";        
        ConfigureCameraSettings(nodeMap2);
        qDebug()<<"configured camera2 settings";

        cameraAvailable = true;
    } else {
        qDebug() << "Not enough cameras found!";
        cameraAvailable = false;
    }

    return isCameraAvailable();
}


void CameraCaptureThread::startCapture() {
    if (!cameraAvailable) {
        qDebug() << "Cameras are not available. Initialization failed.";
        return;
    }
    if (!camera1->IsStreaming()) camera1->BeginAcquisition();
    if (!camera2->IsStreaming()) camera2->BeginAcquisition();
    running = true;
    qDebug()<<"Running is true, starting thread";
    start();  // Starts the thread, triggering the `run` method.
}

CameraCaptureThread::~CameraCaptureThread() {

    qDebug()<<"in destructor";

    // Stop the capture if it's still running
    if (running) {
        stopCapture();
        wait();  // Wait for the thread to finish cleanly
        qDebug()<<"stopped capture";
    }

    // End acquisition and deinitialize each camera if they are initialized
    if (camera1) {
        if (camera1->IsStreaming()) {
            camera1->EndAcquisition();
            qDebug()<<"ended cam1 acquisition in desctruction";
        }
        camera1->DeInit();
        qDebug()<<"deinitialized cam1";
        camera1 = nullptr;  // Release camera pointer
    }

    if (camera2) {
        if (camera2->IsStreaming()) {
            camera2->EndAcquisition();
            qDebug()<<"ended cam2 acquis in desctructor";
        }
        camera2->DeInit();
        qDebug()<<"deinit cam2";
        camera2 = nullptr;  // Release camera pointer
    }

    // Clear the camera list and release the Spinnaker system instance
    camList.Clear();
    if (system) {
        system->ReleaseInstance();
        qDebug()<<"released system instance in desctructor";
        system = nullptr;  // Release system pointer
    }
}


bool CameraCaptureThread::isCameraAvailable() const {
    return cameraAvailable;
}

std::atomic<bool> processingFrame(false); // Flag to track processing

void CameraCaptureThread::run() {
    if (!camera1 || !camera2 || !running) {
        qDebug() << "Cameras not available or capture not started, aborting capture thread.";
        return;
    }

    while (running) {
        try {
            Spinnaker::ImagePtr image1 = camera1->GetNextImage();
            Spinnaker::ImagePtr image2 = camera2->GetNextImage();

            if (image1->IsIncomplete() || image2->IsIncomplete()) {
                image1->Release();
                image2->Release();
                continue;
            }

            cv::Mat matFrame1(image1->GetHeight(), image1->GetWidth(), CV_8UC3, image1->GetData());
            cv::Mat matFrame2(image2->GetHeight(), image2->GetWidth(), CV_8UC3, image2->GetData());

            cv::Mat tempFrame1 = matFrame1.clone();
            cv::Mat tempFrame2 = matFrame2.clone();

            if (!processingFrame.exchange(true)) {  // If processing is not happening, update the frame
                {
                    QMutexLocker locker(&frameMutex);
                    frame1 = tempFrame1;
                    frame2 = tempFrame2;
                }
                emit newFrameReady(frame1, frame2);
            }
            processingFrame = false;  // Reset the flag

            image1->Release();
            image2->Release();

        } catch (Spinnaker::Exception &e) {
            qDebug() << "Spinnaker error:" << e.what();
        }
    }
}

void CameraCaptureThread::stopCapture() {
    if (!running) return;  // Prevent duplicate stop attempts
    running = false;

    qDebug() << "Stopping camera acquisition...";
    emit stoppedCapture();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    

    // Stop acquisition on each camera and ensure trigger mode is set to "Off"
    if (camera1) {
        try {
            if (camera1->IsStreaming()) {
                camera1->EndAcquisition();
                qDebug() << "camera1 stopped acquisition...";
                qDebug()<<"bruh";
                //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            }
            //SetEnumerationNode(camera1->GetNodeMap(), "TriggerMode", "Off");
            camera1->DeInit();
            qDebug() << "Camera 1 deinitialized.";
        } catch (const Spinnaker::Exception &e) {
            qDebug() << "Error stopping camera 1:" << e.what();
        }
        camera1 = nullptr;
    }

    if (camera2) {
        try {
            if (camera2->IsStreaming()) {
                camera2->EndAcquisition();
            }
            SetEnumerationNode(camera2->GetNodeMap(), "TriggerMode", "Off");
            camera2->DeInit();
            qDebug() << "Camera 2 deinitialized.";
        } catch (const Spinnaker::Exception &e) {
            qDebug() << "Error stopping camera 2:" << e.what();
        }
        camera2 = nullptr;
    }

    // Clear the camera list and release the Spinnaker system
    camList.Clear();
    if (system) {
        system->ReleaseInstance();
        system = nullptr;
        qDebug() << "Spinnaker system released.";
    }

    qDebug() << "Capture fully stopped and resources released.";
}



cv::Mat CameraCaptureThread::getFrame1() {
    QMutexLocker locker(&frameMutex);  // Lock mutex for thread-safe access
    return frame1;
}

cv::Mat CameraCaptureThread::getFrame2() {
    QMutexLocker locker(&frameMutex);  // Lock mutex for thread-safe access
    return frame2;
}
