#ifndef YOLOINFERENCEWORKER_H
#define YOLOINFERENCEWORKER_H

#endif // YOLOINFERENCEWORKER_H

// YOLOInferenceWorker.h
#pragma once

#include <QObject>
#include <QMutex>
#include <QQueue>
#include <QWaitCondition>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <QDebug>


struct FramePair {
    cv::Mat frame1;
    cv::Mat frame2;
};

class YOLOInferenceWorker : public QObject {
    Q_OBJECT
public:
    YOLOInferenceWorker(cv::dnn::Net &net, const cv::Size &inputSize, float confThreshold, QObject *parent = nullptr)
        : QObject(parent)
        , net(net)
        , inputSize(inputSize)
        , confThreshold(confThreshold)
    {

        // Create a dummy image matching the network input size
        cv::Mat dummy = cv::Mat::zeros(inputSize, CV_8UC3);
        cv::Mat blob;
        cv::dnn::blobFromImage(dummy, blob, 1.0 / 255.0, inputSize, cv::Scalar(0, 0, 0), true, false);
        net.setInput(blob);
        // Run a forward pass to warm up the network
        cv::Mat warmupOutput = net.forward();
        qDebug() << "Model warm-up complete.";

    }

    // Enqueue a pair of frames for inference
    void enqueueFramePair(const FramePair &pair) {
        QMutexLocker locker(&queueMutex);
        frameQueue.enqueue(pair);
        queueNotEmpty.wakeOne();
    }

signals:
    // Signal emitted when a frame pair has been processed
    void inferenceComplete(const cv::Mat &processedFrame1, const cv::Mat &processedFrame2);

public slots:
    // Main loop for processing the frame queue
    void processQueue() {
        while (true) {
            queueMutex.lock();
            while (frameQueue.isEmpty()) {
                queueNotEmpty.wait(&queueMutex);
            }
            // Instead of taking the first frame pair,
            // flush the queue so that we process only the latest frame pair.
            FramePair pair = frameQueue.dequeue();
            while (!frameQueue.isEmpty()) {
                pair = frameQueue.dequeue();
            }
            queueMutex.unlock();

            // Process the latest pair.
            if (!pair.frame1.empty() && !pair.frame2.empty()) {
                QElapsedTimer timer;
                timer.start();
                cv::Mat proc1 = pair.frame1.clone();
                cv::Mat proc2 = pair.frame2.clone();
                processFrame(proc1);
                processFrame(proc2);
                emit inferenceComplete(proc1, proc2);
                qDebug() << "YOLO processing both frames took:" << timer.elapsed() << "ms";
            }
        }
    }


private:
    // Process a single frame: run inference and draw detection boxes
    void processFrame(cv::Mat &frame) {

        cv::Mat blob;
        // Create blob from image
        cv::dnn::blobFromImage(frame, blob, 1.0/255.0, inputSize, cv::Scalar(0,0,0), true, false);
        net.setInput(blob);
        cv::Mat outputs = net.forward();

        // Assume output shape is (1, 5, 8400). Reshape to (8400, 5):
        cv::Mat detectionMat = outputs.reshape(1, outputs.size[1]); // shape (5, 8400)
        cv::transpose(detectionMat, detectionMat); // shape (8400, 5)

        float x_factor = static_cast<float>(frame.cols) / inputSize.width;
        float y_factor = static_cast<float>(frame.rows) / inputSize.height;

        std::vector<cv::Rect> boxes;
        std::vector<float> confidences;

        float *data = reinterpret_cast<float*>(detectionMat.data);
        int numDetections = detectionMat.rows; // e.g. 8400

        for (int i = 0; i < numDetections; ++i) {
            // Each detection: [center_x, center_y, width, height, confidence]
            float cx = data[0], cy = data[1], w = data[2], h = data[3], conf = data[4];
            if (conf > confThreshold) {
                confidences.push_back(conf);
                int left = static_cast<int>((cx - 0.5f * w) * x_factor);
                int top  = static_cast<int>((cy - 0.5f * h) * y_factor);
                int width = static_cast<int>(w * x_factor);
                int height = static_cast<int>(h * y_factor);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
            data += 5;
        }
        // (Optionally, apply NMS here to filter overlapping boxes.)
        for (size_t i = 0; i < boxes.size(); i++) {
            cv::rectangle(frame, boxes[i], cv::Scalar(0, 255, 0), 2);
            std::string label = cv::format("t: %.2f", confidences[i]);
            cv::putText(frame, label, boxes[i].tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }

    }

    cv::dnn::Net &net;
    cv::Size inputSize;
    float confThreshold;

    // Frame queue for asynchronous processing
    QQueue<FramePair> frameQueue;
    QMutex queueMutex;
    QWaitCondition queueNotEmpty;
};
