#ifndef YOLOINFERENCEWORKER_H
#define YOLOINFERENCEWORKER_H

#pragma once

#include <QObject>
#include <QMutex>
#include <QQueue>
#include <QWaitCondition>
#include <QElapsedTimer>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <QDebug>

struct FramePair {
    cv::Mat frame1;
    cv::Mat frame2;
};

struct TimedFramePair {
    FramePair pair;
    double timestamp;
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

    // Enqueue a pair of frames for inference along with a timestamp.
    void enqueueFramePair(const FramePair &pair, double timestamp) {
        QMutexLocker locker(&queueMutex);
        TimedFramePair tfp;
        tfp.pair = pair;
        tfp.timestamp = timestamp;
        frameQueue.enqueue(tfp);
        queueNotEmpty.wakeOne();
    }

signals:
    // Modified signal that emits the processed frames, their centroids, and the timestamp.
    // If exactly one box is detected in a frame, that box's centroid is emitted.
    // Otherwise, an error value (-1, -1) is emitted.
    void inferenceComplete(const cv::Mat &processedFrame1, const cv::Mat &processedFrame2,
                           cv::Point centroid1, cv::Point centroid2, double timestamp);

public slots:
    // Main loop for processing the frame queue
    void processQueue() {
        while (true) {
            queueMutex.lock();
            while (frameQueue.isEmpty()) {
                queueNotEmpty.wait(&queueMutex);
            }
            // Flush the queue to process only the latest timed frame pair.
            TimedFramePair tfp = frameQueue.dequeue();
            while (!frameQueue.isEmpty()) {
                tfp = frameQueue.dequeue();
            }
            queueMutex.unlock();

            // Process the latest pair.
            if (!tfp.pair.frame1.empty() && !tfp.pair.frame2.empty()) {
                QElapsedTimer timer;
                timer.start();
                cv::Mat proc1 = tfp.pair.frame1.clone();
                cv::Mat proc2 = tfp.pair.frame2.clone();
                cv::Point centroid1 = processFrame(proc1);
                cv::Point centroid2 = processFrame(proc2);
                emit inferenceComplete(proc1, proc2, centroid1, centroid2, tfp.timestamp);
                qDebug() << "YOLO processing both frames took:" << timer.elapsed() << "ms";
            }
        }
    }

private:
    // Process a single frame: run inference, draw detection boxes and the centroid.
    // Returns the centroid if exactly one detection is found; otherwise returns (-1,-1)
    cv::Point processFrame(cv::Mat &frame) {

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

        cv::Point computedCentroid(-1, -1);
        int validDetections = 0;

        // Draw all boxes and compute centroids
        for (size_t i = 0; i < boxes.size(); i++) {
            cv::rectangle(frame, boxes[i], cv::Scalar(0, 255, 0), 2);
            std::string label = cv::format("t: %.2f", confidences[i]);
            cv::putText(frame, label, boxes[i].tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

            cv::Point centroid(boxes[i].x + boxes[i].width / 2,
                               boxes[i].y + boxes[i].height / 2);
            // Draw the centroid as a red dot
            cv::circle(frame, centroid, 3, cv::Scalar(0, 0, 255), -1);
            validDetections++;

            // If this is the first (and ideally only) valid detection, store its centroid.
            if (validDetections == 1) {
                computedCentroid = centroid;
            }
        }

        // If not exactly one detection is found, mark as error.
        if (validDetections != 1) {
            computedCentroid = cv::Point(-1, -1);
            if (validDetections > 1)
                qDebug() << "Multiple centroids found";
            else if (validDetections == 0)
                qDebug() << "No centroid found";
        }
        return computedCentroid;
    }

    cv::dnn::Net &net;
    cv::Size inputSize;
    float confThreshold;

    // Frame queue for asynchronous processing, now using TimedFramePair.
    QQueue<TimedFramePair> frameQueue;
    QMutex queueMutex;
    QWaitCondition queueNotEmpty;
};

#endif // YOLOINFERENCEWORKER_H
