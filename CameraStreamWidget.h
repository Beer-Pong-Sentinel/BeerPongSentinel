#ifndef CAMERASTREAMWIDGET_H
#define CAMERASTREAMWIDGET_H

#include <QtOpenGLWidgets/QOpenGLWidget>
#include <QtGui/QOpenGLFunctions>
#include <QOpenGLTexture>
#include <opencv2/opencv.hpp>
#include <QImage>
#include <QMutex>
#include <QElapsedTimer>
#include <QTimer>
#include "CameraCaptureThread.h"

class CameraCaptureThread;

class CameraStreamWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    explicit CameraStreamWidget(QWidget *parent = nullptr);
    ~CameraStreamWidget() override;
    void setDefaultBlackFrame();

public slots:
    void updateFrame(const cv::Mat &frame1, const cv::Mat &frame2, QImage::Format format);

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;

private:
    QMutex frameMutex;
    QImage currentFrame1;
    QImage currentFrame2;
    QImage fullFrame;

    QOpenGLTexture *m_texture;
    QElapsedTimer m_frameTimer;
    qint64 m_lastUpdateTime;
    qint64 m_updateInterval; // in milliseconds
    // Remove m_pendingUpdate: we rely on a fixed QTimer

    QTimer *refreshTimer; // Timer to trigger repaint at fixed intervals
};

#endif // CAMERASTREAMWIDGET_H
