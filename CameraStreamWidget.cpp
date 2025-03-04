#include "CameraStreamWidget.h"
#include <QtGui/QOpenGLFunctions>
#include <QtGui/QPainter>
#include <QtCore/QDebug>

CameraStreamWidget::CameraStreamWidget(QWidget *parent)
    : QOpenGLWidget(parent) {
    setFixedSize(720, 270);  // Set the widget to a fixed size
    setDefaultBlackFrame();    // Display a black screen initially
}

CameraStreamWidget::~CameraStreamWidget() {
}

void CameraStreamWidget::initializeGL() {
    initializeOpenGLFunctions();

    // Print OpenGL renderer, vendor, and version information
    const GLubyte* renderer = glGetString(GL_RENDERER); // GPU being used
    const GLubyte* vendor = glGetString(GL_VENDOR);     // GPU vendor
    const GLubyte* version = glGetString(GL_VERSION);   // OpenGL version

    qDebug() << "OpenGL Renderer:" << reinterpret_cast<const char*>(renderer);
    qDebug() << "OpenGL Vendor:" << reinterpret_cast<const char*>(vendor);
    qDebug() << "OpenGL Version:" << reinterpret_cast<const char*>(version);
}

void CameraStreamWidget::paintGL() {
    QMutexLocker locker(&frameMutex);  // Lock access to the frames

    // Check if both frames are valid and have the same height
    if (!currentFrame1.isNull() && !currentFrame2.isNull() && currentFrame1.height() == currentFrame2.height()) {
        // Create a new QImage, fullFrame, with dimensions 1440x540 (width x height) to hold both images side by side
        QImage fullFrame(currentFrame1.width() + currentFrame2.width(), currentFrame1.height(), currentFrame1.format());

        // Use a QPainter to draw currentFrame1 and currentFrame2 into fullFrame
        QPainter painter(&fullFrame);
        painter.drawImage(0, 0, currentFrame1);  // Draw currentFrame1 on the left side of fullFrame
        painter.drawImage(currentFrame1.width(), 0, currentFrame2);  // Draw currentFrame2 on the right side of fullFrame
        painter.end();

        // Now paint the fullFrame onto the widget
        QPainter widgetPainter(this);
        widgetPainter.drawImage(rect(), fullFrame);  // Draw the concatenated image to fill the widget
    }
}


void CameraStreamWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
}

void CameraStreamWidget::setDefaultBlackFrame() {

    qDebug()<<"setting default black frame";
    // Set current frames to black images to reset any previous frame data
    currentFrame1 = QImage(360, 270, QImage::Format_RGB888);
    currentFrame2 = QImage(360, 270, QImage::Format_RGB888);
    currentFrame1.fill(Qt::black);
    currentFrame2.fill(Qt::black);

    // Concatenate them into a full black frame
    fullFrame = QImage(720, 270, QImage::Format_RGB888);
    fullFrame.fill(Qt::black);

    update();  // Trigger a repaint to show the black screen
}


void CameraStreamWidget::updateFrame(const cv::Mat &frame1, const cv::Mat &frame2) {
    QMutexLocker locker(&frameMutex);

    // Convert cv::Mat to QImage for display and resize by a factor of 2
    if (!frame1.empty()) {
        // Convert to QImage and scale down to half size
        currentFrame1 = QImage(frame1.data, frame1.cols, frame1.rows, frame1.step, QImage::Format_BGR888)
                            .scaled(frame1.cols / 2, frame1.rows / 2, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    }

    if (!frame2.empty()) {
        // Convert to QImage and scale down to half size
        currentFrame2 = QImage(frame2.data, frame2.cols, frame2.rows, frame2.step, QImage::Format_BGR888)
                            .scaled(frame2.cols / 2, frame2.rows / 2, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    }

    update();  // Trigger a repaint
}
