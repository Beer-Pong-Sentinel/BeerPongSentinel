#include "CameraStreamWidget.h"
#include <QOpenGLTexture>
#include <QPainter>
#include <QDebug>

CameraStreamWidget::CameraStreamWidget(QWidget *parent)
    : QOpenGLWidget(parent)
    , m_texture(nullptr)
    , m_lastUpdateTime(0)
    , m_updateInterval(100)
{
    setFixedSize(720, 270);
    setDefaultBlackFrame();
    m_frameTimer.start();

    // Create a QTimer that triggers repaint at a fixed interval.
    refreshTimer = new QTimer(this);
    connect(refreshTimer, &QTimer::timeout, this, QOverload<>::of(&CameraStreamWidget::update));
    refreshTimer->start(m_updateInterval);
}

CameraStreamWidget::~CameraStreamWidget() {
    makeCurrent();
    delete m_texture;
    doneCurrent();
}

void CameraStreamWidget::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0, 0, 0, 1);

    qDebug() << "OpenGL Renderer:" << reinterpret_cast<const char*>(glGetString(GL_RENDERER));
    qDebug() << "OpenGL Vendor:" << reinterpret_cast<const char*>(glGetString(GL_VENDOR));
    qDebug() << "OpenGL Version:" << reinterpret_cast<const char*>(glGetString(GL_VERSION));
}

void CameraStreamWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
}

void CameraStreamWidget::paintGL() {
    qint64 now = m_frameTimer.elapsed();
    if (now - m_lastUpdateTime < m_updateInterval) {
        return;  // Not enough time has passed, so skip repainting.
    }
    m_lastUpdateTime = now;

    glClear(GL_COLOR_BUFFER_BIT);

    QMutexLocker locker(&frameMutex);

    if (!fullFrame.isNull()) {
        // Create or update the texture from fullFrame.
        if (!m_texture) {
            m_texture = new QOpenGLTexture(fullFrame);
            m_texture->setMinificationFilter(QOpenGLTexture::Linear);
            m_texture->setMagnificationFilter(QOpenGLTexture::Linear);
            m_texture->setWrapMode(QOpenGLTexture::ClampToEdge);
        } else {
            // Update texture with new image data.
            m_texture->destroy();
            m_texture->create();
            m_texture->setData(fullFrame);
        }

        glEnable(GL_TEXTURE_2D);
        m_texture->bind();

        glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 0.0f); glVertex2f(-1.0f,  1.0f);  // Top-left
        glTexCoord2f(1.0f, 0.0f); glVertex2f( 1.0f,  1.0f);  // Top-right
        glTexCoord2f(1.0f, 1.0f); glVertex2f( 1.0f, -1.0f);  // Bottom-right
        glTexCoord2f(0.0f, 1.0f); glVertex2f(-1.0f, -1.0f);  // Bottom-left
        glEnd();

        m_texture->release();
        glDisable(GL_TEXTURE_2D);
    }
}


void CameraStreamWidget::setDefaultBlackFrame() {
    qDebug() << "Setting default black frame";
    // Create black QImages.
    currentFrame1 = QImage(360, 270, QImage::Format_RGB888);
    currentFrame2 = QImage(360, 270, QImage::Format_RGB888);
    currentFrame1.fill(Qt::black);
    currentFrame2.fill(Qt::black);

    fullFrame = QImage(720, 270, QImage::Format_RGB888);
    fullFrame.fill(Qt::black);

    update(); // Repaint.
}

void CameraStreamWidget::updateFrame(const cv::Mat &frame1, const cv::Mat &frame2, QImage::Format format) {
    QMutexLocker locker(&frameMutex);

    // Instead of queuing many updates, simply replace the current frames with the latest.
    if (!frame1.empty()) {
        currentFrame1 = QImage(frame1.data, frame1.cols, frame1.rows, frame1.step, format)
        .scaled(frame1.cols / 2, frame1.rows / 2, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    }
    if (!frame2.empty()) {
        currentFrame2 = QImage(frame2.data, frame2.cols, frame2.rows, frame2.step, format)
        .scaled(frame2.cols / 2, frame2.rows / 2, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    }

    // Concatenate side-by-side.
    if (!currentFrame1.isNull() && !currentFrame2.isNull() && currentFrame1.height() == currentFrame2.height()) {
        fullFrame = QImage(currentFrame1.width() + currentFrame2.width(), currentFrame1.height(), currentFrame1.format());
        QPainter painter(&fullFrame);
        painter.drawImage(0, 0, currentFrame1);
        painter.drawImage(currentFrame1.width(), 0, currentFrame2);
        painter.end();
    }
    // We do not call update() here. The QTimer already triggers repaint at a fixed interval,
    // ensuring that only the latest frame is displayed.
}
