// ProcessTimer.cpp
#include "ProcessTimer.h"

ProcessTimer::ProcessTimer(const QString& name, int bufferSize, int reportIntervalMs, QObject* parent)
    : QObject(parent)
    , m_name(name)
    , m_bufferSize(bufferSize)
    , m_currentIndex(0)
    , m_count(0)
    , m_reportIntervalMs(reportIntervalMs)
    , m_totalTime(0)
    , m_maxTime(0)
    , m_minTime(std::numeric_limits<qint64>::max())
{
    m_measurements.resize(m_bufferSize);
    m_reportTimer.start();
}

ProcessTimer::~ProcessTimer()
{
    // Report final statistics if we have any measurements
    if (m_count > 0) {
        reportNow();
    }
}

void ProcessTimer::recordMeasurement(qint64 elapsed)
{
    // Store the new measurement
    m_measurements[m_currentIndex] = elapsed;

    // Update total time by removing old measurement (if we've wrapped around)
    // and adding new measurement
    if (m_count == m_bufferSize) {
        m_totalTime -= m_measurements[(m_currentIndex + 1) % m_bufferSize];
    }
    m_totalTime += elapsed;

    // Update max and min
    m_maxTime = qMax(m_maxTime, elapsed);
    m_minTime = qMin(m_minTime, elapsed);

    // Update index and count
    m_currentIndex = (m_currentIndex + 1) % m_bufferSize;
    m_count = qMin(m_count + 1, m_bufferSize);

    // Check if it's time to report
    checkReportTime();
}

void ProcessTimer::checkReportTime()
{
    if (m_reportTimer.elapsed() > m_reportIntervalMs) {
        reportNow();
    }
}

void ProcessTimer::reportNow()
{
    if (m_count > 0) {
        qint64 average = m_totalTime / m_count;
        qDebug() << "==============================" << '\n'
                 << "TIMER [" << m_name << "] \n"
                 << "Avg:" << average << "ms \n"
                 << "Min:" << m_minTime << "ms \n"
                 << "Max:" << m_maxTime << "ms \n"
                 << "Count:" << m_count << '\n'
                 << "==============================";

        m_reportTimer.restart();
    }
}

void ProcessTimer::reset()
{
    m_currentIndex = 0;
    m_count = 0;
    m_totalTime = 0;
    m_maxTime = 0;
    m_minTime = std::numeric_limits<qint64>::max();
    m_reportTimer.restart();
}
