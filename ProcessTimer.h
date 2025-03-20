#pragma once

#include <QObject>
#include <QElapsedTimer>
#include <QVector>
#include <QString>
#include <QDebug>
#include <functional>

class ProcessTimer : public QObject
{
    Q_OBJECT

public:
    explicit ProcessTimer(const QString& name, int bufferSize = 50, int reportIntervalMs = 2000, QObject* parent = nullptr);
    ~ProcessTimer();

    // Time a function with no arguments and return its result
    template<typename Func>
    auto time(Func&& func) -> decltype(func())
    {
        QElapsedTimer timer;
        timer.start();
        
        auto result = func();
        
        recordMeasurement(timer.elapsed());
        return result;
    }
    
    // Time a function with no return value
    template<typename Func>
    void timeVoid(Func&& func)
    {
        QElapsedTimer timer;
        timer.start();
        
        func();
        
        recordMeasurement(timer.elapsed());
    }
    
    // Force output report now
    void reportNow();
    
    // Reset all measurements
    void reset();

private:
    void recordMeasurement(qint64 elapsed);
    void checkReportTime();

    QString m_name;
    QVector<qint64> m_measurements;
    int m_bufferSize;
    int m_currentIndex;
    int m_count;
    int m_reportIntervalMs;
    QElapsedTimer m_reportTimer;
    qint64 m_totalTime;
    qint64 m_maxTime;
    qint64 m_minTime;
};
