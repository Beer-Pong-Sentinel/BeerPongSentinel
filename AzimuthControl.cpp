#include "AltitudeControl.h"

#include <windows.h>
#include <iostream>
#include <QSerialPort>
#include <qdebug.h>

#define DEGREE_PER_STEP 0.45

void initializeAzimuthMotor(QSerialPort* serialPort, const int currPosition) {
    int steps  = currPosition * -1;
    QString serialMessage = QString::number(steps) + "\n";

    qint64 bytesWritten = serialPort->write(serialMessage.toUtf8());
    if (bytesWritten == -1) {
        qDebug() << "Error: Failed to write data to serial port.";
    } else {
        qDebug() << "Message sent successfully:" << serialMessage.trimmed();
    }

}


// assumes we are at the 0 position and pos. is to the left and neg. is to the right
int moveAzimuthMotor(QSerialPort* serialPort, const float absoluteAngle) {
    int steps = absoluteAngle / DEGREE_PER_STEP;
    QString serialMessage = QString::number(steps) + "\n";

    qint64 bytesWritten = serialPort->write(serialMessage.toUtf8());
    if (bytesWritten == -1) {
        qDebug() << "Error: Failed to write data to serial port.";
    } else {
        qDebug() << "Message sent successfully:" << serialMessage.trimmed();
    }

    return steps;
}