#ifndef AZIMUTHCONTROL_H
#define AZIMUTHCONTROL_H

#include <string>
#include <QSerialPort>

int moveAzimuthMotor(QSerialPort* serialPort, const float absoluteAngle);
void initializeAzimuthMotor(QSerialPort* serialPort, const int currPosition);

#endif // AZIMUTHCONTROL_H
