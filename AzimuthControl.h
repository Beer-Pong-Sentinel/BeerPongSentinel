#ifndef AZIMUTHCONTROL_H
#define AZIMUTHCONTROL_H

#include <string>
#include <QSerialPort>


struct AzConfigData{

    double minAzAngle;
    double maxAzAngle;
};

int moveAzimuthMotor(QSerialPort* serialPort, const float absoluteAngle);
void initializeAzimuthMotor(QSerialPort* serialPort, const int currPosition);
AzConfigData parseAzimuthConfig();

#endif // AZIMUTHCONTROL_H
