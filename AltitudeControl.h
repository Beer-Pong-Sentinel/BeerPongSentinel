#ifndef ALTITUDECONTROL_H
#define ALTITUDECONTROL_H

#include "pubSysCls.h"
using namespace sFnd;

struct AlConfigData {

    double minAlAngle;
    double maxAlAngle;
    double minAlRPMLimit;
    double maxAlRPMLimit;

};



INode* initializeAltitudeMotor(int portNum);
double moveAltitudeMotor(INode* altitudePointer, float absoluteAngle, int rpmLimit);
double enableAltitudeMotor(INode* altitudePointer, bool enable);
void homeAltitudeMotor(INode* altitudePointer);
AlConfigData parseAltitudeConfig();



#endif // ALTITUDECONTROL_H
