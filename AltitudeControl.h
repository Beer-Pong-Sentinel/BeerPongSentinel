#ifndef ALTITUDECONTROL_H
#define ALTITUDECONTROL_H

#include "pubSysCls.h"
using namespace sFnd;



INode* initializeAltitudeMotor();
double moveAltitudeMotor(INode* altitudePointer, float absoluteAngle, int rpmLimit);
double enableAltitudeMotor(INode* altitudePointer, bool enable);
void homeAltitudeMotor(INode* altitudePointer);

#endif // ALTITUDECONTROL_H
