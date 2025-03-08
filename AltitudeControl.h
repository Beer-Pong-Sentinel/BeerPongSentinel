#ifndef ALTITUDECONTROL_H
#define ALTITUDECONTROL_H

#include "pubSysCls.h"
using namespace sFnd;



INode* initializeAltitudeMotor();
void moveAltitudeMotor(INode* altitudePointer, float absoluteAngle);

#endif // ALTITUDECONTROL_H
