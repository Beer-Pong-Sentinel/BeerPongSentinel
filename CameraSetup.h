#ifndef CAMERASETUP_H
#define CAMERASETUP_H

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <fstream>
#include <string>
#include "json.hpp"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

int SetEnumerationNode(INodeMap& nodeMap, const std::string& enumerationNodeName, const std::string& entryNodeName);
int SetIntegerNode(INodeMap& nodeMap, const std::string& integerNodeName, int integerNodeValue);
int SetFloatNode(INodeMap& nodeMap, const std::string& floatNodeName, const double floatNodeValue);
int GetFloatNode(INodeMap& nodeMap, const std::string& floatNodeName);
int ConfigureCameraSettings(INodeMap& nodeMap);
std::string GetStringNode(INodeMap& nodeMap, const std::string& stringNodeName);
int SetBooleanNode(INodeMap& nodeMap, const std::string& booleanNodeName, const bool booleanNodeValue);
int GetIntegerNode(INodeMap& nodeMap, const std::string& integerNodeName);

#endif
