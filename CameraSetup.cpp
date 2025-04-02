#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <fstream>
#include "CameraSetup.h"
#include <string>
#include "json.hpp"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;
using json = nlohmann::json;


/*
Functions we want to have:

set the parameters (for now just set them through the function input, later read from config file)
stream mode, frame rate, exposure time, gain, white balance, pixel format, buffer size, etc.

get the current parameters as a json string

print the current parameters



*/

int SetEnumerationNode(INodeMap& nodeMap, const std::string& enumerationNodeName, const std::string& entryNodeName) {

    CEnumerationPtr ptrEnumerationNode = nodeMap.GetNode(enumerationNodeName.c_str());
    if (IsReadable(ptrEnumerationNode) && IsWritable(ptrEnumerationNode))
    {
        // Retrieve the desired entry node from the enumeration node
        CEnumEntryPtr ptrEntryNode = ptrEnumerationNode->GetEntryByName(entryNodeName.c_str());
        if (IsReadable(ptrEntryNode)){
            // Retrieve the integer value from the entry node
            int64_t entryNodeValue = ptrEntryNode->GetValue();
            // Set integer as new value for enumeration node
            ptrEnumerationNode->SetIntValue(entryNodeValue);
            cout << enumerationNodeName <<" set to " << ptrEnumerationNode->GetCurrentEntry()->GetSymbolic() << "..." << endl;
        }
        else
        {
            cout << entryNodeName << " not readable..." << endl;
            return -1;
        }
    }
    else
    {
        cout << enumerationNodeName << " not readable or writable..." << endl;
        return -1;
    }

    nodeMap.InvalidateNodes();


    return 0;
}

int GetEnumerationNode(INodeMap& nodeMap, const std::string& enumerationNodeName) {

    CEnumerationPtr ptrEnumerationNode = nodeMap.GetNode(enumerationNodeName.c_str());
    if (IsReadable(ptrEnumerationNode))
    {
        // Retrieve the desired entry node from the enumeration node

        CEnumEntryPtr ptrEntryNode = ptrEnumerationNode->GetCurrentEntry();

        if (IsReadable(ptrEntryNode)) {

            // Retrieve the integer value from the entry node
            int64_t entryNodeValue = ptrEntryNode->GetValue();
            // Set integer as new value for enumeration node
            ptrEnumerationNode->GetIntValue(entryNodeValue);
            cout << enumerationNodeName << " is " << ptrEnumerationNode->GetCurrentEntry()->GetSymbolic() << "..." << endl;
        }
        else
        {
            cout << "Entry value of "<< enumerationNodeName << " not readable..." << endl;
            return -1;
        }
    }
    else
    {
        cout << enumerationNodeName << " not readable or writable..." << endl;
        return -1;
    }

    nodeMap.InvalidateNodes();


    return 0;
}

int SetIntegerNode(INodeMap& nodeMap, const std::string& integerNodeName, int integerNodeValue) {

    CIntegerPtr ptrIntegerNode = nodeMap.GetNode(integerNodeName.c_str());
    if (IsReadable(ptrIntegerNode) && IsWritable(ptrIntegerNode))
    { 
        ptrIntegerNode->SetValue(integerNodeValue);
        cout << integerNodeName <<" set to " << integerNodeValue << "..." << endl; }
    else { cout << integerNodeName << " not readable or writable..." << endl; }

    return 0;
}

int SetFloatNode(INodeMap& nodeMap, const std::string& floatNodeName, const double floatNodeValue) {

    CFloatPtr ptrFloatNode = nodeMap.GetNode(floatNodeName.c_str());
    if (IsReadable(ptrFloatNode) && IsWritable(ptrFloatNode))
    {
        ptrFloatNode->SetValue(floatNodeValue);
        cout << floatNodeName << " set to " << floatNodeValue << "..." << endl;
    }
    else { cout << floatNodeName << " not readable or writable..." << endl; }

    return 0;
}

int GetFloatNode(INodeMap& nodeMap, const std::string& floatNodeName)
{
    CFloatPtr ptrFloatNode = nodeMap.GetNode(floatNodeName.c_str());
    if (IsReadable(ptrFloatNode))
    {
        double floatNodeValue = ptrFloatNode->GetValue();
        cout << floatNodeName << " = " << floatNodeValue << endl;
    }
    else { cout << floatNodeName << " not readable..." << endl; }

    return 0;
}

int GetIntegerNode(INodeMap& nodeMap, const std::string& integerNodeName)
{
    CIntegerPtr ptrIntegerNode = nodeMap.GetNode(integerNodeName.c_str());
    if (IsReadable(ptrIntegerNode))
    {
        double floatNodeValue = ptrIntegerNode->GetValue();
        cout << integerNodeName << " = " << floatNodeValue << endl;
    }
    else { cout << integerNodeName << " not readable..." << endl; }

    return 0;
}

std::string GetStringNode(INodeMap& nodeMap, const std::string& stringNodeName)
{
    CStringPtr ptrStringNode = nodeMap.GetNode(stringNodeName.c_str());

    std::string stringNodeValue = "";

    if (IsReadable(ptrStringNode))
    {
        std::string stringNodeValue = (ptrStringNode->GetValue()).c_str();
        cout << stringNodeName << " = " << stringNodeValue << endl;
        return stringNodeValue;
    }
    else 
    { 
        cout << stringNodeName << " not readable..." << endl; 
    }

    return stringNodeValue;
}

int SetBooleanNode(INodeMap& nodeMap, const std::string& booleanNodeName, const bool booleanNodeValue)
{
    CBooleanPtr ptrBooleanNode = nodeMap.GetNode(booleanNodeName.c_str());
    if (IsReadable(ptrBooleanNode) && IsWritable(ptrBooleanNode))
    {
        ptrBooleanNode->SetValue(booleanNodeValue);
        cout << booleanNodeName << " set to " << booleanNodeValue << "..." << endl;
    }
    else { cout << booleanNodeName << " not readable or writable..." << endl; }

    return 0;
}

struct ConfigData {
    std::string AcquisitionMode;
    std::string ExposureAuto;
    double ExposureTime;
    std::string GainAuto;
    double Gain;
    std::string PixelFormat;
    std::string PrimarySerialNumber;
    std::string SecondarySerialNumber;
    std::string LineSelector;
    std::string LineMode;
    bool V3_3Enable;
    std::string TriggerSource;
    std::string TriggerOverlap;
    std::string TriggerMode;
    bool ChunkModeActive;
    std::string ChunkSelector;
    bool ChunkEnable;
};

ConfigData ParseConfig() {
    cout << "in parse camera config" << endl;
    // Open and parse the JSON file
    std::ifstream f("../../config.json");
    if (!f.is_open()) {
        std::cout << "Error: Could not open config.json file" << std::endl;
    }
    json config = json::parse(f);
    std::cout << "parsed camera config successfully"<< std::endl;
    

    // Populate the ConfigData struct with values from the JSON file
    ConfigData data;

    //Acquisition Mode/Image format for both cameras
    data.AcquisitionMode = config["acquisition"]["AcquisitionMode"];
    data.ExposureAuto = config["acquisition"]["ExposureAuto"];
    data.ExposureTime = config["acquisition"]["ExposureTime"];
    data.GainAuto = config["acquisition"]["GainAuto"];
    data.Gain = config["acquisition"]["Gain"];
    data.PixelFormat = config["image_format"]["PixelFormat"];

    //Primary camera settings
    data.PrimarySerialNumber = config["primary_camera"]["DeviceID"];
    data.LineSelector = config["primary_camera"]["LineSelector"];
    data.LineMode = config["primary_camera"]["LineMode"];
    data.V3_3Enable = config["primary_camera"]["V3_3Enable"];

    //Secondary camera settings
    data.SecondarySerialNumber = config["secondary_camera"]["DeviceID"];
    data.TriggerSource = config["secondary_camera"]["TriggerSource"];
    data.TriggerOverlap = config["secondary_camera"]["TriggerOverlap"];
    data.TriggerMode = config["secondary_camera"]["TriggerMode"];
    

    // Chunk Data
    data.ChunkModeActive = config["chunk_data"]["ChunkModeActive"];
    data.ChunkSelector = config["chunk_data"]["ChunkSelector"];
    data.ChunkEnable = config["chunk_data"]["ChunkEnable"];

    return data;
}

int ConfigureCameraSettings(INodeMap& nodeMap)
{
    int result = 0;

    cout << endl << "*** CONFIGURE CAMERA SETTINGS ***" << endl << endl;

    // Parse config file
    ConfigData config = ParseConfig();

    // Set acquisition mode to continuous
    result = result | SetEnumerationNode(nodeMap, "AcquisitionMode", config.AcquisitionMode);

    // Set pixel format to mono 8
    result = result | SetEnumerationNode(nodeMap, "PixelFormat", config.PixelFormat);

    // Set exposure auto to off
    result = result | SetEnumerationNode(nodeMap, "ExposureAuto", config.ExposureAuto);

    // Set exposure time to 16667.0us (this is ~60fps)
    result = result | SetFloatNode(nodeMap, "ExposureTime", config.ExposureTime);

    // Set gain auto to off
    result = result | SetEnumerationNode(nodeMap, "GainAuto", config.GainAuto);

    // Set gain to 5dB
    result = result | SetFloatNode(nodeMap, "Gain", config.Gain);

    // Set chunk data

    result = result | SetBooleanNode(nodeMap, "ChunkModeActive", config.ChunkModeActive);
    result = result | SetEnumerationNode(nodeMap, "ChunkSelector", config.ChunkSelector);
    result = result | GetEnumerationNode(nodeMap, "ChunkSelector");
    result = result | SetBooleanNode(nodeMap, "ChunkEnable", config.ChunkEnable);
    result = result | GetIntegerNode(nodeMap, "TimestampIncrement");

    
    std::string serialNumber = GetStringNode(nodeMap, "DeviceID");

    if(serialNumber==config.PrimarySerialNumber){

        result = result | SetEnumerationNode(nodeMap, "LineSelector", config.LineSelector);
        result = result | SetEnumerationNode(nodeMap, "LineMode", config.LineMode);
        result = result | SetBooleanNode(nodeMap, "V3_3Enable", config.V3_3Enable);

    }
    else if (serialNumber==config.SecondarySerialNumber)
    {
        result = result | SetEnumerationNode(nodeMap, "TriggerSource", config.TriggerSource);

        result = result | SetEnumerationNode(nodeMap, "TriggerOverlap", config.TriggerOverlap);

        result = result | SetEnumerationNode(nodeMap, "TriggerMode", config.TriggerMode);
    }
    else{
        std::cout<<"Tried setting up camera with unknown serial number!"<<std::endl;
    }
    


 

    return result;

}
