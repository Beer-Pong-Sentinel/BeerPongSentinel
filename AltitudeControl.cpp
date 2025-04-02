#include "AltitudeControl.h"

#include <QDebug>  // Include QDebug for logging
#include <string>
#include <iostream>
#include "pubSysCls.h"

using namespace sFnd;

//*********************************************************************************
//This program will load configuration files onto each node connected to the port, then executes
//sequential repeated moves on each axis.
//*********************************************************************************

#define TIME_TILL_TIMEOUT   10000   //The timeout used for homing(ms)
#define DEGREE_PER_STEP 0.057

INode* initializeAltitudeMotor()
{
    qDebug() << "Started altitude motor initialization";
    size_t portCount = 0;
    std::vector<std::string> comHubPorts;

    //Create the SysManager object. This object will coordinate actions among various ports
    // and within nodes. In this example we use this object to setup and open our port.
    SysManager* myMgr = SysManager::Instance();                           //Create System Manager myMgr

    //This will try to open the port. If there is an error/exception during the port opening,
    //the code will jump to the catch loop where detailed information regarding the error will be displayed;
    //otherwise the catch loop is skipped over
    try
    {

        // SysManager::FindComHubPorts(comHubPorts);
        // qDebug() << "Found" << comHubPorts.size() << "SC Hubs";

        // for (portCount = 0; portCount < comHubPorts.size() && portCount < NET_CONTROLLER_MAX; portCount++) {

        //     myMgr->ComHubPort(portCount, comHubPorts[portCount].c_str());    //define the first SC Hub port (port 0) to be associated
        //         // with COM portnum (as seen in device manager)
        // }


        // if (portCount > 0) {
        //     //qDebug() << "\n I will now open port \t" << portnum << "\n \n";
        //     myMgr->PortsOpen(portCount);             //Open the port

        //     for (size_t i = 0; i < portCount; i++) {
        //         IPort &myPort = myMgr->Ports(i);

        //         qDebug() << " Port[" << i << "]: state=" << myPort.OpenState() << ", nodes=" << myPort.NodeCount();
        //     }
        // }
        // else {
        //     qDebug() << "Unable to locate SC hub port";

        //     return nullptr;  //This terminates the main program
        // }

        myMgr->ComHubPort(0, 17);
        myMgr->PortsOpen(1);
        //Once the code gets past this point, it can be assumed that the Port has been opened without issue
        //Now we can get a reference to our port object which we will use to access the node

        // Get a reference to the port, to make accessing it easier
        IPort &myPort = myMgr->Ports(0);



        //Here we identify the first Node, enable and home the node, then adjust the position reference

        // Create a shortcut reference for the first node
        int iNode = 0;
        INode &theNode = myPort.Nodes(iNode);

        //theNode.EnableReq(false);             //Ensure Node is disabled before starting

        qDebug() << "   Node[" << iNode << "]: type=" << theNode.Info.NodeType();
        qDebug() << "            userID:" << theNode.Info.UserID.Value();
        qDebug() << "        FW version:" << theNode.Info.FirmwareVersion.Value();
        qDebug() << "          Serial #:" << theNode.Info.SerialNumber.Value();
        qDebug() << "             Model:" << theNode.Info.Model.Value();

        //The following statements will attempt to enable the node.  First,
        // any shutdowns or NodeStops are cleared, finally the node in enabled
        theNode.Status.AlertsClear();                   //Clear Alerts on node
        theNode.Motion.NodeStopClear(); //Clear Nodestops on Node
        theNode.EnableReq(true);                    //Enable node

        double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT; //define a timeout in case the node is unable to enable
            //This will loop checking on the Real time values of the node's Ready status
        while (!theNode.Motion.IsReady()) {
            if (myMgr->TimeStampMsec() > timeout) {
                return nullptr;
            }
        }
        //At this point the Node is enabled, and we will now check to see if the Node has been homed
        //Check the Node to see if it has already been homed,
        if (theNode.Motion.Homing.HomingValid())
        {
            if (theNode.Motion.Homing.WasHomed())
            {
                qDebug() << "Node" << iNode << "has already been homed, current position is:" << theNode.Motion.PosnMeasured.Value();
                qDebug() << "Rehoming Node...";
            }
            else
            {
                qDebug() << "Node [" << iNode << "] has not been homed.  Homing Node now...";
            }
            //Now we will home the Node
            theNode.Motion.Homing.Initiate();

            timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;    //define a timeout in case the node is unable to enable
            // Basic mode - Poll until disabled
            while (!theNode.Motion.Homing.WasHomed()) {
                if (myMgr->TimeStampMsec() > timeout) {
                    qDebug() << "Node did not complete homing:  \n\t -Ensure Homing settings have been defined through ClearView. \n\t -Check for alerts/Shutdowns \n\t -Ensure timeout is longer than the longest possible homing move.";
                    return nullptr;
                }
            }
            theNode.Motion.PosnMeasured.Refresh();      //Refresh our current measured position
            qDebug() << "Node completed homing, current position:" << theNode.Motion.PosnMeasured.Value();
            qDebug() << "Soft limits now active";
            qDebug() << "Successfully initialized altitude motor";
            return &theNode;

            // qDebug() << "Adjusting Numberspace by" << CHANGE_NUMBER_SPACE;

            // theNode.Motion.AddToPosition(CHANGE_NUMBER_SPACE);          //Now the node is no longer considered "homed, and soft limits are turned off

            // theNode.Motion.Homing.SignalComplete();     //reset the Node's "sense of home" soft limits (unchanged) are now active again

            // theNode.Motion.PosnMeasured.Refresh();      //Refresh our current measured position
            // qDebug() << "Numberspace changed, current position:" << theNode.Motion.PosnMeasured.Value();
        }
        else {
            qDebug() << "Node[" << iNode << "] has not had homing setup through ClearView.  The node will not be homed.";
        }

        theNode.EnableReq(false);
    }


    catch (mnErr& theErr)
    {
        //This statement will print the address of the error, the error code (defined by the mnErr class),
        //as well as the corresponding error message.
        qDebug() << "Caught error: addr=" << theErr.TheAddr << ", err=" << Qt::hex << theErr.ErrorCode << "\nmsg=" << theErr.ErrorMsg;

        return nullptr;
    }


    // Close down the ports
    // myMgr->PortsClose();

    return nullptr;
}

double moveAltitudeMotor(INode* altitudePointer, float absoluteAngle, int rpmLimit=0) {
    if (altitudePointer == nullptr) {
        qDebug() << "Altitude motor not initialized";
        return 0;
    }
    SysManager* myMgr = SysManager::Instance();                           //Create System Manager myMgr

    INode &theNode = *altitudePointer;
    theNode.Motion.MoveWentDone();  // Clear the move done flag
    theNode.AccUnit(INode::RPM_PER_SEC);                //Set the units for Acceleration to RPM/SEC
    theNode.VelUnit(INode::RPM);                        //Set the units for Velocity to RPM
    theNode.Motion.AccLimit = 1000;      //Set Acceleration Limit (RPM/Sec)
    if (rpmLimit > 0) {
        theNode.Motion.VelLimit = rpmLimit;              //Set Velocity Limit (RPM)
    } else {
        theNode.Motion.VelLimit = 1000;              //Set Velocity Limit (RPM)

    }

    int steps = absoluteAngle / DEGREE_PER_STEP;

    theNode.Motion.MovePosnStart(steps, true);
    qDebug() << theNode.Motion.MovePosnDurationMsec(steps, true) << "estimated time.";
    double timeout = myMgr->TimeStampMsec() + theNode.Motion.MovePosnDurationMsec(steps, true) + 100;         //define a timeout in case the node is unable to enable
    double startTime = myMgr->TimeStampMsec();
    while (!theNode.Motion.MoveIsDone()) {

        // if (myMgr->TimeStampMsec() > timeout) {
        //     qDebug() << "Error: Timed out waiting for move to complete";
        //     return 0;
        // }
    }
    double endTime = myMgr->TimeStampMsec();
    qDebug() << "Move completed";
    return endTime - startTime;
}


double enableAltitudeMotor(INode* altitudePointer, bool enable){

    if (altitudePointer == nullptr) {
        qDebug() << "Altitude motor not initialized";
        return 0;
    }                      //Create System Manager myMgr

    INode &theNode = *altitudePointer;

    theNode.EnableReq(enable);

    return 1;
}


void homeAltitudeMotor(INode* altitudePointer)
{
    if (altitudePointer == nullptr) {
        qDebug() << "Altitude motor not initialized.";
        return;
    }

    // Get the system manager and a reference to the node.
    SysManager* myMgr = SysManager::Instance();
    INode &theNode = *altitudePointer;

    // Check if homing has been set up for this node.
    if (!theNode.Motion.Homing.HomingValid()) {
        qDebug() << "Homing settings not configured for this node.";
        return;
    }

    // Log the current homing status and indicate that homing is beginning.
    if (theNode.Motion.Homing.WasHomed()) {
        qDebug() << "Node already homed, current position:" << theNode.Motion.PosnMeasured.Value();
        qDebug() << "Rehoming Node...";
    } else {
        qDebug() << "Node not yet homed. Initiating homing procedure...";
    }

    // Initiate the homing procedure.
    theNode.Motion.Homing.Initiate();

    // Set a timeout to avoid an infinite loop.
    double timeout = myMgr->TimeStampMsec() + TIME_TILL_TIMEOUT;
    while (!theNode.Motion.Homing.WasHomed()) {
        if (myMgr->TimeStampMsec() > timeout) {
            qDebug() << "Error: Timed out waiting for homing to complete.";
            return;
        }
    }

    // Refresh and report the current position after successful homing.
    theNode.Motion.PosnMeasured.Refresh();
    qDebug() << "Homing complete. Current position:" << theNode.Motion.PosnMeasured.Value();
}

