/*                                                                                             
 * Filename: AUVCtrlMsgsRecorderNode.cpp                                               
 * Path: auv_controller\nodes
 * Created Date: Monday, March 22th 2021, 14:05:02 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <stdio.h>
#include <string.h>
#include "auv_controller/AUVCtrlMsgsRecorderROS.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auv_ctrlmsgs_recorder_node");

    std::string auv_name = "armsauv";
    bool with_ff = true, x_type = false, debug = false;
    int mission = 0;

    if(argc >= 2)
    {
        auv_name = std::string(argv[1]);
        printf("<auv_ctrlmsgs_recorder_node>: model %s\n", auv_name.c_str());
    }

    if(argc >= 3 && strcmp(argv[2], "no_ff") == 0)
    {
        printf("<auv_ctrlmsgs_recorder_node>: without front fins\n");
        with_ff = false;
    }
    else
    {
        printf("<auv_ctrlmsgs_recorder_node>: with front fins\n");
        with_ff = true;
    }

    if(argc >= 4 && strcmp(argv[3], "x") == 0)
    {
        printf("<auv_ctrlmsgs_recorder_node>: X type rudder\n");
        x_type = true;
    }
    else
    {
        printf("<auv_ctrlmsgs_recorder_node>: common crosshead type\n");
    }

    if(argc >= 5)
    {
        int mission_f = atoi(argv[4]);
        switch(mission_f)
        {
        case 0:
            printf("<auv_ctrlmsgs_recorder_node>: Fixed depth\n");
            mission = mission_f;
            break;
        case 1:
            printf("<auv_ctrlmsgs_recorder_node>: Fixed lateral distance\n");
            mission = mission_f;
            break;
        case 2:
            printf("<auv_ctrlmsgs_recorder_node>: Fixed depth & lateral distance\n");
            mission = mission_f;
            break;
        case 3:
            printf("<auv_ctrlmsgs_recorder_node>: Fixed yaw\n");
            mission = mission_f;
            break;
        case 4:
            printf("<auv_ctrlmsgs_recorder_node>: Multi point\n");
            mission = mission_f;
            break;
        case 5:
            printf("<auv_ctrlmsgs_recorder_node>: Docking\n");
            mission = mission_f;
            break;
        case 6:
            printf("<auv_ctrlmsgs_recorder_node>: Outline Control\n");
            mission = mission_f;
            break;
        default:
            printf("<auv_ctrlmsgs_recorder_node>: Fixed depth\n");
        }
    }

    if(argc >= 6 && strcmp(argv[5], "debug") == 0)
    {
        printf("<auv_ctrlmsgs_recorder_node>: debug mode\n");
        debug = true;
    }
    else
    {
        printf("<auv_ctrlmsgs_recorder_node>: no debug mode\n");
    }

    auv_controller::AUVCtrlMsgsRecorderROS auv_ctrlmsgs_recorder(auv_name, with_ff, x_type, debug, mission);

    auv_ctrlmsgs_recorder.startRecord();

    ros::spin();

    return 0;
}

