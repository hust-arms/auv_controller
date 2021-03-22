/*                                                                                             
 * Filename: AUVTrajFollowMissionNode.cpp                                               
 * Path: auv_controller\nodes
 * Created Date: Monday, March 22th 2021, 14:05:02 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <stdio.h>
#include "auv_controller/AUVTrajFollowManagerROS.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auv_traj_follow_manger_node");

    bool with_ff = false, x_type = false, debug = false;
    std::string auv_ns = "";

    if(argc < 2)
    {
        printf("[AUVTrajFollowMissionNode]: Use default namespace\n");
    }
    else
    {
        auv_ns = std::string(argv[1]);
        printf("[AUVTrajFollowMissionNode]: Used namespace: %s\n", auv_ns.c_str());
    }

    // if(argc >= 3 && strcmp(argv[2], "no_ff") == 0)
    if(argc >= 3 && atoll(argv[2]) > 0)
    {
        printf("[AUVTrajFollowMissionNode]: Model with front fins\n");
        with_ff = true;
    }

    // if(argc >= 4 && strcmp(argv[3], "x") == 0)
    if(argc >= 4 && atoll(argv[3]) > 0)
    {
        printf("[AUVTrajFollowMissionNode]: Model with X type rudder\n");
        x_type = true;
    }

    if(argc == 5 && atoll(argv[4]) > 0)
    {
        printf("[AUVTrajFollowMissionNode]: Debug mode\n");
        debug = true;
    }

    auv_controller::AUVTrajFollowManagerROS auv_traj_follow_manager(auv_ns, with_ff, x_type, debug);

    auv_traj_follow_manager.start();

    ros::spin();

    return 0;
}

