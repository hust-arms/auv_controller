/*
 * Filename: AUVPIDControllerTest.cpp                                               
 * Path: auv_controller
 * Created Date: Saturday, Faburary 27th 2021, 14:05:02 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <stdio.h>
#include "auv_controller/AUVPIDControllerROS.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auv_pid_controller_test_node");

    bool with_ff, x_type = false, debug = false;

    if(argc >= 2 && strcmp(argv[1], "no_ff") == 0){
        printf("Controller without front fins\n");
        with_ff = false;
    }
    else{
        printf("Controller with front fins\n");
        with_ff = true;
    }

    if(argc >= 3 && strcmp(argv[2], "x") == 0){
        printf("Controller for X type rudder\n");
        x_type = true;
    }
    else{
        printf("Controller for crosshead type rudder\n");
    }

    if(argc == 4 && strcmp(argv[3], "debug") == 0){
        printf("Debug mode\n");
        debug = true;
    }

    // std::string auv_ns = "auv324";
    std::string auv_ns = "armsauv";
    auv_controller::AUVPIDControllerROS auv_ctrl(auv_ns, with_ff, x_type, debug);

    auv_ctrl.startControl();

    ros::spin();

    return 0;
}

