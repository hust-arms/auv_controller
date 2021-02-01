/*
 * Filename: AUVControllerTest.cpp                                               
 * Path: armsauv_docking\src
 * Created Date: Sunday, Janurary 31th 2021, 14:05:02 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <stdio.h>
#include "auv_controller/AUVControllerROS.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auv_controller_test_node");

    bool with_ff, debug = false;

    if(argc >= 2 && strcmp(argv[1], "no_ff") == 0){
        printf("Controller without front fins\n");
        with_ff = false;
    }
    else{
        printf("Controller with front fins\n");
        with_ff = true;
    }

    if(argc == 3 && strcmp(argv[2], "debug") == 0){
        printf("Debug mode\n");
        debug = true;
    }

    std::string auv_ns = "armsauv";
    auv_controller::AUVControllerROS auv_ctrl(auv_ns, with_ff, debug);

    auv_ctrl.startControl();

    ros::spin();

    return 0;
}

