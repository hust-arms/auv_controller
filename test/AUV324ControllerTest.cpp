/*
 * Filename: AUVControllerTest.cpp                                               
 * Path: armsauv_docking\src
 * Created Date: Sunday, Janurary 31th 2021, 14:05:02 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <stdio.h>
#include "auv_controller/AUV324ControllerROS.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auv_controller_test_node");

    bool debug = false;

    if(argc == 2 && strcmp(argv[1], "debug") == 0){
        printf("Debug mode\n");
        debug = true;
    }

    auv_controller::AUV324ControllerROS auv_ctrl(debug);

    auv_ctrl.startControl();

    ros::spin();

    return 0;
}

