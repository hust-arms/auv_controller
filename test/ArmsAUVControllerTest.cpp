/*
 * Filename: ArmsAUVControllerTest.cpp
 * Path: armsauv_docking\src
 * Created Date: Tuesday, September 15th 2020, 3:42:02 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2020 Your Company
 */

#include "auv_controller/ArmsAUVControllerROS.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "armsauv_controllerler_test");

    std::string auv_ns = "armsauv";
    auv_controller::ArmsAUVControllerROS armsauv_controller(auv_ns);

    ros::spin();

    return 0;  
};
