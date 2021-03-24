/*                                                                                             
 * Filename: AUVCtrlMsgsRecorderNode.cpp                                               
 * Path: auv_controller\nodes
 * Created Date: Monday, March 22th 2021, 14:05:02 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <stdio.h>
#include "auv_controller/AUVCtrlMsgsRecorderROS.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auv_ctrlmsgs_recorder_node");

    auv_controller::AUVCtrlMsgsRecorderROS auv_ctrlmsgs_recorder;

    auv_ctrlmsgs_recorder.startRecord();

    ros::spin();

    return 0;
}

