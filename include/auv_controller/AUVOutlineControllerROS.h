/*                                                                          
 * Filename: auv_controller/AUVOutlineControllerROS.h
 * Path: auv_controller
 * Created Date: Friday, April 17th 2021, 11:06:05 am
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#pragma once
#include "AUVOutlineController.h"

namespace auv_controller
{

/**
 * @brief ROS wrapper of AUVOutlineController
 */
class AUVOutlineControllerROS
{
    typedef boost::shared_ptr<AUVOutlineController> ol_controller_ptr;
public:
    AUVOutlineControllerROS(const std::string& db_filename, bool with_ff, bool x_type);
    
    ~AUVOutlineControllerROS();

private:
    ol_controller_ptr controller_;

}; // AUVOutlineControllerROS

}; // ns
