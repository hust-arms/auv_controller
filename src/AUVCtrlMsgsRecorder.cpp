/*                                                                                               
 * Filename: auv_controller/AUVCtrlMsgsRecorder.cpp
 * Path: auv_controller
 * Created Date: Thirsday, March 24th 2021, 10:58:05 am
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include "auv_controller/AUVCtrlMsgsRecorder.h"

namespace auv_controller
{
///////////////////////////
bool AUVCtrlMsgsRecorder::writeMsgsHeader(const unsigned int model_type, const unsigned int mission_type)
{
    // Get & write record time stamp
    boost::posix_time::ptime time_stamp = boost::posix_time::second_clock::universal_time();
    std::string date = boost::posix_time::to_simple_string(time_stamp);
    fw_ptr_->writeData("Record time: " + date);

    // Write model type & mission type
    std::string model;
    switch(model_type)
    {
    case 0:
        model = "model_type: front fin";
        if(!fw_ptr_->writeData(model))
        {
            return false;
        }
        break;
    case 1:
        model = "model_type: crosshead type";
        if(!fw_ptr_->writeData(model))
        {
            return false;
        }
        break;
    case 2:
        model = "model_type: x type";
        if(!fw_ptr_->writeData(model))
        {
            return false;
        }
        break;
    }

    // Write mission type
    std::string mission;
    switch(mission_type)
    {
    case 0:
        mission = "mission: Fixed Depth";
        if(!fw_ptr_->writeData(mission))
        {
            return false;
        }
        break;
    case 1:
        mission = "mission: Fixed Lateral Distance";
        if(!fw_ptr_->writeData(mission))
        {
            return false;
        }
        break;
    case 2:
        mission = "mission: Fixed Yaw";
        if(!fw_ptr_->writeData(mission))
        {
            return false;
        }
        break;
    case 3:
        mission = "mission: Multi point";
        if(!fw_ptr_->writeData(mission))
        {
            return false;
        }
        break;
    case 4:
        mission = "mission: Docking";
        if(!fw_ptr_->writeData(mission))
        {
            return false;
        }
        break;
    }

    return true;
}

///////////////////////////
bool AUVCtrlMsgsRecorder::ctrlMsgsRecord(const std::vector<double>& data_arr)
{
    if(data_arr.size() != ctrl_msgs_num_)
    {
        return false;
    }

    std::string ctrl_msgs = serializeData(data_arr);

    return fw_ptr_->writeData(ctrl_msgs); 
}

///////////////////////////
std::string AUVCtrlMsgsRecorder::serializeData(const std::vector<double>& data_arr)
{
    std::stringstream ss;
    std::string space = " ";

    // Record data
    for(int i = 0; i < data_arr.size(); ++i)
    {
        ss << data_arr[i] << space;
    }

    return ss.str();
}

}; // ns
