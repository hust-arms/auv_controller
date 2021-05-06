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

    is_record_ot_params_ = false;

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
        mission = "mission: Fixed depth & lateral distance";
        if(!fw_ptr_->writeData(mission))
        {
            return false;
        }
    case 3:
        mission = "mission: Fixed Yaw";
        if(!fw_ptr_->writeData(mission))
        {
            return false;
        }
        break;
    case 4:
        mission = "mission: Multi point";
        if(!fw_ptr_->writeData(mission))
        {
            return false;
        }
        break;
    case 5:
        mission = "mission: Docking";
        if(!fw_ptr_->writeData(mission))
        {
            return false;
        }
        break;
    case 6: 
        mission = "mission: Outline Control";
        if(!fw_ptr_->writeData(mission))
        {
            is_record_ot_params_ = true;
            return false;
        }
        break;
    }

    // parameter header
    std::string header;
    if(mission_type != 6)
    {
        header = "time,x,y,z,depth,u,v,w,roll,pitch,yaw,p,q,r,rpm,l_bow,r_bow,l_stern,r_stern,up_stern,lo_stern,x_d,y_d,depth_d,pitch_d,yaw_d,u_d,depth_dev,latdist_dev,yaw_dev,pitch_dev";
    }
    else
    {
        header = "time,x,y,z,depth,u,v,w,roll,pitch,yaw,p,q,r,rpm,l_bow,r_bow,l_stern,r_stern,up_stern,lo_stern,ts,ol_x,l_y,ol_z,ol_roll,ol_pitch,ol_yaw,ol_u,ol_v,ol_p,ol_q,ol_r";
    }

    if(!fw_ptr_->writeData(header))
    {
        return false;
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
    std::string space = ",";

    // Record data
    for(int i = 0; i < data_arr.size(); ++i)
    {
        ss << data_arr[i] << space;
    }

    return ss.str();
}

}; // ns
