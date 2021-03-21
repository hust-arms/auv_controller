/*                                                                                             
 * Filename: auv_traj_follow_manager/AUVTrajFollowManagerROS.h
 * Path: auv_controller
 * Created Date: Thirsday, March 17th 2021, 10:58:05 pm
 * Author: zhao wang
 *
 * Copyright (c) 2021 hust-arms
 */

#include <cmath>
#include "auv_controller/AUVTrajFollowManagerROS.h"

namespace auv_controller
{
//////////////////////////////////
AUVTrajFollowManagerROS::AUVTrajFollowManagerROS(std::string auv_name, bool with_ff, bool x_type, bool debug) : 
    with_ff_(with_ff), x_type_(x_type), debug_(debug)
{
    ros::NodeHandle private_nh("~"); // private ros node handle
    ros::NodeHandle nh; // public ros node handle

    // private_nh.searchParam("target_waypoint", wp_xmlrpc_);
    // private_nh.param("desired_x_linear_velocity", u_d_, 3.0);
    // private_nh.param("traj_follow_manage_period", manage_dt_, 0.1);
    // private_nh.searchParam("target_waypoint", wp_xmlrpc_);
    private_nh.getParam("desired_x_linear_velocity", u_d_);
    private_nh.getParam("traj_follow_manage_period", manage_dt_);

    // transform string to geometry_msgs::Point vector
    if(!strToGeoPointVector(wp_xmlrpc_, wp_vec_))
    {
        ROS_ERROR("Parse way point list false!");
    }

    if(debug_)
    {
        boost::unique_lock<boost::recursive_mutex> print_lock(print_mutex_);
        printf("Initialize manager components!");
        // print_lock.unlock();
    }

    auv_control_info_pub_ = nh.advertise<auv_control_msgs::AUVCtrlInfo>(auv_name+"/control_info", 1); 
    ctrl_state_reset_cl_ = nh.serviceClient<auv_controller::ResetCtrlState>(auv_name+"/reset_ctrl_state"); 
    pose_sub_ = nh.subscribe<nav_msgs::Odometry>(auv_name+"/pose_gt", 1, boost::bind(&AUVTrajFollowManagerROS::posegtCb, this, _1));

    thruster0_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name+"/thrusters/0/input", 1);
    fin0_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name+"/fins/0/input", 1);
    fin1_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name+"/fins/1/input", 1);
    fin2_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name+"/fins/2/input", 1);
    fin3_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name+"/fins/3/input", 1);
    fin4_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name+"/fins/4/input", 1);
    fin5_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name+"/fins/5/input", 1);

    base_frame_ = auv_name + "base_link";
    wp_index_ = -1;
    is_start_mission_ = false;

    ROS_INFO("desired x linear velocity: %f", u_d_);
    ROS_INFO("manage period: %f", manage_dt_);

    controller_ptr_ = boost::make_shared<auv_controller::AUVControllerExp>(auv_name, with_ff_, x_type_, debug_);
}

//////////////////////////////////
AUVTrajFollowManagerROS::~AUVTrajFollowManagerROS()
{
    if(manage_th_ != nullptr)
    {
        manage_th_->interrupt();
        manage_th_->join();
        delete manage_th_;
        manage_th_ = nullptr;
    }
    if(pub_th_ != nullptr)
    {
        pub_th_->interrupt();
        pub_th_->join();
        delete pub_th_;
        pub_th_ = nullptr;
    }
}

//////////////////////////////////
void AUVTrajFollowManagerROS::start()
{
    if(debug_)
    {
        boost::unique_lock<boost::recursive_mutex> print_lock(print_mutex_);
        printf("Traj follow manage thread has been created!");
        // print_lock.unlock();
    }

    controller_ptr_->startControl();

    pub_th_ = new boost::thread(boost::bind(&AUVTrajFollowManagerROS::publishThread, this));
    manage_th_ = new boost::thread(boost::bind(&AUVTrajFollowManagerROS::trajFollowManageThread, this));

    int ctrl_status_flag = 2;
    controller_ptr_->setCtrlStatus(ctrl_status_flag);
}

//////////////////////////////////
void AUVTrajFollowManagerROS::wakeManageThread(const ros::TimerEvent& event)
{
    manage_cond_.notify_one();
}

//////////////////////////////////
void AUVTrajFollowManagerROS::publishThread()
{
    ros::NodeHandle nh;

    boost::unique_lock<boost::recursive_mutex> manage_lock(manage_mutex_);
    manage_cond_.notify_one(); // wake manage thread
    manage_mutex_.unlock();

    while(nh.ok())
    {
        // Set auv states
        auv_controller::ResetCtrlState srv;

        if(getMissionFlag())
        {
            if(x_type_)
            {
                double upper_p, upper_s, lower_p, lower_s, thruster;
                controller_ptr_->getCtrlVar(upper_p, upper_s, lower_p, lower_s, thruster);
                publishActuatorMsgs(upper_p, upper_s, lower_p, lower_s, thruster);
            }
            else
            {
                double fwdfin, backfin, vertfin, thruster;
                controller_ptr_->getCtrlVar(fwdfin, backfin, vertfin, thruster);
                publishActuatorMsgs(fwdfin, backfin, vertfin, thruster);
            }

            boost::this_thread::sleep(boost::posix_time::milliseconds(manage_dt_*1000));
        }
        else
        {
            int ctrl_status_flag = 0;
            // Set control status as standby
            controller_ptr_->setCtrlStatus(ctrl_status_flag);

            boost::this_thread::sleep(boost::posix_time::milliseconds(manage_dt_*1000));
        }
    }
}

//////////////////////////////////
void AUVTrajFollowManagerROS::publishActuatorMsgs(double fwdfin, double backfin, double vertfin, double thruster)
{
     std_msgs::Header header;
     header.stamp.setNow(ros::Time::now());
     header.frame_id = base_frame_;
     header.seq = ++seq_;
     
     // Publish thruster message
     uuv_gazebo_ros_plugins_msgs::FloatStamped thrusters_msg;
     thrusters_msg.header = header;
     thrusters_msg.data = thruster;
     thruster0_pub_.publish(thrusters_msg);
     
     // Publish fins message
     uuv_gazebo_ros_plugins_msgs::FloatStamped fins_msg;
     fins_msg.header = header;
     if(with_ff_){
         // Vertical fins
         fins_msg.data = vertfin;
     
         fin3_pub_.publish(fins_msg);
         fins_msg.data = -vertfin;
         fin5_pub_.publish(fins_msg);
         // Forward fins
         fins_msg.data = fwdfin;
     
         fin0_pub_.publish(fins_msg);
         fins_msg.data = -fwdfin;
         fin1_pub_.publish(fins_msg);
         // Backward fins
         fins_msg.data = -backfin;
         fin2_pub_.publish(fins_msg);
         fins_msg.data = backfin;
         fin4_pub_.publish(fins_msg);
     }
     else
     {
         // Vertical fins
         fins_msg.data = vertfin;
         
         fin1_pub_.publish(fins_msg);
         fins_msg.data = -vertfin;
         fin3_pub_.publish(fins_msg);
         // Backward fins
         fins_msg.data = -backfin;
         fin0_pub_.publish(fins_msg);
         fins_msg.data = backfin;
         fin2_pub_.publish(fins_msg);
         
         fins_msg.data = 0.0;
     }
}

//////////////////////////////////
void AUVTrajFollowManagerROS::publishActuatorMsgs(double upper_p, double upper_s, double lower_p, double lower_s, double thruster)
{
    std_msgs::Header header;
    header.stamp.setNow(ros::Time::now());
    header.frame_id = base_frame_;
    header.seq = ++seq_;
    
    // Publish thruster message
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrusters_msg;
    thrusters_msg.header = header;
    thrusters_msg.data = thruster;
    thruster0_pub_.publish(thrusters_msg);
    
    // Publish fins message
    uuv_gazebo_ros_plugins_msgs::FloatStamped fins_msg;
    fins_msg.header = header;
    
    // Rudder angle should be reversed due to that slide model control algorithm is based on 
    // NED frame
    // fins_msg.data = upper_p;
    fins_msg.data = -upper_p;
    fin0_pub_.publish(fins_msg);
    // fins_msg.data = upper_s;
    fins_msg.data = -upper_s;
    fin1_pub_.publish(fins_msg);
    
    // fins_msg.data = lower_p;
    fins_msg.data = -lower_p;
    fin2_pub_.publish(fins_msg);
    // fins_msg.data = lower_s;
    fins_msg.data = -lower_s;
    fin3_pub_.publish(fins_msg);
}

//////////////////////////////////
void AUVTrajFollowManagerROS::trajFollowManageThread()
{
    ros::NodeHandle nh;
    ros::Timer manage_timer;
    bool wait_for_wake = false;

    boost::unique_lock<boost::recursive_mutex> lock(manage_mutex_);
    while(nh.ok())
    {
        while(wait_for_wake || !getMissionFlag())
        {
            manage_cond_.wait(lock);
            wait_for_wake = false;
        }
        lock.unlock();

        ros::Time manage_start_t = ros::Time::now();
        
        // update control info 
        updateCtrlInfo();

        controller_ptr_->updateCtrlInfo(x_d_, y_d_, depth_d_, yaw_d_, pitch_d_, u_d_);
        if(debug_)
        {
            boost::unique_lock<boost::recursive_mutex> print_lock(print_mutex_);
            printf("Ctrl info: x: %f y: %f depth: %f yaw: %f pitch: %f u: %f\n",
                   x_d_, y_d_, depth_d_, yaw_d_, pitch_d_, u_d_);
        }

        // update status of vehicle
        double x, y, z, roll, yaw, pitch, u, v, w, roll_dot, pitch_dot, yaw_dot;
        getVehicleStatus(x, y, z, roll, pitch, yaw, u, v, w, roll_dot, pitch_dot, yaw_dot);
        controller_ptr_->updatePose(x, y, z, roll, pitch, yaw, u, v, w, roll_dot, pitch_dot, yaw_dot);

        // check if vehicle accesses the end point
        if(isAccessEndPoint())
        {
            boost::unique_lock<boost::recursive_mutex> print_lock(print_mutex_);
            printf("Vehicle accesses end of point, finish following!\n");
            // print_lock.unlock();
        }

        lock.lock();
        if(manage_dt_ > 0.0)
        {
            ros::Duration sleep_time = (manage_start_t + ros::Duration(manage_dt_)) - ros::Time::now();
            if(sleep_time > ros::Duration(0.0))
            {
                wait_for_wake = true;
                manage_timer = nh.createTimer(sleep_time, & AUVTrajFollowManagerROS::wakeManageThread, this);
            }
        }
    }
}

//////////////////////////////////
void AUVTrajFollowManagerROS::updateCtrlInfo()
{ 
    // get vehicle position
    double x, y, z;
    getPosition(x, y, z);

    // get index of current followed waypoint
    int wp_index = getWayPointIndex();

    if(wp_index >= 0 && wp_index < wp_vec_.size())
    {
        // check if vehicle access the field of way point
        if(abs(x - wp_vec_[wp_index].x) <= thre_ &&
           abs(y - wp_vec_[wp_index].y) <= thre_)
        {
            {
                boost::unique_lock<boost::recursive_mutex> wp_index_lock(wp_index_mutex_);
                ++wp_index_;
                // wp_index_lock.unlock();
            }

            double p1x = wp_vec_[wp_index].x; double p1y = wp_vec_[wp_index].y;
            double p2x = wp_vec_[wp_index+1].x; double p2y = wp_vec_[wp_index+1].y;
            double line_k = std::atan2(p2y - p1y, p2x - p1x);

            {
                boost::unique_lock<boost::recursive_mutex> desired_info_lock(desired_info_mutex_);
                depth_d_ = 0.0; pitch_d_ = 0.0; yaw_d_ = line_k; 
                x_d_ = p1x + 0.2 * (p2x - p1x);
                y_d_ = p1y + 0.2 * (p2y - p2y);
                // desired_info_lock.unlock();
            }
        }
    }  
}

//////////////////////////////////
bool AUVTrajFollowManagerROS::isAccessEndPoint()
{
    int wp_index = getWayPointIndex();

    // get vehicle position
    double x, y, z;
    getPosition(x, y, z);

    // if vehicle accesses the end way point
    if(wp_index == wp_vec_.size())
    {
        if(abs(x - wp_vec_[wp_index].x) <= thre_ &&
           abs(y - wp_vec_[wp_index].y) <= thre_)
        {
            {
                boost::unique_lock<boost::recursive_mutex> wp_index_lock(wp_index_mutex_);
                wp_index_ = -1;
                // wp_index_lock.unlock();
            }

            {
                boost::unique_lock<boost::recursive_mutex> mission_flag_lock(mission_flag_mutex_);
                is_start_mission_ = false;
                // mission_flag_lock.unlock();
            }

            return true;
        }
        else
        {
            return false;
        }
    }

    return false;
}

//////////////////////////////////
bool AUVTrajFollowManagerROS::setMissionStatus(auv_controller::SetMissionStatus::Request& req,
                            auv_controller::SetMissionStatus::Response& res)
{
    if(req.MissionStatus == 0)
    {
        if(is_start_mission_)
        {
            boost::unique_lock<boost::recursive_mutex> mission_flag_lock(mission_flag_mutex_);
            is_start_mission_ = false;
        }
        // mission_flag_lock.unlock();

        {
            boost::unique_lock<boost::recursive_mutex> wp_index_lock(wp_index_mutex_);
            wp_index_ = -1; // reset way point index
        }
        // wp_index_lock.unlock();

        res.FeedbackMsg = "Set mission status as 'stop' successfully!\n";
        return true;
    }
    if(req.MissionStatus == 1)
    {
        if(!is_start_mission_)
        {
            boost::unique_lock<boost::recursive_mutex> mission_flag_lock(mission_flag_mutex_);
            is_start_mission_ = true;
        }
        // mission_flag_lock.unlock();

        {
            boost::unique_lock<boost::recursive_mutex> wp_index_lock(wp_index_mutex_);
            wp_index_ = 0; // reset way point index
            // wp_index_lock.unlock();
        }

        res.FeedbackMsg = "Set mission status as 'start' successfully!\n";
        return true;
    }
    res.FeedbackMsg = "Unknown request status!\n";
    return false;
}

//////////////////////////////////
bool AUVTrajFollowManagerROS::strToGeoPointVector(const XmlRpc::XmlRpcValue& xmlrpc,
                                                std::vector<geometry_msgs::Point>& geo_vec)
{
    if(xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString && xmlrpc != "" && xmlrpc != "[]")
    {
        std::vector<std::vector<double>> vvd;
        std::string err_str;

        // parse string
        // vvd = std::move(parseVVD(std::string(xmlrpc), err_str));
        //
        vvd = parseVVD(std::string(xmlrpc), err_str);
        if(err_str != "")
        {
            return false;
        }
        // convert vvf into points
        geo_vec.reserve(vvd.size());
        for(int i = 0; i < vvd.size(); ++i)
        {
            if(vvd[i].size() == 2)
            {
                geometry_msgs::Point point;
                point.x = vvd[i][0];
                point.y = vvd[i][1];
                point.z = 0.0;
                geo_vec.push_back(point);
            }
            else
            {
                return false;
            }
        }

        return true;
    }
    else
    {
        return false;
    }
}

//////////////////////////////////
std::vector<std::vector<double>> AUVTrajFollowManagerROS::parseVVD(const std::string& input, 
                                                                   std::string& error_return)
{
    std::vector<std::vector<double>> res;

    std::stringstream input_ss(input);
    int depth = 0;

    std::vector<double> cur_vec;
    while(!!input_ss && !input_ss.eof())
    {
        switch(input_ss.peek())
        {
        case EOF:
            break;
        case '[':
            depth++;
            if(depth > 2)
            {
                error_return = "2D point Array depth is greater than 2!";
                return res;
            }
            input_ss.get();
            cur_vec.clear();
            break;
        case ']':
            depth--;
            if(depth < 0)
            {
                error_return = "Error syntax: more close ] than open [!";
                return res;
            }
            input_ss.get();
            if(depth == 1)
            {
                res.push_back(cur_vec);
            }
            break;
        case ',':
        case ' ':
        case '\t':
            input_ss.get();
            break;
        default:
            if(depth != 2)
            {
                std::stringstream err_ss;
                err_ss << "Number at depth other than 2. Char was '" << char(input_ss.peek()) << "'.";
                error_return = err_ss.str();
                return res;
            }
            double value;
            input_ss >> value;
            if(!!input_ss)
            {
                cur_vec.push_back(value);
            }
            break;
        }
    }
    if(depth != 0)
    {
        error_return = "Unterminated vector string!";
    }
    else
    {
        error_return = "";
    }
    return res;
}
}; // ns
   
   
