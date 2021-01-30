/*                                                                                             
 * Filename: AUVControllerROS.cpp
 * Path: auv_controller
 * Created Date: Firday, Janurary 29th 2021, 13:29:16 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <stdio.h>
#include <time.h>
#include <vector>
#include <assert.h>
#include <sstream>
#include <iomanip>
#include "auv_controller/AUVControllerROS.h"


namespace auv_controller{
AUVControllerROS::AUVControllerROS(std::string name, bool with_ff, bool debug) : with_ff_(with_ff), debug_(debug){
   ros::NodeHandle private_nh("~"); // private ros node handle
   ros::NodeHandle nh; // public ros node handle
   
   // Parameters setting
   std::vector<double> dynamic, body_params, ctrl_params, force_params;
   private_nh.getParam("dynamic params", dynamic);
   private_nh.getParam("force params", force_params);
   private_nh.getParam("control params", ctrl_params);
   private_nh.getParam("body params", body_params);

   // Default parameters
   private_nh.param("base frame", base_frame_, std::string("base_link"));
   private_nh.param("rpm", rpm_, 1400);
   private_nh.param("control period", ctrl_dt_, 0.1);
   private_nh.param("publish period", pub_dt_, 0.1);
   private_nh.param("desird y", y_d_, 0.0);
   private_nh.param("desired depth", depth_d_, 20.0);
   double pitch_d = 0.0 * degree2rad;
   double yaw_d = 0.0 * degree2rad;
   private_nh.param("desired pitch", pitch_d_, pitch_d);
   private_nh.param("desired yaw", yaw_d_, yaw_d);

   // Initialization of publisher and subscriber
   imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/armsauv/imu", 1, boost::bind(&AUVControllerROS::imuCb, this, _1));
   pressure_sub_ = nh.subscribe<sensor_msgs::FluidPressure>("/armsauv/pressure", 1, boost::bind(&AUVControllerROS::pressureCb, this, _1));
   posegt_sub_ = nh.subscribe<nav_msgs::Odometry>("/armsauv/pose_gt", 1, boost::bind(&AUVControllerROS::posegtCb, this, _1));
   dvl_sub_ = nh.subscribe<uuv_sensor_ros_plugins_msgs::DVL>("/armsauv/dvl", 1, boost::bind(&AUVControllerROS::dvlCb, this, _1));
   /* reserved for desired params subscription */ 
   
   thruster0_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/thrusters/0/input", 1);
   fin0_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/0/input", 1);
   fin1_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/1/input", 1);
   fin2_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/2/input", 1);
   fin3_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/3/input", 1);
   fin4_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/4/input", 1);
   fin5_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/5/input", 1);

   /* initialize controller */
   if (with_ff){
       controller_ = new AUVControllerWithFF();
   }
   else{
       controller_ = new AUVControllerNoFF();
   }

   /* parameters configuration */
   if(!controller_->setAUVBodyParams(body_params)){
       ROS_WARN("Error in AUV body parameters setting! Use default settings!");
       printAUVBodyParams(); // print body parameters
   }
   if(!controller_->setAUVDynamic(dynamic)){
       ROS_WARN("Error in AUV dynamic parameters setting! Use default settings!");
       printAUVDynamicParams(); // print dyanmic paramters
   }
   if(!controller_->setCtrlParams(ctrl_params)){
       ROS_WARN("Error in AUV control parameters setting! Use default settings!");
       printAUVCtrlParams(); // print control parameters
   }
   if(!controller_->setForceParams(force_params)){
       ROS_WARN("Error in AUV force parameters setting! Use default settings!");
       printAUVForceParams(); // print force parameters
   }

   fwdfin_ = 0.0; backfin_ = 0.0; vertfin_ = 0.0; 

   is_ctrl_run_ = false; is_emerg_run_ = false;

   ctrl_state_ = AUVCtrlState::STANDBY;

   ROS_INFO("Finish controller initialization\n");
}

AUVControllerROS::~AUVControllerROS(){
    if(pub_thread_ != nullptr){
        pub_thread_->interrupt();
        pub_thread_->join();
        delete pub_thread_;
        pub_thread_ = nullptr;
    }

    if(ctrl_thread_ != nullptr){
        ctrl_thread_->interrupt();
        ctrl_thread_->join();
        delete ctrl_thread_;
        ctrl_thread_ = nullptr;
    }

    if(controller_ != nullptr){
        delete controller_;
        controller_ = nullptr;
    }
}

void AUVControllerROS::startControl(){
    ROS_INFO("Start control thread & publish thread");
    ctrl_state_ = AUVCtrlState::CTRL;
    is_ctrl_run_ = true; is_emerg_run_ = false;
    ctrl_thread_ = new boost::thread(boost::bind(&AUVControllerROS::controlThread, this));
    pub_thread_ = new boost::thread(boost::bind(&AUVControllerROS::publishThread, this));
}

void AUVControllerROS::controlThread(){
    ros::NodeHandle nh;
    ros::Timer ctrl_timer;
    bool wait_for_wake = false;

    boost::unique_lock<boost::recursive_mutex> lock(ctrl_mutex_);
    while(nh.ok()){
        while(wait_for_wake || !is_ctrl_run_){
            if(debug_){
                {
                    std::lock_guard<std::mutex> guard(print_mutex_);
                    printf("Control thread is suspending\n");
                }
                ctrl_cond_.wait(lock);
                wait_for_wake = false;
            }
        }
        // control thread wake
        if(debug_){
            std::lock_guard<std::mutex> guard(print_mutex_);
            printf("Control thread is waked\n");
        }
        lock.unlock();

        ros::Time ctrl_start_t = ros::Time::now();

        /* get status */
        AUVKineticSensor sensor_msg;
        sensor_msg.x_ = getGlobalX();
        sensor_msg.y_ = -getGlobalY();
        sensor_msg.z_ = -getGlobalZ();
        sensor_msg.roll_ = getRoll();
        sensor_msg.pitch_ = -getPitch();
        sensor_msg.yaw_ = -getYaw();
        sensor_msg.x_dot_ = getLinVelX();
        // sensor_msg.y_dot_ = -getLinVelY();
        sensor_msg.y_dot_ = 0.0;
        // sensor_msg.z_dot_ = -getLinVelZ();
        sensor_msg.z_dot_ = 0.0;
        sensor_msg.roll_dot_ = -getAngVelRoll();
        sensor_msg.pitch_dot_ = -getAngVelPitch();
        sensor_msg.yaw_dot_ = -getAngVelYaw();

        if(debug_){ std::lock_guard<std::mutex> guard(print_mutex_);
            std::cout << "Vehicle status:{" << "x:" << sensor_msg.x_ << " y:" << sensor_msg.y_ << " z:" << sensor_msg.z_
            << " roll:" << sensor_msg.roll_ << " pitch:" << sensor_msg.pitch_ << " yaw:" << sensor_msg.yaw_
            << " u:" << sensor_msg.x_dot_ << " v:" << sensor_msg.y_dot_ << " w:" << sensor_msg.z_dot_
            << " roll vel:" << sensor_msg.roll_dot_ << " pitch vel:" << sensor_msg.pitch_dot_ << " yaw vel" << sensor_msg.yaw_dot_
            << "}" << std::endl;
        }

        // Create Controller input
        AUVControllerInput input(getDesiredDepth(), getDesiredPitch(), getDesiredYaw(), 30.0, getDesiredY());

        // Print control input
        if(debug_){
            std::lock_guard<std::mutex> guard(print_mutex_);
            std::cout << "Control input:{" << " desired y:" << y_d_ << " desired depth:" << depth_d_ << " desired pitch:" << pitch_d_ << " desired yaw:" << yaw_d_ << "}" << std::endl; 
        }

        AUVControllerOutput output;
        output.fwd_fin_ = 0.0; output.aft_fin_ = 0.0; output.rouder_ = 0.0;
        

        bool get_ctrl_output = false;
        controller_->controllerRun(sensor_msg, input, output, ctrl_dt_);
        get_ctrl_output = nh.ok();

        if(get_ctrl_output){
            std::lock_guard<std::mutex> guard(ctrl_var_mutex_);
            vertfin_ = output.rouder_;
            fwdfin_ = output.fwd_fin_;
            backfin_ = output.aft_fin_;
        }

        if(ctrl_dt_ > 0.0){
            ros::Duration sleep_time = (ctrl_start_t + ros::Duration(ctrl_dt_)) - ros::Time::now();
            if(sleep_time > ros::Duration(0.0)){
                if(debug_){
                    std::lock_guard<std::mutex> guard(print_mutex_);
                    printf("Control thread is waiting for wake\n");
                }
                wait_for_wake = true;
                ctrl_timer = nh.createTimer(sleep_time, &AUVControllerROS::wakeControlThread, this);
            }
        }
    }
}

void AUVControllerROS::wakeControlThread(const ros::TimerEvent& event){
    ctrl_cond_.notify_one();
}

void AUVControllerROS::publishThread(){
    ros::NodeHandle nh;

    // wake control thread
    boost::unique_lock<boost::recursive_mutex> lock(ctrl_mutex_);
    ctrl_cond_.notify_one();
    lock.unlock();

    while(nh.ok()){
        double vertfin, fwdfin, backfin, rpm;
        {
            std::lock_guard<std::mutex> guard(ctrl_var_mutex_);
            vertfin = vertfin_;
            fwdfin = fwdfin_;
            backfin = backfin_;
            rpm = rpm_;
        }

        {
            std::lock_guard<std::mutex> guard(print_mutex_);
            printf("Control output:{vertical fin:%8f forward fin:%8f back fin:%8f}", vertfin, fwdfin, backfin);
        }

        // publish control output
        applyActuatorInput(vertfin, fwdfin, backfin, rpm);

        boost::this_thread::sleep(boost::posix_time::milliseconds(pub_dt_ * 1000));
    };
}

void AUVControllerROS::applyActuatorInput(double vertfin, double fwdfin, double backfin, int rpm){
    std_msgs::Header header;
    header.stamp.setNow(ros::Time::now());
    header.frame_id = base_frame_;
    header.seq = ++seq_;

    // Publish thruster message
    uuv_gazebo_ros_plugins_msgs::FloatStamped thrusters_msg;
    thrusters_msg.header = header;
    thrusters_msg.data = rpm;
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
    else{
        // Vertical fins
        fins_msg.data = vertfin;
        fin3_pub_.publish(fins_msg);
        fins_msg.data = -vertfin;
        fin5_pub_.publish(fins_msg);
        // Backward fins
        fins_msg.data = -backfin;
        fin2_pub_.publish(fins_msg);
        fins_msg.data = backfin;
        fin4_pub_.publish(fins_msg);
    }
}

void AUVControllerROS::imuCb(const sensor_msgs::Imu::ConstPtr& msg) 
{
    // PoseStamped::Quaternion to tf::Quaternion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat);

    // Quaternion to RPY
    std::lock_guard<std::mutex> guard(imu_mutex_);
    tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);                                                                                         

    roll_dot_ = msg->angular_velocity.x;
    pitch_dot_ =  msg->angular_velocity.y;
    yaw_dot_ = msg->angular_velocity.z;
}

void AUVControllerROS::pressureCb(const sensor_msgs::FluidPressure::ConstPtr& msg) // For pressure sensor
{
    std::lock_guard<std::mutex> guard(pressure_mutex_);
    depth_ = static_cast<double>((msg->fluid_pressure - 101) / 10.1) - 0.25;
}

void AUVControllerROS::posegtCb(const nav_msgs::Odometry::ConstPtr& msg) // For pose sensor
{
    std::lock_guard<std::mutex> guard(posegt_mutex_);
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    z_ = msg->pose.pose.position.z;
}

void AUVControllerROS::dvlCb(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg) // For DVL
{
    std::lock_guard<std::mutex> guard(dvl_mutex_);
    u_ = msg->velocity.x;
    v_ = msg->velocity.y;
    w_ = msg->velocity.z;
}

void AUVControllerROS::printAUVDynamicParams(){
    std::stringstream ss;
    ss << "AUV dynamic parameters: {";
    controller_->serializeAUVDynamicParams(ss);
    ss << "}";
    ROS_INFO_STREAM(ss.str());
}

void AUVControllerROS::printAUVCtrlParams(){
    std::stringstream ss;
    ss << "AUV control parameters: {";
    controller_->serializeAUVControlParams(ss);
    ss << "}";
    ROS_INFO_STREAM(ss.str());
}

void AUVControllerROS::printAUVForceParams(){
    std::stringstream ss;
    ss << "AUV Force parameters: {";
    controller_->serializeAUVForceParams(ss);
    ss << "}";
    ROS_INFO_STREAM(ss.str());
}

void AUVControllerROS::printAUVBodyParams(){
    std::stringstream ss;
    ss << "AUV body parameters: {";
    controller_->serializeAUVBodyParams(ss);
    ss << "}";
    ROS_INFO_STREAM(ss.str());
}

}; // ns
