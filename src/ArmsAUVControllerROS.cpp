/*
 * Filename: auv_control\ArmsAUVController.cpp
 * Path: auv_control
 * Created Date: Wednesday, September 9th 2020, 10:01:12 am
 * Author: zhao wang
 * Res work: 1. Warning and error mechanism
 *           2. Other types of demo: xy test, yz test et al.
 * Copyright (c) 2020 Your Company
 */

#include <vector>
#include <assert.h>
#include "auv_controller/ArmsAUVControllerROS.h"

namespace auv_controller{
    /**
     * @brief Constructor
     */
    ArmsAUVControllerROS::ArmsAUVControllerROS(std::string name){
        ros::NodeHandle private_nh("~"); // private ros node handle
        ros::NodeHandle nh; // public ros node handle

        // Parameters setting
        std::vector<double> dynamic, body_params, ctrl_params, force_params; 
        private_nh.getParam("dynamic", dynamic);
        private_nh.getParam("force", force_params);
        private_nh.getParam("control", ctrl_params);
        private_nh.getParam("body", body_params);

        // Default parameters
        private_nh.param("base_frame", base_frame_, std::string("base_link"));
        private_nh.param("rpm", rpm_, 1250);
        private_nh.param("control_period", dt_, 0.1);
        private_nh.param("xd", x_d_, 30.0);
        private_nh.param("yd", y_d_, 0.0);
        private_nh.param("depthd", depth_d_, 10.0);
        private_nh.param("pitchd", pitch_d_, 0.0);
        // private_nh.param("yawd", yaw_d_, 30 * degree2rad);
        private_nh.param("yawd", yaw_d_, 0.0);

        printf("Target:{rpm:%d ctrl_period:%f x:%f y:%f depth:%f pitch:%f yaw:%f}\n", rpm_, dt_, x_d_, y_d_, depth_d_, pitch_d_, yaw_d_);

        // Initialization of publisher and subscriber
        imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/armsauv/imu", 1, boost::bind(&ArmsAUVControllerROS::imuCb, this, _1));
        pressure_sub_ = nh.subscribe<sensor_msgs::FluidPressure>("/armsauv/pressure", 1, boost::bind(&ArmsAUVControllerROS::pressureCb, this, _1)); 
	    posegt_sub_ = nh.subscribe<nav_msgs::Odometry>("/armsauv/pose_gt", 1, boost::bind(&ArmsAUVControllerROS::posegtCb, this, _1));
        depth_sub_ = nh.subscribe<std_msgs::Float64>("/armsauv/control_input/depth", 1, boost::bind(&ArmsAUVControllerROS::depthCb, this, _1));
        pitch_sub_ = nh.subscribe<std_msgs::Float64>("/armsauv/control_input/pitch", 1, boost::bind(&ArmsAUVControllerROS::pitchCb, this, _1));
        yaw_sub_ = nh.subscribe<std_msgs::Float64>("/armsauv/control_input/yaw", 1, boost::bind(&ArmsAUVControllerROS::yawCb, this, _1));

        thruster0_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/thrusters/0/input", 1);
        fin0_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/0/input", 1);
        fin1_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/1/input", 1);
        fin2_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/2/input", 1);
        fin3_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/3/input", 1);
        fin4_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/4/input", 1);
        fin5_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/5/input", 1);

        controller_ = new AUVController();

        /* parameters setting */
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

        // Create Timer
        // ros::Timer timer = nh.createTimer(ros::Duration(dt_), boost::bind(&ArmsAUVControllerROS::timerCb, this, _1));
        
        // Create control thread
        ROS_INFO("Start control thread");
	ctrl_thread_ = new boost::thread(boost::bind(&ArmsAUVControllerROS::controlThread, this));	
    }

    /**
     * @brief Deconstructor
     */ 
    ArmsAUVControllerROS::~ArmsAUVControllerROS(){
        // Release thread
	if(ctrl_thread_ != nullptr){
	    delete ctrl_thread_;
	    ctrl_thread_ = nullptr;
	} 

	// Release controller
        if(controller_ != nullptr){
            delete controller_;
            controller_ = nullptr;
        }
    }

    /**
     * @brief Timer callback
     */
    void ArmsAUVControllerROS::timerCb(const ros::TimerEvent& event){
        if(getX() == 0 || getXVelocity() == 0){
            return;
        }

        // Update sensor messages
        AUVKineticSensor sensor_msg;
        sensor_msg.x_ = getX();
        sensor_msg.y_ = -getY();
        sensor_msg.z_ = -getZ();
        sensor_msg.roll_ = getRoll();
        sensor_msg.pitch_ = -getPitch();
        sensor_msg.yaw_ = -getYaw();
        sensor_msg.x_dot_ = getXVelocity();
        sensor_msg.y_dot_ = -getYVelocity();
        sensor_msg.z_dot_ = -getZVelocity();
        sensor_msg.roll_dot_ = -getRollVelocity();
        sensor_msg.pitch_dot_ = -getPitchVelocity();
        sensor_msg.yaw_dot_ = -getYawVelocity();

        // Create Controller input
        AUVControllerInput input(getXInput(), getYInput(), getDepthInput(), getPitchInput(), getYawInput() * degree2rad);

        // Create Controller output
        AUVControllerOutput output;

        // Run controller
        controller_->controllerRun(sensor_msg, input, output, dt_);

        // Apply controller output
        applyActuatorInput(output.rouder_, output.fwd_fin_, output.aft_fin_, rpm_);
    };

    /**
     * @brief Control thread i
     */
    void ArmsAUVControllerROS::controlThread(){
        /*
	if(getX() == 0 || getXVelocity() == 0){
            return;
        }
	*/
        ros::NodeHandle nh;
        while(nh.ok()){	
            // if(!(getX() == 0 || getXVelocity() == 0)){
                // Update sensor messages
	        AUVKineticSensor sensor_msg;
                sensor_msg.x_ = getX();
                sensor_msg.y_ = -getY();
                sensor_msg.z_ = -getZ();
                sensor_msg.roll_ = getRoll();
                sensor_msg.pitch_ = -getPitch();
                sensor_msg.yaw_ = -getYaw();
                sensor_msg.x_dot_ = getXVelocity();
                sensor_msg.y_dot_ = -getYVelocity();
                sensor_msg.z_dot_ = -getZVelocity();
                sensor_msg.roll_dot_ = -getRollVelocity();
                sensor_msg.pitch_dot_ = -getPitchVelocity();
                sensor_msg.yaw_dot_ = -getYawVelocity();

	        // Print 
	        std::cout << "Update sensor messages:{" << "x:" << sensor_msg.x_ << " y:" << sensor_msg.y_ << " z:" << sensor_msg.z_
		  << " roll:" << sensor_msg.roll_ << " pitch:" << sensor_msg.pitch_ << " yaw:" << sensor_msg.yaw_
		  << " x vel:" << sensor_msg.x_dot_ << " y vel:" << sensor_msg.y_dot_ << " z vel:" << sensor_msg.z_dot_
		  << " roll vel:" << sensor_msg.roll_dot_ << " pitch vel:" << sensor_msg.pitch_dot_ << " yaw vel" << sensor_msg.yaw_dot_
		  << "}" << std::endl;
        
	        // Create Controller input
            AUVControllerInput input(getXInput(), getYInput(), getDepthInput(), getPitchInput(), getYawInput() * degree2rad);
        
	        // Print control input
	        std::cout << "Control input:{" << "desired x:" << x_d_ << " desired y:" << y_d_ << " desired depth:" << depth_d_ << " desired pitch:" << pitch_d_ << " desired yaw:" << yaw_d_ << "}" << std::endl; 

            // Create Controller output
            AUVControllerOutput output;
            output.fwd_fin_ = 0.0; output.aft_fin_ = 0.0; output.rouder_ = 0.0;
            
            // Run controller
            controller_->controllerRun(sensor_msg, input, output, dt_);
	
	        // Print control ouput
	        std::cout << "Control output:{" << "rouder:" << output.rouder_ << " forward fin:" << output.fwd_fin_ << " stern fin:" << output.aft_fin_ << "}" << std::endl; 

            // Apply controller output
            applyActuatorInput(output.rouder_, output.fwd_fin_, output.aft_fin_, rpm_);
            
            // Sleep
            boost::this_thread::sleep(boost::posix_time::milliseconds(dt_ * 1000));
        }
    } 

    /**
     * @brief Apply controller output
     */ 
    void ArmsAUVControllerROS::applyActuatorInput(double rouder, double fwdfin, double aftfin, double rpm){
        // Create new message header
        std_msgs::Header header;
        header.stamp.setNow(ros::Time::now());
        header.frame_id = base_frame_;
        header.seq = ++seq_;

        // Publish thrusters message
        uuv_gazebo_ros_plugins_msgs::FloatStamped thrusters_msg;
        thrusters_msg.header = header;
        thrusters_msg.data = rpm;
        thruster0_pub_.publish(thrusters_msg);

        // Publish fins message
        uuv_gazebo_ros_plugins_msgs::FloatStamped fins_msg;
        fins_msg.header = header;
        // Rouder
        fins_msg.data = rouder;
        fin3_pub_.publish(fins_msg);
        fins_msg.data = -rouder;
        fin5_pub_.publish(fins_msg);
        // Forward fins
        fins_msg.data = fwdfin;
        fin0_pub_.publish(fins_msg);
        fins_msg.data = -fwdfin;
        fin1_pub_.publish(fins_msg);
        // Backward fins
        fins_msg.data = -aftfin;
        fin2_pub_.publish(fins_msg);
        fins_msg.data = aftfin;
        fin4_pub_.publish(fins_msg);
    }

    /**
     * @brief Sensor callback
     */
    void ArmsAUVControllerROS::imuCb(const sensor_msgs::Imu::ConstPtr& msg) // For IMU
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

    void ArmsAUVControllerROS::pressureCb(const sensor_msgs::FluidPressure::ConstPtr& msg) // For pressure sensor
    {
        std::lock_guard<std::mutex> guard(pressure_mutex_);
        depth_ = static_cast<double>((msg->fluid_pressure - 101) / 10.1) - 0.25;
    }

    void ArmsAUVControllerROS::posegtCb(const nav_msgs::Odometry::ConstPtr& msg) // For pose sensor
    {
        std::lock_guard<std::mutex> guard(posegt_mutex_);
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        z_ = msg->pose.pose.position.z;
    }

    void ArmsAUVControllerROS::dvlCb(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg) // For DVL
    {
        std::lock_guard<std::mutex> guard(dvl_mutex_);
        x_dot_ = msg->velocity.x;
        y_dot_ = msg->velocity.y;
        z_dot_ = msg->velocity.z;
    }

    void ArmsAUVControllerROS::depthCb(const std_msgs::Float64::ConstPtr& msg){
        std::lock_guard<std::mutex> guard(depth_mutex_);
        depth_d_ = msg->data;
    }

    void ArmsAUVControllerROS::pitchCb(const std_msgs::Float64::ConstPtr& msg){
        std::lock_guard<std::mutex> guard(pitch_mutex_);
        pitch_d_ = msg->data;
    }

    void ArmsAUVControllerROS::yawCb(const std_msgs::Float64::ConstPtr& msg){
        std::lock_guard<std::mutex> guard(yaw_mutex_);
        yaw_d_ = msg->data;
    }

    void ArmsAUVControllerROS::xdCb(const std_msgs::Float64::ConstPtr& msg){
        std::lock_guard<std::mutex> guard(x_d_mutex_);
        x_d_ = msg->data;
    }

    void ArmsAUVControllerROS::ydCb(const std_msgs::Float64::ConstPtr& msg){
        std::lock_guard<std::mutex> guard(y_d_mutex_);
        y_d_ = msg->data;
    }

    /**
     * @brief Print auv dynamic parameters on the console
     */ 
    void ArmsAUVControllerROS::printAUVDynamicParams(){
        std::stringstream ss;
        ss << "AUV dynamic parameters: {";
        controller_->serializeAUVDynamicParams(ss);
        ss << "}";
        ROS_INFO_STREAM(ss.str());
    }

    /**
     * @brief Print auv control parameters on the console
     */ 
    void ArmsAUVControllerROS::printAUVCtrlParams(){
        std::stringstream ss;
        ss << "AUV control parameters: {";
        controller_->serializeAUVControlParams(ss);
        ss << "}";
        ROS_INFO_STREAM(ss.str());
    }

    /**
     * @brief Print auv force parameters on the console
     */ 
    void ArmsAUVControllerROS::printAUVForceParams(){
        std::stringstream ss;
        ss << "AUV Force parameters: {";
        controller_->serializeAUVForceParams(ss);
        ss << "}";
        ROS_INFO_STREAM(ss.str());
    }

    /**
     * @brief Print auv body parameters on the console
     */ 
    void ArmsAUVControllerROS::printAUVBodyParams(){
        std::stringstream ss;
        ss << "AUV body parameters: {";
        controller_->serializeAUVBodyParams(ss);
        ss << "}";
        ROS_INFO_STREAM(ss.str());
    }
}; // ns
