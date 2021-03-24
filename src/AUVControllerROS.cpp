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

/* Common test macro */
#define SM_CTRL
// #define THRUST_TEST
// #define SPIRAL_TEST
// #define XRUDDER_TEST

/* Emergency test macro*/
// # define DEPTH_EM_TEST
// # define PITCH_EM_TEST
// # define YAW_EM_TEST

namespace auv_controller{
/////////////////////////////////////
AUVControllerROS::AUVControllerROS(std::string auv_name, bool with_ff, bool x_type, bool debug) : 
    with_ff_(with_ff), x_type_(x_type), debug_(debug){
   ros::NodeHandle private_nh("~"); // private ros node handle
   ros::NodeHandle nh; // public ros node handle
   
   // Parameters setting
   std::vector<double> dynamic, body_params, ctrl_params, force_params;

   // private_nh.getParam("name", auv_name);
   private_nh.getParam("dynamic_params", dynamic);
   private_nh.getParam("force_params", force_params);
   private_nh.getParam("control_params", ctrl_params);
   private_nh.getParam("body_params", body_params);

   // Default parameters
   private_nh.param("base_frame", base_frame_, std::string("base_link"));
   private_nh.param("rpm", rpm_, 1400.0);
   ori_rpm_ = rpm_;
   private_nh.param("control_period", ctrl_dt_, 0.1);
   private_nh.param("publish_period", pub_dt_, 0.2);
   private_nh.param("emergency_check_period", em_check_dt_, 0.5);
   private_nh.param("stable_wait_time", stable_wait_t_, 90.0);

   private_nh.param("desired_x", x_d_, 40.0);
   private_nh.param("desired_y", y_d_, 10.0);
   private_nh.param("desired_depth", depth_d_, 10.0);
   double pitch_d = 0.0 * degree2rad;
   double yaw_d = 0.0 * degree2rad;
   private_nh.param("desired_pitch", pitch_d_, pitch_d);
   private_nh.param("desired_yaw", yaw_d_, yaw_d);

   double kp, ki, kd, c_t, r_l, l_l, sigma;
   private_nh.param("kp", kp, 1.0);
   private_nh.param("ki", ki, 0.0);
   private_nh.param("kd", kd, 0.1);
   private_nh.param("desired_u", u_d_, 1.5432);
   // private_nh.param("Ct", c_t, 2.4e-5);
   private_nh.param("Ct", c_t, 0.0875);
   private_nh.param("r_death_area", r_l, 888.0);
   private_nh.param("l_death_area", l_l, -888.0);
   private_nh.param("sigma", sigma, -1.0);

   // Initialization of publisher and subscriber
   imu_sub_ = nh.subscribe<sensor_msgs::Imu>(auv_name+"/imu", 1, boost::bind(&AUVControllerROS::imuCb, this, _1));
   pressure_sub_ = nh.subscribe<sensor_msgs::FluidPressure>(auv_name+"/pressure", 1, boost::bind(&AUVControllerROS::pressureCb, this, _1));
   posegt_sub_ = nh.subscribe<nav_msgs::Odometry>(auv_name+"/pose_gt", 1, boost::bind(&AUVControllerROS::posegtCb, this, _1));
   dvl_sub_ = nh.subscribe<uuv_sensor_ros_plugins_msgs::DVL>(auv_name+"/dvl", 1, boost::bind(&AUVControllerROS::dvlCb, this, _1));
   /* reserved for desired params subscription */ 
   
   thruster0_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name+"/thrusters/0/input", 1);
   fin0_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name+"/fins/0/input", 1);
   fin1_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name+"/fins/1/input", 1);
   fin2_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name+"/fins/2/input", 1);
   fin3_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name+"/fins/3/input", 1);
   fin4_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name+"/fins/4/input", 1);
   fin5_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name+"/fins/5/input", 1);

   front_rudder_ang_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name+"/front_rudder_angle", 1);
   back_rudder_ang_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name+"/back_rudder_angle", 1);
   vert_rudder_ang_pub_ = nh.advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name+"/vertical_rudder_angle", 1);

   ctrl_state_reset_srv_ = nh.advertiseService("reset_ctrl_state", &AUVControllerROS::resetCtrlState, this);

   /* initialize controller */
   if (with_ff){
       ROS_INFO("Model with front fins");
       controller_ = new AUVControllerWithFF();
   }
   else{
       if(x_type){
           ROS_INFO("Model with X type rudder");
           controller_ = new AUVControllerXF();
       }
       else{
           ROS_INFO("Model without front fins");
           controller_ = new AUVControllerNoFF();
       }
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
   if(!x_type){
       if(!controller_->setForceParams(force_params)){
           ROS_WARN("Error in AUV force parameters setting! Use default settings!");
           printAUVForceParams(); // print force parameters
       }
   }
   else{
       if(!controller_->setXForceParams(force_params)){
           ROS_WARN("Error in AUV xforce parameters setting! Use default settings!");
           printAUVXForceParams(); // print force parameters
       }
   }
   controller_->setThrusterFactor(c_t, r_l, l_l, sigma);

   is_ctrl_vel_ = false; is_wait_stable_ = false;

   fwdfin_ = 0.0; backfin_ = 0.0; vertfin_ = 0.0; 

   // is_ctrl_run_ = false; is_emerg_run_ = false;

   ctrl_state_ = AUVCtrlState::STANDBY;
   em_event_ = EmergencyEvent::NO_EM_EVENT;

   prev_yaw_ = 0.0;

   ROS_INFO("Finish controller initialization\n");
}

/////////////////////////////////////
AUVControllerROS::~AUVControllerROS(){
    if(pub_thread_ != nullptr){
        pub_thread_->interrupt();
        pub_thread_->join();
        delete pub_thread_;
        pub_thread_ = nullptr;
    }

    if(vel_ctrl_thread_ != nullptr){
        vel_ctrl_thread_->interrupt();
        vel_ctrl_thread_->join();
        delete vel_ctrl_thread_;
        vel_ctrl_thread_ = nullptr;
    }
    
    if(ctrl_thread_ != nullptr){
        ctrl_thread_->interrupt();
        ctrl_thread_->join();
        delete ctrl_thread_;
        ctrl_thread_ = nullptr;
    }

    if(em_depth_check_thread_ != nullptr)
    {
        em_depth_check_thread_->interrupt();
        em_depth_check_thread_->join();
        delete em_depth_check_thread_;
        em_depth_check_thread_ = nullptr;
    }

    if(em_roll_check_thread_ != nullptr)
    {
        em_roll_check_thread_->interrupt();
        em_roll_check_thread_->join();
        delete em_roll_check_thread_;
        em_roll_check_thread_ = nullptr;
    }

    if(em_pitch_check_thread_ != nullptr)
    {
        em_pitch_check_thread_->interrupt();
        em_pitch_check_thread_->join();
        delete em_pitch_check_thread_;
        em_pitch_check_thread_ = nullptr;
    }
    
    if(em_yaw_check_thread_ != nullptr)
    {
        em_yaw_check_thread_->interrupt();
        em_yaw_check_thread_->join();
        delete em_yaw_check_thread_;
        em_yaw_check_thread_ = nullptr;
    }

    if(controller_ != nullptr){
        delete controller_;
        controller_ = nullptr;
    }
}

/////////////////////////////////////
void AUVControllerROS::startControl(){
    ROS_INFO("Start control thread & publish thread");
    ctrl_state_ = AUVCtrlState::CTRL;
    // is_ctrl_run_ = true; is_emerg_run_ = false;
#ifdef SM_CTRL
    ctrl_thread_ = new boost::thread(boost::bind(&AUVControllerROS::controlThread, this));
    vel_ctrl_thread_ = new boost::thread(boost::bind(&AUVControllerROS::velControlThread, this));
#endif
    // em_depth_check_thread_ = new boost::thread(boost::bind(&AUVControllerROS::emDepthCheckThread, this));
    // em_roll_check_thread_ = new boost::thread(boost::bind(&AUVControllerROS::emRollCheckThread, this));
    // em_pitch_check_thread_ = new boost::thread(boost::bind(&AUVControllerROS::emPitchCheckThread, this));
    // em_yaw_check_thread_ = new boost::thread(boost::bind(&AUVControllerROS::emYawCheckThread, this));
    pub_thread_ = new boost::thread(boost::bind(&AUVControllerROS::publishThread, this));
}

/////////////////////////////////////
void AUVControllerROS::controlThread(){
    ros::NodeHandle nh;
    ros::Timer ctrl_timer;
    bool wait_for_wake = false;

    boost::unique_lock<boost::recursive_mutex> lock(ctrl_mutex_);
    while(nh.ok()){
        // while(wait_for_wake || !is_ctrl_run_){
        while(wait_for_wake || !(ctrl_state_ == AUVCtrlState::CTRL)){
            // if(debug_){
            //     std::lock_guard<std::mutex> guard(print_mutex_);
            //     printf("Control thread is suspending\n");
            // }
            ctrl_cond_.wait(lock);
            wait_for_wake = false;
        }
        // control thread wake
        // if(debug_){
        //     std::lock_guard<std::mutex> guard(print_mutex_);
        //     printf("Control thread is waked\n");
        // }
        lock.unlock();

        ros::Time ctrl_start_t = ros::Time::now();

        /* get status */
        // Y,Z,yaw and pitch should be reversed due to that the slide model control algorithm
        // is based on NED frame
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

        if(debug_){ 
            std::lock_guard<std::mutex> guard(print_mutex_);
            std::cout << "Vehicle status:{" << "x:" << sensor_msg.x_ << " y:" << sensor_msg.y_ << " z:" << sensor_msg.z_
            << " roll:" << sensor_msg.roll_ << " pitch:" << sensor_msg.pitch_ << " yaw:" << sensor_msg.yaw_
            << " u:" << sensor_msg.x_dot_ << " v:" << sensor_msg.y_dot_ << " w:" << sensor_msg.z_dot_
            << " roll vel:" << sensor_msg.roll_dot_ << " pitch vel:" << sensor_msg.pitch_dot_ << " yaw vel" << sensor_msg.yaw_dot_
            << "}" << std::endl;
        }

        // Create Controller input
        // AUVControllerInput input(getDesiredDepth(), getDesiredPitch(), getDesiredYaw(), 30.0, getDesiredY(), getDesiredLinVelX());
        AUVControllerInput input(getDesiredDepth(), -getDesiredPitch(), -getDesiredYaw(), getDesiredX(), -getDesiredY(), getDesiredLinVelX());

        // Print control input
        if(debug_){
            std::lock_guard<std::mutex> guard(print_mutex_);
            std::cout << "Control input:{" << " desired y:" << y_d_ << " desired depth:" << depth_d_ <<
                " desired pitch:" << pitch_d_ << " desired yaw:" << yaw_d_ << "desired u:" << u_d_ << "}" << std::endl; 
        }

        AUVControllerOutput output;
        output.fwd_fin_ = 0.0; output.aft_fin_ = 0.0; output.rudder_ = 0.0; 
        output.upper_p_ = 0.0; output.upper_s_ = 0.0; output.lower_p_ = 0.0; output.lower_s_ = 0.0;

        bool get_ctrl_output = false;
        // bool ctrl_vel = false;
        // if(isStable()){
        //     ctrl_vel = true;
        // }
        // controller_->controllerRun(sensor_msg, input, output, ctrl_dt_, ctrl_vel);
        controller_->controllerRun(sensor_msg, input, output, ctrl_dt_, is_ctrl_vel_);

        get_ctrl_output = nh.ok();

        if(get_ctrl_output){
            std::lock_guard<std::mutex> guard(ctrl_var_mutex_);
            // update rudder info 
            if(!x_type_)
            {
                vertfin_ = output.rudder_;
                fwdfin_ = output.fwd_fin_;
                backfin_ = output.aft_fin_;

            }
            else
            {
                upper_p_ = output.upper_p_;
                upper_s_ = output.upper_s_;
                lower_p_ = output.lower_p_;
                lower_s_ = output.lower_s_;
            }

            // update rotor speed 
            if(is_ctrl_vel_){
                rpm_ += output.rpm_;
                if(debug_){
                    std::lock_guard<std::mutex> guard(print_mutex_);
                    printf("RPM: %f\n", rpm_);
                }
            }
            else
            {
                rpm_ = ori_rpm_;
                if(debug_){
                    std::lock_guard<std::mutex> guard(print_mutex_);
                    printf("RPM: %f\n", rpm_);
                }
            }
        }

        lock.lock();

        if(ctrl_dt_ > 0.0){
            ros::Duration sleep_time = (ctrl_start_t + ros::Duration(ctrl_dt_)) - ros::Time::now();
            if(sleep_time > ros::Duration(0.0)){
                // if(debug_){
                //     std::lock_guard<std::mutex> guard(print_mutex_);
                //     printf("Control thread is waiting for wake\n");
                // }
                wait_for_wake = true;
                ctrl_timer = nh.createTimer(sleep_time, &AUVControllerROS::wakeControlThread, this);
            }
        }
    }
}

/////////////////////////////////////
void AUVControllerROS::velControlThread(){
    ros::NodeHandle nh;
    while(nh.ok()){
        if(isStable()){
            if(!is_ctrl_vel_ && !is_wait_stable_){
                // if stable and flag is not changed, only update stable time.
                last_stable_ = ros::Time::now();
                is_wait_stable_ = true;
                if(debug_){
                    std::lock_guard<std::mutex> guard(print_mutex_);
                    printf("Wait to control velocity\n");
                }
            }
            else{
                if(is_wait_stable_){
                    // if stable and flag has been changed, check wait time.
                    ros::Duration sleep_time = (ros::Time::now() - last_stable_);
                    if(sleep_time >= ros::Duration(stable_wait_t_)){
                        std::lock_guard<std::mutex> guard(ctrl_vel_mutex_);
                        is_ctrl_vel_ = true;
                        // last_stable_ = ros::Time::now();
                        is_wait_stable_ = false;
                        if(debug_){
                            std::lock_guard<std::mutex> guard(print_mutex_);
                            printf("End velocity control waiting\n");
                        }
                    }
                }
            }
        }
        else{
            std::lock_guard<std::mutex> guard(ctrl_vel_mutex_);
            is_ctrl_vel_ = false;
            // if(debug_){
            //     std::lock_guard<std::mutex> guard(print_mutex_);
            //     printf("Unstable\n");
            // }
        }
    }
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
}

/////////////////////////////////////
void AUVControllerROS::wakeControlThread(const ros::TimerEvent& event){
    ctrl_cond_.notify_one();
}

/////////////////////////////////////
void AUVControllerROS::wakeEMDepthCheckThread(const ros::TimerEvent& event){
    em_depth_check_cond_.notify_one();
}

/////////////////////////////////////
void AUVControllerROS::wakeEMRollCheckThread(const ros::TimerEvent& event){
    em_roll_check_cond_.notify_one();
}

/////////////////////////////////////
void AUVControllerROS::wakeEMPitchCheckThread(const ros::TimerEvent& event){
    em_pitch_check_cond_.notify_one();
}

/////////////////////////////////////
void AUVControllerROS::wakeEMYawCheckThread(const ros::TimerEvent& event){
    em_yaw_check_cond_.notify_one();
}

/////////////////////////////////////
void AUVControllerROS::publishThread(){
    ros::NodeHandle nh;

    // wake emergency check threads
    boost::unique_lock<boost::recursive_mutex> em_depth_check_lock(em_depth_check_mutex_);
    em_depth_check_cond_.notify_one();
    em_depth_check_lock.unlock();

    boost::unique_lock<boost::recursive_mutex> em_roll_check_lock(em_roll_check_mutex_);
    em_roll_check_cond_.notify_one();
    em_roll_check_lock.unlock();

    boost::unique_lock<boost::recursive_mutex> em_pitch_check_lock(em_pitch_check_mutex_);
    em_pitch_check_cond_.notify_one();
    em_pitch_check_lock.unlock();

    boost::unique_lock<boost::recursive_mutex> em_yaw_check_lock(em_yaw_check_mutex_);
    em_yaw_check_cond_.notify_one();
    em_yaw_check_lock.unlock();

    //wake control thread
    boost::unique_lock<boost::recursive_mutex> ctrl_lock(ctrl_mutex_);
    ctrl_cond_.notify_one();
    ctrl_lock.unlock();

    while(nh.ok()){
        double vertfin, fwdfin, backfin, rpm;
        double upper_p, upper_s, lower_p, lower_s;
        if(!x_type_)
        {
            std::lock_guard<std::mutex> guard(ctrl_var_mutex_);
            vertfin = vertfin_;
            fwdfin = fwdfin_;
            backfin = backfin_;
            rpm = rpm_;

#ifdef SPIRAL_TEST
            vertfin = 20 / 57.3;
            fwdfin = 5 / 57.3;
            backfin = -5 / 57.3;
            rpm = 1900;
#endif

#ifdef THRUST_TEST
            vertfin = 0.0;
            fwdfin = 0.0;
            backfin = 0.0;
            rpm = 1900;
#endif

#ifdef DEPTH_EM_TEST
            vertfin = 0.0 / 57.3;
            fwdfin = 5.0 / 57.3; 
            backfin = -5.0 / 57.3; 
            rpm = 1900; 
#endif

#ifdef PITCH_EM_TEST
            vertfin = 0.0 / 57.3;
            fwdfin = 40.0 / 57.3;
            backfin = -40.0 / 57.3;
            rpm = 1250;
#endif

#ifdef YAW_EM_TEST
            vertfin = fluc_.getFactor() * 70.0/57.3;
            fwdfin = 0.0;
            backfin = 0.0;
            rpm = 1250;
#endif

            // If current state is in level1 emergency, stop action
            if(ctrl_state_ == AUVCtrlState::EMERGENCY_LEVEL1)
            {
                vertfin = 0.0;
                fwdfin = 0.0;
                backfin = 0.0;
                rpm = 0.0;
                std::lock_guard<std::mutex> guard(print_mutex_);
                printf("Level1 emergency process! Em event: \n", em_event_);
            }
            // If current state is in level2 emergency, set max rudder and wait for AUV come-up
            if(ctrl_state_ == AUVCtrlState::EMERGENCY_LEVEL2)
            {
                vertfin = 0.0 / 57.3;
                fwdfin = -30 / 57.3;
                backfin = 30 / 57.3;
                rpm = 0.0;
                std::lock_guard<std::mutex> guard(print_mutex_);
                printf("Level2 emergency process! Em event: \n", em_event_);
            }
            // If current state is in level3 emergency, set max rudder and reject load, then wait for AUV come-up
            if(ctrl_state_ == AUVCtrlState::EMERGENCY_LEVEL3)
            {
                vertfin = 0.0 / 57.3;
                fwdfin = -30 / 57.3;
                backfin = 30 / 57.3;
                rpm = 0.0;
                std::lock_guard<std::mutex> guard(print_mutex_);
                printf("Level3 emergency process! Em event: \n", em_event_);
                /* Reserved for load rejection action */
            }
        }
        else
        {
            std::lock_guard<std::mutex> guard(ctrl_var_mutex_);
            upper_p = upper_p_;
            upper_s = upper_s_;
            lower_p = lower_p_;
            lower_s = lower_s_;
            rpm = rpm_;

#ifdef XRUDDER_TEST
            upper_p = 20 / 57.3;
            upper_s = 20 / 57.3;
            lower_p = 20 / 57.3;
            lower_s = 20 / 57.3;
            rpm = 1900;
#endif
        }

        // print statues info
        if(!x_type_)
        {
            std::lock_guard<std::mutex> guard(print_mutex_);
            printf("Control output:{vertical fin:%8f forward fin:%8f back fin:%8f rpm:%8f\n}", 
                   vertfin, fwdfin, backfin, rpm);
        }
        else
        {
            std::lock_guard<std::mutex> guard(print_mutex_);
            printf("Control output:{upper port fin:%8f upper starboard fin:%8f lower port fin:%8f lower starboard fin:%8f rpm:%8f\n}", 
                   upper_p, upper_s, lower_p, lower_s, rpm);
        }

        // publish control output
        if(!x_type_)
        {
            applyActuatorInput(vertfin, fwdfin, backfin, rpm);
        }
        else
        {
            applyActuatorInput(upper_p, upper_s, lower_p, lower_s, rpm);
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(pub_dt_ * 1000));
    };
}

/////////////////////////////////////
void AUVControllerROS::emDepthCheckThread()
{
    ros::NodeHandle nh;
    ros::Timer em_depth_check_timer;
    bool wait_for_wake = true;

    int check_cnt = 0;

    while(nh.ok())
    {
        boost::unique_lock<boost::recursive_mutex> lock(em_depth_check_mutex_);
        // Wait for wake
        while(wait_for_wake){
            em_depth_check_cond_.wait(lock);
            wait_for_wake = false;                                                       
        }
        lock.unlock();

        ros::Time check_start_t = ros::Time::now();
        double cur_depth = -getGlobalZ();
        double desired_depth = getDesiredDepth();
        double delta_depth = cur_depth - desired_depth;

        // If depth access the threshold but not access check count, only add check count 
        if(delta_depth >= 2.0 && check_cnt < 2) 
        {
            ++check_cnt; 
        }
        // If depth access the threshold and access check count, check the emergency level 
        else if(delta_depth >= 2.0 && check_cnt == 2)
        {
            if(delta_depth < 4.0)
            {
                // If current state level is lower than level1 emergency, update event and level
                // if(ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL1 &&
                //    ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL2 &&
                //    ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL3)
                // {
                //     boost::unique_lock<boost::recursive_mutex> guard(em_event_mutex_);
                //     em_event_ = EmergencyEvent::ACCESS_DESIRED_DEPTH_LEVEL1_THRESHOLD;
                //     ctrl_state_ = AUVCtrlState::EMERGENCY_LEVEL1;
                // }
                boost::unique_lock<boost::recursive_mutex> guard(em_event_mutex_);
                em_event_ = EmergencyEvent::ACCESS_DESIRED_DEPTH_LEVEL1_THRESHOLD;
                ctrl_state_ = AUVCtrlState::EMERGENCY_LEVEL1;
            }
            if(delta_depth >= 4.0 && delta_depth < 8.0)
            {
                // If current state level is lower than level2 emergency, update event and level
                if(ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL2 &&
                   ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL3)
                {
                    boost::unique_lock<boost::recursive_mutex> guard(em_event_mutex_);
                    em_event_ = EmergencyEvent::ACCESS_DESIRED_DEPTH_LEVEL2_THRESHOLD;
                    ctrl_state_ = AUVCtrlState::EMERGENCY_LEVEL2;
                }
            }
            if(delta_depth >= 8.0)
            {
                // If current state level is lower than level3 emergency, update event and level
                if(ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL3)
                {
                    boost::unique_lock<boost::recursive_mutex> guard(em_event_mutex_);
                    em_event_ = EmergencyEvent::ACCESS_DESIRED_DEPTH_LEVEL3_THRESHOLD;
                    ctrl_state_ = AUVCtrlState::EMERGENCY_LEVEL3;
                }

            }

            // reset check count
            check_cnt = 0;
        }
        else // inerrupt depth emergency check
        {
            // reset check count
            check_cnt = 0;
        }

        lock.lock();


        if(em_check_dt_ > 0.0){
            ros::Duration sleep_time = (check_start_t + ros::Duration(em_check_dt_)) - ros::Time::now();
            if(sleep_time > ros::Duration(0.0)){
                // if(debug_){
                //     std::lock_guard<std::mutex> guard(print_mutex_);
                //     printf("Control thread is waiting for wake\n");
                // }
                wait_for_wake = true;
                em_depth_check_timer = nh.createTimer(sleep_time, &AUVControllerROS::wakeEMDepthCheckThread, this);
            }
        }
    }
}

/////////////////////////////////////
void AUVControllerROS::emRollCheckThread()
{
    ros::NodeHandle nh;
    ros::Timer em_roll_check_timer;
    bool wait_for_wake = true;

    int check_cnt = 0;

    while(nh.ok())
    {
        boost::unique_lock<boost::recursive_mutex> lock(em_roll_check_mutex_);
        // Wait for wake
        while(wait_for_wake){
            em_roll_check_cond_.wait(lock);
            wait_for_wake = false;                                                       
        }
        lock.unlock();

        ros::Time check_start_t = ros::Time::now();
        double cur_roll = abs(getRoll()) * rad2degree;

        // If depth access the threshold but not access check count, only add check count 
        if(cur_roll >= 25.0 && check_cnt < 2) 
        {
            ++check_cnt; 
        }
        // If depth access the threshold and access check count, check the emergency level 
        else if(cur_roll >= 25.0 && check_cnt == 2)
        {
            if(cur_roll < 30.0)
            {
                // If current state level is lower than level1 emergency, update event and level
                if(ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL1 &&
                   ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL2 &&
                   ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL3)
                {
                    boost::unique_lock<boost::recursive_mutex> guard(em_event_mutex_);
                    em_event_ = EmergencyEvent::ACCESS_ROLL_ANGLE_LEVEL1_THRESHOLD;
                    ctrl_state_ = AUVCtrlState::EMERGENCY_LEVEL1;
                }
            }
            if(cur_roll >= 30.0 && cur_roll < 35.0)
            {
                // If current state level is lower than level2 emergency, update event and level
                if(ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL2 &&
                   ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL3)
                {
                    boost::unique_lock<boost::recursive_mutex> guard(em_event_mutex_);
                    em_event_ = EmergencyEvent::ACCESS_ROLL_ANGLE_LEVEL2_THRESHOLD;
                    ctrl_state_ = AUVCtrlState::EMERGENCY_LEVEL2;
                }
            }
            if(cur_roll >= 35.0)
            {
                // If current state level is lower than level3 emergency, update event and level
                if(ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL3)
                {
                    boost::unique_lock<boost::recursive_mutex> guard(em_event_mutex_);
                    em_event_ = EmergencyEvent::ACCESS_ROLL_ANGLE_LEVEL3_THRESHOLD;
                    ctrl_state_ = AUVCtrlState::EMERGENCY_LEVEL3;
                }

            }

            // reset check count
            check_cnt = 0;
        }
        else // inerrupt depth emergency check
        {
            // reset check count
            check_cnt = 0;
        }

        lock.lock();


        if(em_check_dt_ > 0.0){
            ros::Duration sleep_time = (check_start_t + ros::Duration(em_check_dt_)) - ros::Time::now();
            if(sleep_time > ros::Duration(0.0)){
                // if(debug_){
                //     std::lock_guard<std::mutex> guard(print_mutex_);
                //     printf("Control thread is waiting for wake\n");
                // }
                wait_for_wake = true;
                em_roll_check_timer = nh.createTimer(sleep_time, &AUVControllerROS::wakeEMRollCheckThread, this);
            }
        }
    }
}

/////////////////////////////////////
void AUVControllerROS::emPitchCheckThread()
{
    ros::NodeHandle nh;
    ros::Timer em_pitch_check_timer;
    bool wait_for_wake = true;

    int check_cnt = 0;

    while(nh.ok())
    {
        boost::unique_lock<boost::recursive_mutex> lock(em_pitch_check_mutex_);
        // Wait for wake
        while(wait_for_wake){
            em_pitch_check_cond_.wait(lock);
            wait_for_wake = false;                                                       
        }
        lock.unlock();

        ros::Time check_start_t = ros::Time::now();
        double cur_pitch = abs(getPitch()) * rad2degree;

        // If depth access the threshold but not access check count, only add check count 
        if(cur_pitch >= 20.0 && check_cnt < 2) 
        {
            ++check_cnt; 
        }
        // If depth access the threshold and access check count, check the emergency level 
        else if(cur_pitch >= 20.0 && check_cnt == 2)
        {
            if(cur_pitch < 25.0)
            {
                // If current state level is lower than level1 emergency, update event and level
                if(ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL1 &&
                   ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL2 &&
                   ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL3)
                {
                    boost::unique_lock<boost::recursive_mutex> guard(em_event_mutex_);
                    em_event_ = EmergencyEvent::ACCESS_PITCH_ANGLE_LEVEL1_THRESHOLD;
                    ctrl_state_ = AUVCtrlState::EMERGENCY_LEVEL1;
                }
            }
            if(cur_pitch >= 25.0 && cur_pitch < 30.0)
            {
                // If current state level is lower than level2 emergency, update event and level
                if(ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL2 &&
                   ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL3)
                {
                    boost::unique_lock<boost::recursive_mutex> guard(em_event_mutex_);
                    em_event_ = EmergencyEvent::ACCESS_PITCH_ANGLE_LEVEL2_THRESHOLD;
                    ctrl_state_ = AUVCtrlState::EMERGENCY_LEVEL2;
                }
            }
            if(cur_pitch >= 30.0)
            {
                // If current state level is lower than level3 emergency, update event and level
                if(ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL3)
                {
                    boost::unique_lock<boost::recursive_mutex> guard(em_event_mutex_);
                    em_event_ = EmergencyEvent::ACCESS_PITCH_ANGLE_LEVEL3_THRESHOLD;
                    ctrl_state_ = AUVCtrlState::EMERGENCY_LEVEL3;
                }

            }

            // reset check count
            check_cnt = 0;
        }
        else // inerrupt depth emergency check
        {
            // reset check count
            check_cnt = 0;
        }

        lock.lock();


        if(em_check_dt_ > 0.0){
            ros::Duration sleep_time = (check_start_t + ros::Duration(em_check_dt_)) - ros::Time::now();
            if(sleep_time > ros::Duration(0.0)){
                // if(debug_){
                //     std::lock_guard<std::mutex> guard(print_mutex_);
                //     printf("Control thread is waiting for wake\n");
                // }
                wait_for_wake = true;
                em_pitch_check_timer = nh.createTimer(sleep_time, &AUVControllerROS::wakeEMPitchCheckThread, this);
            }
        }
    }
}

/////////////////////////////////////
void AUVControllerROS::emYawCheckThread()
{
    ros::NodeHandle nh;
    ros::Timer em_yaw_check_timer;
    bool wait_for_wake = true;

    double prev_yaw = getYaw();
    double cur_yaw = prev_yaw;

    while(nh.ok())
    {
        boost::unique_lock<boost::recursive_mutex> lock(em_yaw_check_mutex_);
        // Wait for wake
        while(wait_for_wake){
            em_yaw_check_cond_.wait(lock);
            wait_for_wake = false;                                                       
        }
        lock.unlock();

        ros::Time check_start_t = ros::Time::now();
        double cur_yaw = getYaw();
        double delta_yaw = abs(cur_yaw - prev_yaw) * rad2degree;

        // If depth access the threshold and access check count, check the emergency level 
        if(delta_yaw >= 25.0)
        {
            if(delta_yaw < 30.0)
            {
                // If current state level is lower than level1 emergency, update event and level
                if(ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL1 &&
                   ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL2 &&
                   ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL3)
                {
                    boost::unique_lock<boost::recursive_mutex> guard(em_event_mutex_);
                    em_event_ = EmergencyEvent::YAW_LEVEL1_SALTATION;
                    ctrl_state_ = AUVCtrlState::EMERGENCY_LEVEL1;
                }
            }
            if(delta_yaw >= 30.0 && delta_yaw < 35.0)
            {
                // If current state level is lower than level2 emergency, update event and level
                if(ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL2 &&
                   ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL3)
                {
                    boost::unique_lock<boost::recursive_mutex> guard(em_event_mutex_);
                    em_event_ = EmergencyEvent::YAW_LEVEL2_SALTATION;
                    ctrl_state_ = AUVCtrlState::EMERGENCY_LEVEL2;
                }
            }
            if(delta_yaw >= 35.0)
            {
                // If current state level is lower than level3 emergency, update event and level
                if(ctrl_state_ != AUVCtrlState::EMERGENCY_LEVEL3)
                {
                    boost::unique_lock<boost::recursive_mutex> guard(em_event_mutex_);
                    em_event_ = EmergencyEvent::YAW_LEVEL3_SALTATION;
                    ctrl_state_ = AUVCtrlState::EMERGENCY_LEVEL3;
                }

            }
        }
        prev_yaw = cur_yaw;

        lock.lock();


        if(em_check_dt_ > 0.0){
            ros::Duration sleep_time = (check_start_t + ros::Duration(2 * em_check_dt_)) - ros::Time::now();
            if(sleep_time > ros::Duration(0.0)){
                // if(debug_){
                //     std::lock_guard<std::mutex> guard(print_mutex_);
                //     printf("Control thread is waiting for wake\n");
                // }
                wait_for_wake = true;
                em_yaw_check_timer = nh.createTimer(sleep_time, &AUVControllerROS::wakeEMYawCheckThread, this);
            }
        }
    }
}

/////////////////////////////////////
void AUVControllerROS::applyActuatorInput(double vertfin, double fwdfin, double backfin, double rpm){
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
        vert_rudder_ang_pub_.publish(fins_msg); // test

        fin3_pub_.publish(fins_msg);
        fins_msg.data = -vertfin;
        fin5_pub_.publish(fins_msg);
        // Forward fins
        fins_msg.data = fwdfin;
        front_rudder_ang_pub_.publish(fins_msg); // test

        fin0_pub_.publish(fins_msg);
        fins_msg.data = -fwdfin;
        fin1_pub_.publish(fins_msg);
        // Backward fins
        fins_msg.data = -backfin;
        fin2_pub_.publish(fins_msg);
        fins_msg.data = backfin;
        fin4_pub_.publish(fins_msg);

        back_rudder_ang_pub_.publish(fins_msg); // test
    }
    else
    {
        // Vertical fins
        fins_msg.data = vertfin;
        vert_rudder_ang_pub_.publish(fins_msg);

        fin1_pub_.publish(fins_msg);
        fins_msg.data = -vertfin;
        fin3_pub_.publish(fins_msg);
        // Backward fins
        fins_msg.data = -backfin;
        fin0_pub_.publish(fins_msg);
        fins_msg.data = backfin;
        fin2_pub_.publish(fins_msg);

        back_rudder_ang_pub_.publish(fins_msg); // test

        fins_msg.data = 0.0;
        front_rudder_ang_pub_.publish(fins_msg); // test
    }
}

/////////////////////////////////////
void AUVControllerROS::applyActuatorInput(double upper_p, double upper_s, double lower_p, double lower_s, double rpm)
{

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

/////////////////////////////////////
bool AUVControllerROS::isStable(){
    double cur_depth = -getGlobalZ();
    double cur_y = getGlobalY();
    double cur_pitch = getAngVelPitch();
    double cur_yaw = getAngVelYaw();
    if(abs(depth_d_ - cur_depth) < 0.3 && abs(y_d_ - cur_y) < 0.3 
        && abs(yaw_d_ - cur_yaw) < 0.04363 && abs(pitch_d_ - cur_pitch) < 0.04363)
    {
        return true;
    }
    return false;
}

/////////////////////////////////////
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

/////////////////////////////////////
void AUVControllerROS::pressureCb(const sensor_msgs::FluidPressure::ConstPtr& msg) // For pressure sensor
{
    std::lock_guard<std::mutex> guard(pressure_mutex_);
    depth_ = static_cast<double>((msg->fluid_pressure - 101) / 10.1) - 0.25;
}

/////////////////////////////////////
void AUVControllerROS::posegtCb(const nav_msgs::Odometry::ConstPtr& msg) // For pose sensor
{
    std::lock_guard<std::mutex> guard(posegt_mutex_);
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    z_ = msg->pose.pose.position.z;
}

/////////////////////////////////////
void AUVControllerROS::dvlCb(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg) // For DVL
{
    std::lock_guard<std::mutex> guard(dvl_mutex_);
    u_ = msg->velocity.x;
    v_ = msg->velocity.y;
    w_ = msg->velocity.z;
}

/////////////////////////////////////
void AUVControllerROS::desiredParamshCb(const armsauv_msgs::DesiredParams::ConstPtr& msg)
{
    std::lock_guard<std::mutex> guard(desired_mutex_);
    depth_d_ = msg->depth;
    y_d_ = msg->latdev;
    pitch_d_ = msg->dpitch;
    yaw_d_ = msg->dyaw;
    u_d_ = msg->dlinvelx;
}

/////////////////////////////////////
void AUVControllerROS::statusCb(const std_msgs::Int64::ConstPtr& msg)
{
    switch(msg->data)
    {
    case 0:
        {
            boost::unique_lock<boost::recursive_mutex> lock(ctrl_mutex_);
            ctrl_state_ = AUVCtrlState::STANDBY;
        }
        break;
    case 1:
        {
            boost::unique_lock<boost::recursive_mutex> lock(ctrl_mutex_);
            ctrl_state_ = AUVCtrlState::OPENCTRL;
        }
        break;
    case 2:
        {
            boost::unique_lock<boost::recursive_mutex> lock(ctrl_mutex_);
            ctrl_state_ = AUVCtrlState::CTRL;
        }
        break;
    }
}

/////////////////////////////////////
void AUVControllerROS::printAUVDynamicParams(){
    std::stringstream ss;
    ss << "AUV dynamic parameters: {";
    controller_->serializeAUVDynamicParams(ss);
    ss << "}";
    ROS_INFO_STREAM(ss.str());
}

/////////////////////////////////////
void AUVControllerROS::printAUVCtrlParams(){
    std::stringstream ss;
    ss << "AUV control parameters: {";
    controller_->serializeAUVControlParams(ss);
    ss << "}";
    ROS_INFO_STREAM(ss.str());
}

/////////////////////////////////////
void AUVControllerROS::printAUVForceParams(){
    std::stringstream ss;
    ss << "AUV Force parameters: {";
    controller_->serializeAUVForceParams(ss);
    ss << "}";
    ROS_INFO_STREAM(ss.str());
}

/////////////////////////////////////
void AUVControllerROS::printAUVXForceParams(){
    std::stringstream ss;
    ss << "AUV XForce parameters: {";
    controller_->serializeAUVXForceParams(ss);
    ss << "}";
    ROS_INFO_STREAM(ss.str());
}

/////////////////////////////////////
void AUVControllerROS::printAUVBodyParams(){
    std::stringstream ss;
    ss << "AUV body parameters: {";
    controller_->serializeAUVBodyParams(ss);
    ss << "}";
    ROS_INFO_STREAM(ss.str());
}

/////////////////////////////////////
bool AUVControllerROS::resetCtrlState(auv_controller::ResetCtrlState::Request& req, 
                                      auv_controller::ResetCtrlState::Response& res)
{
    if(req.IsReset == 0)
    {
        // Reset control state to standby
        if(ctrl_state_ != AUVCtrlState::STANDBY)
        {
            boost::unique_lock<boost::recursive_mutex> lock(ctrl_mutex_);
            ctrl_state_ = AUVCtrlState::STANDBY;
            res.FeedbackMsg = "Reset control state to standby!";
        }
        else
        {
            res.FeedbackMsg = "State was aready set to standby!";
        }
        return true;
    }

    if(req.IsReset == 1)
    {
        // Reset control state to open control
        if(ctrl_state_ != AUVCtrlState::OPENCTRL)
        {
            boost::unique_lock<boost::recursive_mutex> lock(ctrl_mutex_);
            ctrl_state_ = AUVCtrlState::OPENCTRL;
            res.FeedbackMsg = "Reset control state to open loop control!";
        }
        else
        {
            res.FeedbackMsg = "State was already set to open loop control!";
        }
        return true;
    }

    if(req.IsReset == 2)
    {
        // Reset control state to open control
        if(ctrl_state_ != AUVCtrlState::CTRL)
        {
            boost::unique_lock<boost::recursive_mutex> lock(ctrl_mutex_);
            ctrl_state_ = AUVCtrlState::CTRL;
            res.FeedbackMsg = "Reset control state to close loop control!";
        }
        else
        {
            res.FeedbackMsg = "State was already set to close loop control!";
        }
        return true;
    }
}
}; // ns
