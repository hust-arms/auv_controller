/*
 * Filename: auv_controller/AUVControllerROS.h
 * Path: auv_controller
 * Created Date: Friday, Janurary 29th 2021, 10:58:05 am
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */
#ifndef AUV_CONTROLLER_ROS_H_
#define AUV_CONTROLLER_ROS_H_

#include <thread>
#include <mutex>
#include <string>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/chrono.hpp>
#include <boost/timer.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition.hpp>

/* ros dependencies */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Odometry.h>
#include <uuv_sensor_ros_plugins_msgs/DVL.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
#include <tf/transform_datatypes.h>

#include <armsauv_msgs/DesiredParams.h>

#include "AUVBaseController.h"
#include "AUVControllerWithFF.h"
#include "AUVControllerNoFF.h"
#include "AUVControllerXF.h"
#include "file_writer.h"
#include "PIDController.h"

namespace auv_controller{
/**
 * @brief AUV controller ROS wrapper class
 */
class AUVControllerROS{
public:
    /**
     * @brief Default constructor
     */ 
    AUVControllerROS(std::string name, bool with_ff, bool x_type, bool debug);

    /**
     * @brief Deconstructor
     */
    ~AUVControllerROS();

    /**
     * @brief Start controll
     */
    void startControl();

private:
    /**
     * @brief Control thread
     */
    void controlThread();
    
    /**
     * @brief Control thread
     */
    void velControlThread();
    
    /**
     * @brief Publish thread
     */ 
    void publishThread();

    /**
     * @brief Wake control thread
     */
    void wakeControlThread(const ros::TimerEvent& event);

    /**
     * @brief Apply actuator input for model without front fins and model with front fins 
     */
    void applyActuatorInput(double rudder, double fwdfin, double backfin, double rpm);

    /**
     * @brief Apply actuator input for model with X type fins
     */ 
    void applyActuatorInput(double upper_p, double upper_s, double lower_p, double lower_s, double rpm);

    /**
     * @brief Return true if status of AUV is stable
     */
    bool isStable();

    /* Desired params interface */
    /**
     * @brief Return desired depth
     */
    double getDesiredDepth(){
        std::lock_guard<std::mutex> guard(desired_mutex_);
        return depth_d_;
    }

    /**
     * @brief Return desired pitch
     */
    double getDesiredPitch(){
        std::lock_guard<std::mutex> guard(desired_mutex_);
        return pitch_d_;
    }
    
    /**
     * @brief Return desired lateral deviation
     */
    double getDesiredY(){
        std::lock_guard<std::mutex> guard(desired_mutex_);
        return y_d_;
    }
    
    /**
     * @brief Return desired depth
     */
    double getDesiredYaw(){
        std::lock_guard<std::mutex> guard(desired_mutex_);
        return yaw_d_;
    }

    /**
     * @brief Return desired x linear velocity
     */
    double getDesiredLinVelX(){
        std::lock_guard<std::mutex> guard(desired_mutex_);
        return u_d_;
    }
    
    /* States output interface */
    /**
     * @brief Return x in global frame
     */
    double getGlobalX(){
        std::lock_guard<std::mutex> guard(posegt_mutex_);
        return x_;
    }

    /**
     * @brief Return y in global frame
     */
    double getGlobalY(){
        std::lock_guard<std::mutex> guard(posegt_mutex_);
        return y_;
    }

    /**
     * @brief Return z in global frame
     */
    double getGlobalZ(){
        std::lock_guard<std::mutex> guard(posegt_mutex_);
        return z_;
    }

    /**
     * @brief Return x linear velocity in vehicle frame
     */
    double getLinVelX(){
        std::lock_guard<std::mutex> guard(dvl_mutex_);
        return u_;
    }

    /**
     * @brief Return y linear velocity in vehicle frame
     */
    double getLinVelY(){
        std::lock_guard<std::mutex> guard(dvl_mutex_);
        return v_;
    }
    
    /**
     * @brief Return x linear velocity in vehicle frame
     */
    double getLinVelZ(){
        std::lock_guard<std::mutex> guard(dvl_mutex_);
        return w_;
    }

    /**
     * @brief Return roll of vehicle
     */
    double getRoll(){
        std::lock_guard<std::mutex> guard(imu_mutex_);
        return roll_;
    }

    /**
     * @brief Return pitch of vehicle
     */
    double getPitch(){
        std::lock_guard<std::mutex> guard(imu_mutex_);
        return pitch_;
    }

    /**
     * @brief Return yaw of vehicle
     */
    double getYaw(){
        std::lock_guard<std::mutex> guard(imu_mutex_);
        return yaw_;
    }

    /**
     * @brief Return roll angular velocity
     */
    double getAngVelRoll(){
        std::lock_guard<std::mutex> guard(imu_mutex_);
        return roll_dot_;
    }
    
    /**
     * @brief Return pitch angular velocity
     */
    double getAngVelPitch(){
        std::lock_guard<std::mutex> guard(imu_mutex_);
        return pitch_dot_;
    }
    
    /**
     * @brief Return yaw angular velocity
     */
    double getAngVelYaw(){
        std::lock_guard<std::mutex> guard(imu_mutex_);
        return yaw_dot_;
    }

    /* Sensors callback func */
    /**
     * @brief IMU data input  
     */ 
    void imuCb(const sensor_msgs::Imu::ConstPtr& msg);

    /**
     * @brief Pressure sensor data input  
     */ 
    void pressureCb(const sensor_msgs::FluidPressure::ConstPtr& msg);

    /**
     * @brief Position & pose input  
     */ 
    void posegtCb(const nav_msgs::Odometry::ConstPtr& msg); 

    /**
     * @brief DVL input  
     */ 
    void dvlCb(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg); 

    /**
     * @brief Desired input 
     */
    void desiredParamshCb(const armsauv_msgs::DesiredParams::ConstPtr& msg);

    /* Debug interface */
    /**
     * @brief Print auv dynamic parameters on the console
     */
    void printAUVDynamicParams();
    
    /**
     * @brief Print auv control parameters on the console
     */
    void printAUVCtrlParams();
    
    /**
     * @brief Print auv force parameters on the console
     */
    void printAUVForceParams();
    
    /**
     * @brief Print auv xforce parameters on the console
     */
    void printAUVXForceParams();
    
    /**
     * @brief Print auv body parameters on the console
     */
    void printAUVBodyParams();

private:
    /**
     * @brief AUV controlling state
     */
    enum class AUVCtrlState{
        STANDBY,
        OPENCTRL,
        CTRL,
        EMERGENCY
    }; // AUVCtrlState

private:
    /* Controller */
    AUVBaseController* controller_;
    PIDControllerPtr u_controller_;

    /* ROS components */
    ros::Publisher fin0_pub_, fin1_pub_, fin2_pub_, fin3_pub_, fin4_pub_, fin5_pub_;
    ros::Publisher front_rudder_ang_pub_, back_rudder_ang_pub_, vert_rudder_ang_pub_;
    ros::Publisher thruster0_pub_; 

    ros::Subscriber imu_sub_, pressure_sub_, posegt_sub_, dvl_sub_; // sensors messages sub
    ros::Subscriber desired_sub_;

    ros::Time last_valid_ctrl_, ctrl_time_;
    ros::Time last_stable_;
    
    double ctrl_dt_; // control period
    double pub_dt_; // publish period
    double stable_wait_t_; // time to wait after vehicle is stable

    uint32_t seq_;
    std::string base_frame_; // base frame of vehicle

    AUVCtrlState ctrl_state_;
    bool is_ctrl_vel_, is_wait_stable_;

    /* Vehicle states */
    double x_, y_, z_; // postiion in global frame
    double depth_;

    double u_, v_, w_; // linear velocity in vehicle frame
    double roll_, pitch_, yaw_; // pose in vehicle frame
    double roll_dot_, pitch_dot_, yaw_dot_; // angular velocity in vehicle frame

    double depth_d_, pitch_d_, y_d_, yaw_d_; // desired params
    double u_d_;

    /* Controll var */
    double fwdfin_, backfin_, vertfin_;
    double upper_p_, upper_s_, lower_p_, lower_s_; // for X type rudder
    double rpm_, ori_rpm_;

    /* Threads */
    boost::thread* ctrl_thread_; // thread for slide model control algorithm
    boost::thread* pub_thread_;  
    boost::thread* vel_ctrl_thread_;  

    /* condition var */
    boost::condition_variable_any ctrl_cond_;

    /* Source lock for thread */
    boost::recursive_mutex ctrl_mutex_;

    /* Source lock of sensor */
    std::mutex imu_mutex_, pressure_mutex_, posegt_mutex_, dvl_mutex_;

    /* Source lock for desired params */
    std::mutex desired_mutex_; 

    /* Source lock for control variable */
    std::mutex ctrl_var_mutex_, ctrl_vel_mutex_;

    /* Source lock for print */
    std::mutex print_mutex_;

    /* flags */
    bool with_ff_, x_type_;
    bool is_ctrl_run_, is_emerg_run_;
    bool debug_;
}; // AUVControllerROS
}; // ns

#endif

