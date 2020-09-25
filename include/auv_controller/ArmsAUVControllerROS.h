/*
 * Filename: auv_controller/AUVControllerROS.h
 * Path: auv_controller
 * Created Date: Wednesday, September 9th 2020, 9:24:05 am
 * Author: zhao wang
 * 
 * Copyright (c) 2020 Your Company
 */
#ifndef ARMSAUV_CONTROLLER_ROS_H_
#define ARMSAUV_CONTROLLER_ROS_H_

#include <thread>
#include <mutex>
#include <string>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/chrono.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Odometry.h>
#include <uuv_sensor_ros_plugins_msgs/DVL.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
#include <tf/transform_datatypes.h>

#include "AUVController.h"

namespace auv_controller{
    /**
     * @brief AUVController ROS wrapper for armsauv
     */
    class ArmsAUVControllerROS{
    public:
        /**
         * @brief Constructor
         */
        ArmsAUVControllerROS(std::string name);

        /**
         * @brief Deconstructor
         */ 
        ~ArmsAUVControllerROS();

    private:
        /**
         * @brief Timer callback
         */
        void timerCb(const ros::TimerEvent& event);

	/**
	 * @brief Control thread
	 */
	void controlThread();

        /**
         * @brief Apply controller output
         */ 
        void applyActuatorInput(double rouder, double fwdfin, double aftfin, double rpm);

        /**
         * @brief Sensor callback
         */
        void imuCb(const sensor_msgs::Imu::ConstPtr& msg); // For IMU
        void pressureCb(const sensor_msgs::FluidPressure::ConstPtr& msg); // For pressure sensor
        void posegtCb(const nav_msgs::Odometry::ConstPtr& msg); // For pose sensor
        void dvlCb(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg); // For DVL

        /**
         * @brief Input callback
         */ 
        void depthCb(const std_msgs::Float64::ConstPtr& msg);
        void pitchCb(const std_msgs::Float64::ConstPtr& msg);
        void yawCb(const std_msgs::Float64::ConstPtr& msg);
        void xdCb(const std_msgs::Float64::ConstPtr& msg);
        void ydCb(const std_msgs::Float64::ConstPtr& msg);

        /**
         * @brief Return x
         */ 
        double getX(){
            std::lock_guard<std::mutex> guard(posegt_mutex_);
            return x_;
        }

        /**
         * @brief Return y
         */ 
        double getY(){
            std::lock_guard<std::mutex> guard(posegt_mutex_);
            return y_;
        }

        /**
         * @brief Return z
         */
        double getZ(){
            std::lock_guard<std::mutex> guard(posegt_mutex_);
            return z_;
        }

        /**
         * @brief Return x linear velocity
         */ 
        double getXVelocity(){
            std::lock_guard<std::mutex> guard(dvl_mutex_);
            return x_dot_;
        }

        /**
         * @brief Return x linear velocity
         */ 
        double getYVelocity(){
            std::lock_guard<std::mutex> guard(dvl_mutex_);
            return y_dot_;
        }

        /**
         * @brief Return z linear velocity
         */ 
        double getZVelocity(){
            std::lock_guard<std::mutex> guard(dvl_mutex_);
            return z_dot_;
        }

        /**
         * @brief Return roll
         */ 
        double getRoll(){
            std::lock_guard<std::mutex> guard(imu_mutex_);
            return roll_;
        }

        /**
         * @brief Return pitch
         */ 
        double getPitch(){
            std::lock_guard<std::mutex> guard(imu_mutex_);
            return pitch_;
        }
        
        /**
         * @brief Return yaw
         */ 
        double getYaw(){
            std::lock_guard<std::mutex> guard(imu_mutex_);
            return yaw_;
        }

        /**
         * @brief Return roll angular velocity
         */ 
        double getRollVelocity(){
            std::lock_guard<std::mutex> guard(imu_mutex_);
            return roll_dot_;
        }

        /**
         * @brief Return pitch angular velocity
         */ 
        double getPitchVelocity(){
            std::lock_guard<std::mutex> guard(imu_mutex_);
            return pitch_dot_;
        }

        /**
         * @brief Return yaw angular velocity
         */ 
        double getYawVelocity(){
            std::lock_guard<std::mutex> guard(imu_mutex_);
            return yaw_dot_;
        }

        /**
         * @brief Return depth
         */ 
        double getDepth(){
            std::lock_guard<std::mutex> guard(pressure_mutex_);
            return depth_;
        }

        /**
         * @brief Return depth input
         */ 
        double getDepthInput(){
            std::lock_guard<std::mutex> guard(depth_mutex_);
            return depth_d_;
        }

        /**
         * @brief Return pitch input
         */ 
        double getPitchInput(){
            std::lock_guard<std::mutex> guard(pitch_mutex_);
            return pitch_d_;
        }

        /**
         * @brief Return yaw input
         */ 
        double getYawInput(){
            std::lock_guard<std::mutex> guard(yaw_mutex_);
            return yaw_d_;
        }

        /**
         * @brief Return x coordinate of desired point
         */ 
        double getXInput(){
            std::lock_guard<std::mutex> guard(x_d_mutex_);
            return x_d_;
        }

        /**
         * @brief Return y coordinate of desired point
         */ 
        double getYInput(){
            std::lock_guard<std::mutex> guard(y_d_mutex_);
            return y_d_;
        }
        
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
         * @brief Print auv body parameters on the console
         */ 
        void printAUVBodyParams();

    private:
        AUVController* controller_;

        ros::Publisher fin0_pub_, fin1_pub_, fin2_pub_, fin3_pub_, fin4_pub_, fin5_pub_; // fins input
        ros::Publisher thruster0_pub_; // thruster input

        ros::Subscriber imu_sub_, pressure_sub_, posegt_sub_, dvl_sub_; // sensors messages sub
        ros::Subscriber depth_sub_, pitch_sub_, yaw_sub_; // desired parameters sub
        ros::Subscriber x_d_sub_, y_d_sub_;

        double dt_; // control period
        int rpm_; // rotate per minutes of thruster
        uint32_t seq_; // header sequence
        std::string base_frame_; // base frame name of AUV

        double x_, y_, z_; // position
        double x_dot_, y_dot_, z_dot_; // linear velocity
        double roll_, pitch_, yaw_; // pose
        double roll_dot_, pitch_dot_, yaw_dot_; // angular velocity

        double depth_;

        double depth_d_, pitch_d_, yaw_d_; // desired parameters
        double x_d_, y_d_;

	// Control thread
        boost::thread* ctrl_thread_;

	// Source lock
        std::mutex imu_mutex_, pressure_mutex_, posegt_mutex_, dvl_mutex_;
        std::mutex depth_mutex_, pitch_mutex_, yaw_mutex_;
        std::mutex x_d_mutex_, y_d_mutex_;

    }; // AUVControllerROS
}; // ns

#endif
