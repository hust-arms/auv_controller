/*                                                                                               
 * Filename: auv_controller/AUVCtrlMsgsRecorderROS.h
 * Path: auv_controller
 * Created Date: Thirsday, March 24th 2021, 10:58:05 am
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#ifndef AUV_CTRLMSGS_RECORDER_ROS_H_
#define AUV_CTRLMSGS_RECORDER_ROS_H_

#include "AUVCtrlMsgsRecorder.h"

/* ros dependencies */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Odometry.h>
#include <uuv_sensor_ros_plugins_msgs/DVL.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>

#include <tf/transform_datatypes.h>

#include "auv_control_msgs/AUVCtrlInfo.h"
#include "auv_control_msgs/AUVCtrlDeviation.h"

#include <boost/thread.hpp>

namespace auv_controller
{
typedef boost::shared_ptr<AUVCtrlMsgsRecorder> AUVCtrlMsgsRecorderPtr; 

class AUVCtrlMsgsRecorderROS
{
public:
    AUVCtrlMsgsRecorderROS();

    ~AUVCtrlMsgsRecorderROS();

    void startRecord();

private:
    /**
     * @brief Data writing thread
     */
    void recordThread();

    /**
     * @brief Receive AUV odometry
     * @msg AUV odometry
     */ 
    void auvOdometryCb(const nav_msgs::Odometry::ConstPtr& msg);

    /**
     * @brief Return pose
     */
    void getPosition(double& x, double& y, double& z); 

    /**
     * @brief Receive imu message
     * @msg AUV imu message
     */
    void imuCb(const sensor_msgs::Imu::ConstPtr& msg);

    /**
     * @brief Return pose
     */
    void getPose(double& roll, double& pitch, double& yaw, 
                 double& roll_v, double& pitch_v, double& yaw_v);

    /**
     * @brief Receive dvl message
     * @msg AUV dvl message
     */
    void dvlCb(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg);

    /**
     * @brief Return linear velocity
     */
    void getLinVel(double& x_v, double& y_v, double& z_v); 

    /**
     * @brief Receive AUV rotor speed
     * @msg AUV rotor speed 
     */ 
    void rotorSpeedCb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg);

    /**
     * @brief Return rotor speed
     */
    double getRotorSpeed();

    /**
     * @brief Receive AUV odometry
     * @msg Forward left fin angle 
     */ 
    void forwardLeftFinCb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg);

    /**
     * @brief Return forward left fin angle
     */
    double getFwdLeftFin();

    /**
     * @brief Receive AUV odometry
     * @msg Forward right fin angle 
     */ 
    void forwardRightFinCb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg);

    /**
     * @brief Return forward right fin angle
     */
    double getFwdRightFin();

    /**
     * @brief Receive AUV odometry
     * @msg Back left fin angle 
     */ 
    void backLeftFinCb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg);

    /**
     * @brief Return back left fin angle
     */
    double getBackLeftFin();

    /**
     * @brief Receive AUV odometry
     * @msg Back right fin angle 
     */ 
    void backRightFinCb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg);

    /**
     * @brief Return back right fin angle
     */
    double getBackRightFin();

    /**
     * @brief Receive AUV odometry
     * @msg Vertical upper fin angle 
     */ 
    void vertUpperFinCb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg);

    /**
     * @brief Return vertical upper fin angle
     */
    double getVertUpperFin();

    /**
     * @brief Receive AUV odometry
     * @msg Vertical lower fin angle 
     */ 
    void vertLowerFinCb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg);
    
    /**
     * @brief Return vertical lower fin angle
     */
    double getVertLowerFin();

    /**
     * @brief Receive AUV control target data
     * @msg Control information
     */
    void auvCtrlInfoCb(const auv_control_msgs::AUVCtrlInfo::ConstPtr& msg);

    /**
     * @brief Return control informations
     */
    void getCtrlInfo(double& x_d, double& y_d, double& depth_d, 
                     double& pitch_d, double& yaw_d, double& u_d);

    /**
     * @brief Receive AUV control deviation
     * @msg Control information
     */
    void auvCtrlDeviationCb(const auv_control_msgs::AUVCtrlDeviation::ConstPtr& msg);

    /**
     * @brief Return AUV control deviation
     */
    void getCtrlDeviation(double& depthdev, double& latdistdev, double& yawdev, double& pitchdev);

private:
    AUVCtrlMsgsRecorderPtr ctrl_msgs_recorder_;

    std::string auv_name_;
    std::string filename_, path_;
    int record_freq_;
    bool with_ff_, x_type_, debug_;
    int mission_;

    ros::Time time0_;

    boost::thread* record_th_;

    /* data to record */
    // Time period
    double ptime_;

    // AUV pose
    double x_, y_, z_, depth_;
    double u_, v_, w_;
    double roll_, pitch_, yaw_;
    double roll_vel_, pitch_vel_, yaw_vel_;

    // AUV control information
    double x_d_, y_d_, depth_d_, pitch_d_, yaw_d_, u_d_;
    double fwd_l_fin_, fwd_r_fin_, back_l_fin_, back_r_fin_, vert_up_fin_, vert_lo_fin_;
    double rpm_;
    double latdist_dev_, depth_dev_, yaw_dev_, pitch_dev_;

    /* ros components */
    ros::Subscriber auv_odom_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber dvl_sub_;
    ros::Subscriber rotor_speed_sub_;
    ros::Subscriber fwd_l_fin_sub_;
    ros::Subscriber fwd_r_fin_sub_;
    ros::Subscriber back_l_fin_sub_;
    ros::Subscriber back_r_fin_sub_;
    ros::Subscriber vert_up_fin_sub_;
    ros::Subscriber vert_lo_fin_sub_;
    ros::Subscriber auv_ctrl_info_sub_;
    ros::Subscriber auv_ctrl_dev_sub_;

    /* resource mutex */
    boost::recursive_mutex print_mutex_;

    boost::recursive_mutex auv_odom_mutex_;
    boost::recursive_mutex imu_mutex_;
    boost::recursive_mutex dvl_mutex_;
    boost::recursive_mutex rotor_speed_mutex_;
    boost::recursive_mutex fwd_l_fin_mutex_;
    boost::recursive_mutex fwd_r_fin_mutex_;
    boost::recursive_mutex back_l_fin_mutex_;
    boost::recursive_mutex back_r_fin_mutex_;
    boost::recursive_mutex vert_up_fin_mutex_;
    boost::recursive_mutex vert_lo_fin_mutex_;
    boost::recursive_mutex auv_ctrl_info_mutex_;
    boost::recursive_mutex auv_ctrl_dev_mutex_;

}; // class AUVCtrlMsgsRecorderROS
}; // ns

#endif
