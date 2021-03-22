/*
 * Filename: auv_traj_follow_manager/AUVTrajFollowManagerROS.h
 * Path: auv_traj_follow_manager
 * Created Date: Thirsday, March 17th 2021, 10:58:05 pm
 * Author: zhao wang
 *
 * Copyright (c) 2021 hust-arms
 */

#ifndef AUV_TRAJ_FOLLOW_MANAGER_ROS_H_
#define AUV_TRAJ_FOLLOW_MANAGER_ROS_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <uuv_sensor_ros_plugins_msgs/DVL.h>
#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>
#include <tf/transform_datatypes.h>

#include "auv_control_msgs/AUVCtrlInfo.h"
#include "auv_controller/ResetCtrlState.h"
#include "auv_controller/AUVControllerExp.h"
#include "auv_controller/SetMissionStatus.h"

/* boost lib */
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/chrono.hpp>
#include <boost/timer.hpp>

namespace auv_controller
{

typedef boost::shared_ptr<AUVControllerExp> AUVControllerExpPtr;

/**
 * @brief AUV trajectory following manager
 */
class AUVTrajFollowManagerROS
{
public:
    /**
     * @brief Constructor
     */
    AUVTrajFollowManagerROS(std::string auv_name, bool with_ff, bool x_type, bool debug);

    /**
     * @brief Deconstructor
     */
    ~AUVTrajFollowManagerROS();

    /**
     * @brief Start threads
     */
    void start();

    /**
     * @brief Service to set mission status 
     */
    bool setMissionStatus(auv_controller::SetMissionStatus::Request& req,
                          auv_controller::SetMissionStatus::Response& res);

private:
    /**
     * @brief Transform string to geometry_msgs::Point vector
     */
    bool strToGeoPointVector(const XmlRpc::XmlRpcValue& xmlrpc, std::vector<geometry_msgs::Point>& geo_vec);

    /**
     * @brief Parse 2D point from string 
     */
    std::vector<std::vector<double>> parseVVD(const std::string& input, std::string& error_return);

    /**
     * @brief Start trajectory follow manage
     */
    void trajFollowManageThread();

    /**
     * @brief Wake control 
     */
    void wakeManageThread(const ros::TimerEvent& event);

    /**
     * @brief Start desired info publishment
     */
    void publishThread();

    /**
     * @brief Publish fin message & thruster message
     */
    void publishActuatorMsgs(double fwdfin, double backfin, double vertfin,
                             double thruster);

    /**
     * @brief Publish fin message & thruster message
     */
    void publishActuatorMsgs(double upper_p, double upper_s, double lower_p, double lower_s,
                             double thruster);

    /**
     * @brief Return mission flag
     */
    bool getMissionFlag()
    {
        boost::unique_lock<boost::recursive_mutex> mission_flag_lock(mission_flag_mutex_);
        // mission_flag_lock.unlock();
        return is_start_mission_;
    }

    /**
     * @brief If vehicle accesses the field of ont way point, update the control information
     */
    void updateCtrlInfo();

    /**
     * @brief If vehicle accesses the end point of waypoint list, reset base property info
     */
    bool isAccessEndPoint();

    /**
     * @brief Return x in global frame
     */
    double getGlobalX()
    {
        boost::unique_lock<boost::recursive_mutex> pose_lock(pose_mutex_);
        return x_;
    }

    /**
     * @brief Return y in global frame
     */
    double getGlobalY()
    {
        boost::unique_lock<boost::recursive_mutex> pose_lock(pose_mutex_);
        return y_;
    }

    /**
     * @brief Return z in global frame
     */
    double getGlobalZ()
    {
        boost::unique_lock<boost::recursive_mutex> pose_lock(pose_mutex_);
        return z_;
    }

    /**
     * @brief Return x linear velocity in vehicle frame
     */
    double getLinVelX()
    {
        boost::unique_lock<boost::recursive_mutex> pose_lock(pose_mutex_);
        return u_;
    }

    /**
     * @brief Get vehicle pose 
     */
    void getPosition(double& x, double& y, double& z)
    {
        boost::unique_lock<boost::recursive_mutex> pose_lock(pose_mutex_);
        x = x_; y = y_; z = z_;
    }

    /**
     * @biref Get vehicle status
     */
    void getVehicleStatus(double& x, double& y, double& z, double& roll, double& pitch, double& yaw,
                    double& u, double& v, double& w, double& roll_dot, double& pitch_dot, double& yaw_dot)
    {
        boost::unique_lock<boost::recursive_mutex> pose_lock(pose_mutex_);
        x = x_; y = y_; z = z_;
        roll = roll_; pitch = pitch_; yaw = yaw_;
        u = u_; v = v_; w = w_;
        roll_dot = roll_dot_; pitch_dot = pitch_dot_; yaw_dot = yaw_dot_;
    }

    /**
     * @brief Get index of current followed way point 
     */
    int getWayPointIndex()
    {
        boost::unique_lock<boost::recursive_mutex> wp_index_lock(wp_index_mutex_);
        return wp_index_;
    }
  
    /**
     * @brief Position & pose input
     */
    void posegtCb(const nav_msgs::Odometry::ConstPtr& msg){
        boost::unique_lock<boost::recursive_mutex> posegt_lock(posegt_mutex_);
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        z_ = msg->pose.pose.position.z;
    }

    /* Sensors callback func */
    /**
     * @brief IMU data input
     */
    void imuCb(const sensor_msgs::Imu::ConstPtr& msg)
    {
        // PoseStamped::Quaternion to tf::Quaternion
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg->orientation, quat);
        
        // Quaternion to RPY
        boost::unique_lock<boost::recursive_mutex> imu_lock(imu_mutex_);
        tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);
        
        roll_dot_ = msg->angular_velocity.x;
        pitch_dot_ =  msg->angular_velocity.y;
        yaw_dot_ = msg->angular_velocity.z;
    }

    /**
     * @brief Pressure sensor data input
     */
    void pressureCb(const sensor_msgs::FluidPressure::ConstPtr& msg)
    {
        boost::unique_lock<boost::recursive_mutex> pressure_lock(pressure_mutex_);
        depth_ = static_cast<double>((msg->fluid_pressure - 101) / 10.1) - 0.25;
    }

    /**
     * @brief DVL input
     */
    void dvlCb(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg)
    {
        boost::unique_lock<boost::recursive_mutex> dvl_lock(dvl_mutex_);
        u_ = msg->velocity.x;
        v_ = msg->velocity.y;
        w_ = msg->velocity.z;
    }


private:
    /* ros components */
    ros::Publisher auv_control_info_pub_;
    ros::ServiceClient ctrl_state_reset_cl_;

    ros::Publisher fin0_pub_, fin1_pub_, fin2_pub_, fin3_pub_, fin4_pub_, fin5_pub_;
    ros::Publisher thruster0_pub_;

    // should be replaced by TCP client
    ros::Subscriber imu_sub_, pressure_sub_, pose_sub_, dvl_sub_;

    /* controller */
    AUVControllerExpPtr controller_ptr_;

    /* base property */
    std::string base_frame_;
    bool with_ff_, x_type_;
    double manage_dt_;
    XmlRpc::XmlRpcValue wp_xmlrpc_;
    int seq_;

    /* vehicle states */
    double x_, y_, z_, depth_;
    double u_, v_, w_;
    int wp_index_;
    double thre_;

    double roll_, pitch_, yaw_;
    double roll_dot_, pitch_dot_, yaw_dot_;

    /* desired vehicle states */
    double depth_d_, pitch_d_, yaw_d_;
    double x_d_, y_d_, u_d_;

    /* threads */
    boost::thread* manage_th_;
    boost::thread* pub_th_;
    boost::condition_variable_any manage_cond_;

    /* target waypoints */
    std::vector<geometry_msgs::Point> wp_vec_;

    /* resources mutex */
    boost::recursive_mutex print_mutex_;
    boost::recursive_mutex desired_info_mutex_;
    boost::recursive_mutex mission_flag_mutex_;
    boost::recursive_mutex wp_index_mutex_;

    boost::recursive_mutex manage_mutex_;

    boost::recursive_mutex pose_mutex_;
    boost::recursive_mutex posegt_mutex_;
    boost::recursive_mutex imu_mutex_;
    boost::recursive_mutex pressure_mutex_;
    boost::recursive_mutex dvl_mutex_;

    /* flags */
    bool is_start_mission_;

    /* debug */
    bool debug_;
}; // class AUVTrajFollowManagerROS

}; // ns

#endif
