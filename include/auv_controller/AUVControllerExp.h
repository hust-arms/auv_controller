/*
 * Filename: auv_controller/AUVControllerExp.h
 * Path: auv_controller
 * Created Date: Saturday, March 20th 2021, 10:58:05 am
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */
#ifndef AUV_CONTROLLER_EXP_H_
#define AUV_CONTROLLER_EXP_H_

#include <thread>
#include <mutex>
#include <string>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/chrono.hpp>
#include <boost/timer.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/asio.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/make_unique.hpp>

#include <armsauv_msgs/DesiredParams.h>

#include "AUVBaseController.h"
#include "AUVControllerWithFF.h"
#include "AUVControllerNoFF.h"
#include "AUVControllerXF.h"
#include "file_writer.h"
#include "PIDController.h"

#include "auv_controller/ResetCtrlState.h"

namespace auv_controller{

// typedef boost::unique_ptr<boost::thread> ThreadPtr;
//
typedef boost::shared_ptr<boost::asio::deadline_timer> DeadlineTimerPtr;

/**
 * @brief AUV controller ROS wrapper class
 */
class AUVControllerExp{
public:
    /**
     * @brief Default constructor
     */ 
    AUVControllerExp(std::string name, bool with_ff, bool x_type, bool debug);

    /**
     * @brief Deconstructor
     */
    ~AUVControllerExp();

    /**
     * @brief Start controll
     */
    void startControl();

    /**
     * @brief Get control var
     */
    void getCtrlVar(double& fwdfin, double& backfin, double& vertfin, double& thruster);

    /**
     * @brief Get control var
     */
    void getCtrlVar(double& upper_p, double& upper_s, double& lower_p, double& lower_s, 
                    double& thruster);

    /**
     * @brief Update pose and position
     */
    void updatePose(double x, double y, double z, double roll, double pitch, double yaw,
                    double u, double v, double w, double roll_dot, double pitch_dot, double yaw_dot);

    /**
     * @brief Get pose and position
     */
    // void getPose(double& x, double& y, double& z, double& roll, double& pitch, double& yaw,
    //              double& u, double& v, double& w, double& roll_dot, double& pitch_dot, double& yaw_dot);

    /**
     * @brief Update control information
     */
    void updateCtrlInfo(double x_d, double y_d, double depth_d, double yaw_d, double pitch_d, double u_d);

    /**
     * @brief Set controller status
     */
    void setCtrlStatus(const int flag)
    {
        switch(flag)
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

    /**
     * @brief 
     */
    // void getCtrlInfo(double& x_d, double& y_d, double& depth_d, double& yaw_d, double& pitch_d, double& u_d);

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

    // /**
    //  * @brief Emergency event check
    //  */ 
    // void emEventCheckThread(const boost::system::error_code& ec);

    /**
     * @brief Thread to wake control
     */
    void wakeControlThread(double sleep_time)
    {
        boost::asio::io_service io;
        boost::asio::deadline_timer ctrl_timer(io, boost::posix_time::seconds(sleep_time));
        ctrl_timer.expires_from_now(boost::posix_time::seconds(sleep_time));
        ctrl_timer.async_wait(boost::bind(&AUVControllerExp::wakeControl, this, _1));
        // ctrl_timer.wait();
        // ctrl_cond_.notify_one();
        // printf("sync wait\n");
        io.run();
    }

    /**
     * @brief Wake control thread
     */
    void wakeControl(const boost::system::error_code& ec);
    
    /**
     * @brief Depth emergency check thread
     */ 
    void emDepthCheckThread();

    /**
     * @brief Roll emergency check thread
     */ 
    void emRollCheckThread();

    /**
     * @brief Pitch emergency check thread
     */ 
    void emPitchCheckThread();

    /**
     * @brief Yaw emergency check thread
     */ 
    void emYawCheckThread();

    /**
     * @brief Thread to wake depth emergency checking thread 
     */
    void wakeEMDepthCheckThread(double sleep_time)
    {
        boost::asio::io_service io;
        boost::asio::deadline_timer ctrl_timer(io, boost::posix_time::seconds(sleep_time));
        ctrl_timer.expires_from_now(boost::posix_time::seconds(sleep_time));
        ctrl_timer.async_wait(boost::bind(&AUVControllerExp::wakeEMDepthCheck, this, _1));
        io.run();
    }

    /**
     * @brief Wake depth emergency check thread
     */ 
    void wakeEMDepthCheck(const boost::system::error_code& ec);

    /**
     * @brief Thread to wake roll emergency checking thread 
     */
    void wakeEMRollCheckThread(double sleep_time)
    {
        boost::asio::io_service io;
        boost::asio::deadline_timer ctrl_timer(io, boost::posix_time::seconds(sleep_time));
        ctrl_timer.expires_from_now(boost::posix_time::seconds(sleep_time));
        ctrl_timer.async_wait(boost::bind(&AUVControllerExp::wakeEMRollCheck, this, _1));
        io.run();
    }

    /**
     * @brief Roll emergency check thread
     */
    void wakeEMRollCheck(const boost::system::error_code& ec);

    /**
     * @brief Thread to wake Pitch emergency checking thread 
     */
    void wakeEMPitchCheckThread(double sleep_time)
    {
        boost::asio::io_service io;
        boost::asio::deadline_timer ctrl_timer(io, boost::posix_time::seconds(sleep_time));
        ctrl_timer.expires_from_now(boost::posix_time::seconds(sleep_time));
        ctrl_timer.async_wait(boost::bind(&AUVControllerExp::wakeEMPitchCheck, this, _1));
        io.run();
    }

    /**
     * @brief Roll emergency check thread
     */
    void wakeEMPitchCheck(const boost::system::error_code& ec);
    
    /**
     * @brief Thread to wake Yaw emergency checking thread 
     */
    void wakeEMYawCheckThread(double sleep_time)
    {
        boost::asio::io_service io;
        boost::asio::deadline_timer ctrl_timer(io, boost::posix_time::seconds(sleep_time));
        ctrl_timer.expires_from_now(boost::posix_time::seconds(sleep_time));
        ctrl_timer.async_wait(boost::bind(&AUVControllerExp::wakeEMYawCheck, this, _1));
        io.run();
    }

    /**
     * @brief Yaw emergency check thread
     */
    void wakeEMYawCheck(const boost::system::error_code& ec);

    /**
     * @brief Return true if status of AUV is stable
     */
    bool isStable();

    /**
     * @brief Return desired depth
     */
    double getDesiredDepth(){
        std::lock_guard<std::mutex> guard(ctrl_info_mutex_);
        return depth_d_;
    }

    /**
     * @brief Return desired pitch
     */
    double getDesiredPitch(){
        std::lock_guard<std::mutex> guard(ctrl_info_mutex_);
        return pitch_d_;
    }

    /**
     * @brief Return desired lateral deviation
     */
    double getDesiredX()
    {
        std::lock_guard<std::mutex> guard(ctrl_info_mutex_);
        return x_d_;
    }
    
    /**
     * @brief Return desired lateral deviation
     */
    double getDesiredY(){
        std::lock_guard<std::mutex> guard(ctrl_info_mutex_);
        return y_d_;
    }

    /**
     * @brief Return desired depth
     */
    double getDesiredYaw(){
        std::lock_guard<std::mutex> guard(ctrl_info_mutex_);
        return yaw_d_;
    }
    
    /**
     * @brief Return desired x linear velocity
     */
    double getDesiredLinVelX(){
        std::lock_guard<std::mutex> guard(ctrl_info_mutex_);
        return u_d_;
    }

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
        std::lock_guard<std::mutex> guard(posegt_mutex_);
        return u_;
    }
    
    /**
     * @brief Return y linear velocity in vehicle frame
     */
    double getLinVelY(){
        std::lock_guard<std::mutex> guard(posegt_mutex_);
        return v_;
    }
    
    /**
     * @brief Return x linear velocity in vehicle frame
     */
    double getLinVelZ(){
        std::lock_guard<std::mutex> guard(posegt_mutex_);
        return w_;
    }

    /**
     * @brief Return roll of vehicle
     */
    double getRoll(){
        std::lock_guard<std::mutex> guard(posegt_mutex_);
        return roll_;
    }
    
    /**
     * @brief Return pitch of vehicle
     */
    double getPitch(){
        std::lock_guard<std::mutex> guard(posegt_mutex_);
        return pitch_;
    }

    /**
     * @brief Return yaw of vehicle
     */
    double getYaw(){
        std::lock_guard<std::mutex> guard(posegt_mutex_);
        return yaw_;
    }
    
    /**
     * @brief Return roll angular velocity
     */
    double getAngVelRoll(){
        std::lock_guard<std::mutex> guard(posegt_mutex_);
        return roll_dot_;
    }

    /**
     * @brief Return pitch angular velocity
     */
    double getAngVelPitch(){
        std::lock_guard<std::mutex> guard(posegt_mutex_);
        return pitch_dot_;
    }
    
    /**
     * @brief Return yaw angular velocity
     */
    double getAngVelYaw(){
        std::lock_guard<std::mutex> guard(posegt_mutex_);
        return yaw_dot_;
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
        EMERGENCY_LEVEL1,
        EMERGENCY_LEVEL2,
        EMERGENCY_LEVEL3
    }; // AUVCtrlState

    /**
     * @brief AUV Emergency event
     */
    enum class EmergencyEvent
    {
        /* Common */
        NO_EM_EVENT,

        /* Level 1 emergency states */
        ACCESS_DESIRED_DEPTH_LEVEL1_THRESHOLD,
        ACCESS_BOTTOM_HEIGHT_LEVEL1_THRESHOLD,
        ACCESS_FORWARD_OBSTACLE_DISTANCE_LEVEL1_THRESHOLD,
        ACCESS_ROLL_ANGLE_LEVEL1_THRESHOLD,
        ACCESS_PITCH_ANGLE_LEVEL1_THRESHOLD,
        YAW_LEVEL1_SALTATION,
        ACCESS_TRAJECTORY_DEVIATION_LEVEL1_THRESHOLD,
        VERTICAL_RUDDER_STUCK,
        LATERAL_RUDDER_STUCK,

        /* Level 2 emergency states */
        ACCESS_DESIRED_DEPTH_LEVEL2_THRESHOLD,
        ACCESS_BOTTOM_HEIGHT_LEVEL2_THRESHOLD,
        ACCESS_FORWARD_OBSTACLE_DISTANCE_LEVEL2_THRESHOLD,
        ACCESS_ROLL_ANGLE_LEVEL2_THRESHOLD,
        ACCESS_PITCH_ANGLE_LEVEL2_THRESHOLD,
        YAW_LEVEL2_SALTATION,
        ACCESS_TRAJECTORY_DEVIATION_LEVEL2_THRESHOLD,
        ALTIMETER_SALTATION_OR_STUCK,

        /* Level 3 emergency states */
        ACCESS_DESIRED_DEPTH_LEVEL3_THRESHOLD,
        ACCESS_BOTTOM_HEIGHT_LEVEL3_THRESHOLD,
        ACCESS_FORWARD_OBSTACLE_DISTANCE_LEVEL3_THRESHOLD,
        ACCESS_ROLL_ANGLE_LEVEL3_THRESHOLD,
        ACCESS_PITCH_ANGLE_LEVEL3_THRESHOLD,
        YAW_LEVEL3_SALTATION
    }; // EmergencyEvent

    class FlucFactor
    {
    public:
        /**
         * @brief return fluctuate factor
         */
        double getFactor()
        {
            double f_copy = f_;
            f_ = -f_;
            return f_copy;
        };
    
    private:
        static double f_;
    };

private:
    /* Controller */
    AUVBaseController* controller_;
    PIDControllerPtr u_controller_;

    /* Boost components */
    boost::posix_time::ptime last_valid_ctrl_, ctrl_time_;
    boost::posix_time::ptime last_stable_;

    double ctrl_dt_; // control period
    double pub_dt_; // publish period
    double stable_wait_t_; // time to wait after vehicle is stable
    double em_check_dt_; // emergency check period

    uint32_t seq_;
    std::string base_frame_; // base frame of vehicle

    AUVCtrlState ctrl_state_;
    EmergencyEvent em_event_;

    bool is_ctrl_vel_, is_wait_stable_;

    /* Vehicle states */
    double x_, y_, z_; // postiion in global frame
    double depth_;

    double u_, v_, w_; // linear velocity in vehicle frame
    double roll_, pitch_, yaw_; // pose in vehicle frame
    double roll_dot_, pitch_dot_, yaw_dot_; // angular velocity in vehicle frame

    double x_d_, y_d_, depth_d_;
    double pitch_d_, yaw_d_; // desired params
    double u_d_;

    /* Controll var */
    double fwdfin_, backfin_, vertfin_;
    double upper_p_, upper_s_, lower_p_, lower_s_; // for X type rudder
    double rpm_, ori_rpm_;

    /* Var for emergency */
    double prev_yaw_;

    /* Threads */
    boost::thread* ctrl_thread_; // thread for slide model control algorithm
    boost::thread* pub_thread_;  
    boost::thread* vel_ctrl_thread_;  
    // ThreadPtr ctrl_thread_;
    // ThreadPtr pub_thread_;
    // ThreadPtr vel_ctrl_thread_;
    //
    /* Emergency check threads */
    boost::thread* em_depth_check_thread_;
    boost::thread* em_roll_check_thread_;
    boost::thread* em_pitch_check_thread_;
    boost::thread* em_yaw_check_thread_;

    /* condition var */
    boost::condition_variable_any ctrl_cond_;
    // boost::condition_variable_any em_depth_check_cond_;
    // boost::condition_variable_any em_roll_check_cond_;
    // boost::condition_variable_any em_pitch_check_cond_;
    // boost::condition_variable_any em_yaw_check_cond_;

    /* Source lock for thread */
    boost::recursive_mutex ctrl_mutex_;

    /* Source lock for emergency event states */
    //boost::recursive_mutex em_depth_check_mutex_;
    //boost::recursive_mutex em_roll_check_mutex_;
    //boost::recursive_mutex em_pitch_check_mutex_;
    //boost::recursive_mutex em_yaw_check_mutex_;
    boost::recursive_mutex em_event_mutex_;

    /* Source lock for status*/
    std::mutex posegt_mutex_;

    /* Source lock for ctrl informations */
    std::mutex ctrl_info_mutex_; 

    /* Source lock for control variable */
    std::mutex ctrl_var_mutex_, ctrl_vel_mutex_;

    /* Source lock for print */
    std::mutex print_mutex_;

    /* flags */
    bool with_ff_, x_type_;
    // bool is_ctrl_run_, is_emerg_run_;
    bool debug_;

    FlucFactor fluc_;

    bool stop_;

}; // AUVControllerExp

double AUVControllerExp::FlucFactor::f_ = 1.0;

}; // ns

#endif

