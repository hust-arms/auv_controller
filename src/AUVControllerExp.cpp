/*                                                                                             
 * Filename: AUVControllerExp.cpp
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
#include "auv_controller/AUVControllerExp.h"

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
AUVControllerExp::AUVControllerExp(std::string auv_name, bool with_ff, bool x_type, bool debug) : 
    with_ff_(with_ff), x_type_(x_type), debug_(debug)
{
    debug_ = !debug_;

   // Default parameters
   base_frame_ = auv_name + "/base_link";
   rpm_ = 1400.0;
   ori_rpm_ = rpm_;
   ctrl_dt_ = 0.2;
   pub_dt_ = 0.2;
   em_check_dt_ = 0.5;
   stable_wait_t_ = 90.0;

   // Default control info
   x_d_ = 0.0;
   y_d_ = 0.0;
   depth_d_ = 0.0;

   double pitch_d = 0.0 * degree2rad;
   double yaw_d = 0.0 * degree2rad;

   pitch_d_ = 0.0;
   yaw_d_ = 0.0;

   double kp, ki, kd, c_t, r_l, l_l, sigma;
   kp = 1.0; ki = 0.0; kd = 0.1; 
   u_d_ = 1.5432;

   c_t = 0.0875; r_l = 888.0; l_l = -888.0;
   sigma = -1.0;

   /* initialize controller */
   if (with_ff){
       if(debug_)
       {
           std::lock_guard<std::mutex> guard(print_mutex_);
           printf("[AUVControllerExp]: Model with front fins\n");
       }
       controller_ = new AUVControllerWithFF();
   }
   else{
       if(x_type)
       {
           if(debug_)
           {
               std::lock_guard<std::mutex> guard(print_mutex_);
               printf("[AUVControllerExp]: Model with X type rudder\n");
           }
           controller_ = new AUVControllerXF();
       }
       else
       {
           if(debug_)
           {
               std::lock_guard<std::mutex> guard(print_mutex_);
               printf("[AUVControllerExp]: Model without front fins\n");
           }
           controller_ = new AUVControllerNoFF();
       }
   }

   controller_->setThrusterFactor(c_t, r_l, l_l, sigma);

   is_ctrl_vel_ = false; is_wait_stable_ = false;

   fwdfin_ = 0.0; backfin_ = 0.0; vertfin_ = 0.0; 
   upper_p_ = 0.0; upper_s_ = 0.0; lower_p_ = 0.0; lower_s_ = 0.0;

   // is_ctrl_run_ = false; is_emerg_run_ = false;

   ctrl_state_ = AUVCtrlState::STANDBY;
   em_event_ = EmergencyEvent::NO_EM_EVENT;

   prev_yaw_ = 0.0;

   stop_ = false; // loop flag

   if(debug_)
   {
       std::lock_guard<std::mutex> guard(print_mutex_);
       printf("[AUVControllerExp]: Finish controller initialization\n");
   }
}

/////////////////////////////////////
AUVControllerExp::~AUVControllerExp(){
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
void AUVControllerExp::startControl(){
    if(debug_)
    {
        printf("[AUVControllerExp]: Start control thread & publish thread\n");
    }
    // ctrl_state_ = AUVCtrlState::CTRL;
    // is_ctrl_run_ = true; is_emerg_run_ = false;
    em_depth_check_thread_ = new boost::thread(boost::bind(&AUVControllerExp::emDepthCheckThread, this));
    em_roll_check_thread_ = new boost::thread(boost::bind(&AUVControllerExp::emRollCheckThread, this));
    em_pitch_check_thread_ = new boost::thread(boost::bind(&AUVControllerExp::emPitchCheckThread, this));
    em_yaw_check_thread_ = new boost::thread(boost::bind(&AUVControllerExp::emYawCheckThread, this));
#ifdef SM_CTRL
    ctrl_thread_ = new boost::thread(boost::bind(&AUVControllerExp::controlThread, this));
    vel_ctrl_thread_ = new boost::thread(boost::bind(&AUVControllerExp::velControlThread, this));
#endif
    pub_thread_ = new boost::thread(boost::bind(&AUVControllerExp::publishThread, this));
}

/////////////////////////////////////
void AUVControllerExp::stopControl()
{
    boost::unique_lock<boost::recursive_mutex> stop_ctrl_lock(stop_ctrl_mutex_);
    stop_ = true;
}

/////////////////////////////////////
void AUVControllerExp::getCtrlVar(double& fwdfin, double& backfin, double& vertfin, double& thruster)
{
    std::lock_guard<std::mutex> guard(ctrl_var_mutex_);
    fwdfin =  fwdfin_;
    backfin = backfin_;
    vertfin = vertfin_;
    thruster = rpm_;
}

/////////////////////////////////////
void AUVControllerExp::getCtrlVar(double& upper_p, double& upper_s, double& lower_p, double& lower_s, 
                                  double& thruster)
{
    std::lock_guard<std::mutex> guard(ctrl_var_mutex_);
    upper_p = upper_p_;
    upper_s = upper_s_;
    lower_p = lower_p_;
    lower_s = lower_s_;
    thruster = rpm_;
}

/////////////////////////////////////
void AUVControllerExp::updatePose(double x, double y, double z, double roll, double pitch, double yaw,
                                  double u, double v, double w, double roll_dot, double pitch_dot, double yaw_dot)
{
    std::lock_guard<std::mutex> guard(posegt_mutex_);
    x_ = x; y_ = y; z_ = z; depth_ = -z_; 
    u_ = u; v_ = v; w_ = w;
    roll_ = roll; pitch_ = pitch; yaw_ = yaw; 
    roll_dot_ = roll_dot; pitch_dot_ = pitch_dot; yaw_dot_ = yaw_dot;
}

/////////////////////////////////////
void AUVControllerExp::updateCtrlInfo(double x_d, double y_d, double depth_d, double yaw_d, double pitch_d, double u_d)
{
    std::lock_guard<std::mutex> guard(ctrl_info_mutex_);
    x_d_ = x_d; y_d_ = y_d; depth_d_ = depth_d;
    yaw_d_ = yaw_d; pitch_d_ = pitch_d; u_d_ = u_d;
}

/////////////////////////////////////
void AUVControllerExp::controlThread(){
    bool wait_for_wake = false;

    boost::unique_lock<boost::recursive_mutex> lock(ctrl_mutex_);
    while(!stop_){
        // while(wait_for_wake || ctrl_state_ != AUVCtrlState::CTRL){
        while(ctrl_state_ != AUVCtrlState::CTRL){
            if(debug_){
                std::lock_guard<std::mutex> guard(print_mutex_);
                printf("[AUVControllerExp]: Control thread is suspending\n");
            }
            ctrl_cond_.wait(lock);
            // wait_for_wake = false;
        }
        // control thread wake
        if(debug_){
            std::lock_guard<std::mutex> guard(print_mutex_);
            printf("[AUVControllerExp]: Control thread is waked\n");
        }
        lock.unlock();

        boost::timer count_t;

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
            std::cout << "[AUVControllerExp]: Vehicle status:{" << "x:" << sensor_msg.x_ << " y:" << sensor_msg.y_ << " z:" << sensor_msg.z_
            << " roll:" << sensor_msg.roll_ << " pitch:" << sensor_msg.pitch_ << " yaw:" << sensor_msg.yaw_
            << " u:" << sensor_msg.x_dot_ << " v:" << sensor_msg.y_dot_ << " w:" << sensor_msg.z_dot_
            << " roll vel:" << sensor_msg.roll_dot_ << " pitch vel:" << sensor_msg.pitch_dot_ << " yaw vel" << sensor_msg.yaw_dot_
            << "}" << std::endl;
        }

        // Create Controller input
        AUVControllerInput input(getDesiredDepth(), -getDesiredPitch(), -getDesiredYaw(), getDesiredX(), -getDesiredY(), getDesiredLinVelX());

        // Print control input
        if(debug_){
            std::lock_guard<std::mutex> guard(print_mutex_);
            std::cout << "[AUVControllerExp]: Control input:{" << " desired x: " << x_d_ << " desired y:" << y_d_ << " desired depth:" << depth_d_ <<
                " desired pitch:" << pitch_d_ << " desired yaw:" << yaw_d_ << " desired u:" << u_d_ << "}" << std::endl; 
        }

        AUVControllerOutput output;
        output.fwd_fin_ = 0.0; output.aft_fin_ = 0.0; output.rudder_ = 0.0; 
        output.upper_p_ = 0.0; output.upper_s_ = 0.0; output.lower_p_ = 0.0; output.lower_s_ = 0.0;

        // bool ctrl_vel = false;
        // if(isStable()){
        //     ctrl_vel = true;
        // }
        // controller_->controllerRun(sensor_msg, input, output, ctrl_dt_, ctrl_vel);
        controller_->controllerRun(sensor_msg, input, output, ctrl_dt_, is_ctrl_vel_);

        {
            std::lock_guard<std::mutex> guard(ctrl_var_mutex_);
            // update rudder info 
            if(!x_type_)
            {
                vertfin_ = output.rudder_;
                fwdfin_ = output.fwd_fin_;
                backfin_ = output.aft_fin_;
                printf("[AUVControllerExp]: vertfin: %f fwdfin: %f backfin: %f\n", vertfin_, fwdfin_, backfin_);
            }
            else
            {
                upper_p_ = output.upper_p_;
                upper_s_ = output.upper_s_;
                lower_p_ = output.lower_p_;
                lower_s_ = output.lower_s_;
                printf("[AUVControllerExp]: upper_p: %f upper_s: %f lower_p: %f lower_f: %f\n", upper_p_, upper_s_, lower_p_, lower_s_);
            }
            
            // update rotor speed 
            if(is_ctrl_vel_){
                rpm_ += output.rpm_;
                if(debug_){
                    std::lock_guard<std::mutex> guard(print_mutex_);
                    printf("[AUVControllerExp]: RPM: %f\n", rpm_);
                }
            }
            else
            {
                rpm_ = ori_rpm_;
                if(debug_){
                    std::lock_guard<std::mutex> guard(print_mutex_);
                    printf("[AUVControllerExp]: RPM: %f\n", rpm_);
                }
            }
        }

        lock.lock();

        double sleep_time = ctrl_dt_ - count_t.elapsed();
        boost::this_thread::sleep(boost::posix_time::milliseconds(sleep_time * 1000));
    }
}

/////////////////////////////////////
void AUVControllerExp::velControlThread(){
    while(!isCtrlStop()){
        if(isStable()){
            if(!is_ctrl_vel_ && !is_wait_stable_){
                // if stable and flag is not changed, only update stable time.
                boost::posix_time::ptime now(boost::posix_time::second_clock::local_time());
                last_stable_ = now;
                is_wait_stable_ = true;
                if(debug_){
                    std::lock_guard<std::mutex> guard(print_mutex_);
                    printf("[AUVControllerExp]: Wait to control velocity\n");
                }
            }
            else{
                if(is_wait_stable_){
                    // if stable and flag has been changed, check wait time.
                    boost::posix_time::ptime now(boost::posix_time::second_clock::local_time());
                    boost::posix_time::time_duration sleep_time = now - last_stable_;
                    if(sleep_time.total_seconds() >= stable_wait_t_){
                        std::lock_guard<std::mutex> guard(ctrl_vel_mutex_);
                        is_ctrl_vel_ = true;
                        // last_stable_ = ros::Time::now();
                        is_wait_stable_ = false;
                        if(debug_){
                            std::lock_guard<std::mutex> guard(print_mutex_);
                            printf("[AUVControllerExp]: End velocity control waiting\n");
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
void AUVControllerExp::wakeControl(const boost::system::error_code& ec){
    printf("wake\n");
    ctrl_cond_.notify_one();
}

/////////////////////////////////////
void AUVControllerExp::wakeEMDepthCheck(const boost::system::error_code& ec){
    // em_depth_check_cond_.notify_one();
}

/////////////////////////////////////
void AUVControllerExp::wakeEMRollCheck(const boost::system::error_code& ec){
    // em_roll_check_cond_.notify_one();
}

/////////////////////////////////////
void AUVControllerExp::wakeEMPitchCheck(const boost::system::error_code& ec){
    // em_pitch_check_cond_.notify_one();
}

/////////////////////////////////////
void AUVControllerExp::wakeEMYawCheck(const boost::system::error_code& ec){
    // em_yaw_check_cond_.notify_one();
}

/////////////////////////////////////
void AUVControllerExp::publishThread(){
    // wake emergency check threads
    // boost::unique_lock<boost::recursive_mutex> em_depth_check_lock(em_depth_check_mutex_);
    // em_depth_check_cond_.notify_one();
    // em_depth_check_lock.unlock();
    // 
    // boost::unique_lock<boost::recursive_mutex> em_roll_check_lock(em_roll_check_mutex_);
    // em_roll_check_cond_.notify_one();
    // em_roll_check_lock.unlock();
    // 
    // boost::unique_lock<boost::recursive_mutex> em_pitch_check_lock(em_pitch_check_mutex_);
    // em_pitch_check_cond_.notify_one();
    // em_pitch_check_lock.unlock();
    // 
    // boost::unique_lock<boost::recursive_mutex> em_yaw_check_lock(em_yaw_check_mutex_);
    // em_yaw_check_cond_.notify_one();
    // em_yaw_check_lock.unlock();

    //wake control thread
    boost::unique_lock<boost::recursive_mutex> ctrl_lock(ctrl_mutex_);
    ctrl_cond_.notify_one();
    ctrl_lock.unlock();

    while(!isCtrlStop())
    {
        double vertfin, fwdfin, backfin, rpm;
        double upper_p, upper_s, lower_p, lower_s;
        vertfin = 0.0; fwdfin = 0.0; backfin = 0.0; rpm = 0.0;
        upper_p = 0.0; upper_s = 0.0; lower_p = 0.0; lower_s = 0.0;
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
                printf("[AUVControllerExp]: Level1 emergency process!\n");
            }
            // If current state is in level2 emergency, set max rudder and wait for AUV come-up
            if(ctrl_state_ == AUVCtrlState::EMERGENCY_LEVEL2)
            {
                vertfin = 0.0 / 57.3;
                fwdfin = -30 / 57.3;
                backfin = 30 / 57.3;
                rpm = 0.0;
                std::lock_guard<std::mutex> guard(print_mutex_);
                printf("[AUVControllerExp]: Level2 emergency process!\n");
            }
            // If current state is in level3 emergency, set max rudder and reject load, then wait for AUV come-up
            if(ctrl_state_ == AUVCtrlState::EMERGENCY_LEVEL3)
            {
                vertfin = 0.0 / 57.3;
                fwdfin = -30 / 57.3;
                backfin = 30 / 57.3;
                rpm = 0.0;
                std::lock_guard<std::mutex> guard(print_mutex_);
                printf("[AUVControllerExp]: Level3 emergency process!\n");
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

        if(debug_)
        {
            // print statues info
            if(!x_type_)
            {
                std::lock_guard<std::mutex> guard(print_mutex_);
                printf("[AUVControllerExp]: Control output:{vertical fin:%8f forward fin:%8f back fin:%8f rpm:%8f}\n", 
                       vertfin, fwdfin, backfin, rpm);
            }
            else
            {
                std::lock_guard<std::mutex> guard(print_mutex_);
                printf("[AUVControllerExp]: Control output:{upper port fin:%8f upper starboard fin:%8f lower port fin:%8f lower starboard fin:%8f rpm:%8f}\n", 
                       upper_p, upper_s, lower_p, lower_s, rpm);
            }
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(pub_dt_ * 1000));
    };
}

/////////////////////////////////////
void AUVControllerExp::emDepthCheckThread()
{
    // bool wait_for_wake = true;
    int check_cnt = 0;

    while(!isCtrlStop())
    {
        // boost::unique_lock<boost::recursive_mutex> lock(em_depth_check_mutex_);
        // Wait for wake
        // while(wait_for_wake){
        //     em_depth_check_cond_.wait(lock);
        //     wait_for_wake = false;                                                       
        // }
        // lock.unlock();

        boost::timer count_t;

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

        // lock.lock();

        double sleep_time = em_check_dt_ - count_t.elapsed();
        boost::this_thread::sleep(boost::posix_time::milliseconds(sleep_time * 1000));

        // if(em_check_dt_ > 0.0){
        //     double sleep_time = em_check_dt_ - count_t.elapsed();
        //     if(sleep_time > 0.0){
        //         // if(debug_){
        //         //     std::lock_guard<std::mutex> guard(print_mutex_);
        //         //     printf("Control thread is waiting for wake\n");
        //         // }
        //         wait_for_wake = true;
        //         boost::thread wake_em_depth_check_th(boost::bind(&AUVControllerExp::wakeEMDepthCheckThread, this, sleep_time));
        //     }
        // }
    }
}

/////////////////////////////////////
void AUVControllerExp::emRollCheckThread()
{
    // bool wait_for_wake = true;
    int check_cnt = 0;

    while(!isCtrlStop())
    {
        // boost::unique_lock<boost::recursive_mutex> lock(em_roll_check_mutex_);
        // // Wait for wake
        // while(wait_for_wake){
        //     em_roll_check_cond_.wait(lock);
        //     wait_for_wake = false;                                                       
        // }
        // lock.unlock();

        boost::timer count_t;
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

        // lock.lock();

        double sleep_time  = em_check_dt_ - count_t.elapsed();

        boost::this_thread::sleep(boost::posix_time::milliseconds(sleep_time * 1000));

        // if(em_check_dt_ > 0.0){
        //     double sleep_time  = em_check_dt_ - count_t.elapsed();
        //     if(sleep_time > 0.0){
        //         // if(debug_){
        //         //     std::lock_guard<std::mutex> guard(print_mutex_);
        //         //     printf("Control thread is waiting for wake\n");
        //         // }
        //         wait_for_wake = true;
        //         boost::thread wake_em_roll_check_th(boost::bind(&AUVControllerExp::wakeEMRollCheckThread, this, sleep_time));
        //     }
        // }
    }
}

/////////////////////////////////////
void AUVControllerExp::emPitchCheckThread()
{
    // bool wait_for_wake = true;
    int check_cnt = 0;

    while(!isCtrlStop())
    {
        // boost::unique_lock<boost::recursive_mutex> lock(em_pitch_check_mutex_);
        // Wait for wake
        // while(wait_for_wake){
        //     em_pitch_check_cond_.wait(lock);
        //     wait_for_wake = false;                                                       
        // }
        // lock.unlock();

        boost::timer count_t;
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

        // lock.lock();

        double sleep_time = em_check_dt_ - count_t.elapsed();
        boost::this_thread::sleep(boost::posix_time::milliseconds(sleep_time * 1000));

        // if(em_check_dt_ > 0.0){
        //     double sleep_time = em_check_dt_ - count_t.elapsed();
        //     if(sleep_time > 0.0){
        //         // if(debug_){
        //         //     std::lock_guard<std::mutex> guard(print_mutex_);
        //         //     printf("Control thread is waiting for wake\n");
        //         // }
        //         wait_for_wake = true;
        //         boost::thread wake_em_pitch_check_th(boost::bind(&AUVControllerExp::wakeEMPitchCheckThread, this, sleep_time));
        //     }
        // }
    }
}

/////////////////////////////////////
void AUVControllerExp::emYawCheckThread()
{
    // bool wait_for_wake = true;
    double prev_yaw = getYaw();
    double cur_yaw = prev_yaw;

    while(!isCtrlStop())
    {
        // boost::unique_lock<boost::recursive_mutex> lock(em_yaw_check_mutex_);
        // // Wait for wake
        // while(wait_for_wake){
        //     em_yaw_check_cond_.wait(lock);
        //     wait_for_wake = false;                                                       
        // }
        // lock.unlock();

        boost::timer count_t;
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

        // lock.lock();

        double sleep_time = em_check_dt_ - count_t.elapsed(); 
        boost::this_thread::sleep(boost::posix_time::milliseconds(sleep_time * 1000));

        // if(em_check_dt_ > 0.0){
        //     double sleep_time = em_check_dt_ - count_t.elapsed(); 
        //     if(sleep_time > 0.0){
        //         // if(debug_){
        //         //     std::lock_guard<std::mutex> guard(print_mutex_);
        //         //     printf("Control thread is waiting for wake\n");
        //         // }
        //         wait_for_wake = true;
        //         boost::thread wake_em_yaw_check_th(boost::bind(&AUVControllerExp::wakeEMYawCheckThread, this, sleep_time));
        //     }
        // }
    }
}

/////////////////////////////////////
bool AUVControllerExp::isStable(){
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
void AUVControllerExp::printAUVDynamicParams(){
    std::stringstream ss;
    ss << "AUV dynamic parameters: {";
    controller_->serializeAUVDynamicParams(ss);
    ss << "}";
    std::lock_guard<std::mutex> guard(print_mutex_);
    std::cout << ss.str() << std::endl;
}

/////////////////////////////////////
void AUVControllerExp::printAUVCtrlParams(){
    std::stringstream ss;
    ss << "AUV control parameters: {";
    controller_->serializeAUVControlParams(ss);
    ss << "}";
    std::lock_guard<std::mutex> guard(print_mutex_);
    std::cout << ss.str() << std::endl;
}

/////////////////////////////////////
void AUVControllerExp::printAUVForceParams(){
    std::stringstream ss;
    ss << "AUV Force parameters: {";
    controller_->serializeAUVForceParams(ss);
    ss << "}";
    std::lock_guard<std::mutex> guard(print_mutex_);
    std::cout << ss.str() << std::endl;
}

/////////////////////////////////////
void AUVControllerExp::printAUVXForceParams(){
    std::stringstream ss;
    ss << "AUV XForce parameters: {";
    controller_->serializeAUVXForceParams(ss);
    ss << "}";
    std::lock_guard<std::mutex> guard(print_mutex_);
    std::cout << ss.str() << std::endl;
}

/////////////////////////////////////
void AUVControllerExp::printAUVBodyParams(){
    std::stringstream ss;
    ss << "AUV body parameters: {";
    controller_->serializeAUVBodyParams(ss);
    ss << "}";
    std::lock_guard<std::mutex> guard(print_mutex_);
    std::cout << ss.str() << std::endl;
}

}; // ns
