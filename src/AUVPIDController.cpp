/*                                                                                                                                               
 * Filename: AUVPIDController.cpp
 * Path: auv_controller
 * Created Date: Saturday, Faburary 27th 2021, 15:59:16
 * Author: zhao wang
 * 
 * Copyright (c) 2021 Your Company
 */

#include <math.h>
#include "auv_controller/AUVPIDController.h"

namespace auv_controller{
AUVPIDController::AUVPIDController(){
    defaultInit();
}


bool AUVPIDController::setAUVBodyParams(const std::vector<double>& body){
    if(body.size() == body_num_){
        setAUVBodyParams(body[0], body[1], body[2], body[3], body[4], body[5], body[6],
            body[7], body[8], body[9], body[10], body[11], body[12]);
        return true;
    }
    else{
        return false;
    }
};

void AUVPIDController::setCtrlParams(double kpz, double kiz, double kdz, double kptheta, double kitheta, double kdtheta,
        double kpy, double kiy, double kdy, double kppsi, double kipsi, double kdpsi)
{
    pid_ctrl_.z_.setParameters(kpz, kiz, kdz); // depth
    pid_ctrl_.theta_.setParameters(kptheta, kitheta, kdtheta); // pitch
    pid_ctrl_.y_.setParameters(kpy, kiy, kdy); // y
    pid_ctrl_.psi_.setParameters(kppsi, kipsi, kdpsi); // yaw
};


/**
 * @brief Set control parameters
 */
bool AUVPIDController::setCtrlParams(const std::vector<double>& ctrl){
    if(ctrl.size() == ctrl_num_){
        setCtrlParams(ctrl[0], ctrl[1], ctrl[2], ctrl[3], ctrl[4], ctrl[5], ctrl[6], ctrl[7], ctrl[8], ctrl[9], ctrl[10], ctrl[11]);
        return true;
    }
    else{
        return false;
    }
}

/**
 * @brief Set control parameters
 */
void AUVPIDController::setCtrlParams(double kp, double ki, double kd, unsigned int flag)
{
    switch (flag)
    {
    case 0:
        pid_ctrl_.z_.setParameters(kp, ki, kd);
        break;
    case 1:
        pid_ctrl_.theta_.setParameters(kp, ki, kd);
        break;
    case 2:
        pid_ctrl_.y_.setParameters(kp, ki, kd);
    case 3:
        pid_ctrl_.psi_.setParameters(kp, ki, kd);
        break;
    default:
        break;
    }
}


/**
 * @brief Initialize with default paramters
 */
void AUVPIDController::defaultInit(){
    setAUVBodyParams(228.486, 3.145, 2239.163, 2295.142, 0, 0, 0, 1.64, 0, 0.015, 2.923, 150.3313, 150.3313);
    setCtrlParams(0.06, 0.0, 3.0, 0.06, 0.0, 3.0, 0.06, 0.0, 0.0, 0.06, 0.0, 3.0); // x type rudder
    //
    depth_controller_ = boost::make_shared<PIDController>(pid_ctrl_.z_.kp_, pid_ctrl_.z_.ki_, pid_ctrl_.z_.kd_);
    latdev_controller_ = boost::make_shared<PIDController>(pid_ctrl_.y_.kp_, pid_ctrl_.y_.ki_, pid_ctrl_.y_.kd_);
    vel_controller_ = boost::make_shared<PIDController>(1000.0, 0.0, 0.0);

    deltab_ = 0.0; deltas_ = 0.0; deltar_ = 0.0;
    deltaup_ = 0.0; deltaus_ = 0.0; deltalp_ = 0.0; deltals_ = 0.0;
}

/**
 * @brief Serialize auv body parameters
 */ 
void AUVPIDController::serializeAUVBodyParams(std::stringstream& str){
    str << "m: " << body_.m_ << " l: " << body_.l_ << " w: " << body_.w_ << " b: " << body_.b_;
    str << " xb: " << body_.x_b_ << " yb: " << body_.y_b_ << " zb: " << body_.z_b_;
    str << " xg: " << body_.x_g_ << " yg: " << body_.y_g_ << " zg: " << body_.z_g_;
    str << " ixx: " << body_.i_xx_ << " iyy: " << body_.i_yy_ << " izz: " << body_.i_zz_;
}


/**
 * @brief Serialize auv control parameters
 */
void AUVPIDController::serializeAUVControlParams(std::stringstream& str){
    str << "kpz: " << pid_ctrl_.z_.kp_ << " kiz: " << pid_ctrl_.z_.ki_ << " kdz: " << pid_ctrl_.z_.kd_;
    str << " kptheta: " << pid_ctrl_.theta_.kp_ << " kitheta: " << pid_ctrl_.theta_.ki_ << " kdtheta: " << pid_ctrl_.theta_.kd_;
    str << " kpy: " << pid_ctrl_.y_.kp_ << " kiy: " << pid_ctrl_.y_.ki_ << " kdy: " << pid_ctrl_.y_.kd_;
    str << " kppsi: " << pid_ctrl_.psi_.kp_ << " kipsi: " << pid_ctrl_.psi_.ki_ << " kdpsi: " << pid_ctrl_.psi_.kd_;
}

}; // ns

