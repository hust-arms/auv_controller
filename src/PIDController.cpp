#include "auv_controller/PIDController.h"

/* Implementation of PIDController */
void PIDController::setTargetParams(double tar){
    boost::unique_lock<boost::recursive_mutex> params_lock(params_mutex_);
    params_tar_ = tar;
    params_lock.unlock();
}

void PIDController::setPIDParams(double kp, double ki, double kd){
    boost::unique_lock<boost::recursive_mutex> params_lock(params_mutex_);
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    params_lock.unlock();
}

void PIDController::resetDeviation(){
    boost::unique_lock<boost::recursive_mutex> params_lock(params_mutex_);
    dev_ = 0.0;
    dev_last_ = 0.0;
    params_lock.unlock();
}

double PIDController::positionalPID(double params_cur){
    dev_ = -(params_cur - params_tar_);
    dev_integral_ += dev_;
    double res = kp_ * dev_ + ki_ * dev_integral_ + kd_ * (dev_ - dev_last_); // position pid
    dev_last_ = dev_;
    return res;
}

double PIDController::incrementalPID(double params_cur){
    dev_ = -(params_cur - params_tar_);
    double res = kp_ * (dev_ - dev_last_)  + ki_ * dev_ + kd_ * (dev_ + dev_last_ - 2.0 * dev_last_bef_);
    dev_last_bef_ = dev_last_; 
    dev_last_ = dev_;
    return res;
}

PIDControllerFactory& PIDControllerFactory::getInstance(){
    static PIDControllerFactory instance;
    return instance;
}

PIDControllerPtr PIDControllerFactory::create(double kp, double ki, double kd, double params_tar){
    return boost::shared_ptr<PIDController>(new PIDController(kp, ki, kd, params_tar));
}
