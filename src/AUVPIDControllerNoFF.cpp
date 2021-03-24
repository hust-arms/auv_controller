/*                                                                                                                                               
 * Filename: AUVPIDControllerNoFF.cpp
 * Path: auv_controller
 * Created Date: Thirsday, March 24th 2021, 18:00:00 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "auv_controller/Common.h"
#include "auv_controller/AUVPIDControllerNoFF.h"

namespace auv_controller{

/**
 * @brief Control solution
 */
void AUVPIDControllerNoFF::controllerRun(const AUVKineticSensor& sensor, const AUVControllerInput& input, AUVControllerOutput& output, const double dt, bool vel_ctrl)
{
    // Update kinetic parameters
    this->kinetic_.setPosition(sensor.x_, sensor.y_, sensor.z_, sensor.roll_, sensor.pitch_, sensor.yaw_);
    this->kinetic_.setVelocity(sensor.x_dot_, sensor.y_dot_, sensor.z_dot_, sensor.roll_dot_, sensor.pitch_dot_, sensor.yaw_dot_);

    double vertical_dev = this->kinetic_.z_ - input.depth_d_;
    // printf("VerticalDist: %f\n", vertical_dev);

    double lateral_dev = (this->kinetic_.x_ - input.x_d_) * sin(input.yaw_d_) - (this->kinetic_.y_ - input.y_d_) * cos(input.yaw_d_);
    // printf("LateralDist: %f\n", lateral_dev);

    /* Depth PID control */
    // double ref_pitch = input.pitch_d_ + std::atan2(vertical_dev, 2 * this->body_.l_);
    double ref_pitch = input.pitch_d_ + static_cast<double>(pitch_limit_ / 57.3) * ((exp(pitch_k_ * vertical_dev) - 1) / (exp(pitch_k_ * vertical_dev) + 1));
    this->depth_controller_->setTargetParams(ref_pitch);
    this->deltas_ = this->depth_controller_->positionalPID(sensor.pitch_, dt);
    // printf("Ref pitch: %f, Current pitch: %f\n", ref_pitch, sensor.pitch_);

    /* Lateral deviation PID control */
    double ref_yaw = input.yaw_d_ + std::atan2(lateral_dev, yaw_k_ * this->body_.l_);

    // Transform yaw from [-pi,pi] to [0,2*pi], if yaw accesses pi or -pi, the controller will encounter the fluctuation

    this->latdev_controller_->setTargetParams(ref_yaw);
    this->deltar_ = this->latdev_controller_->positionalPID(sensor.yaw_, dt);
    printf("Ref yaw: %f, Current yaw: %f\n", ref_yaw, sensor.yaw_);

    printf("Lateral rudder resolution: forward rudder: %f, backward rudder: %f\n", this->deltas_, this->deltar_);

    // PID
    if(vel_ctrl)
    {
        this->vel_controller_->setTargetParams(input.u_d_);
        // output.rpm_ = this->vel_controller_->positionalPID(sensor.x_dot_);
        output.rpm_ = this->vel_controller_->incrementalPID(kinetic_.u_, dt);
        printf("Deisred u: %f Current u: %f\n", input.u_d_, kinetic_.u_);
    }
}

}; // ns
