/*                                                                                                                                               
 * Filename: AUVPIDControllerXF.cpp
 * Path: auv_controller
 * Created Date: Saturday, Faburary 27th 2021, 18:00:00 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "auv_controller/Common.h"
#include "auv_controller/AUVPIDControllerXF.h"

namespace auv_controller{

/**
 * @brief Control solution
 */
void AUVPIDControllerXF::controllerRun(const AUVKineticSensor& sensor, const AUVControllerInput& input, AUVControllerOutput& output, const double dt, bool vel_ctrl){
    // Update kinetic parameters
    this->kinetic_.setPosition(sensor.x_, sensor.y_, sensor.z_, sensor.roll_, sensor.pitch_, sensor.yaw_);
    this->kinetic_.setVelocity(sensor.x_dot_, sensor.y_dot_, sensor.z_dot_, sensor.roll_dot_, sensor.pitch_dot_, sensor.yaw_dot_);

    double vertical_dev = (this->kinetic_.x_ - input.x_d_) *sin(input.pitch_d_) - (this->kinetic_.z_ - input.depth_d_) * cos(input.pitch_d_);
    printf("VerticalDist: %f\n", vertical_dev);

    double lateral_dev = (this->kinetic_.x_ - input.x_d_) * sin(input.yaw_d_) - (this->kinetic_.y_ - input.y_d_) * cos(input.yaw_d_);
    printf("LateralDist: %f\n", lateral_dev);

    /* Depth PID control */
    // double ref_pitch = input.pitch_d_ + std::atan2(vertical_dev, 4 * this->body_.l_);
    double ref_pitch = input.pitch_d_ + std::atan2(vertical_dev, this->body_.l_);
    this->depth_controller_->setTargetParams(ref_pitch);
    this->deltas_ = this->depth_controller_->positionalPID(sensor.pitch_);
    printf("Ref pitch: %f, Current pitch: %f\n", ref_pitch, sensor.pitch_);

    /* Lateral deviation PID control */
    // double ref_yaw = input.yaw_d_ + std::atan2(lateral_dev, 4 * this->body_.l_);
    double ref_yaw = input.yaw_d_ + std::atan2(lateral_dev, this->body_.l_);
    this->latdev_controller_->setTargetParams(ref_yaw);
    this->deltar_ = this->latdev_controller_->positionalPID(sensor.yaw_);
    printf("Ref yaw: %f, Current yaw: %f\n", ref_yaw, sensor.yaw_);

    /* X type rudder Allocate */
    Eigen::Matrix<double,2,1> deltarud;
    deltarud << this->deltar_, this->deltas_;

    printf("Lateral rudder resolution: forward rudder: %f, backward rudder: %f\n", this->deltas_, this->deltar_);

    Eigen::Matrix<double,4,2> x_rudder_map;
    // x_rudder_map << 0.25, 0.25, 0.25, -0.25, 0.25, 0.25, 0.25, -0.25;
    x_rudder_map << 0.5, 0.5, 0.5, -0.5, 0.5, 0.5, 0.5, -0.5;
    x_rudder_map *= sqrt(2);
    
    // std::vector<double> x_rudder_mapvec{0.25, 0.25, 0.25, 0.25, 0.25, -0.25, 0.25, -0.25};
    // Eigen::Map<DynamicMatrix> x_rudder_map(x_rudder_mapvec.data(), 2, 4);
    // x_rudder_map *= sqrt(2);
    // 
    // // auto deltar = x_rudder_map.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullU).solve(deltarud); 
    // // auto svd = x_rudder_map.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    // Eigen::JacobiSVD<DynamicMatrix> svd(x_rudder_map, Eigen::ComputeThinU | Eigen::ComputeThinV);
    // double tolerance = std::numeric_limits<double>::epsilon() * std::max(x_rudder_map.cols(), x_rudder_map.rows()) * 
    //     svd.singularValues().array().abs()(0);
    // auto x_rudder_mapinv = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * 
    //     svd.matrixU().adjoint();
    // 
    // auto deltar = x_rudder_mapinv * deltarud;
    auto deltar = x_rudder_map * deltarud;

    std::cout << "X allocate: " << deltar << std::endl;

    this->deltaus_ = deltar(0);
    this->deltaup_ = deltar(1);
    this->deltalp_ = deltar(2);
    this->deltals_ = deltar(3);

    if(fabs(this->deltaup_) > 30 / 57.3){
        this->deltaup_ = (30 / 57.3) * sign(this->deltaup_);
    }
    if(fabs(this->deltaus_) > 30 / 57.3){
        this->deltaus_ = (30 / 57.3) * sign(this->deltaus_);
    }
    if(fabs(this->deltalp_) > 30 / 57.3){
        this->deltalp_ = (30 / 57.3) * sign(this->deltalp_);
    }
    if(fabs(this->deltals_) > 30 / 57.3){
        this->deltals_ = (30 / 57.3) * sign(this->deltals_);
    }

    // Print control value of forward, afterward and orientation rudder
    printf("uppper port: %f upper starboard: %f lower port: %f lower starboard: %f\n", this->deltaup_, this->deltaus_, this->deltalp_, this->deltals_);
    output.upper_p_ = this->deltaup_;
    output.upper_s_ = this->deltaus_;
    output.lower_p_ = this->deltalp_;
    output.lower_s_ = this->deltals_;
    /*
    if(vel_ctrl){
        double u_d = input.u_d_;
        // double th_d = -(this->dynamic_.z_dotw_ * (this->kinetic_.w_ + this->kinetic_.q_) * this->kinetic_.q_ - 
        //      (this->dynamic_.y_dotv_ + this->dynamic_.y_dotr_) * this->kinetic_.r_ + this->dynamic_.x_uu_ * this->kinetic_.u_ * abs(this->kinetic_.u_) -
        //      (this->body_.w_ - this->body_.b_) * sin(this->kinetic_.theta_)); 
        double th_d = -this->dynamic_.x_uu_ * u_d * abs(u_d);

        double rpm_d;
        if(u_d > 0){
            rpm_d = (th_d - this->th_.sigma_) / this->th_.c_t_ + this->th_.r_death_area_;
        }
        else{
            rpm_d = -((abs(th_d) - this->th_.sigma_) / this->th_.c_t_ + abs(this->th_.l_death_area_));
        }
        output.rpm_ = rpm_d;
    }*/

    // PID
    if(vel_ctrl)
    {
        this->vel_controller_->setTargetParams(input.u_d_);
        // output.rpm_ = this->vel_controller_->positionalPID(sensor.x_dot_);
        output.rpm_ = this->vel_controller_->incrementalPID(kinetic_.u_);
        printf("Deisred u: %f Current u: %f\n", input.u_d_, kinetic_.u_);
    }
}

}; // ns
