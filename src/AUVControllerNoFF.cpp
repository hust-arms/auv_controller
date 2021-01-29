/*                                                                                                                                               
 * Filename: AUVControllerNoFF.cpp
 * Path: auv_controller
 * Created Date: Friday, Janurary 29th 2021, 10:07:39 am
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include "auv_controller/Common.h"
#include "auv_controller/AUVControllerNoFF.h"

namespace auv_controller{
/**
 * @brief Control solution
 */
void AUVControllerNoFF::controllerRun(const AUVKineticSensor& sensor, const AUVControllerInput& input, AUVControllerOutput& output, const double dt){
    // Update kinetic parameters
    this->kinetic_.setPosition(sensor.x_, sensor.y_, sensor.z_, sensor.roll_, sensor.pitch_, sensor.yaw_);
    this->kinetic_.setVelocity(sensor.x_dot_, sensor.y_dot_, sensor.z_dot_, sensor.roll_dot_, sensor.pitch_dot_, sensor.yaw_dot_);
    
    // Draw and compute control mission parameters
    this->mission_.z_.ref_ = input.depth_d_;
    this->mission_.z_.ref_dot_ = 0.0;
    this->mission_.z_.ref_dot2_ = 0.0;
    this->mission_.z_.Update(); // update previous depth and depth differential
    
    this->mission_.theta_.ref_ = input.pitch_d_ + 0.1 * atan((this->kinetic_.z_ - this->mission_.z_.ref_) / (4 * this->body_.l_));
    this->mission_.theta_.ref_dot_ = (this->mission_.theta_.ref_ - this->mission_.theta_.pre_ref_) / dt;
    this->mission_.theta_.ref_dot2_ = (this->mission_.theta_.ref_dot_ - this->mission_.theta_.pre_ref_dot_) / dt;
    this->mission_.theta_.Update();
    
    this->mission_.lateral_dist_ = (this->kinetic_.x_ - input.x_d_) * sin(input.yaw_d_) - (this->kinetic_.y_ - input.y_d_) * cos(input.yaw_d_);
    printf("LateralDist:%f\n", this->mission_.lateral_dist_);
    
    this->mission_.y_.ref_ = input.y_d_;
    this->mission_.y_.ref_dot_ = 0.0;
    this->mission_.y_.ref_dot2_ = 0.0;
    this->mission_.y_.Update();
    
    this->mission_.psi_.ref_ = input.yaw_d_ + 0.1 * atan((this->kinetic_.y_ - this->mission_.y_.ref_) / (4 * this->body_.l_));
    this->mission_.psi_.ref_dot_ = (this->mission_.psi_.ref_ - this->mission_.psi_.pre_ref_) / dt;
    this->mission_.psi_.ref_dot2_ = (this->mission_.psi_.ref_dot_ - this->mission_.psi_.pre_ref_dot_) / dt;
    this->mission_.psi_.Update();
    
    /* Slide control factor calculating for simplified depth variable */
    // Depth variable substitution
    this->depth_sf_.a_zw_ = this->body_.m_ - this->dynamic_.z_dotw_;
    this->depth_sf_.a_zq_ = -(this->body_.m_ * this->body_.x_g_ + this->dynamic_.z_dotq_);
    this->depth_sf_.a_zs_ = this->force_.z_uuds_ * this->kinetic_.u_ * this->kinetic_.u_;
    this->depth_sf_.f_z_ = this->body_.m_ * this->kinetic_.u_ * this->kinetic_.q_ +
        this->body_.m_ * this->body_.z_g_ * this->kinetic_.q_ * this->kinetic_.q_ -
        this->dynamic_.x_dotu_ * this->kinetic_.u_ * this->kinetic_.q_ +
        this->dynamic_.z_ww_ * this->kinetic_.w_ * fabs(this->kinetic_.w_) +
        this->dynamic_.z_uw_ * this->kinetic_.u_ * this->kinetic_.w_ +
        this->dynamic_.z_qq_ * this->kinetic_.q_ * fabs(this->kinetic_.q_) +
        this->dynamic_.z_uq_ * this->kinetic_.u_ * this->kinetic_.q_ +
        (this->body_.w_ - this->body_.b_) * cos(this->kinetic_.theta_);
    
    // pitch variable substitution
    this->depth_sf_.a_tw_ = -(this->body_.m_ * this->body_.x_g_ + this->dynamic_.m_dotw_);
    this->depth_sf_.a_tq_ = this->body_.i_yy_ - this->dynamic_.m_dotq_;
    this->depth_sf_.a_ts_ = this->force_.m_uuds_ * this->kinetic_.u_ * this->kinetic_.u_;
    this->depth_sf_.f_t_ = -this->body_.m_ * this->body_.z_g_ * this->kinetic_.w_ * this->kinetic_.q_ -
        this->body_.m_ * this->body_.x_g_ * this->kinetic_.u_ * this->kinetic_.q_ -
        (this->dynamic_.z_dotw_ * this->kinetic_.w_ + this->dynamic_.z_dotq_ * this->kinetic_.q_) * this->kinetic_.u_ +
        this->dynamic_.x_dotu_ * this->kinetic_.u_ * this->kinetic_.w_ +
        this->dynamic_.m_ww_ * this->kinetic_.w_ * fabs(this->kinetic_.w_) +
        this->dynamic_.m_qq_ * this->kinetic_.q_ * fabs(this->kinetic_.q_) +
        this->dynamic_.m_uq_ * this->kinetic_.u_ * this->kinetic_.q_ -
        (this->body_.z_g_ * this->body_.w_ - this->body_.z_b_ * this->body_.b_) * sin(this->kinetic_.theta_) -
        (this->body_.x_g_ * this->body_.w_ - this->body_.x_b_ * this->body_.b_) * cos(this->kinetic_.theta_);
   
    // Calculate simplified formula of state in depth
    double deno_depth =  this->depth_sf_.a_zw_ * this->depth_sf_.a_tq_ - this->depth_sf_.a_zq_ * this->depth_sf_.a_tw_;
    this->depth_sf_.b_z_ =  (this->depth_sf_.a_tq_ * this->depth_sf_.f_z_ - this->depth_sf_.a_zq_ * this->depth_sf_.f_t_) / deno_depth; 
    this->depth_sf_.b_zs_ = (this->depth_sf_.a_tq_ * this->depth_sf_.a_zs_ - this->depth_sf_.a_zq_ * this->depth_sf_.a_ts_) / deno_depth;
    this->depth_sf_.b_t_ = (this->depth_sf_.a_zw_ * this->depth_sf_.f_t_ - this->depth_sf_.a_tw_ * this->depth_sf_.f_z_) / deno_depth;
    this->depth_sf_.b_ts_ = (this->depth_sf_.a_zw_ * this->depth_sf_.a_ts_ - this->depth_sf_.a_tw_ * this->depth_sf_.a_zs_) / deno_depth;

    // Calculate quadratic factor of z & pitch
    this->depth_sf_.g_z_ = this->depth_sf_.b_z_ * cos(this->kinetic_.theta_) - this->kinetic_.u_ * this->kinetic_.q_ * cos(this->kinetic_.theta_) -
            this->kinetic_.w_ * this->kinetic_.q_ * sin(this->kinetic_.theta_);
    this->depth_sf_.g_zs_ = this->depth_sf_.b_zs_ * cos(this->kinetic_.theta_);
    this->depth_sf_.g_t_ = this->depth_sf_.b_t_;
    this->depth_sf_.g_ts_ = this->depth_sf_.b_ts_;

    // Update dot z & dot pitch
    this->depth_sf_.dot_z_ = -this->kinetic_.u_ * sin(this->kinetic_.theta_) + this->kinetic_.w_ * cos(this->kinetic_.theta_);
    this->depth_sf_.dot_theta_ = this->kinetic_.q_;


    /* Slide control factor calculating for simplified horizontal plane variable */
    // lateral variable substitution
    this->horizon_sf_.a_yv_ = this->body_.m_ - this->dynamic_.y_dotv_;
    this->horizon_sf_.a_yr_ = this->body_.m_ * this->body_.x_g_ - this->dynamic_.y_dotr_;
    this->horizon_sf_.a_ydr_ = this->force_.y_uudr_ * this->kinetic_.u_ * this->kinetic_.u_;
    this->horizon_sf_.f_y_ = this->body_.m_ * this->body_.y_g_ * this->kinetic_.r_ * this->kinetic_.r_ -
        this->body_.m_ * this->kinetic_.u_ * this->kinetic_.r_ + this->dynamic_.x_dotu_ * this->kinetic_.u_ * this->kinetic_.r_ + 
        this->dynamic_.y_vv_ * this->kinetic_.v_ * fabs(this->kinetic_.v_) +
        this->dynamic_.y_uv_ * this->kinetic_.u_ * this->kinetic_.v_ +
        this->dynamic_.y_rr_ * this->kinetic_.r_ * fabs(this->kinetic_.r_) +
        this->dynamic_.y_ur_ * this->kinetic_.u_ * this->kinetic_.r_;

    // Variable substitution for yaw
    this->horizon_sf_.a_pv_ = this->body_.m_ * this->body_.x_g_ - this->dynamic_.n_dotv_;
    this->horizon_sf_.a_pr_ = this->body_.i_zz_ - this->dynamic_.n_dotr_;
    this->horizon_sf_.a_pdr_ = this->force_.n_uudr_ * this->kinetic_.u_ * this->kinetic_.u_;
    this->horizon_sf_.f_p_ = -this->body_.m_ * this->body_.x_g_ * this->kinetic_.u_ * this->kinetic_.r_ -
        this->body_.m_ * this->body_.y_g_ * this->kinetic_.v_ * this->kinetic_.r_ +
        (this->dynamic_.y_dotv_ * this->kinetic_.v_ + this->dynamic_.y_dotr_ * this->kinetic_.r_) * this->kinetic_.u_ -
        this->dynamic_.x_dotu_ * this->kinetic_.u_ * this->kinetic_.v_ +
        this->dynamic_.n_vv_ * this->kinetic_.v_ * fabs(this->kinetic_.v_) +
        this->dynamic_.n_uv_ * this->kinetic_.u_ * this->kinetic_.v_ +
        this->dynamic_.n_rr_ * this->kinetic_.r_ * fabs(this->kinetic_.r_) +
        this->dynamic_.n_ur_ * this->kinetic_.u_ * this->kinetic_.r_;

    // Calculate simplified formula of state in lateral plane
    double deno_lateral = (this->horizon_sf_.a_yv_ * this->horizon_sf_.a_pr_ - this->horizon_sf_.a_pv_ * this->horizon_sf_.a_yr_);
    this->horizon_sf_.b_y_ = (this->horizon_sf_.a_pr_ * this->horizon_sf_.f_y_ - this->horizon_sf_.a_yr_ * this->horizon_sf_.f_p_) / deno_lateral;
    this->horizon_sf_.b_ydr_ = (this->horizon_sf_.a_pr_ * this->horizon_sf_.a_ydr_ - this->horizon_sf_.a_yr_ * this->horizon_sf_.a_pdr_) / deno_lateral;
    this->horizon_sf_.b_p_ = (this->horizon_sf_.a_yv_ * this->horizon_sf_.f_p_ - this->horizon_sf_.a_pv_ * this->horizon_sf_.f_y_) / deno_lateral;
    this->horizon_sf_.b_pdr_ = (this->horizon_sf_.a_yv_ * this->horizon_sf_.a_pdr_ - this->horizon_sf_.a_pv_ * this->horizon_sf_.a_ydr_) / deno_lateral;

    // Calculate quadratic factor of y & yaw
    this->horizon_sf_.g_y_ = this->horizon_sf_.b_y_ * cos(this->kinetic_.psi_) + this->kinetic_.u_ * this->kinetic_.r_ * cos(this->kinetic_.psi_) -
          this->kinetic_.v_ * this->kinetic_.r_ * sin(this->kinetic_.psi_);
    this->horizon_sf_.g_ydr_ = this->horizon_sf_.b_ydr_ * cos(this->kinetic_.psi_);
    this->horizon_sf_.g_p_ = this->horizon_sf_.b_p_;
    this->horizon_sf_.g_pdr_ = this->horizon_sf_.b_pdr_;
    
    // Update dot z & dot pitch
    this->horizon_sf_.dot_y_ = this->kinetic_.u_ * sin(this->kinetic_.psi_) + this->kinetic_.v_ * cos(this->kinetic_.psi_);
    this->horizon_sf_.dot_psi_ = this->kinetic_.r_;

    /* Slide model control */
    // Depth turnel
    this->slide_model_.z_.e_ = this->kinetic_.z_ - this->mission_.z_.ref_;
    this->slide_model_.z_.dot_e_ = this->depth_sf_.dot_z_ - this->mission_.z_.ref_dot_;
    this->slide_model_.z_.s_ = this->slide_model_.z_.dot_e_ + this->ctrl_.z_.c_ * this->slide_model_.z_.e_;
    // Pitch turnel
    this->slide_model_.theta_.e_ = this->kinetic_.theta_ - this->mission_.theta_.ref_;
    this->slide_model_.theta_.dot_e_ = this->depth_sf_.dot_theta_ - this->mission_.theta_.ref_dot_;
    this->slide_model_.theta_.s_ = this->slide_model_.theta_.dot_e_ + this->ctrl_.theta_.c_ * this->slide_model_.theta_.e_;
    // Lateral dev turnel
    this->slide_model_.y_.e_ = this->kinetic_.y_ - this->mission_.y_.ref_;
    this->slide_model_.y_.dot_e_ = this->horizon_sf_.dot_y_ - this->mission_.y_.ref_dot_;
    this->slide_model_.y_.s_ = this->slide_model_.y_.dot_e_ + this->ctrl_.y_.c_ * this->slide_model_.y_.e_;
    // Yaw turnel
    this->slide_model_.psi_.e_ = this->kinetic_.psi_ - this->mission_.psi_.ref_;
    this->slide_model_.psi_.dot_e_ = this->horizon_sf_.dot_psi_ - this->mission_.psi_.ref_dot_;
    this->slide_model_.psi_.s_ = this->slide_model_.psi_.dot_e_ + this->ctrl_.psi_.c_ * this->slide_model_.psi_.e_;

    // Intermediate quantity computation
    this->slide_model_.z_.l_ = this->mission_.z_.ref_dot2_ - this->depth_sf_.g_z_ - this->ctrl_.z_.c_ * this->slide_model_.z_.dot_e_ -
        this->ctrl_.z_.k_ * pow(fabs(this->slide_model_.z_.s_), this->ctrl_.z_.alpha_) * sat(this->slide_model_.z_.s_, this->ctrl_.bondary_thick_);
    this->slide_model_.theta_.l_ = this->mission_.theta_.ref_dot2_ - this->depth_sf_.g_t_ - this->ctrl_.theta_.c_ * this->slide_model_.theta_.dot_e_ -
        this->ctrl_.theta_.k_ * pow(fabs(this->slide_model_.theta_.s_), this->ctrl_.theta_.alpha_) * sat(this->slide_model_.theta_.s_, this->ctrl_.bondary_thick_);
    this->slide_model_.y_.l_ = this->mission_.y_.ref_dot2_ - this->horizon_sf_.g_y_ - this->ctrl_.y_.c_ * this->slide_model_.y_.dot_e_ -
        this->ctrl_.y_.k_ * pow(fabs(this->slide_model_.y_.s_), this->ctrl_.y_.alpha_) * sat(this->slide_model_.y_.s_, this->ctrl_.bondary_thick_);
    this->slide_model_.psi_.l_ = this->mission_.psi_.ref_dot2_ - this->horizon_sf_.b_p_ - this->ctrl_.psi_.c_ * this->slide_model_.psi_.dot_e_ -
        this->ctrl_.psi_.k_ * pow(fabs(this->slide_model_.psi_.s_), this->ctrl_.psi_.alpha_) * sat(this->slide_model_.psi_.s_, this->ctrl_.bondary_thick_);

    // Caculate target rudder angle
    double deno_depth_ctrl = this->depth_sf_.g_zs_ - this->depth_sf_.g_ts_;
    this->deltas_ = (this->slide_model_.z_.l_- this->slide_model_.theta_.l_) / deno_depth_ctrl;

    double deno_lateral_ctrl = this->horizon_sf_.g_ydr_ - this->horizon_sf_.g_pdr_;
    this->deltar_ = (this->slide_model_.y_.l_ - this->slide_model_.psi_.l_) / deno_lateral_ctrl;

    this->deltab_ = 0;
    if(fabs(this->deltas_) > 30 / 57.3){
        this->deltas_ = (30 / 57.3) * sign(this->deltas_);
    }
    if(fabs(this->deltar_) > 30 / 57.3){
        this->deltar_ = (30 / 57.3) * sign(this->deltar_);
    }
  
    // Print control value of forward, afterward and orientation rouder
    printf("afterward fin: %f vertical fin: %f", this->deltas_, this->deltar_);
    output.fwd_fin_ = this->deltab_;
    output.aft_fin_ = this->deltas_;
    output.rouder_ = this->deltar_;
}

}; // ns
