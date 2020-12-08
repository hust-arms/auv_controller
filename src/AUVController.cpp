/*
 * Filename: AUVController.cpp
 * Path: auv_controller
 * Created Date: Tuesday, September 8th 2020, 2:53:16 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2020 Your Company
 */

#include <math.h>
#include "auv_controller/AUVController.h"

namespace auv_controller{
    AUVController::AUVController(){
        defaultInit();
    }

    /**
     * @brief Set AUV body parameters whicih includes mass, length, gravity, buoyancy et al.
     */
    bool AUVController::setAUVBodyParams(const std::vector<double>& body){
        if(body.size() == body_num_){
            setAUVBodyParams(body[0], body[1], body[2], body[3], body[4], body[5], body[6],
                body[7], body[8], body[9], body[10], body[11], body[12]);
            return true;
        }
        else{
            return false;
        }
    };

    /**
     * @brief Set AUV hydrodyanamic parameters 
     */ 
    bool AUVController::setAUVDynamic(const std::vector<double>& dynamic){
        if(dynamic.size() == dynamic_num_){
            setAUVDynamic(dynamic[0], dynamic[1], dynamic[2], dynamic[3], dynamic[4], dynamic[5],
                dynamic[6], dynamic[7], dynamic[8], dynamic[9], dynamic[10], dynamic[11], dynamic[12], 
                dynamic[13], dynamic[14], dynamic[15], dynamic[16], dynamic[17], dynamic[18], dynamic[19],
                dynamic[20], dynamic[21], dynamic[22], dynamic[23], dynamic[24], dynamic[25], dynamic[26],
                dynamic[27]);
            return true;
        }
        else{
            return false;
        }
    };

    /**
     * @brief Set control parameters
     */ 
    bool AUVController::setCtrlParams(const std::vector<double>& ctrl){
        if(ctrl.size() == ctrl_num_){
            setCtrlParams(ctrl[0], ctrl[1], ctrl[2], ctrl[3], ctrl[4], ctrl[5], ctrl[6], ctrl[7], ctrl[8], ctrl[9]);
            return true;
        }
        else{
            return false;
        }
    }

    /**
     * @brief Set force parameters
     */ 
    bool AUVController::setForceParams(const std::vector<double>& force){
        if(force.size() == force_num_){
            setForceParams(force[0], force[1], force[2], force[3], force[4], force[5]);
            return true;
        }
        return false;
    }

    /**
     * @brief Initialize with default paramters
     */ 
    void AUVController::defaultInit(){
        setAUVBodyParams(84.71, 2.712, 831, 838, 0, 0, 0, 0, 0, 0.0086, 0.82, 30.14, 30.14);
        setAUVDynamic(-1.432, -120.645, -4.895, -130.513, 16.488,
            -0.386, 16.488, -78.266, -4.895, -67.489,
            -3.9, -373.287, -4.204, -489.07, 23.016,
            -0.1177, 23.342, -353.406, 0.4193, -227.024,
            -130.64, 40.25, -522.87, 4.27,
            140.68, 73, -57.47, -50.3);
        setForceParams(38.279, -38.279, -44.981,
            41.686, -44.531, -41.686);
        setCtrlParams(0.08, 0.1, 0.6, 0.1, 0.1, 0.6, 0.8, 0.8, 0.8, 0.1);
    }

    /**
     * @brief Set control parameters
     */
    void AUVController::setCtrlParams(double cz, double kz, double alphaz,
            double ctheta, double ktheta, double alphatheta,
            double cpsi, double kpsi, double alphapsi, double bt)
    {  
        ctrl_.z_.setParameters(cz, kz, alphaz); // depth 
        ctrl_.theta_.setParameters(ctheta, ktheta, alphatheta); // pitch
        ctrl_.psi_.setParameters(cpsi, kpsi, alphapsi); // yaw
        ctrl_.bondary_thick_ = bt;
    };

    /**
     * @brief Set control parameters
     */
    void AUVController::setCtrlParams(double c, double k, double alpha, unsigned int flag)
    {
        switch (flag)
        {
        case 0:
            ctrl_.z_.setParameters(c, k, alpha);
            break;
        case 1:
            ctrl_.theta_.setParameters(c, k, alpha);
            break;
        case 2:
            ctrl_.psi_.setParameters(c, k, alpha);
            break;
        default:
            break;
        }
    }

    /**
     * @brief Serialize auv body parameters
     */ 
    void AUVController::serializeAUVBodyParams(std::stringstream& str){
        str << "m: " << body_.m_ << " l: " << body_.l_ << " w: " << body_.w_ << " b: " << body_.b_;
        str << " xb: " << body_.x_b_ << " yb: " << body_.y_b_ << " zb: " << body_.z_b_;
        str << " xg: " << body_.x_g_ << " yg: " << body_.y_g_ << " zg: " << body_.z_g_;
        str << " ixx: " << body_.i_xx_ << " iyy: " << body_.i_yy_ << " izz: " << body_.i_zz_;
    }

    /**
     * @brief Serialize auv dynamic parameters
     */ 
    void AUVController::serializeAUVDynamicParams(std::stringstream& str){
        str << "xdotu: " << dynamic_.x_dotu_ << " ydotv: " << dynamic_.y_dotv_ <<" ydotr: " << dynamic_.y_dotr_ << " zdotw: " << dynamic_.z_dotw_ << " zdotq: " << dynamic_.z_dotq_;
        str << " kdotp: " << dynamic_.k_dotp_ << " mdotw: " << dynamic_.m_dotw_ << " mdotq: " << dynamic_.m_dotq_ << " ndotv: " << dynamic_.n_dotv_ << " ndotr: " << dynamic_.n_dotr_;
        str << " xuu: " << dynamic_.x_uu_ << " yvv: " << dynamic_.y_vv_ << " yrr: " << dynamic_.y_rr_ << " zww: " << dynamic_.z_ww_ << " zqq: " << dynamic_.z_qq_;
        str << " kpp: " << dynamic_.k_pp_ << " mww: " << dynamic_.m_ww_ << " mqq: " << dynamic_.m_qq_ << " nvv: " << dynamic_.n_vv_ << " nrr: " << dynamic_.n_rr_;
        str << " yuv: " << dynamic_.y_uv_ << " yur: " << dynamic_.y_ur_ << " zuw: " << dynamic_.z_uw_ << " zuq: " << dynamic_.z_uq_;
        str << " muw: " << dynamic_.m_uw_ << " muq: " << dynamic_.m_uq_ << " nuv: " << dynamic_.n_uv_ << " nur: " << dynamic_.n_ur_;
    } 

    /**
     * @brief Serialize auv force parameters
     */ 
    void AUVController::serializeAUVForceParams(std::stringstream& str){
        str << "yuudr: " << force_.y_uudr_ << " zuuds: " << force_.z_uuds_ << " zuudb: " << force_.z_uudb_;
        str << " muuds: " << force_.m_uuds_ << " muudb: " << force_.m_uudb_ << " nuudr: " << force_.n_uudr_;
    }

    /**
     * @brief Serialize auv control parameters
     */ 
    void AUVController::serializeAUVControlParams(std::stringstream& str){
        str << "cz: " << ctrl_.z_.c_ << " kz: " << ctrl_.z_.k_ << " alphaz: " << ctrl_.z_.alpha_;
        str << " ctheta: " << ctrl_.theta_.c_ << " ktheta: " << ctrl_.theta_.k_ << " alphatheta: " << ctrl_.theta_.alpha_;
        str << " cpsi: " << ctrl_.psi_.c_ << " kpsi: " << ctrl_.psi_.k_ << " alphapsi: " << ctrl_.psi_.alpha_;
        str << " boundary thick: " << ctrl_.bondary_thick_;
    }

    /**
     * @brief Control solution
     */
    void AUVController::controllerRun(const AUVKineticSensor& sensor, const AUVControllerInput& input, AUVControllerOutput& output, const double dt)
    {
        // Update kinetic parameters
        kinetic_.setPosition(sensor.x_, sensor.y_, sensor.z_, sensor.roll_, sensor.pitch_, sensor.yaw_);
        kinetic_.setVelocity(sensor.x_dot_, sensor.y_dot_, sensor.z_dot_, sensor.roll_dot_, sensor.pitch_dot_, sensor.yaw_dot_);

        // Draw and compute control mission parameters
        mission_.z_.ref_ = input.depth_d_;
        mission_.z_.ref_dot_ = 0;
        mission_.z_.ref_dot2_ = 0;
        mission_.z_.Update(); // update previous depth and depth differential

        mission_.theta_.ref_ = input.pitch_d_ + 0.1 * atan((kinetic_.z_ - mission_.z_.ref_) / (4 * body_.l_));
        mission_.theta_.ref_dot_ = (mission_.theta_.ref_ - mission_.theta_.pre_ref_) / dt;
        mission_.theta_.ref_dot2_ = (mission_.theta_.ref_dot_ - mission_.theta_.pre_ref_dot_) / dt;
        mission_.theta_.Update();

        mission_.lateral_dist_ = (kinetic_.x_ - input.x_d_) * sin(input.yaw_d_) - (kinetic_.y_ - input.y_d_) * cos(input.yaw_d_);
        printf("LateralDist:%f\n", mission_.lateral_dist_);

        mission_.psi_.ref_ = input.yaw_d_ + atan(mission_.lateral_dist_ / 10);
        mission_.psi_.ref_dot_ = (mission_.psi_.ref_ - mission_.psi_.pre_ref_) / dt;
        mission_.psi_.ref_dot2_ = (mission_.psi_.ref_dot_ - mission_.psi_.pre_ref_dot_) / dt;
        mission_.psi_.Update();

        // Lateral variable substitution
        depth_sf_.a_zw_ = body_.m_ - dynamic_.z_dotw_;
        depth_sf_.a_zq_ = -(body_.m_ * body_.x_g_ + dynamic_.z_dotq_);
        depth_sf_.a_zs_ = force_.z_uuds_ * kinetic_.u_ * kinetic_.u_;
        depth_sf_.a_zb_ = force_.z_uudb_ * kinetic_.u_ * kinetic_.u_;
        depth_sf_.f_z_ = body_.m_ * kinetic_.u_ * kinetic_.q_ + 
            body_.m_ * body_.z_g_ * kinetic_.q_ * kinetic_.q_ - 
            dynamic_.x_dotu_ * kinetic_.u_ * kinetic_.q_ + 
            dynamic_.z_ww_ * kinetic_.w_ * fabs(kinetic_.w_) +
            dynamic_.z_uw_ * kinetic_.u_ * kinetic_.w_ +
            dynamic_.z_qq_ * kinetic_.q_ * fabs(kinetic_.q_) + 
            dynamic_.z_uq_ * kinetic_.u_ * kinetic_.q_ +
            (body_.w_ - body_.b_) * cos(kinetic_.theta_);
        
        // Pitch variable substitution
        depth_sf_.a_tw_ = -(body_.m_ * body_.x_g_ + dynamic_.m_dotw_);
        depth_sf_.a_tq_ = body_.i_yy_ - dynamic_.m_dotq_;
        depth_sf_.a_ts_ = force_.m_uuds_ * kinetic_.u_ * kinetic_.u_;
        depth_sf_.a_tb_ = force_.m_uudb_ * kinetic_.u_ * kinetic_.u_;
        depth_sf_.f_t_ = -body_.m_ * body_.z_g_ * kinetic_.w_ * kinetic_.q_ - 
            body_.m_ * body_.x_g_ * kinetic_.u_ * kinetic_.q_ - 
            (dynamic_.z_dotw_ * kinetic_.w_ + dynamic_.z_dotq_ * kinetic_.q_) * kinetic_.u_ +
            dynamic_.x_dotu_ * kinetic_.u_ * kinetic_.w_ + 
            dynamic_.m_ww_ * kinetic_.w_ * fabs(kinetic_.w_) + 
            dynamic_.m_qq_ * kinetic_.q_ * fabs(kinetic_.q_) + 
            dynamic_.m_uq_ * kinetic_.u_ * kinetic_.q_ - 
            (body_.z_g_ * body_.w_ - body_.z_b_ * body_.b_) * sin(kinetic_.theta_) - 
            (body_.x_g_ * body_.w_ - body_.x_b_ * body_.b_) * cos(kinetic_.theta_);

        // dot_w & dot_q
        double common1 = depth_sf_.a_zw_ * depth_sf_.a_tq_ - depth_sf_.a_zq_ * depth_sf_.a_tw_;
        depth_sf_.b_z_ = (depth_sf_.a_tq_ * depth_sf_.f_z_ - depth_sf_.a_zq_ * depth_sf_.f_t_) / common1;
        depth_sf_.b_zb_ = (depth_sf_.a_tq_ * depth_sf_.a_zb_ - depth_sf_.a_zq_ * depth_sf_.a_tb_) / common1;
        depth_sf_.b_zs_ = (depth_sf_.a_tq_ * depth_sf_.a_zs_ - depth_sf_.a_zq_ * depth_sf_.a_ts_) / common1;
        depth_sf_.b_t_ = (depth_sf_.a_zw_ * depth_sf_.f_t_ - depth_sf_.a_tw_ * depth_sf_.f_z_) / common1;
        depth_sf_.b_tb_ = (depth_sf_.a_zw_ * depth_sf_.a_tb_ - depth_sf_.a_tw_ * depth_sf_.a_zb_) / common1;
        depth_sf_.b_ts_ = (depth_sf_.a_zw_ * depth_sf_.a_ts_ - depth_sf_.a_tw_ * depth_sf_.a_zs_) / common1;
        printf("common1:%f\n", common1);

        // dot2_z & dot2_theta
        depth_sf_.g_z_ = depth_sf_.b_z_ * cos(kinetic_.theta_) - kinetic_.u_ * kinetic_.q_ * cos(kinetic_.theta_) - 
            kinetic_.w_ * kinetic_.q_ * sin(kinetic_.theta_);
        depth_sf_.g_zb_ = depth_sf_.b_zb_ * cos(kinetic_.theta_);
        depth_sf_.g_zs_ = depth_sf_.b_zs_ * cos(kinetic_.theta_);
        depth_sf_.g_t_ = depth_sf_.b_t_;
        depth_sf_.g_tb_ = depth_sf_.b_tb_;
        depth_sf_.g_ts_ = depth_sf_.b_ts_;
        printf("Temp:{g_zb:%f g_ts:%f g_tb:%f g_zs:%f}\n", depth_sf_.g_zb_, depth_sf_.g_ts_, depth_sf_.g_tb_, depth_sf_.g_zs_);
        printf("Temp:{b_z:%f theta:%f u:%f q:%f w:%f b_zb:%f b_zs:%f b_t:%f b_tb%f b_ts%f\n}", 
               depth_sf_.b_z_, kinetic_.theta_, kinetic_.u_, kinetic_.q_, kinetic_.w_, depth_sf_.b_zb_, depth_sf_.b_zs_, depth_sf_.b_t_, depth_sf_.b_tb_, depth_sf_.b_ts_);
        
        // depth & theta_dot
        depth_sf_.dot_z_ = -kinetic_.u_ * sin(kinetic_.theta_) + kinetic_.w_ * cos(kinetic_.theta_);
        depth_sf_.dot_theta_ = kinetic_.q_;

        // Horizontal variable substitution
        horizon_sf_.a_yv_ = body_.m_ - dynamic_.y_dotv_;
        horizon_sf_.a_yr_ = body_.m_ * body_.x_g_ - dynamic_.y_dotr_;
        horizon_sf_.a_ydr_ = force_.y_uudr_ * kinetic_.u_ * kinetic_.u_;
        horizon_sf_.f_y_ = body_.m_ * body_.y_g_ * kinetic_.r_ * kinetic_.r_ - 
            body_.m_ * kinetic_.u_ * kinetic_.r_ + dynamic_.x_dotu_ * kinetic_.u_ * kinetic_.r_ +
            dynamic_.y_vv_ * kinetic_.v_ * fabs(kinetic_.v_) + 
            dynamic_.y_uv_ * kinetic_.u_ * kinetic_.v_ + 
            dynamic_.y_rr_ * kinetic_.r_ * fabs(kinetic_.r_) + 
            dynamic_.y_ur_ * kinetic_.u_ * kinetic_.r_;

        // Rolling variable substitution 
        horizon_sf_.a_pv_ = body_.m_ * body_.x_g_ - dynamic_.n_dotv_;
        horizon_sf_.a_pr_ = body_.i_zz_ - dynamic_.n_dotr_;
        horizon_sf_.a_pdr_ = force_.n_uudr_ * kinetic_.u_ * kinetic_.u_;
        horizon_sf_.f_p_ = -body_.m_ * body_.x_g_ * kinetic_.u_ * kinetic_.r_ - 
            body_.m_ * body_.y_g_ * kinetic_.v_ * kinetic_.r_ + 
            (dynamic_.y_dotv_ * kinetic_.v_ + dynamic_.y_dotr_ * kinetic_.r_) * kinetic_.u_ - 
            dynamic_.x_dotu_ * kinetic_.u_ * kinetic_.v_ +
            dynamic_.n_vv_ * kinetic_.v_ * fabs(kinetic_.v_) + 
            dynamic_.n_uv_ * kinetic_.u_ * kinetic_.v_ + 
            dynamic_.n_rr_ * kinetic_.r_ * fabs(kinetic_.r_) + 
            dynamic_.n_ur_ * kinetic_.u_ * kinetic_.r_;

        // dot_v & dot_r
        double common2 = (horizon_sf_.a_yv_ * horizon_sf_.a_pr_ - horizon_sf_.a_pv_ * horizon_sf_.a_yr_);
        horizon_sf_.b_y_ = (horizon_sf_.a_pr_ * horizon_sf_.f_y_ - horizon_sf_.a_yr_ * horizon_sf_.f_p_) / common2;
        horizon_sf_.b_ydr_ = (horizon_sf_.a_pr_ * horizon_sf_.a_ydr_ - horizon_sf_.a_yr_ * horizon_sf_.a_pdr_) / common2;
        horizon_sf_.b_p_ = (horizon_sf_.a_yv_ * horizon_sf_.f_p_ - horizon_sf_.a_pv_ * horizon_sf_.f_y_) / common2;
        horizon_sf_.b_pdr_ = (horizon_sf_.a_yv_ * horizon_sf_.a_pdr_ - horizon_sf_.a_pv_ * horizon_sf_.a_ydr_) / common2;
        printf("common2:%f\n", common2);

        // dot_psi
        horizon_sf_.dot_psi_ = kinetic_.r_;

        // Slide Model Control
        // Depth
        slide_model_.z_.e_ = kinetic_.z_ - mission_.z_.ref_;
        slide_model_.z_.dot_e_ = depth_sf_.dot_z_ - mission_.z_.ref_dot_;
        slide_model_.z_.s_ = slide_model_.z_.dot_e_ + ctrl_.z_.c_ * slide_model_.z_.e_;
        // Pitch
        slide_model_.theta_.e_ = kinetic_.theta_ - mission_.theta_.ref_;
        slide_model_.theta_.dot_e_ = depth_sf_.dot_theta_ - mission_.theta_.ref_dot_;
        slide_model_.theta_.s_ = slide_model_.theta_.dot_e_ + ctrl_.theta_.c_ * slide_model_.theta_.e_;
        // Yaw
        slide_model_.psi_.e_ = kinetic_.psi_ - mission_.psi_.ref_;
        slide_model_.psi_.dot_e_ = horizon_sf_.dot_psi_ - mission_.psi_.ref_dot_;
        slide_model_.psi_.s_ = slide_model_.psi_.dot_e_ + ctrl_.psi_.c_ * slide_model_.psi_.e_;

        // Intermediate quantity computation
        slide_model_.z_.l_ = mission_.z_.ref_dot2_ - depth_sf_.g_z_ - ctrl_.z_.c_ * slide_model_.z_.dot_e_ - 
            ctrl_.z_.k_ * pow(fabs(slide_model_.z_.s_), ctrl_.z_.alpha_) * sat(slide_model_.z_.s_, ctrl_.bondary_thick_);
        slide_model_.theta_.l_ = mission_.theta_.ref_dot2_ - depth_sf_.g_t_ - ctrl_.theta_.c_ * slide_model_.theta_.dot_e_ - 
            ctrl_.theta_.k_ * pow(fabs(slide_model_.theta_.s_), ctrl_.theta_.alpha_) * sat(slide_model_.theta_.s_, ctrl_.bondary_thick_);
        slide_model_.psi_.l_ = mission_.psi_.ref_dot2_ - horizon_sf_.b_p_ - ctrl_.psi_.c_ * slide_model_.psi_.dot_e_ - 
            ctrl_.psi_.k_ * pow(fabs(slide_model_.psi_.s_), ctrl_.psi_.alpha_) * sat(slide_model_.psi_.s_, ctrl_.bondary_thick_);

        // Command computation
        double common3 = depth_sf_.g_zb_ * depth_sf_.g_ts_ - depth_sf_.g_tb_ * depth_sf_.g_zs_;
        printf("Temp:{g_zb:%f g_ts:%f g_tb:%f g_zs:%f}\n", depth_sf_.g_zb_, depth_sf_.g_ts_, depth_sf_.g_tb_, depth_sf_.g_zs_);
        deltab_ = (slide_model_.z_.l_ * depth_sf_.g_ts_ - slide_model_.theta_.l_ * depth_sf_.g_zs_) / common3;
        deltas_ = (slide_model_.theta_.l_ * depth_sf_.g_zb_ - slide_model_.z_.l_ * depth_sf_.g_tb_) / common3;
        deltar_ = slide_model_.psi_.l_ / horizon_sf_.b_pdr_;
        printf("common3:%f\n", common3);

        if(fabs(deltab_) > 30 / 57.3){
            deltab_ = (30 / 57.3) * sign(deltab_);
        }
        if(fabs(deltas_) > 30 / 57.3){
            deltas_ = (30 / 57.3) * sign(deltas_);
        }
        if(fabs(deltar_) > 30 / 57.3){
            deltar_ = (30 / 57.3) * sign(deltar_);
        }

        // Print control value of forward, afterward and orientation rouder
        // printf("forward fin: %f afterward fin: %f rouder: %f", deltab_, deltas_, deltar_);

        output.fwd_fin_ = deltab_;
        output.aft_fin_ = deltas_;
        output.rouder_ = deltar_;
    };
}; // ns
