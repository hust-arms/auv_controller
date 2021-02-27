/*                                                                                                                                               
 * Filename: AUVBaseController.cpp
 * Path: auv_controller
 * Created Date: Thursday, Janurary 28th 2021, 15:59:16
 * Author: zhao wang
 * 
 * Copyright (c) 2021 Your Company
 */

#include <math.h>
#include "auv_controller/AUVBaseController.h"

namespace auv_controller{
AUVBaseController::AUVBaseController(){
    defaultInit();
}

/**
 * @brief Set AUV body parameters whicih includes mass, length, gravity, buoyancy et al.
 */
bool AUVBaseController::setAUVBodyParams(const std::vector<double>& body){
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
bool AUVBaseController::setAUVDynamic(const std::vector<double>& dynamic){
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
void AUVBaseController::setCtrlParams(double cz, double kz, double alphaz, double ctheta, double ktheta, double alphatheta,
        double cy, double ky, double alphay, double cpsi, double kpsi, double alphapsi, double bt)
{
    ctrl_.z_.setParameters(cz, kz, alphaz); // depth
    ctrl_.theta_.setParameters(ctheta, ktheta, alphatheta); // pitch
    ctrl_.y_.setParameters(cy, ky, alphay); // y
    ctrl_.psi_.setParameters(cpsi, kpsi, alphapsi); // yaw
    ctrl_.bondary_thick_ = bt;
};


/**
 * @brief Set control parameters
 */
bool AUVBaseController::setCtrlParams(const std::vector<double>& ctrl){
    if(ctrl.size() == ctrl_num_){
        setCtrlParams(ctrl[0], ctrl[1], ctrl[2], ctrl[3], ctrl[4], ctrl[5], ctrl[6], ctrl[7], ctrl[8], ctrl[9], ctrl[10], ctrl[11], ctrl[12]);
        return true;
    }
    else{
        return false;
    }
}

/**
 * @brief Set control parameters
 */
void AUVBaseController::setCtrlParams(double c, double k, double alpha, unsigned int flag)
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
        ctrl_.y_.setParameters(c, k, alpha);
    case 3:
        ctrl_.psi_.setParameters(c, k, alpha);
        break;
    default:
        break;
    }
}

/**
 * @brief Set force parameters
 */ 
bool AUVBaseController::setForceParams(const std::vector<double>& force){
    if(force.size() == force_num_){
        setForceParams(force[0], force[1], force[2], force[3], force[4], force[5]);
        return true;
    }
    return false;
}

/**
 * @brief Set X force parameters
 */ 
bool AUVBaseController::setXForceParams(const std::vector<double>& force){
    if(force.size() == xforce_num_){
        setXForceParams(force[0], force[1], force[2], force[3], force[4], force[5], force[6], force[7],
                        force[8], force[9], force[10], force[11], force[12], force[13], force[14], force[15]);
        return true;
    }
    return false;
}


/**
 * @brief Initialize with default paramters
 */
void AUVBaseController::defaultInit(){
    setAUVBodyParams(84.71, 2.712, 831, 838, 0, 0, 0, 0, 0, 0.0086, 0.82, 30.14, 30.14);
    setAUVDynamic(-1.432, -120.645, -4.895, -130.513, 16.488,
        -0.386, 16.488, -78.266, -4.895, -67.489,
        -3.9, -373.287, -4.204, -489.07, 23.016,
        -0.1177, 23.342, -353.406, 0.4193, -227.024,
        -130.64, 40.25, -522.87, 4.27,
        140.68, 73, -57.47, -50.3);
    setForceParams(38.279, -38.279, -44.981,
        41.686, -44.531, -41.686);
    // setThrusterFactor(2.4e-5, 888.0, -888.0, -1.0);
    setThrusterFactor(0.0875, 888.0, -888.0, -1.0);
    setCtrlParams(0.1, 0.1, 0.1, 0.1, 0.1, 0.6, 0.1, 0.1, 0.1, 0.8, 0.8, 0.1, 0.1); // with front fins
    // setCtrlParams(0.5, 0.5, 0.1, 0.5, 0.5, 0.1, 0.5, 0.5, 0.1, 0.8, 0.8, 0.1, 0.1); // without front fins
    //
    vel_controller_ = boost::make_shared<PIDController>(1000.0, 0.0, 0.0);

    depth_sf_.init();
    horizon_sf_.init();

    /* Initialize mission control variable */
    mission_.z_.ref_ = 0.0;
    mission_.z_.ref_dot_ = 0.0;
    mission_.z_.ref_dot2_ = 0.0;
    mission_.z_.pre_ref_ = 0.0;
    mission_.z_.pre_ref_dot_ = 0.0;

    mission_.theta_.ref_ = 0.0;
    mission_.theta_.ref_dot_ = 0.0;
    mission_.theta_.ref_dot2_ = 0.0;
    mission_.theta_.pre_ref_ = 0.0;
    mission_.theta_.pre_ref_dot_ = 0.0;

    mission_.y_.ref_ = 0.0;
    mission_.y_.ref_dot_ = 0.0;
    mission_.y_.ref_dot2_ = 0.0;
    mission_.y_.pre_ref_ = 0.0;
    mission_.y_.pre_ref_dot_ = 0.0;

    mission_.psi_.ref_ = 0.0;
    mission_.psi_.ref_dot_ = 0.0;
    mission_.psi_.ref_dot2_ = 0.0;
    mission_.psi_.pre_ref_ = 0.0;
    mission_.psi_.pre_ref_dot_ = 0.0;

    deltab_ = 0.0; deltas_ = 0.0; deltar_ = 0.0;
    deltaup_ = 0.0; deltaus_ = 0.0; deltalp_ = 0.0; deltals_ = 0.0;
}

/**
 * @brief Serialize auv body parameters
 */ 
void AUVBaseController::serializeAUVBodyParams(std::stringstream& str){
    str << "m: " << body_.m_ << " l: " << body_.l_ << " w: " << body_.w_ << " b: " << body_.b_;
    str << " xb: " << body_.x_b_ << " yb: " << body_.y_b_ << " zb: " << body_.z_b_;
    str << " xg: " << body_.x_g_ << " yg: " << body_.y_g_ << " zg: " << body_.z_g_;
    str << " ixx: " << body_.i_xx_ << " iyy: " << body_.i_yy_ << " izz: " << body_.i_zz_;
}

/**
 * @brief Serialize auv dynamic parameters
 */ 
void AUVBaseController::serializeAUVDynamicParams(std::stringstream& str){
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
void AUVBaseController::serializeAUVForceParams(std::stringstream& str){
    str << "yuudr: " << force_.y_uudr_ << " zuuds: " << force_.z_uuds_ << " zuudb: " << force_.z_uudb_;
    str << " muuds: " << force_.m_uuds_ << " muudb: " << force_.m_uudb_ << " nuudr: " << force_.n_uudr_;
}

void AUVBaseController::serializeAUVXForceParams(std::stringstream& str){
    str << "ydup: " << xforce_.y_dup_ << " ydus: " << xforce_.y_dus_ << " ydlp: " << xforce_.y_dlp_ << " yls: " << xforce_.y_dls_;
    str << "zdup: " << xforce_.z_dup_ << " zdus: " << xforce_.z_dus_ << " zdlp: " << xforce_.z_dlp_ << " zls: " << xforce_.z_dls_;
    str << "mdup: " << xforce_.m_dup_ << " mdus: " << xforce_.m_dus_ << " mdlp: " << xforce_.m_dlp_ << " mls: " << xforce_.m_dls_;
    str << "ndun: " << xforce_.n_dup_ << " ndus: " << xforce_.n_dus_ << " ndln: " << xforce_.n_dlp_ << " nls: " << xforce_.n_dls_;
}

/**
 * @brief Serialize auv control parameters
 */
void AUVBaseController::serializeAUVControlParams(std::stringstream& str){
    str << "cz: " << ctrl_.z_.c_ << " kz: " << ctrl_.z_.k_ << " alphaz: " << ctrl_.z_.alpha_;
    str << " ctheta: " << ctrl_.theta_.c_ << " ktheta: " << ctrl_.theta_.k_ << " alphatheta: " << ctrl_.theta_.alpha_;
    str << " cy: " << ctrl_.y_.c_ << " ky: " << ctrl_.y_.k_ << " alphay: " << ctrl_.y_.alpha_;
    str << " cpsi: " << ctrl_.psi_.c_ << " kpsi: " << ctrl_.psi_.k_ << " alphapsi: " << ctrl_.psi_.alpha_;
    str << " boundary thick: " << ctrl_.bondary_thick_;
}

}; // ns

