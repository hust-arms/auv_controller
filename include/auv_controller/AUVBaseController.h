/*
 * Filename: AUVBaseController.h
 * Path: auv_controller
 * Created Date: Thursday, Janurary 28th 2021, 15:14:39         
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#ifndef AUV_BASE_CONTROLLER_H_
#define AUV_BASE_CONTROLLER_H_

#include <vector>
#include <string>
#include <sstream>
#include "BodyParams.h"
#include "ForceParams.h"
#include "CtrlParams.h"
#include "Dynamic.h"
#include "AUVCommon.h"

namespace auv_controller{

/**
 * @brief AUV controller input parameters
 */
struct AUVControllerInput{
    /**
     * @brief Constructor
     */
    AUVControllerInput(double depthd, double pitchd, double yawd, double xd, double yd) :
        depth_d_(depthd), pitch_d_(pitchd), yaw_d_(yawd), x_d_(xd), y_d_(yd){};

    double depth_d_; // desired depth
    double pitch_d_; // desired pitch
    double yaw_d_; // desired yaw
    double x_d_; // waypoint on the desired line
    double y_d_; // waypoint on the desired line
}; //  AUVControllerInput

/**
  * @brief AUV controller output parameters
  */
struct AUVControllerOutput{
    double rouder_;
    double fwd_fin_;
    double aft_fin_;
}; // AUVControllerOutput


/**
 * @brief Base auv slide model controller 
 */ 
class AUVBaseController
{
public:
    /**
     * @brief Default class initialization
     */
    AUVBaseController();

    /**
     * @brief Deconstructor
     */
    virtual ~AUVBaseController() {} 

    /**
     * @brief Set AUV body parameters whicih includes mass, length, gravity, buoyancy et al
     */
    void setAUVBodyParams(double m, double l, double w, double b,
        double xb, double yb, double zb, double xg, double yg, double zg,
        double ixx, double iyy, double izz)
    {
        body_.setParameters(m, l, w, b, xb, yb, zb, xg, yg, zg, ixx, iyy, izz);
    };

    /**
     * @brief Set AUV body parameters whicih includes mass, length, gravity, buoyancy et al
     */
    bool setAUVBodyParams(const std::vector<double>& body);

    /**
     * @brief Set AUV hydrodyanamic parameters
     */
    void setAUVDynamic(double xdotu, double ydotv, double ydotr, double zdotw, double zdotq,
        double kdotq, double mdotw, double mdotq, double ndotv, double ndotr,
        double xuu, double yvv, double yrr, double zww, double zqq,
        double kpp, double mww, double mqq, double nvv, double nrr,
        double yuv, double yur, double zuw, double zuq,
        double muw, double muq, double nuv, double nur)
    {
        dynamic_.setParameters(xdotu, ydotv, ydotr, zdotw, zdotq, kdotq, mdotw, mdotq, ndotv, ndotr,
                    xuu, yvv, yrr, zww, zqq, kpp, mww, mqq, nvv, nrr, yuv, yur, zuw, zuq, muw, muq, nuv, nur);
    };

    /**
     * @brief Set AUV hydrodyanamic parameters
     */
    bool setAUVDynamic(const std::vector<double>& dynamic);

    /**
     * @brief Set control parameters
     */
    void setCtrlParams(double cz, double kz, double alphaz,
        double ctheta, double ktheta, double alphatheta,
        double cy, double ky, double alphay,
        double cpsi, double kpsi, double alphapsi, double bt);

    /**
     * @brief Set control parameters
     */ 
    bool setCtrlParams(const std::vector<double>& ctrl);
    
    /**
     * @brief Set control parameters
     */
    void setCtrlParams(double c, double k, double alpha, unsigned int flag);

    /**
     * @brief Set boundary thick
     */
    void setBoundaryThick(double bt){
        ctrl_.bondary_thick_ = bt;
    }

    /**
     * @brief Set force parameters
     */ 
    void setForceParams(double yuudr, double zuuds, double zuudb, double muuds, double muudb, double nuudr)
    {
        force_.setParameters(yuudr, zuuds, zuudb, muuds, muudb, nuudr);
    };
    
    /**
     * @brief Set force parameters
     */ 
    bool setForceParams(const std::vector<double>& force);


    /**
     * @brief Control solution
     */
    virtual void controllerRun(const AUVKineticSensor& sensor, const AUVControllerInput& input, AUVControllerOutput& output, const double dt) = 0;

    /**
     * @brief Serialize auv body parameters
     */
    virtual void serializeAUVBodyParams(std::stringstream& str);
    
    /**
     * @brief Serialize auv dynamic parameters
     */
    virtual void serializeAUVDynamicParams(std::stringstream& str);
    
    /**
     * @brief Serialize auv force parameters
     */
    virtual void serializeAUVForceParams(std::stringstream& str);
    
    /**
     * @brief Serialize auv control parameters
     */
    virtual void serializeAUVControlParams(std::stringstream& str);

protected:
    /**
     * @brief Initialize with default parameters
     */
    virtual void defaultInit();

    /**
     * @brief Filter function
     */
    int sign(double input){
        if(input > 0){
            return 1;
        }
        else if (input < 0){
            return -1;
        }
        else
        {
            return 0;
        }
    };
    
    double sat(double input, double thick){
        return fabs(input) >= thick ? sign(input) : (input / thick);
    };


protected:
    /**
     * @brief AUV depth surface status
     */ 
    struct AUVDepthSFStatus{
        double a_zw_, a_zq_, a_zs_, a_zb_, f_z_;
        double a_tw_, a_tq_, a_ts_, a_tb_, f_t_;
        double b_z_, b_zb_, b_zs_, b_t_, b_tb_, b_ts_;
        double g_z_, g_zb_, g_zs_, g_t_, g_tb_, g_ts_;
        double dot_z_, dot_theta_;
    
        void init(){
            a_zw_=0.0; a_zq_=0.0; a_zs_=0.0; a_zb_=0.0; f_z_=0.0;
            a_tw_=0.0; a_tq_=0.0; a_ts_=0.0; a_tb_=0.0; f_t_=0.0;
            b_z_=0.0; b_zb_=0.0; b_zs_=0.0; b_t_=0.0; b_tb_=0.0; b_ts_=0.0;
            g_z_=0.0; g_zb_=0.0; g_zs_=0.0; g_t_=0.0; g_tb_=0.0; g_ts_=0.0;
            dot_z_=0.0; dot_theta_=0.0;
        }
    }; // AUVDepthSFStatus
    
    /**
     * @brief AUV horizontal surface status
     */ 
    struct AUVHorizonSFStatus{
        double a_yv_, a_yr_, a_ydr_, f_y_;
        double a_pv_, a_pr_, a_pdr_, f_p_;
        double b_y_, b_ydr_, b_p_, b_pdr_;
        double g_y_, g_ydr_, g_p_, g_pdr_;
        double dot_y_, dot_psi_;
    
        void init(){
            a_yv_=0.0; a_yr_=0.0; a_ydr_=0.0; f_y_=0.0;
            a_pv_=0.0; a_pr_=0.0; a_pdr_=0.0; f_p_=0.0;
            b_y_=0.0; b_ydr_=0.0; b_p_=0.0; b_pdr_=0.0;
            dot_psi_=0.0;
        }
    }; // AUVHorizonSFStatus

    /**
     * @brief Slide model parameters
     */
    struct SlideModelParams{
        SMCtrlParams z_; // depth
        SMCtrlParams y_;
        SMCtrlParams theta_; // picth
        SMCtrlParams psi_; // yaw
    }; // SlideModelParams

protected:
    AUVBodyParams body_; 
    AUVDynamic dynamic_;
    ForceParams force_;
    CtrlParams ctrl_;
    
    AUVKinetic kinetic_;
    CtrlMissionParams mission_;
    SlideModelParams slide_model_;
    
    AUVDepthSFStatus depth_sf_;
    AUVHorizonSFStatus horizon_sf_;
    
    // Commands
    double deltab_, deltas_, deltar_;
    
    // Const value
    const unsigned int body_num_ = 13;
    const unsigned int dynamic_num_ = 28;
    const unsigned int ctrl_num_ = 13;
    const unsigned int force_num_ = 6;

}; // AUVBaseController
}; // ns

#endif

