/*
 * Filename: AUVPIDController.h
 * Path: auv_controller
 * Created Date: Saturday, Faburary 27th 2021, 15:14:39         
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#ifndef AUV_PID_CONTROLLER_H_
#define AUV_PID_CONTROLLER_H_

#include <vector>
#include <string>
#include <sstream>
#include "BodyParams.h"
#include "PIDCtrlParams.h"
#include "AUVCommon.h"

#include "PIDController.h"

#include <boost/shared_ptr.hpp>

namespace auv_controller{

typedef boost::shared_ptr<PIDController> PIDControllerPtr;

/**
 * @brief AUV controller input parameters
 */
struct AUVControllerInput{
    /**
     * @brief Constructor
     */
    AUVControllerInput(double depthd, double pitchd, double yawd, double xd, double yd, double ud) :
        depth_d_(depthd), pitch_d_(pitchd), yaw_d_(yawd), x_d_(xd), y_d_(yd), u_d_(ud){};

    double depth_d_; // desired depth
    double pitch_d_; // desired pitch
    double yaw_d_; // desired yaw
    double x_d_; // waypoint on the desired line
    double y_d_; // waypoint on the desired line
    double u_d_; // desired linear x velocity
}; //  AUVControllerInput

/**
  * @brief AUV controller output parameters
  */
struct AUVControllerOutput{
    double rudder_;
    double fwd_fin_;
    double aft_fin_;
    double rpm_;

    double upper_p_; // for X type rudder
    double upper_s_;
    double lower_p_;
    double lower_s_;
}; // AUVControllerOutput


/**
 * @brief Base auv slide model controller 
 */ 
class AUVPIDController
{
public:
    /**
     * @brief Default class initialization
     */
    AUVPIDController();

    /**
     * @brief Deconstructor
     */
    virtual ~AUVPIDController() {} 

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
     * @brief Set control parameters
     */
    void setCtrlParams(double kpz, double kiz, double kdz,
        double kptheta, double kitheta, double kdtheta,
        double kpy, double kiy, double kdy,
        double kppsi, double kipsi, double kdpsi);

    /**
     * @brief Set depth control variable
     */
    void setDepthCtrlVar(const double& ang_limit, const unsigned int& k){
        pitch_limit_ = ang_limit;
        pitch_k_ = k;
    }

    /**
     * @brief Set lateral control variable
     */
    void setLateralCtrlVar(const unsigned int& k)
    {
        yaw_k_ = k;
    }

    /**
     * @brief Set control parameters
     */ 
    bool setCtrlParams(const std::vector<double>& ctrl);
    
    /**
     * @brief Set control parameters
     */
    void setCtrlParams(double kp, double ki, double kd, unsigned int flag);

    /**
     * @brief Control solution
     */
    virtual void controllerRun(const AUVKineticSensor& sensor, const AUVControllerInput& input, AUVControllerOutput& output, const double dt, bool vel_ctrl) = 0;

    /**
     * @brief Serialize auv body parameters
     */
    virtual void serializeAUVBodyParams(std::stringstream& str);
    
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
    PIDControllerPtr depth_controller_;
    PIDControllerPtr latdev_controller_;
    PIDControllerPtr vel_controller_;

protected:
    AUVBodyParams body_; 
    AUVKinetic kinetic_;
    PIDCtrlParams pid_ctrl_;
    
    // Commands
    double deltab_, deltas_, deltar_;
    double deltaup_, deltaus_, deltalp_, deltals_; // for X type rudder

    // control var
    double pitch_limit_;
    unsigned int pitch_k_;
    unsigned int yaw_k_;
    
    // Const value
    const unsigned int body_num_ = 13;
    const unsigned int ctrl_num_ = 12;
}; // AUVPIDController
}; // ns

#endif

