/*                                                                            
 * Filename: AUVPIDControllerXF.h
 * Path: auv_controller
 * Created Date: Saturday, Faburary 27th 2021, 18:00:00 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#ifndef AUV_PID_CONTROLLER_XF_H_
#define AUV_PID_CONTROLLER_XF_H_

#include "AUVPIDController.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace auv_controller{

typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DynamicMatrix;


/**
 * @brief AUV slide model controller for model with X type back fins
 */ 
class AUVPIDControllerXF : public AUVPIDController
{
public:
    /**
     * @brief Default constructor
     */ 
    AUVPIDControllerXF() : AUVPIDController() {}

    /**
     * @brief Deconstructor
     */
    ~AUVPIDControllerXF() {}

    /**
     * @brief Implementation of controller
     */ 
    virtual void controllerRun(const AUVKineticSensor& sensor, const AUVControllerInput& input, AUVControllerOutput& output, const double dt, bool vel_ctrl) override;

}; // AUVPIDControllerXF
}; // ns

#endif
