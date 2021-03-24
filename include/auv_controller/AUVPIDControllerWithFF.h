/*                                                                            
 * Filename: AUVPIDControllerWithFF.h
 * Path: auv_controller
 * Created Date: Thirsday, March 24th 2021, 18:00:00 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#ifndef AUV_PID_CONTROLLER_WITH_FF_H_
#define AUV_PID_CONTROLLER_WITH_FF_H_

#include "AUVPIDController.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace auv_controller{

/**
 * @brief AUV slide model controller for model with X type back fins
 */ 
class AUVPIDControllerWithFF : public AUVPIDController
{
public:
    /**
     * @brief Default constructor
     */ 
    AUVPIDControllerWithFF() : AUVPIDController() {}

    /**
     * @brief Deconstructor
     */
    ~AUVPIDControllerWithFF() {}

    /**
     * @brief Implementation of controller
     */ 
    virtual void controllerRun(const AUVKineticSensor& sensor, const AUVControllerInput& input, AUVControllerOutput& output, const double dt, bool vel_ctrl) override;

}; // AUVPIDControllerWithFF
}; // ns

#endif
