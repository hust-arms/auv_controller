/*                                                                                                         
 * Filename: AUVControllerWithFF.h
 * Path: auv_controller
 * Created Date: Thursday, Janurary 28th 2021, 16:12:39 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2021 Your Company
 */

#ifndef AUV_CONTROLLER_WITH_FF_H_
#define AUV_CONTROLLER_WITH_FF_H_

#include "AUVBaseController.h"

namespace auv_controller{


/**
 * @brief AUV slide model controller for model with both of front and back fins
 */
class AUVControllerWithFF : public AUVBaseController
{
public:
    /**
     * @brief Defualt constructor
     */
    AUVControllerWithFF() : AUVBaseController(){}

    /**
     * @brief Deconstructor
     */ 
    ~AUVControllerWithFF() {}

    /**
     * @brief Implementation of controller
     */ 
    virtual void controllerRun(const AUVKineticSensor& sensor, const AUVControllerInput& input, AUVControllerOutput& output, const double dt, bool ctrl_vel) override;

}; // AUVControllerWithFF
};  // ns

#endif

