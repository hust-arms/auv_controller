/*                                                                            
 * Filename: AUVControllerNoFF.h
 * Path: auv_controller
 * Created Date: Thursday, Janurary 28th 2021, 23:46:39 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2021 Your Company
 */

#ifndef AUV_CONTROLLER_NO_FF_H_
#define AUV_CONTROLLER_NO_FF_H_

#include "AUVBaseController.h"

namespace auv_controller{

/**
 * @brief AUV slide model controller for model with only back fins
 */ 
class AUVControllerNoFF : public AUVBaseController
{
public:
    /**
     * @brief Default constructor
     */ 
    AUVControllerNoFF() : AUVBaseController() {}

    /**
     * @brief Deconstructor
     */
    ~AUVControllerNoFF() {}

    /**
     * @brief Implementation of controller
     */ 
    virtual void controllerRun(const AUVKineticSensor& sensor, const AUVControllerInput& input, AUVControllerOutput& output, const double dt, bool vel_ctrl) override;

}; // AUVControllerNoFF
}; // ns

#endif
