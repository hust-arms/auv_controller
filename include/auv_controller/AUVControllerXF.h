/*                                                                            
 * Filename: AUVControllerXF.h
 * Path: auv_controller
 * Created Date: Thursday, Faburary 4th 2021, 18:00:00 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#ifndef AUV_CONTROLLER_XF_H_
#define AUV_CONTROLLER_XF_H_

#include "AUVBaseController.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace auv_controller{

typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;

/**
 * @brief AUV slide model controller for model with X type back fins
 */ 
class AUVControllerXF : public AUVBaseController
{
public:
    /**
     * @brief Default constructor
     */ 
    AUVControllerXF() : AUVBaseController() {}

    /**
     * @brief Deconstructor
     */
    ~AUVControllerXF() {}

    /**
     * @brief Implementation of controller
     */ 
    virtual void controllerRun(const AUVKineticSensor& sensor, const AUVControllerInput& input, AUVControllerOutput& output, const double dt, bool vel_ctrl) override;

}; // AUVControllerXF
}; // ns

#endif
