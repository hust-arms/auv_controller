/*
 * Filename: PIDCtrlParams.h
 * Path: auv_controller
 * Created Date: Saturday, Faburary 27th 2021, 9:32:01 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */
#ifndef PID_CTRL_PARAMS_H_
#define PID_CTRL_PARAMS_H_

#include <stdio.h>

namespace auv_controller{
    /**
     * @brief Variable of control parameters
     */ 
    struct PIDCtrlVar{
        double kp_; 
        double ki_;
        double kd_;

        /**
         * @brief Set parameters
         */ 
        void setParameters(double kp, double ki, double kd){
            kp_ = kp;
            ki_ = ki;
            kd_ = kd;
        }

        /**
         * @brief Print slide control parameters
         */
        void printPIDCtrlParameters()const{
            printf("PIDCtrlParams:{kp%f ki:%f kd:%f}\n", kp_, ki_, kd_);
        }

    }; // PIDCtrlParams

    /**
     * @brief Control parameters for AUV
     */
    struct PIDCtrlParams{
        PIDCtrlVar z_;
        PIDCtrlVar theta_;
        PIDCtrlVar y_;
        PIDCtrlVar psi_;
    }; // PIDCtrlParams

}; // ns

#endif
