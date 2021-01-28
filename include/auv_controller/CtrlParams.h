/*
 * Filename: CtrlParams.h
 * Path: auv_controller
 * Created Date: Monday, September 7th 2020, 9:32:01 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2020 Your Company
 */
#ifndef CTRL_PARAMS_H_
#define CTRL_PARAMS_H_

#include <stdio.h>
#include "SMCtrlParams.h"

namespace auv_controller{
    /**
     * @brief Variable of control parameters
     */ 
    struct CtrlVar{
        double c_; 
        double k_;
        double alpha_;

        /**
         * @brief Set parameters
         */ 
        void setParameters(double c, double k, double alpha){
            c_ = c;
            k_ = k;
            alpha_ = alpha;
        }

        /**
         * @brief Print slide control parameters
         */
        void printCtrlParameters()const{
            printf("CtrlParams:{c%f k:%f alpha:%f}\n", c_, k_, alpha_);
        }

    }; // CtrlParams

    /**
     * @brief Control parameters for AUV
     */
    struct CtrlParams{
        CtrlVar z_;
        CtrlVar theta_;
        CtrlVar y_;
        CtrlVar psi_;
        double bondary_thick_;
    }; // CtrlParams

    /**
     * @brief Variable of control mission parameters
     */ 
    struct CtrlMissionVar{
        double ref_;
        double ref_dot_;
        double ref_dot2_;
        double pre_ref_;
        double pre_ref_dot_;

        /**
         * @brief Run and update values
         */
        void Update(){
            pre_ref_ = ref_;
            pre_ref_dot_ = ref_dot_;
        }

    }; // CtrlMissionVar

    /**
     * @brief Control mission parameters
     */
    struct CtrlMissionParams{
        CtrlMissionVar z_;
        CtrlMissionVar theta_;
        CtrlMissionVar y_;
        CtrlMissionVar psi_;
        double lateral_dist_;
    }; // CtrlMissionParams
}; // ns

#endif
