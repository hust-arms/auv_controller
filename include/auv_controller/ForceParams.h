/*
 * Filename: CtrlForceParams.h
 * Path: auv_controller
 * Created Date: Monday, September 7th 2020, 9:11:29 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2020 Your Company
 */
#ifndef FORCE_PARAMS_H_
#define FORCE_PARAMS_H_

namespace auv_controller{
    struct ForceParams{
        double y_uudr_, z_uuds_, z_uudb_;
        double m_uuds_, m_uudb_, n_uudr_;

        /**
         * @brief Set parameters
         */
        void setParameters(double yuudr, double zuuds, double zuudb, double muuds, double muudb, double nuudr){
            y_uudr_ = yuudr; z_uuds_ = zuuds; z_uudb_ = zuudb;
            m_uuds_ = muuds; m_uudb_ = muudb; n_uudr_ = nuudr;
        }
    }; // CtrlForceParams
}; // ns

#endif
