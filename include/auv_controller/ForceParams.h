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
    }; // CtrlForceParams for lateral rouder

    struct XForceParams{
        double y_dup_, y_dus_, y_dlp_, y_dls_;
        double z_dup_, z_dus_, z_dlp_, z_dls_;
        double m_dup_, m_dus_, m_dlp_, m_dls_;
        double n_dup_, n_dus_, n_dlp_, n_dls_;

        void setParameters(double ydup, double ydus, double ydlp, double ydls, double zdup, double zdus, double zdlp, double zdls,
                           double mdup, double mdus, double mdlp, double mdls, double ndup, double ndus, double ndlp, double ndls){
            y_dup_ = ydup; y_dus_ = ydus; y_dlp_ = ydlp; y_dls_ = ydls;
            z_dup_ = zdup; z_dus_ = zdus; z_dlp_ = zdlp; z_dls_ = zdls;
            m_dup_ = mdup; m_dus_ = mdus; m_dlp_ = mdlp; m_dls_ = mdls;
            n_dup_ = ndup; n_dus_ = ndus; n_dlp_ = ndlp; n_dls_ = ndls;
        }
    };

}; // ns

#endif
