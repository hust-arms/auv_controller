/*
 * Filename: Dynamic.h
 * Path: auv_controller
 * Created Date: Monday, September 7th 2020, 8:51:07 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2020 Your Company
 */
#ifndef DYNAMIC_H_
#define DYNAMIC_H_

namespace auv_controller{
    /**
     * @brief AUV dynamic parameters
     */
    struct AUVDynamic{
        double x_dotu_, y_dotv_, y_dotr_, z_dotw_, z_dotq_;
        double k_dotp_, m_dotw_, m_dotq_, n_dotv_, n_dotr_;
        double x_uu_, y_vv_, y_rr_, z_ww_, z_qq_;
        double k_pp_, m_ww_, m_qq_, n_vv_, n_rr_;
        double y_uv_, y_ur_, z_uw_, z_uq_;
        double m_uw_, m_uq_, n_uv_, n_ur_;

        /**
         * @brief AUV dynamic parameters initialization
         */
        /*
        AUVDynamic(double xdotu, double ydotv, double ydotr, double zdotw, double zdotq,
            double kdotq, double mdotw, double mdotq, double ndotv, double ndotr,
            double xuu, double yvv, double yrr, double zww, double zqq,
            double kpp, double mww, double mqq, double nvv, double nrr,
            double yuv, double yur, double zuw, double zuq,
            double muw, double muq, double nuv, double nur) : 
        x_dotu_(xdotu), y_dotv_(ydotv), y_dotr_(ydotr), z_dotw_(zdotw), z_dotq_(zdotq),
        k_dotq_(kdotq), m_dotw_(mdotw), m_dotq_(mdotq), n_dotv_(ndotv), n_dotr_(ndotr),
        x_uu_(xuu), y_vv_(yvv), y_rr_(yrr), z_ww_(zww), z_qq_(zqq),
        k_pp_(kpp), m_ww_(mww), m_qq_(mqq), n_vv_(nvv), n_rr_(nrr),
        y_uv_(yuv), y_ur_(yur), z_uw_(zuw), z_uq_(zuq),
        m_uw_(muw), m_uq_(muq), n_uv_(nuv), n_ur_(nur){};
        */

        /**
         * @brief AUV dynamic parameters setting
         */
        void setParameters(double xdotu, double ydotv, double ydotr, double zdotw, double zdotq,
            double kdotp, double mdotw, double mdotq, double ndotv, double ndotr,
            double xuu, double yvv, double yrr, double zww, double zqq,
            double kpp, double mww, double mqq, double nvv, double nrr,
            double yuv, double yur, double zuw, double zuq,
            double muw, double muq, double nuv, double nur)
        {
            x_dotu_ = xdotu; y_dotv_ = ydotv; y_dotr_ = ydotr; z_dotw_ = zdotw; z_dotq_ = zdotq;
            k_dotp_ = kdotp; m_dotw_ = mdotw; m_dotq_ = mdotq; n_dotv_ = ndotv; n_dotr_ = ndotr;
            x_uu_ = xuu; y_vv_ = yvv; y_rr_ = yrr; z_ww_ = zww; z_qq_ = zqq;
            k_pp_ = kpp; m_ww_ = mww; m_qq_ = mqq; n_vv_ = nvv; n_rr_ = nrr;
            y_uv_ = yuv; y_ur_ = yur; z_uw_ = zuw; z_uq_ = zuq;
            m_uw_ = muw; m_uq_ = muq; n_uv_ = nuv; n_ur_ = nur;
        }
    }; // AUVDynamic
}; // ns

#endif
