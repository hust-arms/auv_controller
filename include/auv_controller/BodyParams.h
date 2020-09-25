/*
 * Filename: BodyParams.h
 * Path: auv_controller
 * Created Date: Monday, September 7th 2020, 9:08:36 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2020 Your Company
 */
#ifndef BODY_PARAMS_H_
#define BODY_PARAMS_H_

namespace auv_controller{
    /**
     * @brief Kinetic parameters of AUV
     */ 
    struct AUVKinetic{
        double x_, y_, z_, phi_, theta_, psi_; // position | pose
        double u_, v_, w_, p_, q_, r_; // linear velocity | angular velocity

        /**
         * @brief Kinetic parameters initialization
         */ 
        /*
        AUVKinetic(double x, double y, double z, double phi, double theta, double psi,
            double u, double v, double w, double p, double q, double r) : 
            x_(x), y_(y), z_(z), phi_(phi), theta_(theta), psi_(psi), 
            u_(u), v_(v), w_(w), p_(p), q_(q), r_(r){};
        */

        /**
         * @brief Set position
         */ 
        void setPosition(double x, double y, double z, double phi, double theta, double psi){
            x_ = x; y_ = y; z_ = z;
            phi_ = phi; theta_ = theta; psi_ = psi;
        }

        /**
         * @brief Set velocity
         */
        void setVelocity(double u, double v, double w, double p, double q, double r){
            u_ = u; v_ = v; w_ = w; p_ = p; q_ = q; r_ = r;
        }
    }; // AUVKinetic

    /**
     * @brief Body parameters of AUV
     */
    struct AUVBodyParams{
        double m_, l_, w_, b_; // mass | length | gravity | buoyancy
        double x_b_, y_b_, z_b_, x_g_, y_g_, z_g_; // buoyancy and gravity center coordinates
        double i_xx_, i_yy_, i_zz_; // inertial comment

        /**
         * @brief Body parameters initialization
         */ 
        /*
        AUVBodyParams(double m, double l, double w, double b,
            double xb, double yb, double zb, double xg, double yg, double zg,
            double ixx, double iyy, double izz) : 
                m_(m), l_(l), w_(w), b_(b),
                x_b_(xb), y_b_(yb), z_b_(zb), x_g_(xg), y_g_(yg), z_g_(zg),
                i_xx_(ixx), i_yy_(iyy), i_zz_(izz){};
        */

        /**
         * @brief Body parameters setting
         */ 
        void setParameters(double m, double l, double w, double b,
            double xb, double yb, double zb, double xg, double yg, double zg,
            double ixx, double iyy, double izz)
        {
            m_ = m; l_ = l; w_ = w, b_ = b;
            x_b_ = xb; y_b_ = yb; z_b_ = zb; x_g_ = xg; y_g_ = yg; z_g_ = zg;
            i_xx_ = ixx; i_yy_ = iyy; i_zz_ = izz;
        }
    }; // AUVBodyParams

}; // ns

#endif
