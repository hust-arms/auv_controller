/*
 * Filename: SMCtrlParams.h
 * Path: auv_controller
 * Created Date: Tuesday, September 8th 2020, 10:32:15 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2020 Your Company
 */

namespace auv_controller{
    /**
     * @brief Slide control parameters
     */
    struct SMCtrlParams{
        double e_;
        double dot_e_;
        double s_;
        double l_;
    }; // SMCtrlParams
}; // ns
