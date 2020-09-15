/*
 * Filename: Common.h
 * Path: auv_controller
 * Created Date: Tuesday, September 8th 2020, 3:09:58 pm
 * Author: zhao wang
 * 
 * Copyright (c) 2020 Your Company
 */

namespace auv_controller{
    const u_int32_t ms = 1000;
    const double pi = 3.1415926535;
    const double rad2degree = 180 / pi;
    const double degree2rad = 1 / rad2degree;

    /**
     * @brief Data of sensors
     */ 
    struct AUVKineticSensor{
        double x_, y_, z_;
        double roll_, pitch_, yaw_;
        double x_dot_, y_dot_, z_dot_;
        double roll_dot_, pitch_dot_, yaw_dot_;
    }; // SensorData
}; // ns
