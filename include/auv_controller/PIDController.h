#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

/// \brief standard lib components
#include <string>
#include <map>
#include <memory>

/// \brief boost components
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

/**
 * @brief PID controlle class with implementation of singleton
 */ 
class PIDController
{
public:
    /**
     * @brief Constructor
     * @param kp Proportional item
     * @param ki Inertial item
     * @param kd Differential item
     */
    PIDController(double kp, double ki, double kd, bool is_angle) : 
        kp_(kp), ki_(ki), kd_(kd), is_angle_(is_angle){}
    
    /**
     * @brief Constructor
     * @param kp Proportional item
     * @param ki Inertial item
     * @param kd Differential item
     */
    PIDController(double kp, double ki, double kd, double tar, bool is_angle) : 
        kp_(kp), ki_(ki), kd_(kd), params_tar_(tar), is_angle_(is_angle) {}
   
    /**
     * @brief Deconstructor
     */
    ~PIDController() {}

    /**
     * @brief Set target parameters
     */
    void setTargetParams(double tar);
    
    /**
     * @brief Set PID parameters 
     */
    void setPIDParams(double kp, double ki, double kd);

    /**
     * @brief Reset deviation of target parameter
     */
    void resetDeviation();

    /**
     * @brief Get control variable
     */
    double positionalPID(double params_cur, double dt);

    /**
     * @brief Get control variable
     */
    double incrementalPID(double parms_cur, double dt);

protected:
    double params_tar_;

    double kp_, ki_, kd_; // PID params
    double dev_, dev_last_, dev_last_bef_;
    double dev_integral_;

    bool is_angle_;

    const double pi_ = 3.141592654;

    boost::recursive_mutex params_mutex_;

}; // PIDController 

typedef boost::shared_ptr<PIDController> PIDControllerPtr;

/**
 * @brief Factory class of PIDController
 */
class PIDControllerFactory{
public:
    /**
     * @brief Return instance of pid controller factory
     */
    static PIDControllerFactory& getInstance();

    /**
     * @brief Return pid controller ptr
     */
    PIDControllerPtr create(double kp, double ki, double kd, double params_tar);

private:
    /**
     * @brief Singleton constructor
     */
    PIDControllerFactory(){}

}; // PIDControllerFactory

#endif
