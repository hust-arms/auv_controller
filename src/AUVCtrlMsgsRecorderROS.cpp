/*                                                                                             
 * Filename: auv_controller/AUVCtrlMsgsRecorderROS.cpp
 * Path: auv_controller
 * Created Date: Thirsday, March 24th 2021, 10:58:05 am
 * Author: zhao wang
 * 
 * Copyright (c) 2021 hust-arms
 */

#include "auv_controller/AUVCtrlMsgsRecorderROS.h"
#include "auv_control_msgs/AUVOutlineStatus.h"

namespace auv_controller
{
/////////////////////////
AUVCtrlMsgsRecorderROS::AUVCtrlMsgsRecorderROS(std::string auv_name, bool with_ff, bool x_type, bool debug, int mission) : 
    auv_name_(auv_name), with_ff_(with_ff), x_type_(x_type), debug_(debug), mission_(mission)
{
    // initialize parameters
    x_ = 0.0; y_ = 0.0; z_ = 0.0;
    depth_ = 0.0;
    u_ = 0.0; v_ = 0.0; w_ = 0.0;
    roll_ = 0.0; pitch_ = 0.0; yaw_ = 0.0;
    roll_vel_ = 0.0; pitch_vel_ = 0.0; yaw_vel_ = 0.0;
    x_d_ = 0.0; y_d_ = 0.0; depth_d_ = 0.0; pitch_d_ = 0.0; yaw_d_ = 0.0; u_d_ = 0.0;
    fwd_l_fin_ = 0.0; fwd_r_fin_ = 0.0; back_l_fin_ = 0.0; back_r_fin_ = 0.0; vert_up_fin_ = 0.0; vert_lo_fin_ = 0.0;
    rpm_ = 0.0;
    latdist_dev_ = 0.0; depth_dev_ = 0.0; yaw_dev_ = 0.0; pitch_dev_ = 0.0;
    ol_x_ = 0.0; ol_y_ = 0.0; ol_z_ = 0.0;
    ol_roll_ = 0.0; ol_pitch_ = 0.0; ol_yaw_ = 0.0;
    ol_u_ = 0.0; ol_v_ = 0.0; ol_w_ = 0.0;
    ol_p_ = 0.0; ol_q_ = 0.0; ol_r_ = 0.0;
    ol_time_ = 0;

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Params configuration
    // private_nh.param("mission", mission_, 6);
    // private_nh.param("with_ff", with_ff_, false);
    // private_nh.param("x_type", x_type_, true);
    // private_nh.param("debug", debug_, true);
    // private_nh.param("auv_name", auv_name_, std::string("auv324"));
    private_nh.param("file_name", filename_, std::string("control_record"));
    private_nh.param("path", path_, std::string("../record/"));
    private_nh.param("frequency", record_freq_, 50);

    auv_odom_sub_ = nh.subscribe<nav_msgs::Odometry>(auv_name_ + "/pose_gt", 1, 
                        boost::bind(&AUVCtrlMsgsRecorderROS::auvOdometryCb, this, _1));
    imu_sub_ = nh.subscribe<sensor_msgs::Imu>(auv_name_ + "/imu", 1, 
                        boost::bind(&AUVCtrlMsgsRecorderROS::imuCb, this, _1));
    dvl_sub_ = nh.subscribe<uuv_sensor_ros_plugins_msgs::DVL>(auv_name_ + "/dvl", 1, 
                        boost::bind(&AUVCtrlMsgsRecorderROS::dvlCb, this, _1));
    rotor_speed_sub_ = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name_ + "/thruster/0/output", 1, 
                        boost::bind(&AUVCtrlMsgsRecorderROS::rotorSpeedCb, this, _1));

    outline_status_sub_ = nh.subscribe<auv_control_msgs::AUVOutlineStatus>(auv_name_ + "/outline_status", 1, 
                        boost::bind(&AUVCtrlMsgsRecorderROS::outlineStatusCb, this, _1));

    std::string fwd_l_fin_topic, fwd_r_fin_topic, back_l_fin_topic, back_r_fin_topic, 
        vert_up_fin_topic, vert_lo_fin_topic;

    if(with_ff_)
    {
        fwd_l_fin_topic = "/fins/1/output";
        fwd_r_fin_topic = "/fins/0/output";
        back_l_fin_topic = "/fins/2/output";
        back_r_fin_topic = "/fins/4/output";
        vert_up_fin_topic = "/fins/3/output";
        vert_lo_fin_topic = "/fins/5/output";
    }
    else
    {
        if(!x_type_)
        {
            fwd_l_fin_topic = "null_l";
            fwd_r_fin_topic = "null_r";
            back_l_fin_topic = "/fins/0/output";
            back_r_fin_topic = "/fins/2/output";
            vert_up_fin_topic = "/fins/1/output";
            vert_lo_fin_topic = "/fins/3/output";
        }
        else
        {
            fwd_l_fin_topic = "null_l";
            fwd_r_fin_topic = "null_r";
            back_l_fin_topic = "/fins/0/output"; // use back left represent upper port
            back_r_fin_topic = "/fins/3/output"; // use back right represent lower starboard
            vert_up_fin_topic = "/fins/1/output"; // use vertical upper represent upper starboard 
            vert_lo_fin_topic = "/fins/2/output"; // use vertical lower represent lower port
        }
    }
    
    fwd_l_fin_sub_ = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name_ + fwd_l_fin_topic, 1, 
                        boost::bind(&AUVCtrlMsgsRecorderROS::forwardLeftFinCb, this, _1));
    fwd_r_fin_sub_ = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name_ + fwd_r_fin_topic, 1, 
                        boost::bind(&AUVCtrlMsgsRecorderROS::forwardRightFinCb, this, _1));
    back_l_fin_sub_ = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name_ + back_l_fin_topic, 1, 
                        boost::bind(&AUVCtrlMsgsRecorderROS::backLeftFinCb, this, _1));
    back_r_fin_sub_ = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name_ + back_r_fin_topic, 1, 
                        boost::bind(&AUVCtrlMsgsRecorderROS::backRightFinCb, this, _1));
    vert_up_fin_sub_ = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name_ + vert_up_fin_topic, 1, 
                        boost::bind(&AUVCtrlMsgsRecorderROS::vertUpperFinCb, this, _1));
    vert_lo_fin_sub_ = nh.subscribe<uuv_gazebo_ros_plugins_msgs::FloatStamped>(auv_name_ + vert_lo_fin_topic, 1, 
                        boost::bind(&AUVCtrlMsgsRecorderROS::vertLowerFinCb, this, _1));

    auv_ctrl_info_sub_ = nh.subscribe<auv_control_msgs::AUVCtrlInfo>(auv_name_ + "/ctrl_info", 1,
                        boost::bind(&AUVCtrlMsgsRecorderROS::auvCtrlInfoCb, this, _1));
    auv_ctrl_dev_sub_ = nh.subscribe<auv_control_msgs::AUVCtrlDeviation>(auv_name_ + "/ctrl_deviation", 1,
                        boost::bind(&AUVCtrlMsgsRecorderROS::auvCtrlDeviationCb, this, _1));

    if(debug_)
    {
        boost::unique_lock<boost::recursive_mutex> lock(print_mutex_);
        printf("[AUVCtrlMsgsRecorderROS]: model info: with front fins: %d x_type: %d\n", with_ff_, x_type_);
        printf("[AUVCtrlMsgsRecorderROS]: mission: %d\n", mission_);
        printf("[AUVCtrlMsgsRecorderROS]: record path: %s\n", path_.c_str());
    }

    std::stringstream trans_ss;
    trans_ss << boost::gregorian::to_simple_string(boost::gregorian::day_clock::local_day());
    std::string time_stamp = trans_ss.str();
    // filename_ = filename_ + "_" + time_stamp + ".txt";
    filename_ = filename_ + "_" + time_stamp + ".csv";

    if(debug_)
    {
        boost::unique_lock<boost::recursive_mutex> lock(print_mutex_);
        printf("[AUVCtrlMsgsRecorderROS]: file name: %s\n", filename_.c_str());
    }

    ctrl_msgs_recorder_ = boost::make_shared<AUVCtrlMsgsRecorder>(filename_, path_);

    unsigned int model_type;

    if(with_ff_)
        model_type = 0;
    else if(x_type_)
        model_type = 2;
    else
        model_type = 1;
    
    if(!ctrl_msgs_recorder_->writeMsgsHeader(model_type, mission_))
    {
        boost::unique_lock<boost::recursive_mutex> lock(print_mutex_);
        printf("[AUVCtrlMsgsRecorderROS]: Error in message head writing");
    }
}

/////////////////////////
AUVCtrlMsgsRecorderROS::~AUVCtrlMsgsRecorderROS()
{
    if(record_th_ != nullptr)
    {
        record_th_->interrupt();
        record_th_->join();
        delete record_th_;
        record_th_ = nullptr;
    }
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::startRecord()
{
    // start record thread
    record_th_ = new boost::thread(boost::bind(&AUVCtrlMsgsRecorderROS::recordThread, this));
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::recordThread()
{ 
    ros::NodeHandle nh;
    time0_ = ros::Time::now();

    while(nh.ok())
    {
        double x, y, z; 
        double roll, pitch, yaw, roll_v, pitch_v, yaw_v;
        double u, v, w;
        double rpm; 
        double fwd_l_fin, fwd_r_fin, back_l_fin, back_r_fin, vert_up_fin, vert_lo_fin;
        double x_d, y_d, depth_d, pitch_d, yaw_d, u_d;
        double depth_dev, latdist_dev, yaw_dev, pitch_dev;

        getPosition(x, y, z);
        getPose(roll, pitch, yaw, roll_v, pitch_v, yaw_v);
        getLinVel(u, v, w);
        rpm = getRotorSpeed();
        fwd_l_fin = getFwdLeftFin();
        fwd_r_fin = getFwdRightFin();
        back_l_fin = getBackLeftFin();
        back_r_fin = getBackRightFin();
        vert_up_fin = getVertUpperFin();
        vert_lo_fin = getVertLowerFin();
        getCtrlInfo(x_d, y_d, depth_d, pitch_d, yaw_d, u_d);
        getCtrlDeviation(depth_dev, latdist_dev, yaw_dev, pitch_dev);

        std::vector<double> data_arr;
        double ptime = (ros::Time::now() - time0_).toSec();
        data_arr.push_back(ptime);
        data_arr.push_back(x); data_arr.push_back(y); data_arr.push_back(z);
        data_arr.push_back(-z); // depth
        data_arr.push_back(u); data_arr.push_back(v); data_arr.push_back(w);
        data_arr.push_back(roll); data_arr.push_back(pitch); data_arr.push_back(yaw);
        data_arr.push_back(roll_v); data_arr.push_back(pitch_v); data_arr.push_back(yaw_v);
        data_arr.push_back(rpm);
        if(with_ff_)
        {
            data_arr.push_back(fwd_l_fin); data_arr.push_back(fwd_r_fin); 
        }
        else
        {
            data_arr.push_back(0.0); data_arr.push_back(0.0); 
        }
        data_arr.push_back(back_l_fin); data_arr.push_back(back_r_fin); 
        data_arr.push_back(vert_up_fin); data_arr.push_back(vert_lo_fin); 

        if(ctrl_msgs_recorder_->isRecordOutlineParams())
        {
            // if(debug_)
            // {
            //     boost::unique_lock<boost::recursive_mutex> lock(print_mutex_);
            //     printf("[AUVCtrlMsgsRecorderROS]: record outline parameters\n");
            // }
            // 
            int ts;
            double ol_x, ol_y, ol_z, ol_roll, ol_pitch, ol_yaw;
            double ol_u, ol_v, ol_p, ol_q, ol_r;
            getOutlineStatus(ts, ol_x, ol_y, ol_z, ol_roll, ol_pitch, ol_yaw,
                             ol_u, ol_v, ol_p, ol_q, ol_r);
            // printf("<AUVCtrlMsgsRecorderROS>: getOutlineStatus: x:%f y:%f z:%f roll:%f pitch:%f yaw:%f\n", 
            //        ol_x, ol_y, ol_z, ol_roll, ol_pitch, ol_yaw);

            data_arr.push_back(ts);
            data_arr.push_back(ol_x); data_arr.push_back(ol_y); data_arr.push_back(ol_z);
            data_arr.push_back(ol_roll); data_arr.push_back(ol_pitch); data_arr.push_back(ol_yaw);
            data_arr.push_back(ol_u); data_arr.push_back(ol_v); 
            data_arr.push_back(ol_p); data_arr.push_back(ol_q); data_arr.push_back(ol_r);
        }
       else
        {
            data_arr.push_back(x_d); data_arr.push_back(y_d); data_arr.push_back(depth_d);
            data_arr.push_back(pitch_d); data_arr.push_back(yaw_d); data_arr.push_back(u_d);
            data_arr.push_back(depth_dev); data_arr.push_back(latdist_dev); 
            data_arr.push_back(yaw_dev); data_arr.push_back(pitch_dev); 
        }

        // if(debug_)
        // {
        //     boost::unique_lock<boost::recursive_mutex> lock(print_mutex_);
        //     std::cout << "[AUVCtrlMsgsRecorderROS]: ";
        //     for(int i = 0; data_arr.size(); ++i)
        //     {
        //         std::cout << data_arr[i] << " ";
        //     }
        //     std::cout << std::endl;
        // }


        if(!ctrl_msgs_recorder_->ctrlMsgsRecord(data_arr) && debug_)
        {
            boost::unique_lock<boost::recursive_mutex> lock(print_mutex_);
            printf("[AUVCtrlMsgsRecorderROS]: Error in control message record!\n");
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(1.0 / record_freq_ * 1000));
    }
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::auvOdometryCb(const nav_msgs::Odometry::ConstPtr& msg)
{
    boost::unique_lock<boost::recursive_mutex> lock(auv_odom_mutex_);
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    z_ = msg->pose.pose.position.z;
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::getPosition(double& x, double& y, double& z)
{
    boost::unique_lock<boost::recursive_mutex> lock(auv_odom_mutex_);
    x = x_; y = y_; z = z_;
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
    // PoseStamped::Quaternion to tf::Quaternion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat);
    
    // Quaternion to RPY
    boost::unique_lock<boost::recursive_mutex> lock(imu_mutex_);
    tf::Matrix3x3(quat).getRPY(roll_, pitch_, yaw_);
    
    roll_vel_ = msg->angular_velocity.x;
    pitch_vel_ =  msg->angular_velocity.y;
    yaw_vel_ = msg->angular_velocity.z;
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::getPose(double& roll, double& pitch, double& yaw, 
                                      double& roll_v, double& pitch_v, double& yaw_v)
{
    boost::unique_lock<boost::recursive_mutex> lock(imu_mutex_);
    roll = roll_; pitch = pitch_; yaw = yaw_;
    roll_v = roll_vel_; pitch_v = pitch_vel_; yaw_v = yaw_vel_;
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::dvlCb(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr& msg)
{
    boost::unique_lock<boost::recursive_mutex> lock(dvl_mutex_);
    u_ = msg->velocity.x;
    v_ = msg->velocity.y;
    w_ = msg->velocity.z;
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::getLinVel(double& x_v, double& y_v, double& z_v)
{
    boost::unique_lock<boost::recursive_mutex> lock(dvl_mutex_);
    x_v = u_; y_v = v_; z_v = w_;
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::rotorSpeedCb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    boost::unique_lock<boost::recursive_mutex> lock(rotor_speed_mutex_);
    rpm_ = msg->data;
}

/////////////////////////
double AUVCtrlMsgsRecorderROS::getRotorSpeed()
{
    boost::unique_lock<boost::recursive_mutex> lock(rotor_speed_mutex_);
    return rpm_;
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::forwardLeftFinCb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    boost::unique_lock<boost::recursive_mutex> lock(fwd_l_fin_mutex_);
    fwd_l_fin_ = msg->data;
}

/////////////////////////
double AUVCtrlMsgsRecorderROS::getFwdLeftFin()
{
    boost::unique_lock<boost::recursive_mutex> lock(fwd_l_fin_mutex_);
    return fwd_l_fin_;
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::forwardRightFinCb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    boost::unique_lock<boost::recursive_mutex> lock(fwd_r_fin_mutex_);
    fwd_r_fin_ = msg->data;
}

/////////////////////////
double AUVCtrlMsgsRecorderROS::getFwdRightFin()
{
    boost::unique_lock<boost::recursive_mutex> lock(fwd_r_fin_mutex_);
    return fwd_r_fin_;
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::backLeftFinCb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    boost::unique_lock<boost::recursive_mutex> lock(back_l_fin_mutex_);
    back_l_fin_ = msg->data;
}

/////////////////////////
double AUVCtrlMsgsRecorderROS::getBackLeftFin()
{
    boost::unique_lock<boost::recursive_mutex> lock(back_l_fin_mutex_);
    return back_l_fin_;
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::backRightFinCb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    boost::unique_lock<boost::recursive_mutex> lock(back_r_fin_mutex_);
    back_r_fin_ = msg->data;
}

/////////////////////////
double AUVCtrlMsgsRecorderROS::getBackRightFin()
{
    boost::unique_lock<boost::recursive_mutex> lock(back_r_fin_mutex_);
    return back_r_fin_;
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::vertUpperFinCb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    boost::unique_lock<boost::recursive_mutex> lock(vert_up_fin_mutex_);
    vert_up_fin_ = msg->data;
}

/////////////////////////
double AUVCtrlMsgsRecorderROS::getVertUpperFin()
{
    boost::unique_lock<boost::recursive_mutex> lock(vert_up_fin_mutex_);
    return vert_up_fin_;
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::vertLowerFinCb(const uuv_gazebo_ros_plugins_msgs::FloatStamped::ConstPtr& msg)
{
    boost::unique_lock<boost::recursive_mutex> lock(vert_lo_fin_mutex_);
    vert_lo_fin_ = msg->data;
}

/////////////////////////
double AUVCtrlMsgsRecorderROS::getVertLowerFin()
{
    boost::unique_lock<boost::recursive_mutex> lock(vert_lo_fin_mutex_);
    return vert_lo_fin_;
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::auvCtrlInfoCb(const auv_control_msgs::AUVCtrlInfo::ConstPtr& msg)
{
    boost::unique_lock<boost::recursive_mutex> lock(auv_ctrl_info_mutex_);
    x_d_ = msg->desiredx;
    y_d_ = msg->desiredy;
    depth_d_ = msg->desireddepth;
    pitch_d_ = msg->desiredpitch;
    yaw_d_ = msg->desiredyaw;
    u_d_ = msg->desiredu;
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::getCtrlInfo(double& x_d, double& y_d, double& depth_d,
                 double& pitch_d, double& yaw_d, double& u_d)
{
    boost::unique_lock<boost::recursive_mutex> lock(auv_ctrl_info_mutex_);
    x_d = x_d_; y_d = y_d_; depth_d = depth_d_;
    pitch_d = pitch_d_; yaw_d = yaw_d_; u_d = u_d_;
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::auvCtrlDeviationCb(const auv_control_msgs::AUVCtrlDeviation::ConstPtr& msg)
{
    boost::unique_lock<boost::recursive_mutex> lock(auv_ctrl_dev_mutex_);
    depth_dev_ = msg->depthdev;
    latdist_dev_ = msg->latdistdev;
    yaw_dev_ = msg->yawdev;
    pitch_dev_ = msg->pitchdev;
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::getCtrlDeviation(double& depthdev, double& latdistdev, double& yawdev, double& pitchdev)
{
    boost::unique_lock<boost::recursive_mutex> lock(auv_ctrl_dev_mutex_);
    depthdev = depth_dev_; 
    latdistdev = latdist_dev_;
    yawdev = yaw_dev_;
    pitchdev = pitch_dev_;
}

/////////////////////////
void AUVCtrlMsgsRecorderROS::outlineStatusCb(const auv_control_msgs::AUVOutlineStatus::ConstPtr& msg)
{
    boost::unique_lock<boost::recursive_mutex> lock(outline_status_mutex_);
    ol_x_ = msg->x; ol_y_ = msg->y; ol_z_ = msg->z;
    ol_roll_ = msg->roll; ol_pitch_ = msg->pitch; ol_yaw_ = msg->yaw;
    ol_u_ = msg->u; ol_v_ = msg->v; 
    ol_p_ = msg->p; ol_q_ = msg->q; ol_r_ = msg->r;
    ol_time_ = msg->ts;
    // printf("<AUVCtrlMsgsRecorderROS>: outlineStatusCb: x:%f y:%f z:%f roll:%f pitch:%f yaw:%f\n", 
    //        ol_x_, ol_y_, ol_z_, ol_roll_, ol_pitch_, ol_yaw_);
}

}; // ns


