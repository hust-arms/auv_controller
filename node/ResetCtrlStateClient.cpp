#include "cstdlib"
#include "ros/ros.h"
#include "auv_controller/ResetCtrlState.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "reset_ctrl_state");
    if(argc != 2)
    {
        ROS_WARN("usage: reset_ctrl_state type");
        return 1;
    }

    ros::NodeHandle nh;
    ros::ServiceClient ctrl_state_reset_cl = nh.serviceClient<auv_controller::ResetCtrlState>("reset_ctrl_state");

    auv_controller::ResetCtrlState srv;
    srv.request.IsReset = atoll(argv[1]);

    if(ctrl_state_reset_cl.call(srv))
    {
        ROS_INFO("Server feedback: %s", srv.response.FeedbackMsg);
    }
    else
    {
        ROS_ERROR("Failed to call service reset_ctrl_state!");
        return 1;
    }

    return 0;
}

