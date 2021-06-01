#!/usr/bin/env python
# coding=utf-8

import sqlite3
import time, datetime
import rospy

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

class auv324Param():
    def __init__(self):
        self.time_ = 0.0
        self.ctrl_mode_ = 0
        self.mission_ = 0
        self.is_mission_on_ = 0
        self.lng_ori_ = 0.0
        self.lat_ori_ = 0.0
        # Status
        self.lng_ = 0.0
        self.lat_ = 0.0
        self.x_ = 0.0
        self.y_ = 0.0
        self.hei_ = 0.0
        self.ori_hei_ = 0.0
        self.depth_ = 0.0
        self.roll_ = 0.0
        self.pitch_ = 0.0
        self.yaw_ = 0.0
        self.u_ = 0.0
        self.v_ = 0.0
        self.w_ = 0.0
        self.p_ = 0.0
        self.q_ = 0.0
        self.r_ = 0.0
        self.vel_n_ = 0.0
        self.vel_e_ = 0.0
        self.vel_d_ = 0.0
        self.path_course_ = 0.0
        self.ye_ = 0.0
        # Control reference
        self.desired_course_ = 0.0
        self.desired_depth_ = 0.0
        self.desired_pitch_ = 0.0
        self.desired_rudder_ = 0.0
        self.desired_stern_ = 0.0
        # Rudder&Stern
        self.desired_fin0_ = 0.0
        self.desired_fin1_ = 0.0
        self.desired_fin2_ = 0.0
        self.desired_fin3_ = 0.0
        # Thruster
        self.pwm_ = 0
        self.rpm_ = 0.0
        # Binocular
        self.bin_x_ = 0.0
        self.bin_y_ = 0.0
        self.bin_z_ = 0.0
        # USBL
        self.usbl_x_ = 0.0
        self.usbl_y_ = 0.0
        self.usbl_z_ = 0.0
        self.usbl_roll_ = 0.0
        self.usbl_pitch_ = 0.0
        self.usbl_yaw_ = 0.0
        self.usbl_lng_ = 0.0
        self.usbl_lat_ = 0.0
        # Log
        self.rise_res_ = 0
        self.emg_level_ = 0
        self.emg_state_ = 0

class auv324DBParser():
    def __init__(self, table):
        self.param_list_ = []
        self.table_ = table

        self.time_ind_ = 0
        self.ctrl_mode_ind_ = 1
        self.mission_ind_ = 2
        self.is_mission_on_ind_ = 3
        self.lng_ori_ind_ = 4
        self.lat_ori_ind_ = 5
        # Status
        self.lng_ind_ = 6
        self.lat_ind_ = 7
        self.x_ind_ = 8
        self.y_ind_ = 9
        self.hei_ind_ = 10
        self.ori_hei_ind_ = 11
        self.depth_ind_ = 12
        self.roll_ind_ = 13
        self.pitch_ind_ = 14
        self.yaw_ind_ = 15
        self.u_ind_ = 16
        self.v_ind_ = 17
        self.w_ind_ = 18
        self.p_ind_ = 19
        self.q_ind_ = 20
        self.r_ind_ = 21
        self.vel_n_ind_ = 22
        self.vel_e_ind_ = 23
        self.vel_d_ind_ = 24
        self.path_course_ind_ = 25
        self.ye_ind_ = 26
        # Control reference
        self.desired_course_ind_ = 27
        self.desired_depth_ind_ = 28
        self.desired_pitch_ind_ = 29
        self.desired_rudder_ind_ = 30
        self.desired_stern_ind_ = 31
        # Rudder&Stern
        self.desired_fin0_ind_ = 32
        self.desired_fin1_ind_ = 33
        self.desired_fin2_ind_ = 34
        self.desired_fin3_ind_ = 35
        # Thruster
        self.pwm_ind_ = 36
        self.rpm_ind_ = 37
        # Binocular
        self.bin_x_ind_ = 38
        self.bin_y_ind_ = 39
        self.bin_z_ind_ = 40
        # USBL
        self.usbl_x_ind_ = 41 
        self.usbl_y_ind_ = 42
        self.usbl_z_ind_ = 43
        self.usbl_roll_ind_ = 44 
        self.usbl_pitch_ind_ = 45
        self.usbl_yaw_ind_ = 46
        self.usbl_lng_ind_ = 47
        self.usbl_lat_ind_ = 48
        # Log
        self.rise_res_ind_ = 49
        self.emg_level_ind_ = 50
        self.emg_state_ind_ = 51

    def get_params(self):
        return self.param_list_

    def parse_db(self, db_fn):
        print('<auvParser>: start parse')

        # open database
        conn = sqlite3.connect(db_fn)
        
        # parse data
        c = conn.cursor()
        
        # create auv param object
        cursor = c.execute('SELECT * from {}'.format(self.table_))

        print('<auvParser>: parsing')
        try:
            for row in cursor:
                param = auv324Param()

                param.time_ = row[self.time_ind_]
                param.ctrl_mode_ = row[self.ctrl_mode_ind_]
                param.mission_ = row[self.mission_ind_]
                param.is_mission_on_ = row[self.is_mission_on_ind_]
                param.lng_ori_ = row[self.lng_ori_ind_]
                param.lat_ori_ = row[self.lat_ori_ind_]
                # Status
                param.lng_ = row[self.lng_ind_]
                param.lat_ = row[self.lat_ind_]
                param.x_ = row[self.x_ind_]
                param.y_ = row[self.y_ind_]
                param.hei_ = row[self.hei_ind_]
                param.ori_hei_ = row[self.ori_hei_ind_]
                param.depth_ = row[self.depth_ind_]
                param.roll_ = row[self.roll_ind_]
                param.pitch_ = row[self.pitch_ind_]
                param.yaw_ = row[self.yaw_ind_]
                param.u_ = row[self.u_ind_]
                param.v_ = row[self.v_ind_]
                param.w_ = row[self.w_ind_]
                param.p_ = row[self.p_ind_]
                param.q_ = row[self.q_ind_]
                param.r_ = row[self.r_ind_]
                param.vel_n_ = row[self.vel_n_ind_]
                param.vel_e_ = row[self.vel_e_ind_]
                param.vel_d_ = row[self.vel_d_ind_]
                param.path_course_ = row[self.path_course_ind_]
                param.ye_ = row[self.ye_ind_]
                # Control reference
                param.desired_course_ = row[self.desired_course_ind_]
                param.desired_depth_ = row[self.desired_depth_ind_]
                param.desired_pitch_ = row[self.desired_pitch_ind_]
                param.desired_rudder_ = row[self.desired_rudder_ind_]
                param.desired_stern_ = row[self.desired_stern_ind_]
                # Rudder&Stern
                param.desired_fin0_ = row[self.desired_fin0_ind_]
                param.desired_fin1_ = row[self.desired_fin1_ind_]
                param.desired_fin2_ = row[self.desired_fin2_ind_]
                param.desired_fin3_ = row[self.desired_fin3_ind_]
                # Thruster
                param.pwm_ = row[self.pwm_ind_]
                param.rpm_ = row[self.rpm_ind_]
                # Binocular
                param.bin_x_ = row[self.bin_x_ind_]
                param.bin_y_ = row[self.bin_y_ind_]
                param.bin_z_ = row[self.bin_z_ind_]
                # USBL
                param.usbl_x_ = row[self.usbl_x_ind_]
                param.usbl_y_ = row[self.usbl_y_ind_]
                param.usbl_z_ = row[self.usbl_z_ind_]
                param.usbl_roll_ = row[self.usbl_roll_ind_]
                param.usbl_pitch_ = row[self.usbl_pitch_ind_]
                param.usbl_yaw_ = row[self.usbl_yaw_ind_]
                param.usbl_lng_ = row[self.usbl_lng_ind_]
                param.usbl_lat_ = row[self.usbl_lat_ind_]
                # Log
                param.rise_res_ = row[self.rise_res_ind_]
                param.emg_level_ = row[self.emg_level_ind_]
                param.emg_state_ = row[self.emg_state_ind_]

                self.param_list_.append(param)
        except sqlite3.DatabaseError:
            pass

        cursor.close()
        conn.close()
        print('<auvParser>: parse finish')

def plot2D(x, y, xlabel, ylabel, path, color):
    plt.plot(x, y, color)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.savefig(path)
    plt.close()

def plotMultiCurve(x, y_l, xlabel, ylabel, ylabel_l, path, color_l, size):
    fig, ax = plt.subplots()
    for i in range(len(size)):
        ax.plot(x, y_l[i], color_l[i], label=ylabel_l[i])
        
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.legend()
    fig.savefig(path)
    plt.close()

def plotAUV324Param(param_list, path):
    prefix = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

    param_len = len(param_list)

    time_list = []
    ctrl_mode_list = []
    mission_list = []
    is_mission_on_list = []
    lng_ori_list = []
    lat_ori_list = []
    # Status
    lng_list = []
    lat_list = []
    x_list = []
    y_list = []
    hei_list = []
    ori_hei_list = []
    depth_list = []
    roll_list = []
    pitch_list = []
    yaw_list = []
    u_list = []
    v_list = []
    w_list = []
    p_list = []
    q_list = []
    r_list = []
    vel_n_list = []
    vel_e_list = []
    vel_d_list = []
    path_course_list = []
    ye_list = []
    # Control reference
    desired_course_list = []
    desired_depth_list = []
    desired_pitch_list = []
    desired_rudder_list = []
    desired_stern_list = []
    # Rudder&Stern
    desired_fin0_list = []
    desired_fin1_list = []
    desired_fin2_list = []
    desired_fin3_list = []
    # Thruster
    pwm_list = []
    rpm_list = []
    # Binocular
    bin_x_list = []
    bin_y_list = []
    bin_z_list = []
    # USBL
    usbl_x_list = []
    usbl_y_list = []
    usbl_z_list = []
    usbl_roll_list = []
    usbl_pitch_list = []
    usbl_yaw_list = []
    usbl_lng_list = []
    usbl_lat_list = []
    # Log
    rise_res_list = []
    emg_level_list = []
    emg_state_list = []

    for i in range(param_len):
        time_list.append(param_list[i].time_)
        ctrl_mode_list.append(param_list[i].ctrl_mode_)
        mission_list.append(param_list[i].mission_)
        is_mission_on_list.append(param_list[i].is_mission_on_)
        lng_ori_list.append(param_list[i].lng_ori_)
        lat_ori_list.append(param_list[i].lat_ori_)
        # Status
        lng_list.append(param_list[i].lng_)
        lat_list.append(param_list[i].lat_)
        x_list.append(param_list[i].x_)
        y_list.append(param_list[i].y_)
        hei_list.append(param_list[i].hei_)
        ori_hei_list.append(param_list[i].ori_)
        depth_list.append(param_list[i].depth_)
        roll_list.append(param_list[i].roll_)
        pitch_list.append(param_list[i].pitch_)
        yaw_list.append(param_list[i].yaw_)
        u_list.append(param_list[i].u_)
        v_list.append(param_list[i].v_)
        w_list.append(param_list[i].w_)
        p_list.append(param_list[i].p_)
        q_list.append(param_list[i].q_)
        r_list.append(param_list[i].r_)
        vel_n_list.append(param_list[i].vel_n_)
        vel_e_list.append(param_list[i].vel_e_)
        vel_d_list.append(param_list[i].vel_d_)
        path_course_list.append(param_list[i].path_course_)
        ye_list.append(param_list[i].ye_)
        # Control reference
        desired_course_list.append(param_list[i].desired_course_)
        desired_depth_list.append(param_list[i].desired_depth_)
        desired_pitch_list.append(param_list[i].desired_pitch_)
        desired_rudder_list.append(param_list[i].desired_rudder_)
        desired_stern_list.append(param_list[i].desired_stern_)
        # Rudder&Stern
        desired_fin0_list.append(param_list[i].desired_fin0_)
        desired_fin1_list.append(param_list[i].desired_fin1_)
        desired_fin2_list.append(param_list[i].desired_fin2_)
        desired_fin3_list.append(param_list[i].desired_fin3_)
        # Thruster
        pwm_list.append(param_list[i].pwm_)
        rpm_list.append(param_list[i].rpm_)
        # Binocular
        bin_x_list.append(param_list[i].bin_x_)
        bin_y_list.append(param_list[i].bin_y_)
        bin_z_list.append(param_list[i].bin_z_)
        # USBL
        usbl_x_list.append(param_list[i].usbl_x_)
        usbl_y_list.append(param_list[i].usbl_y_)
        usbl_z_list.append(param_list[i].usbl_z_)
        usbl_roll_list.append(param_list[i].usbl_roll_)
        usbl_pitch_list.append(param_list[i].usbl_pitch_)
        usbl_yaw_list.append(param_list[i].usbl_yaw_)
        usbl_lng_list.append(param_list[i].usbl_lng_)
        usbl_lat_list.append(param_list[i].usbl_lat_)
        # Log
        rise_res_list.append(param_list[i].rise_res_)
        emg_level_list.append(param_list[i].emg_level_)
        emg_state_list.append(param_list[i].emg_state_)


        # plot 
        print('<plot auv324 database parameters')
        plot2D(lng_list, lat_list, 'lng', 'lat', path+'{}_lng-lat.png'.format(prefix), 'b-'):
        plot2D(y_list, x_list, 'y', 'x', path+'{}_y-x.png'.format(prefix), 'b-'):
        plot2D(time_list, y_list, 'time', 'y', path+'{}_time-y.png'.format(prefix), 'b-'):
        plot2D(time_list, yaw_list, 'time', 'yaw', path+'{}_time-yaw.png'.format(prefix), 'b-')
        plotMultiCurve(time_list, [desired_fin0_list, desired_fin1_list, desired_fin2_list, desired_fin3_list], 'time', 'fin angle', ['fin0', 'fin1', 'fin2', 'fin3'], path, ['r-', 'b-', 'g-', 'c-'], 4):
        plot2D(time_list, rpm_list, 'time', 'rpm', path+'{}_time-rpm.png'.format(prefix), 'r-')

        
'''
class auvOutlineControl():
    def __init__(self, auv_name, db_fn, table, date):
        # initialize
        print('<auvOutlineControl>: initialize')
        self.thruster0_pub_ = rospy.Publisher(auv_name + '/thruster/0/input', FloatStamped, queue_size = 1)
        self.fin0_pub_ = rospy.Publisher(auv_name + '/fins/0/input', FloatStamped, queue_size = 1)
        self.fin1_pub_ = rospy.Publisher(auv_name + '/fins/1/input', FloatStamped, queue_size = 1)
        self.fin2_pub_ = rospy.Publisher(auv_name + '/fins/2/input', FloatStamped, queue_size = 1)
        self.fin3_pub_ = rospy.Publisher(auv_name + '/fins/3/input', FloatStamped, queue_size = 1)

        # parse
        print('<auvOutlineControl>: parse database')
        self.auv_db_parser_ = auv324DBParser(table, date)
        self.auv_db_parser_.parse_db(db_fn)
        self.param_list_ = self.auv_db_parser_.get_params()

        rate = rospy.Rate(10) # 10hz

        # publish
        print('<auvOutlineControl>: publish parsed parameters')
        for i in range(len(self.param_list_)):
            if not(rospy.is_shutdown()):
                # thruster
                th0_cmd = FloatStamped()
                th0_cmd.data = self.param_list_[i].rpm_
                self.thruster0_pub_.publish(th0_cmd)
                
                # fin0
                fin0_cmd = FloatStamped()
                fin0_cmd.data = self.param_list_[i].fin0_
                self.fin0_pub_.publish(fin0_cmd)
                
                # fin1
                fin1_cmd = FloatStamped()
                fin1_cmd.data = self.param_list_[i].fin1_
                self.fin1_pub_.publish(fin1_cmd)
                
                # fin2
                fin2_cmd = FloatStamped()
                fin2_cmd.data = self.param_list_[i].fin1_
                self.fin2_pub_.publish(fin2_cmd)
                
                # fin3
                fin3_cmd = FloatStamped()
                fin3_cmd.data = self.param_list_[i].fin1_
                self.fin3_pub_.publish(fin3_cmd)
                
                rate.sleep()
'''

