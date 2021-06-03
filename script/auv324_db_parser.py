#!/usr/bin/env python
# coding=utf-8

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sqlite3
import time, datetime

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
        # self.param_list_ = []
        self.table_ = table
        # self.mission_start_row_list_ = []
        # self.mission_end_row_list_ = []

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

    # def get_params(self):
    #     return self.param_list_
		
    def parse_mission(self, db_fn):
	print('<auvParser>: start parse mission')
        
        # open database
        conn = sqlite3.connect(db_fn)
		
	# parse data
	c = conn.cursor()
		
	cursor = c.execute('SELECT * from {}'.format(self.table_))
        print('<auvParser>: parsing')

        mission_start_row_list = []
        mission_end_row_list = []

        try:
            last_mission_flag = 0 #　default is standby
            last_is_mission_on = 0 #　default is off
            is_mission_parsed = False
            ind = 0
            for row in cursor:
                if last_is_mission_on == 0 and row[self.is_mission_on_ind_] == 1:
                    if not(is_mission_parsed):
                        print('mission start row: {}'.format(ind))
                        mission_start_row_list.append(ind)
                        is_mission_parsed = True # set mission parsed flag
                        last_mission_flag = row[self.mission_ind_] # set new mission type flag
                        last_is_mission_on = row[self.is_mission_on_ind_] # set new mission type flag

                    # if is_mission_parsed:
                    #     print('mission end row: {}'.format(ind))
                    #     mission_end_row_list.append(ind)
                    #     is_mission_parsed = False # reset mission parsed flag
                    # else:
                    #     print('mission start row: {}'.format(ind))
                    #     mission_start_row_list.append(ind)
                    #     is_mission_parsed = True # set mission parsed flag
                    #     last_mission_flag = row[self.mission_ind_] # set new mission type flag
 
                if last_is_mission_on == 1 and row[self.is_mission_on_ind_] == 0:
                    if is_mission_parsed:
                        print('mission end row: {}'.format(ind))
                        mission_end_row_list.append(ind)
                        is_mission_parsed = False
                        last_mission_flag = row[self.mission_ind_] # set new mission type flag
                        last_is_mission_on = row[self.is_mission_on_ind_] # set new mission type flag
                
                ind += 1

            #　process end of list
            if len(mission_end_row_list) == len(mission_start_row_list) - 1:
                mission_end_row_list.append(ind)
        except sqlite3.DatabaseError:
            pass
        cursor.close()
        conn.close()
        print('<auvParser>: field parse finish')

        list_len = -1
        if len(mission_start_row_list) == len(mission_end_row_list):
            list_len = len(mission_start_row_list)

        return [mission_start_row_list, mission_end_row_list, list_len]

    def parse_param(self, db_fn):
        print('<auvParser>: start parse parameters')

        # open database
        conn = sqlite3.connect(db_fn)
        
        # parse data
        c = conn.cursor()
        
        # create auv param object
        cursor = c.execute('SELECT * from {}'.format(self.table_))

        print('<auvParser>: parsing')
        try:
            param_list = []
            k = 0
            for row in cursor:
                # if k == 0 or k == 1:
                #     print(row[self.time_ind_])
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

                param_list.append(param)
                k += 1
        except sqlite3.DatabaseError:
            pass

        cursor.close()
        conn.close()
        print('<auvParser>: parse finish')

        return param_list

    def parse_field_param(self, db_fn, start_row, end_row):
        print('<auvParser>: start parse parameters in limit area')

        # open database
        conn = sqlite3.connect(db_fn)
        
        # parse data
        c = conn.cursor()
        
        # create auv param object
        cursor = c.execute('SELECT * from {}'.format(self.table_))

        print('<auvParser>: parsing')

        try:
            ind = 0
            param_list = []

            for row in cursor:
                if ind <= end_row - 1 and ind > start_row - 1:
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
                    
                    param_list.append(param)
                ind += 1
        except sqlite3.DatabaseError:
            pass
        cursor.close()
        conn.close()
        print('<auvParser>: field parse finish')

        return param_list
                

def plot2D(x, y, xlabel, ylabel, path, color):
    plt.plot(x, y, color)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.savefig(path)
    plt.close()

def plot2DWithxtick(x, y, xlabel, ylabel, path, color, xtick):
    plt.plot(x, y, color)
    plt.xlabel(xlabel)
    plt.xticks(xtick)
    plt.ylabel(ylabel)
    plt.savefig(path)
    plt.close()

def plotMultiCurve(x, y_l, xlabel, ylabel, ylabel_l, path, color_l, size, linestyles):
    fig, ax = plt.subplots()
    for i in range(size):
        ax.plot(x, y_l[i], color_l[i], label=ylabel_l[i], linestyle=linestyles[i])
        
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.legend()
    fig.savefig(path)
    plt.close()

def plotMultiCurveWithxtick(x, y_l, xlabel, ylabel, ylabel_l, path, color_l, size, linestyles, xtick):
    fig, ax = plt.subplots()
    for i in range(size):
        ax.plot(x, y_l[i], color_l[i], label=ylabel_l[i], linestyle=linestyles[i])
        
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_xticks(xtick)
    ax.legend()
    fig.savefig(path)
    plt.close()

def plotAUV324Param(param_list, prefix, path):
    # prefix = time.strftime("%Y-%m-%d-%H:%M:%S", time.localtime())

    param_len = len(param_list)

    print('initialization')

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
        time_list.append(int(time.mktime(time.strptime(param_list[i].time_, '%H:%M:%S.%f'))) - \
                int(time.mktime(time.strptime(param_list[0].time_, '%H:%M:%S.%f'))))
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
        ori_hei_list.append(param_list[i].ori_hei_)
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
    
    print('finish initialization')

    # Others
    course_err_list = []
    for i in range(len(desired_course_list)):
        course_err_list.append(desired_course_list[i] - yaw_list[i])

    # print('course error length: {}'.format(len(course_err_list)))

    depth_err_list = []
    for i in range(len(desired_depth_list)):
        depth_err_list.append(desired_depth_list[i] - depth_list[i])

    # print('depth error length: {}'.format(len(depth_err_list)))

    pitch_err_list = []
    for i in range(len(desired_pitch_list)):
        pitch_err_list.append(desired_pitch_list[i] - pitch_list[i])

    # print('pitch error length: {}'.format(len(pitch_err_list)))

    # plot 
    print('plot auv324 database parameters')
    # fig1
    print('plot lng-lat')
    plot2D(lng_list, lat_list, 'Lng[deg]', 'Lat[deg]', path+'{}_lng-lat.png'.format(prefix), 'b-')
    print('plot y-x')
    plot2D(y_list, x_list, 'Y[m]', 'X[m]', path+'{}_y-x.png'.format(prefix), 'b-')
    print('plot time-ye')
    plot2DWithxtick(time_list, ye_list, 'Time[s]', 'Ye[m]', path+'{}_time-ye.png'.format(prefix), 'b-', range(0, 120, 20))
    print('plot time-desired_course')
    plotMultiCurveWithxtick(time_list, [path_course_list, desired_course_list, yaw_list], 'Time[s]', 'Course[deg]', ['PathCourse', 'DesiredCourse', 'Yaw'], path+'{}_time-desired_course.png'.format(prefix), ['b-', 'r-', 'g-'], 3, ['-', '--', '-'], range(0, 120, 20))
    print('plot time-rudder')
    plot2DWithxtick(time_list, desired_rudder_list, 'Time[s]', 'Rudder[deg]', path+'{}_time-rudder.png'.format(prefix), 'b-', range(0, 120, 20))
    print('plot time-course_error')
    plot2DWithxtick(time_list, course_err_list, 'Time[s]', 'Course Error[deg]', path+'{}_time-course_error.png'.format(prefix), 'b-', range(0, 120, 20))
    # fig2
    print('plot time-depth')
    plotMultiCurveWithxtick(time_list, [desired_depth_list, depth_list], 'Time[s]', 'Depth[m]', ['DesiredDepth', 'Depth'], path+'{}_time-desired_depth.png'.format(prefix), ['r-', 'b-'], 2, ['--', '-'], range(0, 120, 20))
    # plotMultiCurveWithxtick(time_list, [desired_fin0_list, desired_fin1_list, desired_fin2_list, desired_fin3_list], 'time', 'fin angle', ['fin0', 'fin1', 'fin2', 'fin3'], path, ['r-', 'b-', 'g-', 'c-'], 4)
    print('plot time-height')
    plot2DWithxtick(time_list, hei_list, 'Time[s]', 'Height[m]', path+'{}_time-height.png'.format(prefix), 'b-', range(0, 120, 20))
    print('plot time-depth_error')
    plot2DWithxtick(time_list, depth_err_list, 'Time[s]', 'Depth Error[m]', path+'{}_time-depth_error.png'.format(prefix), 'b-', range(0, 120, 20))
    print('plot time-desired_pitch')
    plotMultiCurveWithxtick(time_list, [desired_pitch_list, pitch_list], 'Time[s]', 'Pitch[deg]', ['DesiredPitch', 'Pitch'], path+'{}_time-desired_pitch.png'.format(prefix), ['r-', 'b-'], 2, ['--', '-'], range(0, 120, 20))
    print('plot time-stern')
    plot2DWithxtick(time_list, desired_stern_list, 'Time[s]', 'Stern[deg]', path+'{}_time-stern.png'.format(prefix), 'b-', range(0, 120, 20))
    print('plot time-pitch_error')
    plot2DWithxtick(time_list, pitch_err_list, 'Time[s]', 'Pitch Error[deg]', path+'{}_time-pitch_error.png'.format(prefix), 'b-', range(0, 120, 20))
    # fig3
    print('plot time-u')
    plot2DWithxtick(time_list, u_list, 'Time[s]', 'Forward Velocity[m/s]', path+'{}_time-forward_velocity.png'.format(prefix), 'b-', range(0, 120, 20))
    print('plot time-north_vel')
    plot2DWithxtick(time_list, vel_n_list, 'Time[s]', 'North Velocity[m/s]', path+'{}_time-north_velocity.png'.format(prefix), 'b-', range(0, 120, 20))
    print('plot time-east_vel')
    plot2DWithxtick(time_list, vel_e_list, 'Time[s]', 'East Velocity[m/s]', path+'{}_time-east_velocity.png'.format(prefix), 'b-', range(0, 120, 20))
    print('plot time-depth_vel')
    plot2DWithxtick(time_list, vel_d_list, 'Time[s]', 'Depth Velocity[m/s]', path+'{}_time-depth_velocity.png'.format(prefix), 'b-', range(0, 120, 20))
    # fig4
    print('plot time-roll')
    plot2DWithxtick(time_list, roll_list, 'Time[s]', 'Roll[deg]', path+'{}_time-roll.png'.format(prefix), 'b-', range(0, 120, 20))
    print('plot time-pitch')
    plot2DWithxtick(time_list, pitch_list, 'Time[s]', 'Pitch[deg]', path+'{}_time-pitch.png'.format(prefix), 'b-', range(0, 120, 20))
    print('plot time-yaw')
    plot2DWithxtick(time_list, yaw_list, 'Time[s]', 'Yaw[deg]', path+'{}_time-yaw.png'.format(prefix), 'b-', range(0, 120, 20))
    # fig5
    print('plot time-emerge_level')
    plot2DWithxtick(time_list, emg_level_list, 'Time[s]', 'Emerge Level', path+'{}_time-emerge_level.png'.format(prefix), 'b-', range(0, 120, 20))
    print('plot time-emerge_state')
    plot2DWithxtick(time_list, emg_state_list, 'Time[s]', 'Emerge State', path+'{}_time-emerge_state.png'.format(prefix), 'b-', range(0, 120, 20))
    print('plot time-pwm')
    plot2DWithxtick(time_list, pwm_list, 'Time[s]', 'PWM', path+'{}_time-pwm.png'.format(prefix), 'b-', range(0, 120, 20))
    print('plot time-rpm')
    plot2DWithxtick(time_list, rpm_list, 'Time[s]', 'RPM', path+'{}_time-rpm.png'.format(prefix), 'b-', range(0, 120, 20))

    print('finish plot')
        
def subPlotAUV324Param(param_list, prefix, path):
    # prefix = time.strftime("%Y-%m-%d-%H:%M:%S", time.localtime())

    param_len = len(param_list)

    print('initialization')

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
        ori_hei_list.append(param_list[i].ori_hei_)
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
    
    print('finish initialization')

    # Others
    course_err_list = []
    for i in range(len(desired_course_list)):
        course_err_list.append(desired_course_list[i] - yaw_list[i])

    # print('course error length: {}'.format(len(course_err_list)))

    depth_err_list = []
    for i in range(len(desired_depth_list)):
        depth_err_list.append(desired_depth_list[i] - depth_list[i])

    # print('depth error length: {}'.format(len(depth_err_list)))

    pitch_err_list = []
    for i in range(len(desired_pitch_list)):
        pitch_err_list.append(desired_pitch_list[i] - pitch_list[i])

    # plot
    # fig1 
    plt.subplot(2, 3, 1)
    plt.plot(lng_list, lat_list, 'b-')
    plt.xlabel('Lng[deg]')
    plt.ylabel('Lat[deg]')
    plt.subplot(2, 3, 2)
    plt.plot(y_list, x_list, 'b-')
    plt.xlabel('Y[m]')
    plt.ylabel('X[m]')
    plt.subplot(2, 3, 3)
    plt.plot(time_list, ye_list, 'b-')
    plt.xlabel('Time[s]')
    plt.ylabel('Ye[m]')
    plt.subplot(2, 3, 4)
    fig, ax = plt.subplots()
    ax.plot(time_list, path_course_list, 'g-', 'Path Course')
    ax.plot(time_list, desired_course_list, 'r-', 'Desired Course')
    ax.plot(time_list, yaw_list, 'g-', 'Yaw')
    ax.set_xlabel('Time[s]')
    ax.set_ylabel('Course[deg]')
    ax.legend()
    plt.subplot(2, 3, 5)
    plt.plot(time_list, desired_rudder_list, 'b-')
    plt.xlabel('Time[s]')
    plt.ylabel('Ye[m]')
    plt.subplot(2, 3, 6)
    plt.plot(time_list, course_err_list, 'b-')
    plt.xlabel('Time[s]')
    plt.ylabel('Course Error[deg]')
    plt.savefig(path+'{}_fig1.png'.format(prefix))
    # fig2
    plt.subplot(2, 3, 1)
    fig, ax = plt.subplots()
    ax.plot(time_list, depth_list, 'b-', 'Depth')
    ax.plot(time_list, desired_depth_list, 'r-', 'Desired depth')
    ax.set_xlabel('Time[s]')
    ax.set_ylabel('Depth[m]')
    ax.legend()
    plt.subplot(2, 3, 2)
    plt.plot(time_list, hei_list, 'b-')
    plt.xlabel('Time[s]')
    plt.ylabel('Height[m]')
    plt.subplot(2, 3, 3)
    plt.plot(time_list, depth_err_list, 'b-')
    plt.xlabel('Time[s]')
    plt.ylabel('Depth Error[m]')
    plt.subplot(2, 3, 4)
    fig, ax = plt.subplots()
    ax.plot(time_list, pitch_list, 'b-', 'Pitch')
    ax.plot(time_list, desired_pitch_list, 'r-', 'Desired Pitch')
    ax.set_xlabel('Time[s]')
    ax.set_ylabel('Pitch[deg]')
    ax.legend()
    plt.subplot(2, 3, 5)
    plt.plot(time_list, desired_stern_list, 'b-')
    plt.xlabel('Time[s]')
    plt.ylabel('Stern[deg]')
    plt.subplot(2, 3, 6)
    plt.plot(time_list, pitch_err_list, 'b-')
    plt.xlabel('Time[s]')
    plt.ylabel('Pitch Error[m]')
    plt.savefig(path+'{}_fig2.png'.format(prefix))

'''
class auvOutlineControl():
    def __init__(self, auv_name, db_fn, table, date):
        # initialize
        print('<auvOutlineControl>: initialize')
        self.thruster0_pub_ = rospy.Publisher(auv_name + '/thruster/0/input', FloatStamped, queue_size = 1)
        self.fin0_pub_ = rospy.Publisher(auv_name + '/fins/0/input', FloatStamped, queue_size = 1) self.fin1_pub_ = rospy.Publisher(auv_name + '/fins/1/input', FloatStamped, queue_size = 1)
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

'''
# test interface: parse_param
if __name__ == '__main__':
    try:
        db_parser = auv324DBParser('T20210602')
        param_list = db_parser.parse_param('AUV_DB')

    except Exception:
        print('<auv324_db_parser_test_node>: test failed') 
        pass
'''

'''
# test interface: parse_mission
if __name__ == '__main__':
    try:
        db_parser = auv324DBParser('T20210602')
        [mission_start_row_list, mission_end_row_list, list_len] = db_parser.parse_mission('AUV_DB')

    except Exception:
        print('<auv324_db_parser_test_node>: test failed') 
        pass
'''

'''
# test interface: parse_field_param
if __name__ == '__main__':
    db_parser = auv324DBParser('T20210602')
    param_list1 = db_parser.parse_field_param('AUV_DB', 14377, 14495)
    # param_list2 = db_parser.parse_field_param('AUV_DB', 15105, 15242)

    print('<auv324DBParser>: param length: {}'.format(len(param_list1)))

    plotAUV324Param(param_list1, 'nav1', 'plot/')
'''

# test interface: parse_field_param
if __name__ == '__main__':
    db_parser = auv324DBParser('T20210602')
    [mission_start_row_list, mission_end_row_list, list_len] = db_parser.parse_mission('AUV_DB')
    for i in range(list_len):
        param_list = db_parser.parse_field_param('AUV_DB', mission_start_row_list[i], mission_end_row_list[i])
        plotAUV324Param(param_list, 'nav{}'.format(i), 'plot/')
'''
# test interface: parse_field_param
if __name__ == '__main__':
    db_parser = auv324DBParser('T20210602')
    [mission_start_row_list, mission_end_row_list, list_len] = db_parser.parse_mission('AUV_DB')
    for i in range(list_len):
        param_list = db_parser.parse_field_param('AUV_DB', mission_start_row_list[i], mission_end_row_list[i])
        subPlotAUV324Param(param_list, 'nav{}'.format(i), 'plot/')
'''
    
