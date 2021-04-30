#!/usr/bin/env python
# coding=utf-8

import csv
import time, datetime
import rospy

from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

class auvParam():
    def __init__(self):
        self.ts_ = 0.0
        self.rpm_ = 0.0
        self.fin0_ = 0.0
        self.fin1_ = 0.0
        self.fin2_ = 0.0
        self.fin3_ = 0.0
        self.fin4_ = 0.0
        self.fin5_ = 0.0

    def set_auv_ff_param(self, ts, rpm, fin0, fin1, fin2, fin3, fin4, fin5):
        self.ts_ =  ts
        self.rpm_ = rpm
        self.fin0_ = fin0
        self.fin1_ = fin1
        self.fin2_ = fin2
        self.fin3_ = fin3
        self.fin4_ = fin4
        self.fin5_ = fin5

    def set_auv_noff_param(self, ts, rpm, fin0, fin1, fin2, fin3):
        self.ts_ = ts
        self.rpm = rpm
        self.fin0_ = fin0
        self.fin1_ = fin1
        self.fin2_ = fin2
        self.fin3_ = fin3
        self.fin4_ = 0.0
        self.fin5_ = 0.0

class auvCSVParser():
    def __init__(self, date, with_ff):
        self.param_list_ = []
        
        self.date_ = date

        self.with_ff_ = with_ff

        self.ts_ind_ = 0
        self.rpm_ind_ = 33

        self.fin0_ind_ = 37 # ff: right bow | x_type: upper port | no_ff: left stern
        self.fin1_ind_ = 36 # ff: left bow | x_type: upper starboard | no_ff: upper stern 
        self.fin2_ind_ = 38 # ff: left stern | x_type: lower port | no_ff: right stern
        self.fin3_ind_ = 34 # ff: upper stern | x_type: lower starboard | no_ff: lower stern
        self.fin4_ind_ = 39 # ff: right stern 
        self.fin5_ind_ = 35 # ff: lower stern

    def get_params(self):
        return self.param_list_

    def parse_csv(self, csv_fn):
        print('<auvCSVParser>: start parse')

        with open(csv_fn) as f:
            f_csv = csv.reader(f)
            k = 0
            # first row is params label
            for row in f_csv:
                if not(k == 0):
                    param = auvParam()
                    param.ts_ = int(time.mktime(time.strptime(self.date_ + ' ' + row[self.ts_ind_], '%Y-%m-%d %H:%M:%S.%f')))
                    param.rpm_ = float(row[self.rpm_ind_])
                    param.fin0_ = float(row[self.fin0_ind_]) / 57.3
                    param.fin1_ = float(row[self.fin1_ind_]) / 57.3
                    param.fin2_ = float(row[self.fin2_ind_]) / 57.3
                    param.fin3_ = float(row[self.fin3_ind_]) / 57.3
                    param.fin4_ = float(row[self.fin4_ind_]) / 57.3
                    param.fin5_ = float(row[self.fin5_ind_]) / 57.3
                    self.param_list_.append(param)

                k += 1

class auvOutlineControl():
    def __init__(self, auv_name, with_ff, csv_fn, date):
        # initialize
        print('<auvOutlineControl>: initialize')
        self.thruster0_pub_ = rospy.Publisher(auv_name + '/thrusters/0/input', FloatStamped, queue_size = 1)
        self.fin0_pub_ = rospy.Publisher(auv_name + '/fins/0/input', FloatStamped, queue_size = 1)
        self.fin1_pub_ = rospy.Publisher(auv_name + '/fins/1/input', FloatStamped, queue_size = 1)
        self.fin2_pub_ = rospy.Publisher(auv_name + '/fins/2/input', FloatStamped, queue_size = 1)
        self.fin3_pub_ = rospy.Publisher(auv_name + '/fins/3/input', FloatStamped, queue_size = 1)
        self.fin4_pub_ = rospy.Publisher(auv_name + '/fins/4/input', FloatStamped, queue_size = 1)
        self.fin5_pub_ = rospy.Publisher(auv_name + '/fins/5/input', FloatStamped, queue_size = 1)

        # parse
        print('<auvOutlineControl>: parse csv')
        self.auv_csv_parser_ = auvCSVParser(date, with_ff)
        self.auv_csv_parser_.parse_csv(csv_fn)
        self.param_list_ = self.auv_csv_parser_.get_params()

        rate = rospy.Rate(100)

        # publish
        print('<auvOutlineControl>: publish parsed parameters')
        for i in range(len(self.param_list_)):
            if not(rospy.is_shutdown()):
                if with_ff:
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
                    fin2_cmd.data = self.param_list_[i].fin2_
                    self.fin2_pub_.publish(fin2_cmd)
                    
                    # fin3
                    fin3_cmd = FloatStamped()
                    fin3_cmd.data = self.param_list_[i].fin3_
                    self.fin3_pub_.publish(fin3_cmd)
                    
                    # fin4
                    fin4_cmd = FloatStamped()
                    fin4_cmd.data = self.param_list_[i].fin4_
                    self.fin4_pub_.publish(fin4_cmd)
                    
                    # fin5
                    fin5_cmd = FloatStamped()
                    fin5_cmd.data = self.param_list_[i].fin5_
                    self.fin5_pub_.publish(fin5_cmd)
                else:
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
                    fin2_cmd.data = self.param_list_[i].fin2_
                    self.fin2_pub_.publish(fin2_cmd)
                    
                    # fin3
                    fin3_cmd = FloatStamped()
                    fin3_cmd.data = self.param_list_[i].fin3_
                    self.fin3_pub_.publish(fin3_cmd)
                
                rate.sleep()


