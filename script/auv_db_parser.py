#!/usr/bin/env python
# coding=utf-8

import sqlite3
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

class auvDBParser():
    def __init__(self, table, date):
        self.param_list_ = []
        self.table_ = table
        self.date_ = date

        self.ts_ind_ = 0
        self.rpm_ind_ = 30
        self.fin0_ind_ = 40
        self.fin1_ind_ = 39
        self.fin2_ind_ = 37
        self.fin3_ind_ = 35
        self.fin4_ind_ = 38
        self.fin5_ind_ = 36

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
                param = auvParam()
                param.ts_ = int(time.mktime(time.strptime(self.date_ + ' ' + row[0], '%Y-%m-%d %H:%M:%S.%f')))
                param.rpm_ = row[self.ts_ind_]
                param.fin0_ = row[self.fin0_ind_]
                param.fin1_ = row[self.fin1_ind_]
                param.fin2_ = row[self.fin2_ind_]
                param.fin3_ = row[self.fin3_ind_]
                param.fin4_ = row[self.fin4_ind_]
                param.fin5_ = row[self.fin5_ind_]
                self.param_list_.append(param)
        except sqlite3.DatabaseError:
            pass

        cursor.close()
        conn.close()
        print('<auvParser>: parse finish')

class auvOutlineControl():
    def __init__(self, auv_name, db_fn, table, date):
        # initialize
        print('<auvOutlineControl>: initialize')
        self.thruster0_pub_ = rospy.Publisher(auv_name + '/thruster/0/input', FloatStamped, queue_size = 1)
        self.fin0_pub_ = rospy.Publisher(auv_name + '/fins/0/input', FloatStamped, queue_size = 1)
        self.fin1_pub_ = rospy.Publisher(auv_name + '/fins/1/input', FloatStamped, queue_size = 1)
        self.fin2_pub_ = rospy.Publisher(auv_name + '/fins/2/input', FloatStamped, queue_size = 1)
        self.fin3_pub_ = rospy.Publisher(auv_name + '/fins/3/input', FloatStamped, queue_size = 1)
        self.fin4_pub_ = rospy.Publisher(auv_name + '/fins/4/input', FloatStamped, queue_size = 1)
        self.fin5_pub_ = rospy.Publisher(auv_name + '/fins/5/input', FloatStamped, queue_size = 1)

        # parse
        print('<auvOutlineControl>: parse database')
        self.auv_db_parser_ = auvDBParser(table, date)
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
                
                # fin4
                fin4_cmd = FloatStamped()
                fin4_cmd.data = self.param_list_[i].fin1_
                self.fin4_pub_.publish(fin4_cmd)
                
                # fin5
                fin5_cmd = FloatStamped()
                fin5_cmd.data = self.param_list_[i].fin1_
                self.fin5_pub_.publish(fin5_cmd)
                
                rate.sleep()


