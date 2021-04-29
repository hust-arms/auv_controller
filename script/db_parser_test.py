#!/usr/bin/env python
# coding=utf-8

import sqlite3
import time, datetime

# AUV actuator status parameters 
class auvParam():
    def __init__(self):
        self.ts_ = 0
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


def parse_db(db_fn):
    # open database
    conn = sqlite3.connect(db_fn)

    # parse data
    c = conn.cursor()

    # create auv param object
    cursor = c.execute('SELECT * from T20201020')

    param_list = []

    # values = cursor.fetchall()
    # print(values)

    # parse
    for row in cursor:
        print('col: {}'.format(len(row)))
        param = auvParam()
        param.ts_ = int(time.mktime(time.strptime('2020-10-20 ' + row[0], '%Y-%m-%d %H:%M:%S.%f')))
        param.rpm_ = row[30]
        param.fin0_ = row[40]
        param.fin1_ = row[39]
        param.fin2_ = row[37]
        param.fin3_ = row[35]
        param.fin4_ = row[38]
        param.fin5_ = row[36]
        print('Parse params: {} {} {} {} {} {} {} {}'.format(param.ts_, param.rpm_, \
            param.fin0_, param.fin1_, param.fin2_, param.fin3_, param.fin4_, param.fin5_))
        param_list.append(param)

if __name__ == '__main__':
    parse_db('../db/AUV_DB')
