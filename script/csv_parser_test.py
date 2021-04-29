#!/usr/bin/env python
# coding=utf-8

import csv
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

def parse_csv(csv_fn):
    # open csv
    with open(csv_fn) as f:
        f_csv = csv.reader(f)
        for row in f_csv:
            for i in range(len(row)):
                print('{}'.format(row[i]))

if __name__ == '__main__':
    parse_csv('../csv/T20201019.csv')
