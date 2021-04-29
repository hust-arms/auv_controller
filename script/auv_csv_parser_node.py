#!/usr/bin/env python
# coding=utf-8

import rospy
import auv_csv_parser

if __name__ == '__main__':
    rospy.init_node('auv_csv_parser_node', anonymous=True)

    try:
        auv_csv_parser.auvOutlineControl('armsauv', '../csv/T20201019.csv', '2020-10-19')
    except rospy.ROSInterruptException:
        pass

