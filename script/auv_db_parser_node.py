#!/usr/bin/env python
# coding=utf-8

import rospy
import auv_db_parser

if __name__ == '__main__':
    rospy.init_node('auv_db_parser_node', anonymous = True)

    try:
        auv_db_parser.auvOutlineControl('armsauv', '../db/AUV_DB', 'T20201020', '2020-10-20')
    except rospy.ROSInterruptException:
        pass


