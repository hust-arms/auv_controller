#!/usr/bin/env python
# coding=utf-8

import sys

import csv

import time, datetime

import matplotlib
import matplotlib.pyplot as plt

# Parser 
def model_params_parse(csv_fn, is_outline, stamp):
    print('<model_params_plot>: parse model params')

    # parameter lists
    time = []
    x = []; y = []; z = []; depth = []
    u = []; v = []; w = []
    roll = []; pitch = []; yaw = []
    p = []; q = []; r = []
    rpm = []
    left_bow = []; right_bow = []
    left_stern = []; right_stern = []; upper_stern = []; lower_stern = []

    ts = []; 
    ol_x = []; ol_y = []; ol_z = []
    ol_roll = []; ol_pitch = []; ol_yaw = []
    ol_u = []; ol_v = []; 
    ol_p = []; ol_q = []; ol_r = []

    x_d = []; y_d = []; depth_d = []
    pitch_d = []; yaw_d = []; u_d = []
    depth_dev = []; latdist_dev = []
    yaw_dev = []; pitch_dev = []

    with open(csv_fn) as f:
        f_csv = csv.reader(f)
        is_parse = False
        for row in f_csv:  
            if is_parse:
                time.append(float(row[0])) 
                x.append(float(row[1])) 
                y.append(float(row[2])) 
                z.append(float(row[3])) 
                depth.append(float(row[4])) 
                u.append(float(row[5])) 
                v.append(float(row[6])) 
                w.append(float(row[7])) 
                roll.append(float(row[8])) 
                pitch.append(float(row[9])) 
                yaw.append(float(row[10])) 
                p.append(float(row[11])) 
                q.append(float(row[12])) 
                r.append(float(row[13])) 
                rpm.append(float(row[14])) 
                left_bow.append(float(row[15])) 
                right_bow.append(float(row[16])) 
                left_stern.append(float(row[17])) 
                right_stern.append(float(row[18])) 
                upper_stern.append(float(row[19])) 
                lower_stern.append(float(row[20])) 
                if is_outline:
                    ts.append(float(row[21])) 
                    ol_x.append(float(row[22])) 
                    ol_y.append(float(row[23])) 
                    ol_z.append(float(row[24])) 
                    ol_u.append(float(row[25])) 
                    ol_v.append(float(row[26])) 
                    ol_roll.append(float(row[27])) 
                    ol_pitch.append(float(row[28])) 
                    ol_yaw.append(float(row[29])) 
                    ol_p.append(float(row[30])) 
                    ol_q.append(float(row[31])) 
                    ol_r.append(float(row[33])) 
                else:
                    x_d.append(float(row[21]))
                    y_d.append(float(row[22]))
                    depth_d.append(float(row[23]))
                    pitch_d.append(float(row[24]))
                    yaw_d.append(float(row[25]))
                    u_d.append(float(row[26]))
                    depth_dev.append(float(row[27]))
                    latdist_dev.append(float(row[28]))
                    yaw_dev.append(float(row[29]))
                    pitch_dev.append(float(row[30]))

            if not(row.find('mission')):
                is_plot = True

    # plot and save
    plt.plot(time, x)
    plt.xlabel('time')
    plt.ylabel('model x')
    plt.savefig('../plt/{}/time-x.png'.format(stamp).format(stamp))
    plt.close()

    plt.plot(time, y)
    plt.xlabel('time')
    plt.ylabel('model y')
    plt.savefig('../plt/{}/time-x.png'.format(stamp).format(stamp))
    plt.close()

    plt.plot(time, z)
    plt.xlabel('time')
    plt.ylabel('model z')
    plt.savefig('../plt/{}/time-y.png'.format(stamp))
    plt.close()

    plt.plot(time, u)
    plt.xlabel('time')
    plt.ylabel('model u')
    plt.savefig('../plt/{}/time-z.png'.format(stamp))
    plt.close()

    plt.plot(time, v)
    plt.xlabel('time')
    plt.ylabel('model v')
    plt.savefig('../plt/{}/time-v.png'.format(stamp))
    plt.close()

    plt.plot(time, w)
    plt.xlabel('time')
    plt.ylabel('model w')
    plt.savefig('../plt/{}/time-w.png'.format(stamp))
    plt.close()

    plt.plot(time, roll)
    plt.xlabel('time')
    plt.ylabel('model roll')
    plt.savefig('../plt/{}/time-roll.png'.format(stamp))
    plt.close()

    plt.plot(time, pitch)
    plt.xlabel('time')
    plt.ylabel('model pitch')
    plt.savefig('../plt/{}/time-pitch.png'.format(stamp))
    plt.close()

    plt.plot(time, yaw)
    plt.xlabel('time')
    plt.ylabel('model yaw')
    plt.savefig('../plt/{}/time-yaw.png'.format(stamp))
    plt.close()

    plt.plot(time, p)
    plt.xlabel('time')
    plt.ylabel('model p')
    plt.savefig('../plt/{}/time-p.png'.format(stamp))
    plt.close()

    plt.plot(time, q)
    plt.xlabel('time')
    plt.ylabel('model q')
    plt.savefig('../plt/{}/time-q.png'.format(stamp))
    plt.close()

    plt.plot(time, r)
    plt.xlabel('time')
    plt.ylabel('model r')
    plt.savefig('../plt/{}/time-r.png'.format(stamp))
    plt.close()

    plt.plot(time, rpm)
    plt.xlabel('time')
    plt.ylabel('model rpm')
    plt.savefig('../plt/{}/time-rpm.png'.format(stamp))
    plt.close()

    plt.plot(time, left_bow)
    plt.xlabel('time')
    plt.ylabel('model left_bow')
    plt.savefig('../plt/{}/time-leftbow.png'.format(stamp))
    plt.close()

    plt.plot(time, right_bow)
    plt.xlabel('time')
    plt.ylabel('model right_bow')
    plt.savefig('../plt/{}/time-rightbow.png'.format(stamp))
    plt.close()

    plt.plot(time, left_stern)
    plt.xlabel('time')
    plt.ylabel('model left_stern')
    plt.savefig('../plt/{}/time-leftstern.png'.format(stamp))
    plt.close()

    plt.plot(time, right_stern)
    plt.xlabel('time')
    plt.ylabel('model rightstern')
    plt.savefig('../plt/{}/time-rightstern.png'.format(stamp))
    plt.close()

    plt.plot(time, upper_stern)
    plt.xlabel('time')
    plt.ylabel('model upper_stern')
    plt.savefig('../plt/{}/time-upperstern.png'.format(stamp))
    plt.close()

    plt.plot(time, lower_stern)
    plt.xlabel('time')
    plt.ylabel('model lower_stern')
    plt.savefig('../plt/{}/time-lowerstern.png'.format(stamp))
    plt.close()

    if is_outline:
        plt.plot(time, ol_x)
        plt.xlabel('time')
        plt.ylabel('model outline_x')
        plt.savefig('../plt/{}/time-outlinex.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_y)
        plt.xlabel('time')
        plt.ylabel('model outline_y')
        plt.savefig('../plt/{}/time-outliney.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_z)
        plt.xlabel('time')
        plt.ylabel('model ol_z')
        plt.savefig('../plt/{}/time-outlinez.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_roll)
        plt.xlabel('time')
        plt.ylabel('model outline_roll')
        plt.savefig('../plt/{}/time-outlineroll.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_pitch)
        plt.xlabel('time')
        plt.ylabel('model outline_pitch')
        plt.savefig('../plt/{}/time-outliepitch.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_yaw)
        plt.xlabel('time')
        plt.ylabel('model outline_yaw')
        plt.savefig('../plt/{}/time-outlineyaw.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_u)
        plt.xlabel('time')
        plt.ylabel('model outline_u')
        plt.savefig('../plt/{}/time-outlineu.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_v)
        plt.xlabel('time')
        plt.ylabel('model outline_v')
        plt.savefig('../plt/{}/time-outlinev.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_p)
        plt.xlabel('time')
        plt.ylabel('model outline_p')
        plt.savefig('../plt/{}/time-outlinep.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_q)
        plt.xlabel('time')
        plt.ylabel('model outline_q')
        plt.savefig('../plt/{}/time-outlineq.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_r)
        plt.xlabel('time')
        plt.ylabel('model outline_r')
        plt.savefig('../plt/{}/time-outliner.png'.format(stamp))
        plt.close()

        plt.plot(time, x - ol_x)
        plt.xlabel('time')
        plt.ylabel('outline x deviation')
        plt.savefig('../plt/{}/time-outline-model-xdev.png'.format(stamp))

        plt.plot(time, y - ol_y)
        plt.xlabel('time')
        plt.ylabel('outline y deviation')
        plt.savefig('../plt/{}/time-outline-model-ydev.png'.format(stamp))

        plt.plot(time, z - ol_z)
        plt.xlabel('time')
        plt.ylabel('outline z deviation')
        plt.savefig('../plt/{}/time-outline-model-zdev.png'.format(stamp))

        plt.plot(time, u - ol_u)
        plt.xlabel('time')
        plt.ylabel('outline u deviation')
        plt.savefig('../plt/{}/time-outline-model-udev.png'.format(stamp))

        plt.plot(time, v - ol_v)
        plt.xlabel('time')
        plt.ylabel('outline v deviation')
        plt.savefig('../plt/{}/time-outline-model-vdev.png'.format(stamp))

        # w is ignored in outline data

        plt.plot(time, roll - ol_roll)
        plt.xlabel('time')
        plt.ylabel('outline roll deviation')
        plt.savefig('../plt/{}/time-outline-model-rolldev.png'.format(stamp))

        plt.plot(time, pitch - ol_pitch)
        plt.xlabel('time')
        plt.ylabel('outline pitch deviation')
        plt.savefig('../plt/{}/time-outline-model-pitchdev.png'.format(stamp))

        plt.plot(time, yaw - ol_yaw)
        plt.xlabel('time')
        plt.ylabel('outline yaw deviation')
        plt.savefig('../plt/{}/time-outline-model-yawdev.png'.format(stamp))

        plt.plot(time, p - ol_p)
        plt.xlabel('time')
        plt.ylabel('outline p deviation')
        plt.savefig('../plt/{}/time-outline-model-pdev.png'.format(stamp))

        plt.plot(time, q - ol_q)
        plt.xlabel('time')
        plt.ylabel('outline q deviation')
        plt.savefig('../plt/{}/time-outline-model-qdev.png'.format(stamp))

        plt.plot(time, r - ol_r)
        plt.xlabel('time')
        plt.ylabel('outline r deviation')
        plt.savefig('../plt/{}/time-outline-model-rdev.png'.format(stamp))

    else:
        fig, ax = plt.subplots()
        ax.plot(time, y_d, label='desired y')
        ax.plot(time, y, label='model y')
        ax.set_xlabel('time')
        ax.set_ylabel('y')
        ax.legend()
        fig.savefig('../plt/{}/time-desiredy-y.png'.format(stamp))
        plt.close()

        fig, ax = plt.subplots()
        ax.plot(time, depth_d, label='desired depth')
        ax.plot(time, depth, label='model depth')
        ax.set_xlabel('time')
        ax.set_ylabel('depth')
        ax.legend()
        fig.savefig('../plt/{}/time-desireddepth-depth.png'.format(stamp))
        plt.close()

        fig, ax = plt.subplots()
        ax.plot(time, pitch_d, label='desired pitch')
        ax.plot(time, pitch, label='model pitch')
        ax.set_xlabel('time')
        ax.set_ylabel('pitch')
        ax.legend()
        fig.savefig('../plt/{}/time-desiredpitch-pitch.png'.format(stamp))
        plt.close()

        fig, ax = plt.subplots()
        ax.plot(time, yaw_d, label='desired yaw')
        ax.plot(time, yaw, label='model yaw')
        ax.set_xlabel('time')
        ax.set_ylabel('yaw')
        ax.legend()
        fig.savefig('../plt/{}/time-desiredyaw-yaw.png'.format(stamp))
        plt.close()

        plt.plot(time, depth_dev)
        plt.xlabel('time')
        plt.ylabel('depth deviation')
        plt.savefig('../plt/{}/time-depthdev.png'.format(stamp))
        plt.close()

        plt.plot(time, latdist_dev)
        plt.xlabel('time')
        plt.ylabel('lateraldistance deviation')
        plt.savefig('../plt/{}/time-latdev.png'.format(stamp))
        plt.close()

        plt.plot(time, yaw_dev)
        plt.xlabel('time')
        plt.ylabel('yaw deviation')
        plt.savefig('../plt/{}/time-yawdev.png'.format(stamp))
        plt.close()

        plt.plot(time, pitch_dev)
        plt.xlabel('time')
        plt.ylabel('pitch deviation')
        plt.savefig('../plt/{}/time-pitchdev.png'.format(stamp))
        plt.close()

if __name__ == '__main__':
    try:
        stamp = time.time()
        model_params_parse('', False, stamp):
    except rospy.ROSInterruptException:
        print('<model_params_parse>: call error')
        pass
        
