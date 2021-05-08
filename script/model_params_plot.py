#!/usr/bin/env python
# coding=utf-8

import sys

import csv

import time, datetime

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# @Parser 
# Pose use degree not rad 
def model_params_parse(csv_fn, is_outline, stamp):
    print('<model_params_plot>: start parse model params')

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

    print('<model_params_plot>: open target file')
    with open(csv_fn) as f:
        print('<model_params_plot>: read header')
        f_csv = csv.reader(f)
        is_parse = False
        k = 0
        print('<model_params_plot>: parse')
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
                roll.append(57.3*float(row[8])) 
                pitch.append(57.3*float(row[9])) 
                yaw.append(57.3*float(row[10])) 
                p.append(57.3*float(row[11])) 
                q.append(57.3*float(row[12])) 
                r.append(57.3*float(row[13])) 
                rpm.append(float(row[14])) 
                left_bow.append(57.3*float(row[15])) 
                right_bow.append(57.3*float(row[16])) 
                left_stern.append(57.3*float(row[17])) 
                right_stern.append(57.3*float(row[18])) 
                upper_stern.append(57.3*float(row[19])) 
                lower_stern.append(57.3*float(row[20])) 
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
                    ol_r.append(float(row[32])) 
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

            for i in range(len(row)):
                if not(row[i].find('depth') == -1):
                    # print('depth content: {}'.format(row[i]))
                    is_parse = True
                    break

    # plot and save
    print('<model_params_plot>: plot')

    print("len of x: {}".format(len(x)))
    print("len of y: {}".format(len(y)))

    plt.plot(time, x, 'r-')
    plt.xlabel('time')
    plt.ylabel('model x')
    plt.savefig('../plt/{}_time-x.png'.format(stamp))
    plt.close()

    plt.plot(time, y, 'r-')
    plt.xlabel('time')
    plt.ylabel('model y')
    plt.savefig('../plt/{}_time-y.png'.format(stamp))
    plt.close()

    plt.plot(time, z, 'r-')
    plt.xlabel('time')
    plt.ylabel('model z')
    plt.savefig('../plt/{}_time-z.png'.format(stamp))
    plt.close()

    plt.plot(time, u, 'r-')
    plt.xlabel('time')
    plt.ylabel('model u')
    plt.savefig('../plt/{}_time-u.png'.format(stamp))
    plt.close()

    plt.plot(time, v, 'r-')
    plt.xlabel('time')
    plt.ylabel('model v')
    plt.savefig('../plt/{}_time-v.png'.format(stamp))
    plt.close()

    plt.plot(time, w, 'r-')
    plt.xlabel('time')
    plt.ylabel('model w')
    plt.savefig('../plt/{}_time-w.png'.format(stamp))
    plt.close()

    plt.plot(time, roll, 'r-')
    plt.xlabel('time')
    plt.ylabel('model roll')
    plt.savefig('../plt/{}_time-roll.png'.format(stamp))
    plt.close()

    plt.plot(time, pitch, 'r-')
    plt.xlabel('time')
    plt.ylabel('model pitch')
    plt.savefig('../plt/{}_time-pitch.png'.format(stamp))
    plt.close()

    plt.plot(time, yaw, 'r-')
    plt.xlabel('time')
    plt.ylabel('model yaw')
    plt.savefig('../plt/{}_time-yaw.png'.format(stamp))
    plt.close()

    plt.plot(time, p, 'r-')
    plt.xlabel('time')
    plt.ylabel('model p')
    plt.savefig('../plt/{}_time-p.png'.format(stamp))
    plt.close()

    plt.plot(time, q, 'r-')
    plt.xlabel('time')
    plt.ylabel('model q')
    plt.savefig('../plt/{}_time-q.png'.format(stamp))
    plt.close()

    plt.plot(time, r, 'r-')
    plt.xlabel('time')
    plt.ylabel('model r')
    plt.savefig('../plt/{}_time-r.png'.format(stamp))
    plt.close()

    plt.plot(time, rpm, 'r-')
    plt.xlabel('time')
    plt.ylabel('model rpm')
    plt.savefig('../plt/{}_time-rpm.png'.format(stamp))
    plt.close()

    plt.plot(time, left_bow, 'r-')
    plt.xlabel('time')
    plt.ylabel('model left_bow')
    plt.savefig('../plt/{}_time-leftbow.png'.format(stamp))
    plt.close()

    plt.plot(time, right_bow, 'r-')
    plt.xlabel('time')
    plt.ylabel('model right_bow')
    plt.savefig('../plt/{}_time-rightbow.png'.format(stamp))
    plt.close()

    plt.plot(time, left_stern, 'r-')
    plt.xlabel('time')
    plt.ylabel('model left_stern')
    plt.savefig('../plt/{}_time-leftstern.png'.format(stamp))
    plt.close()

    plt.plot(time, right_stern, 'r-')
    plt.xlabel('time')
    plt.ylabel('model rightstern')
    plt.savefig('../plt/{}_time-rightstern.png'.format(stamp))
    plt.close()

    plt.plot(time, upper_stern, 'r-')
    plt.xlabel('time')
    plt.ylabel('model upper_stern')
    plt.savefig('../plt/{}_time-upperstern.png'.format(stamp))
    plt.close()

    plt.plot(time, lower_stern, 'r-')
    plt.xlabel('time')
    plt.ylabel('model lower_stern')
    plt.savefig('../plt/{}_time-lowerstern.png'.format(stamp))
    plt.close()

    if is_outline:
        dev_x = []; dev_y = []; dev_z = [] 
        dev_roll = []; dev_pitch = []; dev_yaw = []
        dev_u = []; dev_v = []
        dev_p = []; dev_q = []; dev_r = []

        for i in range(len(x)):
            dev_x.append(x[i] - ol_x[i])

        for i in range(len(y)):
            dev_y.append(y[i] - ol_y[i])

        for i in range(len(z)):
            dev_z.append(z[i] - ol_z[i])

        for i in range(len(roll)):
            dev_roll.append(roll[i] - ol_roll[i])

        for i in range(len(pitch)):
            dev_pitch.append(pitch[i] - ol_pitch[i])

        for i in range(len(yaw)):
            dev_yaw.append(yaw[i] - ol_yaw[i])

        for i in range(len(u)):
            dev_u.append(u[i] - ol_u[i])

        for i in range(len(v)):
            dev_v.append(v[i] - ol_v[i])

        for i in range(len(p)):
            dev_p.append(p[i] - ol_p[i])

        for i in range(len(q)):
            dev_q.append(q[i] - ol_q[i])

        for i in range(len(r)):
            dev_r.append(r[i] - ol_r[i])

        plt.plot(time, ol_x, 'r-')
        plt.xlabel('time')
        plt.ylabel('model outline_x')
        plt.savefig('../plt/{}_time-outlinex.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_y, 'r-')
        plt.xlabel('time')
        plt.ylabel('model outline_y')
        plt.savefig('../plt/{}_time-outliney.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_z, 'r-')
        plt.xlabel('time')
        plt.ylabel('model ol_z')
        plt.savefig('../plt/{}_time-outlinez.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_roll, 'r-')
        plt.xlabel('time')
        plt.ylabel('model outline_roll')
        plt.savefig('../plt/{}_time-outlineroll.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_pitch, 'r-')
        plt.xlabel('time')
        plt.ylabel('model outline_pitch')
        plt.savefig('../plt/{}_time-outliepitch.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_yaw, 'r-')
        plt.xlabel('time')
        plt.ylabel('model outline_yaw')
        plt.savefig('../plt/{}_time-outlineyaw.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_u, 'r-')
        plt.xlabel('time')
        plt.ylabel('model outline_u')
        plt.savefig('../plt/{}_time-outlineu.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_v, 'r-')
        plt.xlabel('time')
        plt.ylabel('model outline_v')
        plt.savefig('../plt/{}_time-outlinev.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_p, 'r-')
        plt.xlabel('time')
        plt.ylabel('model outline_p')
        plt.savefig('../plt/{}_time-outlinep.png'.format(stamp))
        plt.close()
        
        plt.plot(time, ol_q, 'r-')
        plt.xlabel('time')
        plt.ylabel('model outline_q')
        plt.savefig('../plt/{}_time-outlineq.png'.format(stamp))
        plt.close()
        
        # plt.plot(time, ol_r, 'r-')
        # plt.xlabel('time')
        # plt.ylabel('model outline_r')
        # plt.savefig('../plt/{}_time-outliner.png'.format(stamp))
        # plt.close()


        plt.plot(time, dev_x, 'b-')
        plt.xlabel('time')
        plt.ylabel('outline x deviation')
        plt.savefig('../plt/{}_time-outline-model-xdev.png'.format(stamp))

        plt.plot(time, dev_y, 'b-')
        plt.xlabel('time')
        plt.ylabel('outline y deviation')
        plt.savefig('../plt/{}_time-outline-model-ydev.png'.format(stamp))

        plt.plot(time, dev_z, 'b-')
        plt.xlabel('time')
        plt.ylabel('outline z deviation')
        plt.savefig('../plt/{}_time-outline-model-zdev.png'.format(stamp))

        plt.plot(time, dev_u, 'b-')
        plt.xlabel('time')
        plt.ylabel('outline u deviation')
        plt.savefig('../plt/{}_time-outline-model-udev.png'.format(stamp))

        plt.plot(time, dev_v, 'b-')
        plt.xlabel('time')
        plt.ylabel('outline v deviation')
        plt.savefig('../plt/{}_time-outline-model-vdev.png'.format(stamp))

        # w is ignored in outline data

        plt.plot(time, dev_roll, 'b-')
        plt.xlabel('time')
        plt.ylabel('outline roll deviation')
        plt.savefig('../plt/{}_time-outline-model-rolldev.png'.format(stamp))

        plt.plot(time, dev_pitch, 'b-')
        plt.xlabel('time')
        plt.ylabel('outline pitch deviation')
        plt.savefig('../plt/{}_time-outline-model-pitchdev.png'.format(stamp))

        plt.plot(time, dev_yaw, 'b-')
        plt.xlabel('time')
        plt.ylabel('outline yaw deviation')
        plt.savefig('../plt/{}_time-outline-model-yawdev.png'.format(stamp))

        plt.plot(time, dev_p, 'b-')
        plt.xlabel('time')
        plt.ylabel('outline p deviation')
        plt.savefig('../plt/{}_time-outline-model-pdev.png'.format(stamp))

        plt.plot(time, dev_q, 'b-')
        plt.xlabel('time')
        plt.ylabel('outline q deviation')
        plt.savefig('../plt/{}_time-outline-model-qdev.png'.format(stamp))

        plt.plot(time, dev_r, 'b-')
        plt.xlabel('time')
        plt.ylabel('outline r deviation')
        plt.savefig('../plt/{}_time-outline-model-rdev.png'.format(stamp))

    else:
        fig, ax = plt.subplots()
        ax.plot(time, y_d, 'b-', label='desired y')
        ax.plot(time, y, 'r-', label='model y')
        ax.set_xlabel('time')
        ax.set_ylabel('y')
        ax.legend()
        fig.savefig('../plt/{}_time-desiredy-y.png'.format(stamp))
        plt.close()

        fig, ax = plt.subplots()
        ax.plot(time, depth_d, 'b-', label='desired depth')
        ax.plot(time, depth, 'r-', label='model depth')
        ax.set_xlabel('time')
        ax.set_ylabel('depth')
        ax.legend()
        fig.savefig('../plt/{}_time-desireddepth-depth.png'.format(stamp))
        plt.close()

        fig, ax = plt.subplots()
        ax.plot(time, pitch_d, 'b-', label='desired pitch')
        ax.plot(time, pitch, 'r-', label='model pitch')
        ax.set_xlabel('time')
        ax.set_ylabel('pitch')
        ax.legend()
        fig.savefig('../plt/{}_time-desiredpitch-pitch.png'.format(stamp))
        plt.close()

        fig, ax = plt.subplots()
        ax.plot(time, yaw_d, 'b-', label='desired yaw')
        ax.plot(time, yaw, 'r-', label='model yaw')
        ax.set_xlabel('time')
        ax.set_ylabel('yaw')
        ax.legend()
        fig.savefig('../plt/{}_time-desiredyaw-yaw.png'.format(stamp))
        plt.close()

        plt.plot(time, depth_dev, 'r-')
        plt.xlabel('time')
        plt.ylabel('depth deviation')
        plt.savefig('../plt/{}_time-depthdev.png'.format(stamp))
        plt.close()

        plt.plot(time, latdist_dev, 'r-')
        plt.xlabel('time')
        plt.ylabel('lateraldistance deviation')
        plt.savefig('../plt/{}_time-latdev.png'.format(stamp))
        plt.close()

        plt.plot(time, yaw_dev, 'r-')
        plt.xlabel('time')
        plt.ylabel('yaw deviation')
        plt.savefig('../plt/{}_time-yawdev.png'.format(stamp))
        plt.close()

        plt.plot(time, pitch_dev, 'r-')
        plt.xlabel('time')
        plt.ylabel('pitch deviation')
        plt.savefig('../plt/{}_time-pitchdev.png'.format(stamp))
        plt.close()

if __name__ == '__main__':
    '''
    try:
        stamp = time.time()
        model_params_parse('../record/control_record_2021-May-07.csv', False, stamp)
    except BaseException:
        print('<model_params_parse>: call error')
        pass
    '''
    stamp = time.time()
    model_params_parse('../record/control_record_2021-May-07.csv', True, stamp)
        
