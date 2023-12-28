#!/usr/bin/env python
# coding: utf-8


# ## 1. read 2D point cloud
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal
from math import *




df = pd.read_csv(r'output_zhang_ascend.csv', encoding='utf-8', header=None, low_memory=False)
data = df.to_numpy()
print(data.shape)
print('----------------------')
print(data)


# ## 2. split all data to each scan 
num_scan = 1 # 1 scan = 360 degree
num_beam = 0 # 1 beam = 0.12 degree
previous_degree = 0 # meaningless
scan_split = [0]

print('---------- {} time scan ----------'.format(num_scan))
for beam in data:
    # each scan
    # if int(previous_degree) != 0 and int(beam[0]) == 0: 
    if int(previous_degree) - int(beam[0]) > 2: 
        scan_split.append(num_beam)
        num_scan += 1
        print('---------- {} time scan ----------'.format(num_scan))
    previous_degree = beam[0]

    num_beam += 1

scan_split.append(data.shape[0])
print('scan_split:', scan_split)

all_data = []
for i in range(len(scan_split)-1):
    all_data.append(data[scan_split[i] : scan_split[i+1]])


print()
print()
print()


# ## 3. traverse each scan (real time)

########## draw video
plt.ion()
plt.figure(1)


h = 710
window_size = 7
k = 3 # k-th order polynomial of SGF (should less than window_size)
gradient_window_size = 10 # number of continuous distance gradients (to find inflection point) (!!must be even!!)
gradient_threshold = 0 # decide gradient boundary (to find inflection point)
if_visual = False

for i, scan_data in enumerate(all_data):
    plt.clf()
    print('---------- {} time scan ----------'.format(i+1))
    print('all data: {}'.format(len(scan_data)))

    ##### number of error data #####
    num_error = 0
    for beam_data in scan_data:
        if beam_data[1] == 0:
            num_error += 1
    print('error data: {}'.format(num_error))
    
    if if_visual:
        ##### original scan data #####
        # plt.xlim((50, 55)) # degree
        # plt.ylim((1300, 1900)) # distance
        plt.scatter(scan_data[:, 0], scan_data[:, 1], s=1, label='dist')
        plt.xlabel('ang')
        plt.ylabel('dist (original)')
        plt.legend()
        plt.show()

    ##### non-error scan data #####
    data = scan_data[scan_data[:, 1]!=0]


    ########## denoise/smooth ##########
    # original (non-error)
    df = pd.DataFrame(data = data, columns=['ang', 'dist'])
    # SMA
    # df['SMA({})'.format(window_size)] = df['dist'].rolling(window_size).mean().round(2)
    # WMA
    # df['WMA({})'.format(window_size)] = df['dist'].rolling(window_size).apply(wma, raw=True).round(2)
    # EMA
    # df['EMA({})'.format(window_size)] = ema(df['dist'], window_size).round(2) 
    # FM
    df['FM({})'.format(window_size)] = signal.medfilt(df['dist'], window_size).round(2) 
    # SGF
    # df['SGF({})'.format(window_size)] = signal.savgol_filter(df['dist'], window_size, k, mode='nearest') 

    if if_visual:
        plt.plot(df['ang'], df['dist'], label='dist') # original (non-error)
        plt.plot(df['ang'], df['SMA({})'.format(window_size)], 'g--', label='SMA({})'.format(window_size)) # SMA
        plt.plot(df['ang'], df['WMA({})'.format(window_size)], 'r--', label='WMA({})'.format(window_size)) # WMA
        plt.plot(df['ang'], df['EMA({})'.format(window_size)], 'y--', label='EMA({})'.format(window_size)) # EMA
        plt.plot(df['ang'], df['FM({})'.format(window_size)], 'k--', label='FM({})'.format(window_size)) # FM
        plt.plot(df['ang'], df['SGF({})'.format(window_size)], 'c--', label='SGF({})'.format(window_size)) # SGF
        plt.axvline(x=66, color='r', linestyle='--')
        plt.axvline(x=71, color='r', linestyle='--')
        # plt.xlim((65, 75))
        # plt.ylim((1550, 1750))
        plt.xlabel('angle')
        plt.ylabel('dist')
        plt.legend()
        plt.show()

    ########## !!! use/not use smoothing !!! ##########
    # SMA
    # data = np.hstack((df['ang'].to_numpy()[:, np.newaxis], df['SMA({})'.format(window_size)].to_numpy()[:, np.newaxis]))
    # WMA
    # data = np.hstack((df['ang'].to_numpy()[:, np.newaxis], df['WMA({})'.format(window_size)].to_numpy()[:, np.newaxis]))
    # EMA
    # data = np.hstack((df['ang'].to_numpy()[:, np.newaxis], df['EMA({})'.format(window_size)].to_numpy()[:, np.newaxis]))
    # FM
    data = np.hstack((df['ang'].to_numpy()[:, np.newaxis], df['FM({})'.format(window_size)].to_numpy()[:, np.newaxis])) # best for skewness
    # SGF
    # data = np.hstack((df['ang'].to_numpy()[:, np.newaxis], df['SGF({})'.format(window_size)].to_numpy()[:, np.newaxis]))

    ########## find inflection point ##########
    ##### get gradient #####
    diffs = []
    for i in range(len(data)-1):
        # diffs.append([non_0_scan_data[i][0], non_0_scan_data[i+1][1] - non_0_scan_data[i][1]]) # subtraction
        # diffs.append([non_0_scan_data[i][0], non_0_scan_data[i+1][1] / non_0_scan_data[i][1]]) # division
        diffs.append([ data[i][0], (data[i+1][1] - data[i][1]) / (data[i+1][0] - data[i][0]) ]) # gradient

    diffs = np.array(diffs)

    ##### decision via mean gradient #####
    mean_i1_2_i10 = []
    mean_i10_2_i20 = []

    inflection_points = []

    for i, (ang, dist_diffs) in enumerate(diffs):
        if (i+2*gradient_window_size) > len(diffs): # prevent moving window overflow
            break

        i1_2_i10 = diffs[i : i+gradient_window_size][:, 1]
        i10_2_i20 = diffs[i+gradient_window_size : i+2*gradient_window_size][:, 1]
        
        mean_i1_2_i10.append(i1_2_i10.mean())
        mean_i10_2_i20.append(i10_2_i20.mean())
        if i1_2_i10.mean() < gradient_threshold and i10_2_i20.mean() > gradient_threshold:
            # angle, distance, distance gradient
            inflection_points.append([  data[i+gradient_window_size][0], data[i+gradient_window_size][1], diffs[i+gradient_window_size][1]  ]) ##### inflection point(s)

    if if_visual:
        plt.scatter(diffs[:, 0], diffs[:, 1], s=1, label='dist') # gradient
        mean_i10_2_i20_start = int(gradient_window_size*1.5 - 1)
        mean_i10_2_i20_end = int(-(gradient_window_size/2))
        plt.plot(diffs[:, 0][mean_i10_2_i20_start:mean_i10_2_i20_end], mean_i10_2_i20, 'g-', label='mean grad') # mean of 10-20 gradients window
        # plt.xlim((0, 90))
        plt.ylim((-150, 150)) 
        plt.xlabel('angle')
        plt.ylabel('dist diffs')
        plt.legend()
        plt.axhline(y=0, color='r', linestyle='--')
        plt.axvline(x=66, color='r', linestyle='--')
        plt.axvline(x=71, color='r', linestyle='--')
        plt.show()

    ##### only use the last inflection point #####
    if len(inflection_points) == 0:
        print('~~~ NO STEP AHEAD ~~~')
        print()
        print()
        print()
        continue

    ##### only use the last inflection point #####
    infle_ang, infle_dist, infle_dist_diff = inflection_points[-1]
    print('!!! STEP FOUND AHEAD !!!')
    print('Angle:{}, Distance:{}'.format(infle_ang, infle_dist)) ########## !!! final inflection point !!! ##########

    ########## calculate step height(h2) & step distance(d2) ##########
    ##### given #####
    d = infle_dist
    theta = infle_ang
    theta = theta * np.pi / 180

    ##### calculate h2 & d2 #####
    h1 = d * np.cos(theta)
    h2 = h - h1
    d1 = d * np.sin(theta)
    d2 = d1
    print('Step Height: {}, Step Distance: {}'.format(int(h2), int(d2)))

    if True:
        ##### original scan data #####
        # plt.xlim((50, 55)) # degree
        # plt.ylim((1300, 1900)) # distance
        if len(inflection_points) != 0:
            plt.scatter(infle_ang, infle_dist, s=50, c='r', label='step') # inflection point

        plt.scatter(scan_data[:, 0], scan_data[:, 1], s=1, label='dist')
        plt.xlabel('ang')
        plt.ylabel('dist (original)')
        plt.legend()
        plt.show()

        plt.pause(0.01) # second per frame


    print()
    print()
    print()




