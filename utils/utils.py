import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal
import pandas as pd
from scipy.ndimage import gaussian_filter1d

from hyperparameters import h, min_theta, max_theta, if_visual, if_visual_step_and_ditch, if_visual_main
from hyperparameters import diffence_threshold
from hyperparameters import window_size, k, gradient_window_size, gradient_threshold



def smooth_real_time_vibration(theta, flag):
    ##### show original d2
    plt.plot(theta, 'k', label='original')

    # ##### SMA 
    # df = pd.DataFrame(data = theta, columns=['angle'])
    # df['SMA({})'.format(window_size)] = df['angle'].rolling(window_size).mean().round(2)
    # theta_sma = df['SMA({})'.format(window_size)]
    # plt.plot(theta_sma, 'g--', label='SMA({})'.format(window_size))

    ##### Gaussian 
    theta_gaussian = gaussian_filter1d(input=theta, sigma=3)
    plt.plot(theta_gaussian, 'b--', label='Gaussian(sigma=3)')
    plt.legend()
    plt.grid()

    if flag == 'ditch':
        '''
        calculate h2, d2
        '''
        pass
    elif flag == 'step':
        '''
        calculate h4, d4
        '''
        pass




def wma(y):
    weights = np.arange(len(y)) + 1
    wma = np.sum(weights * y) / weights.sum()

    return wma




def ema(y, n):
    ema = np.zeros(len(y))
    ema[:] = np.nan # initial via NAN
    ema[n-1] = y[:n].mean() # only calculate the head part via SMA
    for i in range(n, len(y)):
        ema[i] = ema[i-1] + (y[i] - ema[i-1]) / (n + 1) * 2

    return ema




def data_preprocessing(scan_data):
    ##### 1. 0-360-0  -->  0-360 
    scan_data = np.array(scan_data[:-1])

    ##### 2. only 31 ~ 83 degree 
    ang_range = scan_data[:, 0].astype(int)
    scan_data = scan_data[(ang_range >= min_theta) == (ang_range <= max_theta)]
    print('all data: {}'.format(len(scan_data)))

    ## visulize original scan data 
    if if_visual:
        # plt.xlim((50, 55)) # degree
        # plt.ylim((1300, 1900)) # distance
        plt.scatter(scan_data[:, 0], scan_data[:, 1], s=1, label='dist')
        plt.xlabel('ang')
        plt.ylabel('dist (original)')
        plt.legend()
        plt.show()

    ## print number of error data 
    num_error = 0
    for beam_data in scan_data:
        if beam_data[1] == 0:
            num_error += 1
    print('error data: {}'.format(num_error))

    ##### 3. delet error/dist:0 scan data 
    data = scan_data[scan_data[:, 1]!=0]


    return scan_data, data




def ditch_detector(scan_data, data):
    less_dist = 80
    less_deg = 5.5

    break_points = []
    for i in range(len(data)-1):
        ##### 1. calculate step height (h4) to find break point(s) 
        # given
        # d_after = data[i+1][1] # correct/original
        # d_before = data[i][1] # correct/original
        d_after = data[i+1][1] + less_dist # because RPLiDAR S2 measured 3cm less
        d_before = data[i][1] + less_dist
        # theta_after = data[i+1][0] * np.pi / 180 # correct/original
        # theta_before = data[i][0] * np.pi / 180 # correct/original
        theta_after = (data[i+1][0] + less_deg )* np.pi / 180 # because RPLiDAR S2's position is 2 degree less
        theta_before = (data[i][0] + less_deg )* np.pi / 180
        # calculate h4 & d3 
        h3_after = d_after * np.cos(theta_after)
        h3_before = d_before * np.cos(theta_before)

        h4 = h3_after - h3_before # subtraction
        if h4 > 0 and h4 > diffence_threshold:
            # degree, distance, distance different (h4)
            break_points.append([ data[i][0], data[i][1], h4])
    break_points = np.array(break_points)

    ## visulize difference value 
    if if_visual:
        plt.scatter(break_points[:, 0], break_points[:, 2], s=1, label='dist diff') # differences
        plt.axhline(y=diffence_threshold, color='r', linestyle='--')
        plt.xlabel('angle')
        plt.ylabel('dist diffs')
        plt.legend()
        plt.show()

    if len(break_points) == 0:
        print('~~~ NO DITCH AHEAD ~~~')
        plt.text(28, 4000, '溝無し', fontname='MS Gothic') # Japanese
    else:
        ##### 2. only use the first break point 
        bre_ang, bre_dist, h4 = break_points[0]
        print('!!! DITCH FOUND AHEAD !!!')
        print('Angle: {}, Distance: {}'.format(bre_ang, bre_dist)) ########## !!! final break point !!! ##########
        # theta = bre_ang # correct/original
        theta_degree = bre_ang + less_deg # because RPLiDAR S2's position is 3 degree less
        # d = bre_dist # correct/original
        d = bre_dist + less_dist # because RPLiDAR S2 measured 3cm less
        theta = theta_degree * np.pi / 180
        d3 = d * np.sin(theta)
        d4 = d3
        print('Ditch Height: {}, Ditch Distance: {}'.format(int(-h4), int(d4)))

        ##### 3. visulize ditch 
        if if_visual_step_and_ditch:
            # original scan data
            # plt.scatter(scan_data[:, 0], scan_data[:, 1], s=1, label='dist')
            # break point
            # plt.scatter(bre_ang, bre_dist, s=50, c='y', label='ditch') # English
            plt.scatter(bre_ang, bre_dist, s=50, c='y', label='溝発見！') # Japanese
            # plt.xlabel('ang')
            # plt.ylabel('dist (original)')
            # plt.xlim((0, 90))
            # plt.ylim((0, 5000))
            # plt.legend()
            # plt.show()

            # plt.text(bre_ang + 2, bre_dist + 20, 'H:{}/D:{}'.format(int(-h4), int(d4))) # English
            plt.text(bre_ang + 2, bre_dist + 20, '高さ:{}cm / 距離:{}cm'.format(int(-h4/10), int(d4/10)), fontname='MS Gothic') # Japanese


    if len(break_points) != 0:
        return theta_degree, int(h4), int(d4)
    else:
        return None, None, None




def step_detector(scan_data, data):
    less_dist = 0
    less_deg = 5.5

    ##### 1. denoise/smooth 
    # original (non-error)
    df = pd.DataFrame(data = data, columns=['ang', 'dist'])
    # SMA
    # df['SMA({})'.format(window_size)] = df['dist'].rolling(window_size).mean().round(2)
    # WMA
    # df['WMA({})'.format(window_size)] = df['dist'].rolling(window_size).apply(wma, raw=True).round(2)
    # EMA
    # df['EMA({})'.format(window_size)] = ema(df['dist'], window_size).round(2) 
    # MF
    df['MF({})'.format(window_size)] = signal.medfilt(df['dist'], window_size).round(2) 
    # SGF
    # df['SGF({})'.format(window_size)] = signal.savgol_filter(df['dist'], window_size, k, mode='nearest') 

    ## visulize smoothed value 
    if if_visual:
        plt.plot(df['ang'], df['dist'], label='dist') # original (non-error)
        plt.plot(df['ang'], df['SMA({})'.format(window_size)], 'g--', label='SMA({})'.format(window_size)) # SMA
        plt.plot(df['ang'], df['WMA({})'.format(window_size)], 'r--', label='WMA({})'.format(window_size)) # WMA
        plt.plot(df['ang'], df['EMA({})'.format(window_size)], 'y--', label='EMA({})'.format(window_size)) # EMA
        plt.plot(df['ang'], df['MF({})'.format(window_size)], 'k--', label='MF({})'.format(window_size)) # MF
        plt.plot(df['ang'], df['SGF({})'.format(window_size)], 'c--', label='SGF({})'.format(window_size)) # SGF
        # plt.axvline(x=56, color='r', linestyle='--')
        # plt.axvline(x=59.3, color='r', linestyle='--')
        plt.xlim((50, 70))
        plt.ylim((1050, 1150))
        plt.xlabel('angle')
        plt.ylabel('dist (smooth)')
        plt.legend()
        plt.show()

    ##### !!! use / not use smoothing !!! 
    # SMA
    # data = np.hstack((df['ang'].to_numpy()[:, np.newaxis], df['SMA({})'.format(window_size)].to_numpy()[:, np.newaxis]))
    # WMA
    # data = np.hstack((df['ang'].to_numpy()[:, np.newaxis], df['WMA({})'.format(window_size)].to_numpy()[:, np.newaxis]))
    # EMA
    # data = np.hstack((df['ang'].to_numpy()[:, np.newaxis], df['EMA({})'.format(window_size)].to_numpy()[:, np.newaxis]))
    # MF
    data = np.hstack((df['ang'].to_numpy()[:, np.newaxis], df['MF({})'.format(window_size)].to_numpy()[:, np.newaxis])) # best for skewness
    # SGF
    # data = np.hstack((df['ang'].to_numpy()[:, np.newaxis], df['SGF({})'.format(window_size)].to_numpy()[:, np.newaxis]))

    ##### 2. find inflection point 
    ## get gradient
    diffs = []
    for i in range(len(data)-1):
        dist_after = data[i+1][1]
        dist_before = data[i][1]
        ang_after = data[i+1][0]
        ang_before = data[i][0]

        if ang_after - ang_before != 0: # for divided-by-0-error
            diffs.append([ data[i][0], (dist_after - dist_before) / (ang_after - ang_before) ]) # gradient
        else:
            diffs.append([ data[i][0], (dist_after - dist_before) / 0.24 ]) # gradient

    diffs = np.array(diffs)

    ## decision via mean gradient
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
            
            # index = int(i + gradient_window_size) ##### !!! use 10th (window size th) beam !!!
            index = int(i + gradient_window_size*0.5)
            
            # angle, distance, distance gradient
            inflection_points.append([  data[index][0], data[index][1], diffs[index][1]  ]) ##### !!! inflection point(s) !!! 

    ## visulize gradient value 
    if if_visual:
        plt.scatter(diffs[:, 0], diffs[:, 1], s=1, label='dist') # gradient
        mean_i10_2_i20_start = int(gradient_window_size*1.5 - 1)
        mean_i10_2_i20_end = int(-(gradient_window_size/2))
        # plt.axvline(x=56, color='r', linestyle='--')
        # plt.axvline(x=59.3, color='r', linestyle='--')
        plt.plot(diffs[:, 0][mean_i10_2_i20_start:mean_i10_2_i20_end], mean_i10_2_i20, 'g-', label='mean grad') # mean of 10-20 gradients window
        # plt.xlim((0, 90))
        # plt.ylim((-150, 150)) 
        plt.xlabel('angle')
        plt.ylabel('dist grad')
        plt.legend()
        plt.axhline(y=0, color='r', linestyle='--')
        plt.show()

    if len(inflection_points) == 0:
        print('~~~ NO STEP AHEAD ~~~')
        plt.text(28, 3000, '階段無し', fontname='MS Gothic') # Japanese
        if if_visual_step_and_ditch:
            # plt.scatter(scan_data[:, 0], scan_data[:, 1], c='b', s=1, label='dist') # English
            plt.scatter(scan_data[:, 0], scan_data[:, 1], c='b', s=1, label='すべてのスキャン点') # Japanese
            

    else:
        if len(inflection_points) == 1:
            infle_ang, infle_dist, infle_dist_diff = inflection_points[0]
            if if_visual_step_and_ditch:
                # plt.scatter(scan_data[:, 0], scan_data[:, 1], c='b', s=1, label='dist') # English
                plt.scatter(scan_data[:, 0], scan_data[:, 1], c='b', s=1, label='すべてのスキャン点') # Japanese

        else:
            ## !!! use the nearest (to sensor) inflection point(s) & select only the last one of them !!! 
            # infle_ang, infle_dist, infle_dist_diff = inflection_points[-1] # only consider one
            for i in range(len(inflection_points)-1):
                if inflection_points[i+1][0] - inflection_points[i][0] >= 1: # 1 degree
                    infle_ang = inflection_points[i][0]
                    infle_dist = inflection_points[i][1]
                    break
                else:
                    infle_ang = inflection_points[i+1][0]
                    infle_dist = inflection_points[i+1][1]                

        print('!!! STEP FOUND AHEAD !!!')
        print('Angle: {}, Distance: {}'.format(infle_ang, infle_dist)) ## !!! final inflection point !!! 

        ##### 3. calculate step height (h2) & step distance (d2)
        # given
        # d = infle_dist # correct/original
        d = infle_dist + less_dist # because RPLiDAR S2 measured 3cm less
        # theta = infle_ang # correct/original
        theta_deg = infle_ang + less_deg # because RPLiDAR S2's position is 2 degree less
        theta = theta_deg * np.pi / 180
        # calculate h2 & d2
        h1 = d * np.cos(theta)
        h2 = h - h1
        d1 = d * np.sin(theta)
        d2 = d1
        print('Step Height: {}, Step Distance: {}'.format(int(h2), int(d2)))

        ##### 4. visulize step
        if if_visual_step_and_ditch:
            ## original scan data
            # plt.xlim((50, 70)) # degree
            # plt.ylim((600, 1200)) # distance
            if len(inflection_points) != 0:
                # inflection point
                # plt.scatter(infle_ang, infle_dist, s=50, c='r', label='step') # English
                plt.scatter(infle_ang, infle_dist, s=50, c='r', label='階段発見！') # Japanese
            
            
            # plt.scatter(scan_data[:, 0], scan_data[:, 1], c='b', s=1, label='dist') # English
            plt.scatter(scan_data[:, 0], scan_data[:, 1], c='b', s=1, label='すべてのスキャン点') # Japanese

            # English
            # plt.xlabel('ang')
            # plt.ylabel('dist (original)')
            plt.legend()

            # Japanese
            plt.xlabel('角度[度]', fontname='MS Gothic')
            plt.ylabel('距離[mm] (センサーから障害まで)', fontname='MS Gothic')
            plt.legend(prop={'family':'Yu Gothic'})
            # plt.show()

            # plt.text(infle_ang + 2, infle_dist + 20, 'H:{}/D:{}'.format(int(h2), int(d2))) # English
            plt.text(infle_ang + 2, infle_dist + 20, '高さ:{}cm / 距離:{}cm'.format(int(h2/10), int(d2/10)), fontname='MS Gothic') # Japanese

    if len(inflection_points) != 0:
        return theta_deg, int(h2), int(d2)
    else:
        return None, None, None




def ditch_direction(d4_left, d4_right, w):

    theta = np.arctan(w/(abs(d4_left - d4_right)))*180/np.pi




def step_direction_and_all_plot(d2_left, d2_right, w, step_height_left):

    theta = np.arctan( w / abs(d2_right - d2_left + 1e-8) )*180/np.pi
    
    print('Final Step Dist (L): {}, Final Step Dist (R): {}'.format(d2_left, d2_right))
    # only use left
    print('Final Step Height: {}, Final Step Direction: {}'.format(int(step_height_left), '{:.2f}deg'.format(theta)))

    # left LiDAR & right LiDAR
    lidar_left = [0, 0]
    lidar_right = [0+w, 0]
    # if if_visual_main:
    #     plt.scatter(lidar_left[0], lidar_left[1], color='b')    
    #     plt.scatter(lidar_right[0], lidar_right[1], color='b')    
    #     plt.plot([lidar_left[0], lidar_right[0]],[lidar_left[1], lidar_right[1]], color='b') 

    # left step & right step
    step_left = [0, d2_left]
    step_right = [0+w, d2_right]
    if if_visual_main:
        plt.scatter(step_left[0], step_left[1], color='r')    
        plt.scatter(step_right[0], step_right[1], color='r')    
        plt.plot([step_left[0], step_right[0]],[step_left[1], step_right[1]], color='r') 

        # # left vertical
        # plt.plot([lidar_left[0], step_left[0]],[lidar_left[1], step_left[1]], color='b') 
        # # right vertical
        # plt.plot([lidar_right[0], step_right[0]],[lidar_right[1], step_right[1]], color='b')
    
    if d2_left < d2_right:
        # print('right long')
        temp = [0+w, d2_left]
        if if_visual_main:
            # plt.plot([step_left[0], temp[0]],[step_left[1], temp[1]], 'b--')
            plt.text(step_right[0] + 30, step_right[1] + 30, '{:.2f}deg'.format(theta))
    elif d2_left > d2_right:
        # print('left long')
        temp = [0, d2_right]
        if if_visual_main:
            # plt.plot([temp[0], step_right[0]],[temp[1], step_right[1]], 'b--')
            plt.text(step_left[0] + 30, step_left[1] + 30, '{:.2f}deg'.format(theta))
    else:
        # print('left right equal')
        if if_visual_main:
            plt.text(step_left[0] + 30, step_left[1] + 30, '{:.2f}deg'.format(theta))


    if if_visual_main:
        # plt.text(step_left[0] + 30, int(d2_left*0.4), 'L{}mm'.format(d2_left))
        # plt.text(step_right[0] + 30, int(d2_right*0.6), 'R{}mm'.format(d2_right))

        # plt.xticks([]), plt.yticks([]) # hide x and y axes
        # plt.xlim(-100, 600)
        # plt.ylim(-500, 6000)
        plt.axis('equal')
        # plt.show()




