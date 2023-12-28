from pyrplidar import PyRPlidar
import time
import matplotlib.pyplot as plt
from utils.utils import ditch_detector, step_detector, data_preprocessing, smooth_real_time_vibration
from utils.hyperparameters import if_visual_main, window_size

import numpy as np
import pandas as pd


########## connect to 2D LiDAR ##########
lidar = PyRPlidar()
lidar.connect(port='COM7', baudrate=1000000, timeout=3) # S2
# lidar.connect(port='COM6', baudrate=256000, timeout=3) # A3
print('====================')


########## start to scan ##########
lidar.set_motor_pwm(500)
time.sleep(2)
scan_generator = lidar.start_scan() # angular resolution: 0.24, fps: 10
# scan_generator = lidar.start_scan_express(0) # 0.12 , fps: 10, distance unit ???
# scan_generator = lidar.force_scan() # error


try:
    count_scan = 0
    scan_data = []
    previous_degree = 0 # meaningless
    start_time = time.time()

    if if_visual_main: 
        ## draw video
        plt.ion()
        plt.figure('right')



    theta_ditchs = []
    theta_steps = []
    ########## start to scan from several noisy scan & 70+ degree ##########
    for count_beam, scan in enumerate(scan_generator()):

        ########## if angle != 0, start a new scan ##########
        if not (int(previous_degree) == 359 and int(scan.angle) == 0):
            if count_scan >= 2: ##### use scan data to detect from 2nd scan
                scan_data.append([scan.angle, scan.distance])

        ########## if angle == 0, that means 1 time is done ##########
        elif int(previous_degree) == 359 and int(scan.angle) == 0:
            print('========== {} time scan =========='.format(count_scan))
            if count_scan >= 2: ##### use scan data to detect from 2nd scan 

                if if_visual_main: 
                    ## clear figure
                    plt.clf()

                ##### Data preprocessing #####
                scan_data, data = data_preprocessing(scan_data)


                ##### Descend / Ditch #####
                print()
                print('---------- Descend ----------')
                theta_ditch, h4, d4 = ditch_detector(scan_data, data)


                # ##### smooth ditch angle during real-tim vibration #####
                # theta_ditchs.append(theta_ditch)
                # if len(theta_ditchs) == 5:
                #     smooth_real_time_vibration(np.array(theta_ditch), 'ditch')
                #     plt.pause(0.1)
                #     # reinitial
                #     theta_ditchs = []


                ##### Ascend / Step #####
                print()
                print('---------- Ascend ----------')
                theta_step, h2, d2 = step_detector(scan_data, data)


                # ##### smooth step angle during real-tim vibration #####
                # if theta_step != None:
                #     theta_steps.append(theta_step)
                #     if len(theta_steps) == 10:
                #         smooth_real_time_vibration(np.array(theta_steps), 'step')
                #         plt.show()
                #         plt.pause(0.1)
                #         # reinitial
                #         theta_steps = []
                # print(theta_steps)


                ##### End a scan #####
                print()
                print('---------- FPS ----------')
                end_time = time.time()
                print('{:.2f}'.format(1/(end_time - start_time)))
                # Japanese
                plt.text(28, 300, '実行速度:{}FPS'.format(int(1/(end_time - start_time))), fontname='MS Gothic')
                print()
                print()
                print()


                if if_visual_main: 
                    plt.show()
                    plt.pause(0.01) # second per frame


                ##### Re-initial a new scan #####
                scan_data = []
                scan_data.append([scan.angle, scan.distance])
                start_time = time.time()

            count_scan += 1

        previous_degree = scan.angle


except (Exception, KeyboardInterrupt): # if error / press ctrl + c, skip
    print('!!! ERROR / STOP !!!')
    pass


########## disconnect to 2D LiDAR ##########
print('========== End ==========')
lidar.stop()
lidar.set_motor_pwm(0)
lidar.disconnect()


