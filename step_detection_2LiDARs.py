import time

import matplotlib.pyplot as plt
from utils.hyperparameters import if_visual_main, w
from pyrplidar import PyRPlidar
from utils.utils import (data_preprocessing, ditch_detector, ditch_direction,
                  step_detector, step_direction_and_all_plot)


########## connect to 2D LiDAR ##########
lidar_left = PyRPlidar()
lidar_right = PyRPlidar()
lidar_left.connect(port='COM6', baudrate=1000000, timeout=3) # S2 (old / left)
lidar_right.connect(port='COM5', baudrate=1000000, timeout=3) # S2 (new / right)
# lidar.connect(port='COM6', baudrate=256000, timeout=3) # A3
print('====================')


########## start to scan ##########
lidar_left.set_motor_pwm(500)
# time.sleep(2)
lidar_right.set_motor_pwm(500)
time.sleep(2)
scan_generator_left = lidar_left.start_scan() # angular resolution: 0.24, fps: 10
scan_generator_right = lidar_right.start_scan() # angular resolution: 0.24, fps: 10
# scan_generator = lidar.start_scan_express(0) # 0.12 , fps: 10, distance unit ???
# scan_generator = lidar.force_scan() # error




try:
    count_scan_left = 0
    count_scan_right = 0
    scan_data_left = []
    scan_data_right = []
    previous_degree_left = 0 # meaningless
    previous_degree_right = 0 # meaningless
    start_time_left = time.time()
    start_time_right = time.time()

    if if_visual_main: 
        ## draw video
        plt.ion()
        plt.figure('main')


    ########## left: start to scan from several noisy scan & 70+ degree ##########
    ########## right: start to scan from several noisy scan & 60+ degree ##########
    for count_beam, (scan_left, scan_right) in enumerate(zip(scan_generator_left(), scan_generator_right())):
        # # just for check
        # print('left', scan_left.angle, scan_left.distance)
        # print('right', scan_right.angle, scan_right.distance)
        # time.sleep(0.01) # !!!!!!!!!!

        ########## if angle != 0, start a new scan ##########
        ## left
        if not (int(previous_degree_left) == 359 and int(scan_left.angle) == 0): 
            if count_scan_left >= 2: ##### use scan data to detect from 2nd scan
                scan_data_left.append([scan_left.angle, scan_left.distance])

        ## right
        if not (int(previous_degree_right) == 359 and int(scan_right.angle) == 0):
            if count_scan_right >= 2: ##### use scan data to detect from 2nd scan
                scan_data_right.append([scan_right.angle, scan_right.distance])


        ########## if angle == 0, that means 1 time is done ##########
        ## left
        if (int(previous_degree_left) == 359 and int(scan_left.angle) == 0):
            print('========== {} time scan (L) =========='.format(count_scan_left))
            if count_scan_left >= 2: ##### use scan data to detect from 2nd scan 

                ##### Data preprocessing #####
                scan_data_left, data_left = data_preprocessing(scan_data_left)
                

                ##### Descend / Ditch #####
                print()
                print('---------- Descend (L) ----------')
                theta_ditch_left, h4_left, d4_left = ditch_detector(scan_data_left, data_left)


                ##### Ascend / Step #####
                print()
                print('---------- Ascend (L) ----------')
                theta_step_left, h2_left, d2_left = step_detector(scan_data_left, data_left)


                ##### End a scan #####
                print()
                print('---------- FPS (L) ----------')
                end_time_left = time.time()
                print('{:.2f}'.format(1/(end_time_left - start_time_left)))
                print()


                ##### Re-initial a new scan #####
                scan_data_left = []
                scan_data_left.append([scan_left.angle, scan_left.distance])
                start_time_left = time.time()

            count_scan_left += 1
        previous_degree_left = scan_left.angle


        ## right
        if (int(previous_degree_right) == 359 and int(scan_right.angle) == 0):
            print('========== {} time scan (R) =========='.format(count_scan_right))
            if count_scan_right >= 2: ##### use scan data to detect from 2nd scan 
                    
                ##### Data preprocessing #####
                scan_data_right, data_right = data_preprocessing(scan_data_right)
                

                ##### Descend / Ditch #####
                print()
                print('---------- Descend (R) ----------')
                theta_ditch_right, h4_right, d4_right = ditch_detector(scan_data_right, data_right)


                ##### Ascend / Step #####
                print()
                print('---------- Ascend (R) ----------')
                theta_step_right, h2_right, d2_right = step_detector(scan_data_right, data_right)


                ##### End a scan #####
                print()
                print('---------- FPS (R) ----------')
                end_time_right = time.time()
                print('{:.2f}'.format(1/(end_time_right - start_time_right)))


                ##### Direction (use left & right) #####
                if d2_left != None and d2_right != None:
                    print()
                    print('---------- Step Direction ----------')
                    step_direction_and_all_plot(d2_left, d2_right, w, h2_left)
                if if_visual_main:
                    plt.pause(0.001) ## second per frame
                    plt.clf() ## clear figure
                print()
                print()
                print()
                print()


                ##### Re-initial a new scan #####
                scan_data_right = []
                scan_data_right.append([scan_right.angle, scan_right.distance])
                start_time_right = time.time()

            count_scan_right += 1
        previous_degree_right = scan_right.angle




except (Exception, KeyboardInterrupt): # if error / press ctrl + c, skip
    print('!!! ERROR / STOP !!!')
    pass




########## disconnect to 2D LiDAR ##########
print('========== End ==========')
lidar_left.stop()
lidar_right.stop()
lidar_left.set_motor_pwm(0)
lidar_right.set_motor_pwm(0)
lidar_left.disconnect()
lidar_right.disconnect()


