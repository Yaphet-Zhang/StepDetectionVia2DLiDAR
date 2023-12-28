import matplotlib.pyplot as plt
import numpy as np

d4_left = 2000
d4_right = 1600
w = 490

def step_direction(d4_left, d4_right, w):
    theta = np.arctan( w / abs(d4_right - d4_left + 1e-8) )*180/np.pi
    
    # left LiDAR & right LiDAR
    lidar_left = [0, 0]
    lidar_right = [0+w, 0]
    plt.scatter(lidar_left[0], lidar_left[1], color='b')    
    plt.scatter(lidar_right[0], lidar_right[1], color='b')    
    plt.plot([lidar_left[0], lidar_right[0]],[lidar_left[1], lidar_right[1]], color='b') 

    # left step & right step
    step_left = [0, d4_left]
    step_right = [0+w, d4_right]
    plt.scatter(step_left[0], step_left[1], color='r')    
    plt.scatter(step_right[0], step_right[1], color='r')    
    plt.plot([step_left[0], step_right[0]],[step_left[1], step_right[1]], color='r') 

    # left vertical
    plt.plot([lidar_left[0], step_left[0]],[lidar_left[1], step_left[1]], color='b') 
    # right vertical
    plt.plot([lidar_right[0], step_right[0]],[lidar_right[1], step_right[1]], color='b')
    
    if d4_left < d4_right:
        print('right long')
        temp = [0+w, d4_left]
        plt.plot([step_left[0], temp[0]],[step_left[1], temp[1]], 'b--')
        plt.text(step_right[0] + 30, step_right[1] + 30, '{:.2f}deg'.format(theta))


    elif d4_left > d4_right:
        print('left long')
        temp = [0, d4_right]
        plt.plot([temp[0], step_right[0]],[temp[1], step_right[1]], 'b--')
        plt.text(step_left[0] + 30, step_left[1] + 30, '{:.2f}deg'.format(theta))

    else:
        print('left right equal')
        plt.text(step_left[0] + 30, step_left[1] + 30, '{:.2f}deg'.format(theta))


    plt.text(step_left[0] + 30, int(d4_left*0.4), 'L{}mm'.format(d4_left))
    plt.text(step_right[0] + 30, int(d4_right*0.6), 'R{}mm'.format(d4_right))


    # plt.xticks([]), plt.yticks([]) # hide x and y axes
    # plt.xlim(-100, 600)
    # plt.ylim(-500, 6000)
    plt.axis('equal')
    plt.show()


step_direction(d4_left, d4_right, w)
