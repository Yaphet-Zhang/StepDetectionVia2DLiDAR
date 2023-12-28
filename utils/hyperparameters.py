# common
h = 955 # 675 # lidar height to ground
# w = 490 # 2 lidar baseline
min_theta = 29 # min dis: 300 (24 degree)
max_theta = 83 # max dis: 6000 (83 degree)
if_visual = False # viz detail
if_visual_step_and_ditch = False # just viz results
if_visual_main = False # if viz

# descend
diffence_threshold = 40


# ascend
window_size = 5
k = 3 # k-th order polynomial of SGF (should less than window_size)
gradient_window_size = 5 # number of continuous distance gradients (to find inflection point) (!!must be even!!)
gradient_threshold = 0 # decide gradient boundary (to find inflection point)


