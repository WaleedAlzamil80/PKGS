import numpy as np
import pandas as pd
import cv2 as cv

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import seaborn as sns
sns.set()

# im = cv.imread('/home/waleed/catkin_ws/src/global_planning/Brilliant_Map_80.yaml.pgm')
# imgray = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
# ret, thresh = cv.threshold(imgray, 127, 255, 0)
# contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)

df = pd.read_csv(r"/home/waleed/catkin_ws/src/global_planning/scripts/global_racetrajectory_optimization-master/inputs/tracks/berlin_2018.csv")
ref_line = np.array(df[["# x_m","y_m"]])
right_dis = np.array(df["w_tr_right_m"])
left_dis = np.array(df["w_tr_left_m"])

norm_vec = np.zeros_like(ref_line)
outer_wall = np.zeros_like(ref_line)
inner_wall = np.zeros_like(ref_line)

for i in range(ref_line.shape[1]-1):
    theta = np.arctan2((ref_line[i+1,:] - ref_line[i,:])[1], (ref_line[i+1,:] - ref_line[i,:])[0])
    norm_vec[i,0], norm_vec[i+1,1] = -np.sin(theta), np.cos(theta)
    if(ref_line[i+1,0] >= ref_line[i,0]):
        outer_wall[i,:] = ref_line[i,:] + left_dis[i]*norm_vec[i,:]
        inner_wall[i,:] = ref_line[i,:] - right_dis[i]*norm_vec[i,:]
    else:
        outer_wall[i,:] = ref_line[i,:] - left_dis[i]*norm_vec[i,:]
        inner_wall[i,:] = ref_line[i,:] + right_dis[i]*norm_vec[i,:]


outer_wall[-1,:] = ref_line[-1,:] + left_dis[i]*norm_vec[i,:]
inner_wall[-1,:] = ref_line[-1,:] - right_dis[i]*norm_vec[i,:]

plt.figure(figsize=(15,15))
plt.scatter(ref_line[:,0], ref_line[:,1], label = "Ref_Line")
plt.scatter(outer_wall[:,0], outer_wall[:,1], label = "outer_wall")
plt.scatter(inner_wall[:,0], inner_wall[:,1], label = "inner_wall")
plt.legend()
plt.show()




print(norm_vec.shape)



