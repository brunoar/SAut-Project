#!/usr/bin/env python

import rospy
from scipy import misc
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import matplotlib.pyplot as plt
import numpy as np


def predict_measurement (msg):
    msg=msg


map = misc.imread('/home/bruno/catkin_ws/src/project/src/images/map001.png', 'L')

 print (type (map))
 im_min = map.max()
 im_max = map.min()
 print ("shape: ", map.shape)
 print ("min: ",im_min)
 print ("max: ", im_max)
 print(np.unique(map))

# pixel = np.array(map.shape[0]*map.shape[1])
# for i, pixel[i] in np.ndenumerate(map):
#    counter=0
#    for j in range(0, i):
#        if pixel[i] == pixel[j]:
#            counter = counter + 1
#            continue
#    if counter == 1:
#        counter = 0
#        continue
#    print(pixel[i])

# rows, cols = np.where(map == 89)
# for i in rows:
   # map[i,cols[i]] =0
# print(rows, cols)

plt.imshow(map)
plt.show()

pub = rospy.Publisher('predict_measurement', Float32, queue_size=10)
rospy.Subscriber("predict_localization", Pose2D, predict_measurement)
rospy.spin()
