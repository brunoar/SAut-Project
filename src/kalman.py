#!/usr/bin/env python

import numpy as np
import rospy 
import math
from geometry_msgs.msg import Pose2D

def prediction(goodOdom):
	global counter, ref_rob, ref_global, ref_global_cov, new_ref_global, new_ref_global_cov, pub
	counter = counter + 1
	if counter == 1:
		#update robot referential variables
		ref_rob.x = goodOdom.x
		ref_rob.y = goodOdom.y
		ref_rob.theta = goodOdom.theta
	else:
		#auxiliary variables
		d = math.sqrt(math.pow(goodOdom.x-ref_rob.x,2) + math.pow(goodOdom.y-ref_rob.y,2))
		d_theta = goodOdom.theta - ref_rob.theta
		#prediction
		new_ref_global.x = ref_global.x + d * math.cos(ref_global.theta)
		new_ref_global.y = ref_global.y + d * math.sin(ref_global.theta)
		new_ref_global.theta = ref_global.theta + d_theta
		#Matrix of linearization
		F= np.array( [[1, 0, -d*math.sin(ref_global.theta)],
			         [0, 1, d*math.cos(ref_global.theta)],
			         [0, 0, 1]] )
		#covariance of prediction gaussian distribution
		new_ref_global_cov = F.dot(ref_global_cov.dot(F.transpose())) + Q
		#update robot referential variables
		ref_rob.x = goodOdom.x
		ref_rob.y = goodOdom.y
		ref_rob.theta = goodOdom.theta
		#update global referential variables
		ref_global.x = new_ref_global.x
		ref_global.y = new_ref_global.y
		ref_global.theta = new_ref_global.theta
		ref_global_cov = new_ref_global_cov
		
	odom = [ref_global.x, ref_global.y, ref_global.theta]
	print(odom)
	print(ref_global_cov)
    #publish to node that calculates predicted measurement of kinect
	pub.publish(ref_global)

#global variables---------------------------------------------
counter = 0
#previous robot position in its referential
ref_rob = Pose2D()
ref_rob.x = 0
ref_rob.y = 0
ref_rob.theta = 0
#previous robot position in global referential
#good initial estimations are needed for convergence
ref_global = Pose2D()
ref_global.x = -2.2632214107095705
ref_global.y = -5.280959225396302
ref_global.theta = -2.6477866483230375
ref_global_cov= np.array( [[1, 0, 0],
			   			   [0, 1, 0],
			  			   [0, 0, 1]] )
#predicted robot position in global referential
new_ref_global = Pose2D()
new_ref_global.x = 0
new_ref_global.y = 0
new_ref_global.theta = 0
new_ref_global_cov = np.array( [[0, 0, 0],
	     		        		[0, 0, 0],
			        			[0, 0, 0]] )
#error characterization
Q = np.array( [[0, 0, 0],
	      	   [0, 0, 0],
	           [0, 0, 0]] )
#main code----------------------------------------------------
#get odometry data and call the prediction function

rospy.init_node('kalman_filter', anonymous=True)
pub = rospy.Publisher('predict_localization', Pose2D, queue_size=10)  # type: object
#rate = rospy.Rate(10) # 10hz
rospy.Subscriber("good_odom", Pose2D, prediction)
rospy.spin()
