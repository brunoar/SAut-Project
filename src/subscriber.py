#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D

def Position(msg):
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	explicit_quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
	theta = tf.transformations.euler_from_quaternion(explicit_quat)

	global goodOdom	
	goodOdom.x = x
	goodOdom.y = y
	goodOdom.theta = theta[2]
	pub.publish(goodOdom)
	
	odom = [x, y, theta [2]]
	print(odom)

goodOdom = Pose2D()

rospy.init_node('odometry', anonymous=True)
pub = rospy.Publisher('good_odom', Pose2D, queue_size=10)
#rate = rospy.Rate(10) # 10hz
rospy.Subscriber("robot_0/odom", Odometry, Position)
rospy.spin()
