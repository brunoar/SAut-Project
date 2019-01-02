#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def Position(msg):
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	explicit_quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
	theta = tf.transformations.euler_from_quaternion(explicit_quat)

	global seq
	goodOdom = PoseStamped()
	goodOdom.header.seq = seq
	goodOdom.header.stamp = rospy.Time.now()
	goodOdom.header.frame_id = "map"
	goodOdom.pose.position.x = x
	goodOdom.pose.position.y = y
	goodOdom.pose.position.z = theta[2]
	pub.publish(goodOdom)
	
	odom = [x, y, theta [2]]
	print(odom)
	seq = seq + 1


seq = 0
rospy.init_node('odometry', anonymous=True)
pub = rospy.Publisher('good_odom', PoseStamped, queue_size=10)
#rospy.Subscriber("/RosAria/pose", Odometry, Position)
rospy.Subscriber("/robot_0/odom", Odometry, Position)
rospy.spin()
