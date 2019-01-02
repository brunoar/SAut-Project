#!/usr/bin/env python
import roslib
import rospy
import tf
from geometry_msgs.msg import PoseStamped
import numpy as np
from sensor_msgs.msg import Image


def broadcaster(msg, cam_dist = 0.134):

    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.position.x, msg.pose.position.y, 0), tf.transformations.quaternion_from_euler(0, 0, msg.pose.position.z), rospy.Time.now(), "base_link", "map")
    br.sendTransform((msg.pose.position.x + cam_dist * np.cos(msg.pose.position.z), msg.pose.position.y + cam_dist * np.sin(msg.pose.position.z), 0), tf.transformations.quaternion_from_euler(0, 0, msg.pose.position.z), rospy.Time.now(), "camera_link", "map")


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    rospy.Subscriber('predict_localization',PoseStamped,broadcaster )
    rospy.spin()


def change_time(msg):
    msg.header.stamp = rospy.Time.now()
    pub_camera.publish(msg)

rospy.init_node('correct_stamp')
pub_camera = rospy.Publisher("image_new_stamp", Image, queue_size = 10)
rospy.Subscriber("/camera/depth/image_rect_raw", Image, change_time)
rospy.spin()
