#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D
from scipy import misc
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped


def predict_measurement (Odom):
    global counter, dist_cam
    new_pose = Pose2D()
    new_pose.theta = Odom.pose.position.z
    new_pose.x = Odom.pose.position.x  + dist_cam * np.cos(new_pose.theta)
    new_pose.y = Odom.pose.position.y  + dist_cam * np.sin(new_pose.theta)

    i = 0
    range_for = np.arange(new_pose.theta + angle_min, new_pose.theta + angle_max, (angle_max - angle_min)/n_rays)

    if range_for.shape[0] == n_rays + 1:
        range_for = range_for[0:n_rays]

    dist = np.zeros(np.size(range_for))

    for theta in range_for:
        new_pose.theta = theta
        dist[i] = get_distance(new_pose)
        i = i + 1

    print(dist)

    prediction = LaserScan()
    prediction.header.seq = counter
    prediction.header.stamp = rospy.Time.now()
    prediction.header.frame_id = "camera_link"
    prediction.ranges = dist
    prediction.angle_min = angle_min
    prediction.angle_max = angle_max
    prediction.range_max = range_max
    prediction.range_min = range_min
    prediction.angle_increment = (angle_max - angle_min)/n_rays
    pub.publish(prediction)
    counter = counter + 1


def get_distance (pose):
    x = int(round((pose.x+length/2)/pixel_size))
    y = int(((-pose.y+length/2)/pixel_size))

    if x <= 600 and y <= 600:
        dist = find_obstacle(x, y, pose.theta)
        return dist
    return np.NAN   #flag: out of map


def find_obstacle(x, y, theta):
    m = -np.tan(theta)

    if theta> np.pi/4 or theta < -3*np.pi/4:
        m = -m
    limit_color = 17

    if np.abs(theta)< np.pi /4 or np.abs(theta) > 3*np.pi/ 4:
        if np.abs(theta) < np.pi/2:
            for_range=range(x, 599)
        else:
            for_range=range(x, 0, -1)
        v = y
        for u in for_range:
            w = int(round(v))
            print(w)

            if w < 600 and w > -1:
                if map[w, u] < limit_color:
                    dist = np.sqrt(np.power(w-y,2)+np.power(u-x,2))*pixel_size
                    if dist > range_max:
                        dist = dist #np.NAN
                    if dist < range_min:
                        dist = dist #np.NAN
                    return dist
            v = v + m
    else:
        m = 1/m
        if theta < 0:
            for_range=range(y, 599)
        else:
            for_range=range(y,0,-1)
        u = x
        for v in for_range:
            z = int(round(u))
            if z < 600 and z > -1:
                if map[v,z] < limit_color:
                    dist = np.sqrt(np.power(v-y,2)+np.power(z-x,2))*pixel_size
                    if dist > range_max:
                        dist = dist #np.NAN
                    if dist < range_min:
                        dist = dist #np.NAN
                    return dist
            u = u + m
    return np.NaN                #flag : no obstacle


map = misc.imread('/home/bruno/catkin_ws/src/project/src/images/new_map.PNG', 'L')

counter = 0

dist_cam = 0.13 #distance between camera and odom
n_rays = 20 * 4
angle_max = 0.507887542248
angle_min = -0.496869921684
range_max = 10
range_min = 0.44999

pixel_size = 0.05
length = 30


rospy.init_node('predict measurement', anonymous=True)
pub = rospy.Publisher('predict_measurement', LaserScan, queue_size=10)
rospy.Subscriber("predict_localization", PoseStamped, predict_measurement)
rospy.spin()
