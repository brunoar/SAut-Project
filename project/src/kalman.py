#!/usr/bin/env python

import numpy as np
import rospy 
import math
from geometry_msgs.msg import Pose2D
from numpy.core.multiarray import ndarray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from scipy import misc
from skimage.draw import line
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
import copy


def prediction(Odom):
    global counter, ref_rob, ref_global, ref_global_cov, new_ref_global, new_ref_global_cov, update_done, qx_static, qy_static, qz_static, qx_din, qy_din, qz_din, frob_prediction
    global rviz_path
    goodOdom= Pose2D()
    goodOdom.x = Odom.pose.position.x
    goodOdom.y = Odom.pose.position.y
    goodOdom.theta = Odom.pose.position.z

    print("n_prediction", counter)
    print("n_update", seq_update)

    if update_done:
        if goodOdom != ref_rob:
            update_done = False
            counter = counter + 1
            if counter == 1:
                # update robot referential variables
                ref_rob.x = goodOdom.x
                ref_rob.y = goodOdom.y
                ref_rob.theta = goodOdom.theta
            else:
                draw()

                rviz_path.header.frame_id = "map"
                rviz_path.header.stamp = rospy.Time.now()
                rviz_path.header.seq = counter
                pose_rviz = PoseStamped()
                pose_rviz.header=rviz_path.header
                pose_rviz.pose.position.x = copy.deepcopy(ref_global.x)
                pose_rviz.pose.position.y = copy.deepcopy(ref_global.y)
                pose_rviz.pose.position.z = 0
                [pose_rviz.pose.orientation.x, pose_rviz.pose.orientation.y, pose_rviz.pose.orientation.z, pose_rviz.pose.orientation.w] = quaternion_from_euler(0, 0, copy.deepcopy(ref_global.theta))
                rviz_path.poses.append(pose_rviz)
                pub_rviz.publish(rviz_path)

                d = math.sqrt(math.pow(goodOdom.x-ref_rob.x, 2) + math.pow(goodOdom.y-ref_rob.y, 2))
                d_theta = goodOdom.theta - ref_rob.theta

                # prediction

                aux_d_x = d
                aux_d_y = d

                if goodOdom.x-ref_rob.x < 0 and (goodOdom.theta > -np.pi/2) and (goodOdom.theta < np.pi/2):
                    aux_d_x = -d

                if goodOdom.x-ref_rob.x > 0 and (np.abs(goodOdom.theta) > np.pi/2 ):
                    aux_d_x = -d
                #print('aux_d_x= ', aux_d_x)
                new_ref_global.x = ref_global.x + aux_d_x * math.cos(ref_global.theta)

                if goodOdom.y-ref_rob.y < 0 and (goodOdom.theta > 0):
                    aux_d_y = -d

                if goodOdom.y-ref_rob.y > 0 and (goodOdom.theta <0 ):
                    aux_d_y = -d
                #print('aux_d_y= ', aux_d_y)
                new_ref_global.y = ref_global.y + aux_d_y * math.sin(ref_global.theta)

                new_ref_global.theta = ref_global.theta + d_theta
                # Matrix of linearization
                F = np.array([[1, 0, -aux_d_x*math.sin(ref_global.theta)],
                              [0, 1, aux_d_y*math.cos(ref_global.theta)],
                              [0, 0, 1]])

                delta_x_abs = np.abs(aux_d_x*math.sin(ref_global.theta))
                delta_y_abs = np.abs(aux_d_y*math.cos(ref_global.theta))
                delta_theta_abs = np.abs(d_theta)

                qx = qx_static + qx_din*delta_x_abs
                qy = qy_static + qy_din*delta_y_abs
                qz = qz_static + qz_din*delta_theta_abs

                Q = np.array([[qx, 0, 0],
                              [0, qy, 0],
                              [0, 0, qz]])

                # covariance of prediction gaussian distribution
                new_ref_global_cov = F.dot(ref_global_cov.dot(np.transpose(F))) + Q
                eigen_prediction = np.linalg.eig(new_ref_global_cov)
                print('eigen prediction', eigen_prediction)
                frob_prediction = np.append(frob_prediction, np.linalg.norm(new_ref_global_cov, 'fro'))
                np.save('frob_prediction.npy', frob_prediction)

                # update robot referential variables
                ref_rob.x = goodOdom.x
                ref_rob.y = goodOdom.y
                ref_rob.theta = goodOdom.theta
                # update global referential variables
                ref_global.x = new_ref_global.x
                ref_global.y = new_ref_global.y
                ref_global.theta = new_ref_global.theta
                ref_global_cov = new_ref_global_cov

            odom = [ref_global.x, ref_global.y, ref_global.theta]
            #print("odom= ",odom)
            # publish to node that calculates predicted measurement of kinect
            Odom.header.seq = counter
            Odom.header.stamp = rospy.Time.now()
            Odom.header.frame_id = "map"
            Odom.pose.position.x = ref_global.x
            Odom.pose.position.y = ref_global.y
            Odom.pose.position.z = ref_global.theta
            pub_prediction.publish(Odom)



def update(error, gamma=1000):
    global ref_global_cov, ref_global, K, S, H, R, update_done, seq_update

    obs_error = Pose2D()
    obs_error.x = error.pose.position.x
    obs_error.y = error.pose.position.y
    obs_error.theta = error.pose.position.z
    flag_conv = error.pose.orientation.x
    if not flag_conv:
        update_done = True
        return
    else:
        S = H.dot(ref_global_cov.dot(np.transpose(H))) + R

        innovation = np.array([[obs_error.x], [obs_error.y], [obs_error.theta]])
        if np.transpose(innovation).dot(np.linalg.inv(S).dot(innovation)) <= gamma:
            K = ref_global_cov.dot((np.transpose(H)).dot(np.linalg.inv(S)))
            ref_global.x = ref_global.x + K[0].dot(innovation)
            ref_global.y = ref_global.y + K[1].dot(innovation)
            ref_global.theta = ref_global.theta + K[2].dot(innovation)
            ref_global_cov = ref_global_cov - K.dot(S.dot(np.transpose(K)))
            eigen_update = np.linalg.eig(ref_global_cov)
            print('eigen update', eigen_update)
            frob_prediction = np.linalg.norm(new_ref_global_cov, 'fro')
            print('frob norm update', frob_prediction)
            #print("S")
            #print(S)
            #print("K")
            #print(K)
            #print("ref_global")
            #print(ref_global)
            #print("covariancia_update ", ref_global_cov)
            upd = [ref_global.x, ref_global.y, ref_global.theta]
            #print("update ", upd)
            error.header.seq = seq_update
            error.header.stamp = rospy.Time.now()
            error.header.frame_id = "map"
            error.pose.position.x = ref_global.x
            error.pose.position.y = ref_global.y
            error.pose.position.z = ref_global.theta
            pub_update.publish(error)
            seq_update = seq_update + 1
    update_done = True


def draw():
    x = int(round((ref_global.x+length/2)/pixel_size))
    y = int(((-ref_global.y+length/2)/pixel_size))

    global counter2, last_x2, last_y2 , trajectory
    if counter2 == 0:
        last_x2 = x
        last_y2 = y
    elif x < 600 and y < 600:
        rr, cc = line(last_y2, last_x2, y, x)
        trajectory[rr,cc] = 89
        last_x2 = x
        last_y2 = y
        if counter2 == 20:
            counter2 = 1
            misc.imsave('/home/bruno/catkin_ws/src/project/src/images/experiments/new.png', trajectory)
    counter2 = counter2 + 1


pixel_size = 0.05
length = 30
trajectory = misc.imread('/home/bruno/catkin_ws/src/project/src/images/new_map.PNG', 'L')
counter2 = 0
last_x2 = 0
last_y2 = 0


seq_update = 0
# global variables---------------------------------------------
counter = 0
# previous robot position in its referential
ref_rob = Pose2D()
ref_rob.x = 0
ref_rob.y = 0
ref_rob.theta = 0
# previous robot position in global referential
# good initial estimations are needed for convergence
ref_global = Pose2D()
'''
ref_global.x = goodOdom.x
ref_global.y = -6.918715728752538
ref_global.theta = 0.7853981633974484
'''
'''
# a comecar no outro canto
ref_global = Pose2D()
ref_global.x = 9.3
ref_global.y = 3.3
ref_global.theta = -2.3
'''
'''
#goodguy
ref_global.x = -10.20595
ref_global.y = 4.3905263
ref_global.theta = -0.76460134
'''
ref_global.x = -6.18
ref_global.y = 1.33
#ref_global.theta = -0.89
ref_global.theta = -0.76460134
'''
#rostage
ref_global.x=-2.228
ref_global.y=-5.262
ref_global.theta-2.597786648323038
'''
'''
# a comecar perto do centro da zona do elevador virado para os elevadores
ref_global.x = -0.1
ref_global.y = -0.1
ref_global.theta = 0.735
'''
ref_global_cov = np.array([[100, 0, 0],
                           [0, 100, 0],
                           [0, 0, 100]])

# predicted robot position in global referential
new_ref_global = Pose2D()
new_ref_global.x = 0
new_ref_global.y = 0
new_ref_global.theta = 0
new_ref_global_cov = np.array([[0, 0, 0],
                               [0, 0, 0],
                               [0, 0, 0]])
# error characterization
qx_static = 0.0005
qy_static = 0.0005
qz_static = 0.0005

qx_din = 0.001
qy_din = 0.001
qz_din = 0.0011
rx = 0.008
ry = 0.008
rz = 0.01

H = np.array([[1, 0, 0],
              [0, 1, 0],
              [0, 0, 1]])


R = np.array([[rx, 0, 0],
              [0, ry, 0],
              [0, 0, rz]])  # type: ndarray

frob_prediction = np.array([])


def change_time(msg):
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "camera_link"
    pub_laser.publish(msg)

rviz_path = Path()
# main code ----------------------------------------------------
# get odometry data and call the prediction function
update_done = True
rospy.init_node('kalman_filter', anonymous=True)
pub_prediction = rospy.Publisher('predict_localization', PoseStamped, queue_size=10)
pub_update = rospy.Publisher('update_localization', PoseStamped, queue_size=10)
pub_laser = rospy.Publisher('/scan_new_stamp', LaserScan, queue_size=10)
pub_rviz = rospy.Publisher('rviz_traj', Path, queue_size=10)

rospy.Subscriber("good_odom", PoseStamped, prediction)
rospy.Subscriber("/scan", LaserScan, change_time)
rospy.Subscriber("obs_error", PoseStamped, update)

rospy.spin()
