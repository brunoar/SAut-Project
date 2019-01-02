#!/usr/bin/env python

import numpy as np
from sklearn.neighbors import NearestNeighbors
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
import message_filters
import rospy
from geometry_msgs.msg import PoseStamped
import warnings


warnings.filterwarnings("ignore")


def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
    Input:
      A: Nxm numpy array of corresponding points
      B: Nxm numpy array of corresponding points
    Returns:
      T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
      R: mxm rotation matrix
      t: mx1 translation vector
    '''

    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
       Vt[m-1,:] *= -1
       R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R,centroid_A.T)

    # homogeneous transformation
    T = np.identity(m+1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t


def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nxm array of points
        dst: Nxm array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''

    assert src.shape == dst.shape

    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)
    distances, indices = neigh.kneighbors(src, return_distance=True)
    return distances.ravel(), indices.ravel()


def icp(A, B, init_pose=None, max_iterations=20, tolerance=0.001):
    '''
    The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
    Input:
        A: Nxm numpy array of source mD points
        B: Nxm numpy array of destination mD point
        init_pose: (m+1)x(m+1) homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation that maps A on to B
        distances: Euclidean distances (errors) of the nearest neighbor
        i: number of iterations to converge
    '''

    assert A.shape == B.shape

    # get number of dimensions
    m = A.shape[1]

    # make points homogeneous, copy them to maintain the originals
    src = np.ones((m+1,A.shape[0]))
    dst = np.ones((m+1,B.shape[0]))
    src[:m,:] = np.copy(A.T)
    dst[:m,:] = np.copy(B.T)

    # apply the initial pose estimation
    if init_pose is not None:
        src = np.dot(init_pose, src)

    prev_error = 0

    converge = False
    for i in range(max_iterations):
        # find the nearest neighbors between the current source and destination points
        distances, indices = nearest_neighbor(src[:m,:].T, dst[:m,:].T)

        # compute the transformation between the current source and nearest destination points
        T,_,_ = best_fit_transform(src[:m,:].T, dst[:m,indices].T)

        # update the current source
        src = np.dot(T, src)

        # check error
        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:
            converge = True
            break
        prev_error = mean_error

    # calculate final transformation
    T,_,_ = best_fit_transform(A, src[:m,:].T)

    return T, distances, converge


def get_z_minus_h(h,z, Odom, n_rays=20* 4):
    pose = Pose2D()
    pose.x = Odom.pose.position.x
    pose.y = Odom.pose.position.y
    pose.theta = Odom.pose.position.z

    ranges = np.array(z.ranges)

    new_z = ranges[np.round(np.linspace(0, len(ranges)-1, n_rays)).astype(int)]
    theta = np.linspace(z.angle_min + pose.theta,z.angle_max + pose.theta, len(ranges))
    new_theta = theta[np.round(np.linspace(0, len(ranges)-1, n_rays)).astype(int)]
    new_z = new_z.reshape(-1,1)
    new_theta = new_theta.reshape(-1,1)

    z_points = np.concatenate((new_z*np.cos(new_theta), new_z*np.sin(new_theta)), axis=1)
    # z_points = np.array([new_z*np.cos(new_theta), new_z*np.sin(new_theta)])

    new_h=np.array(h.ranges)
    new_h = new_h.reshape(-1,1)
    h_points = np.concatenate((new_h*np.cos(new_theta), new_h*np.sin(new_theta)), axis=1)
    # h_points = np.array([new_h*np.cos(new_theta), new_h*np.sin(new_theta)])

    indices = np.argwhere(np.isnan(z_points))
    z_points = np.delete(z_points, indices[:, 0], axis=0)
    h_points = np.delete(h_points, indices[:, 0], axis=0)

    # TESTE
    indices = np.argwhere(np.isnan(h_points))
    z_points = np.delete(z_points, indices[:, 0], axis=0)
    h_points = np.delete(h_points, indices[:, 0], axis=0)

    '''
    indices = np.argwhere(np.isnan(h_points))
    h_points = np.delete(h_points, indices)
    z_points = np.delete(z_points, indices)
    '''
    delta = PoseStamped()
    if z_points.shape[0] < 9:        #prof disse para mudar isto
        delta.pose.orientation.x = 0 #flag of not wnough points
        delta.pose.position.x = 1    #random numbers
        delta.pose.position.y = 2
        delta.pose.position.z = 3
    else:
        T, d, flag = icp(z_points, h_points)
        delta_x = T[0,2]
        delta_y = T[1,2]
        sen = T[1,0]
        cos = T[1,1]
        delta_theta = np.arctan(sen/cos)

        if cos < 0 and sen < 0:
            delta_theta = delta_theta - np.pi
        if cos < 0 and sen > 0:
            delta_theta = delta_theta + np.pi

        delta.pose.position.x = delta_x
        delta.pose.position.y = delta_y
        delta.pose.position.z = delta_theta

        if flag:                                #converge=True
            delta.pose.orientation.x = 1
        else:
            delta.pose.orientation.x = 0

    pub.publish(delta)

rospy.init_node('icp', anonymous=True)
pub = rospy.Publisher('obs_error', PoseStamped, queue_size=10)
prediction = message_filters.Subscriber('/predict_measurement', LaserScan)
measurement = message_filters.Subscriber('/scan_new_stamp', LaserScan)
localization = message_filters.Subscriber('predict_localization', PoseStamped)
ts = message_filters.ApproximateTimeSynchronizer([prediction, measurement, localization], queue_size=10, slop=2)
ts.registerCallback(get_z_minus_h)
rospy.spin()

