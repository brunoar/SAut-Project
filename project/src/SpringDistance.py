#!/usr/bin/env python

import numpy as np


def u_type_1 ():
    u = 0
    range_i = np.arange(0,N-2)
    for i in range_i:
        range_j = np.arange(i+1, N-1)
        for j in range_j:
            v_ij_r = np.sqrt(np.power(real_nodes[i,0]-real_nodes[j,0],2)+np.power(real_nodes[i,1]-real_nodes[j,1],2))
            v_ij_p = np.sqrt(np.power(predicted_nodes[i,0]-predicted_nodes[j,0],2)+np.power(predicted_nodes[i,1]-predicted_nodes[j,1],2))
            aux = k_rel * np.power(v_ij_r- v_ij_p,2) / (2*v_ij_r)
            u = u + aux
    u = 2*u /(N*(N-1))
    return u


def u_type_2 ():
    u = 0
    range_i = np.arange(0,N-1)
    for i in range_i:
        aux = k_shift * (np.power(real_nodes[i,0]-predicted_nodes[i,0],2)+np.power(real_nodes[i,1]-predicted_nodes[i,1],2))
        u = u + aux/2
    u = u/N
    return u


def u_type_3 ():
    u = 0
    range_i = np.arange(0,N-2)
    for i in range_i:
        range_j = np.arange(i+1, N-1)
        for j in range_j:
            v_ij_r = np.sqrt(np.power(real_nodes[i,0]-real_nodes[j,0],2)+np.power(real_nodes[i,1]-real_nodes[j,1],2))
            v_ij_p = np.sqrt(np.power(predicted_nodes[i,0]-predicted_nodes[j,0],2)+np.power(predicted_nodes[i,1]-predicted_nodes[j,1],2))
            prod = np.dot(real_nodes[j,:]-real_nodes[i,:], predicted_nodes[j,:]-predicted_nodes[i,:])
            if prod/(v_ij_p*v_ij_r) > 1:
                theta = 0
            elif prod/(v_ij_p*v_ij_r) < -1:
                theta = np.pi
            else:
                theta = np.arccos(prod/(v_ij_p*v_ij_r))
            u = u + k_rot * np.power(theta,2)/2
    u = 2*u / (N*(N-1))
    return u

k_rel = 1
k_shift = 0
k_rot = 1
N = 12
#precise_trajectory_2
'''
#gamma 1000  n80
predicted_nodes = np.array([ [-9.18, 4.33], [-4.18, -1.46], [-1.16, -5.33],  [1.63, -2.3], [-0.18, -0.47], [-1.71, 1.05],
                           [-1.22, 1.79], [0.17, -0.22], [2.34, -2.20], [5.63, 0.70], [8.6, 3.08], [8.87, 3.73] ])

#gamma 10 n80
predicted_nodes = np.array([ [-9.2, 4.24], [-4.14, -1.44], [-1.14, -5.25],  [1.12, -2.36], [-0.47, -0.89], [-1.16, 1.02],
                           [-0.23, 2.26], [1.67, 0.32], [3.31, -1.33], [5.61, 0.65], [9.06, 3.12], [10.45, 4.63] ])

#gamma 1000 N80 bad                           
predicted_nodes = np.array([ [-7.41, 1.65], [-4.13, -1.45], [-1.19, -5.39],  [1.66, -2.44], [-0.46, -0.1013], [-1.7, 1.106],
                           [-1.19, 1.77], [-0.18, -0.15], [2.32, -2.16], [5.62, 0.67], [8.67, 3.1], [8.78, 3.72] ])

#gamma 10 N80 bad
predicted_nodes = np.array([ [-7.31, 1.56], [-4.2, 1.33], [0.41, -7.06],  [2.82, -5], [1.42, -2], [-0.97, -0.92],
                           [-0.56, -0.28], [1.58, -1.42], [4.16, -3.5], [6.04, 0.98], [9.08, 2.75], [9.13, 3.72] ])

#gamma 1000 N80 bad orientation
predicted_nodes = np.array([ [-9.1, 4.95], [-4.2, -1.44], [-1.37, -5.3],  [1.54, -2.57], [-0.46, 0.37], [-1.67, 1.02],
                           [-1.3, 1.76], [0.17, -0.23], [2.37, -2.18], [5.6, 0.52], [8.77, 3.21], [8.94, 3.78] ])


#gamma 10 N20
predicted_nodes = np.array([ [-9.78, 3.97], [-4.18, -1.44], [-1.3, -5.4],  [1.6, -2.65], [-0.43, -0.29], [-1.69, 1.1],
                           [-1.27, 1.78], [0.19, -0.19], [2.18, -2.39], [5.62, 0.7], [8.5, 2.95], [8.73, 4.12] ])
'''
#limit
predicted_nodes = np.array([ [-6.18, 1.33], [-4.15, -1.4], [-1.13, -5.21],  [1.6, -2.68], [-0.30, -0.58], [-1.59, 1.02],
                           [-1.24, 1.79], [0.17, -0.15], [2.46, -2.13], [5.63, 0.65], [8.51, 3.09], [8.85, 3.78] ])

real_nodes = np.array([ [-9.18, 4.33], [-4.18, -1.63], [-1.03, -5.05],  [1.55, -2.66], [-0.43, -0.47], [-1.58, 1.10],
                           [-1.21, 1.6], [0.17, 0.34], [2.2, -2.15], [5.61, 0.83], [8.5, 3.03], [8.64, 4.12] ])

# Overall potential energy stored in Type-1, Type-2 and Type-3 springs
u_spring = u_type_1() + u_type_2() + u_type_3()
print(u_spring)
