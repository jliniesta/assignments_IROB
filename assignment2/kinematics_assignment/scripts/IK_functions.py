#! /usr/bin/env python3

"""
    # Javier Lopez Iniesta Diaz del Campo
    # jliddc@kth.se
"""

from math import sin, cos, acos, atan2
import numpy as np

def scara_IK(point):

    # End-effector pose
    x = point[0]
    y = point[1]
    z = point[2]
    l = [0.07, 0.3, 0.35] # Length of the links

    """
    Fill in your IK solution here and return the three joint values in q
    """

    # Forward kinematics:
    # x = l[0] + l[1]*cos(th1) + l[2]*cos(th1+th2)
    # y = l[1]*sin(th1) + l[2]*sin(th1+th2)
    # z = d3
    # Phi = th1 + th2

    # Inverse kinematrics:   
    th2 = acos(((x-l[0])**2+y**2-l[1]**2-l[2]**2) / (2*l[1]*l[2])) 

    v1 = l[1]+l[2]*cos(th2)
    v2 = l[2]*sin(th2)
    th1 = atan2(v1*y-(x-l[0])*v2,(x-l[0])*v1+v2*y)

    d3 = z

    q = [th1, th2, d3]

    print("theta1: %s", th1)
    print("theta2: %s", th2)
    print("d3: %s", d3)

    return q

def kuka_IK(point, R, joint_positions):

    q = joint_positions # It must contain 7 elements

    """
    Fill in your IK solution here and return the seven joint values in q
    """
    
    # Parameters
    tolerance = 0.2
    d_base = 0.311 # Base link lenght
    L = 0.4
    M = 0.39
    d_end_effector = 0.078 # End effector lenght

    # D-H table
    alpha_i = [np.pi/2, -np.pi/2, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, 0]
    d_i = [0, 0, L, 0, M, 0, 0]
    a_i = np.zeros(7)

    # print("Point: ", point)
    # print("R: ", R)
    print("q: ", q)

    eps_big = True 
    j = 0

    # While eps is bigger than the maximum tolerance
    while eps_big :

        eps_big = False

        j = j+1
        print("Iteration: ", j)

        temp = np.eye(4)

        # Jacobian parameters
        p = np.zeros(shape = (7, 3, ))
        z = np.zeros(shape = (7, 3, ))

        # Matrix T of the end effector
        T_end_effector =  np.array([[1, 0, 0, 0             ],
                                    [0, 1, 0, 0             ],
                                    [0, 0, 1, d_end_effector],
                                    [0, 0, 0, 1             ]])
        
        # Matrix T of the base
        T_base = np.array([[1, 0, 0, 0     ],
                           [0, 1, 0, 0     ],
                           [0, 0, 1, d_base],
                           [0, 0, 0, 1     ]])
        
        T_01 = np.array([
                [ cos(q[0]), -sin(q[0])*cos(alpha_i[0]),   sin(q[0])*sin(alpha_i[0]),        a_i[0]*cos(q[0])],
                [ sin(q[0]),  cos(q[0])*cos(alpha_i[0]),  -cos(q[0])*sin(alpha_i[0]),  a_i[0]*sin(alpha_i[0])],
                [         0,            sin(alpha_i[0]),             cos(alpha_i[0]),                  d_i[0]],
                [         0,                          0,                           0,                       1]])

        T_12 = np.array([
                [ cos(q[1]), -sin(q[1])*cos(alpha_i[1]),   sin(q[1])*sin(alpha_i[1]),        a_i[1]*cos(q[1])],
                [ sin(q[1]),  cos(q[1])*cos(alpha_i[1]),  -cos(q[1])*sin(alpha_i[1]),  a_i[1]*sin(alpha_i[1])],
                [         0,            sin(alpha_i[1]),             cos(alpha_i[1]),                  d_i[1]],
                [         0,                          0,                           0,                       1]])

        T_23 = np.array([
                [ cos(q[2]), -sin(q[2])*cos(alpha_i[2]),   sin(q[2])*sin(alpha_i[2]),        a_i[2]*cos(q[2])],
                [ sin(q[2]),  cos(q[2])*cos(alpha_i[2]),  -cos(q[2])*sin(alpha_i[2]),  a_i[2]*sin(alpha_i[2])],
                [         0,            sin(alpha_i[2]),             cos(alpha_i[2]),                  d_i[2]],
                [         0,                          0,                           0,                       1]])

        T_34 = np.array([
                [ cos(q[3]), -sin(q[3])*cos(alpha_i[3]),   sin(q[3])*sin(alpha_i[3]),        a_i[3]*cos(q[3])],
                [ sin(q[3]),  cos(q[3])*cos(alpha_i[3]),  -cos(q[3])*sin(alpha_i[3]),  a_i[3]*sin(alpha_i[3])],
                [         0,            sin(alpha_i[3]),             cos(alpha_i[3]),                  d_i[3]],
                [         0,                          0,                           0,                       1]])

        T_45 = np.array([
                [ cos(q[4]), -sin(q[4])*cos(alpha_i[4]),   sin(q[4])*sin(alpha_i[4]),        a_i[4]*cos(q[4])],
                [ sin(q[4]),  cos(q[4])*cos(alpha_i[4]),  -cos(q[4])*sin(alpha_i[4]),  a_i[4]*sin(alpha_i[4])],
                [         0,            sin(alpha_i[4]),             cos(alpha_i[4]),                  d_i[4]],
                [         0,                          0,                           0,                       1]])
             
        T_56 = np.array([
                [ cos(q[5]), -sin(q[5])*cos(alpha_i[5]),   sin(q[5])*sin(alpha_i[5]),        a_i[5]*cos(q[5])],
                [ sin(q[5]),  cos(q[5])*cos(alpha_i[5]),  -cos(q[5])*sin(alpha_i[5]),  a_i[5]*sin(alpha_i[5])],
                [         0,            sin(alpha_i[5]),             cos(alpha_i[5]),                  d_i[5]],
                [         0,                          0,                           0,                       1]])

        T_67 = np.array([
                [ cos(q[6]), -sin(q[6])*cos(alpha_i[6]),   sin(q[6])*sin(alpha_i[6]),        a_i[6]*cos(q[6])],
                [ sin(q[6]),  cos(q[6])*cos(alpha_i[6]),  -cos(q[6])*sin(alpha_i[6]),  a_i[6]*sin(alpha_i[6])],
                [         0,            sin(alpha_i[6]),             cos(alpha_i[6]),                  d_i[6]],
                [         0,                          0,                           0,                       1]])                                                

        T_02 = np.dot(T_01, T_12)
        T_03 = np.dot(T_02, T_23)
        T_04 = np.dot(T_03, T_34)
        T_05 = np.dot(T_04, T_45)
        T_06 = np.dot(T_05, T_56)
        T_07 = np.dot(T_06, T_67)

        z[0] = [0, 0, 1]
        z[1] = np.array(T_01[:3,2])
        z[2] = np.array(T_02[:3,2])
        z[3] = np.array(T_03[:3,2])
        z[4] = np.array(T_04[:3,2])
        z[5] = np.array(T_05[:3,2])
        z[6] = np.array(T_06[:3,2])

        p[0] = [0, 0, 0]
        p[1] = np.array(T_01[:3,3])
        p[2] = np.array(T_02[:3,3])
        p[3] = np.array(T_03[:3,3])
        p[4] = np.array(T_04[:3,3])
        p[5] = np.array(T_05[:3,3])
        p[6] = np.array(T_06[:3,3])

        T_0_end_effector = np.dot(T_07, T_end_effector) # T_base_end_effector = T_base_07 * T_end_effector
        T_base_end_effector = np.dot(T_base, T_0_end_effector) # T_base_07 = T_base * T_07

        R_computed = T_base_end_effector[:3, :3] # Rotation matrix (R): first 3 rows and columns of the matrix T (end effector - base)
        point_computed = T_base_end_effector[:3, 3] # [x, y, z]

        p_e = T_07[:3, 3] # [x, y, z]

        # Compute inverse jacobian
        J_inv = compute_jacobian(z, p_e, p)

        # X + eps_x = K(theta + eps_theta)
        # K(theta) + eps_x = K(theta + eps_theta) -> eps_x = K(theta + eps_theta) - K(theta)
        e_x = point_computed - point
        e_o = orientation_error(R, R_computed) # [alpha, beta, gamma]
        eps_x = np.concatenate((np.transpose(e_x), np.transpose(e_o)), axis=0) 
        # print("eps_x: ", eps_x)

        # eps_x = J(theta) * eps_theta -> eps_theta = J(theta)^-1 * eps_x
        eps_theta = np.dot(J_inv, eps_x)

        # Check if eps is smaller than tolerance
        error = np.linalg.norm(eps_x) # Module of eps_x
        print("Error: ", error)

        if error > tolerance: 
            eps_big = True

        # theta_hat = theta + eps_theta -> theta = theta_hat - eps_theta
        q = q - eps_theta

        # for i in range(len(q)):
        #     while (q[i] > np.pi/2):
        #         q[i] = q[i] - 2*np.pi/2
        #     while (q[i] < np.pi/2):
        #         q[i] = q[i] + 2*np.pi/2

        # print("q: ", q)


    print("The final q is: ", q)

    return q

def compute_jacobian(z, p_e, p):
    '''
    Compute Jacobian and its inverse
    ''' 
    J_p = np.zeros(shape = (7, 3, ))

    for i in range(len(z)):
        # J_p(i) = z(i-1) x (pe - p(i-1))
        J_p[i] = np.transpose(np.cross(np.transpose(z[i]), (p_e - np.transpose(p[i]))))
        
    J_p_total = np.transpose(J_p)
    J_o = z
    J_o_total = np.transpose(J_o)
    J = np.concatenate((J_p_total, J_o_total), axis = 0)

    J_inv = np.linalg.pinv(J) # (3.52 R-MPC)

    return J_inv

def orientation_error(R_d, R_e):
    '''
    Compute Euler angles (alpha, beta, gamma) from a rotation matrix.
    ''' 
    R_d = np.array(R_d)
    R_e = np.array(R_e)

    n_d = np.array(R_d[:3,0]) 
    s_d = np.array(R_d[:3,1]) 
    a_d = np.array(R_d[:3,2]) 

    n_e = np.array(R_e[:3,0]) 
    s_e = np.array(R_e[:3,1]) 
    a_e = np.array(R_e[:3,2]) 

    e_o = (1/2)*(np.cross(n_e,n_d)+np.cross(s_e,s_d)+np.cross(a_e,a_d)) # (3.85 R-MPC)

    return e_o  


kuka_IK([-0.123, 0., 0.42], [[1, 0, -1], [0, 1, 0], [1, 0, 0]], [0, 1.11, 1.30, 1.1, 0, 1.52, 0])