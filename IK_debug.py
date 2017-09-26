from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}





def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!
    #### degrees to radians
    dtr = pi/180

    #### create symbols
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')




    #### dh transformation function:

    def dh_transformation_step(alpha_subi_1, a_subi_1, dsubi, thetai):
        """
        returns a DH Transformation matrix.
        """
        transformation_matrix = Matrix([
            [cos(thetai), -sin(thetai), 0, a_subi_1],
            [sin(thetai) * cos(alpha_subi_1), cos(thetai) * cos(alpha_subi_1), -sin(alpha_subi_1), -sin(alpha_subi_1) * dsubi],
            [sin(thetai) * sin(alpha_subi_1), cos(thetai) * sin(alpha_subi_1), cos(alpha_subi_1), cos(alpha_subi_1) * dsubi],
            [0, 0, 0, 1]
        ])
        return transformation_matrix


    def extract_rotation_matrix(rotation_matrix):
        return rotation_matrix[0:3, 0:3]


    ### define rotation functions
    def rot_x(q):
        r_x = Matrix([[1, 0, 0, 0],
                 [0, cos(q), -sin(q), 0],
                 [0, sin(q), cos(q), 0],
                     [0, 0, 0, 1]])
        return r_x
      
    def rot_y(q):
        r_y = Matrix([[cos(q), 0, sin(q), 0],
                [0, 1, 0, 0],
                 [-sin(q), 0, cos(q), 0],
                     [0, 0, 0, 1]])
        return r_y
      
    def rot_z(q):
        r_z = Matrix([[cos(q), -sin(q), 0, 0],
                 [sin(q), cos(q), 0, 0],
                 [0,0,1, 0],
                [0, 0, 0, 1]])
        return r_z

    def get_thetas(x, y, z):
        #theta 1
        q1 = atan2(y, x)
        # project to x-z layer:
        y = y / sin(q1)
        x = x / cos(q1)
        # joint offset substraction
        x -= 0.35
        z -= 0.75
        # static lengths 
        l_a = 1.50 #length link 4(0.96) + length_link5 (0.54)
        l_c = 1.25
        l_b = sqrt(x ** 2 + z ** 2)
        beta = acos((l_a**2 + l_c**2 - l_b**2) / (2 * l_a * l_c))
        q3 = 90 * dtr - beta
        h = atan2(z, x)
        alpha = acos((l_c**2 + l_b**2 - l_a**2) / (2 * l_c * l_b))
        q2 = 90 *dtr - h - alpha
        return q1.evalf(), q2.evalf(), q3.evalf()

    dh_params = {
        alpha0: 0, a0: 0, d1: 0.75, q1: q1,
        alpha1: -90 * dtr, a1: 0.35, d2: 0, q2: q2 - 90 * dtr,
        alpha2: 0, a2: 1.25, d3: 0, q3: q3,
        alpha3: -90 * dtr, a3: -0.054, d4:  1.5, q4: q4,
        alpha4: 90 * dtr, a4: 0, d5: 0, q5: q5,
        alpha5: -90 * dtr, a5: 0, d6: 0.193, q6: q6,
        alpha6: 0, a6: 0, d7: 0.303, q7: 0 #d7 = middle of gripper.
            }
    
    T_01 = dh_transformation_step(alpha0, a0, d1, q1).subs(dh_params)
    T_12 = dh_transformation_step(alpha1, a1, d2, q2).subs(dh_params)
    T_23 = dh_transformation_step(alpha2, a2, d3, q3).subs(dh_params)
    T_34 = dh_transformation_step(alpha3, a3, d4, q4).subs(dh_params)
    T_45 = dh_transformation_step(alpha4, a4, d5, q5).subs(dh_params)
    T_56 = dh_transformation_step(alpha5, a5, d6, q6).subs(dh_params)
    T_6G = dh_transformation_step(alpha6, a6, d7, q7).subs(dh_params)
    

    R_corr = rot_z(180 * dtr) * rot_y(-90 * dtr)
    

    T_0G = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * T_6G * R_corr
    #
    #
    # Extract rotation matrices from the transformation matrices
    rot_matrices = []
    for matrix in [T_01, T_12, T_23, T_34, T_45, T_56, T_6G]:
        rot_matrices.append(extract_rotation_matrix(matrix))


    R0_1, R1_2, R2_3, R3_4, R4_5, R5_6, R6_G = rot_matrices
    

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

    R_EE = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    
    nx, ny, nz = R_EE[0:4, 2][0:3]
    #l_ee = 0.303 -> (dh_params[d7] + l_ee) # should be equal to 0.303 - where does this come from???
    l_ee = 0.303

    wx = px -  l_ee * nx
    wy = py - l_ee * ny
    wz = pz - l_ee * nz


    theta1, theta2, theta3 = get_thetas(wx, wy, wz)

    ###inverse orientation problem
    R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6

    R0_3 = (R0_1 * R1_2 * R2_3).subs({q1: theta1, q2: theta2, q3: theta3})
    R3_6 = R0_3.inv('LU') * extract_rotation_matrix(R_EE)

    # my euler angles solution
    def get_euler_angles(matrix):
        r11 = matrix[0, 0]
        r12 = matrix[0, 1]
        r13 = matrix[0, 2]
        r21 = matrix[1, 0]
        r22 = matrix[1, 1]
        r23 = matrix[1, 2]
        r31 = matrix[2, 0]
        r32 = matrix[2, 1]
        r33 = matrix[2, 2]
        #Rz - alpha:
        alpha = atan2(r32, r33)
        #Ry - beta
        beta =  atan2(-r31, sqrt(r11**2 + r23**2))
        #Rx - gamma:
        gamma = atan2(r21, r11)
        return alpha, beta, gamma

    theta4, theta5, theta6 = get_euler_angles(R3_6)


    ### Euler angles implementation from solution

    theta4 = atan2(R3_6[2,2], R3_6[0,2])
    theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2]+ R3_6[2, 2]*R3_6[2, 2]), R3_6[1, 2])
    theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    your_ee = T_0G.subs({q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [wx, wy, wz] # <--- Load your calculated WC values in this array
    #your_ee = [1,1,1] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 3

    test_code(test_cases[test_case_number])
