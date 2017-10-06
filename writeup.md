## Project: Kinematics Pick & Place
Writeup for the second project on the Udacity Robotics Nanodegree Term 1



[rviz_urdf_positions]: ./writeup_assets/rviz_urdf.png
[dhparam_skizze]: ./writeup_assets/dhparam_skizze.png
[ikq1]: ./writeup_assets/ikq1.png
[ikq2q3]: ./writeup_assets/ikq2q3.png
[gif]: ./writeup_assets/robond_project2.gif
[img1]: ./writeup_assets/s1.png
[img2]: ./writeup_assets/s5.png
[img3]: ./writeup_assets/s11.png
[img4]: ./writeup_assets/s16.png
[img5final]: ./writeup_assets/s17.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

![gif]

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

We can extract the relative positions of the joints out of the urdf file. Or by looking at the rviz TF Display.
![how to extract relative positions out of rviz][rviz_urdf_positions]


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The joints have already been labeled and so we just had to derive the right DH Parameters and fill in the table.
![dhparam_skizze]

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  - pi / 2 | -0.054 | 1.5 | q4
4->5 | pi / 2 | 0 | 0 | q5
5->6 | -pi / 2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0


All joints are revolute. So the joint angle is variable (q1:q6)

### Explanation how i found the DH-Parameters
alpha0, a0: 0... because Z0 and Z1 are coincident  
d1: 0.75... z-offset of joint2 (x1[z] is at 0.75)  
alpha1 = z2 points into the screen and x1 to the right   hand side. so the alpha1 offset is -90degrees  
a1: we need to extract the x offset of joint2 from the urdf file. it is 0.35.  
d2: 0  
theta2: there is a constant offset between x1 and x2 of -90degrees plus a variable offset of the joint variable theta2  
alpha2: z2 and z3 are parallel  
a2: relative z length of link3  
d3: 0 because x2 and x3 are coincident  
q3: q3.  
alpha3: -90 degrees  
a3: there is a slight offset of the revolutes in the relative z positions. -0.054  
d4: z4 origin is in revolute 5 so its the relative x distance between revolute 3 and 5. 0.96+0.54  
alpha4: 90degrees  
a4, d5: 0  
alpha5: -90degrees  
a5, d6: 0  
alpha6: 0  
a6: 0  
d7: xg to x4 measured along zG. relative z positions from link6 + the gripper link 0.193+0.11 distance between wrist and end-effector  

## indiviual transformation matrices
I set up a function, so I dont need to write the transformation matrix for every transformation.

```python
def dh_transformation_step(alpha, a, d, q):
    """
    returns a DH Transformation matrix.
    """
    transformation_matrix = Matrix([
        [cos(q), -sin(q), 0, a],
        [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
        [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
        [0, 0, 0, 1]
    ])
    return transformation_matrix
```
According to this function, my transformation matrices are written down below. I created them by inserting the DH Paramters into the transformation matrix given in the lessons.

```python
T_01 = Matrix([
[cos(q1), -sin(q1), 0,    0],
[sin(q1),  cos(q1), 0,    0],
[      0,        0, 1, 0.75],
[      0,        0, 0,    1]]),

T_12 = Matrix([
[sin(q2),  cos(q2), 0, 0.35],
[      0,        0, 1,    0],
[cos(q2), -sin(q2), 0,    0],
[      0,        0, 0,    1]]),

T_23 = Matrix([
[cos(q3), -sin(q3), 0, 1.25],
[sin(q3),  cos(q3), 0,    0],
[      0,        0, 1,    0],
[      0,        0, 0,    1]]),

T_34 = Matrix([
[ cos(q4), -sin(q4), 0, -0.054],
[       0,        0, 1,    1.5],
[-sin(q4), -cos(q4), 0,      0],
[       0,        0, 0,      1]]),

T_45 = Matrix([
[cos(q5), -sin(q5),  0, 0],
[      0,        0, -1, 0],
[sin(q5),  cos(q5),  0, 0],
[      0,        0,  0, 1]]),

T_56 = Matrix([
[ cos(q6), -sin(q6), 0, 0],
[       0,        0, 1, 0],
[-sin(q6), -cos(q6), 0, 0],
[       0,        0, 0, 1]]),

T_6G = Matrix([
[1, 0, 0,     0],
[0, 1, 0,     0],
[0, 0, 1, 0.303],
[0, 0, 0,     1]]))


```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

##### Inverse Kinematics
Given the wrist center position, Theta1 can be derived by looking from the top to the x-z plane.

![ikq1]

`theta1 = atan2(WCy, WCx)`

theta2 and theta3 are more difficult. We need our previously derived parameters.

Also if Theta1 is not 0 we need to project the parameters to the right plane by dividing WCy by the sin(theta1) and the WCx by cos(theta1)

![ikq2q3]

```python
def get_thetas123(x, y, z):
    #theta 1
    q1 = atan2(y, x)
    # project to x-z layer:
    y = y / sin(q1)
    x = x / cos(q1)
    # joint offset substraction
    x -= 0.35
    z -= 0.75
    # static lengths
    l_a = 1.50 #length link 4(0.96) + length_link5
    l_c = 1.25
    l_b = sqrt(x ** 2 + z ** 2)
    beta = acos((l_a**2 + l_c**2 - l_b**2) / (2 * l_a * l_c))
    q3 = 90 * dtr - (beta + 0.036)
    h = atan2(z, x)
    alpha = acos((l_c**2 + l_b**2 - l_a**2) / (2 * l_c * l_b))
    q2 = 90 *dtr - h - alpha
    return q1.evalf(), q2.evalf(), q3.evalf()
```
##### Inverse Orientation

Derive Rotation Matrix between Wrist and end-effector R_36

Given the position of the end-effector, we can derive the position of the wrist center. Were nx, ny, nz is the rotation of the End Effector and l_ee is the distance between the wrist center and the end-effector
```python
nx, ny, nz = R_EE[0:4, 2][0:3]
l_ee = 0.303

wx = px -  l_ee * nx
wy = py - l_ee * ny
wz = pz - l_ee * nz

```
We can then insert the positions of the wrist center in the calculation of q1, q2, q3.

`theta1, theta2, theta3 = KR210.get_thetas123(wx, wy, wz)`

After we know q1, q2 and q3 it is possible to derive the postition and orientation of the wrist center.

We transpose the Rotation matrix of R0_3 and multiply it with the Rotation Matrix of the end-effector. Out of this rotation matrix we can get the euler angles with the given formulas.

```python
R0_3 = extract_rotation_matrix(KR210.R0_3.subs({q1: theta1, q2: theta2, q3: theta3}))
R3_6 = R0_3.T * R_EE
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2]+ R3_6[2, 2]*R3_6[2, 2]), R3_6[1, 2])
theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
```
It took me a long time, to realize that inversing the matrix with the 'LU' decomposition does not lead to a good solution. Transposing the matrix helped.

### Project Implementation
First I set up the symbols for sympy.
Next I store the DH Parameters into a dict.
Then I define helper functions for DH Transformation and the elementary rotation matrices.  
Then I define a class for a robot. It has methods for calculating the forward kinematics and q1, q2, q3.  

I define a function to optimize the angles because the solutions that are calculated for the euler angles (q4, q5, q6) are not always perfect. Thats because there are multiple angle solutions that lead to one end-effector position. So I implemented a optimize_angle function. It helps the robot to perform a smoother movement.


```python
def optimize_angle(angle, freedeg):
    angle = degrees(angle)
    print("angle before optimization:" + str(angle))
    angle = angle.evalf()
    if angle > 0:
        angle = angle % freedeg
        print("angle after optimization:" + str(angle))
        return radians(angle)
    else:
        angle = abs(angle)
        angle = angle % freedeg
        print("angle after optimization:" + str(angle))
        angle = radians(angle)
        return - angle
```

Before we are ready to handle the IK requests, I instanciate the Robot and the forward kinematics is calculated. `KR210 = Robot(dh_params)`  
We can then access the forward kinematics `T_0G = KR210.T_0G`

I calculate the wrist center:
```python
nx, ny, nz = R_EE[0:4, 2][0:3]
l_ee = 0.303

wx = px -  l_ee * nx
wy = py - l_ee * ny
wz = pz - l_ee * nz
```

And after we know the wrist center, we can pass the coordinates as parameters into the get_thetas123-function.
`theta1, theta2, theta3 = KR210.get_thetas123(wx, wy, wz)`

q4, q5, q6 are calculated with the help of the transpose of R0_3 and the Rotation of the end-effector. (Inverse with "Lu" decomposition does not work for me!)

```python
R0_3 = extract_rotation_matrix(KR210.R0_3.subs({q1: theta1, q2: theta2, q3: theta3}))
R3_6 = R0_3.T * R_EE #inv("LU") does not lead to a good solution.
### get euler anngles from roationmatrix
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2]+ R3_6[2, 2]*R3_6[2, 2]), R3_6[1, 2])
theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
```

Those angles then get passed into the optimize_angle function, where I define theta4 to be in the range between -180..180 degrees and q5 and q6 between -360 and 360 degrees.
```python
#optimize angles for a smoother movement.
theta4 = optimize_angle(theta4, 180)
theta5 = optimize_angle(theta5, 180)
theta6 = optimize_angle(theta6, 360)
```

Overall there are many possibilities to improve the movement of the robot and the processing time by improving the algorithms and changing from symbolic computation to numeric computation.  
One could improve the boot time of the service if we moved the FK-Part into a database or a pickle file.

Below I provide some screenshots on how the robot moves.
![img1]
![img2]
![img3]
![img4]
![img5final]
