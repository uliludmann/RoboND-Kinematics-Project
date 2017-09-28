## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---



[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[rviz_urdf_positions]: ./writeup_assets/rviz_urdf.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

We can extract the relaive positions of the joints out of the urdf file. Or by looking at the rviz TF Display.
![how to extract relative positions out of rviz][rviz_urdf_positions]


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The joints have already been labeled and so we just had to derive the right DH Parameters and fill in the table.

![image1]



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

alpha0, a0: 0 because Z0 and Z1 are coincident
d1: 0.75 - z-offset of joint2 (x1[z] is at 0.75)
alpha1: z2 points into the screen and x1 to the right hand side. so the alpha1 offset is -90degrees
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


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

##### Inverse Kinematics
Given the wrist center position, Theta1 can be derived by looking from the top to the x-z plane.

theta1 = atan2(WCy, WCx)

theta2 and theta3 are more difficult. We need our previously derived parameters.

Also if Theta1 is not 0 we need to project the parameters to the right plane by dividing WCy by the sin(theta1) and the WCx by cos(theta1)

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

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]
