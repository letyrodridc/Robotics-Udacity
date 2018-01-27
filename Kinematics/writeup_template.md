## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

To build the DH parameters table I used the file kr210.urdf.xacro located under the directory: `/home/robond/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/urd`. That files contains the robot specification.
The links and joints sizes, types, connections, etc.

For example, let take a look to the `joint_1` specification in the xacro file:

```
  <joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.33" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link_1"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-185*deg}" upper="${185*deg}" effort="300" velocity="${123*deg}"/>
  </joint>

```

In origin, we have the location of the joint. Notice that the xyz value is (0,0,0.33). That means it will move 0.33 from the 0 position in the z direction. It also has the limits specified in degrees. They can be easily converted to radians -3.23 to +3.23 radians range of movement. It also, indicates the type, it a revolute. The parameter axis indicates its orientation. It goes vertical in z axis direction.

Using this information, I could build the base model of my robot indicating links and joints. 

Then, I will follow the DH parameters steps to add more information to my diagram. 


1) Label all the joints from 1 to n
2) Label al lthe links from 0 to n
3) Draw the lines defining all joint axis
	Goes in the same direction that the axis defined in the xacro file
4) Define the common normals between joint axis (minimizing the non-zero DH parameters)
	In robotics, common normal of two non-intersecting joint angles is a line perpedicular to both axis.
5) Assigning z-axis of frame i to a pont aloing the ith join axis. 
	Goes in the same direction that the axis defined in the xacro file
6) Define the positive x-axis for intermediate lengths
7) Assign x-axis of length 0 
7) Assign x-axis link n xn and xn-1 same direction (Add the gripper frame)

![robot](misc_images/fk.png)
 
For building the DH parameters tables, I would need to calculate for each link:

twisted angles(αi-1): angle between zi and zi-1 counter-clockwise
distance along(ai-1): distance from zi and zi-1 along xi-1 
link offset(di): sine distance from xi-1 to xi along zi 
angle(Ɵ): angle between xi and xi-1

Examples:

* For calculating _alpha(0)_ I look the angle between z0 and z1, and it's 0.

* For calculating _alpha(1)_ I look the angle between z1 and z2, and it's -pi/2 counter clockwise.

* For calculating _a1_, I look the distance between z1 and z0 along x0. x0 points left, so, it's x in the urdf file. z0 and z1 are in the same place. So the value is 0.

* For calculating _a2_, I look the distance between z2 and z1 along x1. x1 points left, so, it's x in the urdf file. z1 is in joint_0 and z2 is on the right, in joint_2. 
  I can look the distance between joint0 and joint1 in x in the urdf file, it's 0.35

* For calculating _dEE_ distance from x6 to xEE along zEE 0.0375 (link6 X)+0.193(link_gripper X) 0.2305+0.075 (finger)
 
 0.2305

Links | alpha(i-1) | a(i-1) | d(i) | theta(i) q
--- | ---          | ---    | ---    | ---
0->1 | 0           | 0      | 0.75     | 0
1->2 | - pi/2      | 0.35     | 0      | -pi/2 + q2
2->3 | 0           | 1.25      | 0      | 0
3->4 |  -pi/2      | -0.054      | 1.5      | 0
4->5 | pi/2        | 0      | 0      | 0
5->6 | -pi/2       | 0      | 0      | 0
6->EE | 0          | 0      | 0.303      | 0


![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

**Transformation Matrices**

The transformation matrix for each link will have these parameters q,a,d, and alpha.

```
   TF = Matrix ([ [           cos(q),            -sin(q),           0,               a],
                           [sin(q)*cos(alpha),  cos(q)*cos(alpha), -sin(alpha),   -sin(alpha)*d],
                           [sin(q)*sin(alpha),  cos(q)*sin(alpha),  cos(alpha),    cos(alpha)*d],
                           [                0,                  0,           0,                1]
                         ]
                        )
```

I've replaced each parameter in the Matrix with the corresponding values in the row in the DH parameters.

Joint 1
--------


```
   TF_1 = Matrix ([ [           cos(0),            -sin(0),           0,               0],
                           [sin(q)*cos(0),  cos(q)*cos(0), -sin(0),   -sin(0)*0.75],
                           [sin(q)*sin(0),  cos(q)*sin(0),  cos(0),    cos(0)*0.75],
                           [                0,                  0,           0,                1]
                         ]
                        )
```


Joint 2
--------

```
   TF_2 = Matrix ([ [           cos(-pi/2 + q2),            -sin(-pi/2 + q2),           0,               0.35],
                           [sin(-pi/2 + q2)*cos( - pi/2 ),  cos(-pi/2 + q2)*cos( - pi/2 ), -sin( - pi/2 ),   -sin( - pi/2 )*0],
                           [sin(-pi/2 + q2)*sin( - pi/2 ),  cos(-pi/2 + q2)*sin( - pi/2 ),  cos( - pi/2 ),    cos( - pi/2 )*0],
                           [                0,                  0,           0,                1]
                         ]
                        )
```

Joint 3
--------

```
   TF_3 = Matrix ([ [           cos(0),            -sin(0),           0,               1.25],
                           [sin(0)*cos(0),  cos(0)*cos(0), -sin(0),   -sin(0)*0],
                           [sin(0)*sin(0),  cos(0)*sin(0),  cos(0),    cos(0)*0],
                           [                0,                  0,           0,                1]
                         ]
                        )
```

Joint 4
--------

```
   TF_4 = Matrix ([ [           cos(0),            -sin(0),           0,               -0.054],
                           [sin(0)*cos(-pi/2),  cos(0)*cos(-pi/2), -sin(-pi/2),   -sin(-pi/2)*1.5],
                           [sin(0)*sin(-pi/2),  cos(0)*sin(-pi/2),  cos(-pi/2),    cos(-pi/2)*1.5],
                           [                0,                  0,           0,                1]
                         ]
                        )
```

Joint 5
--------

```
   TF_5 = Matrix ([ [           cos(0),            -sin(0),           0,               0],
                           [sin(0)*cos(pi/2),  cos(0)*cos(pi/2), -sin(pi/2),   -sin(pi/2)*0],
                           [sin(0)*sin(pi/2),  cos(0)*sin(pi/2),  cos(pi/2),    cos(pi/2)*0],
                           [                0,                  0,           0,                1]
                         ]
                        )
```

Joint 6
--------

```
   TF_6 = Matrix ([ [           cos(0),            -sin(0),           0,               0],
                           [sin(0)*cos(-pi/2),  cos(0)*cos(-pi/2), -sin(-pi/2),   -sin(-pi/2)*0],
                           [sin(0)*sin(-pi/2),  cos(0)*sin(-pi/2),  cos(-pi/2),    cos(-pi/2)*0],
                           [                0,                  0,           0,                1]
                         ]
                        )
```

Joint EE
--------

```
   TF_EE = Matrix ([ [           cos(0),            -sin(0),           0,               0],
                           [sin(0)*cos(0),  cos(0)*cos(0), -sin(0),   -sin(0)*0.303],
                           [sin(0)*sin(0),  cos(0)*sin(0),  cos(0),    cos(0)*0.303],
                           [                0,                  0,           0,                1]
                         ]
                        )
```


Finally, I have to compose this matrices to get the homogeneous transform between base\_link and gripper\_link.

```
T_end = TF_EE*TF_6*TF_5*TF_4*TF_3*TF_2*TF_1 
```

I need to add a correction based on the different convention between urdf and DH:

```
R_z = Matrix([[ cos(pi), -sin(pi), 0, 0],
              [ sin(pi), cos(pi), 0, 0],
              [ 0 , 0 , 1, 0],
              [ 0 , 0 , 0, 1]])

R_y = Matrix([[ cos(-pi/2), 0, sin(-pi/2), 0],
		[0, 1, 0, 0],
		[-sin(-pi/2), 0, cos(-pi/2), 0],
		[0, 0, 0, 1]])
```


```

R_corr = simplify(R_z * R_y) 

T_end = TF_EE*TF_6*TF_5*TF_4*TF_3*TF_2*TF_1 
T_end = simplify(T0_end * R_corr)

```
	
#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

*Spherical wrist: Finding the wrist center* 

The 3 last joints: joint4, joint5, and joint6 intersect in the same point. That constitutes a *_spherical wrist_*.

The first thing that we need to find in the wrist center _WC_.

According to the lesson: , we can use the following formula.

_wx_ = _px_ - (_d6_+_l_) \* _n\_x_

_wy_ = _py_ - (_d6_+_l_) \* _n\_y_

_wz_ = _pz_ - (_d6_+_l_) \* _n\_z_

where:
* _px_, _py_, _pz_  it's the current coordinates. The pose that is received from the simulator, ROS. 

* _d6_ corresponds to joint\_6 link offset. I can search the value in the DH_Table. It's 0.

* _l_ end-effector length _d7_ in DH Table. From joint\_6 to the end of the finger in the gripper. It's value is 0.303 according to the xacro ant table.

* _n\_x_, _n\_y_, _n\_z_ Needs to be calculated and I can use the rotation matrix to do it.

The pose position is given in _quaternions_ (x,y,z,w). 

*Calculating _n\_x_, _n\_y_, _n\_z_ using Rotation Composition*

I will generate the rotation matrix using this formula:

_Rrpy_ = _Rotz(yaw)_ \* _Roty(pitch)_  \* _Roty(roll)_ \* _Rcorr_


The rotation matrices are defined this way:


```
Rotz(Ɵ) =  [  [ cos(Ɵ), -sin(Ɵ),        0],
              [ sin(Ɵ),  cos(Ɵ),        0],
              [      0,       0,        1]]  
```


```
Roty(Ɵ) =  [  [  cos(Ɵ),  0,   sin(Ɵ)],
              [       0,  1,        0],
              [ -sin(Ɵ),  0,   cos(Ɵ)]] 
```


```
Rotx(Ɵ) =  [  [ 1,         0,       0],
              [ 0,    cos(Ɵ), -sin(Ɵ)],
              [ 0,    sin(Ɵ),  cos(Ɵ)]]  
```

_Rcorr_ is the correction from transforming xacro to DH standars.

```
_Rcorr_ = Rotz(π) * Roty(-π/2)
```

Notice that we don't have the values of roll, pitch, and yaw. I need to convert the quaternion to euler angles. To do that, I will use a function `euler\_from\_quaternion(x,y,z,w)` provided in TF package.


*Inverse Position Kinematics*

In this part, I'm going to find Ɵ1, Ɵ2, and Ɵ3 rotations angles for joints 1, 2, and 3.

I'm going to build a triangle between joint2, joint3, and the WC (joint5) and label the sides and angles.

![IK](misc_images/IK_pic.gif)

First, I will calculate all the sides and the all the angles. 

To calculate the sides:

A = Distance joint 3 to WC. It's d4=1.5

B = I will use pitagoras to calculate the line that goes from the base to the WC. 
    `(sqrt(WC[0]\*WC[0] + WC[1]\*WC[1])` and I will take it off the distance to the first from the ground to the first joint in Y (d1)
    The distance from the origin in z WC[2] minus a1 (location of the first joint in X)

C = Distance between joint 2 and 3. That's a2 in the DH table = 1.25

To calculate the angles, I will use the Cosine formula:

```
C = sqrt(A²+B²-2*A*B*cos(Ɵ))
```

*_Theta1_*

Theta 1 is the angle from x to y. I can use arctan2 to calculate it.

```
theta1 = arctan2(y,x)
```


*_Theta2_*

```
theta2 = pi / 2 - angle_a - angle_from_x_to_y changing coordinates frame to joint 2
```

The joint 2 coordinates frame is in a1, d1 and that's 0.35,0.75.

I need the angle of a point x1, y1 that the point of WC in the new coordinates frame. 


*_Theta3_*

```
theta3 = pi / 2 - angle_b + small_part_that_takes_to_x_axis
```

One side: -0.054m and the other: side_a=1.5
I can use arctan2 to calculate the small angle between x axis.

```
>>> np.arctan2(-0.054,1.5)
-0.035984460082051591
```

```
theta3 = pi / 2 - angle_b - 0.036
```



*Inverse Orientation Kinematics*


* Find transformation matrix

I will calculate from the pose of moving the first 3 joints. To do that, I will calculate the transformation matrix 0T3.

```
0T3 = 3T2 \* 2T1 \* 1T0 
```

This transformation matrix contains the pose. 

```
0T3 = [[r11, r12, r13, px]
       [r21, r22, r23, py]
       [r31, r32, r33, pz]
       [  0,   0,   0,  1]
      ]
```

* Find rotation matrix

```
 3R6 = inv(0R3) * 0R6 = 0R3.T * 0R6 = 0R3.T * Rrpy
```


*_Theta4_*
The position of the revolute rotates x and z. 


*_Theta5_*
The position of the revolute rotates x and y. 


*_Theta6_*
The position of the revolute rotates x and z. 

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


* Gets the WC position

```            
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

```


* Converts the quaternion to euler angles
```
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

```


* I'm going to build the rotation matrix. I will use it to calculate the WC wrist center.

```
            r,p,y = symbols('r p y')

            ROT_x = rot_x(r)    # roll
            ROT_y = rot_y(p)    # pitch
            ROT_z = rot_z(y)    # yaw

            ROT_EE = ROT_z * ROT_y * ROT_x
```

* I need to apply a correction error because DH Table uses another standard.

            Rot_error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))

            ROT_EE = ROT_EE * Rot_error  #compenstate for error
            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

* Calculates the WC

As I've said before, the WC are going to be 3 coordinates x,y,z and:

we can use the following formula.

_wx_ = _px_ - (_d6_+_l_) \* _n\_x_

_wy_ = _py_ - (_d6_+_l_) \* _n\_y_

_wz_ = _pz_ - (_d6_+_l_) \* _n\_z_


  * _n\_x_, _n\_y_, _n\_z_ = ROT_EE

  * _d6_ value in table is 0

  * _l_ is 0.303

So, in the code, I've written:

```
            EE = Matrix([[px],
                         [py],
                         [pz]])

            WC = EE - (0.303) * ROT_EE[:,2] # wrist center
```


* Sides and angles in the triangle

```
            side_a = 1.501
            side_b = sqrt(pow((sqrt(WC[0]*WC[0] + WC[1]*WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
            side_c = 1.25

            angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
            angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))
```

* Theta 1

```
            theta1 = atan2(WC[1],WC[0])
```


* Theta 2
```
            theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
```


* Theta 3
```
            theta3 = pi / 2 - (angle_b + 0.036)  # sag in link 4 -0.054m
```

