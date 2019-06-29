# RoboND_Robotic-Arm-Pick-Place

This project covers the forward and inverse kinematic for a six DOF Kuka KR210 robot arm for objects pick and place.
ROS, Gazebo, RViz, Moveit! are also implemented in this project.

This is a README that includes all the key points and how I addressed each one.

## 1 Kinematic Analysis
### 1.1 Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.
Here is the RViz TF Display with Robot Model:

![RViz TF Display with Robot Model](image/RViz_TF_Display_with_Robot_Model.png)

Here is the RViz TF Display without Robot Model:

![RViz TF Display without Robot Model](image/RViz_TF_Display_without_Robot_Model.png)

They provides very clear schematics of URDF coordinate for each joints. The TF information in left Display section will also help to come up with the modified DH parameters and build the modified DH Table using the convention described in John J Craig's book.

DH reference frames of each joint are shown below:

![DH_reference_frame](image/DH_reference_frame.png)

And the according DH table calculated using the URDF files is:
Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0
