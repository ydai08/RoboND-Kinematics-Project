## Project: Kinematics Pick & Place
### Completed by Yang Dai

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[kinemat1]: ./writeup_images/kinemat_notes-1.png
[kinemat2]: ./writeup_images/kinemat_notes-2.png
[kinemat3]: ./writeup_images/kinemat_notes-3.png
[kinemat4]: ./writeup_images/kinemat_notes-4.png
[transform1]: ./writeup_images/direct_transform.png
[image1]: ./writeup_images/complete_pick.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

URDF frame information extracted from the kr210.urdf.xacro file and DH reference frames and parameter derivations are shown below:

![Kinemat Notes][kinemat1]
![Kinemat Notes][kinemat2]


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

See FK.py lines 9-48 for the generation of individual DH transform matrices.  The generalized homogeneous transform using only gripper pose calculated by FK.py lines 78-104 is the following:

![Direct Transform][transform1]

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Inverse kinematics derivations are shown below:

![Inverse Kinemat][kinemat3]
![Inverse Kinemat][kinemat4]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The same structure for deriving transforms in FK.py was used in IK_server.py.  To avoid repetition and typos, a generic DH transform function was created so individual joint symbol sets could be substituted in for evaluation.  The symbolic transform matrices do not change from point to point, so these were removed from the loop and calculated only once prior to receiving gripper pose input.  DH transform matrices for joints 3 to gripper were not explicitly needed for calculating the inverse kinematics since the equations dictating thetas 4-6 were derived from above, so these matrix calculations were removed.  Similarly, the full direct homogeneous transform from base link to gripper using end effector pose was reduced to only the rotation matrix for purposes of deriving thetas 4-6.

Simplify functions were removed to speed up the loop performance.  Because all outputs of interest were numerical, keeping symbolic representations was unnecessary and equations containing symbols were evaluated numerically as early in the loop as possible to avoid carrying symbols forward.  This significantly reduced the inverse kinematic calculation times (from minutes to seconds).  While substituting numerical values early can potentially amplify compounding rounding errors, the current implementation still passes the 8/10 pick and place cycle requirement, so the speed improvement is well worth the minor loss in accuracy.

Per Slack channel recommendations, a delay was placed in the trajectory_sampler.cpp file to allow use of the 'continue' function in rviz rather than manually clicking through 'next' at each step.  Without the delay, the gripper would not fully close on the target prior to retraction and thus would repeatedly leave targets on the shelf.

Future improvements: there are extraneous symbols defined in the DH parameter dictionary which are not used in the inverse kinematics calculations and can be removed, but for ease of reading and code review, the complete table was included.  A minor initialization speed improvement would be to hardcode the T0_3 and R_direct matrices rather than recalculating them each time IK_server.py is run.  Occasionally, the gripper would appear to be in the correct location in front of the target at the end of the IK movement, but the following automatic "reach" step would back it out of position, execute a waving motion, and knock over the target rather than reaching forward directly for it.  This could be investigated to improve pickup performance.

Completed picking screencap:
![Completed Picking][image1]


