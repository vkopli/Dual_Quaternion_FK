# Dual_Quaternion_FK

## Summmary

Forward Kinematics (FK) is the use of kinematic equations 
to compute the position of the end-effector of a robot arm 
given the arm's configuration. 

This project demonstrates how dual quaternions can be used 
to implement FK on the Lynx 6-dof robot arm, and how the methods 
and results compare to that of homogeneous transformations. 

Report: https://www.dropbox.com/s/ou234a83mgn7tpd/Dual%20Quaternion%20FK.pdf?dl=0 \
Presentation: https://www.dropbox.com/s/sobsietfmfgoyqg/Dual%20Quaternion%20FK.pptx?dl=0

## FK Algorithm Code

Both "calculateFK_quaternion.m" and "calculateFK_sol.m"
are functions that take a 1x6 input of joint angles (in
radians) and output a 6x3 matrix of <x,y,z> joint 
positions (in mm). The former uses dual quaternions in
the computation, and the latter uses homogeneous
transformations.

## Testing Code

To compare the results between "calculateFK_quaternion.m" 
and "calculateFK_sol.m", open "code_tests.m", and replace 
the numbers in q1-q6 with 1x6 joint angle inputs of your
choosing. The program outputs 6x3 matrices of joint
positions for each implementation and input.

The results can be checked manually with the robot or 
visually with the plotting code. 

## Plotting Code

To plot the robot arm configuration: 
1) Add the "plotting_code" folder to your search paths in 
   Matlab. 
2) Call "lynxRotationSim" on a 1x7 vector, where the first 6
   numbers are joint angles in radians and the 7th number is 
   any integer from 1 to 5.
   
The 7th input is used for the Methods section of the report to 
visualize the plane of movement for joints 1 thru 5 when the 
robot is in the zero pose. Just use 1 as a default input.
