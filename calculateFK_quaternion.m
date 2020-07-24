function [jointPositions, Q0e] = calculateFK_quaternion(q)
% tic
% Input: q - 1 x 6 vector of joint angles (configuration) [q1,q2,q3,q4,q5,~]

% Outputs:  jointPositions - 6 x 3 matrix, where each row represents one 
%               joint along the robot. Each row contains the [x,y,z]
%               coordinates of the respective joint's center (mm). For
%               consistency, the first joint should be located at [0,0,0].
%               These values are used to plot the robot.
% 
%           Q0e - dual quaternion (point-vector transformation) from base
%           to end effector

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Constants

% Lynx ADL5 constants in mm
d1 = 76.2; % base height (table to center of joint 2)
d2 = 146.05; % shoulder to elbow length
d3 = 187.325; %elbow to wrist length
d4 = 34; %wrist to joint 5
d5 = 42.2; %joint 5 to base of gripper
lg = 28.575; %length of gripper

%% Dual Quaternion Definitions

%Frame 2 w.r.t Frame 1
Q1.q = [cos(q(1)/2), sin(q(1)/2)*[0, 0, 1]];
Q1.p = [0, 0, 0, d1];
          
%Frame 3 w.r.t Frame 2          
Q2.q = [cos(q(2)/2), sin(q(2)/2)*[0, 1, 0]];
Q2.p = [0, d2*sin(q(2)), 0, d2*cos(q(2))];

%Frame 4 w.r.t Frame 3
Q3.q = [cos(q(3)/2), sin(q(3)/2)*[0, 1, 0]];
Q3.p = [0, d3*cos(q(3)), 0, -d3*sin(q(3))];

%Frame 5 w.r.t Frame 4
Q4.q = [cos(q(4)/2), sin(q(4)/2)*[0, 1, 0]];
Q4.p = [0, d4*cos(q(4)), 0, -d4*sin(q(4))];

%Frame 6 w.r.t Frame 5 
Q5.q = [cos(q(5)/2), sin(q(5)/2)*[1, 0, 0]];
Q5.p = [0, d5, 0, 0];
          
%Frame e w.r.t Frame 6 
Q6.q = [1, 0, 0, 0];
Q6.p = [0, lg, 0, 0];

Q02 = Q1;
Q03 = Q_mult(Q02, Q2);
Q04 = Q_mult(Q03, Q3);
Q05 = Q_mult(Q04, Q4);
Q06 = Q_mult(Q05, Q5);
Q0e = Q_mult(Q06, Q6);

jointPositions = [[0,0,0]; Q02.p(2:4); Q03.p(2:4); Q04.p(2:4); Q05.p(2:4); Q06.p(2:4)];

%% Functions

% DESCRIPTION:
%   Multiply 2 dual quaternions (point-vector transformations) 
% INPUTS
%   Q1: dual quaternion (Q.q: [s, vx, vy, vz], Q.p = [0, p1, p2, p3])
%   Q2: dual quaternion (Q.q: [s, vx, vy, vz], Q.p = [0, p1, p2, p3])
%
function Q = Q_mult(Q1, Q2)
    Q.q = quatmultiply(Q1.q, Q2.q);    
    q1_inv = [Q1.q(1), -Q1.q(2:4)];
    Q.p = quatmultiply(quatmultiply(Q1.q, Q2.p), q1_inv) + Q1.p;
end

% toc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
