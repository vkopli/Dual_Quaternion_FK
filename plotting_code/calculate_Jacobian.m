% Determine 6 x 5 Jacobian Matrix for Inputted Angles
function [J, T01, T02, T03, T04, T05] = calculate_Jacobian(q)
    
    %% Constants: Lynx Joint Lengths
    % Lynx ADL5 constants in mm
    d1 = 76.2; % base height (table to center of joint 2)
    a2 = 146.05; % shoulder to elbow length
    a3 = 187.325; %elbow to wrist length
    d5 = 76.2; %wrist to base of gripper
    lg = 28.575; %length of gripper
    
    %% Expressions: Transformation Matrices and their Derivatives
    
    %Frame 1 w.r.t Frame 0
    T1 = [cos(q(1)) -sin(q(1))*cos(-pi/2)  sin(q(1))*sin(-pi/2)  0;
          sin(q(1))  cos(q(1))*cos(-pi/2) -cos(q(1))*sin(-pi/2)  0;
                  0            sin(-pi/2)            cos(-pi/2) d1;
                  0                     0                  0     1];
    dT1 = [-sin(q(1)) -cos(q(1))*cos(-pi/2)  cos(q(1))*sin(-pi/2)  0;
          cos(q(1))  -sin(q(1))*cos(-pi/2)  sin(q(1))*sin(-pi/2)   0;
                  0                     0                  0      0;
                  0                     0                  0     0];

    %Frame 2 w.r.t Frame 1          
    T2 = [cos(q(2)-(pi/2)) -sin(q(2)-(pi/2))  0   a2*cos(q(2)-(pi/2));
          sin(q(2)-(pi/2))  cos(q(2)-(pi/2))  0   a2*sin(q(2)-(pi/2));
                  0                        0  1                     0;
                  0                        0  0                     1];
    dT2 = [-sin(q(2)-(pi/2)) -cos(q(2)-(pi/2))  0   -a2*sin(q(2)-(pi/2));
          cos(q(2)-(pi/2))  -sin(q(2)-(pi/2))  0   a2*cos(q(2)-(pi/2));
                  0                        0  0                     0;
                  0                        0  0                     0];
    %Frame 3 w.r.t Frame 2
    T3 = [cos(q(3)+(pi/2)) -sin(q(3)+(pi/2))  0   a3*cos(q(3)+(pi/2));
          sin(q(3)+(pi/2))  cos(q(3)+(pi/2))  0   a3*sin(q(3)+(pi/2));
                  0                        0  1                     0;
                  0                        0  0                     1];
    dT3 = [-sin(q(3)+(pi/2)) -cos(q(3)+(pi/2))  0   -a3*sin(q(3)+(pi/2));
          cos(q(3)+(pi/2))  -sin(q(3)+(pi/2))  0   a3*cos(q(3)+(pi/2));
                  0                        0  0                     0;
                  0                        0  0                     0];

    %Frame 4 w.r.t Frame 3
    T4 = [cos(q(4)-(pi/2)) -sin(q(4)-(pi/2))*cos(-pi/2)   sin(q(4)-(pi/2))*sin(-pi/2)   0;
          sin(q(4)-(pi/2))  cos(q(4)-(pi/2))*cos(-pi/2)  -cos(q(4)-(pi/2))*sin(-pi/2)   0;
                  0                          sin(-pi/2)                    cos(-pi/2)   0;
                  0                                   0                             0   1];
    dT4 = [-sin(q(4)-(pi/2)) -cos(q(4)-(pi/2))*cos(-pi/2)   cos(q(4)-(pi/2))*sin(-pi/2)   0;
          cos(q(4)-(pi/2))  -sin(q(4)-(pi/2))*cos(-pi/2)  sin(q(4)-(pi/2))*sin(-pi/2)   0;
                  0                                   0                             0   0;
                  0                                   0                             0   0];

    %Frame 6 w.r.t Frame 4 
    T5 = [cos(q(5)) -sin(q(5))  0        0;
          sin(q(5))  cos(q(5))  0        0;
                  0          0  1       d5 + lg;
                  0          0  0        1];
    %Frame 6 w.r.t Frame 4 
    dT5 = [-sin(q(5)) -cos(q(5))  0        0;
          cos(q(5))  -sin(q(5))  0        0;
                  0          0  0       0;
                  0          0  0        0];

    %% Calculate each column of Linear Velocity Jacobian
    
    J1 = dT1 * T2 * T3 * T4 * T5 * [0; 0; 0; 1];
    J2 = T1 * dT2 * T3 * T4 * T5 * [0; 0; 0; 1];
    J3 = T1 * T2 * dT3 * T4 * T5 * [0; 0; 0; 1];
    J4 = T1 * T2 * T3 * dT4 * T5 * [0; 0; 0; 1];
    J5 = T1 * T2 * T3 * T4 * dT5 * [0; 0; 0; 1];
    
    %% Calculate Full Jacobian, including Angular Velocity Jacobian
    
    Jv = [J1, J2, J3, J4, J5];
    Jv = Jv(1:end-1, :);
    
    T01 = T1;
    T02 = T1*T2;
    T03 = T02*T3;
    T04 = T03*T4;
    T05 = T04*T5;
    
    Jw = [[0,0,1]', T01(1:end-1,3), T02(1:end-1,3), T03(1:end-1,3), T04(1:end-1,3)];
    
    J = [Jv; Jw];
end