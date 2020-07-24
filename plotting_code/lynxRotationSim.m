% function to simulate end effector velocities
% adds extra stuff to helper method plotVelocity.m: draws circles around
%   joint to represent trajectory of path for changing one joint angle
function lynxRotationSim(q, moving_joint)

if (moving_joint < 1 || moving_joint > 5)
    error('moving joint must be joint 1 thru 5')
end

close all
figure();
joint_pos = plotRotation(q, moving_joint);
hold on

% plot circle representing range of motion of joint
[J, T01, T02, T03, T04, ~] = calculate_Jacobian(q);
disp(rank(J))
n = [0 0 1;
    T01(1:3,3)';
    T02(1:3,3)';
    T03(1:3,3)';
    T04(1:3,3)']; % axis of rotation for each joint

j = moving_joint;
pos = joint_pos(j,:);
r = joint_pos(j+1,:) - joint_pos(j,:); R = sqrt(sum(r.^2));
a = cross(r/R, n(j,:)); b = cross(a, n(j,:));
th = linspace(0,2*pi,100);
xunit = pos(1) + R*cos(th)*a(1) + R*sin(th)*b(1);
yunit = pos(2) + R*cos(th)*a(2) + R*sin(th)*b(2);
zunit = pos(3) + R*cos(th)*a(3) + R*sin(th)*b(3);
plot3(xunit, yunit, zunit, '--k')

end