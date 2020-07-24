% helper function for lynxVelocitySim
% Input: q - 1 x 6 vector of joint inputs [q1,q2,q3,q4,q5,q6]
%        qdot - 1 x 6 vector of joint velocities [q1dot,q2dot,q3dot,q4dot,q5dot,q6dot]
function joint_pos = plotRotation(q, moving_joint)

% scaling factors for plotting velocity line segments on end effector
lin_scale = 0.4;
ang_scale = 100;

% constants for plotting
line_width = 2;
arrow_size = 1;
axis_lim = [-305, 315, -300, 320, 0, 400];

% plot links of Lynx
[joint_pos,~] = calculateFK_sol(q);
joint_pos(abs(joint_pos) < 10^-14) = 0; % eliminate tiny precision errors from calculateFK
h_links = plot3(joint_pos(:,1), joint_pos(:,2), joint_pos(:,3), 'k-o',...
    'LineWidth', 2);
axis(axis_lim)
view([-36,30])
% xlabel('x'), ylabel('y'), zlabel('z')
set(gca, 'FontSize', 12)
hold on

% solve for end effector position and velocity
e_pos = joint_pos(6,:);
qdot = zeros(1,6); qdot(moving_joint) = 1;
e_vel = FK_velocity(q, qdot);

% create velocity line segments from end effector position
e_lin_vel = e_vel(1:3)' * lin_scale;
e_ang_vel = e_vel(4:6)' * ang_scale;
lin_vel_line = [e_pos; e_pos + e_lin_vel];
ang_vel_line = [e_pos; e_pos + e_ang_vel];

% highlight moving_joint
p_pos = joint_pos(moving_joint, :);
h_highlight = plot3(p_pos(1), p_pos(2),p_pos(3),'go', 'LineWidth', 2);
hold on

% plot axis of rotation
h_ang = quiver3(e_pos(1), e_pos(2), e_pos(3), e_ang_vel(1), e_ang_vel(2), e_ang_vel(3),...
    'b', 'LineWidth', line_width,'MaxHeadSize',arrow_size);
text(e_pos(1)+20, e_pos(2), e_pos(3)+20,'$\vec{v}$','Interpreter','latex','FontSize',14)
hold on

% plot base axis
h_base = quiver3([0;0;0], [0;0;0], [0;0;0], [80;0;0], [0;80;0], [0;0;80],...
    'r', 'LineWidth', line_width,'MaxHeadSize',arrow_size);
text([100;0;30],[0;130;0],[0;0;60],['$x$';'$y$';'$z$'],'Interpreter','latex','FontSize',14)

% plot position vector
len = joint_pos(moving_joint+1, :) - p_pos;
h_lin = quiver3(p_pos(1), p_pos(2), p_pos(3), len(1), len(2), len(3),...
    'g', 'LineWidth', line_width,'MaxHeadSize',0.5);
off = [-40,0,60];
text(p_pos(1)+off(1),p_pos(2)+off(2),p_pos(3)+off(3),...
    ['$d_{',num2str(moving_joint),'}$'],'Interpreter','latex','FontSize',14)
hold on
end