clc;
clear;
close all;

global L0; global L1; global L2; global L3;
global o1; global o2; global o3; global gripper_link;

addpath('../manipulator_kinematics');

% Theta
theta = [0.1,-0.5,0.1,0.1,0.1,0.5,0.1];
% theta = zeros(1,7);

% Joint types for the manipulator
type_joint = ['R'; 'R'; 'R'; 'R'; 'R'; 'R'; 'R'];

% Link lengths
L0 = 0.27035;
o1 = 0.069;
L1 = 0.36435;
o2 = 0.069;
L2 = 0.37429;
o3 = 0.010;
L3 = 0.22952;
gripper_link = 0.025 + 0.1372; % this needs to be confirmed

% Define base
base_mat = translation(0.025447174775, 0.219028201403, 0.10797895129)...
    *rot_z(pi/4)*translation(0.055695, 0, 0.011038);

% Define rotation axes
w1 = [0;0;1];
w2 = [0;1;0];
w3 = [1;0;0];
w4 = w2;
w5 = w3;
w6 = w2;
w7 = w3;
wr = [w1, w2, w3, w4, w5, w6, w7];

% Frame origins
q1 = [0;0;0];         % s0
q2 = [o1;0;L0];       % s1
q3 = q2;              % e0
q4 = [o1+L1;0;L0-o2]; % e1
q5 = q4;              % w0
q6 = [o1+L1+L2;0;L0-o2-o3]; % w1
q7 = q6;              % w2
qr = [q1, q2, q3, q4, q5, q6, q7];

% Define g_st0
g_st0 = [0,0,1,1.194935;
                0,1,0,0;
         -1,0,0,0.19135;
                0,0,0,1];

% Solve direct position kinematics
[gst, transform_upto_joint] = direct_kin(g_st0, type_joint, wr, qr, theta);

% Transformation of the end frame
g_end = base_mat*gst;

% Plot robot
draw_baxter(base_mat, transform_upto_joint, g_end);

% Direct Velocity Kinematics
% [spatial_jac, spatial_vel_st] = velocity_direct_kin(gst0, type_joint, wr, qr, theta, thetadot);
