clc;
clear;
close all;

addpath('../manipulator_kinematics/');
addpath('../Baxter_supporting_files/');

global g_st0; global type_joint; global wr;
global qr;    global base_mat;   global cir;
global gst0_art; global plot_count_num;

plot_count_num = 1;

joint_q = [0.1760, -0.5563, 0.1000, 0.1719, 0.1479, 0.1479, 0.1000;
0.1752, -0.5303, 0.0995, 0.1853, 0.1563, 0.1566, 0.1000;
0.1484, -0.3444, 0.0988, 0.2787, 0.2177, 0.2186, 0.1000;
0.0746, -0.2417, 0.0593, 0.3444, 0.2125, 0.2528, 0.0926;
0.0825, -0.1578, 0.0790, 0.3787, 0.2599, 0.2806, 0.0955;
0.0963, -0.1121, 0.0956, 0.3956, 0.2916, 0.2959, 0.0991];

% /////////////////////////////////////////////
%           Obstacle parameters              //
%//////////////////////////////////////////////
% Obstacle parameters
cir = [0.05,0.5,0.8,0.6;
       0.05,0.35,0.7,0.4;
       0.05,0.6,1.0,0.3];    % format [r,x,y,z]
safe_dist = 0.01;           % safety distance (buffer)

%///////////////////////////////////////////
%     Robot parameters (Baxter robot)     //
%///////////////////////////////////////////
num_dof = 7;           % Define DoF
num_contacts = 4;      % Number of possible contacts

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
wr_base = base_mat(1:3,1:3)*wr;   

% Frame origins
q1 = [0;0;0];               % s0
q2 = [o1;0;L0];             % s1
q3 = q2;                    % e0
q4 = [o1+L1;0;L0-o2];       % e1
q5 = q4;                    % w0
q6 = [o1+L1+L2;0;L0-o2-o3]; % w1
q7 = q6;                    % w2
qr = [q1, q2, q3, q4, q5, q6, q7];
qr_base = base_mat(1:3,1:3)*qr + base_mat(1:3,4);

% Define g_st0 (end point)
g_st0 = [0,0,1,1.194935;
         0,1,0,0.0000;
        -1,0,0,0.19135;
         0,0,0,1.0000];
g_st0_base = base_mat*g_st0;
     
% g_st0 of intermediate points
gst0_art(:,:,1) = eye(4,4);
gst0_art(:, :, 2) = [eye(3,3), [0;0;L0]; 0 0 0 1];
gst0_art(:, :, 3) = [eye(3,3), [o1;0;L0]; 0 0 0 1];
gst0_art(:, :, 4) = [eye(3,3), [o1+L1;0;L0]; 0 0 0 1];
gst0_art(:, :, 5) = [eye(3,3), [o1+L1;0;L0-o2]; 0 0 0 1];
gst0_art(:, :, 6) = [eye(3,3), [o1+L1+L2;0;L0-o2]; 0 0 0 1];
gst0_art(:, :, 7) = [eye(3,3), [o1+L1+L2;0;L0-o2-o3]; 0 0 0 1];

for i = 1:size(joint_q, 1)
    visualizeBaxter(joint_q(i, :));
end

function [] = visualizeBaxter(theta)
    global g_st0; global type_joint; global wr;
    global qr;    global base_mat;

    % Solve direct position kinematics
    [gst, transform_upto_joint] = direct_kin(g_st0, type_joint, wr, qr, theta);

    % Transformation of the end frame
    g_end = base_mat*gst;

    % Plot robot
    draw_baxter(base_mat, transform_upto_joint, g_end);
end