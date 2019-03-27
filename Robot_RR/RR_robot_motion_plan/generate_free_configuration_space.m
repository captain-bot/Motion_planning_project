clc
clear
close all

addpath('../RR_supporting_files');

global links;      global num_dof;
global initial_st; global goal_st;
global center_c1;  global radius1;
global center_c2;  global radius2;
global center_c3;  global radius3;

% Safe distance
d_safe = 0.005;

% Define the robot parameters
links = [1,1];
num_dof = length(links);

% Initial and goal states
initial_st = [0.22*pi, 1.94*pi, 0, 0];
goal_st = [0.5*pi, 0.1*pi, 0, 0];

% Define obstacles
center_c1 = [0.8; 1.4];  radius1 = 0.5;
center_c2 = [-0.6; 1.0]; radius2 = 0.4;
center_c3 = [1.1; 0.1];  radius3 = 0.4;

% Define ranges for theta_1 and theta_2
th1 = 0.4:0.01:2;
th2 = 0:0.01:2*pi;
x_invalid = [];
y_invalid = [];
for x=th1
    for y=th2
        closest_dists = check_closest_dist([x;y]);
        if closest_dists(1) < d_safe || closest_dists(2) < d_safe
            x_invalid = [x_invalid, x];
            y_invalid = [y_invalid, y];           
        end
    end
end

plot(x_invalid, y_invalid, '.')
hold on
grid on
x_valid = load('valid_actual_config.mat');
exp_nodes = load('expanded_nodes.mat', 'exp_node_list');
plot(x_valid.config_joint_space(1,:), x_valid.config_joint_space(2,:), 'r', 'LineWidth', 1.0);
plot(exp_nodes.exp_node_list(1:end-2, 1), exp_nodes.exp_node_list(1:end-2, 2), 'bo', 'MarkerFaceColor', 'b')
plot(initial_st(1), initial_st(2), 'o', 'MarkerFaceColor', 'r');
plot(goal_st(1), goal_st(2), 'o', 'MarkerFaceColor', 'g')
xlabel('q_1')
ylabel('q_2')
legend({'Obstacles', 'Robot path', 'Nodes expanded', 'Start', 'Goal'}, 'Location', 'northwest', 'NumColumns', 3);
title('Robot path along free configuration space')
