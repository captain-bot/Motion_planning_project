clc
clear
close all

addpath('../RR_supporting_files');

global links;      
global center_c1;  global radius1;
global center_c2;  global radius2;
global center_c3;  global radius3;

% Joint angles
q1 = 0.6; q2 = 5;
q = [q1;q2];

% Links
links = [1,1];

% Define obstacles
center_c1 = [0.8; 1.4];  radius1 = 0.5;
center_c2 = [-0.6; 1.0]; radius2 = 0.4;
center_c3 = [1.1; 0.1];  radius3 = 0.4;

% Get closest distances
closest_dists = check_closest_dist(q);

% Compute end effector configurations
x1 = links(1)*cos(q1);
y1 = links(1)*sin(q1);
x2 = x1 + links(2)*cos(q1+q2);
y2 = y1 + links(2)*sin(q1+q2);

% Plot obstacles
viscircles([center_c1,center_c2,center_c3]',[radius1,radius2,radius3]);

% Plot arms
xlim([-0.8,2]);
ylim([-0.25,2]);
grid on;
plot_config = line([0,x1,x2],[0,y1,y2]);
xlabel('x [m]');
ylabel('y [m]');
title('Planer RR robot motion plan')