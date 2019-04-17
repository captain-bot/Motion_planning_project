clc
clear
close all

% Seed of random numbers
% rng(100);

% Maze parameter
maze_thickness = 0.1; maze_gap = 0.5; maze_height = 1.0;

% Point to check
X_MIN = maze_thickness; X_MAX = maze_gap; Y_MIN = 0; Y_MAX = maze_height; 
pt = [X_MIN + rand*(X_MAX - X_MIN), Y_MIN + rand*(Y_MAX - Y_MIN)];

% Two end points of the line
% X1 = [maze_thickness, maze_height-maze_thickness; maze_gap, 0;];
% X2 = [maze_thickness maze_height; maze_gap maze_thickness];

X1 = [maze_gap maze_thickness; maze_thickness maze_height];
X2 = [maze_gap, 0; maze_thickness, maze_height-maze_thickness];

% Define the line equations
w1 = [X1(2, 2) - X1(1, 2); X1(1, 1) - X1(2, 1)];
b1 = X1(1, 1)*(X1(1, 2) - X1(2, 2)) + X1(1, 2)*(X1(2, 1) - X1(1, 1));

w2 = [X2(2, 2) - X2(1, 2); X2(1, 1) - X2(2, 1)];
b2 = X2(1, 1)*(X2(1, 2) - X2(2, 2)) + X2(1, 2)*(X2(2, 1) - X2(1, 1));

% Check values for w^T*x + b
val1 = pt*w1 + b1; fprintf("Distance of the point from line 1: %2.6f\n", val1);
val2 = pt*w2 + b2; fprintf("Distance of the point from line 2: %2.6f\n", val2);

is_collision = 1;
if val1 < 0 && val2 > 0
    is_collision = 0;
    fprintf("The point is not in collision \n");
end

% Plot line
plot(X1(:, 1), X1(:, 2), 'r');
hold on;
plot(X2(:, 1), X2(:, 2), 'r');

if is_collision
    marker_color = 'm';
else
    marker_color = 'g';
end

plot(pt(1), pt(2), 'o', 'MarkerFaceColor', marker_color);
grid minor;
title("Collision check visualization")
xlabel('x [m]');
ylabel('y [m]');