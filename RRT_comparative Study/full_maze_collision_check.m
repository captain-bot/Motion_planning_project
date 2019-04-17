clc
clear
close all

addpath("my_supporting_files");

global maze_height;    global maze_length;    global qs; 
global maze_thick;     global maze_gap;       global qg;

% Maze parameter
maze_thick = 0.1; maze_gap = 0.5; maze_height = 1.0; maze_length = 5.0;

% Maze range
X_MIN = maze_gap; X_MAX = maze_length-maze_gap; Y_MIN = 0; Y_MAX = maze_height; 
rand_pt = [X_MIN + rand*(X_MAX - X_MIN), Y_MIN + rand*(Y_MAX - Y_MIN)];
% rand_pt = [3, 0.09];

% Define start and goal configuration
qs = [0.5; 0.9]; qg = [5.5; 0.5];

% Draw Maze
[inner_maze_pts, outer_maze_pts] = new_drawmaze();

% Point
% pt = [1.25, 0.75];
% rand_pt = [3.9, 0.10];
plot(rand_pt(1), rand_pt(2), 'o', 'MarkerFaceColor', 'b');

% Check for collision with the maze wall
is_collision = check_collision(inner_maze_pts, outer_maze_pts, rand_pt);
fprintf("is_collision: %d\n", is_collision);


% Collision Check
function [is_collision] = check_collision(inner_maze_pts, outer_maze_pts, pt)
    % Set the collision flag to 1
    is_collision = 1;
    
    % Check which segment the point is lying
    for i = 1:length(inner_maze_pts)-1
        if pt(1) >= inner_maze_pts(i) && pt(1) < inner_maze_pts(i+1)
            fprintf("Point lies in the %d segment of the maze\n", i);
            break;
        end
    end

    % Check for collision within that range
    outer_line = [outer_maze_pts(i, :); outer_maze_pts(i+1, :)];
    w_outer = [-outer_line(2, 2) + outer_line(1, 2); -outer_line(1, 1) + outer_line(2, 1)];
    b_outer = outer_line(1, 1)*(-outer_line(1, 2) + outer_line(2, 2)) + outer_line(1, 2)*(-outer_line(2, 1) + outer_line(1, 1));

    inner_line = [inner_maze_pts(i, :); inner_maze_pts(i+1, :)];
    w_inner = [-inner_line(2, 2) + inner_line(1, 2); -inner_line(1, 1) + inner_line(2, 1)];        
    b_inner = inner_line(1, 1)*(-inner_line(1, 2) + inner_line(2, 2)) + inner_line(1, 2)*(-inner_line(2, 1) + inner_line(1, 1));

    val1 = pt*w_outer + b_outer;
    val2 = pt*w_inner + b_inner;

    if val1 <= 0 && val2 >= 0
        % Flip the collision flag
        is_collision = 0;
        fprintf("Point is in segment %d\n", i);
        fprintf("val1: %2.6f, val2: %2.6f\n", val1, val2);
    else
        fprintf("Generated point is in collision\n");
    end
end