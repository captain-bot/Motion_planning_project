clc
clear
close all

addpath("my_supporting_files");

global maze_height;    global maze_length;    global qs; 
global maze_thick;     global maze_gap;       global qg;

tic;
% Maze parameter
maze_thick = 0.1; maze_gap = 0.5; maze_height = 1.0; maze_length = 5.0;

% % Maze range
X_MIN = maze_gap; X_MAX = maze_length-maze_gap; Y_MIN = 0; Y_MAX = maze_height; 
% rand_pt = [X_MIN + rand*(X_MAX - X_MIN), Y_MIN + rand*(Y_MAX - Y_MIN)];

% Define start and goal configuration
qs = [0.5; 0.9]; qg = [4.5; 0.9];
% qg = [1.9228, 0.2296]; % qg = [4.5; 0.9];
start = qs'; goal = qg';

% Draw Maze
[inner_maze_pts, outer_maze_pts] = new_drawmaze();

% Define control parameters
STRIDE = 0.025; dt = 0.2;
MAX_MOVE = 0.1; MAX_ITR = 4000;
BIG = 1000;
% rng(150);

% Build and initiate tree
root_node.coord = start;
root_node.parent = 0;
root_node.cost = 0;
root_node.localcost = 0;

% Add root node to the tree
TREE(1) = root_node;
node_count = 1;

% Grow tree
% EXIT_FLAG = false;
for i = 1:1:MAX_ITR
% while EXIT_FLAG == false
    q_rand_collision = 1;
    while q_rand_collision ~= 0
        q_r = [X_MIN + (X_MAX - X_MIN)*rand, Y_MIN + (Y_MAX - Y_MIN)*rand];
        q_rand_collision = check_collision(inner_maze_pts, outer_maze_pts, q_r);
    end

    % Generate q_rand
    q_rand.x = q_r(1);
    q_rand.y = q_r(2);
    
    % Find nearest node in the tree that is closest to q_rand
    dist_min = BIG;
    for j = 1:length(TREE)
        dist_current = sqrt((TREE(j).coord(1) - q_rand.x)^2 ...
                                      + (TREE(j).coord(2) - q_rand.y)^2);
        if dist_current < dist_min
            dist_min_idx = j;
            dist_min = dist_current;
        end
    end
    
    % Steer from q_nearest toward q_rand
    DIST = sqrt((q_rand.x - TREE(dist_min_idx).coord(1))^2 ...
                            + (q_rand.y - TREE(dist_min_idx).coord(2))^2);
    add_q_new = 0;
    if DIST > STRIDE
        t = 0;
        while t < dt
            move_dir = [q_rand.x - TREE(dist_min_idx).coord(1),...
                                (q_rand.y - TREE(dist_min_idx).coord(2))];
            move_dir = move_dir/norm(move_dir);
            if t == 0
                old_coord = TREE(dist_min_idx).coord + STRIDE * move_dir;
            else
                new_coord = old_coord + STRIDE * move_dir;
            end
            if t > 0
                if check_collision(inner_maze_pts, outer_maze_pts, new_coord) == 1
                    coord = old_coord;
                    break;
                end
                old_coord = new_coord;
                coord = new_coord;
            else
                if check_collision(inner_maze_pts, outer_maze_pts, old_coord) == 1
                    break;
                end
                add_q_new = 1;
            end
            t = t + STRIDE;
        end
    end
    
    if add_q_new == 1
        % Construct q_new
        q_new.coord = coord;
        q_new.parent = dist_min_idx;
        q_new.localcost = norm(q_new.coord - TREE(dist_min_idx).coord);
        q_new.cost = TREE(q_new.parent).cost + q_new.localcost;

        % Add q_new to teh tree
        TREE(node_count+1) = q_new;
        node_count = node_count + 1;
        
        if norm(goal - q_new.coord) <= 0.1
            break;
        end
    end
    
end

% Find the path out
min_dist_goal = BIG;
for i = 1:length(TREE)
    dist_goal = norm(TREE(i).coord - goal);
    if dist_goal < min_dist_goal
        min_dist_node_indx = i;
        min_dist_goal = dist_goal;
    end
end
% plot(TREE(min_dist_node_indx).coord(1), TREE(min_dist_node_indx).coord(2), 'o', 'MarkerFaceColor', 'm')

% Add goal node to the tree
goal_node.coord = goal;
goal_node.parent = min_dist_node_indx;
goal_node.localcost = min_dist_goal;
goal_node.cost = TREE(goal_node.parent).cost + min_dist_goal;
TREE(node_count+1) = goal_node;

% % Plot the path
end_node = goal_node;
while end_node.parent ~= 0
    start_node = TREE(end_node.parent);
    plot([start_node.coord(1), end_node.coord(1)], ...
           [start_node.coord(2), end_node.coord(2)], 'b', 'LineWidth', 2);
    end_node = start_node;
end
toc;

% Collision Check
function [is_collision] = check_collision(inner_maze_pts, outer_maze_pts, pt)
    % Set the collision flag to 1
    is_collision = 1;
    
    % Check which segment the point is lying
    for i = 1:length(inner_maze_pts)-1
        if pt(1) >= inner_maze_pts(i) && pt(1) < inner_maze_pts(i+1)
            % fprintf("Point lies in the %d segment of the maze\n", i);
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
%         fprintf("Point is in segment %d\n", i);
%         fprintf("val1: %2.6f, val2: %2.6f\n", val1, val2);
    else
%         fprintf("Generated point is in collision\n");
    end
end