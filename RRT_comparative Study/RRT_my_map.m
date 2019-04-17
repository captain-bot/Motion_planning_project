clc
clear
close all

addpath("figures");

% Describe C-Space
X_MAX = 2.0; X_MIN = -1.0; Y_MAX = 7.0; Y_MIN = -1.0;
safe_dist = 0.005;
start = [0.22*pi, 1.94*pi]; goal = [0.5*pi, 0.1*pi];

% Robot details
links = [1, 1];
num_dof = length(links);

% Obstacle details
center_c1 = [0.8, 1.4, 0.5 + safe_dist];
center_c2 = [-0.6, 1.0, 0.4 + safe_dist];
center_c3 = [1.1, 0.1, 0.4 + safe_dist];
obs = [center_c1; center_c2; center_c3];

% Define control parameters
STRIDE = 0.01; dt = 0.1;
MAX_MOVE = 0.2; MAX_ITR = 1000;
BIG = 1000; 
rng(150);

% Build and initiate tree
root_node.coord = start;
root_node.parent = 0;
root_node.cost = 0;
root_node.localcost = 0;

% Add root node to the tree
TREE(1) = root_node;
node_count = 1;

% Grow tree
for i = 1:1:MAX_ITR
    q_rand_collision = 1;
    while q_rand_collision ~= 0
        q_r = [X_MIN + (X_MAX - X_MIN)*rand, Y_MIN + (Y_MAX - Y_MIN)*rand];
        q_rand_collision = collision_check(q_r, links, num_dof, obs);
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
                if collision_check(new_coord, links, num_dof, obs) == 1
                    coord = old_coord;
                    break;
                end
                old_coord = new_coord;
                coord = new_coord;
            else
                if collision_check(old_coord, links, num_dof, obs) == 1
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
    end
end

% Open the free configuration space figure generate before
openfig("free_C_space.fig");
hold on;

% Visualize tree
plot(TREE(1).coord(1), TREE(1).coord(2), 'mo', 'MarkerSize', 10, ...
                                                  'MarkerFaceColor', 'm');
hold on;
plot(goal(1), goal(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
% for i = 2:length(TREE)
%     plot([TREE(TREE(i).parent).coord(1), TREE(i).coord(1)], ...
%               [TREE(TREE(i).parent).coord(2), TREE(i).coord(2)], 'r')
% end

% Find the path out
min_dist_goal = 1000;
for i = 1:length(TREE)
    dist_goal = norm(TREE(i).coord - goal);
    if dist_goal < min_dist_goal
        min_dist_node_indx = i;
        min_dist_goal = dist_goal;
    end
end

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

xlabel('x [m]')
ylabel('y [m]')
title('RRT 2D Implemention')
grid on
% axis('equal')

% Collision check function
function[is_collision] = collision_check(coord, links, num_dof, obs)
    is_collision = 0;
    
    % Solve forward kinematics
    th = coord;
    p_base = [0, 0];
    p_l1 = [links(1)*cos(th(1)), links(1)*sin(th(1))];
    p_l2 = [p_l1(1)+links(2)*cos(th(1)+th(2)), p_l1(2)+links(1)*sin(th(1)+th(2))];
    joint_pts = [p_base; p_l1; p_l2];

    % Check Collision with the obstacles
    for j = 1:num_dof
        line_seg = joint_pts(j:j+1, :);
        for i = 1 : size(obs, 1)
            [closest_dist, closest_pt, is_collision] ...
               = collision_check_line_pt(line_seg, obs(i, 1:2), obs(i, 3));        
            if is_collision == 1
                break;
            end
        end
        if is_collision == 1
            break;
        end
    end
end

function[dist, pt_close, isCollision] = collision_check_line_pt(line_seg, pt, obs_radius)
    isCollision = false;
    w = [line_seg(1, 2) - line_seg(2, 2);
            line_seg(2, 1) - line_seg(1, 1)];
    b =  (line_seg(2, 2) - line_seg(1, 2))*line_seg(1, 1) ...
            - (line_seg(2, 1) - line_seg(1, 1))*line_seg(1, 2);
    dist = pt*w + b;
    pt_close = pt - dist*w';
    if pt_close(1) <= max(line_seg(:,1)) && pt_close(1) >= min(line_seg(:,1)) && pt_close(2) <= max(line_seg(:,2)) && pt_close(2) >= min(line_seg(:,2))
        dist = abs(dist);
    else
        if norm(pt-line_seg(2,:)) >  norm(pt-line_seg(1,:))
            dist = norm(pt-line_seg(1, :));
            pt_close = line_seg(1, :);
        else
            dist = norm(pt-line_seg(2, :));
            pt_close = line_seg(2, :);
        end
    end
    if dist < obs_radius
        isCollision = true;
    end    
end
