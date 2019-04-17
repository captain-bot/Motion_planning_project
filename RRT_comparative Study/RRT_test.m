clc
clear
close all

% Describe C-Space
X_MAX = 10; X_MIN = 0; Y_MAX = 10; Y_MIN = 0;
start = [0, 0]; goal = [10, 10];
obstacle_origin = [5, 5; 2, 2];
obstacle_height = 1; obstacle_width = 1;

% Define control parameters
STRIDE = 0.4;
MAX_MOVE = 0.2; MAX_ITR = 5000;
BIG = 1000; rng(100);

% Build and initiate tree
root_node.coord = start;
root_node.parent = 0;
root_node.cost = 0;

% Add root node to the tree
TREE(1) = root_node;
node_count = 1;

% Grow tree
for i = 1:1:MAX_ITR
    
    % Generate q_rand
    q_rand.x = X_MIN + (X_MAX - X_MIN)*rand;
    q_rand.y = Y_MIN + (Y_MAX - Y_MIN)*rand;
    
    % Find nearest node in the tree that is closest to q_rand
    dist_min = BIG;
    for j = 1:length(TREE)
        dist_current = sqrt((TREE(j).coord(1) - q_rand.x)^2 + (TREE(j).coord(2) - q_rand.y)^2);
        if dist_current < dist_min
            dist_min_idx = j;
            dist_min = dist_current;
        end
    end
    
    % Steer from q_nearest toward q_rand
    DIST = sqrt((q_rand.x - TREE(dist_min_idx).coord(1))^2 + (q_rand.y - TREE(dist_min_idx).coord(2))^2);
    if DIST > STRIDE
        if STRIDE * (q_rand.x - TREE(dist_min_idx).coord(1)) < MAX_MOVE && STRIDE * (q_rand.x - TREE(dist_min_idx).coord(2)) < MAX_MOVE
            coord(1) = TREE(dist_min_idx).coord(1) + STRIDE * (q_rand.x - TREE(dist_min_idx).coord(1));
            coord(2) = TREE(dist_min_idx).coord(2) + STRIDE * (q_rand.y - TREE(dist_min_idx).coord(2));
        else
            coord(1) = TREE(dist_min_idx).coord(1) + 0.1 * STRIDE * (q_rand.x - TREE(dist_min_idx).coord(1));
            coord(2) = TREE(dist_min_idx).coord(2) + 0.1 * STRIDE * (q_rand.y - TREE(dist_min_idx).coord(2));
        end
    else
        coord(1) = q_rand.x; coord(2) = q_rand.y;
    end
    
    % Check collision

    % Construct q_new
    q_new.coord = coord;
    q_new.parent = dist_min_idx;
    q_new.cost = sqrt((q_new.coord(1) - TREE(dist_min_idx).coord(1))^2 + (q_new.coord(2) - TREE(dist_min_idx).coord(2))^2);
    
    % Add q_new to teh tree
    TREE(node_count+1) = q_new;
    node_count = node_count + 1;
    
end

% Visualize tree
plot(TREE(1).coord(1), TREE(1).coord(2), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
hold on;
plot(goal(1), goal(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
for i = 2:length(TREE)
    plot([TREE(TREE(i).parent).coord(1), TREE(i).coord(1)], [TREE(TREE(i).parent).coord(2), TREE(i).coord(2)], 'r')
end

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
goal_node.cost = TREE(goal_node.parent).cost + min_dist_goal;
TREE(node_count) = goal_node;

end_node = goal_node;
while end_node.parent ~= 0
    start_node = TREE(end_node.parent);
    plot([start_node.coord(1), end_node.coord(1)], [start_node.coord(2), end_node.coord(2)], 'b', 'LineWidth', 2);
    end_node = start_node;
end

xlabel('x [m]')
ylabel('y [m]')
title('RRT 2D Implemention')
grid on
axis('square')
