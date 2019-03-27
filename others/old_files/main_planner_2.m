clc
clear
close all

global links;
global center_c1; global radius1;
global center_c2; global radius2;
global center_c3; global radius3;
global exp_node_list;

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

% Tree building parameters
max_iter = 20;               % Maximum number of iterations
dt = 2;                       % total input time
dh = 0.01;                    % input velocity interval
vel_magnitude = 3;          % velocity magnitude
tol = 1.5e-2;
min_move = 1e-3;

% ///////////////////////////////////////////////
%             Start of the Algorithm           //
%////////////////////////////////////////////////
dim_ip = 2*num_dof+1;                                         % Dimension of input space
total_dst = norm(goal_st(1:num_dof)-initial_st(1:num_dof));   % heuristic cost
build_tree = [initial_st(1:num_dof)];                         % strat building tree
node_unexpanded = [total_dst, initial_st(1:num_dof),...
                              initial_st(num_dof+1:end)];     % unexpanded nodes
exp_node_idx = 1;
exp_node_list = [];
edges = [];

% Start building tree
reached_flag = 0;
num_iter = 0;
ip_vel = vel_magnitude*[eye(num_dof,num_dof);-eye(num_dof,num_dof);zeros(1,num_dof)];

while (num_iter <= max_iter) && (reached_flag == 0)
    % Node to expand from the list of unexpanded nodes
    exp_node = [node_unexpanded(exp_node_idx, 2:num_dof+1)]; % node to expand now
    exp_node_list = [exp_node_list; node_unexpanded(exp_node_idx, :)];
    
    % Remove just expanded node from the list of unexpanded nodes
    if exp_node_idx == 1
        node_unexpanded = node_unexpanded(2:end, :);
    elseif exp_node_idx == size(node_unexpanded, 1)
        node_unexpanded = node_unexpanded(1:end-1, :);
    else
        node_unexpanded = [node_unexpanded(1:exp_node_idx-1, :); node_unexpanded(1:exp_node_idx+1, :)];
    end
    
    % Get velocity potential and add it to velocity list
    v_dir = goal_st(1:num_dof) - exp_node(1, :);
    v_dir = v_dir/norm(v_dir);
    ip_vel(end,:) = vel_magnitude*v_dir;
    
    % Generate nodes after searching in all directions
    for num_ip = 1:dim_ip
        new_node = my_local_planner(exp_node, goal_st(1:num_dof), ip_vel(num_ip, :),dt,dh,tol, num_dof);
%         edges = [edges; exp_node, new_node];
        new_dist_2_goal = norm(goal_st(1:num_dof)-new_node);
        new_dist_2_exp_node = norm(exp_node-new_node);
        if new_dist_2_exp_node > min_move || new_dist_2_goal < tol
            node_unexpanded = [node_unexpanded; new_dist_2_goal, new_node, ip_vel(num_ip, :)];
        end
        if new_dist_2_goal < tol           
            reached_flag = 1;
        end
    end
    
    % Limit the list of unexpanded node (Turn it off!!)
    if (size(node_unexpanded,1) > 100)
        node_unexpanded = node_unexpanded(size(node_unexpanded,1)-100:size(node_unexpanded,1), :);
    end
    
    % Find the index of node to be expanded from the list
    [min_dist, exp_node_idx] = min(node_unexpanded(:, 1));
    fprintf('Minimum distance: %2.6f\n', min_dist);
    fprintf('Number of iterations: %d\n', num_iter);
    
    if reached_flag == 1
        exp_node_list = [exp_node_list; node_unexpanded(exp_node_idx, :)];
    end
    
    % Update iteration counter
    num_iter = num_iter + 1;
end
% Add the final node with zero distance
exp_node_list = [exp_node_list; 0, goal_st];

% Plot the result
visualize();