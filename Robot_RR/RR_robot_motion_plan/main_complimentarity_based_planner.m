clc
clear
close all

addpath('../RR_supporting_files');
addpath('../pathmexa64');

global links;      global num_dof;
global initial_st; global goal_st;
global center_c1;  global radius1;
global center_c2;  global radius2;
global center_c3;  global radius3;
global tree;       global path_node_ind;
global dt;         global dh;
global tol;        global min_move;

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
max_iter = 25;                % Maximum number of iterations
dt = 1;                       % total input time
dh = 0.01;                    % input velocity interval
vel_mag = 1;
tol = 2e-3;                   % closeness to goal
min_move = 0.1;               % threshold of minimum movement

% ///////////////////////////////////////////////
%             Start of the Algorithm           //
%////////////////////////////////////////////////
dim_ip = 2*num_dof+1;                                         % Dimension of input space
total_dst = norm(goal_st(1:num_dof)-initial_st(1:num_dof));   % heuristic cost

% Initialize Tree with fields
tree.nodes = [initial_st(1:num_dof), initial_st(num_dof+1:end), total_dst];
tree.node_count = 1;
tree.edges = [];              % To do: find the edges 
tree.unexp_ind = [1];
tree.exp_ind = [];

% Start building tree
reached_flag = 0;
num_iter = 0;
ip_vel = vel_mag*[eye(num_dof,num_dof);-eye(num_dof,num_dof);zeros(1,num_dof)];

while (num_iter <= max_iter) && (reached_flag == 0)
    
    % Expand the node based on heuristic cost
    % Add and remove it from the list of expanded 
    % and unexpanded node lists respectively
    if num_iter > 0        
        tree.exp_ind = [tree.exp_ind, temp_ind];
        tree.unexp_ind = [tree.unexp_ind(1:min_ind-1), tree.unexp_ind(min_ind+1:end)];
    else 
        tree.exp_ind = [1];
        tree.unexp_ind = [];
        temp_ind = 1;
    end
    
    % Node to expand
    node_to_expnd = tree.nodes(tree.exp_ind(end),1:num_dof);
    
    % Get velocity potential and add it to velocity list
    v_dir = goal_st(1:num_dof) - node_to_expnd;
    v_dir = v_dir/norm(v_dir);
    ip_vel(end,:) = vel_mag*v_dir;

    % Generate nodes after searching in all directions
    for num_ip = 1:dim_ip
        % Call the local_planner to generate new_node
        new_node = my_local_planner(node_to_expnd, goal_st(1:num_dof), ip_vel(num_ip, :),dt,dh,tol, num_dof);
        new_dist_2_goal = norm(goal_st(1:num_dof)-new_node); % distance from new_node to goal_node
        new_dist_2_exp_node = norm(node_to_expnd-new_node);  % distance from new_node to last expanded node
        if new_dist_2_exp_node > min_move || new_dist_2_goal < tol
              tree.nodes = [tree.nodes; new_node, ip_vel(num_ip, :), new_dist_2_goal];
              tree.node_count = tree.node_count + 1;
              tree.unexp_ind = [tree.unexp_ind, tree.node_count];
              tree.edges = [tree.edges; temp_ind, tree.node_count, new_dist_2_goal];
        end
        % Change flag if reached with given tol
        if new_dist_2_goal < tol
            reached_flag = 1;
            % tree.exp_ind = [tree.exp_ind, temp_ind];   
            tree.exp_ind = [tree.exp_ind, tree.node_count];
            % tree.unexp_ind = [tree.unexp_ind(1:min_ind-1), tree.unexp_ind(min_ind+1:end)];
            tree.unexp_ind = tree.unexp_ind(1:end-1);
            continue;
        end
    end
    
    if reached_flag == 0
        % Find the index of node to be expanded from the list
        [min_dist, min_ind] = min(tree.nodes(tree.unexp_ind, end));
        temp_ind = tree.unexp_ind(min_ind);  % index of min-cost node in overall node list
        fprintf('Minimum distance: %2.6f\n', min_dist);
        fprintf('Number of iterations: %d\n', num_iter);
    
        % Update iteration counter
        num_iter = num_iter + 1;
    else
        fprintf('Minimum distance: %2.6f\n', new_dist_2_goal);
    end
end

%//////////////////////////////////////////
%               Extract Path             //
%//////////////////////////////////////////
% If reached goal, extract the path (search tree bottom up)
current_lvl = length(tree.exp_ind);
start_node_ind = 1;
current_node_ind = tree.exp_ind(current_lvl);
path_node_ind = current_node_ind; % index of nodes in the tree in the path
for i = current_lvl-1:-1:1
    check_node_ind = tree.exp_ind(i);
    % Look for an edge between check_node and current_node 
    find_ind = find(tree.edges(:,1)==check_node_ind);
    if ~isempty(find_ind)
        for j = 1:length(find_ind)
            if tree.edges(find_ind(j),2) == current_node_ind
                path_node_ind = [path_node_ind, check_node_ind];
                current_node_ind = check_node_ind;
                break;
            end
        end
    end
    current_lvl = current_lvl - 1;
end

% Flip path node indices
path_node_ind = fliplr(path_node_ind);

% Visualize the RR arm motion
visualizeRR();
% visualizeRR_paper();
