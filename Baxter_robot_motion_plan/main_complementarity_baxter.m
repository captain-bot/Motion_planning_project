clc
clear
close all

addpath('../manipulator_kinematics/');
addpath('../Baxter_supporting_files/');
addpath('../pathmexa64/');

global g_st0;           global gst0_art;        global g_st0_base;
global wr;              global qr;              global wr_base;
global cir;             global base_mat;        global qr_base;
global safe_dist;       global plot_count_num;  global type_joint;
global num_dof;         global num_contacts;    global array_qo;
plot_count_num = 1;

%//////////////////////////////////////////////
%           Initial and Goal States          //
%//////////////////////////////////////////////
initial_st = [0.2, -0.7, 0.1, 0.1, 0.1, 0.1, 0.1, zeros(1,7)];
goal_st = [0.1, -0.1, 0.1, 0.1, 0.1, 0.1, 0.1, zeros(1,7)];

% /////////////////////////////////////////////
%           Obstacle parameters              //
%//////////////////////////////////////////////
% Obstacle parameters
cir = [0.05,0.6,0.5,0.8;    % [r,x,y,z]
       0.05,0.7,0.5,0.6;
       0.05,0.8,1.2,0.5];
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
% End of robot parameters
            
%/////////////////////////////////////////////
%         Tree building parameters          //
%/////////////////////////////////////////////
max_iter = 50;               % Maximum number of iterations
dt = 1;                   % total input time
dh = 0.01;                   % input velocity interval
vel_magnitude = 0.2;         % velocity magnitude
tol = 1e-3;
min_move = 1e-3;

% ///////////////////////////////////////////////
%             Start of the Algorithm           //
%////////////////////////////////////////////////
dim_ip = 2*num_dof + 1;                                       % Dimension of input space
total_dst = norm(goal_st(1:num_dof)-initial_st(1:num_dof));   % heuristic cost

% Tree Structure
tree.nodes = [initial_st(1:num_dof), initial_st(num_dof+1:end), total_dst];
tree.node_count = 1;
tree.edges = [];              % To do: find the edges
tree.unexp_ind = [1];
tree.exp_ind = [];

% Start building tree
reached_flag = 0;
num_iter = 0;
ip_vel = vel_magnitude*[eye(num_dof,num_dof);-eye(num_dof,num_dof);zeros(1,num_dof)];

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
    
    % Get velocity potential and add it to velocity list
    v_dir = goal_st(1:num_dof) - tree.nodes(tree.exp_ind(end),1:num_dof);
    v_dir = v_dir/norm(v_dir);
    ip_vel(end,:) = vel_magnitude*v_dir;
    
    % Node to expand
    node_to_expnd = tree.nodes(tree.exp_ind(end),1:num_dof);
    
    % Generate nodes after searching in all directions
    for num_ip = 1:dim_ip % qe,qg,vel,tm,tc,err_tol,dof
        % Call the local_planner to generate new_node
        new_node = bax_local_planner(node_to_expnd, goal_st(1:num_dof), ip_vel(num_ip, :),dt,dh,tol);
        new_dist_2_goal = norm(goal_st(1:num_dof)-new_node);
        new_dist_2_exp_node = norm(node_to_expnd-new_node);
        if new_dist_2_exp_node > min_move || new_dist_2_goal < tol
              tree.nodes = [tree.nodes; new_node, ip_vel(num_ip, :), new_dist_2_goal];
              tree.node_count = tree.node_count + 1;
              tree.unexp_ind = [tree.unexp_ind, tree.node_count];
              tree.edges = [tree.edges; temp_ind, tree.node_count, new_dist_2_goal];
        end
        
        % Change flag if reached with given tol
        if new_dist_2_goal < tol
            reached_flag = 1;
            tree.exp_ind = [tree.exp_ind, temp_ind];
            tree.unexp_ind = [tree.unexp_ind(1:min_ind-1), tree.unexp_ind(min_ind+1:end)];
            break;
        end
    end
    
    % Find the index of node to be expanded from the list
    [min_dist, min_ind] = min(tree.nodes(tree.unexp_ind, end));
    temp_ind = tree.unexp_ind(min_ind);     % index of min-cost node in overall node list
    fprintf('Minimum distance: %2.6f\n', min_dist);
    fprintf('Number of iterations: %d\n', num_iter);
    
    % Update iteration counter
    num_iter = num_iter + 1;
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

%////////////////////////////////////////
%        Call visualize Baxter         //
%////////////////////////////////////////
plot_initial_goal(initial_st);  % Plot start state
hold on
plot_initial_goal(goal_st);     % Plot goal state

% Plot the motion
for i = 1:length(path_node_ind)-1
    bax_local_planner(tree.nodes(path_node_ind(i),1:num_dof),goal_st(1:num_dof),tree.nodes(path_node_ind(i+1),num_dof+1:2*num_dof),dt,dh,tol);
%     node2exp = tree.nodes(path_node_ind(i),:);
    for j = 1:size(array_qo,2)    
        visualizeBaxter(array_qo(:,j));
    end
end

%//////////////////////////////////////////////////
%           plot initial and goal states         //
%//////////////////////////////////////////////////
function [] = plot_initial_goal(theta)
    global g_st0; global type_joint; global wr; global qr; global base_mat;

    % Solve direct position kinematics
    [gst, ~] = direct_kin(g_st0, type_joint, wr, qr, theta);

    % Transformation of the end frame
    g_end = base_mat*gst;
    
    % Plot
    plot3(g_end(1,4),g_end(2,4),g_end(3,4),'ko','MarkerSize',12,'MarkerfaceColor','g');
end

%//////////////////////////////////////////////////
%               Visualize the motion             //
%//////////////////////////////////////////////////
function [] = visualizeBaxter(theta)
    global g_st0; global type_joint; global wr; global qr; global base_mat;

    % Solve direct position kinematics
    [gst, transform_upto_joint] = direct_kin(g_st0, type_joint, wr, qr, theta);

    % Transformation of the end frame
    g_end = base_mat*gst;

    % Plot robot
    draw_baxter(base_mat, transform_upto_joint, g_end);
end