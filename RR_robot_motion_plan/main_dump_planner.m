clc
clear
close all

% Define the robot parameters
links = [1,1];
num_dof = length(links);

% Initial and goal states
initial_st = [0.22*pi, 1.94*pi, 0, 0];
goal_st = [0.5*pi, 0.1*pi, 0, 0];

% Tree building parameters
max_iter = 250;               % Maximum number of iterations
dt = 0.25;                       % total input time
dh = 0.01;                    % input velocity interval
vel_magnitude = 0.1;            % velocity magnitude
tol = 1.5e-2;
min_move = 1e-3;

% ///////////////////////////////////////////////
%             Start of the Algorithm           //
%////////////////////////////////////////////////
dim_ip = 2*num_dof + 1;                                         % Dimension of input space
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
    for num_ip = 1:dim_ip
        % Call the local_planner to generate new_node
        new_node = dump_local_planner(node_to_expnd, goal_st(1:num_dof), ip_vel(num_ip, :),dt,dh,tol);
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

%////////////////////////////////////////////////
%             Plot the expanded nodes          //
%////////////////////////////////////////////////
% Get the list of all expanded nodes
exp_node_list = tree.nodes(tree.exp_ind, 1:num_dof);

% Get initial and final end effector configurations in SE(2)
xi = links(1)*cos(initial_st(1, 1)) + links(2)*cos(initial_st(1, 1)+initial_st(1, 2));
yi = links(1)*sin(initial_st(1, 1)) + links(2)*sin(initial_st(1, 1)+initial_st(1, 2));

xg = links(1)*cos(goal_st(end, 1)) + links(2)*cos(goal_st(end, 1)+goal_st(end, 2));
yg = links(1)*sin(goal_st(end, 1)) + links(2)*sin(goal_st(end, 1)+goal_st(end, 2));

my_plot1 = plot(xi,yi,'ko','MarkerSize',12,'MarkerFaceColor','r');
hold on;
my_plot2 = plot(xg,yg,'ko','MarkerSize',12,'MarkerFaceColor','g');
hold off;

for plot_i = 1:size(exp_node_list, 1)
    x1 = links(1)*cos(exp_node_list(plot_i, 1));
    y1 = links(1)*sin(exp_node_list(plot_i, 1));
    x2 = x1 + links(2)*cos(exp_node_list(plot_i, 1)+exp_node_list(plot_i, 2));
    y2 = y1 + links(2)*sin(exp_node_list(plot_i, 1)+exp_node_list(plot_i, 2));
    if (plot_i == 1) || (mod(plot_i, 10) == 0) || (plot_i == size(exp_node_list,1))
        xlim([-0.8,2]);
        ylim([-0.25,2]);
        grid on;
        line([0,x1,x2],[0,y1,y2]);
        xlabel('x [m]');
        ylabel('y [m]');
        title('Planer RR robot motion plan')
        legend([my_plot1, my_plot2],{'Start', 'Goal'},'Location','NorthEast');
        drawnow;
        pause(0.01);
%         delete (obs_plot1);
%         delete (findobj('Type', 'line'));
    end
end

% Dumpest local planner
function [q_new] = dump_local_planner(qs,qg,vel,time_to_expd,time_impulse,err_tol)
    t = 0;
    q_new = qs;
    while t < time_to_expd
        q_new = q_new + time_impulse*vel;
        t = t + time_impulse;
    end
end