clc
clear
close all

% Define the robot parameters
links = [1,1];
num_dof = length(links);

% Initial and goal states
initial_st = [0.22*pi, 1.9*pi, 0, 0];
goal_st = [0.5*pi, 0.1*pi, 0, 0];

% Tree building parameters
max_iter = 250;                % Maximum number of iterations
dt = 0.25;                     % total input time
dh = 0.01;                     % input velocity interval
vel_magnitude = 0.1;           % velocity magnitude
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
        new_node = local_planner(exp_node, goal_st(1:num_dof), ip_vel(num_ip, :),dt,dh,tol);
        edges = [edges; exp_node, new_node];
        new_dist_2_goal = norm(goal_st(1:num_dof)-new_node);
        new_dist_2_exp_node = norm(exp_node-new_node);
        if new_dist_2_exp_node > min_move || new_dist_2_goal < tol
            node_unexpanded = [node_unexpanded; new_dist_2_goal, new_node, ip_vel(num_ip, :)];
        end
        if new_dist_2_goal < tol           
            reached_flag = 1;
        end
    end
    
    % Find the index of node to be expanded from the list
    [min_dist, exp_node_idx] = min(node_unexpanded(:, 1));
    fprintf('Minimum distance: %2.6f\n', min_dist);
    
    if reached_flag == 1
        exp_node_list = [exp_node_list; node_unexpanded(exp_node_idx, :)];
    end
    
    % Update iteration counter
    num_iter = num_iter + 1;
end
% Add the final node with zero distance
exp_node_list = [exp_node_list; 0, goal_st];

%///////////////////////////////////
%     Plot the expanded nodes     //
%///////////////////////////////////
% Get initial and fial end effector configurations in SE(2)
xi = links(1)*cos(exp_node_list(1, 2)) + links(2)*cos(exp_node_list(1, 2)+exp_node_list(1, 3));
yi = links(1)*sin(exp_node_list(1, 2)) + links(2)*sin(exp_node_list(1, 2)+exp_node_list(1, 3));

xg = links(1)*cos(exp_node_list(end, 2)) + links(2)*cos(exp_node_list(end, 2)+exp_node_list(end, 3));
yg = links(1)*sin(exp_node_list(end, 2)) + links(2)*sin(exp_node_list(end, 2)+exp_node_list(end, 3));
my_plot1 = plot(xi,yi,'ko','MarkerSize',12,'MarkerFaceColor','r');
hold on;
my_plot2 = plot(xg,yg,'ko','MarkerSize',12,'MarkerFaceColor','g');
hold off;
for plot_i = 1:size(exp_node_list, 1)
    x1 = links(1)*cos(exp_node_list(plot_i, 2));
    y1 = links(1)*sin(exp_node_list(plot_i, 2));
    x2 = x1 + links(2)*cos(exp_node_list(plot_i, 2)+exp_node_list(plot_i, 3));
    y2 = y1 + links(2)*sin(exp_node_list(plot_i, 2)+exp_node_list(plot_i, 3));
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
        pause(0.1);
%         delete (findobj('Type', 'line'));
    end
end

% Dumpest local planner
function [q_new] = local_planner(qs,qg,vel,time_to_expd,time_impulse,err_tol)
    t = 0;
    q_new = qs;
    while t < time_to_expd
        q_new = q_new + time_impulse*vel;
        t = t + time_impulse;
    end
end




