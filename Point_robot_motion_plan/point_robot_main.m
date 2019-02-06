clc
clear
close all

% Make simulation parameters global
global contact_normal; global closeset_dist;   global safe_dist;  global num_of_contact;
global ln_segs;        global contact_normal_array; global ln_bias_array;
global h;              global kp;              global v;
global q_o;            global mid_pt;          global dof;   global qg; global qs;

% Make maze parameters global
global maze_height; global maze_length; global maze_thick; global gap;

% Add PATH solver to matlab's search path
addpath('../pathmexa64'); addpath('../Point_robot_supporting_files');

% Define parameters
num_of_contact = 1; safe_dist = 0.02; dof = 2; h = 0.02; kp = 2; tf = 10;

% Define start and goal configuration
qs = [0.5; 0.9]; qg = [5.5; 0.5];

% define maze parameters
maze_height = 1; maze_length = 5; maze_thick = 0.1; gap = 0.5;
[ln_segs, contact_normal_array, ln_bias_array] = surf_norm();

% Draw the maze
drawmaze(); axis('equal');

% ///////////////////////////////////
%        Start of PATH solver      //
%////////////////////////////////////
num_unknown = dof + num_of_contact;
for i = 1:num_unknown
    l(i) = -Inf;     u(i) = Inf;
    if i == num_unknown
        l(i) = 0;
    end
end
t = 0; q_o = qs; z = zeros(num_unknown, 1); 
qo_array = []; ip_array = []; total_tm = 0; comp_vel_array = [];
while t < tf
    
    dist_qg = (qg - q_o);                          % current error 
    % Termination criterion
    fprintf('dist_goal: %2.4f\n', norm(dist_qg));
    if dist_qg <= 1e-1
        fprintf("Time taken: %2.4f\n", t);
%         total_tm = t;
%         t = tf;
%         continue;
        break;
    end
    v = kp*(dist_qg/norm(dist_qg));                % input vel towards goal
    
    % Compute collision information
    [contact_normal, closeset_dist] = get_colli_info(q_o);
    
    % Solve complementarity problem    
    [z,f,J] = pathmcp(z,l,u,'mcpfuncjacEval');
    fprintf('Collision-free node: \n'); 
    disp(z); fprintf('--------------\n');
    
    % Update
    q_o = z(1:dof);
    qo_array = [qo_array, q_o];
    ip_array = [ip_array, v];
    comp_vel_array = [comp_vel_array, z(end)*contact_normal];
    t = t + h;
    
    % Visualize
    visualize_points(q_o, mid_pt, contact_normal);
    
end

% drawmaze();
h3 = plot(qo_array(1, :), qo_array(2, :), 'r--');
m1 = plot(qs(1), qs(2), 'o', 'MarkerFaceColor', 'r');
m2 = plot(qg(1), qg(2), 'o', 'MarkerFaceColor', 'g');
legend([h3, m1, m2],{'configuration trajectory', 'Start', 'Goal'},'Location','northwest','NumColumns', 3)

figure(2)
time = 0:h:t;
subplot(2, 1, 1)
plot(time, ip_array(1,:), time, ip_array(2,:))
gird on
subplot(2, 1, 2)
plot(time, comp_vel_array(1,:), time, comp_vel_array(2,:))
grid on