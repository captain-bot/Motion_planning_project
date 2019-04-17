clc; clear; close all;
addpath('../../pathmexa64');

% Make simulation parameters global
global contact_normal; global closeset_dist;  global safe_dist;
global dof;            global num_of_contact; global mid_pt;
global q_o;            global qs;             global qg;
global h;              global kp;             global v;

% Define parameters
dof = 2; num_of_contact = 1; h = 0.01; kp = 1; safe_dist = 0.0; Tf = 3;
% Define start and goal configuration
qs = [-0.25; 1]; qg = [0.7; 0.1];
% Define parameters of the half-line obstacle
% ln_seg = [0, 0.5; 0, 1];
ln_seg = [0, 0.5; 1.0, 0];
temp = (ln_seg(:, 2) - ln_seg(:, 1))/norm(ln_seg(:, 2) - ln_seg(:, 1));
ln_bias = ln_seg(1,1)*temp(2) - ln_seg(2,1)*temp(1);
contact_normal = [-temp(2); temp(1)];
mid_pt = (ln_seg(:,1) + ln_seg(:, 2))/2;
plot_basic(ln_seg);                               % visualize

contact_normal = -contact_normal;
ln_bias = - ln_bias;

% Start of motion planning
num_unknown = dof + num_of_contact;
for i = 1:num_unknown                             % Set bounds of unknowns
    l(i) = -Inf; u(i) = Inf;
    if i == num_unknown
        l(i) = 0;
    end
end
q_o = qs;
z = zeros(dof+num_of_contact, 1);
z(1:2)=qs;
t = 0;
while t < Tf
    dist_qg = (qg - q_o);                          % current error 
    % Termination criterion
    fprintf('dist_goal: %2.4f\n', norm(dist_qg));
    if (norm(dist_qg) <= 1e-2)
        t = Tf;
    end
    v = kp*(dist_qg/norm(dist_qg));                % input vel towards goal
    closeset_dist = contact_normal'*q_o + ln_bias; % current obstacle distance
    [z,f,J] = pathmcp(z,l,u,'mcpfuncjacEval'); z   % solve EOM
    q_o = z(1:dof, 1);                             % current state
    t = t + h;                                     % update time
    plot_current();                                % Visualize
end