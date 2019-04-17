clc
clear
close all

% Joint angles
% th = [0.5, 0.8];
% th = [0.5, 0.5];
% th = [1.4, 2];
% th = [0.6, 5.0];
th = [0.75, 4.5];
initial_st = [0.22*pi, 1.94*pi];
goal_st = [0.5*pi, 0.1*pi];

% Robot details
links = [1, 1];
num_dof = length(links);

% Obstacle details
center_c1 = [0.8, 1.4, 0.5];
center_c2 = [-0.6, 1.0, 0.4];
center_c3 = [1.1, 0.1, 0.4];
obs = [center_c1; center_c2; center_c3];

% Solve forward kinematics
p_base = [0, 0];
p_l1 = [links(1)*cos(th(1)), links(1)*sin(th(1))];
p_l2 = [p_l1(1)+links(2)*cos(th(1)+th(2)), p_l1(2)+links(1)*sin(th(1)+th(2))];
joint_pts = [p_base; p_l1; p_l2];

% Check Collision with the obstacles
for j = 1:num_dof
    line_seg = joint_pts(j:j+1, :);
    for i = 1 : size(obs, 1)
        [closest_dist, closest_pt, is_collision] = collision_check(line_seg, obs(i, 1:2), obs(i, 3));        
        fprintf("Dist of link %d from obstacle %d is %2.4f\n", j, i, closest_dist);
        fprintf("Closest point: x=%2.6f, y=%2.6f\n", closest_pt(1), closest_pt(2));
        fprintf("In Collision: %d\n", is_collision);
    end
    fprintf("----------------\n");
end

% Visualize
% Plot obstacles
% viscircles(obs(:, 1:2), obs(:, 3));
for i = 1:length(obs)
    c = 'r';
    plot_circle(obs(i, 1), obs(i, 2), obs(i, 3), c)
end

% Plot arms
xlim([-0.8,2]);
ylim([-0.25,2]);
grid on;
plot_config = line(joint_pts(:, 1), joint_pts(:, 2), 'LineWidth', 2);
xlabel('x [m]');
ylabel('y [m]');
title('Planer RR robot motion plan')

function[dist, pt_close, isCollision] = collision_check(line_seg, pt, obs_radius)
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

% Plot circles function
function cir = plot_circle(x,y,r,c)
    hold on
    th = 0:pi/50:2*pi;
    x_circle = r * cos(th) + x;
    y_circle = r * sin(th) + y;
    cir = plot(x_circle, y_circle);
    fill(x_circle, y_circle, c)
    hold off
    axis equal
end