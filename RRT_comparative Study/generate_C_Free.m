clc
clear
close all

% Robot details
links = [1, 1];
num_dof = length(links);

% Safe distance
d_safe = 0.005;

% Obstacle details
center_c1 = [0.8, 1.4, 0.5];
center_c2 = [-0.6, 1.0, 0.4];
center_c3 = [1.1, 0.1, 0.4];
obs = [center_c1; center_c2; center_c3];

% Define ranges for theta_1 and theta_2
th1 = 0.4:0.01:2;
th2 = 0:0.01:2*pi;
x_invalid = [];
y_invalid = [];
for x=th1
    for y=th2
        is_collision = 0;
        th = [x, y];
        p_base = [0, 0];
        p_l1 = [links(1)*cos(th(1)), links(1)*sin(th(1))];
        p_l2 = [p_l1(1)+links(2)*cos(th(1)+th(2)), p_l1(2)+links(1)*sin(th(1)+th(2))];
        joint_pts = [p_base; p_l1; p_l2];
        for j = 1:num_dof
            line_seg = joint_pts(j:j+1, :);
            for i = 1 : size(obs, 1)
                [closest_dist, closest_pt, is_collision] = collision_check(line_seg, obs(i, 1:2), obs(i, 3) + d_safe);        
                if is_collision == 1
                    break;
                end
            end
            if is_collision == 1
                break;
            end
        end
        if is_collision == 1
            x_invalid = [x_invalid, x];
            y_invalid = [y_invalid, y];
        end
    end
end

plot(x_invalid, y_invalid, '.')
grid on;
xlabel('q_1')
ylabel('q_2')


% Helper function
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