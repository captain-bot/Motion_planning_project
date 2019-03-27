function [] = visualizeRR_paper()
    global links;      global num_dof;
    global initial_st; global goal_st;
    global center_c1;  global center_c2;    global center_c3;
    global radius1;    global radius2;      global radius3;
    global tree;       global path_node_ind;
    
    % Close any opened window
    close all;
    
    % Get the list of all expanded nodes
    exp_node_list = tree.nodes(path_node_ind, 1:2*num_dof);
    
    % See the plots in joint space
    figure(1)
    plot(exp_node_list(:,1),exp_node_list(:,2),'-o', 'MarkerFaceColor', 'b', 'LineWidth', 1);
    xlabel('q_1 [rad]')
    ylabel('q_2 [rad]')
    grid on
    
    % Get initial and final end effector configurations in SE(2)
    xi = links(1)*cos(initial_st(1, 1)) + links(2)*cos(initial_st(1, 1)+initial_st(1, 2));
    yi = links(1)*sin(initial_st(1, 1)) + links(2)*sin(initial_st(1, 1)+initial_st(1, 2));

    xg = links(1)*cos(goal_st(1, 1)) + links(2)*cos(goal_st(1, 1)+goal_st(1, 2));
    yg = links(1)*sin(goal_st(1, 1)) + links(2)*sin(goal_st(1, 1)+goal_st(1, 2));
    
    figure(2)
    my_plot1 = plot(xi,yi,'ko','MarkerSize',12,'MarkerFaceColor','r');
    hold on;
    my_plot2 = plot(xg,yg,'ko','MarkerSize',12,'MarkerFaceColor','g');
    obs_plot1 = viscircles([center_c1,center_c2,center_c3]',[radius1,radius2,radius3]);
    hold off;
    
    for plot_i = 1:size(exp_node_list, 1)
        x1 = links(1)*cos(exp_node_list(plot_i, 1));
        y1 = links(1)*sin(exp_node_list(plot_i, 1));
        x2 = x1 + links(2)*cos(exp_node_list(plot_i, 1)+exp_node_list(plot_i, 2));
        y2 = y1 + links(2)*sin(exp_node_list(plot_i, 1)+exp_node_list(plot_i, 2));
        plot_config = line([0,x1,x2],[0,y1,y2]);
    end
    grid on;
    xlim([-0.8,2]);
    ylim([-0.25,2]);
    xlabel('x [m]');
    ylabel('y [m]');
    title('Planer RR robot motion plan')
    legend([my_plot1, my_plot2],{'Start', 'Goal'},'Location','NorthEast');
end