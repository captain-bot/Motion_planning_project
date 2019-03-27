function [] = visualize()
    global links; global num_dof;
    global initial_st; global goal_st;
    global center_c1; global center_c2; global center_c3;
    global radius1; global radius2; global radius3;
    global tree; global path_node_ind;
    
    % GIF file name
    filename = 'testFile';
    
    % Get the figure handle
    h = figure;
    
    % Get the list of all expanded nodes
%     exp_node_list = tree.nodes(tree.exp_ind, 1:num_dof);
    exp_node_list = tree.nodes(path_node_ind, 1:num_dof);
    
    % Get initial and final end effector configurations in SE(2)
    xi = links(1)*cos(initial_st(1, 1)) + links(2)*cos(initial_st(1, 1)+initial_st(1, 2));
    yi = links(1)*sin(initial_st(1, 1)) + links(2)*sin(initial_st(1, 1)+initial_st(1, 2));

    xg = links(1)*cos(goal_st(1, 1)) + links(2)*cos(goal_st(1, 1)+goal_st(1, 2));
    yg = links(1)*sin(goal_st(1, 1)) + links(2)*sin(goal_st(1, 1)+goal_st(1, 2));

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
%         if (plot_i == 1) || (mod(plot_i, 5) == 0) || (plot_i == size(exp_node_list,1))
        xlim([-0.8,2]);
        ylim([-0.25,2]);
        grid on;
        plot_config = line([0,x1,x2],[0,y1,y2]);
        xlabel('x [m]');
        ylabel('y [m]');
        title('Planer RR robot motion plan')
        legend([my_plot1, my_plot2],{'Start', 'Goal'},'Location','NorthEast');
        drawnow;
        pause(0.01);
%           delete (obs_plot1);
%           delete (plot_config);
%           delete (findobj('Type', 'line'));
%         end

%           % Get the plot in GIF file 
%           frame = getframe(h); 
%           im = frame2im(frame); 
%           [imind,cm] = rgb2ind(im,256);
% 
%           % Write to the GIF File 
%           if plot_i == 1 
%               imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
%           else 
%               imwrite(imind,cm,filename,'gif','WriteMode','append'); 
%           end
%           
%           delete (plot_config);
    end
end