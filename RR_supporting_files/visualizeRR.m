function [] = visualizeRR()
    global links;      global num_dof;
    global initial_st; global goal_st;
    global center_c1;  global center_c2;    global center_c3;
    global radius1;    global radius2;      global radius3;
    global tree;       global path_node_ind;
    global dt;         global dh;
    global tol;        global array_qo;
    
    % Close any opened window
    close all;
    
    % GIF file name
    filename = 'testFile';
    
    % Get the figure handle
    h = figure;
    
    % Get the list of all expanded nodes
    exp_node_list = tree.nodes(path_node_ind, 1:2*num_dof);
    
    % See the plots in joint space
    plot(exp_node_list(:,1),exp_node_list(:,2),'*');
    
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

    count_iter = 0;
    for plot_i = 1:size(exp_node_list, 1)-1
        my_local_planner(exp_node_list(plot_i, 1:num_dof),goal_st(1:num_dof),exp_node_list(plot_i+1, num_dof+1:2*num_dof),dt,dh,tol,num_dof);
        for j = 1:size(array_qo,2)
            if mod(j,5)==0
                x1 = links(1)*cos(array_qo(1,j));
                y1 = links(1)*sin(array_qo(1,j));
                x2 = x1 + links(2)*cos(array_qo(1,j)+array_qo(2,j));
                y2 = y1 + links(2)*sin(array_qo(1,j)+array_qo(2,j));
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

                % Get the plot in GIF file 
                frame = getframe(h); 
                im = frame2im(frame); 
                [imind,cm] = rgb2ind(im,256);

                % Write to the GIF File 
                if count_iter == 0 
                    imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
                else 
                    imwrite(imind,cm,filename,'gif','WriteMode','append'); 
                end
                count_iter = count_iter + 1;
                delete (plot_config);
            end
        end
    end
end