function [Mov] = draw_baxter(base_mat, transform_upto_joint, gend)
    global gst0_art
    global cir;  global plot_count_num;
    global hfig; global filename;

    % Transformation upto joints w.r.t. actual base
    plot_transform = zeros(4,4,size(transform_upto_joint,3));
    xcoord = zeros(1, size(transform_upto_joint,3));
    ycoord = zeros(1, size(transform_upto_joint,3));
    zcoord = zeros(1, size(transform_upto_joint,3));
    for i = 1:size(transform_upto_joint,3)-1
        plot_transform(:,:,i) = base_mat*transform_upto_joint(:,:,i)*gst0_art(:,:,i);
        xcoord(1,i) = plot_transform(1,4,i);
        ycoord(1,i) = plot_transform(2,4,i);
        zcoord(1,i) = plot_transform(3,4,i);
    end
    plot_transform(:,:, end) = gend;
    xcoord(1,end) = plot_transform(1,4,end);
    ycoord(1,end) = plot_transform(2,4,end);
    zcoord(1,end) = plot_transform(3,4,end);
    
    % Plot the points
    plot_hand1 = plot3(xcoord,ycoord,zcoord, 'ko', 'MarkerFaceColor', 'r');
    hold on
    plot_hand2 = plot3(xcoord,ycoord,zcoord,'b', 'LineWidth', 2.5);
    grid on;
    xlim([0 1.2]);
    ylim([0.2 1.5]);
    zlim([-0.5 1]);
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title('Baxter Left Arm Motion Planning')
    % view(-81,24);
%     view(-112,22);
    
    if plot_count_num == 1
        % Draw obstacles
        [x,y,z] = sphere;
        for i = 1:size(cir,1)
            plot_hand3 = surf(x*cir(i,1)+cir(i,2), y*cir(i,1)+cir(i,3), z*cir(i,1)+cir(i,4));
            shading interp;
        end
    end
       
    drawnow;
%     pause(0.05);
    
%     %/////////////////////////////////////////////////////////
%         % Get the plot in GIF file 
%         frame = getframe(hfig); 
%         im = frame2im(frame); 
%         [imind,cm] = rgb2ind(im,256);
% 
%         % Write to the GIF File 
%         if plot_count_num == 1
%             imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
%         else 
%             imwrite(imind,cm,filename,'gif','WriteMode','append'); 
%         end
%     %////////////////////////////////////////////////

    Mov=getframe(gcf);
    
    %delete(plot_hand1);
    %delete(plot_hand2);
%     delete(plot_hand3);
    plot_count_num = plot_count_num + 1;
end