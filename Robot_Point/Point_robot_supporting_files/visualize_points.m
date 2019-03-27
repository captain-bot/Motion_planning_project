function [] = visualize_points(q_o, mid_point, contact_normal)
    global qg;
    
    drawnow limitrate;
    hold on
    h1 = plot(q_o(1), q_o(2), '.', 'MarkerSize', 10);
    h2 = quiver(mid_point(1), mid_point(2), contact_normal(1), contact_normal(2));
    
    xlim([0, qg(1)+0.5]);
    ylim([0, 1.5]);
        
    drawnow;        
    delete(h1);
    delete(h2);
end