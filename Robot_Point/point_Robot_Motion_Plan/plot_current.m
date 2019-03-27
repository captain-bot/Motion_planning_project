function [] = plot_current()
    global mid_pt;    global contact_normal;    global q_o;
    
    drawnow;
    h1 = quiver(mid_pt(1), mid_pt(2), 0.5*contact_normal(1), 0.5*contact_normal(2), 'm');
    h2 = plot(q_o(1), q_o(2), '.');
    xlim([-0.5, 1])
    ylim([-0.5, 1.5])
    drawnow;
    pause(0.1);  
    delete(h1);
    delete(h2);
    
end