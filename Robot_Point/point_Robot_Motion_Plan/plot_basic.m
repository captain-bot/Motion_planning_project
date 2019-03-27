function [] = plot_basic(point)
    global qs;          global qg;
    
    plot(point(1, :), point(2, :));
    hold on
    plot(qs(1), qs(2), 'o', 'MarkerFaceColor', 'red')
    plot(qg(1), qg(2), 'o', 'MarkerFaceColor', 'green')
    grid on
    title('Point robot motion planning with obstacle avoidance')
    xlabel('x [m]')
    ylabel('y [m]')
    axis('equal')

end