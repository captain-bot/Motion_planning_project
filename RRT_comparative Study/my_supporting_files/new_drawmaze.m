function[inner_maze, outer_maze] = new_drawmaze()
    
    global maze_height;    global maze_length;    global qs; 
    global maze_thick;     global maze_gap;            global qg;
    
    x1 = maze_gap:maze_gap:maze_length-maze_gap;
    x2 = maze_gap:maze_gap:maze_length-maze_gap;   
    
    y1 = zeros(size(x1));
    y2 = zeros(size(x2));
    
    for i = 1:length(x1)
        if (floor(x1(i))==x1(i))
            y1(i) = 0;
        else
            y1(i) = maze_height - maze_thick;
        end
    end
    
    plot (x1,y1)
    hold on
    
    for i = 1:length(x2)
       if (floor(x2(i))==x2(i))
            y2(i)= maze_thick;            
       else
            y2(i)=maze_height;
       end
    end
    
    inner_maze = [x1' y1'];
    outer_maze = [x2' y2'];

    plot (x2,y2)
    m1 = plot(qs(1), qs(2), 'o', 'MarkerFaceColor', 'r');
    m2 = plot(qg(1), qg(2), 'o', 'MarkerFaceColor', 'g');
    legend([m1, m2], {'Start', 'Goal'}, 'Location', 'NorthEast');
    xlabel('q_1')
    ylabel('q_2')
    title('Maze')
    grid on
    
end
