function[dists]= check_closest_dist(th)
    global links;
    global center_c1; global radius1;
    global center_c2; global radius2;
    global center_c3; global radius3;
    
    dists = [];
    
    % Get the line segments for link1
    seg1(1,1) = 0; seg1(2,1) = 0;
    seg1(1,2) = links(1)*cos(th(1)); seg1(2,2) = links(1)*sin(th(1));

    % Get the line segments for link2
    seg2(1,1) = seg1(1,2); seg2(2,1) = seg1(2,2);
    seg2(1,2) = seg2(1,1) + links(2)*cos(th(1) + th(2)); seg2(2,2) = seg2(2,1) + links(2)*sin(th(1) + th(2));

    % Compute distance from obstacle to link1
    [link1_c1_cp, dist_link1_c1]=circle_line_distance(center_c1, radius1, seg1);
    [link1_c2_cp, dist_link1_c2]=circle_line_distance(center_c2, radius2, seg1);
    [link1_c3_cp, dist_link1_c3]=circle_line_distance(center_c3, radius3, seg1);
    dists(1) = min([dist_link1_c1, dist_link1_c2, dist_link1_c3]);

    % Compute distance from obstacle to link2
    [link2_c1_cp, dist_link2_c1]=circle_line_distance(center_c1, radius1, seg2);
    [link2_c2_cp, dist_link2_c2]=circle_line_distance(center_c2, radius2, seg2);
    [link2_c3_cp, dist_link2_c3]=circle_line_distance(center_c3, radius3, seg2);
    dists(2) = min([dist_link2_c1, dist_link2_c2, dist_link2_c3]);
end