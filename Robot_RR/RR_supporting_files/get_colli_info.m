function[contact_wrench,dista,l1contact,l2contact]=get_colli_info(th)
    global links;
    global center_c1; global radius1;
    global center_c2; global radius2;
    global center_c3; global radius3;
    
    % Define contact wrench vector
    contact_wrench = zeros(2,2);
    
    % Get the line segments
    seg1(1,1) = 0; seg1(2,1) = 0;
    seg1(1,2) = links(1)*cos(th(1)); seg1(2,2) = links(1)*sin(th(1));
    seg2(1,1) = seg1(1,2); seg2(2,1) = seg1(2,2);
    seg2(1,2) = seg2(1,1) + links(2)*cos(th(1) + th(2)); seg2(2,2) = seg2(2,1) + links(2)*sin(th(1) + th(2));
    
    % Compute distance from circles to links 
    [link1_c1_cp, dist_link1_c1]=circle_line_distance(center_c1, radius1, seg1);
    [link1_c2_cp, dist_link1_c2]=circle_line_distance(center_c2, radius2, seg1);
    [link1_c3_cp, dist_link1_c3]=circle_line_distance(center_c3, radius3, seg1);


    l1c=[links(1)*cos(th(1,1));links(1)*sin(th(1,1))];
    
    if(dist_link1_c1 <= dist_link1_c2)
        dist_link1_obs = dist_link1_c1;
        radius = radius1;
        center_obs = center_c1;
        link1_cp = link1_c1_cp;
        l1contact=links(1)-sqrt((link1_cp(1,2)-l1c(1,1))^2 + (link1_cp(2,2)-l1c(2,1))^2);
    else
        dist_link1_obs = dist_link1_c2;
        radius = radius2;
        center_obs = center_c2;
        link1_cp = link1_c2_cp;
        l1contact=links(1)-sqrt((link1_cp(1,2)-l1c(1,1))^2 + (link1_cp(2,2)-l1c(2,1))^2);
    end
    if(dist_link1_c3 < dist_link1_obs)
        dist_link1_obs = dist_link1_c3;
        radius = radius3;
        center_obs = center_c3;
        link1_cp = link1_c3_cp;
        l1contact=links(1)-sqrt((link1_cp(1,2)-l1c(1,1))^2 + (link1_cp(2,2)-l1c(2,1))^2);
    end

    % jacobian for contact point of bar 1 at beginning of current step
    Jc11_0 = -l1contact*sin(th(1));
    Jc12_0 =0;
    Jc21_0 = l1contact*cos(th(1));
    Jc22_0 = 0;
    Jcon0= [Jc11_0,Jc12_0;Jc21_0,Jc22_0];

    contact_dir = (link1_cp(:,1) - center_obs)/radius;
    vec_cp_joint = (link1_cp(:,2) - seg1(:,1));
    contact_wrench(1,1) = -(link1_cp(1,1)-link1_cp(1,2))/sqrt((link1_cp(1,1)-link1_cp(1,2))^2 + (link1_cp(2,1)-link1_cp(2,2))^2);
    contact_wrench(2,1) = -(link1_cp(2,1)-link1_cp(2,2))/sqrt((link1_cp(1,1)-link1_cp(1,2))^2 + (link1_cp(2,1)-link1_cp(2,2))^2);

    dista(1) = sqrt((link1_cp(1,2)-link1_cp(1,1))^2 + (link1_cp(2,2)-link1_cp(2,1))^2);
    
    [link2_c1_cp, dist_link2_c1]=circle_line_distance(center_c1, radius1, seg2);
    [link2_c2_cp, dist_link2_c2]=circle_line_distance(center_c2, radius2, seg2);
    [link2_c3_cp, dist_link2_c3]=circle_line_distance(center_c3, radius3, seg2);

    l2c=[links(1)*cos(th(1,1))+links(2)*cos(th(1,1)+th(2,1));links(1)*sin(th(1,1))+links(2)*sin(th(1,1)+th(2,1))];

    if(dist_link2_c1 <= dist_link2_c2)
      dist_link2_obs = dist_link2_c1;
      radius = radius1;
      center_obs = center_c1;
      link2_cp = link2_c1_cp;
      l2contact=links(2)-sqrt((link2_cp(1,2)-l2c(1,1))^2 + (link2_cp(2,2)-l2c(2,1))^2);
    else
      dist_link2_obs = dist_link2_c2;
      radius = radius2;
      center_obs = center_c2;
      link2_cp = link2_c2_cp;
      l2contact=links(2)-sqrt((link2_cp(1,2)-l2c(1,1))^2 + (link2_cp(2,2)-l2c(2,1))^2);
     end
     if(dist_link2_c3 < dist_link2_obs)
       dist_link2_obs = dist_link2_c3;
       radius = radius3;
       center_obs = center_c3;
       link2_cp = link2_c3_cp;
       l2contact=links(2)-sqrt((link2_cp(1,2)-l2c(1,1))^2 + (link2_cp(2,2)-l2c(2,1))^2);
     end
     
     % contact jacobian for bar 2 at the beginning of current step
    Jc11_1 = -links(1)*sin(th(1))-l2contact*sin(th(1) + th(2));
    Jc12_1 =-l2contact*sin(th(1) + th(2));
    Jc21_1 = links(1)*cos(th(1))+l2contact*cos(th(1) + th(2));
    Jc22_1 = l2contact*cos(th(1) + th(2));
    Jcon= [Jc11_1,Jc12_1;Jc21_1,Jc22_1];

    contact_dir = (link2_cp(:,1) - center_obs)/radius;
    vec_cp_joint = link2_cp(:,2) - seg2(:,1);
    contact_wrench(2,2) = -(link2_cp(2,1)-link2_cp(2,2))/sqrt((link2_cp(1,1)-link2_cp(1,2))^2 + (link2_cp(2,1)-link2_cp(2,2))^2);
    contact_wrench(1,2) = -(link2_cp(1,1)-link2_cp(1,2))/sqrt((link2_cp(1,1)-link2_cp(1,2))^2 + (link2_cp(2,1)-link2_cp(2,2))^2);
    
    dista(2) = sqrt((link2_cp(1,2)-link2_cp(1,1))^2 + (link2_cp(2,2)-link2_cp(2,1))^2 );
end