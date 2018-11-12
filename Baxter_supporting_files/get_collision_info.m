function [contact_wrench,dista,jcon1,jcon2,jcon3,jcon4,jcon5,jcon6,jcon7]=get_collision_info(q_o)
    center_s1(1,1) = 0.9; center_s1(2,1) = 0.0; center_s1(3,1) = 0.8; radius1 = 0.05;
    center_s2(1,1) = 0.7; center_s2(2,1) = 0.0; center_s2(3,1) = 0.6; radius2 = 0.05;
    center_s3(1,1) = 0.9; center_s3(2,1) = 0.0; center_s3(3,1) = 0.4;  radius3 = 0.05;

    theta1 = q_o(1,1); theta2 = q_o(2,1); theta3 = q_o(3,1); theta4 = q_o(4,1); theta5 = q_o(5,1); theta6 = q_o(6,1); theta7 = q_o(7,1);
    theta=[theta1,theta2,theta3,theta4,theta5,theta6,theta7];

    contact_wrench=zeros(6,7);

    [g_st,pt1,xi1] = forward_kinematics(theta,1);
    [g_st,pt2,xi2] = forward_kinematics(theta,2);
    [g_st,pt3,xi3] = forward_kinematics(theta,3);
    [g_st,pt4,xi4] = forward_kinematics(theta,4);
    [g_st,pt5,xi5] = forward_kinematics(theta,5);
    [g_st,pt6,xi6] = forward_kinematics(theta,6);
    [g_st,pt7,xi7] = forward_kinematics(theta,7);


    jcon1=zeros(6,7);jcon2=zeros(6,7);jcon3=zeros(6,7);jcon4=zeros(6,7);jcon5=zeros(6,7);jcon6=zeros(6,7);jcon7=zeros(6,7);

    seg1(1,1) = 0; seg1(2,1) = 0; seg1(3,1) = 0;
    seg1(1,2) = pt1(1,1); seg1(2,2) = pt1(1,2);  seg1(3,2) = pt1(1,3);
    seg2(1,1) = seg1(1,2); seg2(2,1) = seg1(2,2); seg2(3,1)= seg1(3,2);
    seg2(1,2) = pt2(1,1); seg2(2,2) = pt2(1,2); seg2(3,2) = pt2(1,3);
    seg3(1,1) = seg2(1,2); seg3(2,1) = seg2(2,2); seg3(3,1)= seg2(3,2);
    seg3(1,2) = pt3(1,1); seg3(2,2) = pt3(1,2); seg3(3,2) = pt3(1,3);
    seg4(1,1) = seg3(1,2); seg4(2,1) = seg3(2,2); seg4(3,1)= seg3(3,2);
    seg4(1,2) = pt4(1,1); seg4(2,2) = pt4(1,2); seg4(3,2) = pt4(1,3);
    seg5(1,1) = seg4(1,2); seg5(2,1) = seg4(2,2); seg5(3,1)= seg4(3,2);
    seg5(1,2) = pt5(1,1); seg5(2,2) = pt5(1,2); seg5(3,2) = pt5(1,3);
    seg6(1,1) = seg5(1,2); seg6(2,1) = seg5(2,2); seg6(3,1)= seg5(3,2);
    seg6(1,2) = pt6(1,1); seg6(2,2) = pt6(1,2); seg6(3,2) = pt6(1,3);
    seg7(1,1) = seg6(1,2); seg7(2,1) = seg6(2,2); seg7(3,1)= seg6(3,2);
    seg7(1,2) = pt7(1,1); seg7(2,2) = pt7(1,2); seg7(3,2) = pt7(1,3);

    eps = 1e-08;

    [link1_s1_cp, dist_link1_s1, cw11]=sphere_line_distance(center_s1, radius1, seg1);
    [link1_s2_cp, dist_link1_s2, cw12]=sphere_line_distance(center_s2, radius2, seg1);
    [link1_s3_cp, dist_link1_s3, cw13]=sphere_line_distance(center_s3, radius3, seg1);

    if(dist_link1_s1 <= dist_link1_s2)
        dist_link1_obs = dist_link1_s1;
        radius = radius1;
        center_obs = center_s1;
        link1_cp = link1_s1_cp;
        contact_wrench(1,1)= cw11(1,1); contact_wrench(2,1)= cw11(2,1); contact_wrench(3,1)= cw11(3,1);
    else
        dist_link1_obs = dist_link1_s2;
        radius = radius2;
        center_obs = center_s2;
        link1_cp = link1_s2_cp;
        contact_wrench(1,1)= cw12(1,1); contact_wrench(2,1)= cw12(2,1); contact_wrench(3,1)= cw12(3,1);
    end
    if(dist_link1_s3 < dist_link1_obs)
        dist_link1_obs = dist_link1_s3;
        radius = radius3;
        center_obs = center_s3;
        link1_cp = link1_s3_cp;
        contact_wrench(1,1)= cw13(1,1); contact_wrench(2,1)= cw13(2,1); contact_wrench(3,1)= cw13(3,1);
    end
    [jcon1,xi1] = contact_jacobian(theta,1,link1_cp(:,2));
    dista(1) = dist_link1_obs;
    
    [link2_s1_cp, dist_link2_s1,cw21]=sphere_line_distance(center_s1, radius1, seg2);
    [link2_s2_cp, dist_link2_s2,cw22]=sphere_line_distance(center_s2, radius2, seg2);
    [link2_s3_cp, dist_link2_s3,cw23]=sphere_line_distance(center_s3, radius3, seg2);

    if(dist_link2_s1 <= dist_link2_s2)
        dist_link2_obs = dist_link2_s1;
        radius = radius1;
        center_obs = center_s1;
        link2_cp = link2_s1_cp;
        contact_wrench(1,2)= cw21(1,1); contact_wrench(2,1)= cw21(2,1); contact_wrench(3,1)= cw21(3,1);
    else
        dist_link2_obs = dist_link2_s2;
        radius = radius2;
        center_obs = center_s2;
        link2_cp = link2_s2_cp;
        contact_wrench(2,2)= cw22(1,1); contact_wrench(2,1)= cw22(2,1); contact_wrench(3,1)= cw22(3,1);
    end
    if(dist_link2_s3 < dist_link2_obs)
        dist_link2_obs = dist_link2_s3;
        radius = radius3;
        center_obs = center_s3;
        link2_cp = link2_s3_cp;
        contact_wrench(1,2)= cw23(1,1); contact_wrench(2,1)= cw23(2,1); contact_wrench(3,1)= cw23(3,1);
    end
    [jcon2,xi2] = contact_jacobian(theta,2,link2_cp(:,2));
    dista(2) = dist_link2_obs;

    [link3_s1_cp, dist_link3_s1,cw31]=sphere_line_distance(center_s1, radius1, seg3);
    [link3_s2_cp, dist_link3_s2,cw32]=sphere_line_distance(center_s2, radius2, seg3);
    [link3_s3_cp, dist_link3_s3,cw33]=sphere_line_distance(center_s3, radius3, seg3);


    if(dist_link3_s1 <= dist_link3_s2)
        dist_link3_obs = dist_link3_s1;
        radius = radius1;
        center_obs = center_s1;
        link3_cp = link3_s1_cp;
        contact_wrench(1,3)= cw31(1,1); contact_wrench(2,3)= cw31(2,1); contact_wrench(3,3)= cw31(3,1);
    else
        dist_link3_obs = dist_link3_s2;
        radius = radius2;
        center_obs = center_s2;
        link3_cp = link3_s2_cp;
        contact_wrench(1,3)= cw32(1,1); contact_wrench(2,3)= cw32(2,1); contact_wrench(3,3)= cw32(3,1);
    end
    if(dist_link3_s3 < dist_link3_obs)
        dist_link3_obs = dist_link3_s3;
        radius = radius3;
        center_obs = center_s3;
        link3_cp = link3_s3_cp;
        contact_wrench(1,3)= cw33(1,1); contact_wrench(2,3)= cw33(2,1); contact_wrench(3,3)= cw33(3,1);
    end
    [jcon3,xi3] = contact_jacobian(theta,3,link3_cp(:,2));
    dista(3) = dist_link3_obs;
    [link4_s1_cp, dist_link4_s1,cw41]=sphere_line_distance(center_s1, radius1, seg4);
    [link4_s2_cp, dist_link4_s2,cw42]=sphere_line_distance(center_s2, radius2, seg4);
    [link4_s3_cp, dist_link4_s3,cw43]=sphere_line_distance(center_s3, radius3, seg4);


    if(dist_link4_s1 <= dist_link4_s2)
        dist_link4_obs = dist_link4_s1;
        radius = radius1;
        center_obs = center_s1;
        link4_cp = link4_s1_cp;
        contact_wrench(1,4)= cw41(1,1); contact_wrench(2,4)= cw41(2,1); contact_wrench(3,4)= cw41(3,1);
    else
        dist_link4_obs = dist_link4_s2;
        radius = radius2;
        center_obs = center_s2;
        link4_cp = link4_s2_cp;
        contact_wrench(1,4)= cw42(1,1); contact_wrench(2,4)= cw42(2,1); contact_wrench(3,4)= cw42(3,1);
    end
    if(dist_link4_s3 < dist_link4_obs)
        dist_link4_obs = dist_link4_s3;
        radius = radius3;
        center_obs = center_s3;
        link4_cp = link4_s3_cp;
        contact_wrench(1,4)= cw43(1,1); contact_wrench(2,4)= cw43(2,1); contact_wrench(3,4)= cw43(3,1);
    end
    [jcon4,xi4] = contact_jacobian(theta,4,link4_cp(:,2));
    dista(4) = dist_link4_obs;

    [link5_s1_cp, dist_link5_s1,cw51]=sphere_line_distance(center_s1, radius1, seg5);
    [link5_s2_cp, dist_link5_s2,cw52]=sphere_line_distance(center_s2, radius2, seg5);
    [link5_s3_cp, dist_link5_s3,cw53]=sphere_line_distance(center_s3, radius3, seg5);

    if(dist_link5_s1 <= dist_link5_s2)
        dist_link5_obs = dist_link5_s1;
        radius = radius1;
        center_obs = center_s1;
        link5_cp = link5_s1_cp;
        contact_wrench(1,5)= cw51(1,1); contact_wrench(2,5)= cw51(2,1); contact_wrench(3,5)= cw51(3,1);
    else
        dist_link5_obs = dist_link5_s2;
        radius = radius2;
        center_obs = center_s2;
        link5_cp = link5_s2_cp;
        contact_wrench(1,5)= cw52(1,1); contact_wrench(2,5)= cw52(2,1); contact_wrench(3,5)= cw52(3,1);
    end
    if(dist_link5_s3 < dist_link5_obs)
        dist_link5_obs = dist_link5_s3;
        radius = radius3;
        center_obs = center_s3;
        link5_cp = link5_s3_cp;
        contact_wrench(1,5)= cw53(1,1); contact_wrench(2,5)= cw53(2,1); contact_wrench(3,5)= cw53(3,1);
    end
    [jcon5,xi5] = contact_jacobian(theta,5,link5_cp(:,2));
    dista(5) = dist_link5_obs;

    [link6_s1_cp, dist_link6_s1,cw61]=sphere_line_distance(center_s1, radius1, seg6);
    [link6_s2_cp, dist_link6_s2,cw62]=sphere_line_distance(center_s2, radius2, seg6);
    [link6_s3_cp, dist_link6_s3,cw63]=sphere_line_distance(center_s3, radius3, seg6);

    if(dist_link6_s1 <= dist_link6_s2)
        dist_link6_obs = dist_link6_s1;
        radius = radius1;
        center_obs = center_s1;
        link6_cp = link6_s1_cp;
        contact_wrench(1,6)= cw61(1,1); contact_wrench(2,6)= cw61(2,1); contact_wrench(3,6)= cw61(3,1);
    else
        dist_link6_obs = dist_link6_s2;
        radius = radius2;
        center_obs = center_s2;
        link6_cp = link6_s2_cp;
        contact_wrench(1,6)= cw62(1,1); contact_wrench(2,6)= cw62(2,1); contact_wrench(3,6)= cw62(3,1);
    end

    if(dist_link6_s3 < dist_link6_obs)
        dist_link6_obs = dist_link6_s3;
        radius = radius3;
        center_obs = center_s3;
        link6_cp = link6_s3_cp;
        contact_wrench(1,6)= cw63(1,1); contact_wrench(2,6)= cw63(2,1); contact_wrench(3,6)= cw63(3,1);
    end

    [jcon6,xi6] = contact_jacobian(theta,6,link6_cp(:,2));
    dista(6) = dist_link6_obs;

    [link7_s1_cp, dist_link7_s1,cw71]=sphere_line_distance(center_s1, radius1, seg7);
    [link7_s2_cp, dist_link7_s2,cw72]=sphere_line_distance(center_s2, radius2, seg7);
    [link7_s3_cp, dist_link7_s3,cw73]=sphere_line_distance(center_s3, radius3, seg7);

    if(dist_link7_s1 <= dist_link7_s2)
        dist_link7_obs = dist_link7_s1;
        radius = radius1;
        center_obs = center_s1;
        link7_cp = link7_s1_cp;
        contact_wrench(1,7)= cw71(1,1); contact_wrench(2,7)= cw71(2,1); contact_wrench(3,7)= cw71(3,1);
    else
        dist_link7_obs = dist_link7_s2;
        radius = radius2;
        center_obs = center_s2;
        link7_cp = link7_s2_cp;
        contact_wrench(1,7)= cw72(1,1); contact_wrench(2,7)= cw72(2,1); contact_wrench(3,7)= cw72(3,1);
    end
    if(dist_link7_s3 < dist_link7_obs)
        dist_link7_obs = dist_link7_s3;
        radius = radius3;
        center_obs = center_s3;
        link7_cp = link7_s3_cp;
        contact_wrench(1,7)= cw73(1,1); contact_wrench(2,7)= cw73(2,1); contact_wrench(3,7)= cw73(3,1);
    end
    [jcon7,xi7] = contact_jacobian(theta,7,link7_cp(:,2));
    dista(7) = dist_link7_obs;
end