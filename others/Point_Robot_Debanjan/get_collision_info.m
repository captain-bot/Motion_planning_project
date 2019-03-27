function [contact_wrench,dista]= get_collision_info(q_o)
    theta = pi/3;
    delta = 0.1;
    temp=0;
    dista = 0;

    if (q_o(1,1)>=0.5 && q_o(1,1)<1)
        seg1(1,1) = 1; 
        seg1(2,1) = 0.1;
        seg1(1,2) = .5;
        seg1(2,2) = 1;
        [distanceSegment, q_closest]= point_line_distance(q_o, seg1);
        dista = point_line_distance(q_o, seg1);
        contact_wrench(1,1) = (seg1(2,1) - seg1(2,2)); %Contact wrench is the lagrange multiplier
        contact_wrench(2,1) = (seg1(1,2) - seg1(1,1));
        q_diff = q_o - q_closest;
        temp = q_diff'*contact_wrench;
    elseif (q_o(1,1)>=1 && q_o(1,1)<=1.5)
        seg2(1,1) = 1.5;
        seg2(2,1) = 0.9;
        seg2(1,2) = 1.0;
        seg2(2,2) = 0.0;
        [distanceSegment, q_closest] = point_line_distance(q_o, seg2);
        dista= distanceSegment;
        contact_wrench(1,1) = -(seg2(2,1) - seg2(2,2));
        contact_wrench(2,1) = -(seg2(1,2) - seg2(1,1));%Contact wrench is the lagrange multiplier
        q_diff = q_o - q_closest;
        temp = q_diff'*contact_wrench;
    elseif (q_o(1,1)>1.5 && q_o(1,1)<=2.0)
        seg1(1,1) = 2.0;
        seg1(2,1) = 0.1;
        seg1(1,2) = 1.5;
        seg1(2,2) = 1.0;
        [distanceSegment, q_closest]= point_line_distance(q_o, seg1);
        dista = point_line_distance(q_o, seg1);
        contact_wrench(1,1) = (seg1(2,1) - seg1(2,2));
        contact_wrench(2,1) = (seg1(1,2) - seg1(1,1));
        q_diff = q_o - q_closest;
        temp = q_diff'*contact_wrench;
    elseif (q_o(1,1)>2.0 && q_o(1,1)<=2.5)
        seg2(1,1) = 2.5;
        seg2(2,1) = 0.9; 
        seg2(1,2) = 2.0;
        seg2(2,2) = 0.0;
        [distanceSegment, q_closest] = point_line_distance(q_o, seg2);
        dista= distanceSegment;
        contact_wrench(1,1) = -(seg2(2,1) - seg2(2,2));
        contact_wrench(2,1) = -(seg2(1,2) - seg2(1,1));
        q_diff = q_o - q_closest;
        temp = q_diff'*contact_wrench;
    elseif (q_o(1,1)>2.5 && q_o(1,1)<=3)
        seg1(1,1) = 3.0; 
        seg1(2,1) = 0.1;
        seg1(1,2) = 2.5;
        seg1(2,2) = 1.0;
        [distanceSegment, q_closest]= point_line_distance(q_o, seg1);
        dista = point_line_distance(q_o, seg1);
        contact_wrench(1,1) = (seg1(2,1) - seg1(2,2));
        contact_wrench(2,1) = (seg1(1,2) - seg1(1,1));
        q_diff = q_o - q_closest;
        temp = q_diff'*contact_wrench;
    elseif (q_o(1,1)>3 && q_o(1,1)<=3.5)
        seg2(1,1) = 3.5;
        seg2(2,1) = .9; 
        seg2(1,2) = 3;
        seg2(2,2) = 0;
        [distanceSegment, q_closest] = point_line_distance(q_o, seg2);
        dista= distanceSegment;
        contact_wrench(1,1) = -(seg2(2,1) - seg2(2,2));
        contact_wrench(2,1) = -(seg2(1,2) - seg2(1,1));
        q_diff = q_o - q_closest;
        temp = q_diff'*contact_wrench;
    elseif (q_o(1,1)>3.5 && q_o(1,1)<=4)
        seg1(1,1) = 4; 
        seg1(2,1) = 0.1;
        seg1(1,2) = 3.5;
        seg1(2,2) = 1;
        [distanceSegment, q_closest]= point_line_distance(q_o, seg1);
        dista = point_line_distance(q_o, seg1);
        contact_wrench(1,1) = (seg1(2,1) - seg1(2,2));
        contact_wrench(2,1) = (seg1(1,2) - seg1(1,1));
        q_diff = q_o - q_closest;
        temp = q_diff'*contact_wrench;
    elseif (q_o(1,1)>4 && q_o(1,1)<=4.5)
        seg2(1,1) = 4.5;
        seg2(2,1) = .9; 
        seg2(1,2) = 4;
        seg2(2,2) = 0;
        [distanceSegment, q_closest] = point_line_distance(q_o, seg2);
        dista= distanceSegment;
        contact_wrench(1,1) = -(seg2(2,1) - seg2(2,2));
        contact_wrench(2,1) = -(seg2(1,2) - seg2(1,1));
        q_diff = q_o - q_closest;
        temp = q_diff'*contact_wrench;
     end

     if(q_o(1,1) > 4.5)
        seg1(1,1) = 4.5;
        seg1(2,1) = sin(theta);
        seg1(1,2) = 5;
        seg1(2,2) = 0;
        [distanceSegment, q_closest]= point_line_distance(q_o, seg1);
        dista = point_line_distance(q_o, seg1);
        contact_wrench(1,1) = (seg1(2,1) - seg1(2,2));
        contact_wrench(2,1) = (seg1(1,2) - seg1(1,2));
        if(dista > 0.01 )
            contact_wrench(1,1) = 0;
            contact_wrench(2,1) = 0;
        end
        q_diff = q_o - q_closest;
        temp = q_diff'*contact_wrench;
     end

    if(temp < 0)
        dista = -dista;
    end
 
end