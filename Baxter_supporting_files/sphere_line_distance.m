function [link_sphere_cp,d,contact_wrench]=sphere_line_distance(center,radius,line_seg)
    s=[-line_seg(1,1)+line_seg(1,2); -line_seg(2,1)+line_seg(2,2); -line_seg(3,1)+line_seg(3,2)];
    t=((s(1,1)*(center(1,1)-line_seg(1,1))) + (s(2,1)*(center(2,1)-line_seg(2,1))) + (s(3,1)*(center(3,1)-line_seg(3,1))))/(s(1,1)^2 + s(2,1)^2 + s(3,1)^2);

    d= sqrt((line_seg(1,1)+(s(1,1)*t)-center(1,1))^2 + (line_seg(2,1)+(s(2,1)*t)-center(2,1))^2 + (line_seg(3,1)+(s(3,1)*t)-center(3,1))^2) - radius;

    point_sphere= [center(1,1)+((line_seg(1,1)+(s(1,1)*t)-center(1,1))*radius)/(radius+d); center(2,1)+((line_seg(2,1)+(s(2,1)*t)-center(2,1))*radius)/(radius+d); ...
                center(3,1)+((line_seg(3,1)+(s(3,1)*t)-center(3,1))*radius)/(radius+d)];

    t1= ((s(1,1)*(point_sphere(1,1)-line_seg(1,1))) + (s(2,1)*(point_sphere(2,1)-line_seg(2,1))) + (s(3,1)*(point_sphere(3,1)-line_seg(3,1))))/(s(1,1)^2 + s(2,1)^2 + s(3,1)^2);
    point_line= [line_seg(1,1) + (s(1,1)*t1); line_seg(2,1) + (s(2,1)*t1); line_seg(3,1) + (s(3,1)*t1)];

    ac= sqrt((line_seg(1,2)-point_line(1,1))^2 + (line_seg(2,2)-point_line(2,1))^2 + (line_seg(3,2)-point_line(3,1))^2);
    bc= sqrt((line_seg(1,1)-point_line(1,1))^2 + (line_seg(2,1)-point_line(2,1))^2 + (line_seg(3,1)-point_line(3,1))^2);
    ab= sqrt((line_seg(1,2)-line_seg(1,1))^2 + (line_seg(2,2)-line_seg(2,1))^2 + (line_seg(3,2)-line_seg(3,1))^2);
    
    if ac> ab || bc>ab
        dis1= sqrt((point_line(1,1)-line_seg(1,1))^2 + (point_line(2,1)-line_seg(2,1))^2 + (point_line(3,1)-line_seg(3,1))^2);
        dis2= sqrt((point_line(1,1)-line_seg(1,2))^2 + (point_line(2,1)-line_seg(2,2))^2 + (point_line(3,1)-line_seg(3,2))^2);
        if dis1>dis2
            point_line(1,1)=line_seg(1,2); point_line(2,1)=line_seg(2,2); point_line(3,1)=line_seg(3,2);
        else
            point_line(1,1)=line_seg(1,1); point_line(2,1)=line_seg(2,1); point_line(3,1)=line_seg(3,1);
        end
    end
    
    d= sqrt((center(1,1)-point_line(1,1))^2 + (center(2,1)-point_line(2,1))^2 + (center(3,1)-point_line(3,1))^2) - radius;
    link_sphere_cp= [point_sphere, point_line];

    contact_wrench(1,1) = -(center(1,1)-point_line(1,1))/sqrt((center(1,1)-point_line(1,1))^2 + (center(2,1)-point_line(2,1))^2 + (center(3,1)-point_line(3,1))^2);
    contact_wrench(2,1) = -(center(2,1)-point_line(2,1))/sqrt((center(1,1)-point_line(1,1))^2 + (center(2,1)-point_line(2,1))^2 + (center(3,1)-point_line(3,1))^2);
    contact_wrench(3,1) = -(center(3,1)-point_line(3,1))/sqrt((center(1,1)-point_line(1,1))^2 + (center(2,1)-point_line(2,1))^2 + (center(3,1)-point_line(3,1))^2);
end