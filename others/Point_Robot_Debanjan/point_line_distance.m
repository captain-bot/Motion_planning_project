function [distanceSegment, q_closest]= point_line_distance(point, line_seg)
% function [LineSegment, q_closest]= point_line_distance(point, line_seg)
    cx = point(1);
    cy = point(2);
    %line_seg= [ax  bx ; ay by];
    ax=line_seg(1,1);
    ay=line_seg(2,1);
    bx=line_seg(1,2);
    by=line_seg(2,2);
    r_numerator = (cx-ax)*(bx-ax) + (cy-ay)*(by-ay);
    r_denominator = (bx-ax)*(bx-ax) + (by-ay)*(by-ay);
    r = r_numerator / r_denominator;
    px = ax + r*(bx-ax);
    py = ay + r*(by-ay);
    s =  ((ay-cy)*(bx-ax)-(ax-cx)*(by-ay) ) / r_denominator;
    distanceLine = abs(s)*sqrt(r_denominator);
    xx = px;
    yy = py;
    if ( (r >= 0) && (r <= 1) )

          distanceSegment = distanceLine;
    else
          dist1 = (cx-ax)*(cx-ax) + (cy-ay)*(cy-ay);
          dist2 = (cx-bx)*(cx-bx) + (cy-by)*(cy-by);
          if (dist1 < dist2)
              xx = ax;
              yy = ay;
              distanceSegment = sqrt(dist1);
          else
              xx = bx;
              yy = by;
              distanceSegment = sqrt(dist2);
          end   
    end
    temp_pt= [xx;yy];
    q_closest = temp_pt;
end