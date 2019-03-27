function [link_circle_cp,distance]=circle_line_distance(center,radius,line_seg)
    cx = center(1,1);
    cy = center(2,1);
    ax = line_seg(1,1);
    ay = line_seg(2,1);
    bx = line_seg(1,2);
    by = line_seg(2,2);
    r_numerator = (cx-ax)*(bx-ax) + (cy-ay)*(by-ay);
    r_denominator = (bx-ax)*(bx-ax) + (by-ay)*(by-ay);
    r = r_numerator / r_denominator;
    px = ax + r*(bx-ax);
    py = ay + r*(by-ay);
    s =  ((ay-cy)*(bx-ax)-(ax-cx)*(by-ay) ) / r_denominator;
    distanceLine = abs(s)*sqrt(r_denominator);
    xx_line = px;
    yy_line = py;

    if ( (r >= 0) && (r <= 1) )
        distanceSegment = distanceLine;
    else
        dist1 = (cx-ax)*(cx-ax) + (cy-ay)*(cy-ay);
        dist2 = (cx-bx)*(cx-bx) + (cy-by)*(cy-by);
        if (dist1 < dist2)
            xx_line = ax;
            yy_line = ay;
          distanceSegment = sqrt(dist1);
        else
            xx_line = bx;
          yy_line = by;
          distanceSegment = sqrt(dist2);
        end
    end
    xx_circle = cx + radius/distanceSegment*(xx_line - cx);
    yy_circle = cy + radius/distanceSegment*(yy_line - cy);
    link_circle_cp(1,1) = xx_circle; link_circle_cp(2,1) = yy_circle;
    link_circle_cp(1,2) = xx_line; link_circle_cp(2,2) = yy_line;
    distance = distanceSegment - radius;
end