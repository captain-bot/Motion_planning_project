function [LineSegment, q_closest, check_side]= point_line_distance_kin(point, line_seg)
%LineSegment gives the coeffcients of the equation of the line to be used
%in the complementarity constraint. The equation of the line will be of the
%form u1*qx + u2*qy + u3 = 0, where u = [u1;u2] is the normal to the line or the unit
% wrench, i.e., ||u|| = 1.
%q_closest gives the closest point (not clear that it is required)
% point is a column vector containing the coordinates of the point.
% line_seg is a 2 x 2 matrix, where each column is the coordinate of an
% end point of the line segment.

cx = point(1); % x-coordinate of point
cy = point(2); % y-coordinate of point
%line_seg= [ax  bx ; ay by];
ax=line_seg(1,1); % x-coordinate of first point on segment
ay=line_seg(2,1); % y-coordinate of first point on segment
bx=line_seg(1,2); % x-coordinate of second point on segment
by=line_seg(2,2); % y-coordinate of second point on segment

mag_u = sqrt((bx - ax)^2 + (ay-by)^2);
u1 = (ay - by)/mag_u; 
u2 = (bx-ax)/mag_u;
u3 = ((ax-bx)*ay + (by-ay)*ax)/mag_u;

LineSegment = [u1;u2;u3];

signed_distance = u1*cx + u2*cy + u3;

distToEnd1 = sqrt((cx-ax)*(cx-ax) + (cy-ay)*(cy-ay));
distToEnd2 = sqrt((cx-bx)*(cx-bx) + (cy-by)*(cy-by));

if (distToEnd1 > distToEnd2)
    q_closest



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
%%return distanceSegment;
end