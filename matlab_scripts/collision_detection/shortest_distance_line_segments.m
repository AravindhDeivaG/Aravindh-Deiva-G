function [distance_min] = shortest_distance_line_segments(p1,p2,q1,q2)
%SHORTEST_DISTANCE_LINE_SEGMENTS: Used to find shortest distance between
%two lines
%   Detailed explanation goes here:
%   Finds m and n such that
%   p = p1+m(p2-p1)
%   q = p2+n(q2-q1)
%   such that pq forms the closest distance of approach between two lines
    
    a = (p2-p1)*(p2-p1)';
    b = -(q2-q1)*(p2-p1)';
    c = (q2-q1)*(p2-p1)';
    d = -(q2-q1)*(q2-q1)';
    e = (q1-p1)*(p2-p1)';
    f = (q1-p1)*(q2-q1)';
    
    A = [a b;c d];
    y = [e;f];
    x = pinv(A)*y;
    m = x(1,1);
    n = x(2,1);
    
    p = p1+m*(p2-p1);
    q = q1+n*(q2-q1);
    
    d1 = norm(p1-q1);
    d2 = norm(p1-q2);
    d3 = norm(p2-q1);
    d4 = norm(p2-q2);
    d = [d1 d2 d3 d4];
    if(m>=0 && m<=1 && n>=0 && n<=1)
        shortest_distance = norm(p-q);
    else
        [kp1,p1_foot] = foot_of_perpendicular(p1,q1,q2);
        [kp2,p2_foot] = foot_of_perpendicular(p2,q1,q2);
        [kq1,q1_foot] = foot_of_perpendicular(q1,p1,p2);
        [kq2,q2_foot] = foot_of_perpendicular(q2,p1,p2);
        cond1 = kp1>=0 && kp1<=1;
        cond2 = kp2>=0 && kp2<=1;
        cond3 = kq1>=0 && kq1<=1;
        cond4 = kq2>=0 && kq2<=1;
        if(cond1)
            d(end+1) = norm(p1-p1_foot);
        end
        if(cond2)
            d(end+1) = norm(p2-p2_foot);
        end
        if(cond3)
            d(end+1) = norm(q1-q1_foot);
        end
        if(cond4)
            d(end+1) = norm(q2-q2_foot);
        end
    end
    
    distance_min = min(d);
            
end

