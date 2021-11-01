%%%%%%%%%%%%%%%%%%%%%%%% foot of perpendicular %%%%%%%%%%%%%%%%%%%%%%%%%%%

function [k p_foot] = foot_of_perpendicular(p,p1,p2)
    k = (p-p1)*(p2-p1)'/(norm(p2-p1)^2);
    p_foot = k*(p2-p1) + p1;
end