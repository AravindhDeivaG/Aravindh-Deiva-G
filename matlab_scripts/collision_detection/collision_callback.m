function [is_collision] = collision_callback(msg)
%COLLISION_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
theta = msg.Position;
theta_1_vector = [theta(1) theta(3) theta(5)];
theta_2_vector = [theta(2) theta(4) theta(6)];

% Position of robots wrt global coordinates

T_m = [0 -1 0 0
       1  0  0 0
       0  0  1 0 
       0  0  0 1];
       
T_s = [0  1 0 0
       -1 0 0 1.2
       0  0 1 0
       0 0 0 1];

[pm_1,pm_2,pm_3] = forward_kinematics(theta_1_vector);
pm_1 = T_m*[pm_1 1]';
pm_2 = T_m*[pm_2 1]';
pm_3 = T_m*[pm_3 1]';
pm_1 = pm_1(1:3)';
pm_2 = pm_2(1:3)';
pm_3 = pm_3(1:3)';

[ps_1,ps_2,ps_3] = forward_kinematics(theta_2_vector);
ps_1 = T_s*[ps_1 1]';
ps_2 = T_s*[ps_2 1]';
ps_3 = T_s*[ps_3 1]';
ps_1 = ps_1(1:3)';
ps_2 = ps_2(1:3)';
ps_3 = ps_3(1:3)';

%disp('*******************')
pm = [0 0 0;pm_1;pm_2;pm_3];
ps = [0 1.2 0;ps_1;ps_2;ps_3];

capsule_width = [0.2 0.11 0.11];
is_collision = false;

for i=1:3
    for j=1:3
        m1 = pm(i,:);
        m2 = pm(i+1,:);
        s1 = ps(j,:);
        s2 = ps(j+1,:);
        d = shortest_distance_line_segments(m1,m2,s1,s2);
        d_min = capsule_width(i) + capsule_width(j);
        is_collision = is_collision + (d<=d_min);
    end
end


end

