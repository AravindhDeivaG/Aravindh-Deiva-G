function [p1,p2,p3] = forward_kinematics(theta_vector)
%FORWARD_KINEMATICS Summary of this function goes here
%   Detailed explanation goes here

% 1. Screw axis of joints in general abb_irb_140 bot

s1 = [0 -1 0 0;
    1 0 0 0;
    0 0 0 0;
    0 0 0 0];

s2 = [0 0 1 -0.3520;
    0 0 0 0 ;
    -1 0 0 0;
    0 0 0 0];

s3 = [0 0 1 -0.7120;
    0 0 0 0;
    -1 0 0 0;
    0 0 0 0];

T43 = [1 0 0 0.5150;
    0 1 0 0;
    0 0 1 0.7120;
    0 0 0 1];

% 2. Transformation matrices of adjacent frames
T10 = expm(s1*theta_vector(1));
T21 = expm(s2*theta_vector(2));
T32 = expm(s3*theta_vector(3));

% 3. Find all points on elbow
% Th --> Transformation matrix in home configuration
Th_2 = [ 0.0000   -1.0000         0    0.0000
    1.0000    0.0000         0    0.0000
         0         0    1.0000    0.7120
         0         0         0    1.0000];
Th_3 = [0.0000   -1.0000         0    0.5150
    1.0000    0.0000         0    0
         0         0    1.0000    0.7120
         0         0         0    1.0000];
     
T20 = T10*T21*Th_2;
T30 = T10*T21*T32*Th_3;

p1 = [0;0;0.3520]';
p2 = T20(1:3,4)';
p3 = T30(1:3,4)';
end

