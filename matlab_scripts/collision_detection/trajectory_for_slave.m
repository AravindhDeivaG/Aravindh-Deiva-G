% URDF file of robot
robot = importrobot('/home/aravindh/cobot_ws/src/abb_irb140_support/matlab_scripts/collision_detection/irb140.urdf');
global traj_slave;
%############################################################
% Position of robots wrt global coordinates

T_m = [1 0 0 0
    0 0 -1 0
    0 1 0 0 
    0 0 0 1];
     
T_s = [1 0 0 0
    0 0 1 1.2
    0 -1 0 0
    0 0 0 1];

%############################################################
% Path planning for slave bot
r = 0.2;
t = 5;
waypoints = [0.6 0.6 0.6 0.6 0.6
    0 -0.2 0 0.2 0
    0.5 0.3 0.1 0.3 0.5];

velocity = 2*pi*r/t;
v = [0 0 0 0 0
    -1 0 1 0 -1
    0 -1 0 1 0];
timepoints = 0:t/4:t;
timesamples = 0:0.1:t';
[q,qd,qdd,pp] = cubicpolytraj(waypoints,timepoints,timesamples,'VelocityBoundaryCondition',v*velocity);

%############################################################
IK = inverseKinematics('RigidBodyTree',robot);
weights = [0 0 0 1 1 1];
Q_prev = homeConfiguration(robot);
%Q_curr = IK('tool0_m',trvec2tform(point),weights,Q_prev);
Q_sol = [];
for i=1:length(q)
    T = trvec2tform(q(:,i)');
    Q_curr = IK('link_6_1',T,weights,Q_prev);
    Q = [Q_curr.JointPosition];
    Q_sol = [Q_sol;Q(1,1:6)];
end

traj_slave = Q_sol;