global traj_slave;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sub_joint_vals = rossubscriber('/abb_irb140/joint_states','DataFormat','struct');
% Publishers for master bot
pub_1_1 = rospublisher('/abb_irb140/joint1_position_controller_1/command');
pub_2_1 = rospublisher('/abb_irb140/joint2_position_controller_1/command');
pub_3_1 = rospublisher('/abb_irb140/joint3_position_controller_1/command');
pub_4_1 = rospublisher('/abb_irb140/joint4_position_controller_1/command');
pub_5_1 = rospublisher('/abb_irb140/joint5_position_controller_1/command');
pub_6_1 = rospublisher('/abb_irb140/joint6_position_controller_1/command');
% Publishers for slave bot
pub_1_2 = rospublisher('/abb_irb140/joint1_position_controller_2/command');
pub_2_2 = rospublisher('/abb_irb140/joint2_position_controller_2/command');
pub_3_2 = rospublisher('/abb_irb140/joint3_position_controller_2/command');
pub_4_2 = rospublisher('/abb_irb140/joint4_position_controller_2/command');
pub_5_2 = rospublisher('/abb_irb140/joint5_position_controller_2/command');
pub_6_2 = rospublisher('/abb_irb140/joint6_position_controller_2/command');

command = rosmessage(pub_1_1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% publish joint values to the simulation (slave)
pub_slave = [pub_1_2,pub_2_2,pub_3_2,pub_4_2,pub_5_2,pub_6_2];
while 1
    msg = receive(sub_joint_vals);
    is_collision = collision_callback(msg)
    if(is_collision==0)
        for i=1:length(traj_slave)
            for j=1:6
                command.Data = traj_slave(i,j);
                send(pub_slave(j),command);
            end
            pause(0.1)
        end
    end
    pause(0.1);
end
        
    