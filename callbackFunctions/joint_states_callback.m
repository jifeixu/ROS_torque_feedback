function joint_states_callback( ~, msg )
%JOINT_STATES_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global u_effort;
global sys_desired;
global odometer;
global global_zmp;
global right_ft; 
global left_ft;
global ankle_joint_ref;
global alpha;
% global joint_states; % do I need global variable for these?
% msg.Position(39) %left leg link 5 joint pos out
% msg.Position(40) %left leg link 6 joint pos out
% msg.Position(45) %right leg link 5 joint pos out
% msg.Position(46) %right leg link 6 joint pos out
% 
% msg.Velocity(39) %left leg link 5 joint vel out
% msg.Velocity(40) %left leg link 6 joint vel out
% msg.Velocity(45) %right leg link 5 joint vel out
% msg.Velocity(46) %right leg link 6 joint vel out
torque_x_axis=ankle_controller_x(sys_desired.x,[odometer.posX,odometer.velX,global_zmp.x]);
torque_y_axis=ankle_controller_y(sys_desired.y,[odometer.posY,odometer.velY,global_zmp.y]);

[u_effort.left_joint_5,u_effort.right_joint_5]=torque_distributer( torque_x_axis,...
                                                                   left_ft.force,right_ft.force,...
                                                                   msg.Position(39),msg.Position(45),...
                                                                   msg.Velocity(39),msg.Velocity(45),...
                                                                   ankle_joint_ref.left_joint_5_pos_ref,ankle_joint_ref.right_joint_5_pos_ref,...
                                                                   0,0,...
                                                                   alpha);
[u_effort.left_joint_6,u_effort.right_joint_6]=torque_distributer( torque_y_axis,...
                                                                   left_ft.force,right_ft.force,...
                                                                   msg.Position(40),msg.Position(46),...
                                                                   msg.Velocity(40),msg.Velocity(46),...
                                                                   ankle_joint_ref.left_joint_6_pos_ref,ankle_joint_ref.right_joint_6_pos_ref,...
                                                                   0,0,...
                                                                   alpha);
                                                               


end

