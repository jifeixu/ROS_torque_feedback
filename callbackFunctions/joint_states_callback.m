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
global calculated_com;
% global control of effort. governed by ankle_control_on or offs
global right_leg_joint_5_effort_pub;
global left_leg_joint_5_effort_pub;
global right_leg_joint_6_effort_pub;
global left_leg_joint_6_effort_pub;
global ankle_control_on;
global right_leg_joint_5_effort_msg;
global left_leg_joint_5_effort_msg;
global right_leg_joint_6_effort_msg;
global left_leg_joint_6_effort_msg;

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
% torque_x_axis=ankle_controller_x(sys_desired.x,[calculated_com.X_com,odometer.velX,global_zmp.x]); % to control x axis motion
% torque_y_axis=ankle_controller_y(sys_desired.y,[calculated_com.Y_com,odometer.velY,global_zmp.y]);   % to control y axis motion

torque_x_axis=ankle_controller_x(sys_desired.x,[odometer.posX,odometer.velX,global_zmp.x]); % to control x axis motion
torque_y_axis=ankle_controller_y(sys_desired.y,[odometer.posY,odometer.velY,global_zmp.y]);   % to control y axis motion


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
                                                               
if (ankle_control_on ==1)
    right_leg_joint_5_effort_msg.Data=double(u_effort.right_joint_5);
    right_leg_joint_6_effort_msg.Data=double(u_effort.right_joint_6);
    left_leg_joint_5_effort_msg.Data=double(u_effort.left_joint_5);
    left_leg_joint_6_effort_msg.Data=double(u_effort.left_joint_6);
    send(right_leg_joint_5_effort_pub,right_leg_joint_5_effort_msg);
    send(right_leg_joint_6_effort_pub,right_leg_joint_6_effort_msg);
    send(left_leg_joint_5_effort_pub,left_leg_joint_5_effort_msg);
    send(left_leg_joint_6_effort_pub,left_leg_joint_6_effort_msg);

else
    right_leg_joint_5_effort_msg.Data=0;
    right_leg_joint_6_effort_msg.Data=0;
    left_leg_joint_5_effort_msg.Data=0;
    left_leg_joint_6_effort_msg.Data=0;
    send(right_leg_joint_5_effort_pub,right_leg_joint_5_effort_msg);
    send(right_leg_joint_6_effort_pub,right_leg_joint_6_effort_msg);
    send(left_leg_joint_5_effort_pub,left_leg_joint_5_effort_msg);
    send(left_leg_joint_6_effort_pub,left_leg_joint_6_effort_msg);

end

end

