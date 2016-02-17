function [ torque_left,torque_right ] = ...
                           torque_distributer( total_torque,...
                                               left_force,right_force,...
                                               left_joint_pos,right_joint_pos,...
                                               left_joint_vel,right_joint_vel,...
                                               left_joint_pos_ref,right_joint_pos_ref,...
                                               left_joint_vel_ref,right_joint_vel_ref,...
                                               alpha)
%TORQUE_DISTRIBUTER Summary of this function goes here
%   use alpha and decompose torques to individual torques
%   note alpha here is not global.

%torque left
if(left_force <=0)
    torque_left=[-1000 -580]*[left_joint_pos-left_joint_pos_ref;left_joint_vel-left_joint_vel_ref];
else
    torque_left=total_torque*alpha;
end

if(right_force <=0)
    torque_right=[-1000 -580]*[right_joint_pos-right_joint_pos_ref; right_joint_vel-right_joint_vel_ref];
else
    torque_right=total_torque*(1-alpha);
end

end

