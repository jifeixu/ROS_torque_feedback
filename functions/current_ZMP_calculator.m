function [ global_zmp ] = current_ZMP_calculator( torqueLR,forceLR,posLR )
%CURRENT_ZMP_CALCULATOR Summary of this function goes here
%   Detailed explanation goes here
total_force=sum(forceLR);
total_torque=sum(torqueLR);
if (total_force > 30)||( total_force< -30)
    local_zmp=-total_torque/total_force;
else
    local_zmp=0;
end
global_zmp=sum(posLR)/2+local_zmp;

end

