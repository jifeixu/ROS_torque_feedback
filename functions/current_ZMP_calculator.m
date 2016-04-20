function [ global_zmp ] = current_ZMP_calculator( torqueLR,forceLR,posLR )
%CURRENT_ZMP_CALCULATOR Summary of this function goes here
%   Detailed explanation goes here

%% this implementation seems errorneos
% total_force=sum(forceLR);
% total_torque=sum(torqueLR);
% if (total_force > 30)||( total_force< -30)
%     local_zmp=-total_torque/total_force;
% else
%     local_zmp=0;
% end
% global_zmp=sum(posLR)/2+local_zmp;
%%
zmp=[0 0];

% if forceLR(1)<=25 || forceLR(2)<=25                         %if any one of the foot is not in contact.
for i=1:2
    if (forceLR(i)>0)
        zmp(i)=-torqueLR(i)/forceLR(i)+posLR(i);
    end
end

global_zmp=sum(zmp);
    
    
end

