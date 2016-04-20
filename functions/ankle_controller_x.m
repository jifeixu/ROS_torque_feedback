function [ u ] = ankle_controller_x( desired,actual )
%ANKLE_CONTROLLER Summary of this function goes here
%   desired=[CoM pos, vel, zmp]
%   actual=[CoM pos,vel,zmp]
%   u = control effort

e= (desired-actual)';

% u=[-3235 -150 20.5]*e+desired(3);
u=350*[ -4.6685   -2.4899    1.0228]*e+desired(3);
end

