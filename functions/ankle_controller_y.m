function [ u ] = ankle_controller_y( desired,actual )
%ANKLE_CONTROLLER Summary of this function goes here
%   desired=[CoM pos, vel, zmp]
%   actual=[CoM pos,vel,zmp]
%   u = control effort

e= (desired-actual)';
desired;
actual;
u=-(500*[-5.1591   -1.4138    0.4595]*e+desired(3));

end

