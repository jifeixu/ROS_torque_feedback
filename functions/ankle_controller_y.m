function [ u ] = ankle_controller_y( desired,actual )
%ANKLE_CONTROLLER Summary of this function goes here
%   desired=[CoM pos, vel, zmp]
%   actual=[CoM pos,vel,zmp]
%   u = control effort

e= (desired-actual)';

% u=-(400*[ -3.3772   -0.9026    0.1322]*e+desired(3));
% u=-([-3235 -150 20.5]*e+desired(3));
u=-(300*[    -3.3772   -0.9026    0.1322]*e+desired(3));
end

