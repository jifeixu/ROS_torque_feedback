function [ a ] = alpha_generator(plx,ply,prx,pry,zmpx,zmpy)
%ALPHA_GENERATOR Summary of this function goes here
%   % plx=position left x, ply position left y etc..

%added x_front 
% x_front=0.05;

% leftDistance=sqrt((zmpx-(plx+x_front))^2+(zmpy-ply)^2);
% rightDistance=sqrt((zmpx-(prx+x_front))^2+(zmpy-pry)^2);
leftDistance=sqrt((zmpx-plx)^2+(zmpy-ply)^2);
rightDistance=sqrt((zmpx-prx)^2+(zmpy-pry)^2);
c=0.03;% the range zmp that is deem to be within a foot

if leftDistance<=c
    a=1;
elseif rightDistance<=c
    a=0;
else
    a=(rightDistance-c)/(leftDistance+rightDistance-2*c);
end

end

