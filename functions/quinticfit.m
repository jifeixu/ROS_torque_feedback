function [n] = quinticfit( t1,y1,y1d,t2,y2,y2d )
%QUINTICFIT Summary of this function goes here
%   Detailed explanation goes here
n=[t1^3 t1^2 t1 1; 3*t1^2 2*t1 1 0; t2^3 t2^2 t2 1; 3*t2^2 2*t2 1 0]\[y1 y1d y2 y2d]';
n=n';
end

