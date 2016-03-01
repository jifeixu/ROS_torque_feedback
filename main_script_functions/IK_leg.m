function [ Pk,th ] = IK_leg( xa,ya,za, L1, L2 )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% z < 0 !!
% xa = Pa(1); % assume Pa is ankle points
% ya = Pa(2);
% za = Pa(3);

L3 = sqrt(xa.^2+ya.^2+za.^2);

th4 = pi - acos((L1.^2+L2.^2-L3.^2)./(2.*L1.*L2));
th6 = -atan2(ya,abs(za));
th5 = atan2(xa,sqrt(ya.^2+za.^2)) - acos((L3.^2+L2.^2-L1.^2)./(2.*L2.*L3));
th1 = zeros(size(th4));
th2 = -th6;
th3 = -(atan2(xa,sqrt(ya.^2+za.^2)) + acos((L3.^2+L1.^2-L2.^2)./(2.*L1.*L3)));

xk = L1.*sin(-th3);
zk = -L1.*cos(-th3).*cos(-th2);
yk = -L1.*cos(-th3).*sin(-th2);

Pk = [xk;yk;zk]; % i assume Pk is knee points.
Ph = [0;0;0];

Pf = [xa+0.1; ya; za];

% P = [Ph Pk Pa Pf];

th = [th1; th2; th3; th4; th5; th6];


end

