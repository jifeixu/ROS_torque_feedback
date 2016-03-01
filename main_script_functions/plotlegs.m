function [  ] = plotlegs( Ph, Pk, Pa, color )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

Pf = [Pa(1)+0.1; Pa(2); Pa(3)];

P = [Ph Pk Pa Pf];

plot3(P(1,:),P(2,:),P(3,:), color, 'LineWidth', 3, 'Marker', 'o', 'MarkerFaceColor',([0,0,0]), 'MarkerSize',10);
axis([-1.2 1.2 -1.2 1.2 -1.2 0])
xlabel('x');
ylabel('y');
zlabel('z');
%hold on
%plot3(Pk(1),Pk(2),Pk(3));
grid on
rotate3d on

end

