function left_ft_callback( ~, msg )
%LEFT_FT_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global left_ft;

left_ft.force=double(msg.Wrench.Force.Z);
left_ft.torqueX=double(msg.Wrench.Torque.X);
left_ft.torqueY=double(msg.Wrench.Torque.Y);



end

