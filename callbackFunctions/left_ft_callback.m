function left_ft_callback( ~, msg )
%LEFT_FT_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global left_ft;

left_ft.force=msg.Wrench.Force.Z;
left_ft.torqueX=msg.Wrench.Torque.X;
left_ft.torqueY=msg.Wrench.Torque.Y;

end

