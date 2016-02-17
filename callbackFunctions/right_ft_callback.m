function right_ft_callback( ~, msg )
%RIGHT_FT_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global right_ft;

right_ft.force=double(msg.Wrench.Force.Z);
right_ft.torqueX=double(msg.Wrench.Torque.X);
right_ft.torqueY=double(msg.Wrench.Torque.Y);

end

