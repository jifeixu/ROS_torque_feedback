function right_ft_callback( ~, msg )
%RIGHT_FT_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global right_ft;

right_ft.force=msg.Wrench.Force;
right_ft.torqueX=msg.Wrench.Torque.X;
right_ft.torqueY=msg.Wrench.Torque.Y;

end

