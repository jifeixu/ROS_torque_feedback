function link_states_callback( ~, msg )
%LINK_STATES_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global link_6_pos;

link_6_pos.leftx=msg.Pose(8).Position.X;
link_6_pos.lefty=msg.Pose(8).Position.Y;
link_6_pos.rightx=msg.Pose(14).Position.X;
link_6_pos.righty=msg.Pose(14).Position.Y;


end

