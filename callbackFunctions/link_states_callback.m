function link_states_callback( ~, msg )
%LINK_STATES_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global link_6_pos;
global global_zmp;
global alpha;

link_6_pos.leftx=msg.Pose(8).Position.X;
link_6_pos.lefty=msg.Pose(8).Position.Y;
link_6_pos.rightx=msg.Pose(14).Position.X;
link_6_pos.righty=msg.Pose(14).Position.Y;

global_zmp.x=current_ZMP_calculator([left_ft.torqueY,right_ft.torqueY],...
                                    [left_ft.force,right_ft.force],...
                                    [link_6_pos.leftx,link_6_pos.rightx]);
global_zmp.y=current_ZMP_calculator([left_ft.torqueX,right_ft.torqueX],...
                                    [left_ft.force,right_ft.force],...
                                    [link_6_pos.lefty,link_6_pos.righty]);
                      
alpha=alpha_generator(msg.Pose(8).Position.X,msg.Pose(8).Position.Y,...
                      msg.Pose(14).Position.X,msg.Pose(14).Position.Y,...
                      global_zmp.x,global_zmp.y);
                      %should change global_zmp.x to desired zmp sometime
                      %later?


end

