function link_states_callback( ~, msg )
%LINK_STATES_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global link_6_pos;
global link_1_pos;
global global_zmp;
global alpha;
global left_ft;
global right_ft;
global sys_desired;
global calculated_com;
global clock;
global kalman_state_zmp_x;
global kalman_state_zmp_y;


persistent previous_time;

if isempty(previous_time)
    previous_time=clock;
else
    delta_time=clock.second-previous_time.second; % for realtime value <=10, its actually getting all the info
    previous_time=clock;
end


mass=[0,                                                        ...for ground plane
      12.33 ,                                                   ...base
      1.03  ,0.70   ,3.98   ,2.86   ,0.68   ,2.01   ,           ...left leg
      1.03  ,0.70   ,3.98   ,2.86   ,0.68   ,2.01   ,           ...right leg
      1.58  ,11.38  ,                                           ...torso
      1.54  ,1.60   ,1.64   ,1.2    ,1.22   ,0.42   ,0.86   ,   ...arm_left
      0.06  ,0.03   ,0.02   ,0.06   ,                           ...hand_left_index
      0.06  ,0.03   ,0.02   ,0.06   ,                           ...hand_left_middle
      0.07  ,                                                   ...hand_left_thumb
      1.54  ,1.60   ,1.64   ,1.2    ,1.22   ,0.42   ,0.86   ,   ...arm_right
      0.06  ,0.03   ,0.02   ,0.06   ,                           ...hand_right_index
      0.06  ,0.03   ,0.02   ,0.06   ,                           ...hand_right_middle
      0.07  ,                                                   ...hand_right_thumb
      1.12  ,1.85   ,                                           ...head
      ];

link_6_pos.leftx=double(msg.Pose(8).Position.X);
link_6_pos.lefty=double(msg.Pose(8).Position.Y);
link_6_pos.leftz=double(msg.Pose(8).Position.Z);
link_6_pos.rightx=double(msg.Pose(14).Position.X);
link_6_pos.righty=double(msg.Pose(14).Position.Y);
link_6_pos.rightz=double(msg.Pose(14).Position.Z);


link_1_pos.leftx=double(msg.Pose(3).Position.X);
link_1_pos.lefty=double(msg.Pose(3).Position.Y);
link_1_pos.leftz=double(msg.Pose(3).Position.Z);
link_1_pos.rightx=double(msg.Pose(9).Position.X);
link_1_pos.righty=double(msg.Pose(9).Position.Y);
link_1_pos.rightz=double(msg.Pose(9).Position.Z);

% global_zmp.x=current_ZMP_calculator([left_ft.torqueY,right_ft.torqueY],...
%                                     [left_ft.force,right_ft.force],...
%                                     [link_6_pos.leftx,link_6_pos.rightx]);

zmpx=current_ZMP_calculator([left_ft.torqueY,right_ft.torqueY],...
                                    [left_ft.force,right_ft.force],...
                                    [link_6_pos.leftx,link_6_pos.rightx]);
[kalman_state_zmp_x,global_zmp.x]=Kal_filter(kalman_state_zmp_x,zmpx);                                
                                
% global_zmp.y=current_ZMP_calculator([left_ft.torqueX,right_ft.torqueX],...
%                                     [left_ft.force,right_ft.force],...
%                                     [link_6_pos.lefty,link_6_pos.righty]);
zmpy=current_ZMP_calculator([left_ft.torqueX,right_ft.torqueX],...
                                    [left_ft.force,right_ft.force],...
                                    [link_6_pos.lefty,link_6_pos.righty]);
[kalman_state_zmp_y,global_zmp.y]=Kal_filter(kalman_state_zmp_y,zmpy);                                
                                
                                
alpha=alpha_generator(msg.Pose(8).Position.X,msg.Pose(8).Position.Y,...
                      msg.Pose(14).Position.X,msg.Pose(14).Position.Y,...
                      sys_desired.x(3),sys_desired.y(3));
%                   global_zmp.x,global_zmp.y);
                  %sys_desired.x(3),sys_desired.y(3));
                      %should change global_zmp.x to desired zmp sometime
                      %later?
X_=zeros(size(msg.Pose));
Y_=zeros(size(msg.Pose));
Z_=zeros(size(msg.Pose));
M=sum(mass);
for i=1:length(msg.Pose)
    X_(i)=msg.Pose(i).Position.X;
    Y_(i)=msg.Pose(i).Position.Y;
    Z_(i)=msg.Pose(i).Position.Z;
end

calculated_com.X_com=mass*X_/M;
calculated_com.Y_com=mass*Y_/M;
calculated_com.Z_com=mass*Z_/M;
end

