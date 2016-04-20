function ground_truth_odom_callback( ~, msg )
%GROUND_TRUTH_ODOM_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global odometer;
global foot_placement;
global joint_angles;
global link_6_pos;
global link_1_pos;
global stance;
global sys_desired;
% odometer have the torso pos and vel.
% whats in link_6_pos:
% link_6_pos.leftx=double(msg.Pose(8).Position.X);
% link_6_pos.lefty=double(msg.Pose(8).Position.Y);
% link_6_pos.rightx=double(msg.Pose(14).Position.X);
% link_6_pos.righty=double(msg.Pose(14).Position.Y);


z_offset=0.16; 

odometer.posX=double(msg.Pose.Pose.Position.X);
odometer.posY=double(msg.Pose.Pose.Position.Y);
odometer.posZ=double(msg.Pose.Pose.Position.Z);
odometer.velX=double(msg.Twist.Twist.Linear.X);
odometer.velY=double(msg.Twist.Twist.Linear.Y);
%  THE Z IS WRONG IN THIS EQUATION>>>>>>>>>>>> NEED MODIFICATION

%foot_position.prz
% [Pkl,joint_angles.th_l]=IK_leg(foot_placement.plx-odometer.posX,foot_placement.ply-odometer.posY,foot_placement.plz,foot_placement.L1,foot_placement.L2);
% [Pkr,joint_angles.th_r]=IK_leg(foot_placement.prx-odometer.posX,foot_placement.pry-odometer.posY,foot_placement.prz,foot_placement.L1,foot_placement.L2);

% [Pkl,joint_angles.th_l]=IK_leg_mod(foot_placement.plx,foot_placement.ply,foot_placement.plz,foot_placement.L1,foot_placement.L2);
% [Pkr,joint_angles.th_r]=IK_leg_mod(foot_placement.prx,foot_placement.pry,foot_placement.prz,foot_placement.L1,foot_placement.L2);

%==============================under construction===================

%if left foot standing, IK calculations..
if stance ==1  % this works!
    [Pkl,joint_angles.th_l]=IK_leg( link_6_pos.leftx-sys_desired.x(1),  ...
                                    link_6_pos.lefty-sys_desired.y(1)-0.0721,  ... left foot offset -0.0721
                                    foot_placement.plz,                 ... should be system desired -(0.70-0.1447-0.1098)-(errordesired-actual), foot_placement.plz-0.1*(-foot_placement.plz-(odometer.posZ-0.1447-0.1098)),
                                    foot_placement.L1,                  ...
                                    foot_placement.L2);
%                                 actual=odometer.posZ-0.1447-0.1098
%                                 desired=-foot_placement.plz
%                                 error=(-foot_placement.plz-(odometer.posZ-0.1447-0.1098))
    [Pkr,joint_angles.th_r]=IK_leg( foot_placement.prx-odometer.posX,   ...
                                    foot_placement.pry-odometer.posY,   ...
                                    -(link_1_pos.rightz-0.1098+foot_placement.plz-foot_placement.prz),	... should be  -(link1-0.1098-(-plz-(-prz))) ?link1 ????invkin
                                    foot_placement.L1,                  ...
                                    foot_placement.L2);
%                       foot_placement.prx
%                       link_1_pos.rightx
% %                     foot_placement.prz
%                     link_6_pos.rightz
%                     -0.1098+foot_placement.plz-foot_placement.prz
%if right foot standing, IK calculations.
elseif stance ==3
    
    [Pkl,joint_angles.th_l]=IK_leg( foot_placement.plx-odometer.posX,   ...
                                    foot_placement.ply-odometer.posY,   ... 
                                    -(link_1_pos.leftz-0.1098+foot_placement.prz-foot_placement.plz),                 ...
                                    foot_placement.L1,                  ...
                                    foot_placement.L2);    
    [Pkr,joint_angles.th_r]=IK_leg( link_6_pos.rightx-sys_desired.x(1),  ...
                                    link_6_pos.righty-sys_desired.y(1)+0.0721,  ...right foot offset 0.0721
                                    foot_placement.plz,                 ...
                                    foot_placement.L1,                  ...
                                    foot_placement.L2);                            
%if at double support
elseif stance ==2
    [Pkl,joint_angles.th_l]=IK_leg( foot_placement.plx-odometer.posX,   ...
                                    foot_placement.ply-odometer.posY,   ...
                                    foot_placement.plz,                 ...
                                    foot_placement.L1,                  ...
                                    foot_placement.L2);
    [Pkr,joint_angles.th_r]=IK_leg( foot_placement.prx-odometer.posX,   ...
                                    foot_placement.pry-odometer.posY,   ...
                                    foot_placement.prz,                 ...
                                    foot_placement.L1,                  ...
                                    foot_placement.L2);

end
%==============================

% 
% plx=llong-X_long; %plx= point, left foot, x axis
% ply=llat-y_offset-X_lat; %ply = point, left foot, y axis= llat- y_offset - COM_lat;
% plz=lh+z_offset-0; %plz
% 
% prx=rlong-X_long;
% pry=rlat+y_offset-X_lat;
% prz=rh+z_offset-0;

end

