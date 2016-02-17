%% connect to ROS and excute the thetas?
close all 
clear all
load('traj');
rosshutdown;
% ipaddress='172.17.5.208';
rosinit('128.174.226.38','NodeHost','172.17.128.146');
% rosinit('128.174.226.38');
% rosinit('172.17.83.139','NodeHost','172.17.119.92');
% rosinit('172.17.19.199','NodeHost','172.17.195.97');%,'NodeHost','172.17.183.168');
%% global variables
% used for callback functions and are continuously updated
global link_6_pos;  %link_state_sub_callback
       link_6_pos=struct('leftx',0,'lefty',0,'rightx',0,'righty',0);
global right_ft;   
       right_ft=struct('force',0,'torqueX',0,'torqueY',0);
global left_ft;
       left_ft=struct('force',0,'torqueX',0,'torqueY',0);
global odometer;
       odometer=struct('posX',0,'posY',0,'velX',0,'velY',0);
global global_zmp;  %link_state_sub_callback
       global_zmp=struct('x',0,'y',0);
global alpha;
       alpha=0.5;
global u_effort;        % outputs, they should be directly outputed into the publisher
       u_effort=struct('right_joint_5',0,'right_joint_6',0,'left_joint_5',0,'left_joint_6',0);
global sys_desired;     % imputs, modify within while 
       sys_desired=struct('x',[0 0 0],'y',[0 0 0]);
global ankle_joint_ref; % inputs, for position control when F<=0
       ankle_joint_ref=struct('left_joint_5_pos_ref',0,'right_joint_5_pos_ref',0,'left_joint_6_pos_ref',0,'right_joint_6_pos_ref',0);
% global joint_states;
%        joint_states=struct(

%% subscribers, with callback.
link_states_sub=rossubscriber('/gazebo/link_states',@link_states_callback);
right_ft_sub=rossubscriber('/right_ft',@right_ft_callback);
left_ft_sub=rossubscriber('/left_ft',@left_ft_callback);
ground_truth_odom_sub=rossubscriber('ground_truth_odom',@ground_truth_odom_callback);
joint_states_sub=rossubscriber('/joint_states',@joint_states_callback);
%% publishers:
right_leg_joint_5_effort_pub=rospublisher('/right_leg_effort_controller2/command','std_msgs/Float64');
left_leg_joint_5_effort_pub=rospublisher('/left_leg_effort_controller2/command','std_msgs/Float64');
right_leg_joint_6_effort_pub=rospublisher('/right_leg_effort_controller3/command','std_msgs/Float64');
left_leg_joint_6_effort_pub=rospublisher('/left_leg_effort_controller3/command','std_msgs/Float64');

%% Controller

