%% connect to ROS and excute the thetas?
close all 
clear all
load('traj_10cm_height');
dX_long=gradient(X_long);
dX_lat=gradient(X_lat);
rosshutdown;
% ipaddress='172.17.5.208';
rosinit('192.168.219.128','NodeHost','172.17.205.74');
% rosinit('128.174.226.38');
% rosinit('172.17.83.139','NodeHost','172.17.119.92');
% rosinit('172.17.19.199','NodeHost','172.17.195.97');%,'NodeHost','172.17.183.168');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
global sys_desired;     % imputs, modify within while [com dcom zmp] desired
       sys_desired=struct('x',[0 0 0],'y',[0 0 0]);
global ankle_joint_ref; % inputs, for position control when F<=0
       ankle_joint_ref=struct('left_joint_5_pos_ref',0,'right_joint_5_pos_ref',0,'left_joint_6_pos_ref',0,'right_joint_6_pos_ref',0);
global clock;           % clock, keep track of sim time
       clock=struct('Sec',0,'Nsec',0,'second',0);
global time_management;
       time_management=struct('t0sec',0,'t0nsec',0,'start_time',0,'end_time',0,'i',0);
global calculated_com
       calculated_com=struct('X_com',0,'Y_com',0,'Z_com',0);
% global joint_states;
%        joint_states=struct(
%% publishers:
%effort controllers for ankle
global right_leg_joint_5_effort_pub;
global left_leg_joint_5_effort_pub;
global right_leg_joint_6_effort_pub;
global left_leg_joint_6_effort_pub;
global ankle_control_on;
        ankle_control_on=0;

        right_leg_joint_5_effort_pub=rospublisher('/right_leg_effort_controller2/command','std_msgs/Float64');
        left_leg_joint_5_effort_pub=rospublisher('/left_leg_effort_controller2/command','std_msgs/Float64');
        right_leg_joint_6_effort_pub=rospublisher('/right_leg_effort_controller3/command','std_msgs/Float64');
        left_leg_joint_6_effort_pub=rospublisher('/left_leg_effort_controller3/command','std_msgs/Float64');
% position controller for everything else
%joint trajectory position publishers
publeg_r = rospublisher('/right_leg_controller/command','trajectory_msgs/JointTrajectory');
publeg_l = rospublisher('/left_leg_controller/command','trajectory_msgs/JointTrajectory');
pub_tor = rospublisher('/torso_controller/command','trajectory_msgs/JointTrajectory');
%set model state for resetting model position
setpose = rospublisher('/gazebo/set_model_state');
%% publisher msg
%message for torques
global right_leg_joint_5_effort_msg;
global left_leg_joint_5_effort_msg;
global right_leg_joint_6_effort_msg;
global left_leg_joint_6_effort_msg;

right_leg_joint_5_effort_msg=rosmessage('std_msgs/Float64');
left_leg_joint_5_effort_msg=rosmessage('std_msgs/Float64');
right_leg_joint_6_effort_msg=rosmessage('std_msgs/Float64');
left_leg_joint_6_effort_msg=rosmessage('std_msgs/Float64');
%msg for legs, torso position control
theta_r = rosmessage(publeg_r);
theta_l = rosmessage(publeg_l);
theta_tor=rosmessage(pub_tor);
%message for reset pose
pose = rosmessage(setpose);
% trajectory msg for set of points
pl = rosmessage('trajectory_msgs/JointTrajectoryPoint'); 
pr = rosmessage('trajectory_msgs/JointTrajectoryPoint');
ptor = rosmessage('trajectory_msgs/JointTrajectoryPoint');
%% subscribers, without callback
% subscriber define joint names
right_leg_state_sub=rossubscriber('/right_leg_controller/state');
left_leg_state_sub=rossubscriber('/left_leg_controller/state');
tor_state_sub = rossubscriber('/torso_controller/state');
%% initializations for joint names,
%recieve names and assigne joint names
rleg_state=receive(right_leg_state_sub);
lleg_state=receive(left_leg_state_sub);
tor_state = receive(tor_state_sub);

theta_r.JointNames = rleg_state.JointNames;
theta_l.JointNames = lleg_state.JointNames;
theta_tor.JointNames = tor_state.JointNames;
%% ros clients 
% rosclients
pause_gazebo = rossvcclient('/gazebo/pause_physics');
unpause_gazebo = rossvcclient('/gazebo/unpause_physics');
reset_simulation = rossvcclient('/gazebo/reset_simulation');
reset_world = rossvcclient('/gazebo/reset_world');
%% subscribers, with callback. let callback function subs run last. cuz it SLOWS DOWN EVERYTHING.
link_states_sub=rossubscriber('/gazebo/link_states',@link_states_callback);
right_ft_sub=rossubscriber('/right_ft',@right_ft_callback);
left_ft_sub=rossubscriber('/left_ft',@left_ft_callback);
ground_truth_odom_sub=rossubscriber('/ground_truth_odom',@ground_truth_odom_callback);
joint_states_sub=rossubscriber('/joint_states',@joint_states_callback);
clock_sub = rossubscriber('/clock',@clock_callback);

sys_desired.x=[0 0 0];
sys_desired.y=[0.09 0 0.065];



%% only stabilizing while standing.
sys_desired.x=[0 0 0];
sys_desired.y=[0.09 0 0.065];

global KEY_IS_PRESSED
KEY_IS_PRESSED = 0;
gcf
set(gcf, 'KeyPressFcn', @thisKeyPressFcn)

i=1;
pause(1)
ankle_control_on=1;     
while(~KEY_IS_PRESSED)
    i=i+1;
    pause(0.0001)
%     disp(['looping...' num2str(i),'with i ',', alpha ',num2str(alpha)]);
%     disp(['com x ' num2str(calculated_com.X_com),' com y  ',num2str(calculated_com.Y_com),', zmpx ',num2str(global_zmp.x),', zmpy ',num2str(global_zmp.y)]);

    
end
ankle_control_on=0;
disp('loop ended')
%% Loop
call(pause_gazebo);
% need to write while loop just to publish all the efforts.
sys_desired.x=[0 0 0];
sys_desired.y=[0.09 0 0.065];
%%%%%%%%%%%%% a press any key to stop loop method
global KEY_IS_PRESSED
KEY_IS_PRESSED = 0;
gcf;
set(gcf, 'KeyPressFcn', @thisKeyPressFcn)
%%%%%%%%%%%%%%%
%copy from the test main:
i=1;
dt=0.001;
n=length(t);
dt_Nsec = dt*10^9;
SEC = [];
NSEC = [];
% theclock = receive(clock_sub); ignored by clock callback.

time_management.t0sec=clock.Sec;    % t0sec=clock.Sec;
time_management.t0nsec=clock.Nsec+0.1*10^9;  % t0nsec=clock.Nsec;
time_management.start_time=time_management.t0sec + time_management.t0nsec/10^9;%starttime = t0sec + t0nsec/10^9;
time_management.end_time=time_management.start_time+t(end);%endtime = starttime+t(end);

time=time_management.start_time;
% rzmp_calc=[]; % [rzmp_calc;x y;]  %they are given in global_zmp
% lzmp_calc=[]; % [lzmp_calc;x y;]
ZMP_time=[];                        % keeps track of time, still useful
% X_com=[];                         % use odobmeter instead. for now.
% Y_com=[];
% Z_com=[];

pause(2)
timeout=0.1;
while(~KEY_IS_PRESSED)
    ankle_control_on=1;
    i=i+1;
    pause(0.1);
    disp(['looping...' num2str(i),'with i ',num2str(time_management.i),', alpha ',num2str(alpha),' actual i is ',num2str(time_management.i-timeout*1000)]);

    if(time_management.i <= 4200 && time_management.i >0)
        if(time_management.i>timeout*1000)
            ankle_joint_ref.left_joint_5_pos_ref=th_l(time_management.i-timeout*1000,5);
            ankle_joint_ref.left_joint_6_pos_ref=th_l(time_management.i-timeout*1000,6);
            ankle_joint_ref.right_joint_5_pos_ref=th_r(time_management.i-timeout*1000,5);
            ankle_joint_ref.right_joint_6_pos_ref=th_r(time_management.i-timeout*1000,6);
            sys_desired.x=[X_long(time_management.i-timeout*1000),dX_long(time_management.i-timeout*1000),Z_long(time_management.i-timeout*1000)];
            sys_desired.y=[X_lat(time_management.i-timeout*1000),dX_lat(time_management.i-timeout*1000),Z_lat(time_management.i-timeout*1000)];
        end
%         global sys_desired;     % imputs, modify within while
%        sys_desired=struct('x',[0 0 0],'y',[0 0 0]); [com dcom zmp] desired
%         global ankle_joint_ref; % inputs, for position control when F<=0
%        ankle_joint_ref=struct('left_joint_5_pos_ref',0,'right_joint_5_pos_ref',0,'left_joint_6_pos_ref',0,'right_joint_6_pos_ref',0);
        pl.Positions = th_l(time_management.i,1:4)';
        pr.Positions = th_r(time_management.i,1:4)';
        theta_l.Points = pl;
        theta_r.Points = pr;
        
        theta_l.Points.TimeFromStart.Sec = 0;
        theta_r.Points.TimeFromStart.Sec = 0;
        theta_l.Points.TimeFromStart.Nsec = dt_Nsec;
        theta_r.Points.TimeFromStart.Nsec = dt_Nsec;
                
        theta_l.Header.Stamp.Sec  = clock.Sec;
        theta_l.Header.Stamp.Nsec = clock.Nsec+timeout*10^9;
        theta_r.Header.Stamp.Sec  = clock.Sec;
        theta_r.Header.Stamp.Nsec = clock.Nsec+timeout*10^9;
        
        
        send(publeg_l,theta_l);
        send(publeg_r,theta_r);
        
    end
    
    
end
ankle_control_on=0;
sys_desired.x=[0 0 0];
sys_desired.y=[0.09 0 0.065];
disp('loop ended')