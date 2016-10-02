%% calculate everything.
clear all;
close all;
rosshutdown 

%%
zh=0.7; %????pole placement??z ???? ankle controller???gain
[Z_lat,X_lat,t_lat]=test_fn_lat(4,1,zh); % foot y direction foot scale should stay 1
[Z_long,X_long,t_long]=test_fn_long(4,0.5,zh); % changed 3rd step length to half step
X_h=zeros(size(X_lat));
% lh and rh assumes the ground plane is at z=-(distance from center of mass from calculation).
[ llat,llong,lh,rlat,rlong,rh ] = test_fn_foot_placement(4,0.5,zh); 
% %%
% % long is ladder(X), lat is zigzag(Y)
% figure(1)
% plot3(t_lat,Z_lat,Z_long,t_lat,X_lat,X_long);
% grid
%% plot CoM (X) position with Left Position(l) Right Position(r)
plot3(llat,llong,lh,rlat,rlong,rh,X_lat,X_long,X_h,Z_lat,Z_long,-zh*ones(size(Z_long)));
grid
xlabel('x');
ylabel('y');
zlabel('z');
%%
% clear all;
% close all;
L1 = 0.3;
L2 = 0.3;
y_offset=0.075;%0.0721?????
z_offset=0.14; 

z_footfromground=0.109844722053;

%4/19 try add a offset to X such that x is toward the front.
% x_front=0.05; %xfront also added in alpha generator for compensation.

% p points are points relative to its respective origin, namely, the hip
% point. for IK_leg calculation.
plx=llong-X_long;%-x_front; %plx= point, left foot, x axis
ply=llat-y_offset-X_lat; %ply = point, left foot, y axis= llat- y_offset - COM_lat;
plz=lh+z_offset+z_footfromground; %plz

prx=rlong-X_long;%-x_front;
pry=rlat+y_offset-X_lat;
prz=rh+z_offset+z_footfromground;

foot_position=struct('prx',rlong,'pry',rlat+y_offset,'prz',prz,'plx',llong,'ply',llat-y_offset,'plz',plz,'L1',L1,'L2',L2);


[Pkl,th_l]=IK_leg(plx,ply,plz,L1,L2);
[Pkr,th_r]=IK_leg(prx,pry,prz,L1,L2);

%change th_l from 6*n, to n*6
th_l=th_l';
th_r=th_r';
t=t_lat;
save('traj','th_l','th_r','t','Z_lat','Z_long','X_lat','X_long','foot_position');
%% 3D plot of the walking robot legs
for i=1:500:floor(size(th_l,1)/2)
    Ph_l=[X_long(i),X_lat(i)+y_offset,-z_offset]';
    Ph_r=[X_long(i),X_lat(i)-y_offset,-z_offset]';
    figure(1);
    plotlegs(Ph_l, Pkl(:,i)+Ph_l,[plx(i),ply(i),plz(i)]'+Ph_l ,'r');
    hold on;
    plotlegs(Ph_r, Pkr(:,i)+Ph_r,[prx(i),pry(i),prz(i)]'+Ph_r,'g');
    view([0 1 0])
    hold off;
    pause(0.1)
end

%% connect to ROS and excute the thetas?
% close all 
% clear all
% load('traj');
% =============================
% %%
rosshutdown;
% ipaddress='172.17.5.208';
rosinit('192.168.159.128','NodeHost','128.174.226.90');
% rosinit('128.174.226.38');
% rosinit('172.17.83.139','NodeHost','172.17.119.92');
% rosinit('172.17.19.199','NodeHost','172.17.195.97');%,'NodeHost','172.17.183.168');
% ==============================
% %%  
% publisher
publeg_r = rospublisher('/right_leg_controller/command','trajectory_msgs/JointTrajectory');
publeg_l = rospublisher('/left_leg_controller/command','trajectory_msgs/JointTrajectory');
pub_tor = rospublisher('/torso_controller/command','trajectory_msgs/JointTrajectory');
setpose = rospublisher('/gazebo/set_model_state');

% msg for legs, torso
theta_r = rosmessage(publeg_r);
theta_l = rosmessage(publeg_l);

theta_tor=rosmessage(pub_tor);

% subscriber define joint names
right_leg_state_sub=rossubscriber('/right_leg_controller/state');
left_leg_state_sub=rossubscriber('/left_leg_controller/state');
tor_state_sub = rossubscriber('/torso_controller/state');
right_ft_sub=rossubscriber('/right_ft');
left_ft_sub=rossubscriber('/left_ft');
%
rleg_state=receive(right_leg_state_sub);
lleg_state=receive(left_leg_state_sub);
tor_state = receive(tor_state_sub);

theta_r.JointNames = rleg_state.JointNames;
theta_l.JointNames = lleg_state.JointNames;
theta_tor.JointNames = tor_state.JointNames;

% force sensor receives
rft_sen=receive(right_ft_sub);
lft_sen=receive(left_ft_sub);

% subscribe to clock, joint states, publisher for reset model states etc.
clock_sub = rossubscriber('/clock');
read_joints = rossubscriber('/joint_states');
gazebo_link_states_sub=rossubscriber('/gazebo/link_states');
pose = rosmessage(setpose);
% trajectory msg for set of points
pl = rosmessage('trajectory_msgs/JointTrajectoryPoint'); 
pr = rosmessage('trajectory_msgs/JointTrajectoryPoint');
ptor = rosmessage('trajectory_msgs/JointTrajectoryPoint');
% rosclients
pause_gazebo = rossvcclient('/gazebo/pause_physics');
unpause_gazebo = rossvcclient('/gazebo/unpause_physics');
reset_simulation = rossvcclient('/gazebo/reset_simulation');
reset_world = rossvcclient('/gazebo/reset_world');

%MASS definition:
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
  
gazebo_link_states=receive(gazebo_link_states_sub);

X_=zeros(size(gazebo_link_states.Pose));
Y_=zeros(size(gazebo_link_states.Pose));
Z_=zeros(size(gazebo_link_states.Pose));

M=sum(mass);
%% reset1
% response = call(unpause_gazebo);
% pr.Positions = [0 0 0 0];
% pl.Positions = [0 0 0 0 ];

% pl.Positions = th_l(1,[1:4 6])';
% pr.Positions = th_r(1,[1:4 6])';

pl.Positions = th_l(1,1:4)';
pr.Positions = th_r(1,1:4)';

theta_l.Points = pl;
theta_r.Points = pr;
theta_l.Points.TimeFromStart.Sec = 1;
theta_r.Points.TimeFromStart.Sec = 1;
theclock=receive(clock_sub);
theta_l.Header.Stamp.Sec = theclock.Clock_.Sec;
theta_l.Header.Stamp.Nsec = theclock.Clock_.Nsec;
theta_r.Header.Stamp.Sec = theclock.Clock_.Sec;
theta_r.Header.Stamp.Nsec =theclock.Clock_.Nsec;

send(publeg_l,theta_l);
send(publeg_r,theta_r);

% pose.ModelName = 'reemc_lower_body';
pose.ModelName = 'reemc_full';
pose.ReferenceFrame = 'world';
pose.Pose.Position.X =-0.022;%0.58 0.377 0.46 % 0.05 due to the x_front
pose.Pose.Position.Y = 0.076;
% pose.Pose.Position.Z = 0.725; % when foot elevation is 10cm?
pose.Pose.Position.Z = 0.70;


send(setpose,pose);
% %% get CoM?
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
% gazebo_link_states2.Pose(55)=gazebo_link_states.Pose(1)
%subscribe to /gazebo/link_states

%do once
gazebo_link_states=receive(gazebo_link_states_sub);

X_=zeros(size(gazebo_link_states.Pose));
Y_=zeros(size(gazebo_link_states.Pose));
Z_=zeros(size(gazebo_link_states.Pose));

M=sum(mass);

% repeat
gazebo_link_states=receive(gazebo_link_states_sub);


for i=1:length(gazebo_link_states.Pose)
    X_(i)=gazebo_link_states.Pose(i).Position.X;
    Y_(i)=gazebo_link_states.Pose(i).Position.Y;
    Z_(i)=gazebo_link_states.Pose(i).Position.Z;
end

X_com=mass*X_/M
Y_com=mass*Y_/M
Z_com=mass*Z_/M
% call(pause_gazebo);

%% open loop walking
i=1;
dt=0.001;
n=length(t);
dt_Nsec = dt*10^9;
SEC = [];
NSEC = [];
theclock = receive(clock);
response = call(pause_gazebo);

t0sec = double(theclock.Clock_.Sec);
t0nsec = double(theclock.Clock_.Nsec);
% starttime = gett0.Header.Stamp.Sec*10^9 + gett0.Header.Stamp.Nsec;
starttime = t0sec + t0nsec/10^9;
endtime = starttime+t(end);
% for i = 1 : n;
%     NSEC = [NSEC rem((t0nsec+dt*i),1)*10^9];
%     SEC = [SEC floor((t0sec+(t0nsec+dt*i)))];    
% end
% endtime = SEC(end) + NSEC(end)/10^9;

response = call(unpause_gazebo);

time=starttime;

rzmp_calc=[]; % [rzmp_calc;x y;]
lzmp_calc=[]; % [lzmp_calc;x y;]
ZMP_time=[];
X_com=[];
Y_com=[];
Z_com=[];

while(time <= endtime+0.2);
    %listen to topic and get variables
    theclock=receive(clock);
    rft_sen=receive(right_ft_sub);
    lft_sen=receive(left_ft_sub);
    gazebo_link_states=receive(gazebo_link_states_sub);
     
    for i=1:length(gazebo_link_states.Pose)
        X_(i)=gazebo_link_states.Pose(i).Position.X;
        Y_(i)=gazebo_link_states.Pose(i).Position.Y;
        Z_(i)=gazebo_link_states.Pose(i).Position.Z;
    end
    
    X_com=[X_com mass*X_/M];
    Y_com=[Y_com mass*Y_/M];
    Z_com=[Z_com mass*Z_/M];

    
    %find the current time
    tsec = double(theclock.Clock_.Sec);
    tnsec = double(theclock.Clock_.Nsec);
    time = tsec + tnsec/10^9;
    i=floor((tsec-t0sec)*1000+(tnsec-t0nsec)/(10^6));
    
    if(rft_sen.Wrench.Force.Z> 50)
        rzmp_calc=[rzmp_calc; -rft_sen.Wrench.Torque.X/rft_sen.Wrench.Force.Z,...
            -rft_sen.Wrench.Torque.Y/rft_sen.Wrench.Force.Z];
    else
        rzmp_calc=[rzmp_calc;0,0];
    end
    if(lft_sen.Wrench.Force.Z>50)
        lzmp_calc=[lzmp_calc; -lft_sen.Wrench.Torque.X/lft_sen.Wrench.Force.Z,...
            -lft_sen.Wrench.Torque.Y/lft_sen.Wrench.Force.Z];
    else
        lzmp_calc=[lzmp_calc;0,0];
    end
    ZMP_time=[ZMP_time,time];
    
    if(i <= 4200 && i >0)
%         if(rft_sen.Wrench.Force.Z> 50)
%             rzmp_calc=[rzmp_calc; -rft_sen.Wrench.Torque.X/rft_sen.Wrench.Force.Z,...
%                                   -rft_sen.Wrench.Torque.Y/rft_sen.Wrench.Force.Z];
%         else
%             rzmp_calc=[rzmp_calc;0,0];
%         end
%         if(lft_sen.Wrench.Force.Z>50)
%             lzmp_calc=[lzmp_calc; -lft_sen.Wrench.Torque.X/lft_sen.Wrench.Force.Z,...
%                                   -lft_sen.Wrench.Torque.Y/lft_sen.Wrench.Force.Z];
%         else
%             lzmp_calc=[lzmp_calc;0,0];
%         end
%         ZMP_time=[ZMP_time,time];
        pl.Positions = th_l(i,:)';
        pr.Positions = th_r(i,:)';
        theta_l.Points = pl;
        theta_r.Points = pr;
        
        theta_l.Points.TimeFromStart.Sec = 0;
        theta_r.Points.TimeFromStart.Sec = 0;
        theta_l.Points.TimeFromStart.Nsec = dt_Nsec;
        theta_r.Points.TimeFromStart.Nsec = dt_Nsec;
                
        theta_l.Header.Stamp.Sec = tsec+1;
        theta_l.Header.Stamp.Nsec =tnsec;
        theta_r.Header.Stamp.Sec = tsec+1;
        theta_r.Header.Stamp.Nsec =tnsec;
        
        
        send(publeg_l,theta_l);
        send(publeg_r,theta_r);
        
    end
    
  
%     joint_angle(i) = receive(read_joints);
%     t_sec = double(joint_angle(i).Header.Stamp.Sec);
%     t_nsec = double(joint_angle(i).Header.Stamp.Nsec)/10^9;
%     time = t_sec + t_nsec;   
    
    
end
%% below are TEST FUNCTIONALITIES, USE WITH CAUTION.
%% plot angles
 starttime = double(joint_angle(1).Header.Stamp.Sec) + double(joint_angle(1).Header.Stamp.Nsec)/(10^9);
 
 for i = 1 : length(joint_angle)
     t_joint(i) = double(joint_angle(i).Header.Stamp.Sec) + double(joint_angle(i).Header.Stamp.Nsec)/(10^9) - starttime;
     angle_l(:,i) = joint_angle(i).Position(1:6);
     angle_r(:,i) = joint_angle(i).Position(7:12);
 end
 
 plot(t_joint,angle_l(3,:),t,th_l(:,3))
 legend('actual','desired')
 %% lean forward
 ptor.Positions = [0;0.03];
 theta_tor.Points=ptor;
 theta_tor.Points.TimeFromStart.Nsec = 1;
theclock=receive(clock_sub);
theta_tor.Header.Stamp.Sec = theclock.Clock_.Sec;
theta_tor.Header.Stamp.Nsec = theclock.Clock_.Nsec;
send(pub_tor,theta_tor);
%% plot ZMP?
plot(ZMP_time,(rzmp_calc+lzmp_calc)/2); % local zmp. need to find global zmp
%% get CoM?
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
% gazebo_link_states2.Pose(55)=gazebo_link_states.Pose(1)
%subscribe to /gazebo/link_states

%do once
gazebo_link_states=receive(gazebo_link_states_sub);

X_=zeros(size(gazebo_link_states.Pose));
Y_=zeros(size(gazebo_link_states.Pose));
Z_=zeros(size(gazebo_link_states.Pose));

M=sum(mass);

% repeat
gazebo_link_states=receive(gazebo_link_states_sub);


for i=1:length(gazebo_link_states.Pose)
    X_(i)=gazebo_link_states.Pose(i).Position.X;
    Y_(i)=gazebo_link_states.Pose(i).Position.Y;
    Z_(i)=gazebo_link_states.Pose(i).Position.Z;
end

X_com=mass*X_/M
Y_com=mass*Y_/M
Z_com=mass*Z_/M
%% CoM plot
plot3(X_com,Y_com,Z_com,X_long, X_lat,0.75*ones(size(X_long)))
grid
zlim([0.48 1]);
xlim([-0.02 0.5]);
ylim([-0.23 0.25]);

% modified ZMP? or global ZMP? under knowledge of positioin of left leg6
% and right leg 6?
%% test effort controller
publeg_eff_r = rospublisher('/right_leg_effort_controller/command','trajectory_msgs/JointTrajectory');
publeg_eff_l = rospublisher('/left_leg_effort_controller/command','trajectory_msgs/JointTrajectory');

theta_eff_r = rosmessage(publeg_eff_r);
theta_eff_l = rosmessage(publeg_eff_l);

right_leg_eff_state_sub=rossubscriber('/right_leg_effort_controller/state');
left_leg_eff_state_sub=rossubscriber('/left_leg_effort_controller/state');

rleg_eff_state=receive(right_leg_eff_state_sub);
lleg_eff_state=receive(left_leg_eff_state_sub);

theta_eff_r.JointNames = rleg_eff_state.JointNames;
theta_eff_l.JointNames = lleg_eff_state.JointNames;

pl_eff = rosmessage('trajectory_msgs/JointTrajectoryPoint'); 
pr_eff = rosmessage('trajectory_msgs/JointTrajectoryPoint');
%% test joint effort controllers
pl_eff.Positions = [0;0];
pr_eff.Positions = [0;0];


theta_eff_l.Points = pl_eff;
theta_eff_r.Points = pr_eff;
theta_eff_l.Points.TimeFromStart.Sec = 1;
theta_eff_r.Points.TimeFromStart.Sec = 1;
theclock=receive(clock);
theta_eff_l.Header.Stamp.Sec = theclock.Clock_.Sec;

theta_eff_l.Header.Stamp.Nsec = theclock.Clock_.Nsec;
theta_eff_r.Header.Stamp.Sec = theclock.Clock_.Sec;
theta_eff_r.Header.Stamp.Nsec =theclock.Clock_.Nsec;

send(publeg_eff_l,theta_eff_l);
send(publeg_eff_r,theta_eff_r);
%% ROS callback function and its utility.
% use dx/dt= Ax+Bu, Tp=0.05s , g=9.8 , z= 0.7
% where A=[0    1   0   ] , B= [0].   Modeled p = 1
%         [g/z  0   -g/z]      [0]                ----pd
%         [0    0   -1/Tp]     [Tp]              1+sTp
% and u=pd*=[k1 k2 k3]*[ xd- x] + pd;
%                      [dxd- dx]
%                      [pd -p]
A=[0 1 0; 14 0 -14; 0 0 -1/0.05];
B=[0;0;1/0.05];
[K,pre,me]=place(A,B,[-300,-100,-80]); % pole placement