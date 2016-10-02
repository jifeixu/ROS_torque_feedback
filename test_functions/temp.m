%  position: 
      x1= 0.0304505645635
      y1= 0.154251932369
      z1= 0.581020318209
% position: 
      x2= 0.189703902404
      y2= 0.110911608217
      z2= 0.330500568051
%%
length=sqrt((x1-x2)^2+(y1-y2)^2+(z1-z2)^2)
%%
time_scale=1;
step_scale=1;
zh=0.7

% Simulation parameters
tf = time_scale*7;              % Total time
deltat = 0.001;
tcnt = 0:deltat:tf;

% zh = 0.75; % center of mass height
offset=zh;
% Steps time duration
tstepss1 = time_scale*0.8; % First Single support step time duration (1st segment)
tstepds2 = time_scale*0.2; % First Double support step time duration (2nd segment)
tstepss3 = time_scale*0.8; % Second Single support step time duration (3rd segment)
tstepds4 = time_scale*0.2; % Second Double support step time duration (4th segment)
tstepss5 = time_scale*0.8; % Third Single support step time duration (5th segment)
tstepds6 = time_scale*0.2; % Third Double support step time duration (6th segment)
tstepss7 = time_scale*0.8; % Fourth Single support step time duration (7th segment)
tstepds8 = time_scale*0.2;

steplength = 2*0.0725; % Step length for lat

steplength1 = step_scale*0.2; % First step length for long
steplength2 = step_scale*0.2; % Second step length for long
steplength3 = step_scale*0.2; % Third step length for long


%position at start of each segment
left_long_0=step_scale*0;  %0
left_long_1=step_scale*0;  %0.8
left_long_2=step_scale*0;  %1
left_long_3=step_scale*0.4;%1.8
left_long_4=step_scale*0.4;%2
left_long_5=step_scale*0.4;
left_long_6=step_scale*0.4;
left_long_7=step_scale*0.4; % stops moving.
left_long_8=step_scale*0.4;


right_long_0=step_scale*0;
right_long_1=step_scale*0.2;
right_long_2=step_scale*0.2;
right_long_3=step_scale*0.2;
right_long_4=step_scale*0.2;
right_long_5=step_scale*0.6;
right_long_6=step_scale*0.6;
right_long_7=step_scale*0.6;
right_long_8=step_scale*0.6;

t0 = 0.0;          % Initial time (first segment)
t1 = t0 + tstepss1;
t2 = t1 + tstepds2;
t3 = t2 + tstepss3;
t4 = t3 + tstepds4;
t5 = t4 + tstepss5; 
t6 = t5 + tstepds6;
t7 = t6 + tstepss7; 
t8 = t7 + tstepds8;

% Terms and coefficients used in the desired ZMP trajectory
T10 = t1 - t0;   % In fact it's tstepss1 ... so it's redundant
T21 = t2 - t1; 
T32 = t3 - t2;
T43 = t4 - t3;
T54 = t5 - t4;
T65 = t6 - t5;
T76 = t7 - t6;
T87 = t8 - t7;

height=0.1;

% lt23=[t2 t2+T32*0.1 t2+T32*0.2 T32/2 t3-T32*0.1 t3]; %2-3
% rt01=[t0 t0+T10*0.1 t0+T10*0.2 T10/2 t1-T10*0.1 t1]; %0-1
% rt45=[t4 t4+T54*0.1 t4+T54*0.2 T54/2 t5-T54*0.1 t4]; %4-5
% y=[0, 0.1*height,0.2*height, height, 0.1*height, 0];
% 
% lp23=polyfit(lt23,y,4);
% rp01=polyfit(rt01,y,4);
% rp45=polyfit(rt45,y,4);
% 
% lh=polyval(lp23,tcnt).*(stepfun(tcnt,t2)-stepfun(tcnt,t3));

% outp=polyval(lp23,tcnt);
% lt23=[t2  t2+T32/2 t3];
% rt01=[t0  t0+T10/2 t1]; %0-1
% rt45=[t4  t4+T54/2 t4]; %0-1
% y=[0,height, 0];
% 
% lp23=polyfit(lt23,y,4);
% rp01=polyfit(rt01,y,4);
% rp45=polyfit(rt45,y,4);
% 
% lh=polyval(lp23,tcnt).*(stepfun(tcnt,t2)-stepfun(tcnt,t3));

% plot(lh)

klht23_1= quinticfit( t2,0,0,t2+T32/2,height,0);
klht23_2= quinticfit( t2+T32/2,height,0,t3,0,0);

lht23_1=(klht23_1*[tcnt.^3; tcnt.^2; tcnt; tcnt.^0]).*(stepfun(tcnt,t2)-stepfun(tcnt,t2+T32/2));
lht23_2=(klht23_2*[tcnt.^3; tcnt.^2; tcnt; tcnt.^0]).*(stepfun(tcnt,t2+T32/2)-stepfun(tcnt,t3));

krht45_1= quinticfit( t4,0,0,t4+T54/2,height,0);
krht45_2= quinticfit( t4+T54/2,height,0,t5,0,0);

rht45_1=(krht45_1*[tcnt.^3; tcnt.^2; tcnt; tcnt.^0]).*(stepfun(tcnt,t4)-stepfun(tcnt,t4+T54/2));
rht45_2=(krht45_2*[tcnt.^3; tcnt.^2; tcnt; tcnt.^0]).*(stepfun(tcnt,t4+T54/2)-stepfun(tcnt,t5));

krht01= quinticfit( t0,height,0,t1,0,0);
rht01=(krht01*[tcnt.^3; tcnt.^2; tcnt; tcnt.^0]).*(stepfun(tcnt,t0)-stepfun(tcnt,t1));


lh=lht23_1+lht23_2;
rh=rht45_1+rht45_2+rht01;


