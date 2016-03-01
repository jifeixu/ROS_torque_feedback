function [ llat,llong,lh,rlat,rlong,rh ] = test_fn_foot_placement()
%TEST_FN_FOOT_PLACEMENT llat,llong,lh,rlat,rlong,rh
%   lat=Y, long=X, h=z, where z is negative.(absolute coordinates f
% Simulation parameters
tf = 7;              % Total time
deltat = 0.001;
tcnt = 0:deltat:tf;

zh = 0.7; % center of mass height
offset=zh;
% Steps time duration
tstepss1 = 0.8; % First Single support step time duration (1st segment)
tstepds2 = 0.2; % First Double support step time duration (2nd segment)
tstepss3 = 0.8; % Second Single support step time duration (3rd segment)
tstepds4 = 0.2; % Second Double support step time duration (4th segment)
tstepss5 = 0.8; % Third Single support step time duration (5th segment)
tstepds6 = 0.2; % Third Double support step time duration (6th segment)
tstepss7 = 0.8; % Fourth Single support step time duration (7th segment)
tstepds8 = 0.2;

steplength = 2*0.0725; % Step length for lat

steplength1 = 0.2; % First step length for long
steplength2 = 0.2; % Second step length for long
steplength3 = 0.2; % Third step length for long

%% for long:
%position at start of each segment
left_long_0=0;  %0
left_long_1=0;  %0.8
left_long_2=0;  %1
left_long_3=0.4;%1.8
left_long_4=0.4;%2
left_long_5=0.4;
left_long_6=0.4;
left_long_7=0.4; % stops moving.
left_long_8=0.4;


right_long_0=0;
right_long_1=0.2;
right_long_2=0.2;
right_long_3=0.2;
right_long_4=0.2;
right_long_5=0.6;
right_long_6=0.6;
right_long_7=0.6;
right_long_8=0.6;

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
%t0-t1, linear
llong_01=0;
llong_11=0;
%t1-t2, linear
llong_02=0;
llong_12=0;
%t2-t3, cubic
llong_03=left_long_2;
llong_13=0;            % const cubic const
Coeff1=[T32^2,T32^3;2*T32,3*T32^2];
Known1=[left_long_3 - left_long_2 - (left_long_2 - left_long_1)*T32/T21;...
         (left_long_4 - left_long_3)/T43 - (left_long_2 - left_long_1)/T21];
sol1=Coeff1\Known1;
llong_23=sol1(1);
llong_33=sol1(2);
%t3-t4, const
llong_04=left_long_3;
llong_14=0;
%t4-t5, const
llong_05=left_long_4;
llong_15=0;
%t5-6, const
llong_06=left_long_5;
llong_16=0;
%t6-t7, cubic
llong_07=left_long_6;
llong_17=0;


%llong means left long
llong = (llong_01 + llong_11*(tcnt - t0)).*(stepfun(tcnt,t0)-stepfun(tcnt,t1)) +... linear 0-1
    (llong_02 + llong_12*(tcnt - t1)).*(stepfun(tcnt,t1)-stepfun(tcnt,t2)) +... linear 1-2
    (llong_03 + llong_13*(tcnt - t2) + llong_23*(tcnt-t2).^2 + llong_33*(tcnt-t2).^3).*...
        (stepfun(tcnt,t2)-stepfun(tcnt,t3))+... cubic 2-3
    (llong_04 + llong_14*(tcnt - t3)).*(stepfun(tcnt,t3)-stepfun(tcnt,t4)) +... linear 3-4
    (llong_05 + llong_15*(tcnt - t4)).*(stepfun(tcnt,t4)-stepfun(tcnt,t5)) +... linear 4-5
    (llong_06 + llong_16*(tcnt - t5)).*(stepfun(tcnt,t5)-stepfun(tcnt,t6)) +... linear 5-6
    (llong_07 + llong_17*(tcnt - t6)).*(stepfun(tcnt,t6)-stepfun(tcnt,t7)) +... linear 6-7
        left_long_8*stepfun(tcnt,t7);
% plot(tcnt,llong)
%%

%t0-t1, cubic
rlong_01=right_long_0; % this equation come from a31*x^3+a21*x^2+a11*x+a01= rlong
rlong_11=0;            % const cubic const
Coeff1=[T10^2,T10^3;2*T10,3*T10^2];             % the equation is:[t^3 t^2]*[b]=[pos2-slop1*t2-pos1]
Known1=[right_long_1 - right_long_0 - 0;...     %                 [3t^2,2t] [d] [slope2-slope1]  
         0];                                    %curlongently both slope are set to 0;
sol1=Coeff1\Known1;
rlong_21=sol1(1);
rlong_31=sol1(2);
%t1-t2, linear
rlong_02=right_long_1;
rlong_12=0;
%t2-t3, linear
rlong_03=right_long_2;
rlong_13=0;
%t3-t4,linear
rlong_04=right_long_3;
rlong_14=0;
%t4-t5, cubic
rlong_05=right_long_4;
rlong_15=0;
Coeff=[T54^2,T54^3;2*T54,3*T54^2];             % the equation is:[t^3 t^2]*[b]=[pos2-slop1*t2-pos1]
Known=[right_long_5 - right_long_4;...     %                 [3t^2,2t] [d] [slope2-slope1]  
         0];                                    %curlongently both slope are set to 0;
sol=Coeff\Known;
rlong_25=sol(1);
rlong_35=sol(2);
%t5-t6, linear
rlong_06=right_long_5;
rlong_16=0;
%t6-t7, const
rlong_07=right_long_6;
rlong_17=0;

rlong = (rlong_01 + rlong_11*(tcnt - t0) + rlong_21*(tcnt-t0).^2 + rlong_31*(tcnt-t0).^3).*...cubic 0-1
        (stepfun(tcnt,t0)-stepfun(tcnt,t1))+... 
    (rlong_02 + rlong_12*(tcnt - t1)).*(stepfun(tcnt,t1)-stepfun(tcnt,t2)) +... linear 1-2
    (rlong_03 + rlong_13*(tcnt - t2)).*(stepfun(tcnt,t2)-stepfun(tcnt,t3)) +... linear 2-3
    (rlong_04 + rlong_14*(tcnt - t3)).*(stepfun(tcnt,t3)-stepfun(tcnt,t4)) +... linear 3-4
    (rlong_05 + rlong_15*(tcnt - t4) + rlong_25*(tcnt-t4).^2 + rlong_35*(tcnt-t4).^3).*...cubic 4-5
        (stepfun(tcnt,t4)-stepfun(tcnt,t5))+...
    (rlong_06 + rlong_16*(tcnt - t5)).*(stepfun(tcnt,t5)-stepfun(tcnt,t6)) +... linear 5-6
    (rlong_07 + rlong_17*(tcnt - t6)).*(stepfun(tcnt,t6)-stepfun(tcnt,t7)) +... linear 6-7
        right_long_8*stepfun(tcnt,t7);
    
%     plot(rlong)
%% plot
% plot(tcnt,llong,tcnt,rlong)
%% llat rlat.
llat=0.075*ones(size(llong)); % for the whole duration;
rlat=-0.075*ones(size(llong)); % for the whole duration;
%% lh rh
% use cosine wave, heigh/2 *(1-cos(2*pi*t/T))
height=0.1;
lh= height*0.5*(1-cos((2*pi/T32)*(tcnt-t2))).*(stepfun(tcnt,t2)-stepfun(tcnt,t3)); %sine 2-3
rh= height*0.5*(1-cos((pi/T10)*(tcnt-t0)+pi)).*(stepfun(tcnt,t0)-stepfun(tcnt,t1))+...sine 0-1
    height*0.5*(1-cos((2*pi/T54)*(tcnt-t4))).*(stepfun(tcnt,t4)-stepfun(tcnt,t5)); % sine 4-5
% plot(tcnt,lh,tcnt,rh)

lh=lh-offset; %height is offset from the COM with offset of distance of COM.
rh=rh-offset;
%%
% figure(5)
% plot3(llat,llong,lh,rlat,rlong,rh)
% grid




end

