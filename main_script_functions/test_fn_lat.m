function [ ZmpLat,Xc,tcnt ] = test_fn_lat(time_scale,step_scale,zh)
%TEST_FN_LAT Summary of this function goes here
%   Generate lateral CoM traj and ZMP traj

xc=0.075;
xcd=-0.01;

% Simulation parameters
tf = time_scale*7;
deltat = 0.001;

syms T t;

% LIPM parameters
g = 9.8;
% zh = 0.75;   % Center of Mass constant height

om = sqrt(g/zh);
w=sqrt(g/zh);
% Steps time duration


tstepss1  = time_scale*0.8; % First Single support step time duration (1st segment)
tstepds2  = time_scale*0.2; % First Double support step time duration (2nd segment)
tstepss3  = time_scale*0.8; % Second Single support step time duration (3rd segment)
tstepds4  = time_scale*0.2; % Second Double support step time duration (4th segment)
tstepss5  = time_scale*0.8; % Third Single support step time duration (5th segment)
tstepds6  = time_scale*0.2; % Third Double support step time duration (6th segment)
tstepss7  = time_scale*0.8; % Fourth Single support step time duration (7th segment)
tstepds8  = time_scale*0.2; % Fourth Double support step time duration (4th segment)
tstepss9  = time_scale*0.8; % Fifth Single support step time duration (5th segment)
tstepds10 = time_scale*0.2; % Fifth Double support step time duration (6th segment)
tstepss11 = time_scale*0.8; % Sixth Single support step time duration (7th segment)

steplength = step_scale*2*0.0725; % Step length


steplength1 = step_scale*2*0.0725; % First step length
steplength2 = step_scale*2*0.0725; % Second step length
steplength3 = step_scale*2*0.0725; % Third step length

ZMPLin = 0*0.05;    % Single support, ZMP variation in the linear case
                  % Of the total foot length 21 cm we set that 5 are
                  % available for ZMP variations
                  
               % To set Constant segment with no linear contribution
               % just set ZMPLin = 0
% Various ZMP positions at the t_i for i= 0, ...5               
% ZMP0 = 0;                    % x_zmp in t=0
% ZMP1 = ZMP0 + ZMPLin;
% ZMP2 = ZMP1 + steplength1;
% ZMP3 = ZMP2 + ZMPLin;
% ZMP4 = ZMP3 + steplength2;
% ZMP5 = ZMP4 + ZMPLin;
% ZMP6 = ZMP5 + steplength3;
% ZMP7 = ZMP6 + ZMPLin;

% ZMP0 = 0.05;                    % x_zmp in t=0
% ZMP1 = 0.05;

ZMP0 = steplength/2;                    % x_zmp in t=0
ZMP1 = ZMP0+T;
ZMP2 = -steplength/2;
ZMP3 = ZMP2;
ZMP4 = steplength/2;
ZMP5 = ZMP4;
ZMP6 = ZMP5 - steplength/2;
ZMP7 = ZMP6;
ZMP8 = 0 ;%+ steplength;
ZMP9 = ZMP8;
ZMP10 = 0;

ZMP11 = ZMP10 + 0*ZMPLin;   % If we prefer that the last segment 
                            % stays constant



t0 = 0;
t1 = t0 + tstepss1;
t2 = t1 + tstepds2;
t3 = t2 + tstepss3;
t4 = t3 + tstepds4;
t5 = t4 + tstepss5; 
t6 = t5 + tstepds6;
t7 = t6 + tstepss7; 
t8 = t7 + tstepds8;
t9 = t8 + tstepss9; 
t10 = t9 + tstepds10;
t11 = t10 + tstepss11; 


% Terms and coefficients used in the desired ZMP trajectory
T10 = t1 - t0;   % In fact it's tstepss1 ... so it's redundant
T21 = t2 - t1; 
T32 = t3 - t2;
T43 = t4 - t3;
T54 = t5 - t4;
T65 = t6 - t5;
T76 = t7 - t6;
T87 = t8 - t7;
T98 = t9 - t8;
T109 = t10 - t9;
T1110 = t11 - t10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% ZMP COEFFICIENTS

% Constant-Linear/Cubic/Cost-Lin parameters
% aij = a_i^{(j)} Harada's notation

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%% From t0i to t0 (2-nd segment, Cubic)
% a0i = ZMP0i;
% a1i = (ZMP0 - ZMP0i)/t0i;
% % Linear equations to be solved for a22 & a32
% Coeff1 = [t0i^2, t0i^3; 2*t0i, 3*t0i^2];
% Known1 = [ZMP0 - ZMP0i - (ZMP0i - 0*ZMP0)*T21/T10;...
%          (ZMP1 - ZMP0)/T10 - (ZMP0i - 0*ZMP0)/T10];
% sol1 = inv(Coeff1)*Known1;
% a2i = sol1(1);
% a3i = sol1(2);
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%% From t0 to t1 (1-st segment, Constant/Linear)
a01 = ZMP0;
a11 = (ZMP1 - ZMP0)/T10;         % If Linear First segment

%%% From t1 to t2 (2-nd segment, Cubic)
a02 = ZMP1;
a12 = (ZMP1 - ZMP0)/T10;
% Linear equations to be solved for a22 & a32
Coeff1 = [T21^2, T21^3; 2*T21, 3*T21^2];
Known1 = [ZMP2 - ZMP1 - (ZMP1 - ZMP0)*T21/T10;...
         (ZMP3 - ZMP2)/T32 - (ZMP1 - ZMP0)/T10];
sol1 = Coeff1\Known1;
a22 = sol1(1);
a32 = sol1(2);

%%% From t2 to t3 (3-rd segment, Constant/Linear)
a03 = ZMP2;
a13 = (ZMP3 - ZMP2)/T32;

%%% From t3 to t4 (4-th segment, Cubic)
a04 = ZMP3;
a14 = (ZMP3 - ZMP2)/T32;
% Linear equations to be solved for a24 & a34
Coeff2 = [T43^2, T43^3; 2*T43, 3*T43^2];
Known2 = [ZMP4 - ZMP3 - (ZMP3 - ZMP2)*T43/T32;...
          (ZMP5 - ZMP4)/T54 - (ZMP3 - ZMP2)/T32];     
sol2 = Coeff2\Known2;
a24 = sol2(1);
a34 = sol2(2);

%%% From t4 to t5 (5-th segment, Constant/Linear)
a05 = ZMP4;
a15 = (ZMP5 - ZMP4)/T54;


% From t5 to t6 (6-th segment, Cubic)
a06 = ZMP5;
a16 = (ZMP5 - ZMP4)/T54;
% Linear equations to be solved for a26 & a36
Coeff3 = [T65^2, T65^3; 2*T65, 3*T65^2];
Known3 = [ZMP6 - ZMP5 - (ZMP5 - ZMP4)*T65/T54;...
          (ZMP7 - ZMP6)/T76 - (ZMP5 - ZMP4)/T54];     
sol3 = Coeff3\Known3;
a26 = sol3(1);
a36 = sol3(2);

%%% From t6 to t7 (7-th segment, Constant/Linear)
a07 = ZMP6;
a17 = (ZMP7 - ZMP6)/T76;

% From t7 to t8 (8-th segment, Cubic)
a08 = ZMP7;
a18 = (ZMP7 - ZMP6)/T76;
% Linear equations to be solved for a28 & a38
Coeff3 = [T87^2, T87^3; 2*T87, 3*T87^2];
Known3 = [ZMP8 - ZMP7 - (ZMP7 - ZMP6)*T87/T76;...
          (ZMP9 - ZMP8)/T98 - (ZMP7 - ZMP6)/T76];     
sol3 = inv(Coeff3)*Known3;
a28 = sol3(1);
a38 = sol3(2);

%%% From t8 to t9 (9-th segment, Constant/Linear)
a09 = ZMP8;
a19 = (ZMP9 - ZMP8)/T98;

% From t9 to t10 (10-th segment, Cubic)
a010 = ZMP9;
a110 = (ZMP9 - ZMP8)/T98;
% Linear equations to be solved for a28 & a38
Coeff3 = [T109^2, T109^3; 2*T109, 3*T109^2];
Known3 = [ZMP10 - ZMP9 - (ZMP9 - ZMP8)*T109/T98;...
          (ZMP11 - ZMP10)/T1110 - (ZMP9 - ZMP8)/T98];     
sol3 = inv(Coeff3)*Known3;
a210 = sol3(1);
a310 = sol3(2);

%%% From t10 to t11 (11-th segment, Constant/Linear)
a011 = ZMP10;
a111 = (ZMP11 - ZMP10)/T1110;

zmpfn1=(a01 + a11*(t - t0));
zmpfn2=(a02 + a12*(t - t1) + a22*(t-t1).^2 + a32*(t-t1).^3);
zmpfn3=(a03 + a13*(t - t2));
zmpfn4=(a04 + a14*(t - t3) + a24*(t-t3).^2 + a34*(t-t3).^3);
zmpfn5=(a05 + a15*(t - t4));
zmpfn6=(a06 + a16*(t - t5) + a26*(t-t5).^2 + a36*(t-t5).^3);
zmpfn7=(a07 + a17*(t - t6));
zmpfn8=ZMP7;
zmpfn9=ZMP8;
zmpfn10=ZMP9;
zmpfn11=ZMP10;

%% integration & solve
xu_T_without_W_1=int(w*exp(-w*t)*zmpfn1,t,0,t1);
xu_T_without_W_2=int(w*exp(-w*t)*zmpfn2,t,t1,t2);
xu_rest_3=int(w*exp(-w*t)*zmpfn3,t,t2,t3);
xu_rest_4=int(w*exp(-w*t)*zmpfn4,t,t3,t4);
xu_rest_5=int(w*exp(-w*t)*zmpfn5,t,t4,t5);
xu_rest_6=int(w*exp(-w*t)*zmpfn6,t,t5,t6);
xu_rest_7=int(w*exp(-w*t)*zmpfn7,t,t6,t7);
xu_rest_8=int(w*exp(-w*t)*zmpfn8,t,t7,tf);
xu_rest_9=0;
xu_rest_10=0;
xu_rest_11=0;
xu_total=xu_T_without_W_1+xu_T_without_W_2+xu_rest_3+xu_rest_4+xu_rest_5...
    +xu_rest_6+xu_rest_7+xu_rest_8;
xu_wanted=xc+xcd/w;
xu_to_solve=xu_total-xu_wanted;
the_T=fsolve(matlabFunction(xu_to_solve),0.1)
%%
T=the_T;
% 
% a01=eval(a01);
% a11=eval(a11);
% a12=eval(a12);
% a22=eval(a22);
% a32=eval(a32);
% 
a02=eval(a02);
% a03=eval(a03);
% a04=eval(a04);
% a05=eval(a05);
% a06=eval(a06);
% a07=eval(a07);
a11=eval(a11);
a12=eval(a12);
% a13=eval(a13);
% a14=eval(a14);
% a15=eval(a15);
% a16=eval(a16);
% a17=eval(a17);
a22=eval(a22);
% a24=eval(a24);
% a26=eval(a26);
a32=eval(a32);
% a34=eval(a34);
% a36=eval(a36);
% ZMP7=eval(ZMP7);
% ZMP0=eval(ZMP0);
ZMP1=eval(ZMP1);

%%
%%%%% END ZMP COEFFICIENTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% (a0i + a1i*(t - t0i) + a2i*(t-t0i).^2 + a3i*(t-t0i).^3).*...
%         (stepfun(t,t0i)-stepfun(t,t0)) + ...
tcnt = 0:deltat:tf;
% Desired ZMP trajectory
ZmpLat =         (a01 + a11*(tcnt - t0)).*(stepfun(tcnt,t0)-stepfun(tcnt,t1)) +...
        (a02 + a12*(tcnt - t1) + a22*(tcnt-t1).^2 + a32*(tcnt-t1).^3).*...
        (stepfun(tcnt,t1)-stepfun(tcnt,t2)) + ...
        (a03 + a13*(tcnt - t2)).*(stepfun(tcnt,t2)-stepfun(tcnt,t3)) + ...
        (a04 + a14*(tcnt - t3) + a24*(tcnt-t3).^2 + a34*(tcnt-t3).^3).*...
        (stepfun(tcnt,t3)-stepfun(tcnt,t4)) + ...
        (a05 + a15*(tcnt - t4)).*(stepfun(tcnt,t4)-stepfun(tcnt,t5)) +...
        (a06 + a16*(tcnt - t5) + a26*(tcnt-t5).^2 + a36*(tcnt-t5).^3).*...
        (stepfun(tcnt,t5)-stepfun(tcnt,t6)) + ...
        (a07 + a17*(tcnt - t6)).*(stepfun(tcnt,t6)-stepfun(tcnt,t7)) +...
        (a08 + a18*(tcnt - t7) + a28*(tcnt-t7).^2 + a38*(tcnt-t7).^3).*...
        (stepfun(tcnt,t7)-stepfun(tcnt,t8)) + ...
        (a09 + a19*(tcnt - t8)).*(stepfun(tcnt,t8)-stepfun(tcnt,t9)) +...
        (a010 + a110*(tcnt - t9) + a210*(tcnt-t9).^2 + a310*(tcnt-t9).^3).*...
        (stepfun(tcnt,t9)-stepfun(tcnt,t10)) + ...
        (a011 + a111*(tcnt - t10)).*(stepfun(tcnt,t10)-stepfun(tcnt,t11)) +...    
        ZMP11*stepfun(tcnt,t11);  % This last constant contribution is new
        % the last term avoids the necessity, in the old version, to
        % add some time to the final instant e.g. t7

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% CoM TRAJECTORY

% Segment from t0 to t1 (Constant + Linear)
Xc1 = a11*Linear(t0,t1,om,tcnt) + a01*Const(t0,t1,om,tcnt);

% Second segment from t1 to t2 --- Spline
% various terms are computed separately and added at the end

% Cubic terms
Cubic2 = Cubic(t1,t2,om,tcnt);
% Quadratic terms
Quad2 = Quadra(t1,t2,om,tcnt );
% Linear terms
Lin2 = Linear(t1,t2,om,tcnt );
% Constant term
Cost2 = Const(t1,t2,om,tcnt);

% Final Spline from t1 to t2 --- Second segment
Xc2 = a32*Cubic2 + a22*Quad2 + a12*Lin2 + a02*Cost2;

% Third segment from t2 to t3 (Constant + Linear)
Xc3 = a13*Linear(t2,t3,om,tcnt)+a03*Const(t2,t3,om,tcnt);

% Fourth segment from t3 to t4 --- Spline

% Cubic terms
Cubic4 = Cubic(t3,t4,om,tcnt);
% Quadratic terms   
Quad4 = Quadra(t3,t4,om,tcnt);
% Linear terms
Lin4 = Linear(t3,t4,om,tcnt);
% Constant terms
Cost4 = Const(t3,t4,om,tcnt);
         
% Final spline from t3 to t4 --- Fourth segment 
Xc4 = a34*Cubic4 + a24*Quad4 + a14*Lin4 + a04*Cost4;

% Fifth segment from t4 to t5 (Constant + Linear)
Xc5 = a15*Linear(t4,t5,om,tcnt) + a05*Const(t4,t5,om,tcnt);

% Sixth segment from t5 to t6 --- Spline

% Cubic terms
Cubic6 = Cubic(t5,t6,om,tcnt);
% Quadratic terms   
Quad6 = Quadra(t5,t6,om,tcnt);
% Linear terms
Lin6 = Linear(t5,t6,om,tcnt);
% Constant terms
Cost6 = Const(t5,t6,om,tcnt);
         
% Final spline from t5 to t6 --- Sixth segment 
Xc6 = a36*Cubic6 + a26*Quad6 + a16*Lin6 + a06*Cost6;

% Seventh segment from t6 to t7 (Constant + Linear)
Xc7 = a17*Linear(t6,t7,om,tcnt) + a07*Const(t6,t7,om,tcnt);

% Eight segment from t7 to t8 --- Spline

% Cubic terms
Cubic8 = Cubic(t7,t8,om,tcnt);
% Quadratic terms   
Quad8 = Quadra(t7,t8,om,tcnt);
% Linear terms
Lin8 = Linear(t7,t8,om,tcnt);
% Constant terms
Cost8 = Const(t7,t8,om,tcnt);
         
% Final spline from t7 to t8 --- 8-th segment 
Xc8 = a38*Cubic8 + a28*Quad8 + a18*Lin8 + a08*Cost8;

% 9-th segment from t8 to t9 (Constant + Linear)
Xc9 = a19*Linear(t8,t9,om,tcnt) + a09*Const(t8,t9,om,tcnt);

% 10-th segment from t9 to t10 --- Spline

% Cubic terms
Cubic10 = Cubic(t9,t10,om,tcnt);
% Quadratic terms   
Quad10 = Quadra(t9,t10,om,tcnt);
% Linear terms
Lin10 = Linear(t9,t10,om,tcnt);
% Constant terms
Cost10 = Const(t9,t10,om,tcnt);
         
% Final spline from t9 to t10 --- 10-th segment 
Xc10 = a310*Cubic10 + a210*Quad10 + a110*Lin10 + a010*Cost10;

% 11-th segment from t10 to t11 (Constant + Linear)
Xc11 = a111*Linear(t10,t11,om,tcnt) + a011*Const(t10,t11,om,tcnt);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%  NEW TERM wrt to the July version
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Last constant segment from t11 on

Xc12 = ZMP11*(0.5*(exp(om*tcnt)*(exp(-om*t11))).*...
              (stepfun(tcnt,0) - stepfun(tcnt,t11))+...
          (1 - 0.5*(exp(-om*(tcnt-t11)))).*...
          (stepfun(tcnt,t11)));
      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% All the trajectories have been computed for
% an initial condition of the stable subsystem 
% x_stable(0) = 0

% Xcstar final with zero subsystem initial condition
Xc_zero = Xc1 + Xc2 + Xc3 + Xc4 + Xc5 + Xc6 + Xc7 + Xc8+Xc9+Xc10+Xc11;

% Corresponding CoM position
% From the change of coordinates
% being Xc = 0.5(x_unstable + x_stable)
% the Xc(0)corresponding to x_stable(0) 
% is equal to 0.5*x_unstable(0)

Xc00 = Xc_zero(1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Actual initial CoM position
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Xc0a = xc;

% Variation of actual wrt to Xc00
dXc0 = Xc0a - Xc00;

% Contribution to Xc

Xc_free = exp(-om*tcnt)*dXc0;

% Final CoM position desired trajectory starting from actual value

Xc = Xc_zero + Xc_free;


end

