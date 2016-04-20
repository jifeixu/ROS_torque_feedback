g=9.8;
z=0.70;
tp=0.03;
A=[0 1 0; g/z 0 -g/z; 0 0 -1/tp];
B=[0 0 1/tp]';
C=eye(3);
D=zeros(3,1);
%% normal pole placement
p=[-20 -14 -sqrt(g/z)];
[K,prec,message] = place(A,B,p);
K

%% try lqr
states={'x' 'x_dot' 'x_zmp'};
inputs={'zmpU'};
outputs={'x_o' 'x_dot_o' 'zmp_o'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
%%
Q=10*[0 0 0; 0 1 0; 0 0 1];
R=1;
K = lqr(A,B,Q,R)

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

sys_cl = ss(Ac,Bc,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r =0.2*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('Step Response with LQR Control')
%% precompensation?s http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=ControlStateSpace
% Cn = [1 0 0];
% sys_ss = ss(A,B,Cn,0);
% Nbar = rscale(sys_ss,K)
% 
% sys_cl = ss(Ac,Bc*Nbar,Cc,Dc,'statename',states,'inputname',inputs,'outputname',outputs);
% 
% t = 0:0.01:5;
% r =0.2*ones(size(t));
% [y,t,x]=lsim(sys_cl,r,t);
% [AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
% set(get(AX(1),'Ylabel'),'String','xn (m)')
% set(get(AX(2),'Ylabel'),'String','zmp')
%% try a different appoach assume no zmp is observed for observer
g=9.8;
z=0.70;
tp=0.03;
A=[0 1 0; g/z 0 -g/z; 0 0 -1/tp];
B=[0 0 1/tp]';
C=[1 0 0;0 1 0];
D=zeros(2,1);
states={'x' 'x_dot' 'x_zmp'};
inputs={'zmpU'};
outputs={'x_o' 'zmp_o'};
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

Ac = [(A-B*K)];
Bc = [B];
Cc = [C];
Dc = [D];

Q=10*C'*C;
R=1;
K = lqr(A,B,Q,R)

poles = eig(Ac);

P=[-50 -51 -52];

L=place(A',C',P)';

Ace = [(A-B*K) (B*K);
       zeros(size(A)) (A-L*C)];
Bce = [B;
       zeros(size(B))];
Cce = [Cc zeros(size(Cc))];
Dce = [0;0];

states2={'x' 'x_dot' 'x_zmp' 'e1' 'e2' 'e3'};

sys_est_cl = ss(Ace,Bce,Cce,Dce,'statename',states2,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r = 0.2*ones(size(t));
[y,t,x]=lsim(sys_est_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
%% only com x is direct measurement?

g=9.8;
z=0.70;
tp=0.03;
A=[0 1 0; g/z 0 -g/z; 0 0 -1/tp];
B=[0 0 1/tp]';
C=[1 0 0];
D=0;
states={'x' 'x_dot' 'x_zmp'};
inputs={'zmpU'};
outputs={'x_o' };
sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

Q=[5 0 0; 0 5 0; 0 0 1];
R=1;
K = lqr(A,B,Q,R)


Ac = (A-B*K);
Bc = B;
Cc = C;
Dc = D;

%$
poles = eig(Ac);

P=[-50 -51 -52];

L=place(A',C',P)'

Ace = [(A-B*K) (B*K);
       zeros(size(A)) (A-L*C)];
Bce = [B;
       zeros(size(B))];
Cce = [Cc zeros(size(Cc))];
Dce = [0];

states2={'x' 'x_dot' 'x_zmp' 'e1' 'e2' 'e3'};

sys_est_cl = ss(Ace,Bce,Cce,Dce,'statename',states2,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r = 0.2*ones(size(t));
[y,t,x]=lsim(sys_est_cl,r,t);
% [AX,H1,H2] = plotyy(t,y(:,1));
% plot(t,y(:,1));
plot(t,x)

%% discretization


g=9.8;
z=0.70;
tp=0.03;
A=[0 1 0; g/z 0 -g/z; 0 0 -1/tp];
B=[0 0 1/tp]';
C=[1 0 0];
D=0;
states={'x' 'x_dot' 'x_zmp'};
inputs={'zmpU'};
outputs={'x_o' };
% sys_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

Q=[5 0 0; 0 5 0; 0 0 1];
R=1;
K = lqr(A,B,Q,R)


Ac = (A-B*K);
Bc = B;
Cc = C;
Dc = D;

%$
poles = eig(Ac);

P=[-100 -101 -102];

L=place(A',C',P)'

states={'x' 'x_dot' 'x_zmp'};
inputs={'zmpU'};
outputs={'x_o' };
sys_ss=ss(A-L*C,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

inputs2={'xu' 'x_dot_u' 'x_zmp_u'};
sys_ss2=ss(A-L*C,L*C,C,D,'statename',states,'inputname',inputs2,'outputname',outputs);
Ts=1/1000;

L_d=mat2str(L)

sys_d1=c2d(sys_ss,Ts,'zoh');
A_d1=mat2str(sys_d1.A)
B_d1=mat2str(sys_d1.B)
C_d1=mat2str(sys_d1.C)
D_d1=mat2str(sys_d1.D)


sys_d2=c2d(sys_ss2,Ts,'zoh');

A_d2=mat2str(sys_d2.A)
B_d2=mat2str(sys_d2.B)
C_d2=mat2str(sys_d2.C)
D_d2=mat2str(sys_d2.D)
