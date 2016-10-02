
kalman_state=struct('q',0.1,    ...process noise covariance
                    'r',100,      ...measurement noise covariance
                    'x',0,      ... value
                    'p',10,      ...estimation error covariance
                    'k',50     ...kalman gain
                    );

x_after=[];

figure()
subplot(2,2,1)
Val=X_zmp;
for i=1:length(Val)
    [kalman_state,x]=Kal_filter(kalman_state,Val(i));
    x_after=[x_after,x];
end
subplot(2,2,1)
plot(1:length(Val),Val,'.y',1:length(Val),x_after,'-.k')
legend('before', 'after')
xlabel('timestamp')
ylabel('distance in m')
title('X zmp')

subplot(2,2,2)
x_after=[];
kalman_state=struct('q',0.1,    ...process noise covariance
                    'r',100,      ...measurement noise covariance
                    'x',0,      ... value
                    'p',10,      ...estimation error covariance
                    'k',50     ...kalman gain
                    );
Val=Y_zmp;
for i=1:length(Val)
    [kalman_state,x]=Kal_filter(kalman_state,Val(i));
    x_after=[x_after,x];
end
plot(1:length(Val),Val,'.y',1:length(Val),x_after,'-.k')
legend('before', 'after')
xlabel('timestamp')
ylabel('distance in m')
title('Y zmp')


subplot(2,2,3)
x_after=[];
kalman_state=struct('q',0.1,    ...process noise covariance
                    'r',100,      ...measurement noise covariance
                    'x',0,      ... value
                    'p',10,      ...estimation error covariance
                    'k',50     ...kalman gain
                    );
Val=X_com;
for i=1:length(Val)
    [kalman_state,x]=Kal_filter(kalman_state,Val(i));
    x_after=[x_after,x];
end
plot(1:length(Val),Val,1:length(Val),x_after)
legend('before', 'after')
xlabel('timestamp')
ylabel('distance in m')
title('X com')


subplot(2,2,4)
x_after=[];
kalman_state=struct('q',0.1,    ...process noise covariance
                    'r',100,      ...measurement noise covariance
                    'x',0,      ... value
                    'p',10,      ...estimation error covariance
                    'k',50     ...kalman gain
                    );
Val=Y_com;
for i=1:length(Val)
    [kalman_state,x]=Kal_filter(kalman_state,Val(i));
    x_after=[x_after,x];
end
plot(1:length(Val),Val,1:length(Val),x_after)
legend('before', 'after')
xlabel('timestamp')
ylabel('distance in m')
title('Y com')

% 
% x = x1
% p = p + q;
% 
% k = p / (p + r);
% x1 = x + k * (measurement – x);
% p = (1 – k) * p;