function clock_callback( ~, msg )
%CLOCK_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global clock;
global time_management;


clock.Sec=double(msg.Clock_.Sec);
clock.Nsec=double(msg.Clock_.Nsec);
clock.second=double(msg.Clock_.Sec)+double(msg.Clock_.Nsec)/10^9;

% time_management.i=...
%     floor((clock.Sec-time_management.t0sec)*1000....
%     +(clock.Nsec-time_management.t0nsec)/10^6);

time_management.i=...
    floor((clock.Sec-time_management.t0sec)*10^9....
    +(clock.Nsec-time_management.t0nsec))*(1000)/10^9; %1000?1000?step??

end

