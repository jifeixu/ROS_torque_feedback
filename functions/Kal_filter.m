function [ next_state, output] = Kal_filter( kalman_state,measurement )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
kalman_state.p=kalman_state.p+kalman_state.q;

% measurement update

kalman_state.k=kalman_state.p/(kalman_state.p+kalman_state.r);
kalman_state.x=kalman_state.x+kalman_state.k*(measurement-kalman_state.x);
kalman_state.p=(1-kalman_state.k)*kalman_state.p;

output=kalman_state.x;

next_state=kalman_state;

end

