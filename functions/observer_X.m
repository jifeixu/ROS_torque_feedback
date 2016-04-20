function [ cur_state_out ] = observer( xcom,u )
%OBSERVER Summary of this function goes here
%   run Pole placement Observer part for detailed L. This is assuming
%   ONLY X_COM is the output

%UNDER CONSTRUCTION>>>>>>>>>>>>>>>>>>>>>>>

% use of persistent variable need to use "clear observer" to reinitiate 
% persistent variables.
persistent cur_state;

if isempty(cur_state)
    cur_state=[0 0 0]';
end

x_input=[xcom 0 0]';

A=[0.754558977988587 0.000873350113383981 -6.32753175751362e-06;-19.0158155370816 0.990072391890244 -0.0137223082593467;18.990340630209 0.0100001784957787 0.967168632719747];
B1=[-7.21176021546112e-08;-0.000230365438630857;0.032783497218563]; %B1*u
B2=[0.245447421660773 0 0;19.0297682107796 0 0;-18.9902927601473 0 0]; %B2*x
% C=[1 0 0];

x_next=A*cur_state+B1*u+B2*x_input;
% y_next=C*cur_state;

cur_state=x_next;

cur_state_out=cur_state;

end

