g=9.8;
z=0.736;
tp=0.03;
A=[0 1 0; g/z 0 -g/z; 0 0 -1/tp];
B=[0 0 1/tp]';
p=[-30 -15 -sqrt(g/z)];
[K,prec,message] = place(A,B,p)
