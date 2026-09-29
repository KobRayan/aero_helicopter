% generate_PRBS
%
% Below is an example for the setting of the 2 PRBS
% Modify the basic setting which does not guarantee to get good data
Ts=0.002;

n1=6;
p1=2000;

u_p=prbs(n1,p1,[-1 1]);
Tp=length(u_p)*Ts

n2=7;
p2=1600;
T1=n1*p1*Ts
T2=n2*p2*Ts

u_y=prbs(n2,p2,[-1 1]);
Ty=length(u_p)*Ts

Np=length(u_p);
u_y=u_y(1:Np);
t=(0:Np+500-1)'*Ts;

u_p=[t [zeros(500,1);u_p]];
u_y=[t [zeros(500,1);u_y]];

save PRBS_inputs u_p u_y n1 p1 n2 p2 Ts;
