function [sys,x0,str,ts] = control_strategy(t,x,u,flag)

switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 3,
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 10;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];

function sys=mdlOutputs(t,x,u)
qd=[u(1) u(4)];
dqd=[u(2) u(5)];
ddqd=[u(3) u(6)];

q=[u(7) u(9)];
dq=[u(8) u(10)];
e=q-qd;
de=dq-dqd;

m1=0.765;m2=0.765;
l1=0.25;l2=0.25;
r1=0.15;r2=0.15;
J1=0.05;J2=0.05;

q1=q(1);
dq1=dq(1);
q2=q(2);
dq2=dq(2);

J=[J1+J2+2*m2*r2*l1*cos(q2) J2+m2*r2*l1*cos(q2);
    J2+m2*r2*l1*cos(q2) J2];
f=[-2*m2*r2*l1*dq2*sin(q2) -m2*r2*l1*dq2*sin(q2);
    m2*r2*l1*dq1*sin(q2) 0];

alfa=60;
eq=0.01;
k1=50;
k2=50;
K=[k1,0;0,k2];

Kxi=de+alfa*e;

rou=3.0*tanh(2.6*norm(Kxi));
Un=[sin(6*Kxi(1)) sin(6*Kxi(2))];
v=Kxi*rou^2/(norm(Kxi)*rou+eq);
%v=[0,0];
w=J*alfa*de'+f*alfa*e';

ut=-K*Kxi'-w-v';

tol=ut+J*ddqd'+f*dqd';

sys(1)=tol(1);
sys(2)=tol(2);
sys(3)=Kxi(1);
sys(4)=Kxi(2);