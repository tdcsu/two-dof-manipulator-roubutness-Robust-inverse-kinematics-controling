% 控制器函数
%根据鲁棒逆运动学中控制器的设计来计算控制项u
function [sys,x0,str,ts] = control_strategy(t,x,u,flag)

switch flag
case 0
    [sys,x0,str,ts]=mdlInitializeSizes;
case 3
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
% 输入的控制指令
qd=[u(1) u(4)];
dqd=[u(2) u(5)];
ddqd=[u(3) u(6)];

%实际的关节空间变量输出
q=[u(7) u(9)];
dq=[u(8) u(10)];

%控制指令与输出之间的偏差
e=q-qd;
de=dq-dqd;

% 带入初始参数
m1=1;m2=2;
l1=1;l2=1.5;
r1=0.5;r2=0.6;
J1=0.12;J2=0.25;

q1=q(1);
dq1=dq(1);
q2=q(2);
dq2=dq(2);

% 惯性矩阵
%J=[J1+J2+2*m2*r2*l1*cos(q2) J2+m2*r2*l1*cos(q2);
%    J2+m2*r2*l1*cos(q2) J2];

J=[J1+m1*r1^2+J2+m2*r2^2+m2*l1^2+2*m2*l1*r2*cos(pi/6)*cos(q2)+2*m2*l1*r2*sin(pi/6)*sin(q2) J2+m2*r2^2+m2*l1*r2*cos(pi/6)*cos(q2)+m2*l1*r2*sin(pi/6)*sin(q2);
   J2+m2*r2^2+m2*l1*r2*cos(pi/6)*cos(q2)+m2*l1*r2*sin(pi/6)*sin(q2) J2+m2*r2^2];
%科里奥利矩阵
%f=[-2*m2*r2*l1*dq2*sin(q2) -m2*r2*l1*dq2*sin(q2);
  %  m2*r2*l1*dq1*sin(q2) 0];

f=[-(m2*l1*r2*cos(pi/6)*sin(q2)-m2*l1*r2*sin(pi/6)*cos(q2))*dq2 -(m2*l1*r2*cos(pi/6)*sin(q2)-m2*l1*r2*sin(pi/6)*cos(q2))*(dq1+dq2);
  (m2*l1*r2*cos(pi/6)*sin(q2)-m2*l1*r2*sin(pi/6)*cos(q2))*dq1 0];

%可调参数
alfa=50;
eq=0.02;
k0=40;
k1=70;

% 控制器需要使用到的参数
K=[k0,0;0,k1];
Kxi=de+alfa*e;

rou=3.0*tanh(2.6*norm(Kxi));
Un=[sin(6*Kxi(1)) sin(6*Kxi(2))];
v=Kxi*rou^2/(norm(Kxi)*rou+eq);
%v=[0,0];
w=J*alfa*de'+f*alfa*e';

%控制器的具体形式
ut=-K*Kxi'-w-v';
% B=[0;1];
% A=[0,1;-k0,-k1];
% if 
% deltaa=-
% ut=ddqd-k0*e-k1*de+deltaa;
tol=ut+J*ddqd'+f*dqd';

sys(1)=tol(1);
sys(2)=tol(2);
sys(3)=Kxi(1);
sys(4)=Kxi(2);