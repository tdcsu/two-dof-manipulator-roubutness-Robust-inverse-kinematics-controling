function [sys,x0,str,ts]=robotplant(t,x,u,flag,x0,y0)
switch flag,
case 0
    [sys,x0,str,ts]=mdlInitializeSizes(x0,y0);
case 1
    sys=mdlDerivatives(t,x,u);
case 3
    sys=mdlOutputs(t,x,u);
case {2, 4, 9 }
    sys = [];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end
function [sys,x0,str,ts]=mdlInitializeSizes(x0,y0)
sizes = simsizes;
sizes.NumContStates  = 4;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 0;
sys=simsizes(sizes);

l1=1;l2=1.2;
if(x0>=0)
   alfa0=atan(y0/x0);
else
   alfa0=pi+atan(y0/x0);
end

beta0=acos((x0^2+y0^2+l1^2-l2^2)/(2*l1*sqrt(x0^2+y0^2)));
qd2_0=-acos((x0^2+y0^2-(l1^2+l2^2))/(2*l1*l2));
if(qd2_0>0)
   qd1_0=alfa0-beta0;
else  
   qd1_0=alfa0+beta0;                                  
end    

x0=[0.05;0.01;-0.1;00.02];   %状态变量的初始值
str=[];
ts=[];

function sys=mdlDerivatives(t,x,u)
% 带入初始参数
m1=1;m2=2;
l1=1;l2=1.5;
r1=0.5;r2=0.6;
J1=0.12;J2=0.25;

q1=x(1);
dq1=x(2);
q2=x(3);
dq2=x(4);

% J=[J1+J2+2*m2*r2*l1*cos(q2) J2+m2*r2*l1*cos(q2);
%    J2+m2*r2*l1*cos(q2) J2];

J=[J1+m1*r1^2+J2+m2*r2^2+m2*l1^2+2*m2*l1*r2*cos(pi/6)*cos(q2)+2*m2*l1*r2*sin(pi/6)*sin(q2) J2+m2*r2^2+m2*l1*r2*cos(pi/6)*cos(q2)+m2*l1*r2*sin(pi/6)*sin(q2);
   J2+m2*r2^2+m2*l1*r2*cos(pi/6)*cos(q2)+m2*l1*r2*sin(pi/6)*sin(q2) J2+m2*r2^2];

% f=[-2*m2*r2*l1*dq2*sin(q2) -m2*r2*l1*dq2*sin(q2);
%     m2*r2*l1*dq1*sin(q2) 0];
f=[-(m2*l1*r2*cos(pi/6)*sin(q2)-m2*l1*r2*sin(pi/6)*cos(q2))*dq2 -(m2*l1*r2*cos(pi/6)*sin(q2)-m2*l1*r2*sin(pi/6)*cos(q2))*(dq1+dq2);
  (m2*l1*r2*cos(pi/6)*sin(q2)-m2*l1*r2*sin(pi/6)*cos(q2))*dq1 0];

tol(1)=u(1);
tol(2)=u(2);
Kxi=[u(3) u(4)];

Delta=[0.1*cos(0.2-0.01*t);0.1*sin(0.2-0.01*t)]; 

S=inv(J)*(tol'-f*[dq1;dq2]-Delta); 

sys(1)=x(2);
sys(2)=S(1);
sys(3)=x(4);
sys(4)=S(2);
function sys=mdlOutputs(t,x,u)
sys(1)=x(1);
sys(2)=x(2);
sys(3)=x(3);
sys(4)=x(4);