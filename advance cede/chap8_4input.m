function [sys,x0,str,ts] = inputsignal(t,x,u,flag)
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
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];
function sys=mdlOutputs(t,x,u)
l1=0.25;l2=0.25;

xd=-0.25*cos(pi/2*t);
yd=0.2*(1-cos(pi*t));

if(xd>=0)
  alfa0=atan(yd/xd);
else
   alfa0=pi+atan(yd/xd);
end
beta0=acos((xd^2+yd^2+l1^2-l2^2)/(2*l1*sqrt(xd^2+yd^2)));
qd2=-acos((xd^2+yd^2-(l1^2+l2^2))/(2*l1*l2));

if(qd2>0)
   qd1=alfa0-beta0;
else  
   qd1=alfa0+beta0;
end    
sys(1)=qd1;
sys(2)=qd2;
sys(3)=xd;
sys(4)=yd;