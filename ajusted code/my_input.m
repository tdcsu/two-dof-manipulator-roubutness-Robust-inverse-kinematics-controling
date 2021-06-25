%input 函数，相当于创建控制指令输入，包含关节空间与操作空间的控制指令输入，实际上就是期望的轨迹
function [sys,x0,str,ts] = inputsignal(t,x,u,flag)
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
sizes.NumInputs      = 0;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];
function sys=mdlOutputs(t,x,u)
l1=1;l2=1.2; %初始化机械臂长度参数

xd=-1.2;%初始化期望操作空间轨迹x
yd=1;   %初始化期望操作空间轨迹y
qd1=pi/2;
qd2=pi/2;
% 因为逆运动学的多解性，需要判断上肘位和下肘位，从而求解出期望的关节空间轨迹
% if(xd>=0)
%   alfa0=atan(yd/xd);
% else
%    alfa0=pi+atan(yd/xd);
% end
% beta0=acos((xd^2+yd^2+l1^2-l2^2)/(2*l1*sqrt(xd^2+yd^2)));
% qd2=-acos((xd^2+yd^2-(l1^2+l2^2))/(2*l1*l2));
% 
% if(qd2>0)
%    qd1=alfa0-beta0;
% else  
%    qd1=alfa0+beta0;
% end 
% 但是此次的跟踪控制相当于期望轨迹是不随时间变化的常量，所以并不需要由上述计算而来，只需要填入对应值即可
% 输出为期望的操作空间轨迹以及期望的关节空间轨迹
% 即系统的控制指令输入
sys(1)=qd1;
sys(2)=qd2;
sys(3)=xd;
sys(4)=yd;

