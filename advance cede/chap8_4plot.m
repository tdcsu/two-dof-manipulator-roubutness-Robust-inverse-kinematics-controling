close all;

figure(1);
subplot(211);
plot(t,q1(:,1),'r',t,q1(:,2),'b');
xlabel('time(s)');ylabel('Position tracking of link 1');
subplot(212);
plot(t,q2(:,1),'r',t,q2(:,2),'b');
xlabel('time(s)');ylabel('Position tracking of link 2');

figure(2);
subplot(211);
plot(t,tol(:,1),'r');
xlabel('time(s)');ylabel('Control input of link 1');
subplot(212);
plot(t,tol(:,2),'r');
xlabel('time(s)');ylabel('Control input of link 2');

figure(3);
plot(y0(:,1),y0(:,2),'r');
l1=0.25;l2=0.25;
q_1=q1(:,2);
q_2=q2(:,2);
x=l1*cos(q_1)+l2*cos(q_1+q_2);
y=l1*sin(q_1)+l2*sin(q_1+q_2);
hold on;
plot(x(:),y(:),'b');
xlabel('x');ylabel('y');
grid on