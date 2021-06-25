close all;

figure(1);
subplot(211);
plot(t,q1(:,1),'r.-','LineWidth',1);hold on
plot(t,q1(:,2),'b.-','LineWidth',1)
legend('期望关节变量','实际关节变量')
xlabel('t\s');ylabel('关节1关节空间跟踪情况');
subplot(212);
plot(t,q2(:,1),'r.-','LineWidth',1);hold on
plot(t,q2(:,2),'b.-','LineWidth',1);
legend('期望关节变量','实际关节变量')
xlabel('t\s');ylabel('关节2关节空间跟踪情况');

figure(2);
subplot(211);
plot(t,tol(:,1),'r.-','LineWidth',1);
xlabel('t\s');ylabel('关节1的控制力矩输入');
subplot(212);
plot(t,tol(:,2),'r.-','LineWidth',1);
xlabel('t\s');ylabel('关节2的控制力矩输入');

figure(3);

l1=1;l2=1.2;
k=size(q);
x_joint1=l1*cos(q(:,1));
y_joint1=l1*sin(q(:,1));
x_joint2=l1*cos(q(:,1))+l2*cos(q(:,1)+q(:,2));
y_joint2=l1*sin(q(:,1))+l2*sin(q(:,1)+q(:,2));
X=[zeros(k(1),1),x_joint1,x_joint2];
Y=[zeros(k(1),1),y_joint1,y_joint2];
plot(X(:,3),Y(:,3),'r','LineWidth',1.5),hold on
plot(y0(:,1),y0(:,2),'r*');hold on
for i=1:3:k(1)
    X(i,:)
    Y(i,:)
    plot(X(i,:),Y(i,:),'LineWidth',1.5)
    hold on
    drawnow
end
xlabel('x');ylabel('y');title('trajectory')