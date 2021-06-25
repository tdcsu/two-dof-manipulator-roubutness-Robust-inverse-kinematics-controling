close all;

l1=0.25;l2=0.25;
k=size(q);
for i=1:100:k(1)
x_joint1=l1*cos(q(i,1));
y_joint1=l1*sin(q(i,1));
x_joint2=l1*cos(q(i,1))+l2*cos(q(i,1)+q(i,2));
y_joint2=l1*sin(q(i,1))+l2*sin(q(i,1)+q(i,2));
X=[0,x_joint1,x_joint2];
Y=[0,y_joint1,y_joint2];
plot(X,Y,'b')
hold on
end
plot(y0(:,1),y0(:,2),'r');
grid on
xlabel('x');ylabel('y');