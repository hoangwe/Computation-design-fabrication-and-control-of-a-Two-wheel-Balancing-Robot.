clc;close all;clear all;
%% programed by: Nguyen Minh Hoang

%  Declare parameters
rR=0.075;
rL=rR;
b=0.3/2;
W=0.15;
tf=1;
% initial conditions
phi0=0;
x0=0;
y0=0;
% time vector and time increment
t=linspace(0,tf,60);
dt=t(2)-t(1);


x=zeros(1,length(t));
y=zeros(1,length(t));
phi=zeros(1,length(t));

phi(1)=phi0;
x(1)=x0;
y(1)=y0;

% command signals (in RPM)
ind1=find(t<=max(t)/3);% 1
ind2=find((t>max(t)/3)&(t<=2*max(t)/3));% 2
ind3=find((t>2*max(t)/3)&(t<=max(t)));%3 
vec=ones(1,length(t));

thedotR=[-240*(2*pi/60)*(vec(ind1)) -240*(2*pi/60)*(vec(ind1)),-120*(2*pi/60)*(vec(ind1))];
thedotL=[-240*(2*pi/60)*(vec(ind1)) -120*(2*pi/60)*(vec(ind1)),120*(2*pi/60)*(vec(ind1))];

% update loop
for i=2:length(t)
    % Jacobian matrix
    J=[ rR*cos(phi(i-1))/2, rL*cos(phi(i-1))/2;...
        rR*sin(phi(i-1))/2, rL*sin(phi(i-1))/2;...
        rR/W              , -rL/W ];
    deltapose=dt*J*[thedotR(i-1);thedotL(i-1)];
    x(i)=x(i-1)+deltapose(1);
    y(i)=y(i-1)+deltapose(2);
    phi(i)=phi(i-1)+deltapose(3);
end

% plot and animation


figure(1);
plot(x,y,'k--');
title('workspace')
xlabel('x')
ylabel('y')
hold on;
wls=[0 0;-W W];
wlsrot=[cos(phi(i)),-sin(phi(i));sin(phi(i)),cos(phi(i))]*wls;
h1=plot(wlsrot(1,1)+x(1),wlsrot(2,1)+y(1),'ro','LineWidth',2,'MarkerFaceColor','r');
h2=plot(wlsrot(1,2)+x(1),wlsrot(2,2)+y(1),'ro','LineWidth',2,'MarkerFaceColor','r');
h3=plot(x(1),y(1),'bo','MarkerSize',20);
axis([-1 1 -1 1]);
text(x(1),y(1),['\fontsize{5}','S:(' , num2str(x(1),1.5), ', ', num2str(y(1),1.5),')']);
grid on;
text(x(i),y(i),['\fontsize{5}','E:(' , num2str(x(i),1.5), ', ', num2str(y(i),1.5),')']);


figure(3);
plot(t,thedotR,'b','linewidth',1.5);
hold on;
plot(t,thedotL,'r','linewidth',1.5);
xlabel('t(hours)');
ylabel('m/hours');
grid on;
legend('v_r','v_l');
title('angular velocities of the wheels');

figure(2);
plot(t,phi,'b','linewidth',1.5);
xlabel('t(hours)');
ylabel('rad');
legend('phi');
grid on;
title('angle of the robot chassis');
for i=2:length(t)
    wlsrot=[cos(phi(i)),-sin(phi(i));sin(phi(i)),cos(phi(i))]*wls;
    set(h1,'XData',wlsrot(1,1)+x(i));
    set(h1,'YData',wlsrot(2,1)+y(i));
    set(h2,'XData',wlsrot(1,2)+x(i));
    set(h2,'YData',wlsrot(2,2)+y(i));
    set(h3,'XData',x(i));
    set(h3,'YData',y(i));
    hold on;
    grid on;
    drawnow;
    pause(0.1);
end