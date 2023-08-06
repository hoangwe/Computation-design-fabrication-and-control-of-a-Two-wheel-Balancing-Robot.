 clc;
close all;
%% data

%% Plot parameters

imgWidthSize = 800;
imgColumnSize = 600;
lw = 1.5;
ms = 1;
numLgdCol = 1;
fs = 14; % legend font size
xls = 13; % x label font size
yls = 12; % y      ""
titleFontSize = 19;
XFontSize = 17;
YFontSize = 17;
lgdFontSize = 15;
ticksFontSIze = 13;
linecolors = linspecer(5, 'qualitative');
LineColors = flipud(linecolors);
%% the, psi, phi
% fig1 = figure('Position', [50, 10, imgWidthSize, imgColumnSize]);
% set(gca,'LooseInset', max(get(gca,'TightInset'), 0.02))
% set(groot, 'defaultAxesTickLabelInterpreter','latex');

figure(1);
time =(0:0.01:20);
time_t=time;
plot(time_t,rawdata(1:2001,1),'LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(time_t,rawdata(1:2001,2),'LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(time_t,rawdata(1:2001,3),'LineWidth', lw, 'MarkerSize', ms)
grid on;
xlabel('Time (Seconds)');
ylabel('Amplitude(degree)');
legend('Rawdata','Complementary Filter','Kalman Filter',"Interpreter", 'latex','Location','southeast', 'FontSize', fs);
title(' Pitch angle data  from IMU with Filter','FontSize', 15, 'interpreter','latex');


%% thetaa
figure(1);
a =ref.time;
b = [ref.signals.values,the_m1.signals.values,the_m2.signals.values,the_m3.signals.values,the_m4.signals.values,the_m5.signals.values];
draw_controller(a,b,6)
hold on;
grid on;
xlabel('Time (Seconds)');
ylabel('Amplitude(rad)');
legend('ref =0','R=10','R=100','R=1000','R=10000','R=100000',"Interpreter", 'latex','Location','southeast', 'FontSize', fs);
title(' Input signal and output response of the system (Theta)','FontSize', 15, 'interpreter','latex');

%% Psi
figure(2);

psi=ones(3001,1)*0.3;
a1 =psi_m1.time;
b1 = [psi_m1.signals.values,psi_m2.signals.values,psi_m3.signals.values,psi_m4.signals.values,psi_m5.signals.values];
draw_controller(a1,b1,5)
hold on;
grid on;
xlabel('Time (Seconds)');
ylabel('Amplitude(rad)');
legend('R=1','R=10','R=100','R=1000','R=10000',"Interpreter", 'latex','Location','southeast', 'FontSize', fs);
title(' Input signal and output response of the system (Psi)','FontSize', 15, 'interpreter','latex');

%% Phi
figure(3);

a2 =phi_m1.time;
b2 = [phi_m1.signals.values,phi_m2.signals.values,phi_m3.signals.values,phi_m4.signals.values,phi_m5.signals.values];
draw_controller(a2,b2,5)
hold on;
grid on;
xlabel('Time (Seconds)');
ylabel('Amplitude(rad)');
legend('R=1','R=10','R=100','R=1000','R=10000',"Interpreter", 'latex','Location','southeast', 'FontSize', fs);
title(' Input signal and output response of the system (Phi)','FontSize', 15, 'interpreter','latex');

%% vl,vr
figure(4);

a3 =vl1.time;
b3 = [vl1.signals.values,vl2.signals.values,vl3.signals.values,vl4.signals.values,vl5.signals.values];
draw_controller(a3,b3,5)
xlabel('Time (Seconds)');
ylabel('Amplitude(V)');
legend('R=1','R=10','R=100','R=1000','R=10000',"Interpreter", 'latex','Location','southeast', 'FontSize', fs);
title('  Control signal of the system(Vl)','FontSize', 15, 'interpreter','latex');
figure(5);

a4 =vr1.time;
b4 = [vr1.signals.values,vr2.signals.values,vr3.signals.values,vr4.signals.values,vr5.signals.values];
draw_controller(a4,b4,5)
xlabel('Time (Seconds)');
ylabel('Amplitude(V)');
legend('R=1','R=10','R=100','R=1000','R=10000',"Interpreter", 'latex','Location','southeast', 'FontSize', fs);
title('  Control signal of the system (Vr)','FontSize', 15, 'interpreter','latex');

%% Error
figure(6);

a5 =e1.time;
b5 = [e1.signals.values,e2.signals.values,e3.signals.values,e4.signals.values,e5.signals.values];
draw_controller(a5,b5,5)
xlabel('Time (Seconds)');
ylabel('Amplitude(rad)');
legend('R=1','R=10','R=100','R=1000','R=10000','R=10000',"Interpreter", 'latex','Location','southeast', 'FontSize', fs);
title(' Error between the input signal and the output response of the system','FontSize', 15, 'interpreter','latex');

