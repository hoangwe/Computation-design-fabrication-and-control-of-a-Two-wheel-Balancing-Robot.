 clc;
close all;
%% data

%% Plot parameters

imgWidthSize = 500;
imgColumnSize = 500;
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
fig1 = figure('Position', [50, 10, imgWidthSize, imgColumnSize]);
set(gca,'LooseInset', max(get(gca,'TightInset'), 0.02))
set(groot, 'defaultAxesTickLabelInterpreter','latex');
%thetaa
subplot(3,1,1);
a =ref.time;
b = [ref.signals.values,the_m1.signals.values,the_m3.signals.values,the_m4.signals.values,the_m5.signals.values];
draw_controller(a,b,5)
plot(ref.time,ref.signals.values ,'r','LineWidth', lw, 'MarkerSize', ms);
hold on;
% plot(the_m.time,the_m.signals.values  ,'-.r','LineWidth', lw, 'MarkerSize', ms);
% hold on;
plot(the_m1.time,the_m1.signals.values  ,'-.k','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(the_m2.time,the_m2.signals.values  ,'-.b','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(the_m3.time,the_m3.signals.values  ,'--r','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(the_m4.time,the_m4.signals.values  ,'--k','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(the_m5.time,the_m5.signals.values  ,'--b','LineWidth', lw, 'MarkerSize', ms);


grid on;
xlabel('Time (Seconds)');
ylabel('Amplitude(rad)');
legend('ref =3.14','Q11=1','Q11=10','Q11=100','Q11=1000','Q11=10000','Q11=100000',"Interpreter", 'latex','Location','southeast', 'FontSize', fs);

% legend('the=20','Q11=1','Q11=10','Q11=100','Q11=1000','Q11=10000','Q11=20000',"Interpreter", 'latex','Location','southeast', 'FontSize', fs);
title(' Input signal and output response of the system (Theta)','FontSize', 15, 'interpreter','latex');

psi=ones(3001,1)*0.3;
subplot(3,1,2);
plot(ref.time,psi ,'r','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(psi_m.time,psi_m.signals.values  ,'-.r','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(psi_m1.time,psi_m1.signals.values  ,'-.k','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(psi_m2.time,psi_m2.signals.values  ,'-.b','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(psi_m3.time,psi_m3.signals.values  ,'--r','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(psi_m4.time,psi_m4.signals.values  ,'--k','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(psi_m5.time,psi_m5.signals.values  ,'--b','LineWidth', lw, 'MarkerSize', ms);
hold on;
grid on;
xlabel('Time (Seconds)');
ylabel('Amplitude(rad)');
legend('ref =0.3','Q11=1','Q11=10','Q11=100','Q11=1000','Q11=10000','Q11=100000',"Interpreter", 'latex','Location','southeast', 'FontSize', fs);

% legend('psi=0.1','Q11=1','Q11=10','Q11=100','Q11=1000','Q11=10000','Q11=20000',"Interpreter", 'latex','Location','southeast', 'FontSize', fs);
title(' Input signal and output response of the system (Psi)','FontSize', 15, 'interpreter','latex');

phi=zeros(3001,1);
subplot(3,1,3);
plot(ref.time,phi ,'r','LineWidth', lw, 'MarkerSize', ms);
hold on;
% plot(phi_m.time,phi_m.signals.values  ,'-.r','LineWidth', lw, 'MarkerSize', ms);
% hold on;
plot(phi_m1.time,phi_m1.signals.values  ,'-.k','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(phi_m2.time,phi_m2.signals.values  ,'-.b','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(phi_m3.time,phi_m3.signals.values  ,'--r','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(phi_m4.time,phi_m4.signals.values  ,'--k','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(phi_m5.time,phi_m5.signals.values  ,'--b','LineWidth', lw, 'MarkerSize', ms);
hold on;
grid on;
xlabel('Time (Seconds)');
ylabel('Amplitude(rad)');
legend('ref =0','Q11=1','Q11=10','Q11=100','Q11=1000','Q11=10000','Q11=100000',"Interpreter", 'latex','Location','southeast', 'FontSize', fs);

% legend('phi=0','Q11=1','Q11=10','Q11=100','Q11=1000','Q11=10000','Q11=20000',"Interpreter", 'latex','Location','southeast','FontSize', fs);
title(' Input signal and output response of the system (Phi)','FontSize', 15, 'interpreter','latex');



%% vl,vr
fig2 = figure('Position', [50, 10, imgWidthSize, imgColumnSize]);
set(gca,'LooseInset', max(get(gca,'TightInset'), 0.02))
set(groot, 'defaultAxesTickLabelInterpreter','latex');
%thetaa
subplot(2,1,1);
plot(vl.time,vl.signals.values    ,'-.r','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(vl1.time,vl1.signals.values  ,'-.k','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(vl2.time,vl2.signals.values  ,'-.b','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(vl3.time,vl3.signals.values  ,'--r','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(vl4.time,vl4.signals.values  ,'--k','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(vl5.time,vl5.signals.values  ,'--b','LineWidth', lw, 'MarkerSize', ms);
grid on;
xlabel('Time (Seconds)');
ylabel('Amplitude(rad)');
legend('Q11=1','Q11=10','Q11=100','Q11=1000','Q11=10000','Q11=100000',"Interpreter", 'latex','Location','southeast', 'FontSize', fs);

% legend('Q11=1','Q11=10','Q11=100','Q11=1000','Q11=10000','Q11=20000',"Interpreter", 'latex','Location','southeast', 'FontSize', fs);
title('  Control signal of the system','FontSize', 15, 'interpreter','latex');



subplot(2,1,2);

plot(vr.time  ,vr.signals.values  ,'-.r','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(vr1.time,vr1.signals.values  ,'-.k','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(vr2.time,vr2.signals.values  ,'-.b','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(vr3.time,vr3.signals.values  ,'--r','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(vr4.time,vr4.signals.values  ,'--k','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(vr5.time,vr5.signals.values  ,'--b','LineWidth', lw, 'MarkerSize', ms);
hold on;
grid on;
xlabel('Time (Seconds)');
ylabel('Amplitude(rad)');

% legend('Q11=1','Q11=10','Q11=100','Q11=1000','Q11=10000','Q11=20000',"Interpreter", 'latex','Location','southeast', 'FontSize', fs);
legend('Q11=1','Q11=10','Q11=100','Q11=1000','Q11=10000','Q11=100000',"Interpreter", 'latex','Location','southeast', 'FontSize', fs);

title(' Control signal of the system','FontSize', 15, 'interpreter','latex');



%% Error
fig3 = figure('Position', [50, 10, imgWidthSize, imgColumnSize]);
set(gca,'LooseInset', max(get(gca,'TightInset'), 0.02))
set(groot, 'defaultAxesTickLabelInterpreter','latex');
%thetaa

plot(e.time,e.signals.values    ,'-.r','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(e1.time,e1.signals.values  ,'-.k','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(e2.time,e2.signals.values  ,'-.b','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(e3.time,e3.signals.values  ,'--r','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(e4.time,e4.signals.values  ,'--k','LineWidth', lw, 'MarkerSize', ms);
hold on;
plot(e5.time,e5.signals.values  ,'--b','LineWidth', lw, 'MarkerSize', ms);
grid on;
xlabel('Time (Seconds)');
ylabel('Amplitude(rad)');
legend('Q11=1','Q11=10','Q11=100','Q11=1000','Q11=10000','Q11=100000',"Interpreter", 'latex','Location','southeast', 'FontSize', fs);

% legend('Q11=1','Q11=10','Q11=100','Q11=1000','Q11=10000','Q11=20000',"Interpreter", 'latex','Location','southeast', 'FontSize', fs);
title(' Error between the input signal and the output response of the system','FontSize', 15, 'interpreter','latex');

