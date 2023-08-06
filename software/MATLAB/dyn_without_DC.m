%Programed by : Nguyen Minh Hoang
%Date:26/09/2022

%% declare the parameter
% g =9.81; % gravity acceleration
% m =0.03; % wheel weight
% R =0.04; % Wheel radius
% M =0.6;  % body weight
% W =0.14; % Body width
% D =0.04; % Body depth
% H =0.144;% Body Height
% L =0.144/2;% distance
% J_psi  = M*L^2/3; % Body inertia moment
% Jw = m*R^2/2;  %Wheel inertia moment
% J_phi = M*(W^2+D^2)/12; % Body yaw inertia moment
% Jm = 0.00001; % Dc motor inertia moment
% Rm = 6.69; % Dc motor resitance
% Kb = 0.468; % DC motor EMF
% Kt = 0.317; % DC motor torque const
% n =1 ; %gear ratio
% fm = 0.0022; % friction cof between body and DC motor.
% fw = 0; % friction cof between wheel and floor


clc; clear all; close all;
syms m M R g Jw Jm J_phi J_psi L n anp bet  W  vl vr  n ...
    the the_dot the_2dot psi psi_dot psi_2dot phi phi_dot phi_2dot;
tau1 = ((2*m + M)*R^2 + 2*Jw )*the_2dot + (M*L*R*cos(psi))*psi_2dot - M*L*R*(psi_dot)^2*sin(psi);
tau2 = (M*L*R*cos(psi))*the_2dot + (M*L^2 + J_psi)*psi_2dot - (M*g*L*sin(psi) + M*L^2*(phi_dot)^2*sin(psi)*cos(psi));
tau3 = (0.5*m*W^2 + J_phi + 0.5*W^2*(Jw)/R^2 + M*L^2*sin(psi)^2)*phi_2dot + 2*M*L^2*psi_dot*phi_dot*sin(psi)*cos(psi);
%% Mass of Inertia Matrix

M11 = diff(tau1,the_2dot);
M12 = diff(tau1,psi_2dot);
M13 = diff(tau1,phi_2dot);

M21 = diff(tau2,the_2dot);
M22 = diff(tau2,psi_2dot);
M23 = diff(tau2,phi_2dot);

M31 = diff(tau3,the_2dot);
M32 = diff(tau3,psi_2dot);
M33 = diff(tau3,phi_2dot);

M = simplify([M11,M12,M13;
    M21,M22,M23;
    M31,M32,M33])
% M = simplify([diff(tau1,the_2dot),diff(tau1,psi_2dot),diff(tau1,phi_2dot);
%               diff(tau2,the_2dot),diff(tau2,psi_2dot),diff(tau2,phi_2dot);
%               diff(tau3,the_2dot),diff(tau3,psi_2dot),diff(tau3,phi_2dot)])
%% Gravity Terms
G = simplify([diff(tau1,g)*g;diff(tau2,g)*g;diff(tau3,g)*g])
%% Coriolis Matrix
thedd = [the,psi,phi,the_dot,psi_dot,phi_dot];
ddthe = diag([the_dot, psi_dot, phi_dot]);
dthe = diag([the_dot, psi_dot, phi_dot, the_2dot, psi_2dot, phi_2dot ]);
dei = transpose([the_dot psi_dot phi_dot]);
ddei = transpose([the_2dot psi_2dot phi_2dot]);


dM = simplify([sum(ddthe*gradient(M(1,1),  thedd(1:3))) ,sum(ddthe*gradient(M(1,2),  thedd(1:3))) ,sum(ddthe*gradient(M(1,3),  thedd(1:3)));
    sum(ddthe*gradient(M(2,1),  thedd(1:3))) ,sum(ddthe*gradient(M(2,2),  thedd(1:3))) ,sum(ddthe*gradient(M(2,3),  thedd(1:3)));
    sum(ddthe*gradient(M(3,1),  thedd(1:3))) ,sum(ddthe*gradient(M(3,2),  thedd(1:3))) ,sum(ddthe*gradient(M(3,3),  thedd(1:3)))]);
dMe1 = Com_dMdei(M,the_dot);
dMe2 = Com_dMdei(M,psi_dot);
dMe3 = Com_dMdei(M,phi_dot);



t = transpose([tau1, tau2, tau3]);
% C = simplify([dei.'*dMthe1;dei.'*dMthe2;dei.'*dMthe3])
n=3;
e=[the;psi;phi];
d_e=[the_dot;psi_dot;phi_dot];
gamma =Chris(n,M,e);
C11=[gamma(1,1),gamma(1,2),gamma(1,3)]*d_e;
C12=[gamma(1,4),gamma(1,5),gamma(1,6)]*d_e;
C13=[gamma(1,7),gamma(1,8),gamma(1,9)]*d_e;

C21=[gamma(1,10),gamma(1,11),gamma(1,12)]*d_e;
C22=[gamma(1,13),gamma(1,14),gamma(1,15)]*d_e;
C23=[gamma(1,16),gamma(1,17),gamma(1,18)]*d_e;

C31=[gamma(1,19),gamma(1,20),gamma(1,21)]*d_e;
C32=[gamma(1,22),gamma(1,23),gamma(1,24)]*d_e;
C33=[gamma(1,25),gamma(1,26),gamma(1,27)]*d_e;

C=simplify([C11,C12,C13;
    C21,C22,C23;
    C31,C32,C33]);

%% Check
simplify(t - M*ddei - C*dei - G)
% Check M-2C is skew symmetric
check_1 = simplify((transpose(dM-2*C))+(dM-2*C))
% M is the symmetric positive-definite mass matrix
check_2 = simplify(transpose(M)-M)
%% Print
fileG = fopen('Gc1.txt','w');
fprintf(fileG,'G=...\n[%s;\n%s;\n%s];',G(1,1),G(2,1),G(3,1));
%
fileM = fopen('Mc1.txt','w');
fprintf(fileM,'M = ...\n[%s,%s,%s;\n %s,%s,%s;\n %s,%s,%s];',M(1,1),M(1,2),M(1,3),M(2,1),M(2,2),M(2,3),M(3,1),M(3,2),M(3,3));

fileC = fopen('Cc1.txt','w');
fprintf(fileC,'C = ...\n[%s,%s,%s;\n %s,%s,%s;\n %s,%s,%s];',C(1,1),C(1,2),C(1,3),C(2,1),C(2,2),C(2,3),C(3,1),C(3,2),C(3,3));

filetau = fopen('tau1.txt','w');
fprintf(filetau,'tau =...\n[%s;\n%s;\n%s];',tau1,tau2,tau3);

