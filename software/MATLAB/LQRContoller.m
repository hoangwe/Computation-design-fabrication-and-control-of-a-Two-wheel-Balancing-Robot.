%Programed by: Nguyen Minh Hoang
% date: 11/10/2022
% Linearization:
clear all; 
clc; close all;
% declare the parameter
g =9.81; % gravity acceleration
m =0.046; % wheel weight
R =0.0335; % Wheel radius
M =0.945;  % body weight
W =0.17; % Body width
D =0.1; % Body depth
H =0.121;% Body Height
L =0.121/2;% distance


J_psi  = M*L^2/3; % Body inertia moment
Jw = m*R^2/2;  %Wheel inertia moment
J_phi = M*(W^2+D^2)/12; % Body yaw inertia moment


Jm = 0.041185; % Dc motor inertia moment
n =30 ; %gear ratio
Rm=8.6123;      %dien tro dong co DC
Kb=0.22244;   %he so emf cua dong co
Kt=3.0896;   %momemt xoan cua dong co DC
fm = 0.0022; % friction cof between body and DC motor.
fw = 0; % friction cof between wheel and floor
% 
% Rm = 1.5628; Kt = 0.023385; Jm = 1.175e-05; Kb = 5.9233e-07; 
% 
anp = (n*Kt)/Rm;
bet = (n*Kt*Kb)/Rm +fm;

% x1_init = 0.01;
% x2_init = 0.01;
% x3_init = 0.01;
% x4_init = 0.01;
% x5_init = 0.01;
% x6_init = 0.01;

% vl=0;vr=0;x1=0;x2=0;x3=0;x6=0;x9=0;x4=0;x5=0;x7=0;x8=0; m= 0.04; n=20; R=0.0325;M=0.8;L=0.0628; Jw = m*R^2/2; J_psi = M*L^2/3;
% Rm = 1.5628; Kt = 0.023385; Jm = 1.175e-05; Kb = 5.9233e-07; anp = n*Kt/Rm; fm=0.0022;
% bet = n*Kt*Kb/Rm + fm;
% W = 0.168;
% fw = 0.0000001;
% D = 0.098;
% J_phi = M*(W^2+D^2)/12;
% g = 9.81;

%% A Matrix:
Am = (2*J_psi*Jw + 2*Jw*L^2*M + J_psi*M*R^2 + 2*J_psi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*J_psi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 + 4*Jm*L*M*R*n^2);
A1 = -(2*(J_psi*bet + J_psi*fw + L^2*M*bet + L^2*M*fw + 2*Jm*fw*n^2 + L*M*R*bet))/Am;
A2 =  (L*M*g*(2*Jm*n^2 - L*M*R))/Am;
A3 =  ( 2*bet*(M*L^2 + M*R*L + J_psi))/Am;
A4 =  (2*(2*Jw*bet + M*R^2*bet - 2*Jm*fw*n^2 + 2*R^2*bet*m + L*M*R*bet + L*M*R*fw))/Am;
A5 =  (L*M*g*(2*Jw + M*R^2 + 2*Jm*n^2 + 2*R^2*m))/Am;
A6 = -(2*bet*(2*Jw + M*R^2 + 2*R^2*m + L*M*R))/Am;
A7 = -(W^2*(bet + fw))/(m*R^2*W^2 + 2*J_phi*R^2 + Jm*W^2*n^2 + Jw*W^2);
AA=...  
      [0,   1,   0,   0,   0,   0;
       0,  A1,  A2,  A3,   0,   0;
       0,   0,   0,   1,   0,   0;
       0,  A4,  A5,  A6,   0,   0;
       0,   0,   0,   0,   0,   1;
       0,   0,   0,   0,   0,  A7]

%% B Matrix:
Bm = (2*J_psi*Jw + 2*Jw*L^2*M + J_psi*M*R^2 + 2*J_psi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*J_psi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 + 4*Jm*L*M*R*n^2);
B1 = (anp*(M*L^2 + M*R*L + J_psi))/Bm;
B2 = -(anp*(2*Jw + M*R^2 + 2*R^2*m + L*M*R))/Bm;
B3 = (R*W*anp)/(m*R^2*W^2 + 2*J_phi*R^2 + Jm*W^2*n^2 + Jw*W^2);

BB =...
        [ 0,  0;
         B1, B1;
          0,  0;
         B2, B2;
          0,  0;
         -B3,B3]


C = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0];
                                                                                                                                                                                                                                                                                                                                                                       
%% FIND LQR
Q=diag([1000,10,1000,0.001,300,0.1]);


R_ = diag([1,1]);
% K=lqr(A,B,Q,R_)
K=lqr(AA,BB,Q,R_)


%%
C = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0];
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% G_ = diag([1,1,1,1,1,1])*0.005;  
% QN=diag([1,1,1,1,1,1])*0.08;
% RN=diag([1,0.0075,1]);
% L=lqe(AA,G_,C,QN,RN)