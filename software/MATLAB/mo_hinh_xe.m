clear all; close all;clc;
%% giá tr? ban ??u
x1_in=0.01;% 
x2_in=0.01;
x4_in=0.01;
x5_in=0.01;
x7_in=0.01;
x8_in=0.01;
% m=1%khoil bánh xe
% M = 5;% khoi l??ng toàn xe
% R = 0.0725;% bán kính bánh xe
% W = 0.24; % chi?u r?ng xe
% D = 0.2; %b? dày xe
% H = 0.5;% chi?u cao xe
% L = 0.18;% chi?u cao thân xe
% fw = 0.18;% h? s? ma sát m?t di chuy?n
% fm = 0.002;% h? s? ma sát ??ng c?
% Jm = m*R^2/2;% momen quán tính ?c
% Jsi = M*L^2/3; % momen quan tính ph?n thân xe
% J_phi = M*(W^2 + D^2)/12; % momen quan tinh toàn xe
% Jw = m*R^2/2; %momen quan tinh banh xe
% Rm = 50; % ddien tr? 
% Kb = 0.468; % h? s? dc
% Kt = 0.317; % h? s? dc
% n = 40; % t? s? truy?n dc
% g = 9.81; %gia tóc tt



m=0.044;%khoil bánh xe
M = 0.92;% khoi l??ng toàn xe
R = 0.0267;% bán kính bánh xe
W = 0.145; % chi?u r?ng xe
D = 0.055;  %b? dày xe
H = 0.185;% chi?u cao xe
L = 0.1;% chi?u cao thân xe
fw = 0;% h? s? ma sát m?t di chuy?n
fm = 0.0022; % h? s? ma sát ??ng c?
Jm = 0.000014762; % momen quán tính ?c
Jsi = M*L^2/3; % momen quan tính ph?n thân xe
J_phi = M*(W^2 + D^2)/12; % momen quan tinh toàn xe
Jw = m*R^2/2; %momen quan tinh banh xe
Rm =40;  % ddien tr? 
Kb=0.0218;   %he so emf cua dong co
Kt=0.0197;   %momemt xoan cua dong co DC
n=30;       %ti so giam toc
g=9.81;     %gia toc trong truong
beta = (n*Kt*Kb)/Rm + fm;
a=(n*Kt)/Rm;

