clear all; close all; clc;
the =0; the_dot = 0; psi = 0;psi_dot = 0; phi = 0; phi_dot= 0;
  %% declare the parameter
  %%% ?i?m c?n tuy?n t�nh % diem can tuyen tinh the = 0; the_dot = 0; psi = 0;psi_dot = 0; phi = 0; phi_dot= 0;
g =9.81; % gravity acceleration
m =0.046; % wheel weight
R =0.0335; % Wheel radius
M =0.945;  % body weight
W =0.17; % Body width
D =0.1; % Body depth
H =0.121;% Body Height
L =H*0.52;% distance
Jsi  = M*L^2/3;% Body inertia moment
Jw = m*R^2/2;  %Wheel inertia moment
J_phi = M*(W^2+D^2)/12; % Body yaw inertia moment
Jm = 0.041185; % Dc motor inertia moment %gear ratio
Rm=8.6123;      %dien tro dong co DC
Kb=0.22244;   %he so emf cua dong co
Kt=3.0896;   %momemt xoan cua dong co DC
fm = 0.18133; % friction cof between body and DC motor.
fw =0;
%% fedffffff
n=30;
a = n*Kt/Rm;
beta = (n*Kt*Kb/Rm) +fm;
vl = 0; vr = 0;
%%

%%
% fm = 0.0022; % friction cof between body and DC motor.
% fw = 0; % friction cof between wheel and floor
 AA = ...
[ 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    0, 0,                                                                                                                                                                                                                                                                                                  0;
  0,          (2*beta*(2*Jm*n^2 - L*M*R*cos(psi)))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)) - ((2*beta + 2*fw)*(M*L^2 + 2*Jm*n^2 + Jsi))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)),  ((2*Jm*n^2 - L*M*R*cos(psi))*(L*M*g*cos(psi) + L^2*M*phi_dot^2*cos(psi)^2 - L^2*M*phi_dot^2*sin(psi)^2))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)) - ((2*cos(psi)*sin(psi)*L^2*M^2*R^2 - 4*Jm*sin(psi)*L*M*R*n^2)*(M*L^2 + 2*Jm*n^2 + Jsi)*(L*M*R*sin(psi)*psi_dot^2 + 2*beta*psi_dot + a*(vl + vr) - the_dot*(2*beta + 2*fw)))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi))^2 - ((2*cos(psi)*sin(psi)*L^2*M^2*R^2 - 4*Jm*sin(psi)*L*M*R*n^2)*(2*Jm*n^2 - L*M*R*cos(psi))*(M*cos(psi)*sin(psi)*L^2*phi_dot^2 + M*g*sin(psi)*L - a*(vl + vr) - 2*beta*psi_dot + 2*beta*the_dot))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi))^2 + (L*M*R*sin(psi)*(M*cos(psi)*sin(psi)*L^2*phi_dot^2 + M*g*sin(psi)*L - a*(vl + vr) - 2*beta*psi_dot + 2*beta*the_dot))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)) + (L*M*R*psi_dot^2*cos(psi)*(M*L^2 + 2*Jm*n^2 + Jsi))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)),          ((2*beta + 2*L*M*R*psi_dot*sin(psi))*(M*L^2 + 2*Jm*n^2 + Jsi))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)) - (2*beta*(2*Jm*n^2 - L*M*R*cos(psi)))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)), 0,         (2*L^2*M*phi_dot*cos(psi)*sin(psi)*(2*Jm*n^2 - L*M*R*cos(psi)))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi));
  0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    1, 0,                                                                                                                                                                                                                                                                                                  0;
  0, (2*beta*(2*Jw + M*R^2 + 2*Jm*n^2 + 2*R^2*m))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)) - ((2*beta + 2*fw)*(2*Jm*n^2 - L*M*R*cos(psi)))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)), ((L*M*g*cos(psi) + L^2*M*phi_dot^2*cos(psi)^2 - L^2*M*phi_dot^2*sin(psi)^2)*(2*Jw + M*R^2 + 2*Jm*n^2 + 2*R^2*m))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)) - ((2*cos(psi)*sin(psi)*L^2*M^2*R^2 - 4*Jm*sin(psi)*L*M*R*n^2)*(2*Jm*n^2 - L*M*R*cos(psi))*(L*M*R*sin(psi)*psi_dot^2 + 2*beta*psi_dot + a*(vl + vr) - the_dot*(2*beta + 2*fw)))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi))^2 - ((2*cos(psi)*sin(psi)*L^2*M^2*R^2 - 4*Jm*sin(psi)*L*M*R*n^2)*(2*Jw + M*R^2 + 2*Jm*n^2 + 2*R^2*m)*(M*cos(psi)*sin(psi)*L^2*phi_dot^2 + M*g*sin(psi)*L - a*(vl + vr) - 2*beta*psi_dot + 2*beta*the_dot))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi))^2 + (L*M*R*sin(psi)*(L*M*R*sin(psi)*psi_dot^2 + 2*beta*psi_dot + a*(vl + vr) - the_dot*(2*beta + 2*fw)))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)) + (L*M*R*psi_dot^2*cos(psi)*(2*Jm*n^2 - L*M*R*cos(psi)))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)), ((2*beta + 2*L*M*R*psi_dot*sin(psi))*(2*Jm*n^2 - L*M*R*cos(psi)))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)) - (2*beta*(2*Jw + M*R^2 + 2*Jm*n^2 + 2*R^2*m))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)), 0, (2*L^2*M*phi_dot*cos(psi)*sin(psi)*(2*Jw + M*R^2 + 2*Jm*n^2 + 2*R^2*m))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi));
  0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    0, 0,                                                                                                                                                                                                                                                                                                  1;
  0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       (8*L^2*M*R^4*cos(psi)*sin(psi)*((R^2*W^2*phi_dot*(beta + fw))/2 + (W*a*(vl - vr))/(2*R) + 2*L^2*M*phi_dot*psi_dot*cos(psi)*sin(psi)))/(2*M*sin(psi)^2*L^2*R^2 + m*R^2*W^2 + 2*J_phi*R^2 + Jm*W^2*n^2 + Jw*W^2)^2 - (2*R^2*(2*L^2*M*phi_dot*psi_dot*cos(psi)^2 - 2*L^2*M*phi_dot*psi_dot*sin(psi)^2))/(2*J_phi*R^2 + Jw*W^2 + Jm*W^2*n^2 + R^2*W^2*m + 2*L^2*M*R^2*sin(psi)^2),                                                                                                                                                                                                                                                                                                                                                                                                                                                    -(4*L^2*M*R^2*phi_dot*cos(psi)*sin(psi))/(2*J_phi*R^2 + Jw*W^2 + Jm*W^2*n^2 + R^2*W^2*m + 2*L^2*M*R^2*sin(psi)^2), 0,                                                                                                                                                     -(2*R^2*((R^2*W^2*(beta + fw))/2 + 2*L^2*M*psi_dot*cos(psi)*sin(psi)))/(2*J_phi*R^2 + Jw*W^2 + Jm*W^2*n^2 + R^2*W^2*m + 2*L^2*M*R^2*sin(psi)^2)]

%%
BB =...
[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   0;
          (a*(M*L^2 + 2*Jm*n^2 + Jsi))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)) - (a*(2*Jm*n^2 - L*M*R*cos(psi)))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)),          (a*(M*L^2 + 2*Jm*n^2 + Jsi))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)) - (a*(2*Jm*n^2 - L*M*R*cos(psi)))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   0;
 (a*(2*Jm*n^2 - L*M*R*cos(psi)))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)) - (a*(2*Jw + M*R^2 + 2*Jm*n^2 + 2*R^2*m))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)), (a*(2*Jm*n^2 - L*M*R*cos(psi)))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi)) - (a*(2*Jw + M*R^2 + 2*Jm*n^2 + 2*R^2*m))/(2*Jsi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + Jsi*M*R^2 + 2*Jsi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*Jsi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(psi)^2 + 4*Jm*L*M*R*n^2*cos(psi));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                 (R*W*a)/(2*J_phi*R^2 + Jw*W^2 + Jm*W^2*n^2 + R^2*W^2*m + 2*L^2*M*R^2*sin(psi)^2),                                                                                                                                                                                                                                                                                                                                                                                                                                                 -(R*W*a)/(2*J_phi*R^2 + Jw*W^2 + Jm*W^2*n^2 + R^2*W^2*m + 2*L^2*M*R^2*sin(psi)^2)]
 

%% c
%  CC = [1 0 0 0 0 0;
%        0 0 1 0 0 0;
%        0 0 0 0 1 0];
CC =eye(6)

% chung minh h? ?i?u khi?n ???c
Mc = [BB AA*BB AA^2*BB AA^3*BB AA^4*BB AA^5*BB];
Mo = [CC; CC*AA; CC*AA^2 ;CC*AA^3; CC*AA^4; CC*AA^5];
ncc = rank(Mc);
noo = rank(Mo)
%% lqr
% QQ =...
%    [ 10000   0     0     0     0     0;
%      0     10000     0     0     0     0;
%      0     0     1     0     0     0;
%      0     0     0     1     0     0;
%      0     0     0     0     1     0;
%      0     0     0     0     0     1];
QQ =...
   [ 10000     0     0     0     0     0;
     0     1000     0     0     0     0;
     0     0     10000   0     0     0;
     0     0     0     100000     0     0;
     0     0     0     0     1000   0;
     0     0     0     0     0     100000];
%  QQ2 =...
%    [ 10000  0     0     0     0     0;
%      0     1000     0     0     0     0;
%      0     0     10000   0     0     0;
%      0     0     0     100000     0     0;
%      0     0     0     0     1000   0;
%      0     0     0     0     0     100000];
%  QQ3 =...
%    [ 10000  0     0     0     0     0;
%      0     1000     0     0     0     0;
%      0     0     10000   0     0     0;
%      0     0     0     100000    0     0;
%      0     0     0     0     1000   0;
%      0     0     0     0     0     100000];
%  QQ4=...
%    [ 10000  0     0     0     0     0;
%      0     1000     0     0     0     0;
%      0     0     10000   0     0     0;
%      0     0     0     100000     0     0;
%      0     0     0     0     1000   0;
%      0     0     0     0     0     100000];
%  QQ5 =...
%    [ 10000  0     0     0     0     0;
%      0     1000     0     0     0     0;
%      0     0     10000   0     0     0;
%      0     0     0     100000     0     0;
%      0     0     0     0     1000   0;
%      0     0     0     0     0     100000];
%  QQ6 =...
%    [ 10000    0     0     0     0     0;
%      0     1000     0     0     0     0;
%      0     0     10000   0     0     0;
%      0     0     0     100000     0     0;
%      0     0     0     0     1000   0;
%      0     0     0     0     0     100000];
RR =  [1 0;0 1];
RR2 = [10 0;0 10];
RR3 = [100 0;0 100];
RR4 = [1000 0;0 1000];
RR5 = [10000 0;0 10000];
RR6 = [100000 0;0 100000];
K = lqr(AA,BB,QQ,RR)
K2 = lqr(AA,BB,QQ,RR)
K3 = lqr(AA,BB,QQ,RR)
K4 = lqr(AA,BB,QQ,RR)
K5 = lqr(AA,BB,QQ,RR)
K6 = lqr(AA,BB,QQ,RR)
%% b? l?c
%  QQ_n =   0.000001*[ 1     0     0     0     0     0;
%                      0     1     0     0     0     0;
%                      0     0     1     0     0     0;
%                      0     0     0     1     0     0;
%                      0     0     0     0     1     0;
%                      0     0     0     0     0     1];% nhieuxw h? th?ng
%  RR_n = 0.11;% nhi?u ?o l??ng
%  LL = lqr(AA',CC',QQ_n,RR_n);
%  

