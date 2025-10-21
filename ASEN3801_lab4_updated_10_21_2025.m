% Matthew Buccio
% ASEN 3801
% lab 4
% Created: 10/14/25

clc
clear
close all

const = get_constants();
%% testing
x_dot_E = 1;
y_dot_E = 2;
z_dot_E = 3;
theta = 0;
psi = 0;
phi = 90;
u_E = 4;
v_E = 5;
w_E = 6;

phi_dot = 0;
theta_dot = 0;
psi_dot = 0;
p = 7;
q = 8;
r = 9;

u_dot_E = 0;
v_dot_E = 0;
w_dot_E = 0;

X = 10;
Y = 11;
Z = 12;

Z_c = 0;

p_dot = 0;
q_dot = 0;
r_dot = 0;

L = 0;
M = 0;
N = 0;

L_c = 0;
M_c = 0;
N_c = 0;

%% setting up matrixs

% Kinematics and Dynamics
% p = psi    w = psi  t = theta 


Kin_1 = [x_dot_E; y_dot_E; z_dot_E];% left had side (unused)
Kin_2 = [cos(theta)*cos(psi)    sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)     cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
         cos(theta)*sin(psi)    sin(psi)*sin(theta)*sin(psi)+cos(psi)*cos(psi)       cos(psi)*sin(theta)*sin(psi)-sin(psi)*cos(psi);
         -sin(theta)              sin(psi)*cos(theta)           cos(psi)*cos(theta)];
Kin_3 = [u_E; v_E; w_E];

Kin_4 = [phi_dot; theta_dot; psi_dot];% left had side (unused)
Kin_5 =[1 sin(psi)*tan(theta)    cos(psi)*tan(theta);
         0   cos(psi)   -sin(psi);
         0    sin(psi)*sec(theta)     cos(psi)*sec(theta)];
Kin_6 =[p; q; r];

Kin_7 = Kin_2 * Kin_3;
Kin_8 = Kin_5 * Kin_4;


Dyn_1 = [u_dot_E; v_dot_E; w_dot_E]; % left had side (unused)
Dyn_2 = [r*v_E - q*w_E; p*w_E - r*u_E; q*u_E - p*v_E];
Dyn_3 = [-sin(theta);  cos(theta)*sin(psi);  cos(theta)*cos(psi)];
Dyn_4 = [X; Y; Z];
Dyn_5 = [0; 0; Z_c];
Dyn_6 = [p_dot; q_dot; r_dot];% left had side (unused)
Dyn_7 = [((const.I_y-const.I_z)/const.I_x)*q*r;  ((const.I_z-const.I_x)/const.I_y)*p*r;  ((const.I_x-const.I_y)/const.I_z)*p*q];
Dyn_8 = [L/const.I_x; M/const.I_y; N/const.I_z];
Dyn_9 = [L_c/const.I_x; M_c/const.I_y; N_c/const.I_z];


Dyn_10 = Dyn_2 + const.g * Dyn_3 + (1/const.m) * Dyn_4 + (1/const.m) * Dyn_5; 
Dyn_11 = Dyn_7 + Dyn_8 + Dyn_9;



Eom = [Kin_7; Kin_8; Dyn_10; Dyn_11]





%% constants
function const = get_constants()

    const.m = 0.068; % kg
    const.d = 0.060; % m
    const.k_m = 0.0024; % N*m/(N)
    const.I_x = 5.8*10^-5; % kg*m^2
    const.I_y = 7.2*10^-5; % kg*m^2
    const.I_z = 1.0*10^-5; % kg*m^2
    const.v = 1*10^-3; % N/(m/s)^2
    const.mu = 2*10^-6; % (N*m)/(rad/s)^2
    const.g = 9.81; % m/s^2


end