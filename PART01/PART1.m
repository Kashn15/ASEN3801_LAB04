%% ASEN 3801 | Lab 04 Task 1 ODE Function
% Creators:
% Assignment Due Date:
% Purpose:
% (1) Determine trim Thrust for rotors in steady hovering flight
% (2) Simulate this trim state for 10 seconds and verify that it produces
% equilibrium motion.

clc;
clear;
close all;

%% Toggling Plots (1 on, 0 off)
P1_2 = 0; % Part 1.2
P1_3 = 0; % Part 1.3
P1_4 = 1; % Part 1.4

%% Initialization of Data

% Initialization / Figure stuff
fig1 = [1,2,3,4,5,6];
fig2 = [7,8,9,10,11,12];
fig3 = [13,14,15,16,17,18];
fig4 = [19,20,21,22,23,24];
col1 = '-b';
col2 = '--b';
col3 = '-r';
col4 = '--r';

% Givens
t = 10; % Period [s]
m = 0.068; % kg
g = 9.81; % m/s^2
I = diag([5.8e-5 7.2e-5 1.0e-4]); % kg*m^2
km = 0.0024; % N*m/(N)
vu = 1e-3; % N/m/s)^2
mu = 2e-6; % N*m/(rad/s)^2
d = 0.060; % m

% Hover Conditioning
f_i = (m*g)/4; % Zc = -(f1+f2+f3+f4) = -m*g
motor_forces_hover = f_i * ones(4,1);

Zc = (m*g)/4;   % equals -m*g at hover
Lc = 0; 
Mc = 0; 
Nc = 0;

%% Problem 1.2
if P1_2 == 1
    % Initialization Stuff
    mu = 0; %to ignore Aerodynamic Forces/Moments
    vu = 0; %to ignore Aerodynamic Forces/Moments
    
    % Establishing State Vector
    x_e = 0;
    y_e = 0;
    z_e = 0;
    phi = 0;
    theta = 0;
    psi = 0;
    u_e = 0;
    v_e = 0;
    w_e = 0;
    p = 0;
    q = 0;
    r = 0;
    var1 = [x_e; y_e; z_e; phi; theta; psi; u_e; v_e; w_e; p; q; r];
    
    odefun = @(t,x) QuadrotorEOM(t, x, g, m, I, d, km, vu, mu, motor_forces_hover);
    [t1, X1] = ode45(odefun, [0, t], var1);
    
    U2 = repmat([Zc, Lc, Mc, Nc], length(t1), 1); % size = length(t) x 4
    
    PlotAircraftSim(t1, X1, U2, fig1, col1)
end

%% Problem 1.3
if P1_3 == 1
    % Establishing State Vector
    x_e = 0;
    y_e = 0;
    z_e = 0;
    phi = 0;
    theta = 0;
    psi = 0;
    u_e = 0;
    v_e = 0;
    w_e = 0;
    p = 0;
    q = 0;
    r = 0;
    
    var2 = [x_e; y_e; z_e; phi; theta; psi; u_e; v_e; w_e; p; q; r];
    
    odefun = @(t,x) QuadrotorEOM(t, x, g, m, I, d, km, vu, mu, motor_forces_hover);
    [t2, X2] = ode45(odefun, [0, t], var2);
    
    U2 = repmat([Zc, Lc, Mc, Nc], length(t2), 1); % size = length(t) x 4
    
    PlotAircraftSim(t2,X2,U2,fig2,col2)
end

%% Problem 1.4
if P1_4 == 1
    % CASE 1: Establishing State Vector (ONLY CHANGING 5 m/s EAST)
    x_e1 = 0;
    y_e1 = 0;
    z_e1 = 0;
    phi1 = asin((vu*25) / (m*g));
    theta1 = 0;
    psi1 = 0;
    u_e1 = 0;
    v_e1 = 5;
    w_e1 = 0;
    p1 = 0;
    q1 = 0;
    r1 = 0;

    % Controls for this Trim
    Zc31 = -m*g*cos(phi1);
    Fc31 = [0, 0, Zc31];
    Lc31 = 0;
    Mc31 = 0; 
    Nc31 = 0;
    Gc31 = [Lc31, Mc31, Nc31]; % [Lc, Mc, Nc]
    f_trim31 = MotorForces(Fc31, Gc31, d, km);
    
    var31 = [x_e1; y_e1; z_e1; phi1; theta1; psi1; u_e1; v_e1; w_e1; p1; q1; r1];
    
    odefun = @(t,x) QuadrotorEOM(t, x, g, m, I, d, km, vu, mu, f_trim31);
    [t31, X31] = ode45(odefun, [0, t], var31);
    
    U31 = repmat([Zc31, Lc31, Mc31, Nc31], length(t31), 1); % size = length(t) x 4
    
    PlotAircraftSim(t31,X31,U31,fig3,col3)
    
    % CASE 2: Establishing State Vector (with 90 deg Yaw)
    x_e2 = 0;
    y_e2 = 0;
    z_e2 = 0;
    phi2 = 0;
    theta2 = -asin((vu*25) / (m*g));
    psi2 = deg2rad(90);
    u_e2 = 5 * cos(theta2);
    v_e2 = 0;
    w_e2 = 5 * sin(theta2);
    p2 = 0;
    q2 = 0;
    r2 = 0;

    % Controls for this Trim
    Zc32 = -m*g*cos(theta2);
    Fc32 = [0, 0, Zc31];
    Gc32 = [0, 0, 0]; % [Lc, Mc, Nc] (same as the part above)
    f_trim32 = MotorForces(Fc32, Gc32, d, km);
    
    var32 = [x_e2; y_e2; z_e2; phi2; theta2; psi2; u_e2; v_e2; w_e2; p2; q2; r2];
    
    odefun = @(t,x) QuadrotorEOM(t, x, g, m, I, d, km, vu, mu, f_trim32);
    [t32, X32] = ode45(odefun, [0, t], var32);
    
    U32 = repmat([Zc32, Lc31, Mc31, Nc31], length(t32), 1); % Control array size = length(t) x 4
    
    PlotAircraftSim(t32,X32,U32,fig4,col4)
end

%% Problem 1.5

% Load Data
RSdata = load('RSdata_nocontrol.mat');

% Read in file and see if it is stable plotaircraft sym
