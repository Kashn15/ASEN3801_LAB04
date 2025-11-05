function [Fc,Gc] = InnerLoopFeedback(var)

%State variables
phi = var(4); theta = var(5); psi = var(6);
p = var(10); q = var(11); r = var(12);

%Constants
m=0.068; % kg
g=9.81; % m/s^2
I=diag([5.8e-5 7.2e-5 1.0e-4]); % kg*m^2
km=0.0024; % N*m/(N)
vu = 1e-3; % N/m/s)^2
mu = 2e-6; % N*m/(rad/s)^2
d = 0.060; % m

% Roots(Chose -2 since 1/0.5; choosing seconded 5-10x larger
pole_1 = 2; pole_2 = 15;

%Roll Feedback Gains
k1 = (pole_1+pole_2)*I(1,1); k2 = (pole_1*pole_2)*I(1,1);

%Pitch Feedback Gains
k3 = (pole_1+pole_2)*I(2,2); k4 = (pole_1*pole_2)*I(2,2);

%Angular rate feedback
k5 = 0.004;

%Control Force
Fc = [0;0;m*g];

%Control Moments
Lc = -k1*p -k2*phi;
Mc = -k3*q - k4*theta;
Nc = -k5*r;

% Control Moments vector
Gc = [Lc; Mc; Nc];
