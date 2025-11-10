function [Fc,Gc] = VelocityReferenceFunction(t,var)

%State variables
x = var(1); y = var(2); z = var(3);
phi = var(4); theta = var(5); psi = var(6);
u = var(7); v = var(8); w = var(9);
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
k1_lat = (pole_1+pole_2)*I(1,1); k2_lat = (pole_1*pole_2)*I(1,1);
k3_lat = 7.8e-05;

%Pitch Feedback Gains
k1_long = (pole_1+pole_2)*I(2,2); k2_long = (pole_1*pole_2)*I(2,2);
k3_long = -7.8e-05;

%Angular rate feedback
k5 = 0.004;

%Velocities
u_r = 0; v_r = 0;
if t<2
    u_r = 0.5;
    % v_r = 0.5; Commented out so only x velo is changing
end

%Control Force
Fc = [0;0;m*g];

%Control Moments
Lc = -k1_lat*p -k2_lat*phi+k3_lat*(v_r-v);
Mc = -k1_long*q - k2_long*theta+k3_long*(u_r-u);
Nc = -k5*r;

% Control Moments vector
Gc = [Lc; Mc; Nc];

end