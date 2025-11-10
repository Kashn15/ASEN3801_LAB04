function var_dot = NonlinearizedEOM_Control_Finalpart(t,var)

%Constants
m=0.068; % kg
g=9.81; % m/s^2
I=diag([5.8e-5 7.2e-5 1.0e-4]); % kg*m^2
km=0.0024; % N*m/(N)
vu = 1e-3; % N/m/s)^2
mu = 2e-6; % N*m/(rad/s)^2
d = 0.060; % m

%State Vector
x = var(1); y = var(2); z = var(3);
phi = var(4); theta = var(5); psi = var(6);
u = var(7); v = var(8); w = var(9);
p = var(10); q = var(11); r = var(12);

omega = [p;q;r];
Vb = [u;v;w];
airspeed = norm(Vb,2);
            

% Control
[Fc,Gc] = VelocityReferenceFunction(t,var);

% Aerodynamics
F_aero = -vu * airspeed * Vb; % N
M_aero = -mu * norm(omega,2) * omega; % N*m

% Rotation matrices
R_EB = Rot_Mat(phi,theta,psi);    % body -> earth
R_BE = R_EB.';   


% Forces and Moments
F_thrust = [0; 0; -Fc(3)];                 % rotor resultant along body z
F_g = m * (R_BE * [0;0;g]);       % gravity expressed in body
F_tot = F_thrust + F_g + F_aero;
M_tot = [Gc(1); Gc(2); Gc(3)] + M_aero;

% Kinematics
posit_dot = R_EB * Vb;
eul_dot = euler_kinematics(phi,theta,psi,omega);

% Dynamics
uvw_dot  = (F_tot - cross(omega, m * Vb)) / m; % Cross() is MatLab's cross function
pqr_dot  = I \ (M_tot - cross(omega, I*omega)); 

var_dot = [posit_dot; eul_dot; uvw_dot; pqr_dot];

end
