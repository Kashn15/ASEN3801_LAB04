% ODE (With Aerodynamics)
function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, vu, mu, motor_forces)

%State Vector
x = var(1); 
y = var(2); 
z = var(3);
phi = var(4); 
theta = var(5); 
psi = var(6);
u = var(7); 
v = var(8); 
w = var(9);
p = var(10); 
q = var(11); 
r = var(12);

omega = [p;q;r];
Vb = [u;v;w];
airspeed = norm(Vb,2);

% Aerodynamics
F_aero = -vu * airspeed * Vb; % N
M_aero = -mu * norm(omega,2) * omega; % N*m

% Rotation matrices
R_EB = Rot_Mat(phi,theta,psi);    % body -> earth
R_BE = R_EB.';                    % earth -> body

% Motor
f = motor_forces(:);
Zc = -(f(1)+f(2)+f(3)+f(4));
Lc = d/sqrt(2) * (-f(1) - f(2) + f(3) + f(4));
Mc = d/sqrt(2) * ( f(1) - f(2) - f(3) + f(4));
Nc = km * ( f(1) - f(2) + f(3) - f(4));

% Forces and Moments
F_thrust = [0; 0; Zc];                 % rotor resultant along body z
F_g = m * (R_BE * [0;0;g]);       % gravity expressed in body
F_tot = F_thrust + F_g + F_aero;
M_tot = [Lc; Mc; Nc] + M_aero;

% Kinematics
posit_dot = R_EB * Vb;
eul_dot = euler_kinematics(phi,theta,psi,omega);

% Dynamics
uvw_dot  = (F_tot - cross(omega, m * Vb)) / m; % Cross() is MatLab's cross function
pqr_dot  = I \ (M_tot - cross(omega, I*omega)); 

var_dot = [posit_dot; eul_dot; uvw_dot; pqr_dot];

end