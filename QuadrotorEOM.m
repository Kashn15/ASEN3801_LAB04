function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
%QUADROTOREOM Summary of this function goes here
%   Detailed explanation goes here
x     = var(1);
y     = var(2);
z     = var(3);
phi   = var(4);
theta = var(5);
psi   = var(6);
u     = var(7);
v     = var(8);
w     = var(9);
p     = var(10);
q     = var(11);
r     = var(12);

x_dot = [cos(theta)*cos(psi), (sin(phi)*sin(theta)*cos(psi)) - (cos(phi)*sin(psi)), (cos(phi)*sin(theta)*cos(psi)) + (sin(phi)*sin(psi))] * u;
y_dot = [cos(theta)*sin(psi), (sin(phi)*sin(theta)*sin(psi)) - (cos(phi)*cos(psi)), (cos(phi)*sin(theta)*sin(psi)) + (sin(phi)*cos(psi))] * v;
z_dot = [-sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)] * w;

phi_dot   = [1, sin(phi)*tan(theta), cos(phi)*tan(theta)] * p;
theta_dot = [0,       cos(phi),                -sin(phi)] * q;
psi_dot   = [0, sin(phi)*sec(theta), cos(phi)*sec(theta)] * r;

V_a = sqrt((u^2) + (v^2) + (w^2));
X = nu*(V_a * u);
Y = nu * (V_a * v);
Z = nu * (V_a * w);
Z_c = sum(motor_forces);

u_dot = (r*v) - (q*w) -     g*sin(theta)      + ((1/m) * X);
v_dot = (p*w) - (r*u) + g*cos(theta)*sin(phi) + ((1/m) * Y);
w_dot = (q*u) - (P*v) + g*cos(theta)*cos(phi) + ((1/m) * Z) + (Z_c / m);

omega = sqrt((p^2) + (q^2) + (r^2));
L     = -mu*omega*p;
M     = -mu*omega*q;
N     = -mu*omega*r;
L_c   = (d/sqrt(2)) * (-motor_forces(1) - motor_forces(2) + motor_forces(3) + motor_forces(4));
M_c   = (d/sqrt(2)) * (motor_forces(1) - motor_forces(2) - motor_forces(3) + motor_forces(4));
N_c   = (d*(motor_forces(1) - motor_forces(2) + motor_forces(3) - motor_forces(4)));

p_dot = ((I(2) - I(3))/I(1))*q*r + (L/I(1) + (L_c/I(1)));
q_dot = ((I(3) - I(1))/I(2))*p*r + (M/I(2) + (M_c/I(2)));
r_dot = ((I(1) - I(2))/I(3))*p*q + (N/I(3) + (N_c/I(2)));

var_dot = [x_dot;    y_dot;     z_dot; 
           phi_dot; theta_dot; psi_dot; 
           u_dot;    v_dot;     w_dot; 
           p_dot;    q_dot;     r_dot];

end