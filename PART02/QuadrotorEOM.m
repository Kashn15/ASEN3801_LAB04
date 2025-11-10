function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
% Unpack state
xE   = var(1);
yE   = var(2); 
zE   = var(3); 
phi  = var(4);
th   = var(5);
psi  = var(6);
u    = var(7);
v    = var(8);
w    = var(9);
p    = var(10);
q    = var(11);
r    = var(12);

vb    = [u; v; w];
omega = [p; q; r];

%% Rotations
Rz  = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
Ry  = [cos(th) 0 sin(th); 0 1 0; -sin(th) 0 cos(th)];
Rx  = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
Rbi = Rz*Ry*Rx;          % body -> inertial
Rib = Rbi.';             % inertial -> body

% Position kinematics
pos_dot = Rbi * vb;
x_dot = pos_dot(1);
y_dot = pos_dot(2);
z_dot = pos_dot(3);

% Euler-angle kinematics
T = [1,  sin(phi)*tan(th),  cos(phi)*tan(th);
     0,  cos(phi),         -sin(phi);
     0,  sin(phi)/cos(th),  cos(phi)/cos(th)];
eul_dot   = T * omega;
phi_dot   = eul_dot(1);
theta_dot = eul_dot(2);
psi_dot   = eul_dot(3);

%% Controls: map motor forces
A = [ -1  -1  -1  -1;
      -d/sqrt(2)  -d/sqrt(2)   d/sqrt(2)   d/sqrt(2);
       d/sqrt(2)  -d/sqrt(2)  -d/sqrt(2)   d/sqrt(2);
       km   -km    km         -km ];
u_ctrl = A * motor_forces(:);
Zc = u_ctrl(1);  % body-Z force (positive down)
Lc = u_ctrl(2);  % roll moment
Mc = u_ctrl(3);  % pitch moment
Nc = u_ctrl(4);  % yaw moment

%% Forces in body
% Gravity expressed in body
Fg_b    = Rib * [0; 0; m*g];

% Aerodynamic drag
Faero_b = -nu * [u*abs(u); v*abs(v); w*abs(w)];

% Control thrust in body
Fctrl_b = [0; 0; Zc];

Ftot_b  = Fctrl_b + Fg_b + Faero_b;

% Translational dynamics
acc_b = (1/m)*Ftot_b - cross(omega, vb);
u_dot = acc_b(1);
v_dot = acc_b(2);
w_dot = acc_b(3);

%% Moments and rotational dynamics
M_ctrl = [Lc; Mc; Nc];
M_aero = -mu * [p*abs(p); q*abs(q); r*abs(r)];
Mtot   = M_ctrl + M_aero;

omega_dot = I \ (Mtot - cross(omega, I*omega));
p_dot = omega_dot(1);
q_dot = omega_dot(2);
r_dot = omega_dot(3);

%% State derivative 
var_dot = [x_dot; y_dot; z_dot;
           phi_dot; theta_dot; psi_dot;
           u_dot; v_dot; w_dot;
           p_dot; q_dot; r_dot];
end
