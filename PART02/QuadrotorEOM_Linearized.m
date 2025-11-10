function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc)
% Unpack State vec
x   = var(1);  y     = var(2);  z   = var(3); 
phi = var(4);  theta = var(5);  psi = var(6);
u   = var(7);  v     = var(8);  w   = var(9); 
p   = var(10); q     = var(11); r   = var(12); 
% Position kinematics
x_dot = u;
y_dot = v;
z_dot = w;

% Euler angle rates
phi_dot   = p;
theta_dot = q;
psi_dot   = r;

% Translational dynamics 
% Gravity coupling into body x,y
Fb_g = [-m*g*theta; m*g*phi; 0];

% Control force deviations (3x1)
F_dev = deltaFc(:);

acc_b = (Fb_g + F_dev)/m;   % [u_dot; v_dot; w_dot]
u_dot = acc_b(1);
v_dot = acc_b(2);
w_dot = acc_b(3);

% Rotational dynamics
p_dot = deltaGc(1)/I(1,1);
q_dot = deltaGc(2)/I(2,2);
r_dot = deltaGc(3)/I(3,3);

% Return State vec
var_dot = [x_dot; y_dot; z_dot;
           phi_dot; theta_dot; psi_dot;
           u_dot; v_dot; w_dot;
           p_dot; q_dot; r_dot];
end
