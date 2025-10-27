clc;
clear;
close all;

function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc)
%State Vector
x = var(1); y = var(2); z = var(3);
phi = var(4); theta = var(5); psi = var(6);
u = var(7); v = var(8); w = var(9);
p = var(10); q = var(11); r = var(12);

%Derivatives
x_dot = u; y_dot = v; z_dot = w;
phi_dot = p; theta_dot = q; psi_dot = r;
u_dot = -g*theta; v_dot = g*phi; w_dot = deltaFc*(1/m);
p_dot = (1/I(1))*deltaGc(1); q_dot = (1/I(2))*deltaGc(2); r_dot = (1/I(3))*deltaGc(3);

var_dot = [x_dot,y_dot,z_dot;phi_dot,theta_dot,psi_dot;u_dot,v_dot,w_dot;p_dot,q_dot,r_dot];

end
