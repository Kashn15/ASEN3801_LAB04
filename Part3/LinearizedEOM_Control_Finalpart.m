function var_dot = LinearizedEOM_Control_Finalpart(t,var)

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

%Control Forces and Moments
[Fc,Gc] = VelocityReferenceFunction(t,var);

%Linearized EOM
x_dot = u; y_dot = v; z_dot = w;
phi_dot = p; theta_dot = q; psi_dot = r; 
u_dot = -g*theta; v_dot = g*phi; w_dot = Fc(3)*(1/m)-g; 
p_dot = (1/I(1,1))*Gc(1); q_dot = (1/I(2,2))*Gc(2); r_dot = (1/I(3,3))*Gc(3);

%New state variable
var_dot = [x_dot;y_dot;z_dot;phi_dot;theta_dot;psi_dot;u_dot;v_dot;w_dot;p_dot;q_dot;r_dot];

end