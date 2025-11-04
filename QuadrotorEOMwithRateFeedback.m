function var_dot = QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu)
d  = 0.060;
km = 0.0024;

[Fc, Gc]      = RotationDerivativeFeedback(var, m, g);
motor_forces  = ComputeMotorForces(Fc, Gc, d, km);

var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces);
end
