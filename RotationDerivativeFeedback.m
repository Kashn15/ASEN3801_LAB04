function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)
% Rate damping only, keep thrust equal to weight.
p = var(10); q = var(11); r = var(12);

Fc = [0; 0; -m*g];      % hover force (body z is down)
K = 0.004;          
Gc = -K * [p; q; r];
end

