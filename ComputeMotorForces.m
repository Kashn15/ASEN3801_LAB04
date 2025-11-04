function motor_forces = ComputeMotorForces(Fc, Gc, d, km)
Zc = Fc(3); Lc = Gc(1); Mc = Gc(2); Nc = Gc(3);

A = [     -1           -1         -1           -1;     % Zc
      -d/sqrt(2)  -d/sqrt(2)   d/sqrt(2)   d/sqrt(2);  % Lc
       d/sqrt(2)  -d/sqrt(2)  -d/sqrt(2)   d/sqrt(2);  % Mc
          km           -km        km           -km ];  % Nc
rhs = [Zc; Lc; Mc; Nc];
motor_forces = A \ rhs;
end
