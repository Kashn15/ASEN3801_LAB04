% Rotational Kinematics
function eul_dot = euler_kinematics(phi, theta, psi, pqr)
p = pqr(1); 
q = pqr(2); 
r = pqr(3);
 
cphi = cos(phi); 
sphi = sin(phi);
cth  = cos(theta); 
sth = sin(theta);

T = [ 1, sphi*sth/cth, cphi*sth/cth;   % = [1, sin(phi)*tan(theta), cos(phi)*tan(theta)]
      0, cphi, -sphi;
      0, sphi/cth, cphi/cth];

eul_dot = T * [p; q; r];

end     