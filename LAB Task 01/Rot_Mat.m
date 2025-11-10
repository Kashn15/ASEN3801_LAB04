% Rotation Matrix
function R_EB = Rot_Mat(phi, theta, psi)

R1 = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)];
R2 = [cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)];
R3 = [-sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];

R_EB = [R1; R2; R3];

end