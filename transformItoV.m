function [T] = transformItoV(ipl)
% ipl(1) = x = x position in inertial frame
% ipl(2) = y = y position in inertial frame
% ipl(3) = z = z position in inertial frame
% ipl(4) = u = x velocity in vehicle frame
% ipl(5) = v = y velocity in vehicle frame
% ipl(6) = w = z velocity in vehicle frame
% ipl(7) = p = roll rate in vehicle frame
% ipl(8) = q = pitch rate in vehicle frame
% ipl(9) = r = yaw rate in vehicle frame
% ipl(10) = phi = roll Euler angle in inertial frame
% ipl(11) = theta = pitch Euler angle in inertial frame
% ipl(12) = psi = yaw Euler angle in inertial frame
phi = ipl(10);
theta = ipl(11);
psi = ipl(12);

T = [cos(theta)*cos(psi)     cos(theta)*sin(psi)     -sin(theta);
    sin(phi)*sin(theta)*cos(psi)-sin(psi)*cos(phi)        cos(phi)*cos(psi)+sin(phi)*sin(psi)*sin(theta) sin(phi)*cos(theta);
    sin(theta)*cos(psi)*cos(phi)+sin(phi)*sin(psi) cos(phi)*sin(theta)*sin(psi)-cos(psi)*sin(phi) cos(phi)*cos(theta)];