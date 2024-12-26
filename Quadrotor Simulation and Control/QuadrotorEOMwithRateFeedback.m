function [var_dot] = QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu)
%QUADROTOREOMWITHRATEFEEDBACK Summary of this function goes here
%   Detailed explanation goes here
d = 0.060; %m
km = 0.0024; %Nm/N
var_dot = zeros(12,1);
phi = var(4); theta = var(5); psi = var(6);
I_x = I(1,1); I_y = I(2,2); I_z = I(3,3);
% Aerodynamic Forces
Aero_F = -nu*norm(var(7:9)).*var(7:9);

Aero_M = -mu*norm(var(10:12)).*var(10:12);

[Fc, Gc] = RotationDerivativeFeedback(var,m,g);
motor_forces = ComputeMotorForces(Fc, Gc, d, km);

L_c = d/sqrt(2)*(-motor_forces(1)-motor_forces(2)+motor_forces(3)+motor_forces(4));
M_c = d/sqrt(2)*(motor_forces(1)-motor_forces(2)-motor_forces(3)+motor_forces(4));
N_c = d/sqrt(2)*(motor_forces(1)-motor_forces(2)+motor_forces(3)-motor_forces(4));
%% Position Data 
R = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi); ...
    cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi); ...
    -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];
var_dot(1:3) = R*(var(7:9));

%% Attitude
A = [1 sin(phi)*tan(theta) cos(phi)*tan(theta); 0 cos(phi) -sin(phi); 0 sin(phi)*sec(theta) cos(phi)*sec(theta)];
var_dot(4:6) = A*var(10:12);

%% Velocity (body frame)
u = var(7); v = var(8); w = var(9); p = var(10); q = var(11); r = var(12);
var_dot(7:9) = [r*v - q*w; p*w - r*u; q*u - p*v] + g.*[-sin(theta); cos(theta)*sin(phi); cos(theta)*cos(phi)] + (1/m).*Aero_F + (1/m).*[0;0; -sum(motor_forces)];

%% Rotation Rates
var_dot(10:12) = [(I_y - I_z)/I_x*q*r; (I_z-I_x)/I_y*p*r;(I_x-I_y)/I_z*p*q] +[1/I_x; 1/I_y; 1/I_z].*Aero_M +[1/I_x*L_c; 1/I_y*M_c; 1/I_z*N_c];


end


