function [Fc, Gc] = VelocityReferenceFeedback(t,var,lat,long,type)
%VELOCITYREFERENCEFEEDBACK Summary of this function goes here
%   Detailed explanation goes here
g = 9.81; 
m = 0.068; %kg
Fc = [0; 0; m*g];

if (type == 1)
v_r = 1.5; u_r = 0;
elseif (type == 2)
v_r = 0; u_r = 1.5;
end

v_r
u_r
% Control moments about each body axis is proportional to the rotational
% rates about their respective axes

% define gain
k = -0.004; %Nm/(rad/s)
Lc = -(lat.k1)*var(10) - (lat.k2)*var(4) - (lat.k3)*(v_r - var(8));
Mc = -(long.k1)*var(11) - (long.k2)*var(5) - (long.k3)*(u_r - var(7));
Nc = k*var(12);
Gc = [Lc; Mc; Nc];
end

