function [var_dot] = QuadrotorEOM_CL_Linearized(t,var,g,m,I,lat,long)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
var_dot = zeros(12,1);
I_x = I(1,1); I_y = I(2,2); I_z = I(3,3);

[Fc, Gc] = InnerLoopFeedback(t,var,lat,long);
% position and attitude
var_dot(1:6) = var(7:12);

var_dot(7:9) = g.*[-var(5); var(4); 0] + 1/m.*Fc;

var_dot(10:12) = [Gc(1)/I_x; Gc(2)/I_y; Gc(3)/I_z];

end
