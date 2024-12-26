function [var_dot] = QuadrotorEOM_Linearized(t,var,g,m,I,deltaFc,deltaGc)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
var_dot = zeros(12,1);
I_x = I(1,1); I_y = I(2,2); I_z = I(3,3);
% position and attitude
var_dot(1:6) = var(7:12);

var_dot(7:9) = g.*[-var(5); var(4); 0] + 1/m.*deltaFc;

var_dot(10:12) = [deltaGc(1)/I_x; deltaGc(2)/I_y; deltaGc(3)/I_z];

end

