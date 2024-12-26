function [Fc, Gc] = InnerLoopFeedback(t,var,lat,long)
%INNERLOOPFEEDBACK Summary of this function goes here
%   Detailed explanation goes here
%   Detailed explanation goes here
g = 9.81; 
m = 0.068; %kg
Fc = [0; 0; m*g];


% Control moments about each body axis is proportional to the rotational
% rates about their respective axes

% define gain
k = -0.004; %Nm/(rad/s)
Lc = -(lat.k1)*var(10) - (lat.k2)*var(4);
Mc = -(long.k1)*var(11) - (long.k2)*var(5);
Nc = k*var(12);
Gc = [Lc; Mc; Nc];
end

