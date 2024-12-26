function [Fc,Gc] = RotationDerivativeFeedback(var,m,g)
% The function takes as input the 12x1 aircraft state var, aircraft mass m,
% and gravitational acceleration g.

% Control force in the body z direction is equal to the weight of the
% quadrotor
Gc = zeros(3,1);
Fc = [0; 0; m*g];


% Control moments about each body axis is proportional to the rotational
% rates about their respective axes

% define gain
k = -0.004; %Nm/(rad/s)
Gc = k.*var(10:12);
end

