function [motor_forces] = ComputeMotorForces(Fc,Gc, d,km)
% a function to calculate the motor thrust forces given the control force and moments
%   motor_forces is a 4x1 column vector [f1; f2; f3; f4]
M = [-1 -1 -1 -1; -d/sqrt(2) -d/sqrt(2) d/sqrt(2) d/sqrt(2);
    d/sqrt(2) -d/sqrt(2) -d/sqrt(2) d/sqrt(2); km -km km -km];
controls = [Fc(3);Gc];
motor_forces = -1*(M\controls);

end

