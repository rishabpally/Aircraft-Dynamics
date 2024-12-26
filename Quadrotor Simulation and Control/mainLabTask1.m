clc; clear; close all;

%% ASEN 3801 Lab 4 Group 28

fig = 1:6;

% Givens
g = 9.81; 
m = 0.068; %kg
d = 0.060; %m
km = 0.0024; %Nm/N
I_x = 5.8*10^-5; %kgm^2
I_y = 7.2*10^-5; %kgm^2
I_z = 1.0*10^-4; %kgm^2
nu = 1*10^-3; %N/(m/s)^2
mu = 2*10^-6; %N*m/(m/s)^2
motor_forces = m*g/4.*ones(4,1);
I = [I_x 0 0; 0 I_y 0; 0 0 I_z];

quad = @(t,var)QuadrotorEOM(t,var,g,m,I,d,km,nu,mu,motor_forces);
var = [0;0;-5;  0;0;1;  0;0;0; 0;0;0];
t = [0 10];
[time, aircraft_state] = ode45(quad,t,var);
%%

Z_c = -sum(motor_forces);
L_c = d/sqrt(2)*(-motor_forces(1)-motor_forces(2)+motor_forces(3)+motor_forces(4));
M_c = d/sqrt(2)*(motor_forces(1)-motor_forces(2)-motor_forces(3)+motor_forces(4));
N_c = d/sqrt(2)*(motor_forces(1)-motor_forces(2)+motor_forces(3)-motor_forces(4));

control_moments = [Z_c, L_c, M_c, N_c].*ones(numel(time),4);

PlotAircraftSim(time,aircraft_state,control_moments,fig,'-b')

%% 1.4 
fig = fig +6;
V_a = 5;
phi = atan2(V_a^2*nu,m*g);

Z_c = -nu*V_a^2/sin(phi);

motor_forces = -Z_c/4.*ones(4,1);
quad = @(t,var)QuadrotorEOM(t,var,g,m,I,d,km,nu,mu,motor_forces);
var = [0;0;-50;  phi;0;0;  0;cos(phi)*V_a;-sin(phi)*V_a; 0;0;0];
t = [0 10];
[time, aircraft_state] = ode45(quad,t,var);


L_c = d/sqrt(2)*(-motor_forces(1)-motor_forces(2)+motor_forces(3)+motor_forces(4));
M_c = d/sqrt(2)*(motor_forces(1)-motor_forces(2)-motor_forces(3)+motor_forces(4));
N_c = d/sqrt(2)*(motor_forces(1)-motor_forces(2)+motor_forces(3)-motor_forces(4));

control_moments = [Z_c, L_c, M_c, N_c].*ones(numel(time),4);
PlotAircraftSim(time,aircraft_state,control_moments,fig,'-b')

%
quad = @(t,var)QuadrotorEOM(t,var,g,m,I,d,km,nu,mu,motor_forces);
var = [0;0;-50;  0;-phi;pi/2;  cos(phi)*V_a;0;-sin(phi)*V_a; 0;0;0];
t = [0 10];
[time, aircraft_state] = ode45(quad,t,var);
for i = 1:length(time)
V(i) = norm(aircraft_state(i,7:9));
end
figure(8)
plot(time, V)

L_c = d/sqrt(2)*(-motor_forces(1)-motor_forces(2)+motor_forces(3)+motor_forces(4));
M_c = d/sqrt(2)*(motor_forces(1)-motor_forces(2)-motor_forces(3)+motor_forces(4));
N_c = d/sqrt(2)*(motor_forces(1)-motor_forces(2)+motor_forces(3)-motor_forces(4));

control_moments = [Z_c, L_c, M_c, N_c].*ones(numel(time),4);
PlotAircraftSim(time,aircraft_state,control_moments,fig,'-r')


