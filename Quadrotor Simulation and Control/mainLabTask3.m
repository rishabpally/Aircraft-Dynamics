close all; clear; clc;
%% Variables
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
I = [I_x 0 0; 0 I_y 0; 0 0 I_z]; % Moment of Inertias
t = [0 10];

%% Design Feedback control system 3.1

T1 = 0.5;
T2 = 0.05;

lam1 = -1/T1;
lam2 = -1/T2;

lat.k1 = -(lam1+lam2)*I_x;
lat.k2 = lam1*lam2*I_x;

long.k1 = -(lam1+lam2)*I_y;
long.k2 = lam1*lam2*I_y;

%% 3.5 

% Lateral Control: find k3
figure(101)
title("Lateral")
hold on;
xline(0); yline(0); xline(-0.8,':k');
for idx = 1:15^3
k3 = idx * 0.000001;

A = [0 g 0; 0 0 1; -k3/I_x -lat.k2/I_x -lat.k1/I_x];

[V,D] = eig(A);

for i = 1:3
x1(i) = real(D(i,i));
y1(i) = imag(D(i,i));
end
if (all(x1 < -0.8)) && (all(y1 == 0))
plot(x1,y1,'go',LineWidth=5)
lat.k3 = k3;
else
plot(x1,y1,'ko')
end
end

% Longitudinal Control: find k3
figure(102)
title('Longitudinal')
hold on;
xline(0); yline(0); xline(-0.8,':k');
for idx = 1:10^3
k3 = -idx * 0.000001;

A = [0 -g 0; 0 0 1; -k3/I_x -long.k2/I_x -long.k1/I_x];

[V,D] = eig(A);

for i = 1:3
x1(i) = real(D(i,i));
y1(i) = imag(D(i,i));
end
if (all(x1 < -0.8)) && (all(y1 == 0))
plot(x1,y1,'go',LineWidth=5)
long.k3 = k3;
else
plot(x1,y1,'ko')
end
end

%% 3.3 & 3.4 & 3.7

% Figure Nonsense
fig1 = (1:6)'; % For 1a
fig2 = (7:12)'; % For 1b
fig3 = (13:18)'; % For 1c
fig4 = (19:24)'; % For 1d
fig5 = (25:30)'; % For 1e
fig6 = (31:36)';  % For 1f
figures = [fig1, fig2, fig3, fig4, fig5, fig6]; % Combining into an array
col = ['b', 'r','g']; % Choice of color
titles = ["Problem 1a (+5 degree roll deviation)", "Problem 1b (+5 degree pitch deviation)", "Problem 1c (+5 degree yaw deviation)", "Problem 1d (+0.1 [rad/s] roll rate deviation)", "Problem 1e (+0.1 [rad/s] pitch rate deviation)", "Problem 1f (+0.1 [rad/s] yaw rate deviation)"];

% Deviatons
for i = 1:4

    deviations = [0;0;0; 0;0;0; 0;0;0; 0;0;0;]; % Initiallizing 12x1 vector of zeros for state vector
    var = [0;0;-20;  0;0;0;  0;0;0; 0;0;0]; % trim state vector from Lab Task 1
    if i <3
       deviations(i+3) = 5 * (pi/180); % [degrees]
    else
       deviations(i+7) = 0.1; % [rad/s]
    end
    var = var + deviations;

    % Linearized
    quadLinear = @(t,var)QuadrotorEOM_CL_Linearized(t,var,g,m,I,lat,long);
    [timeL, aircraft_state_Linearized] = ode45(quadLinear,t,var);
    
    controls = zeros(length(aircraft_state_Linearized),4);
    for j = 1:length(aircraft_state_Linearized)
    [Fc, Gc] = InnerLoopFeedback(t,aircraft_state_Linearized(j,:),lat,long);
    %motor_forces = ComputeMotorForces(Fc, Gc, d, km);
    controls(j,:) = [Fc(3),Gc'];
    end
    PlotAircraftSim(timeL,aircraft_state_Linearized,controls,figures(:,i),col(1));

    % Feedback
    quadFeedback = @(t,v)QuadrotorEOMwithRateFeedback_CL(t,v, g, m, I, nu, mu,lat,long);
    [timeF,aircraft_state_Feedback] = ode45(quadFeedback,t,var);

    controls = zeros(length(aircraft_state_Feedback),4);
    for j = 1:length(aircraft_state_Feedback)
    [Fc, Gc] = InnerLoopFeedback(t,aircraft_state_Feedback(j,:),lat,long);
    %motor_forces = ComputeMotorForces(Fc, Gc, d, km);
    controls(j,:) = [Fc(3),Gc'];
    end
    PlotAircraftSim(timeF,aircraft_state_Feedback,controls,figures(:,i),col(2))

    % % Velocity Feedback
    % quadFeedback = @(t,v)QuadrotorEOMwithRateFeedback_CL_Velocity(t,v, g, m, I, nu, mu,lat,long);
    % [timeF,aircraft_state_Feedback_V] = ode45(quadFeedback,t,var);
    % 
    % controls = zeros(length(aircraft_state_Feedback_V),4);
    % for j = 1:length(aircraft_state_Feedback_V)
    % [Fc, Gc] = VelocityReferenceFeedback(t,aircraft_state_Feedback_V(j,:),lat,long);
    % %motor_forces = ComputeMotorForces(Fc, Gc, d, km);
    % controls(j,:) = [Fc(3),Gc'];
    % end
    % 
    % PlotAircraftSim(timeF,aircraft_state_Feedback_V,controls,figures(:,i),col(3))

end



%% 3.7 & 3.8


t = [0 2];
% Lateral
load("RSdata_11_16.mat")
times = rt_estim.time(:);
stateEstim = rt_estim.signals.values(:,:);
controls = zeros(length(times),4);
PlotAircraftSim(times,stateEstim,controls,figures(:,5),'c');
figure(30)
hold on;
[~,i] = min(abs(times-6));
[~,idx] = min(abs(times-8));
plot3(stateEstim(i,1),stateEstim(i,2),-stateEstim(i,3),'ko')
plot3(stateEstim(idx,1),stateEstim(idx,2),-stateEstim(idx,3),'bo')

type = 1;
var = [stateEstim(end,1);0;-rt_cmd.signals.values(4);  0;0;0;  0;0;0; 0;0;0]; % trim state vector
quadFeedback = @(t,var)QuadrotorEOMwithRateFeedback_CL_Velocity(t, var, g, m, I, nu, mu,lat,long,type);
[timeF,aircraft_state_Feedback_V] = ode45(quadFeedback,t,var);

    controls = zeros(length(aircraft_state_Feedback_V),4);
    for j = 1:length(aircraft_state_Feedback_V)
    [Fc, Gc] = VelocityReferenceFeedback(t,aircraft_state_Feedback_V(j,:),lat,long,type);
    %motor_forces = ComputeMotorForces(Fc, Gc, d, km);
    controls(j,:) = [Fc(3),Gc'];
    end

    PlotAircraftSim(timeF,aircraft_state_Feedback_V,controls,figures(:,5),col(3))

% Longitudinal

load("RSdata_09_36.mat")
times = rt_estim.time(:);
stateEstim = rt_estim.signals.values(:,:);
controls = zeros(length(times),4);
PlotAircraftSim(times,stateEstim,controls,figures(:,6),'c');

type = 2;
var = [0;0;-rt_cmd.signals.values(4);  0;0;0;  0;0;0; 0;0;0]; % trim state vector
quadFeedback = @(t,var)QuadrotorEOMwithRateFeedback_CL_Velocity(t, var, g, m, I, nu, mu,lat,long,type);
[timeF,aircraft_state_Feedback_V] = ode45(quadFeedback,t,var);

    controls = zeros(length(aircraft_state_Feedback_V),4);
    for j = 1:length(aircraft_state_Feedback_V)
    [Fc, Gc] = VelocityReferenceFeedback(t,aircraft_state_Feedback_V(j,:),lat,long,type);
    %motor_forces = ComputeMotorForces(Fc, Gc, d, km);
    controls(j,:) = [Fc(3),Gc'];
    end

    PlotAircraftSim(timeF,aircraft_state_Feedback_V,controls,figures(:,6),col(3))



