%ASEN 3801/3128 Lab 1
%Tyler Hoover, Alexander Keller, Rishab Pally
%Created 9/1/23

clc; close; 
%% Problem 1

% define initial conditions\
angular_velocity= 0.5;
xpi= 0.5;
y_position_i= 0.5;
z_position_i= 0.5;

initial_state_v= [xpi; y_position_i; z_position_i; angular_velocity];


 %% %%%%%%%%%%%%%%%%%%%%%%%%% Part 1A %%%%%%%%%%%%%%%%%%%%%%%%%
tspan_vals= [0 20];
[t, pos_val1]= ode45(@(t, initial_state_v) EquationsOfmotionFun1A(t, initial_state_v), tspan_vals, initial_state_v);

%plot figures
figure(1)
subplot(4,1,1)
plot(t,pos_val1(:,1))
title("Change in X vs Time")
xlabel("Time (s)")
ylabel("Z")
set(gca, 'FontName', 'Times New Roman')
grid minor 

subplot(4,1,2)
plot(t,pos_val1(:,2))
title("Change in Y vs Time")
xlabel("Time (s)")
ylabel("Y")
set(gca, 'FontName', 'Times New Roman')
grid minor

subplot(4,1,3)
plot(t,pos_val1(:,3))
title("Change in Z vs Time")
xlabel("Time (s)")
ylabel("Z")
set(gca, 'FontName', 'Times New Roman')
grid minor

subplot(4,1,4)
plot(t,pos_val1(:,4))
title('Change in W vs Time')
xlabel('Time (s)')
ylabel('W')
set(gca, 'FontName', 'Times New Roman')
grid minor
%% %%%%%%%%%%%%%%%%%%%%%%%%%Part 1B%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulating w a different eom for W and a shorter Time span
tspan_vals2= [0 0.2];
[t, pos_val2]= ode45(@(t, initial_state_v) EquationsOfmotionFun1B(t, initial_state_v), tspan_vals2, initial_state_v);

%plot figures
figure(2)
subplot(4,1,1)
plot(t,pos_val2(:,1))
title("Change in X vs Time")
xlabel("Time (s)")
ylabel("Z")
set(gca, 'FontName', 'Times New Roman')
grid minor 

subplot(4,1,2)
plot(t,pos_val2(:,2))
title("Change in Y vs Time")
xlabel("Time (s)")
ylabel("Y")
set(gca, 'FontName', 'Times New Roman')
grid minor

subplot(4,1,3)
plot(t,pos_val2(:,3))
title("Change in Z vs Time")
xlabel("Time (s)")
ylabel("Z")
set(gca, 'FontName', 'Times New Roman')
grid minor

subplot(4,1,4)
plot(t,pos_val2(:,4))
title("Change in W vs Time")
xlabel("Time (s)")
ylabel("W")
set(gca, 'FontName', 'Times New Roman')
grid minor

%% %%%%%%%%%%%%%%%%%%%%%%%%%Part 1C%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulating for 2 rather than .2 seconds
tspan_vals3= [0 2];
[t, pos_val3]= ode45(@(t, initial_state_v) EquationsOfmotionFun1B(t, initial_state_v), tspan_vals3, initial_state_v);

%plot figures
figure(3)
subplot(4,1,1)
plot(t,pos_val3(:,1))
title("Change in X vs Time")
xlabel("Time (s)")
ylabel("Z")
set(gca, 'FontName', 'Times New Roman')
grid minor 

subplot(4,1,2)
plot(t,pos_val3(:,2))
title("Change in Y vs Time")
xlabel("Time (s)")
ylabel("Y")
set(gca, 'FontName', 'Times New Roman')
grid minor

subplot(4,1,3)
plot(t,pos_val3(:,3))
title("Change in Z vs Time")
xlabel("Time (s)")
ylabel("Z")
set(gca, 'FontName', 'Times New Roman')
grid minor

subplot(4,1,4)
plot(t,pos_val3(:,4))
title("Change in W vs Time")
xlabel("Time (s)")
ylabel("W")
set(gca, 'FontName', 'Times New Roman')
grid minor



%% Problem 2 
%initial parameters

mass= 50; %g
diameter= 2.0 * 10.^(-2); %mass
Cd= 0.6;
air_rho= 1.14; %kg/m3 ( air density in boulder co)
A= (pi/4).*diameter.^2; %m^2
g= 9.81; %m/s2


%% Section B
vector_wind = [0;0;0]; %m/s
xpi= [0;0;0;0;20;-20]; % initial velocity component
tspan= [0 5];
[t, X]= ode45(@(t,X) objectEOM(t,X,air_rho,Cd,A,mass,g,vector_wind), tspan, xpi);

%Trajectory of the Ball
figure()
plot(X(:,2), -X(:,3))
title("Trajectory of the ball, No vector_wind (Y position vs -Z position)")
xlabel("Y position of the ball over Time")
ylabel("-Z position of ball over Time")

% figure()
% plot(X(:,2),X(:,3), LineWidth=2)
% xlabel('Distance Traveled North (mass)')
% % ylabel('Distance Traveled East')
% ylabel('Height (mass)')
% grid minor
% set(gca, 'FontName', 'Times New Roman')
% title('Section B')
%% Section C 
% need to characterize Landing Sensitivity to vector_wind

for i= 0:10:90


vector_wind= [i;0;0]; %m/s
xpi= [0;0;0;0;20;-20];
tspan= [0 5];
[t, X_C]= ode45(@(t,X_C) objectEOM(t,X_C,air_rho,Cd,A,mass,g,vector_wind), tspan, xpi);

varStruct((i/10)+1).px1= X_C(:,1);
varStruct((i/10)+1).py1= X_C(:,2);
varStruct((i/10)+1).pz1= X_C(:,3);
varStruct((i/10)+1).VeloX= X_C(:,4);
varStruct((i/10)+1).VeloY= X_C(:,5);
varStruct((i/10)+1).VeloZ= X_C(:,6);
end

figure()
title('Section C')
subplot(2,1,1)
title("Flight Paths")
xlabel("Distance Traveled North (mass)")
ylabel("Distance Traveled East (mass)")
zlabel("Height (mass)")
grid minor
set(gca, 'FontName', 'Times New Roman') 
hold on
for i=1:10
plot3(varStruct(i).px1, varStruct(i).py1,varStruct(i).pz1, LineWidth=2)
end
hold off
legend('vector_wind= 0 m/s', 'vector_wind= 10 m/s','vector_wind= 20 m/s','vector_wind= 30 m/s','vector_wind= 40 m/s',...
    'vector_wind= 50 m/s', 'vector_wind= 60 m/s','vector_wind= 70 m/s','vector_wind= 80 m/s','vector_wind= 90 m/s')


subplot(2,1,2)
xlabel("vector_wind Speed (m/s)")
ylabel("Deflection North")
grid minor
set(gca, 'FontName', 'Times New Roman')
hold on 
for i=1:10
plot(10*(i-1), varStruct(i).px1(end), 'x')
end

figure()
title("Flight Paths")
xlabel("Distance Traveled North (mass)")
ylabel("Distance Traveled East (mass)")
zlabel("Height (mass)")
grid minor
set(gca, 'FontName', 'Times New Roman') 
hold on
for i=1:10
plot3(varStruct(i).px1, varStruct(i).py1,-varStruct(i).pz1, LineWidth=2)

end



%% Section D

alt_range = 0:500:5000;

figure()
for i = 0:10:90
for j = 1:numel(alt_range)

[Txxxx,axxx,Pxxxx,rho] = atmosisa(alt_range(j));

wind = [i;0;0]; %m/s
X0 = [0;0;0;0;20;-20];
tspan = [0 5];
[t, X_C] = ode45(@(t,X_C) objectEOM(t,X_C,rho,Cd,A,m,g,wind), tspan, X0);

varStruct((i/10)+1).PosX = X_C(:,1);
varStruct((i/10)+1).PosY = X_C(:,2);
varStruct((i/10)+1).PosZ = X_C(:,3);
varStruct((i/10)+1).VeloX = X_C(:,4);
varStruct((i/10)+1).VeloY = X_C(:,5);
varStruct((i/10)+1).VeloZ = X_C(:,6);

values = X_C;
plot3(values(:,1),values(:,2),-1*values(:,3))
hold on

end
end

%% Section E
% comassputing diffence in distance due to varying massasses
vector_wind= [0;0;0];

%setting a vector of different massasses of the ball
mass_s= (10:50)./1000; %kg

%for loop to run through each of the massasses and calculate state variables
%for each
for j= 1:length(mass_s)

%Kinetic energy of the ball at the initial 30 gramasss and 20 m/s east and
%north velocities
calc1= sqrt(20^2 + 20^2)
calcsquare= (calc1).^2
calc2= (mass./1000);
KE= 0.5*calc2*calcsquare;

%Calculating Velocities in the y and z directions for each run through of
%the massass vector
vmcalc= (2*KE);
vmcalc1= mass_s(1,j);
vm_main= vmcalc./vmcalc1;
mass_velocityvec= sqrt(vm_main);
angle1= sind(45);
angle2= cosd(45);
yvity= mass_velocityvec.*angle1;
zvity= mass_velocityvec.*angle2;

%Putting the velocity for the current massass of ball in the initial state
%vector
xpi= [0;0;0;0;yvity;-zvity];

%Calling ODE45
[t, range_x]= ode45(@(t,range_x) objectEOM(t,range_x,air_rho,Cd,A,mass_s(1,j),g,vector_wind), tspan, xpi);

%defining each row of the output state vector
x_pstn= range_x(:,1);
yp1= range_x(:,2);
zp1= range_x(:,3);
xv= range_x(:,4);
yv= range_x(:,5);
zv= range_x(:,6);
py_inertial_vec(j)= yp1(end);
vx_inertial_vec(j)= xv(end);
vy_inertial_vec(j)= yv(end);
vz_inertial_vec(j)= zv(end);
end

%Calculating initial ball velocity for each massass of ball
ball_veloc= sqrt((vx_inertial_vec.^2) + (vy_inertial_vec.^2) + (vz_inertial_vec.^2));

%Plot of massass of ball vs distance traveled
figure()
plot(mass_s,py_inertial_vec)
xlabel("Mass of Ball (kg)")
ylabel("Distance Traveled (mass)")
title("Distance vs massass")

%Plot of massass of ball vs initial ball velocity
figure()
plot(mass_s,ball_veloc)
xlabel("Mass of Ball (kg)")
ylabel("Ball Velocity (m/s)")
title("Velocity vs mass")









%% Part 1 A Eq function
function [delta_var]= EquationsOfmotionFun1A(t, initial_state_v)
x= initial_state_v(1); 
y= initial_state_v(2);
z= initial_state_v(3);
w= initial_state_v(4); 

omega= -9 * w + y ;
range_xot= 4 * w * x * y - x^2;
y_dot= 2 * w - x - 2*z ;
z_dot= x * y - y^2 - 3 * z^3 ;

delta_var= [range_xot; y_dot; z_dot; omega]; 

end

%% Part 1 B Eqmasso function
function [delta_var]= EquationsOfmotionFun1B(t, initial_state_v)
x= initial_state_v(1); 
y= initial_state_v(2);
z= initial_state_v(3);
w= initial_state_v(4); 

calc_w= 9*w;
omega= calc_w+ y;
calc_range= 4*x*y*w;
calcx_square= x^2;
range_xot= calc_range- calcx_square;
calcw2= 2*w;
calcz2= 2*z;
y_dot= calcw2-x-calcz2;
calcxy= x*y;
calc_ys= y^2;
calc3z= 3* z^3;
z_dot= calcxy-calc_ys- calc3z;
delta_var= [range_xot; y_dot; z_dot; omega]; 
end


%% Problem 2 EOM
% part A
function xdot= objectEOM(t,X,air_rho,Cd,A,mass,g,vector_wind)
%{
Inputs: X - State Vector containing x,y,z positions and velocities
= [Px, Py, Pz, x_inertial_vec, y_inertial_vec, z_inertial_vec]';
 air_rho - Air density
 Cd - Coefficient of Drag
 A - Cross Sectional Area
 mass- mass of ball
 g - gravitational constant
 vector_wind - vector_wind vector in xyz= [Wx, Wy, Wz]'
Outputs:
 xdot - Vector containing changes in position and velocity
= [x_inertial_vec, y_inertial_vec, z_inertial_vec, Ax, Ay, Az]'
%}
    if X(3) > 0
     % once Z hits zero, all changes will stop
    xdot= [0; 0; 0; 0; 0; 0];
    else
    % Defining positions and velocities from state vector
    px1= X(1);
    py1= X(2);
    pz1= X(3);
    x_velocity= X(4);
    y_velocity= X(5);
    z_velocity= X(6);
    % calc rel velocity wrt vector_wind
    inertial_v_vec= [x_velocity; y_velocity; z_velocity];
    vr_vw= inertial_v_vec-vector_wind;
    x_inertial_vec= vr_vw(1);
    y_inertial_vec= vr_vw(2);
    z_inertial_vec= vr_vw(3);
    % vector_head unit vector to define drag vector
    vector_head= vr_vw./norm(vr_vw);
    % fdrag
    dragcalc= .5*air_rho*norm(vr_vw).^2
    dragforce= -vector_head*dragcalc*Cd*A;
    grav_force_vec= [0 ; 0; mass.*g];
    % separating vectors into individual 
    df1_vec= dragforce(1)+ grav_force_vec(1);
    df2_vec= dragforce(2)+ grav_force_vec(2);
    df3_vec= dragforce(3)+ grav_force_vec(3);
    % calc acceleration components for output vector
    ax= df1_vec./mass;
    ay= df2_vec./mass;
    az= df3_vec./mass;
    xdot= [x_velocity; y_velocity; z_velocity; ax; ay; az];
    end

end