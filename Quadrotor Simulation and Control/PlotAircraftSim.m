function PlotAircraftSim(time, aircraft_state_array, control_input_array,fig,col)
%PLOTAIRCRAFTSIM Summary of this function goes here
%   Detailed explanation goes here

%% Inertial Position
figure(fig(1));
names = ['x','y','z'];
for i = 1:3
subplot(3,1,i);
hold on;
plot(time,aircraft_state_array(:,i),col);
title(names(i))
end
sgtitle('Inertial Position [x,y,z]')
%% Euler angles
figure(fig(2));
names = ["\phi","\theta","psi"];
for i = 1:3
subplot(3,1,i);
hold on;
plot(time,aircraft_state_array(:,3+i),col);
xlabel('Time [s]')
ylabel('rad')
title(names(i))
end
sgtitle('Euler Angles [\phi,\theta,\psi]')
%% Inertial Velocity (body frame)
figure(fig(3));
names = ['u','v','w'];
for i = 1:3
subplot(3,1,i);
hold on;
plot(time,aircraft_state_array(:,6+i),col);
xlabel('Time [s]')
ylabel('m/s')
title(names(i))
end
sgtitle('Inertial Velocity (Body Frame) [u, v, w]')
%% Angular Velocity
figure(fig(4));
names = ['p','q','r'];
for i = 1:3
subplot(3,1,i);
hold on;
plot(time,aircraft_state_array(:,9+i),col);
xlabel('Time [s]')
ylabel('rad/s')
title(names(i))
end
sgtitle('Angular Velocity [p,q,r]')
%% Control Input Variables
figure(fig(5));
name = ["Zc","Lc","Mc","Nc"];
for i = 1:4
subplot(2,2,i)
hold on;
plot(time,control_input_array(:,i),col)
xlabel('Time [s]')
ylabel('N')
title(name(i))
end
sgtitle('Control Variables [Zc,Lc,Mc,Nc]')
%% 3D Path
figure(fig(6));
hold on;
plot3(aircraft_state_array(1,1),aircraft_state_array(1,2),-aircraft_state_array(1,3),'go');
plot3(aircraft_state_array(:,1),aircraft_state_array(:,2),-aircraft_state_array(:,3),col);
plot3(aircraft_state_array(end,1),aircraft_state_array(end,2),-aircraft_state_array(end,3),'ro');
legend('Start','','End')
% range = max([aircraft_state_array(:,1); aircraft_state_array(:,2);-aircraft_state_array(:,3)])+1;
% xlim([0,range])
% ylim([0,range])
% %zlim([0,-min(aircraft_state_array(:,3))])
% %axis equal;
view([-37.5 30]);


end

