clc;
clear;
close all;


function PlotAircraftSim(time, aircraft_state_array, control_input_array,fig, col)
Inertial_pos = aircraft_state_array(1:3,:);
Euler_angles = aircraft_state_array(4:6,:);
Inertial_velo = aircraft_state_array(7:9,:);
Angular_velo = aircraft_state_array(10:12,:);

%Plot inertial positions
figure(fig(1))
subplot(3,1,1)
plot(time,Inertial_pos(1,:),col); hold on;
xlabel('Time')
ylabel('x (m)')
subplot(3,1,2)
plot(time,Inertial_pos(2,:),col); hold on;
xlabel('Time')
ylabel('y (m)')
subplot(3,1,3)
plot(time,Inertial_pos(3,:),col); hold on;
xlabel('Time')
ylabel('z (m)')

%Plot Euler Angles
figure(fig(2))
subplot(3,1,1)
plot(time,Euler_angles(1,:),col); hold on;
xlabel('Time')
ylabel('Phi (rad)')
subplot(3,1,2)
plot(time,Euler_angles(2,:),col); hold on;
xlabel('Time')
ylabel('Theta (rad)')
subplot(3,1,3)
plot(time,Euler_angles(3,:,col)); hold on;
xlabel('Time')
ylabel('Psi (rad)')

%Plot inertial velocity
figure(fig(3))
subplot(3,1,1)
plot(time,Inertial_velo(1,:),col); hold on;
xlabel('Time')
ylabel('u (m/s)')
subplot(3,1,2)
plot(time,Inertial_velo(2,:),col); hold on;
xlabel('Time')
ylabel('v (m/s)')
subplot(3,1,3)
plot(time,Inertial_velo(3,:),col); hold on;
xlabel('Time')
ylabel('w (m/s)')

%Plot angular velo
figure(fig(4))
subplot(3,1,1)
plot(time,Angular_velo(1,:),col); hold on;
xlabel('Time')
ylabel('p (rad/s)')
subplot(3,1,2)
plot(time,Angular_velo(2,:),col); hold on;
xlabel('Time')
ylabel('q (rad/s)')
subplot(3,1,3)
plot(time,Angular_velo(3,:),col); hold on;
xlabel('Time')
ylabel('r (rad/s)')

%Plot control variables
figure(fig(5))
subplot(4,1,1)
plot(time,control_input_array(1,:),col); hold on;
xlabel('Time')
ylabel('Zc (N)')
subplot(4,1,2)
plot(time,control_input_array(2,:),col); hold on;
xlabel('Time')
ylabel('Lc (N)')
subplot(4,1,3)
plot(time,control_input_array(3,:),col); hold on;
xlabel('Time')
ylabel('Mc (N)')
subplot(4,1,4)
plot(time,control_input_array(4,:),col); hold on;
xlabel('Time')
ylabel('Nc (N)')

%3D Plot of Aircraft Position
figure(fig(6))
plot3(Inertial_pos(1,:), Inertial_pos(2,:), -Inertial_pos(3,:), col); hold on;%Negative z to make positive up
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
%Begin and end circles
plot3(Inertial_pos(1,1), Inertial_pos(2,1), -Inertial_pos(3,1),'g')
plot3(Inertial_pos(1,end), Inertial_pos(2,end), -Inertial_pos(3,end),'r')
end