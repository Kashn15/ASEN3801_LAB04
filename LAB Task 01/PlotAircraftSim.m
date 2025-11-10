%Plot function
function PlotAircraftSim(time, aircraft_state_array, control_input_array,fig, col)
Inertial_pos = aircraft_state_array(:,1:3);
Euler_angles = aircraft_state_array(:,4:6);
Inertial_velo = aircraft_state_array(:,7:9);
Angular_velo = aircraft_state_array(:,10:12);

%Plot inertial positions
figure(fig(1))
subplot(3,1,1)
plot(time,Inertial_pos(:,1),col); 
hold on;
title('X Position');
xlabel('Time [s]')
ylabel('x [m]')
ytickformat('%.2f')
grid on
subplot(3,1,2)
plot(time,Inertial_pos(:,2),col); 
hold on;
title('Y Position');
xlabel('Time [s]')
ylabel('y [m]')
ytickformat('%.2f')
grid on
subplot(3,1,3)
plot(time,Inertial_pos(:,3),col);
hold on;
title('Z Position');
xlabel('Time [s]')
ylabel('z [m]')
ytickformat('%.2f')
grid on
sgtitle('Inertial Postions', 'FontSize', 13);

%Plot Euler Angles
figure(fig(2))
subplot(3,1,1)
plot(time,Euler_angles(:,1),col); 
hold on;
xlabel('Time [s]')
ylabel('\phi [rad]')
ytickformat('%.2f')
grid on
title('Roll');
subplot(3,1,2)
plot(time,Euler_angles(:,2),col); 
hold on;
xlabel('Time [s]')
ylabel('\theta [rad]')
ytickformat('%.2f')
grid on
title('Pitch')
subplot(3,1,3)
plot(time,Euler_angles(:,3),col); 
hold on;
xlabel('Time [s]')
ylabel('\psi [rad]')
ytickformat('%.2f')
grid on
title('Yaw');
sgtitle('Euler Angles', 'FontSize', 13);

%Plot Body-Frame velocity
figure(fig(3))
subplot(3,1,1)
plot(time,Inertial_velo(:,1),col); 
hold on;
xlabel('Time [s]')
ylabel('u [m/s]')
ytickformat('%.2f')
grid on
title('X-directional Velocity');
subplot(3,1,2)
plot(time,Inertial_velo(:,2),col); 
hold on;
xlabel('Time [s]')
ylabel('v [m/s]')
ytickformat('%.2f')
grid on
title('Y-directional Velocity');
subplot(3,1,3)
plot(time,Inertial_velo(:,3),col); 
hold on;
xlabel('Time [s]')
ylabel('w [m/s]')
ytickformat('%.2f')
grid on
title('Z-directional Velocity');
sgtitle('Body-Frame Velocities', 'FontSize', 13);

%Plot angular velo
figure(fig(4))
subplot(3,1,1)
plot(time,Angular_velo(:,1),col); 
hold on;
xlabel('Time [s]')
ylabel('p [rad/s]')
ytickformat('%.2f')
title('Roll Rate');
grid on
subplot(3,1,2)
plot(time,Angular_velo(:,2),col); 
hold on;
xlabel('Time [s]')
ylabel('q [rad/s]')
ytickformat('%.2f')
grid on
title('Pitch Rate');
subplot(3,1,3)
plot(time,Angular_velo(:,3),col); 
hold on;
xlabel('Time [s]')
ylabel('r [rad/s]')
ytickformat('%.2f')
grid on
title('Yaw Rate');
sgtitle('Angular Velocities', 'FontSize', 13);

%Plot control variables
figure(fig(5))
subplot(4,1,1)
plot(time,control_input_array(:,1),col); 
hold on;
xlabel('Time [s]')
ylabel('Z_c [N]')
ytickformat('%.2f')
grid on
subplot(4,1,2)
plot(time,control_input_array(:,2),col); 
hold on;
xlabel('Time [s]')
ylabel('L_c [N*m]')
ytickformat('%.2f')
grid on
subplot(4,1,3)
plot(time,control_input_array(:,3),col); 
hold on;
xlabel('Time [s]')
ylabel('M_c [N*m]')
ytickformat('%.2f')
grid on
subplot(4,1,4)
plot(time,control_input_array(:,4),col); 
hold on;
xlabel('Time [s]')
ylabel('N_c [N*m]')
ytickformat('%.2f')
grid on
sgtitle('Control Variables', 'FontSize', 13);

%3D Plot of Aircraft Position
figure(fig(6))
plot3(Inertial_pos(:,1), Inertial_pos(:,2), -Inertial_pos(:,3), col); %Negative z to make positive up
grid on;
ytickformat('%.2f')
xtickformat('%.2f')
ztickformat('%.2f')
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
%Begin and end circles
hold on;
plot3(Inertial_pos(1,1), Inertial_pos(1,2), -Inertial_pos(1,3),'go', 'MarkerFaceColor','g')
plot3(Inertial_pos(end,1), Inertial_pos(end,2), -Inertial_pos(end, 3),'ro', 'MarkerFaceColor','r')
title('3D Plot of Position', 'FontSize', 13);
hold off;
end