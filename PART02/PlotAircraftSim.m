function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
% Normalize time
time = time(:).';     % 1xN
X = aircraft_state_array;

% Slices of state
Inertial_pos  = X(1:3,:);     % x y z
Euler_angles  = X(4:6,:);     % phi theta psi (rad)
Inertial_velo = X(7:9,:);     % u v w
Angular_velo  = X(10:12,:);   % p q r

% 1) Inertial positions
figure(fig(1))
subplot(3,1,1); plot(time, Inertial_pos(1,:), col); hold on; grid on; sgtitle('Inertial Position')
xlabel('Time [s]'); ylabel('x (m)')
subplot(3,1,2); plot(time, Inertial_pos(2,:), col); hold on; grid on
xlabel('Time [s]'); ylabel('y (m)')
subplot(3,1,3); plot(time, Inertial_pos(3,:), col); hold on; grid on
xlabel('Time [s]'); ylabel('z (m)')

% 2) Euler angles
figure(fig(2))
subplot(3,1,1); plot(time, Euler_angles(1,:), col); hold on; grid on; sgtitle('Euler Angles');
xlabel('Time [s]'); ylabel('\phi (rad)')
subplot(3,1,2); plot(time, Euler_angles(2,:), col); hold on; grid on
xlabel('Time [s]'); ylabel('\theta (rad)')
subplot(3,1,3); plot(time, Euler_angles(3,:), col); hold on; grid on   % <- fixed
xlabel('Time [s]'); ylabel('\psi (rad)')

% 3) Body 
figure(fig(3))
subplot(3,1,1); plot(time, Inertial_velo(1,:), col); hold on; grid on; sgtitle('Body Frame Velocity');
xlabel('Time [s]'); ylabel('u (m/s)')
subplot(3,1,2); plot(time, Inertial_velo(2,:), col); hold on; grid on
xlabel('Time [s]'); ylabel('v (m/s)')
subplot(3,1,3); plot(time, Inertial_velo(3,:), col); hold on; grid on
xlabel('Time [s]'); ylabel('w (m/s)')

% 4) Angular rates
figure(fig(4))
subplot(3,1,1); plot(time, Angular_velo(1,:), col); hold on; grid on; sgtitle('Angular Rates');
xlabel('Time [s]'); ylabel('p (rad/s)')
subplot(3,1,2); plot(time, Angular_velo(2,:), col); hold on; grid on
xlabel('Time [s]'); ylabel('q (rad/s)')
subplot(3,1,3); plot(time, Angular_velo(3,:), col); hold on; grid on
xlabel('Time [s]'); ylabel('r (rad/s)')

% 5) Control inputs
figure(fig(5))
if isempty(control_input_array)
    clf(fig(5)); % clear the figure so plots dont stack
    subplot(4,1,1); sgtitle('Control Inputs (none provided)'); axis off
    subplot(4,1,2); axis off
    subplot(4,1,3); axis off
    subplot(4,1,4); axis off
else
    U = control_input_array;
    subplot(4,1,1); plot(time, U(1,:), col); hold on; grid on; sgtitle('Control Inputs');
    xlabel('Time [s]'); ylabel('Z_c (N)')
    subplot(4,1,2); plot(time, U(2,:), col); hold on; grid on
    xlabel('Time [s]'); ylabel('L_c (N·m)')
    subplot(4,1,3); plot(time, U(3,:), col); hold on; grid on
    xlabel('Time [s]'); ylabel('M_c (N·m)')
    subplot(4,1,4); plot(time, U(4,:), col); hold on; grid on
    xlabel('Time [s]'); ylabel('N_c (N·m)')
end

% 6) 3D Plot of Aircraft Position
% 3D Plot of Aircraft Position

    figure(fig(6))
    ax = gca; hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');

    x = Inertial_pos(1,:);
    y = Inertial_pos(2,:);
    z = -Inertial_pos(3,:);   % positive up

    % Display tolerances
    minCube  = 0.1;     % [m] minimum cube edge so tiny drift doesn't fill the screen
    deadband = 1e-4;     % [m] values smaller than this are treated as zero for plotting


    % Visually "snap" tiny motion to zero
    if (max(x)-min(x)) < deadband, x = x*0 + mean(x); end
    if (max(y)-min(y)) < deadband, y = y*0 + mean(y); end
    if (max(z)-min(z)) < deadband, z = z*0 + mean(z); end

    % Trajectory
    hLine = plot3(ax, x, y, z, col, 'LineWidth', 1.6, 'Tag','trajLine');

    % Start/end markers match line color, hidden from legend
    c = get(hLine,'Color');
    plot3(ax, x(1),   y(1),   z(1),  'o', 'MarkerEdgeColor', c, 'MarkerFaceColor', c, 'MarkerSize', 6, 'HandleVisibility','off', 'Tag','trajMarker');
    plot3(ax, x(end), y(end), z(end), 's', 'MarkerEdgeColor', c, 'MarkerFaceColor', c, 'MarkerSize', 6, 'HandleVisibility','off', 'Tag','trajMarker');

    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
    title(ax,'3D Trajectory (start circle, end square)')

    % Combined ranges across everything already plotted
    if strcmp(get(ax,'XLimMode'),'auto'), XL = [min(x) max(x)]; else, XL = xlim(ax); end
    if strcmp(get(ax,'YLimMode'),'auto'), YL = [min(y) max(y)]; else, YL = ylim(ax); end
    if strcmp(get(ax,'ZLimMode'),'auto'), ZL = [min(z) max(z)]; else, ZL = zlim(ax); end
    XL = [min([XL(1) min(x)]), max([XL(2) max(x)])];
    YL = [min([YL(1) min(y)]), max([YL(2) max(y)])];
    ZL = [min([ZL(1) min(z)]), max([ZL(2) max(z)])];

    % Avoid zero-span weirdness
    if XL(1)==XL(2), XL = XL + [-eps eps]; end
    if YL(1)==YL(2), YL = YL + [-eps eps]; end
    if ZL(1)==ZL(2), ZL = ZL + [-eps eps]; end

    % Make a cube with a minimum edge length
    cx = mean(XL); cy = mean(YL); cz = mean(ZL);
    h  = 0.5 * max([XL(2)-XL(1), YL(2)-YL(1), ZL(2)-ZL(1), minCube]);

    xlim([cx-h, cx+h]); ylim([cy-h, cy+h]); zlim([cz-h, cz+h]);
    daspect([1 1 1]);  
    view(45,25);
    
end