%% Task 2
clear; clc; close all;

% Varibles
m = 0.068; g = 9.81;
I = diag([5.8e-5, 7.2e-5, 1.0e-4]);
d = 0.060; km = 0.0024;
nu = 0; mu = 0;                 
A = [     -1          -1          -1           -1;
      -d/sqrt(2)  -d/sqrt(2)   d/sqrt(2)   d/sqrt(2);
       d/sqrt(2)  -d/sqrt(2)  -d/sqrt(2)   d/sqrt(2);
          km          -km         km           -km ];

% Hover forces
f0 = (m*g/4)*ones(4,1);
U_hover = A * f0;

% Initial condition: small disturbance for 2.1/2.2
x0 = zeros(12,1);
% x0(4) = deg2rad(5);          % 5 deg roll
% x0(5) = deg2rad(5);        % 5 deg pitch
% x0(6) = deg2rad(5);        % 5 deg yaw
% x0(10) = .1;               % .1 rad/s p
% x0(11) = .1;               % .1 rad/s q
 x0(12) = .1;               % .1 rad/s r

tspan = [0 10];

% 2.1 Nonlinear, uncontrolled
ode_fun_non = @(t,x) QuadrotorEOM(t,x,g,m,I,d,km,nu,mu,f0);
[t1, X1] = ode45(ode_fun_non, tspan, x0);

% 2.2 Linearized vs same disturbance 
ode_fun_lin = @(t,xd) QuadrotorEOM_Linearized(t, xd, g, m, I, [0;0;0], [0;0;0]);
[t2, X2] = ode45(ode_fun_lin, tspan, x0);

% 2.5 With rate feedback
ode_rf = @(t,x) QuadrotorEOMwithRateFeedback(t,x,g,m,I,nu,mu);
[t3, X3] = ode45(ode_rf, tspan, x0);

% Build controls for plotting
U1 = U_hover*ones(1,length(t1));   % fixed hover control
U2 = zeros(4,length(t2));          % linearized
U3 = zeros(4,length(t3));          

% Plot each case
figs = (1:6).';
PlotAircraftSim(t1.', X1.', U1, figs, 'b-');   % nonlinear open-loop
PlotAircraftSim(t2.', X2.', U2, figs, 'r--');  % linear model
PlotAircraftSim(t3.', X3.', U3, figs, 'k-.');  % rate-damped

% 2.1 Legend
%legend('Nonlinearized','Start','End','Linearized');

% 2.5 Legend
figure(figs(6)); ax = gca;
hLines = findobj(ax,'Type','line','Tag','trajLine');
if ~isempty(hLines)
    hLines = flipud(hLines);                           % match call order
    names  = {'Nonlinear OL','Linearized','Rate feedback'};
    n = min(numel(hLines), numel(names));
    for i = 1:n
        set(hLines(i),'DisplayName',names{i});
    end
    lgd = legend(ax, hLines(1:n), names(1:n), ...
                 'Location','northeast', ...          % INSIDE the axes
                 'Box','off','Interpreter','none');
    set(lgd,'AutoUpdate','off');                      
end
