clc;
clear;
close all;

%Constants
m=0.068; % kg
g=9.81; % m/s^2
I=diag([5.8e-5 7.2e-5 1.0e-4]); % kg*m^2
km=0.0024; % N*m/(N)
vu = 1e-3; % N/m/s)^2
mu = 2e-6; % N*m/(rad/s)^2
d = 0.060; % m
t = 10;

%Cases
Case(:,1) = [0;0;0;5*pi/180;0;0;0;0;0;0;0;0];
Case(:,2) = [0;0;0;0;5*pi/180;0;0;0;0;0;0;0];
Case(:,3) = [0;0;0;0;0;0;0.1;0;0;0;0;0];
Case(:,4) = [0;0;0;0;0;0;0;0.1;0;0;0;0];

%Figure Labels
fig(:,1) = [1,2,3,4,5,6];
fig(:,2) = [7,8,9,10,11,12];
fig(:,3) = [13,14,15,16,17,18];
fig(:,4) = [19,20,21,22,23,24];

for i = 1:4
    var = Case(:,i);

    %Run ODE45 
    odefun = @(t,var) LinearizedEOM_Control(t,var);
    [t1,X1] = ode45(odefun,[0,t],var);

    %Find control values outside of ode45
    [Fc,Gc] = InnerLoopFeedback(var);

    %Make control variables same length as other graphs
    U = repmat([Fc(3), Gc(1), Gc(2), Gc(3)], length(t1), 1);

    %Plots
    PlotAircraftSim(t1,X1,U,fig(:,i),'b')
end