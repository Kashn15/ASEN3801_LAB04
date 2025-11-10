clc;
clear;
close all;


%% 3.1-3.4
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

    %Run ODE45 for linearized
    odefun = @(t,var) LinearizedEOM_Control(t,var);
    [t1,X1] = ode45(odefun,[0,t],var);

    %Run ODE45 for nonlinearized
    odefun = @(t,var) NonlinearizedEOM_Control(t,var);
    [t2,X2] = ode45(odefun,[0,t],var);

    %Find control values outside of ode45
    Zc1 = zeros(length(X1),1);Lc1 = zeros(length(X1),1);Mc1 = zeros(length(X1),1);Nc1 = zeros(length(X1),1);
    for k = 1:length(X1)
        [Fc_k, Gc_k] = InnerLoopFeedback(X1(k,:));
        Zc1(k) = Fc_k(3); 
        Lc1(k) = Gc_k(1); 
        Mc1(k) = Gc_k(2); 
        Nc1(k) = Gc_k(3);
    end
    U1 =[Zc1, Lc1, Mc1, Nc1];     

%Nonlinear Control Forces
    Zc2 = zeros(length(X2),1);Lc2 = zeros(length(X2),1);Mc2 = zeros(length(X2),1);Nc2 = zeros(length(X2),1);
    for k = 1:length(X2)
    % Extract control values for the final part
        [Fc_k, Gc_k] = InnerLoopFeedback(X2(k,:));
        Zc2(k) = Fc_k(3); 
        Lc2(k) = Gc_k(1); 
        Mc2(k) = Gc_k(2); 
        Nc2(k) = Gc_k(3);
    end
    U2 =[Zc2, Lc2, Mc2, Nc2];     

    %Plots
    PlotAircraftSim(t1,X1,U1,fig(:,i),'b')
    PlotAircraftSim(t2,X2,U2,fig(:,i),'g')
end

%% 3.5-3.7
var = [0;0;0;0;0;0;0;0;0;0;0;0];    

%Linear
odefun = @(t,var) LinearizedEOM_Control_Finalpart(t,var);
[t1,X1] = ode45(odefun,[0,t],var);

%Nonlinear
odefun = @(t,var) NonlinearizedEOM_Control_Finalpart(t,var);
[t2,X2] = ode45(odefun,[0,t],var);

[Fc,Gc] = VelocityReferenceFunction(t,X1);

%Linearized Control Forces
for i = 1:length(X1)
    % Extract control values for the final part
    [Fc_i, Gc_i] = VelocityReferenceFunction(t, X1(i,:));
    Zc3(i) = Fc_i(3); 
    Lc3(i) = Gc_i(1); 
    Mc3(i) = Gc_i(2); 
    Nc3(i) = Gc_i(3);
end
U3 =[Zc3; Lc3; Mc3; Nc3];     
U3 = U3';

%Nonlinear Control Forces
for i = 1:length(X2)
    % Extract control values for the final part
    [Fc_i, Gc_i] = VelocityReferenceFunction(t, X2(i,:));
    Zc4(i) = Fc_i(3); 
    Lc4(i) = Gc_i(1); 
    Mc4(i) = Gc_i(2); 
    Nc4(i) = Gc_i(3);
end
U4 =[Zc4; Lc4; Mc4; Nc4];     
U4 = U4';


%Plot them
PlotAircraftSim(t1,X1,U3,[25,26,27,28,29,30],'b')
PlotAircraftSim(t2,X2,U4,[25,26,27,28,29,30],'g')
