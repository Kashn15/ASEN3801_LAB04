clc;
clear;
close all;

clear; clc; close all;


k1 = 9.86e-04;  %Gains from 3.1
k2 = 0.0017;  
k3 = linspace(0, 0.0001, 1000); %Range of values for k3
Ix = 5.8e-05;   
Iy = 7.2e-05;
g  = 9.82; 

for i = 1:length(k3)
    %Closed loop lateral control matrix 
    A_cl_lat = [ 0,         1,          0;
                -k2/Ix, -k1/Ix, -k3(i)/Ix;
                 g,         0,          0];       
    %Close loop longitudinal control matrix
    A_cl_long = [ 0,         1,          0;
                -k2/Iy, -k1/Iy, k3(i)/Iy;
                 -g,         0,          0];         
    eigs_lat(:, i) = eig(A_cl_lat); %Save Eigenvalues
    %Condition for eigenvalues to be real and below -1/1.25
    index_lat(i) = all(abs(imag(eig(A_cl_lat))) < 1e-8) && all(real(eig(A_cl_lat)) <= -0.8);

    eigs_long(:, i) = eig(A_cl_long); % Save Eigenvalues for longitudinal control
    index_long(i) = all(abs(imag(eig(A_cl_long))) < 1e-8) && all(real(eig(A_cl_long)) <= -0.8);
end

%Locus Plot
figure(1);
plot(real(eigs_lat), imag(eigs_lat), 'b.', 'MarkerSize', 6);
grid on;

%Find valid values for k3
index2_lat = find(index_lat ==1);
k3_valid_lat = k3(index2_lat);

index2_long = find(index_long ==1);
k3_valid_long = k3(index2_long); 

