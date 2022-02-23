%% System parameters - feel free to change these to be more realistic
m = 1000; % Mass of air taxi - in kg
Jcm_zz = 20; % Inertia of air taxi about z-axis going through center of mass - in kg m^2 
L = 10; % Distance from left thruster to right thruster - in meters
delta_t = 0.001; % Delta in time - in seconds
%% Dynamics and Simulation, No Measurement Errors
x = [0;0;0;0;0;0]; % initially at rest at origin
x_dot = [0;0;0;0;0;0]; % initially at rest
% x_next = 
%% EKF Modifications to Track with Imperfect Measurements



