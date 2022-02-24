%% System parameters - feel free to change these to be more realistic
close all; clear all;
params.m = 1000; % Mass of air taxi - in kg
params.Jcm_zz = 200; % Inertia of air taxi about z-axis going through center of mass - in kg m^2 
params.L = 10; % Distance from left thruster to right thruster - in meters
params.delta_t = 0.001; % Delta in time - in seconds
%% Dynamics and Simulation, No Measurement Errors
x0 = [0;0;0;0;0;0]; % initially at rest at origin, state x is [x; y; theta; xdot; ydot; thetadot]
time_to_run = 15; % in seconds
input_length = time_to_run / params.delta_t;
% Force input
left_thrust = 20*(ones(input_length,1));
right_thrust = -20*(ones(input_length,1));
u = [left_thrust, right_thrust];
% Simulate Dynamics and Log
[meas_log, state_log] = simSystem(x0, u, time_to_run, params);

%% Plot
t = linspace(0,time_to_run,length(state_log));
figure;
plot(t, sin(state_log(:,3)));
%% EKF Modifications to Track with Imperfect Measurements



