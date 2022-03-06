%% System parameters - feel free to change these to be more realistic
close all; clear all;
params.m = 1000; % Mass of air taxi - in kg
params.delta_t = 0.001; % Delta in time - in seconds
rng(1); % For repeatability
%% 1. Dynamics and Simulation, True Path
x0 = [0;0;0;0]; % initially at rest at origin, state x is [x; y; xdot; ydot]
time_to_run = 10; % in seconds
t = (0:params.delta_t:time_to_run)';
% Force input
fx = 500*cos(.4*t);
fy = 1000*sin(.05*t);
u = [fx, fy];
% Simulate Dynamics and Log
Q = zeros(2,2);
R = zeros(2,2);
[true_path_meas_log, true_path_state_log] = simSystem(x0, u, time_to_run, params, Q,R);

%% 1. Plot True Path with Trees and Satellites
close all;
EVGRC = [8, 2];
EVGRD = [3, 6];
satellite1_location = [0, 0];
satellite2_location = [10, 0];
satellite3_location = [0, 10];
satellite4_location = [10, 10];

figure;
hold on;
x_positions = true_path_meas_log(:,1);
y_positions = true_path_meas_log(:,2);
plot(x_positions, y_positions, '--');
plot(EVGRC(1), EVGRC(2),  's', 'MarkerFaceColor', 'black', 'MarkerSize', 100);
plot(EVGRD(1), EVGRD(2),  's', 'MarkerFaceColor', 'magenta', 'MarkerSize', 100);
plot(satellite1_location(1), satellite1_location(2),  'bo', 'MarkerFaceColor', 'red', 'MarkerSize', 10);
plot(satellite2_location(1), satellite2_location(2),  'bo', 'MarkerFaceColor', 'blue', 'MarkerSize', 10);
plot(satellite3_location(1), satellite3_location(2),  'bo', 'MarkerFaceColor', 'green', 'MarkerSize', 10);
plot(satellite4_location(1), satellite4_location(2),  'bo', 'MarkerFaceColor', 'yellow', 'MarkerSize', 10);

xlabel('X Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Y Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
title('True Vehicle Trajectory', 'Interpreter', 'latex', 'Fontsize', 16);
legend('Vehicle Path','EVGR C', 'EVGR D', 'Satellite 1', 'Satellite 2', 'Satellite 3', 'Satellite 4','Interpreter', 'latex','Fontsize', 12, 'Location', 'northwest');
grid on;
hold off;
%% 2. Dynamics and Simulation, Fixed Variance, Gaussian White Noise
x0 = [0;0;0;0]; % initially at rest at origin, state x is [x; y; xdot; ydot]
time_to_run = 10; % in seconds
t = (0:params.delta_t:time_to_run)';
% Force input
fx = 500*cos(.4*t);
fy = 1000*sin(.05*t);
u = [fx, fy];
% Simulate Dynamics and Log
Q = 1*10^-5*eye(2,2);
R = 5*10^-5*eye(2,2);
all_sat_positions = [satellite1_location; satellite2_location; satellite3_location; satellite4_location];
[fixed_var_path_meas_log, fixed_var_path_state_log] = simSystemandMeas(x0, u, time_to_run, params, Q,R, all_sat_positions);
%% 2. Plot True Path with Trees vs. Path with Fixed Variance, Gaussian White Noise
close all;
tree1_location = [4.5, 1];
tree2_location = [5.9, 6];
figure;
hold on;
x_positions = true_path_meas_log(:,1);
y_positions = true_path_meas_log(:,2);
% To become pseudorange plot that is in seperate figure
% x_positions_fixed_var = fixed_var_path_meas_log(:,1);
% y_positions_fixed_var = fixed_var_path_meas_log(:,2);

% plot(x_positions, y_positions, 'b--');
% plot(x_positions_fixed_var, y_positions_fixed_var, 'r--')
plot(tree1_location(1), tree1_location(2),  'g^', 'MarkerFaceColor', 'green', 'MarkerSize', 14);
plot(tree2_location(1), tree2_location(2),  'g^', 'MarkerFaceColor', 'green', 'MarkerSize', 14);
plot(satellite1_location(1), satellite1_location(2),  'bs', 'MarkerFaceColor', 'blue', 'MarkerSize', 14);
plot(satellite2_location(1), satellite2_location(2),  'bs', 'MarkerFaceColor', 'blue', 'MarkerSize', 14);
plot(satellite3_location(1), satellite3_location(2),  'bs', 'MarkerFaceColor', 'blue', 'MarkerSize', 14);
plot(satellite4_location(1), satellite4_location(2),  'bs', 'MarkerFaceColor', 'blue', 'MarkerSize', 14);

xlabel('X Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Y Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
title('True Vehicle Trajectory and Vehicle Trajectory with Fixed Variance Noise', 'Interpreter', 'latex', 'Fontsize', 16);
legend('Vehicle Path','Tree 1', 'Tree 2','Interpreter', 'latex','Fontsize', 12, 'Location', 'northwest');
grid on;
hold off;
%% 3. Dynamics and Simulation, Time-Varying Variance, Gaussian White Noise



