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
Q = 1*10^-2*eye(2,2); % process covariance matrix
R = 3; % sensor variance
fixed_variance = 1; % to indicated variance should be fixed
all_sat_positions = [satellite1_location; satellite2_location; satellite3_location; satellite4_location];
[fixed_var_path_meas_log, fixed_var_path_state_log] = simSystemandMeas(x0, u, time_to_run, params, Q,R, all_sat_positions, fixed_variance);
%% 2. Plot Pseudorange Measurements for each Satellite with fixed variance
close all;
sat1_meas = fixed_var_path_meas_log(:,1);
sat2_meas = fixed_var_path_meas_log(:,2);
sat3_meas = fixed_var_path_meas_log(:,3);
sat4_meas = fixed_var_path_meas_log(:,4);

x_positions = fixed_var_path_state_log(:,1);
y_positions = fixed_var_path_state_log(:,2);

% Satellite 1 Pseudorange Meas Plot
figure;
plot3(x_positions, y_positions, sat1_meas);
grid on;
xlabel('X Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Y Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
zlabel('Z Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
title('Satellite 1 Pseudorange Measurements Over Trajectory', 'Interpreter', 'latex', 'Fontsize', 16);

figure;
plot3(x_positions, y_positions, sat2_meas);
grid on;
xlabel('X Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Y Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
zlabel('Z Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
title('Satellite 2 Pseudorange Measurements Over Trajectory', 'Interpreter', 'latex', 'Fontsize', 16);

figure;
plot3(x_positions, y_positions, sat3_meas);
grid on;
xlabel('X Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Y Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
zlabel('Z Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
title('Satellite 3 Pseudorange Measurements Over Trajectory', 'Interpreter', 'latex', 'Fontsize', 16);

figure;
plot3(x_positions, y_positions, sat4_meas);
grid on;
xlabel('X Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Y Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
zlabel('Z Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
title('Satellite 4 Pseudorange Measurements Over Trajectory', 'Interpreter', 'latex', 'Fontsize', 16);
%% 3. Dynamics and Simulation, Time-Varying Variance, Gaussian White Noise
x0 = [0;0;0;0]; % initially at rest at origin, state x is [x; y; xdot; ydot]
time_to_run = 10; % in seconds
t = (0:params.delta_t:time_to_run)';
% Force input
fx = 500*cos(.4*t);
fy = 1000*sin(.05*t);
u = [fx, fy];
% Simulate Dynamics and Log
Q = 1*10^-2*eye(2,2); % process covariance matrix
R = 3; % sensor variance
fixed_variance = 0; % to indicated variance should not be fixed
all_sat_positions = [satellite1_location; satellite2_location; satellite3_location; satellite4_location];
[fixed_var_path_meas_log, fixed_var_path_state_log] = simSystemandMeas(x0, u, time_to_run, params, Q,R, all_sat_positions, fixed_variance);

%% 4 Plot Pseudorange Measurements for each Satellite with time varying variance
close all;
sat1_meas = fixed_var_path_meas_log(:,1);
sat2_meas = fixed_var_path_meas_log(:,2);
sat3_meas = fixed_var_path_meas_log(:,3);
sat4_meas = fixed_var_path_meas_log(:,4);

x_positions = fixed_var_path_state_log(:,1);
y_positions = fixed_var_path_state_log(:,2);

% Satellite 1 Pseudorange Meas Plot
figure;
plot3(x_positions, y_positions, sat1_meas);
grid on;
xlabel('X Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Y Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
zlabel('Z Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
title('Satellite 1 Pseudorange Measurements Over Trajectory', 'Interpreter', 'latex', 'Fontsize', 16);

figure;
plot3(x_positions, y_positions, sat2_meas);
grid on;
xlabel('X Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Y Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
zlabel('Z Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
title('Satellite 2 Pseudorange Measurements Over Trajectory', 'Interpreter', 'latex', 'Fontsize', 16);

figure;
plot3(x_positions, y_positions, sat3_meas);
grid on;
xlabel('X Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Y Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
zlabel('Z Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
title('Satellite 3 Pseudorange Measurements Over Trajectory', 'Interpreter', 'latex', 'Fontsize', 16);

figure;
plot3(x_positions, y_positions, sat4_meas);
grid on;
xlabel('X Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
ylabel('Y Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
zlabel('Z Position (meters)', 'Interpreter', 'latex', 'Fontsize', 14);
title('Satellite 4 Pseudorange Measurements Over Trajectory', 'Interpreter', 'latex', 'Fontsize', 16);

%% 5 EKF 
fixed_variance = 0;
rng(1)
x0 = [0;0;0;0]; % initially at rest at origin, state x is [x; y; xdot; ydot]
time= 10; % in seconds
t = (0:params.delta_t:time)';
% Force input
fx = 500*cos(.4*t);
fy = 1000*sin(.05*t);
u = [fx, fy];
% Simulate Dynamics and Log
Q = 1*10^-10*eye(4);
R = diag([3,3,3,3]);
satellite1_location = [0, 0];
satellite2_location = [10, 0];
satellite3_location = [0, 10];
satellite4_location = [10, 10];
all_sat_positions = [satellite1_location; satellite2_location; satellite3_location; satellite4_location];

[muEst1,~] = EKF(x0,u,time,all_sat_positions,Q,R,params,fixed_variance);

fixed_variance = 1;
[muEst2,mu] = EKF(x0,u,time,all_sat_positions,Q,R,params,fixed_variance);

EVGRC = [8, 2];
EVGRD = [3, 6];
satellite1_location = [0, 0];
satellite2_location = [10, 0];
satellite3_location = [0, 10];
satellite4_location = [10, 10];

figure()
hold on
plot(mu(1,:),mu(2,:))
plot(muEst1(1,:),muEst1(2,:))
plot(muEst2(1,:),muEst2(2,:))
legend('mu','muEstVari','muEstFixed')
plot(EVGRC(1), EVGRC(2),  's', 'MarkerFaceColor', 'black', 'MarkerSize', 100);
plot(EVGRD(1), EVGRD(2),  's', 'MarkerFaceColor', 'magenta', 'MarkerSize', 100);
plot(satellite1_location(1), satellite1_location(2),  'bo', 'MarkerFaceColor', 'red', 'MarkerSize', 10);
plot(satellite2_location(1), satellite2_location(2),  'bo', 'MarkerFaceColor', 'blue', 'MarkerSize', 10);
plot(satellite3_location(1), satellite3_location(2),  'bo', 'MarkerFaceColor', 'green', 'MarkerSize', 10);
plot(satellite4_location(1), satellite4_location(2),  'bo', 'MarkerFaceColor', 'yellow', 'MarkerSize', 10);
grid on
title('EKF')

%% 6 Modified EKF 
fixed_variance = 0;
rng(1)
x0 = [0;0;0;0]; % initially at rest at origin, state x is [x; y; xdot; ydot]
time= 10; % in seconds
t = (0:params.delta_t:time)';
% Force input
fx = 500*cos(.4*t);
fy = 1000*sin(.05*t);
u = [fx, fy];
% Simulate Dynamics and Log
Q = 1*10^-10*eye(4);
R = diag([3,3,3]);
satellite1_location = [0, 0];
satellite2_location = [10, 0];
satellite3_location = [0, 10];
satellite4_location = [10, 10];
all_sat_positions = [satellite1_location; satellite2_location; satellite3_location; satellite4_location];

[muEst1mod,mumod] = EKFmod(x0,u,time,all_sat_positions,Q,R,params,fixed_variance);


EVGRC = [8, 2];
EVGRD = [3, 6];
satellite1_location = [0, 0];
satellite2_location = [10, 0];
satellite3_location = [0, 10];
satellite4_location = [10, 10];

figure()
hold on
plot(mumod(1,:),mumod(2,:))
plot(muEst1mod(1,:),muEst1mod(2,:))
plot(muEst1(1,:),muEst1(2,:))
legend('mu','mod','non mod')
plot(EVGRC(1), EVGRC(2),  's', 'MarkerFaceColor', 'black', 'MarkerSize', 100);
plot(EVGRD(1), EVGRD(2),  's', 'MarkerFaceColor', 'magenta', 'MarkerSize', 100);
plot(satellite1_location(1), satellite1_location(2),  'bo', 'MarkerFaceColor', 'red', 'MarkerSize', 10);
plot(satellite2_location(1), satellite2_location(2),  'bo', 'MarkerFaceColor', 'blue', 'MarkerSize', 10);
plot(satellite3_location(1), satellite3_location(2),  'bo', 'MarkerFaceColor', 'green', 'MarkerSize', 10);
plot(satellite4_location(1), satellite4_location(2),  'bo', 'MarkerFaceColor', 'yellow', 'MarkerSize', 10);
grid on
title('modified EKF')