function [meas_log, state_log] = simSystem(x0, u, time_to_run, params, Q, R)
% Simulates system
% Takes in initial conditions (x0 and xdot0), system inputs (u), the
% time to run (time_to_run), and the system parameters (params, a struct), and 
% returns a log of the state (state_log) and the measured variables (meas_log) of the system
% Q is process noise covariance matrix, R is sensor noise covariance matrix

% Set up Logging
entry_index = 1; 
state_size = 4; % number of states
meas_size = 2; % number of measurements - 1 for position in x and 1 for position in y
num_entries = time_to_run/params.delta_t; % number of entries to log
state_log = zeros(num_entries, state_size); % log states over time
meas_log = zeros(num_entries, meas_size); % log measurements over time

% Log initial state
state_log(entry_index,:) = x0;
meas_log(entry_index,:) = [x0(1); x0(2)];
entry_index = entry_index + 1;

% Simulate Dynamics and Log
t = params.delta_t; % Since we already logged t = 0 entry, start at t = 0 + delta_t
x_last = x0;
while(t < time_to_run)
    % Propagate state forward
    fx = u(entry_index - 1, 1); % (entry_index - 1) so at first time step we reference first input
    fy = u(entry_index - 1, 2); 
    x_dot = [x_last(3); x_last(4); fx/params.m; fy/params.m];
    W = sqrt(Q)*randn(2,1);
    process_noise = [0; 0; W(1); W(2)];
    x_next = x_last + params.delta_t*x_dot + process_noise;
    % Take measurement
    V = sqrt(R)*randn(2,1);
    sensor_noise = [V(1); V(2)];
    curr_meas = [x_next(1); x_next(2)] + sensor_noise;
    % Log
    state_log(entry_index,:) = x_next;
    meas_log(entry_index,:) = curr_meas;

    x_last = x_next;
    entry_index = entry_index + 1;
    t = t + params.delta_t;
end
end