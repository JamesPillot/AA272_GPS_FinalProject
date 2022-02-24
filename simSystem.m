function [meas_log, state_log] = simSystem(x0, u, time_to_run, params)
% Simulates system
% Takes in initial conditions (x0 and xdot0), system inputs (u), the
% time to run (time_to_run), and the system parameters (params, a struct), and 
% returns a log of the state (state_log) and the measured variables (meas_log) of the system

% Set up Logging
entry_index = 1; 
state_size = 6; % number of states
meas_size = 2; % number of measurements - 1 for x and 1 for y
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
    left_thrust = u(entry_index - 1, 1); % (entry_index - 1) so that at first time step we reference first input
    right_thrust = u(entry_index - 1, 2); 
    x_dot = [x_last(4); x_last(5); x_last(6); ...
        ((left_thrust+right_thrust)/params.m)*cos(x_last(3)); ((left_thrust+right_thrust)/params.m)*sin(x_last(3)); ((params.L)/2)*(right_thrust - left_thrust)*(1/params.Jcm_zz)];
    
    x_next = x_last + params.delta_t*x_dot;
    % Take measurement
    curr_meas = [x_next(1); x_next(2)];
    % Log
    state_log(entry_index,:) = x_next;
    meas_log(entry_index,:) = curr_meas;

    x_last = x_next;
    entry_index = entry_index + 1;
    t = t + params.delta_t;
end
end