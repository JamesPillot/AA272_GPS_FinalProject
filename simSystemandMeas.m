function [meas_log, state_log] = simSystemandMeas(x0, u, time_to_run, params, Q, R, sat_positions, fixed_variance)
% Simulates system and measurements using pseudoranges
% - Takes in initial conditions (x0 and xdot0), system inputs (u), the
% time to run (time_to_run), and the system parameters (params, a struct), and 
% returns a log of the state (state_log) and the measured variables (meas_log) of the system
% - Q is process noise covariance matrix, R is sensor noise covariance matrix
% - sat_positions is collection of satellite positions
% - fixed_variance is 1 if the variance of the noise to add to measurements is fixed and 0 if it
% is dynamic

% Set up Logging
entry_index = 1; 
state_size = 4; % number of states
meas_size = 4; % number of measurements - 4, 1 psuedorange for each satellite
num_entries = time_to_run/params.delta_t; % number of entries to log
state_log = zeros(num_entries, state_size); % log states over time
meas_log = zeros(num_entries, meas_size); % log measurements over time

% Log initial state
state_log(entry_index,:) = x0;
true_x_pos = x0(1);
true_y_pos = x0(2);
sensor_noise = sqrt(R)*randn(1,1);
pseudorange1_fixed = calc_pseudorange(sat_positions(1,:), [true_x_pos, true_y_pos]) + sensor_noise;
pseudorange2_fixed = calc_pseudorange(sat_positions(2,:), [true_x_pos, true_y_pos]) + sensor_noise;
pseudorange3_fixed = calc_pseudorange(sat_positions(3,:), [true_x_pos, true_y_pos]) + sensor_noise;
pseudorange4_fixed = calc_pseudorange(sat_positions(4,:), [true_x_pos, true_y_pos]) + sensor_noise;
curr_meas = [pseudorange1_fixed; pseudorange2_fixed; pseudorange3_fixed; pseudorange4_fixed];
meas_log(entry_index,:) = curr_meas;
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
    true_x_pos = x_next(1);
    true_y_pos = x_next(2);
    % Take pseudorange measurements and add fixed variance noise
    if(fixed_variance)
        sensor_noise = sqrt(R)*randn(1,1);
        pseudorange1_fixed = calc_pseudorange(sat_positions(1,:), [true_x_pos, true_y_pos]) + sensor_noise;
        pseudorange2_fixed = calc_pseudorange(sat_positions(2,:), [true_x_pos, true_y_pos]) + sensor_noise;
        pseudorange3_fixed = calc_pseudorange(sat_positions(3,:), [true_x_pos, true_y_pos]) + sensor_noise;
        pseudorange4_fixed = calc_pseudorange(sat_positions(4,:), [true_x_pos, true_y_pos]) + sensor_noise;
        curr_meas = [pseudorange1_fixed; pseudorange2_fixed; pseudorange3_fixed; pseudorange4_fixed];
    else        
        [R1, R2, R3, R4] = getCN0var(entry_index); % Time varying variance with obstacle consideration
        sensor_noise1 = sqrt(R1)*randn(1,1);
        sensor_noise2 = sqrt(R2)*randn(1,1);
        sensor_noise3 = sqrt(R3)*randn(1,1);
        sensor_noise4 = sqrt(R4)*randn(1,1);

        pseudorange1_fixed = calc_pseudorange(sat_positions(1,:), [true_x_pos, true_y_pos]) + sensor_noise1;
        pseudorange2_fixed = calc_pseudorange(sat_positions(2,:), [true_x_pos, true_y_pos]) + sensor_noise2;
        pseudorange3_fixed = calc_pseudorange(sat_positions(3,:), [true_x_pos, true_y_pos]) + sensor_noise3;
        pseudorange4_fixed = calc_pseudorange(sat_positions(4,:), [true_x_pos, true_y_pos]) + sensor_noise4;
        curr_meas = [pseudorange1_fixed; pseudorange2_fixed; pseudorange3_fixed; pseudorange4_fixed];
    end
    % Log
    state_log(entry_index,:) = x_next;
    meas_log(entry_index,:) = curr_meas;

    x_last = x_next;
    entry_index = entry_index + 1;
    t = t + params.delta_t;
end
end