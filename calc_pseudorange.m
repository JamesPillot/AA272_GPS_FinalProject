function [meas] = calc_pseudorange(satellite_position, vehicle_position)
% Takes in satellite position, vehicle position, and noise and outputs
% pseudorange 
x_veh = vehicle_position(1);
y_veh = vehicle_position(2);
x_sat = satellite_position(1);
y_sat = satellite_position(2);

meas = sqrt((x_sat - x_veh)^2 + (y_sat - y_veh)^2);
end