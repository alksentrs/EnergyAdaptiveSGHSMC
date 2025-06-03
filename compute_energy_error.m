function error = compute_energy_error(pos, vel, target_pos, target_vel, robot)
% Compute energy error between current and target states
% Inputs:
%   pos: current position
%   vel: current velocity
%   target_pos: target position
%   target_vel: target velocity
%   robot: robot parameters
% Outputs:
%   error: energy error

% Kinetic energy error
kinetic_error = 0.5 * robot.mass * (norm(vel - target_vel))^2;

% Potential energy error (gravitational + position)
potential_error = 0.5 * robot.mass * robot.g * norm(pos - target_pos);

% Total energy error
error = kinetic_error + potential_error;

end 