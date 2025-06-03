function grad_U = compute_potential_gradient(pos, vel, target_pos, target_vel, robot)
% Compute gradient of potential energy
% This includes both position and velocity tracking terms

% Position error term
pos_error = pos - target_pos;
vel_error = vel - target_vel;

% Compute gradient
grad_U = [pos_error; vel_error];

end 