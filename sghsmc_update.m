function x_next = sghsmc_update(x, target_pos, target_vel, target_theta, params, robot, dt)
% SGHSMC update step for quadrupedal robot tracking
% Inputs:
%   x: current state [x_pos; y_pos; x_vel; y_vel; angle; angular_velocity]
%   target_pos: target position
%   target_vel: target velocity
%   target_theta: target angle
%   params: SGHSMC parameters
%   robot: robot parameters
%   dt: time step
% Outputs:
%   x_next: next state

% Extract current state
pos = x(1:2);
vel = x(3:4);
theta = x(5);
omega = x(6);

% Compute gradient of potential energy
grad_U = compute_potential_gradient(pos, vel, target_pos, target_vel, robot);

% Compute stochastic gradient
grad_U = grad_U + params.beta * randn(size(grad_U));

% Update momentum (velocity)
p = vel + dt * (-params.gamma * vel - grad_U(3:4) + params.alpha * (target_vel - vel));

% Update position
q = pos + dt * p;

% Update angular velocity
omega_next = omega + dt * (-params.gamma * omega + params.alpha * (target_theta - theta));

% Update angle
theta_next = theta + dt * omega_next;

% Combine states
x_next = [q; p; theta_next; omega_next];

end