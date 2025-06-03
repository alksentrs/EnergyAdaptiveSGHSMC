function alpha_new = adapt_energy(alpha, energy_error)
% Adapt the energy parameter based on the energy error
% Inputs:
%   alpha: current energy adaptation rate
%   energy_error: computed energy error
% Outputs:
%   alpha_new: adapted energy parameter

% Adaptation parameters
alpha_min = 0.01;
alpha_max = 1.0;
adapt_rate = 0.1;

% Compute adaptation
alpha_new = alpha + adapt_rate * energy_error;

% Bound the adaptation parameter
alpha_new = max(min(alpha_new, alpha_max), alpha_min);

end 