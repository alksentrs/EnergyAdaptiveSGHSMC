function grad_U = compute_gradient(x, y, x_prev, params)
    % Compute gradient of potential energy
    % Includes measurement error and state transition terms
    
    % Measurement error term
    h_x = h(x);
    measurement_error = y - h_x;
    
    % State transition term
    state_error = x - x_prev;
    
    % Combined gradient (both terms are 6x1 vectors)
    grad_U = [-params.R \ measurement_error; 0; 0; 0; 0] + 2 * params.alpha * state_error;
end