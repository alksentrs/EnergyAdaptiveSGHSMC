function M = update_mass_matrix(dx, params)
    % Update mass matrix according to state change rate
    M = diag(params.beta0 + params.beta1 * exp(-params.lambda * abs(dx)));
end
