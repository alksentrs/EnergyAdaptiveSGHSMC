function x_pred = predict_state(x, dt)
    % Simple prediction model
    pos = x(1:2);
    vel = x(3:4);
    theta = x(5);
    omega = x(6);
    
    % Update position
    pos_next = pos + dt * vel;
    
    % Update angle
    theta_next = theta + dt * omega;
    
    x_pred = [pos_next; vel; theta_next; omega];
end