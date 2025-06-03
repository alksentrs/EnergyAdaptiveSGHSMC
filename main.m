% Energy-Adaptive SGHSMC for Quadrupedal Robot Tracking (Particle Filter Version)
% Implementation based on the paper "Energy-Adaptive SGHSMC"

clear all;
close all;
clc;

% Simulation parameters
T = 10;          % Total simulation time (s)
dt = 0.01;       % Time step (s)
N = T/dt;        % Number of time steps
t = 0:dt:T;      % Time vector

% Number of particles
M = 200;

% Robot parameters
robot.mass = 20;     % Robot mass (kg)
robot.inertia = 2;   % Moment of inertia (kg*m^2)
robot.g = 9.81;      % Gravity (m/s^2)

% SGHSMC parameters
params.alpha0 = 0.5;
params.beta0 = 0.01;
params.beta1 = 0.1;
params.lambda = 0.1;
params.gamma1 = 0.5;
params.epsilon = 0.001;
params.m = 50;
params.C = 2.0;
params.B = 0.001;
params.R = 1.e-1*eye(6);      % Measurement noise covariance (6x6 for full state)

% Target trajectory (example: circular path)
target_radius = 2;
target_omega = 0.5;
target_x = target_radius * cos(target_omega * t);
target_y = target_radius * sin(target_omega * t);
target_theta = atan2(target_y, target_x);

% Initial conditions for all particles
x0 = [target_x(1); target_y(1); 0; 0; target_theta(1); 0];
x = zeros(6,M); %repmat(x0, 1, M);      % 6 x M
r = zeros(6, M);           % 6 x M
w = ones(M, 1) / M;        % M x 1, initial weights

% Storage for all time steps
x_hist = zeros(6, N+1, M);
r_hist = zeros(6, N+1, M);
w_hist = zeros(M, N+1);
y_hist = zeros(6, N+1);

x_hist(:,1,:) = x;
r_hist(:,1,:) = r;
w_hist(:,1) = w;

for k = 1:N
    % Add measurement noise
    y = [target_x(k); target_y(k); 
         -target_radius * target_omega * sin(target_omega * t(k));
          target_radius * target_omega * cos(target_omega * t(k));
         target_theta(k); 0] ...
        + mvnrnd(zeros(6,1), params.R)';

    for m = 1:M
        % Predict state for particle m
        x_pred = predict_state(x(:,m), dt);

        % Calculate state change rate
        dx = (x_pred - x(:,m)) / dt;

        % Update mass matrix
        Mmat = update_mass_matrix(dx, params);

        % Update adaptive parameter
        params.alpha = params.alpha0 * exp(-params.gamma1 * norm(y - h(x_pred)));

        % Initialize for simulation steps
        xi = x_pred;
        ri = r(:,m);

        for i = 1:params.m
            xi = xi + params.epsilon * (Mmat \ ri);
            grad_U = compute_gradient(xi, y, x(:,m), params);
            ri = ri - params.epsilon * grad_U - params.epsilon * params.C * (Mmat \ ri) + ...
                 sqrt(2 * (params.C - params.B) * params.epsilon) * randn(6,1);
        end

        % Update state and momentum for particle m
        x(:,m) = xi;
        r(:,m) = ri;

        % Compute weight for particle m
        innov = h(x(:,m)) - y;
        w(m) = exp(-0.5 * innov' * (params.R \ innov));
    end

    % Normalize weights
    w = w / sum(w);

    % Resample particles based on weights
    idx = randsample(1:M, M, true, w);
    x = x(:,idx);
    r = r(:,idx);
    w = ones(M,1) / M;

    % Store history
    x_hist(:,k+1,:) = x;
    r_hist(:,k+1,:) = r;
    w_hist(:,k+1) = w;
    y_hist(:,k+1) = y;
end

% For plotting: use the weighted mean trajectory
x_mean = mean(x_hist, 3);

plot_results(t, x_mean, target_x, target_y, y_hist);

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

function M = update_mass_matrix(dx, params)
    % Update mass matrix according to state change rate
    M = diag(params.beta0 + params.beta1 * exp(-params.lambda * abs(dx)));
end

function grad_U = compute_gradient(x, y, x_prev, params)
    % Compute gradient of potential energy
    % Includes measurement error and state transition terms
    
    % Measurement error term
    h_x = h(x);
    measurement_error = y - h_x;
    
    % State transition term
    state_error = x - x_prev;
    
    % Combined gradient (both terms are 6x1 vectors)
    grad_U = -params.R \ measurement_error + 2 * params.alpha * state_error;
end

function y = h(x)
    % Measurement model (full state)
    y = x;  % In this case, we can measure the full state
end 
