function plot_results(t, x, target_x, target_y, hist_y)
% Plot the simulation results
% Inputs:
%   t: time vector
%   x: state history
%   target_x: target x position
%   target_y: target y position

% Create figure
figure('Name', 'Energy-Adaptive SGHSMC Tracking Results', 'NumberTitle', 'off');

% Plot position tracking
subplot(2,2,1);
plot(t, hist_y(1,:), 'y--', 'LineWidth', 1.5);
hold on;
plot(t, x(1,:), 'b-', 'LineWidth', 1.5);
plot(t, target_x, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('X Position (m)');
legend('Measure', 'Filter', 'Target');
title('X Position Tracking');
grid on;

subplot(2,2,2);
plot(t, hist_y(2,:), 'y--', 'LineWidth', 1.5);
hold on;
plot(t, x(2,:), 'b-', 'LineWidth', 1.5);
plot(t, target_y, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Y Position (m)');
legend('Measure', 'Filter', 'Target');
title('Y Position Tracking');
grid on;

% Plot velocity
subplot(2,2,3);
plot(t, x(3,:), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('X Velocity (m/s)');
title('X Velocity');
grid on;

subplot(2,2,4);
plot(t, x(4,:), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Y Velocity (m/s)');
title('Y Velocity');
grid on;

% Plot trajectory
figure('Name', 'Robot Trajectory', 'NumberTitle', 'off');
plot(hist_y(1,:), hist_y(2,:), 'y--', 'LineWidth', 1.5);
hold on;
plot(x(1,:), x(2,:), 'b-', 'LineWidth', 1.5);
plot(target_x, target_y, 'r--', 'LineWidth', 1.5);
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Measure', 'Filter', 'Target');
title('Robot Trajectory');
grid on;
axis equal;

end 