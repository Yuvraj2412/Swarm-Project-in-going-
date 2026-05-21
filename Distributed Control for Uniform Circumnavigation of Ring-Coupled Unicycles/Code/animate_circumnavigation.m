function animate_circumnavigation(t, x, params, speed_multiplier)
%ANIMATE_CIRCUMNAVIGATION Animate the circumnavigation behavior
%   Creates real-time animation showing agents' trajectories and final formation
%
%   Inputs:
%       t               - Time vector from simulation
%       x               - State matrix from simulation
%       params          - Parameter structure
%       speed_multiplier - Animation speed (1.0 = real-time)

if nargin < 4
    speed_multiplier = 1.0;
end

N = params.N;
target_pos = params.target_pos;
radii = params.radii;

% Create figure
fig = figure('Position', [100, 100, 900, 800]);
ax = axes('Parent', fig);
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
xlabel(ax, 'x [m]');
ylabel(ax, 'y [m]');
title(ax, 'Uniform Circumnavigation - Real-time Animation');

% Set axis limits
max_rad = max(radii) * 1.5;
xlim(ax, target_pos(1) + [-max_rad, max_rad]);
ylim(ax, target_pos(2) + [-max_rad, max_rad]);

% Plot target
plot(ax, target_pos(1), target_pos(2), 'rp', 'MarkerSize', 20, ...
    'MarkerFaceColor', 'r', 'LineWidth', 2);
text(ax, target_pos(1), target_pos(2) - 0.3, 'Target', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');

% Plot desired orbit circles
theta_circle = linspace(0, 2*pi, 100);
unique_radii = unique(radii);
for r = unique_radii
    plot(ax, target_pos(1) + r*cos(theta_circle), ...
         target_pos(2) + r*sin(theta_circle), ...
         'k--', 'LineWidth', 1.5, 'Color', [0.5, 0.5, 0.5]);
end

% Colors for agents
colors = lines(N);

% Initialize plot objects for each agent
agent_bodies = gobjects(N, 1);
agent_directions = gobjects(N, 1);
agent_trails = gobjects(N, 1);
agent_labels = gobjects(N, 1);

% Agent size parameters
agent_size = 0.15;  % Size of the agent body
direction_length = 0.25;  % Length of direction indicator

for i = 1:N
    % Initial position
    xi = x(1, 3*i-2);
    yi = x(1, 3*i-1);
    thetai = x(1, 3*i);
    
    % Agent body (circle)
    agent_bodies(i) = plot(ax, xi, yi, 'o', 'Color', colors(i,:), ...
        'MarkerSize', 15, 'MarkerFaceColor', colors(i,:), 'LineWidth', 2);
    
    % Direction indicator (line from center)
    agent_directions(i) = plot(ax, ...
        [xi, xi + direction_length*cos(thetai)], ...
        [yi, yi + direction_length*sin(thetai)], ...
        'Color', colors(i,:), 'LineWidth', 2);
    
    % Trail (empty initially)
    agent_trails(i) = plot(ax, xi, yi, 'Color', colors(i,:), ...
        'LineWidth', 1, 'LineStyle', '-');
    
    % Agent label
    agent_labels(i) = text(ax, xi, yi + 0.3, sprintf('%d', i), ...
        'HorizontalAlignment', 'center', 'FontSize', 9, ...
        'FontWeight', 'bold', 'Color', colors(i,:));
end

% Time display
time_text = text(ax, 0.02, 0.98, sprintf('Time: %.2f s', 0), ...
    'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold', ...
    'BackgroundColor', 'w', 'EdgeColor', 'k');

% Animation loop
dt = t(2) - t(1);  % Approximate time step
frame_skip = max(1, round(0.05 / dt / speed_multiplier));  % Target ~20 fps

for k = 1:frame_skip:length(t)
    if ~ishandle(fig)
        break;  % Figure was closed
    end
    
    % Update each agent
    for i = 1:N
        xi = x(k, 3*i-2);
        yi = x(k, 3*i-1);
        thetai = x(k, 3*i);
        
        % Update agent body position
        set(agent_bodies(i), 'XData', xi, 'YData', yi);
        
        % Update direction indicator
        set(agent_directions(i), ...
            'XData', [xi, xi + direction_length*cos(thetai)], ...
            'YData', [yi, yi + direction_length*sin(thetai)]);
        
        % Update trail
        trail_x = x(1:k, 3*i-2);
        trail_y = x(1:k, 3*i-1);
        set(agent_trails(i), 'XData', trail_x, 'YData', trail_y);
        
        % Update label position
        set(agent_labels(i), 'Position', [xi, yi + 0.3, 0]);
    end
    
    % Update time display
    set(time_text, 'String', sprintf('Time: %.2f s', t(k)));
    
    % Force graphics update
    drawnow;
    
    % Pause to maintain real-time speed
    if k < length(t)
        pause_time = (t(k+min(frame_skip, length(t)-k)) - t(k)) / speed_multiplier;
        pause(pause_time);
    end
end

% Final display - hold for a moment
if ishandle(fig)
    title(ax, 'Uniform Circumnavigation - Final Formation');
    pause(2);
end

% Create summary plot
if ishandle(fig)
    create_summary_plot(t, x, params);
end

end

function create_summary_plot(t, x, params)
%CREATE_SUMMARY_PLOT Create a summary figure showing key metrics

N = params.N;
target_pos = params.target_pos;
radii = params.radii;

figure('Position', [150, 150, 1200, 400]);

% Subplot 1: Distance from target over time
subplot(1, 3, 1);
hold on; grid on;
colors = lines(N);
for i = 1:N
    xi = x(:, 3*i-2);
    yi = x(:, 3*i-1);
    dist = sqrt((xi - target_pos(1)).^2 + (yi - target_pos(2)).^2);
    plot(t, dist, 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Agent %d', i));
    % Plot desired radius
    plot(t, radii(i)*ones(size(t)), '--', 'Color', colors(i,:), 'LineWidth', 1, 'HandleVisibility', 'off');
end
xlabel('Time [s]');
ylabel('Distance from target [m]');
title('Distance from Target');
legend('Location', 'best');

% Subplot 2: Angular velocities over time
subplot(1, 3, 2);
hold on; grid on;
for i = 1:N
    theta = x(:, 3*i);
    omega = [0; diff(theta)./diff(t)];  % Approximate angular velocity
    plot(t, omega, 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', sprintf('Agent %d', i));
end
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');
title('Angular Velocities');
legend('Location', 'best');

% Subplot 3: Final formation
subplot(1, 3, 3);
hold on; axis equal; grid on;

% Plot target
plot(target_pos(1), target_pos(2), 'rp', 'MarkerSize', 15, 'MarkerFaceColor', 'r');

% Plot orbits
theta_circle = linspace(0, 2*pi, 100);
unique_radii = unique(radii);
for r = unique_radii
    plot(target_pos(1) + r*cos(theta_circle), ...
         target_pos(2) + r*sin(theta_circle), 'k--', 'LineWidth', 1);
end

% Plot final agent positions
for i = 1:N
    xi = x(end, 3*i-2);
    yi = x(end, 3*i-1);
    thetai = x(end, 3*i);
    
    % Agent body
    plot(xi, yi, 'o', 'Color', colors(i,:), 'MarkerSize', 12, 'MarkerFaceColor', colors(i,:));
    
    % Direction
    quiver(xi, yi, 0.3*cos(thetai), 0.3*sin(thetai), 0, ...
        'Color', colors(i,:), 'LineWidth', 2, 'MaxHeadSize', 1);
    
    % Label
    text(xi, yi + 0.25, sprintf('%d', i), 'HorizontalAlignment', 'center', ...
        'FontSize', 9, 'FontWeight', 'bold');
end

xlabel('x [m]');
ylabel('y [m]');
title('Final Formation');
max_rad = max(radii) * 1.3;
xlim(target_pos(1) + [-max_rad, max_rad]);
ylim(target_pos(2) + [-max_rad, max_rad]);

sgtitle('Circumnavigation Performance Summary', 'FontSize', 14, 'FontWeight', 'bold');

end
