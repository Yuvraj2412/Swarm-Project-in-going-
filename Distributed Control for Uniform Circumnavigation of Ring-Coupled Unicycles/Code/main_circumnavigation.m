%% Distributed Control for Uniform Circumnavigation of Ring-Coupled Unicycles
% Implementation of: Zheng et al., Automatica 53 (2015) 23-29
% 
% This script simulates a team of unicycle-type agents achieving uniform
% circumnavigation around a stationary target using distributed control.

clear; close all; clc;

%% ========================================================================
%  CONFIGURABLE PARAMETERS - MODIFY THESE AS NEEDED
%  ========================================================================

% Number of agents
N = 5;

% Desired radii for each agent (must be length N)
% Example: All same radius -> radii = 2*ones(1,N)
% Example: Different radii -> radii = [2, 2, 2, 3, 3, 3]
radii = [1, 2, 3, 4, 5];

% Formation parameter d (controls the angular spacing)
% Valid values: d ∈ {1, 2, ..., N-1}
% d = 1 or d = N-1 gives clockwise/counterclockwise motion (most stable)
d = 4;

% Target location [xb, yb]
target_pos = [0, 0];

% Simulation time (seconds)
t_final = 30;

% Control gains
kv = 2.75;  % Linear velocity gain
kw = 3.897; % Angular velocity gain

% Initial conditions: 'random' or 'circle'
% 'random': random positions and orientations
% 'circle': agents start on a circle around target
initial_type = 'random';

% Animation settings
animate_realtime = true;  % Set to false for faster simulation
animation_speed = 1.0;     % Speed multiplier (1.0 = real-time)

%% ========================================================================
%  PARAMETER VALIDATION AND AUTOMATIC CALCULATIONS
%  ========================================================================

% Validate inputs
if length(radii) ~= N
    error('Length of radii must equal N');
end
if d < 1 || d > N-1
    error('d must be in the range {1, 2, ..., N-1}');
end

% Calculate psi_bar (desired angular spacing)
psi_bar = 2*d*pi/N;

% Determine parameter c based on Theorem 5
% For simplicity, we use Case I: c = 1 - ε (stable for d = 1 or d = N-1)
% For other cases, adjust c according to Theorem 5
epsilon = 0.1;  % Small positive value
if d == 1 || d == N-1
    c = 1 - epsilon;
elseif mod(N, 2) == 1 && (d == floor(N/2) || d == ceil(N/2))
    % Case II: odd N
    c = -1;
elseif mod(N, 2) == 0 && (d == N/2-1 || d == N/2+1)
    % Case III: even N
    c = -1 + epsilon;
else
    % Default to Case I approach
    c = 1 - epsilon;
    warning('Formation may not be asymptotically stable for this d value');
end

% Calculate control parameters for each agent (Theorem 1)
delta = 1 ./ radii;  % δi = 1/ri
a = zeros(1, N);
for i = 1:N
    i_next = mod(i, N) + 1;
    a(i) = 1 - c * (radii(i) / radii(i_next));  % ai = 1 - c*(ri/ri+1)
end

% Verify kv/kw ratio from Theorem 3
kv_kw_theoretical = (1 - c*cos(psi_bar)) / abs(c*sin(psi_bar));
kv_kw_actual = kv / kw;
if abs(kv_kw_theoretical - kv_kw_actual) > 0.01
    fprintf('Warning: kv/kw ratio mismatch!\n');
    fprintf('Theoretical: %.4f, Actual: %.4f\n', kv_kw_theoretical, kv_kw_actual);
    fprintf('Adjusting kw to match theoretical ratio...\n');
    kw = kv / kv_kw_theoretical;
end

%% ========================================================================
%  INITIAL CONDITIONS
%  ========================================================================

% State vector: [x1, y1, theta1, x2, y2, theta2, ..., xN, yN, thetaN]
x0 = zeros(3*N, 1);

if strcmp(initial_type, 'random')
    % Random initial positions in a circle around target
    radius_init = 1.5 * max(radii);            %Through this step we are randomizing the agents radius relative to target
    for i = 1:N
        angle_init = 2*pi*rand();              %through this tep we are ranomizing the angle of each agent from the target
        x0(3*i-2) = target_pos(1) + radius_init * cos(angle_init);  % xi       This is to convert polar to cartesian coordinates
        x0(3*i-1) = target_pos(2) + radius_init * sin(angle_init);  % yi
        x0(3*i) = 2*pi*rand();  % thetai
    end                                        %Although here we are using global coordinates of agents and target but this only for the purpose of randomizing the positions, the control law still uses the relative position and velocity to target
elseif strcmp(initial_type, 'circle')
    % Start agents uniformly on a circle
    avg_radius = mean(radii);
    for i = 1:N
        angle_init = 2*pi*(i-1)/N;
        x0(3*i-2) = target_pos(1) + avg_radius * cos(angle_init);
        x0(3*i-1) = target_pos(2) + avg_radius * sin(angle_init);
        x0(3*i) = angle_init + pi/2;  % Tangent to circle
    end
end

%% ========================================================================
%  SIMULATION
%  ========================================================================

fprintf('Starting simulation with %d agents...\n', N);
fprintf('Formation parameter d = %d, psi_bar = %.4f rad (%.2f deg)\n', ...
    d, psi_bar, psi_bar*180/pi);
fprintf('Control parameter c = %.4f\n', c);

% Package parameters for ODE solver
params.N = N;
params.radii = radii;
params.a = a;
params.delta = delta;
params.kv = kv;
params.kw = kw;
params.target_pos = target_pos;

% Solve ODE
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);         %These are nothing but the accuracy and preision defined for ode45 
[t, x] = ode45(@(t,x) unicycle_dynamics(t, x, params), [0 t_final], x0, options);
%{returns the time and positon.  tells to use this file's this function
%giving the values as t,x,params.     tells the simulation time,tells
%the initial position and angle for each agent, tells the options to solve
%with}
fprintf('Simulation complete!\n');

%% ========================================================================
%  ANIMATION AND VISUALIZATION
%  ========================================================================

if animate_realtime
    fprintf('Creating animation...\n');
    animate_circumnavigation(t, x, params, animation_speed);
else
    % Just plot final trajectories
    plot_trajectories(t, x, params);
end

fprintf('Done!\n');

%% ========================================================================
%  HELPER FUNCTIONS
%  ========================================================================

function plot_trajectories(t, x, params)
    % Plot final trajectories only
    
    figure('Position', [100, 100, 800, 800]);
    hold on; axis equal; grid on;
    
    N = params.N;
    target_pos = params.target_pos;
    radii = params.radii;
    
    % Plot target
    plot(target_pos(1), target_pos(2), 'rp', 'MarkerSize', 20, ...
        'MarkerFaceColor', 'r', 'DisplayName', 'Target');
    
    % Plot desired orbits
    theta_circle = linspace(0, 2*pi, 100);
    unique_radii = unique(radii);
    for r = unique_radii
        plot(target_pos(1) + r*cos(theta_circle), ...
             target_pos(2) + r*sin(theta_circle), ...
             'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
    end
    
    % Colors for agents
    colors = lines(N);
    
    % Plot trajectories
    for i = 1:N
        xi = x(:, 3*i-2);
        yi = x(:, 3*i-1);
        
        % Trajectory
        plot(xi, yi, 'Color', colors(i,:), 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Agent %d', i));
        
        % Initial position (circle)
        plot(xi(1), yi(1), 'o', 'Color', colors(i,:), ...
            'MarkerSize', 8, 'MarkerFaceColor', 'w', 'HandleVisibility', 'off');
        
        % Final position (filled circle)
        plot(xi(end), yi(end), 'o', 'Color', colors(i,:), ...
            'MarkerSize', 8, 'MarkerFaceColor', colors(i,:), 'HandleVisibility', 'off');
    end
    
    xlabel('x [m]');
    ylabel('y [m]');
    title('Agent Trajectories - Uniform Circumnavigation');
    legend('Location', 'best');
    
    % Set axis limits
    max_rad = max(radii) * 1.5;
    xlim(target_pos(1) + [-max_rad, max_rad]);
    ylim(target_pos(2) + [-max_rad, max_rad]);
end