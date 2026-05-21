%% Advanced Distributed Circumnavigation with CBF and Custom Spacing
% Enhanced implementation with:
% 1. Control Barrier Functions (CBF) for collision avoidance
% 2. Custom spacing support via strategic radii assignment
% 3. Smart feasibility checking and adjustment

clear; close all; clc;

%% ========================================================================
%  CONFIGURATION MODE - Choose one:
%  ========================================================================

% Mode 1: UNIFORM SPACING (original algorithm, proven stable)
% Mode 2: CUSTOM SPACING (user-defined angular spacing)
config_mode = 'UNIFORM';  % Options: 'UNIFORM' or 'CUSTOM'

%% ========================================================================
%  BASIC PARAMETERS
%  ========================================================================

N = 6;                    % Number of agents
target_pos = [0, 0];      % Target location
t_final = 40;             % Simulation time (seconds)

% Collision avoidance parameters
enable_cbf = true;        % Enable CBF collision avoidance
d_safe = 0.4;             % Minimum safe distance between agents (meters)
d_target_min = 0.3;       % Minimum distance from target (meters)
cbf_alpha = 1.0;          % CBF decay rate (higher = more aggressive avoidance)

% Initial conditions
initial_type = 'random';  % 'random' or 'circle'

% Animation
animate_realtime = true;
animation_speed = 1.0;

%% ========================================================================
%  MODE-SPECIFIC CONFIGURATION
%  ========================================================================

if strcmp(config_mode, 'UNIFORM')
    %% UNIFORM SPACING MODE
    fprintf('=== MODE: UNIFORM SPACING ===\n\n');
    
    % Standard parameters
    radii = 2.5 * ones(1, N);  % All agents on same radius
    d = 1;                      % Formation parameter
    
    % Calculate spacing
    psi_bar = 2*d*pi/N;
    desired_spacings = psi_bar * ones(1, N);
    
else
    %% CUSTOM SPACING MODE
    fprintf('=== MODE: CUSTOM SPACING ===\n\n');
    
    % USER INPUT: Desired angular spacings (in degrees)
    % These are the spacings BETWEEN consecutive agents
    desired_spacings_deg = [45, 60, 75, 90, 90];  % Example: 5 agents
    
    % If you want to specify for different N, make sure length matches!
    % Or set N based on length:
    if length(desired_spacings_deg) ~= N
        fprintf('Adjusting N to match spacing vector length...\n');
        N = length(desired_spacings_deg);
        fprintf('New N = %d\n', N);
    end
    
    % Convert to radians
    desired_spacings = desired_spacings_deg * pi/180;
    
    % Check feasibility and adjust if needed
    [desired_spacings, N, is_feasible, adjustment_made] = ...
        check_spacing_feasibility(desired_spacings, N);
    
    if adjustment_made
        fprintf('\n⚠ Spacing adjusted to ensure feasibility (sum = 360°)\n');
        fprintf('Final spacings: [%s] degrees\n', ...
            num2str(desired_spacings*180/pi, '%.1f '));
        fprintf('Sum = %.1f degrees\n\n', sum(desired_spacings)*180/pi);
    end
    
    % Base radius (will be adjusted slightly for each agent)
    r_nominal = 2.5;
    
    % Generate strategic radii based on spacings
    [radii, radii_variation] = calculate_radii_from_spacings(...
        desired_spacings, r_nominal, N);
    
    fprintf('Strategic radii generated:\n');
    fprintf('  Nominal radius: %.2f m\n', r_nominal);
    fprintf('  Variation range: ±%.2f%% (%.4f m)\n', ...
        radii_variation*100, radii_variation*r_nominal);
    fprintf('  Radii: [%s]\n\n', num2str(radii, '%.4f '));
    
    % For custom spacing, we use d=1 as base but optimize control params
    d = 1;
end

%% ========================================================================
%  CONTROL PARAMETER CALCULATION
%  ========================================================================

fprintf('Calculating control parameters...\n');

% Calculate standard parameters
psi_bar = mean(desired_spacings);  % Use mean spacing for calculations
epsilon = 0.1;

% Determine c based on stability conditions
if d == 1 || d == N-1
    c = 1 - epsilon;
elseif mod(N, 2) == 1 && (d == floor(N/2) || d == ceil(N/2))
    c = -1;
elseif mod(N, 2) == 0 && (d == N/2-1 || d == N/2+1)
    c = -1 + epsilon;
else
    c = 1 - epsilon;
end

% For custom spacing mode, optimize control parameters
if strcmp(config_mode, 'CUSTOM')
    fprintf('Optimizing control parameters for custom spacing...\n');
    [c, kv, kw, optimization_success] = optimize_control_parameters(...
        desired_spacings, radii, c);
    
    if ~optimization_success
        warning('Optimization did not fully converge. Using approximate values.');
    end
else
    % Standard control gains
    kv = 2.5;
    kv_kw_theoretical = (1 - c*cos(psi_bar)) / abs(c*sin(psi_bar));
    kw = kv / kv_kw_theoretical;
end

fprintf('Control parameters:\n');
fprintf('  c = %.4f\n', c);
fprintf('  kv = %.4f\n', kv);
fprintf('  kw = %.4f\n\n', kw);

% Calculate agent-specific parameters
delta = 1 ./ radii;
a = zeros(1, N);
for i = 1:N
    i_next = mod(i, N) + 1;
    a(i) = 1 - c * (radii(i) / radii(i_next));
end

%% ========================================================================
%  INITIAL CONDITIONS
%  ========================================================================

x0 = zeros(3*N, 1);

if strcmp(initial_type, 'random')
    radius_init = 1.5 * max(radii);
    for i = 1:N
        angle_init = 2*pi*rand();
        x0(3*i-2) = target_pos(1) + radius_init * cos(angle_init);
        x0(3*i-1) = target_pos(2) + radius_init * sin(angle_init);
        x0(3*i) = 2*pi*rand();
    end
elseif strcmp(initial_type, 'circle')
    avg_radius = mean(radii);
    for i = 1:N
        angle_init = 2*pi*(i-1)/N;
        x0(3*i-2) = target_pos(1) + avg_radius * cos(angle_init);
        x0(3*i-1) = target_pos(2) + avg_radius * sin(angle_init);
        x0(3*i) = angle_init + pi/2;
    end
end

%% ========================================================================
%  PACKAGE PARAMETERS
%  ========================================================================

params.N = N;
params.radii = radii;
params.a = a;
params.delta = delta;
params.kv = kv;
params.kw = kw;
params.target_pos = target_pos;
params.desired_spacings = desired_spacings;

% CBF parameters
params.enable_cbf = enable_cbf;
params.d_safe = d_safe;
params.d_target_min = d_target_min;
params.cbf_alpha = cbf_alpha;

%% ========================================================================
%  SIMULATION
%  ========================================================================

fprintf('Starting simulation...\n');
if enable_cbf
    fprintf('  ✓ CBF collision avoidance ENABLED\n');
    fprintf('    - Safe distance: %.2f m\n', d_safe);
    fprintf('    - Target min distance: %.2f m\n', d_target_min);
else
    fprintf('  ✗ CBF collision avoidance DISABLED\n');
end
fprintf('  Simulating for %.1f seconds...\n\n', t_final);

% Solve ODE
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
tic;
[t, x] = ode45(@(t,x) unicycle_dynamics_cbf(t, x, params), ...
    [0 t_final], x0, options);
elapsed = toc;

fprintf('Simulation complete! (%.2f seconds)\n\n', elapsed);

%% ========================================================================
%  PERFORMANCE ANALYSIS
%  ========================================================================

fprintf('=== PERFORMANCE METRICS ===\n\n');

% Final distances from target
fprintf('1. Distance from target:\n');
final_distances = zeros(N, 1);
for i = 1:N
    xi = x(end, 3*i-2);
    yi = x(end, 3*i-1);
    final_distances(i) = sqrt((xi - target_pos(1))^2 + (yi - target_pos(2))^2);
    error_percent = abs(final_distances(i) - radii(i)) / radii(i) * 100;
    fprintf('   Agent %d: %.4f m (desired: %.4f m, error: %.2f%%)\n', ...
        i, final_distances(i), radii(i), error_percent);
end

% Final angular spacings
fprintf('\n2. Angular spacing:\n');
angles = zeros(N, 1);
for i = 1:N
    xi = x(end, 3*i-2) - target_pos(1);
    yi = x(end, 3*i-1) - target_pos(2);
    angles(i) = atan2(yi, xi);
end
angles = mod(angles, 2*pi);
[angles_sorted, idx] = sort(angles);
actual_spacings = diff([angles_sorted; angles_sorted(1) + 2*pi]);

% Reorder to match agent numbering
actual_spacings_ordered = zeros(N, 1);
for i = 1:N
    agent_num = idx(i);
    next_agent_num = idx(mod(i, N) + 1);
    agent_pos = find(1:N == agent_num);
    actual_spacings_ordered(agent_num) = actual_spacings(agent_pos);
end

for i = 1:N
    desired_deg = desired_spacings(i) * 180/pi;
    actual_deg = actual_spacings_ordered(i) * 180/pi;
    error_deg = abs(desired_deg - actual_deg);
    fprintf('   Agent %d to %d: %.2f° (desired: %.2f°, error: %.2f°)\n', ...
        i, mod(i,N)+1, actual_deg, desired_deg, error_deg);
end

% Check for collisions during simulation
fprintf('\n3. Collision check:\n');
min_distance_overall = inf;
min_distance_to_target = inf;
collision_detected = false;

for k = 1:length(t)
    for i = 1:N
        xi = x(k, 3*i-2);
        yi = x(k, 3*i-1);
        
        % Check distance to target
        dist_to_target = sqrt((xi - target_pos(1))^2 + (yi - target_pos(2))^2);
        min_distance_to_target = min(min_distance_to_target, dist_to_target);
        
        % Check distance to other agents
        for j = i+1:N
            xj = x(k, 3*j-2);
            yj = x(k, 3*j-1);
            dist = sqrt((xi - xj)^2 + (yi - yj)^2);
            min_distance_overall = min(min_distance_overall, dist);
            
            if dist < d_safe
                collision_detected = true;
            end
        end
    end
end

fprintf('   Minimum inter-agent distance: %.4f m (safe threshold: %.2f m)\n', ...
    min_distance_overall, d_safe);
fprintf('   Minimum target distance: %.4f m (safe threshold: %.2f m)\n', ...
    min_distance_to_target, d_target_min);

if enable_cbf
    if ~collision_detected && min_distance_overall >= d_safe
        fprintf('   ✓ No collisions detected - CBF working correctly!\n');
    else
        fprintf('   ⚠ Warning: Constraints may have been violated\n');
    end
else
    if collision_detected
        fprintf('   ⚠ Collisions detected (CBF disabled)\n');
    else
        fprintf('   ✓ No collisions (lucky!)\n');
    end
end

%% ========================================================================
%  VISUALIZATION
%  ========================================================================

if animate_realtime
    fprintf('\nCreating animation...\n');
    animate_circumnavigation(t, x, params, animation_speed);
end

fprintf('\n=== SIMULATION COMPLETE ===\n');
