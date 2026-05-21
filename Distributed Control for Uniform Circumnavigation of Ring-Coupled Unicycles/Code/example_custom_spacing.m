%% Example: Custom Spacing with CBF Collision Avoidance
% This example demonstrates:
% 1. User-defined angular spacings (non-uniform)
% 2. Strategic radii assignment
% 3. CBF collision avoidance
% 4. Automatic parameter optimization

clear; close all; clc;

fprintf('========================================\n');
fprintf('  CUSTOM SPACING EXAMPLE\n');
fprintf('========================================\n\n');

%% Scenario 1: Clustered formation (agents in pairs)
fprintf('--- Scenario: Paired agents (60-30 pattern) ---\n\n');

% 6 agents in 3 pairs, close spacing within pairs, wider between pairs
desired_spacings_deg = [60, 30, 60, 30, 60, 30];  % degrees

% Convert to radians
desired_spacings = desired_spacings_deg * pi/180;

% Check sum
fprintf('Desired spacings: [%s] degrees\n', num2str(desired_spacings_deg, '%.0f '));
fprintf('Sum: %.1f degrees ', sum(desired_spacings_deg));
if abs(sum(desired_spacings_deg) - 360) < 0.1
    fprintf('✓ (valid)\n\n');
else
    fprintf('✗ (will be normalized)\n\n');
end

% Run simulation
run_custom_spacing_simulation(desired_spacings);

%% Scenario 2: Asymmetric coverage pattern
fprintf('\n\n--- Scenario: Asymmetric surveillance pattern ---\n\n');

% Wide coverage on one side, concentrated on the other
desired_spacings_deg = [90, 90, 45, 45, 45, 45];

desired_spacings = desired_spacings_deg * pi/180;

fprintf('Desired spacings: [%s] degrees\n', num2str(desired_spacings_deg, '%.0f '));
fprintf('Sum: %.1f degrees ', sum(desired_spacings_deg));
if abs(sum(desired_spacings_deg) - 360) < 0.1
    fprintf('✓ (valid)\n\n');
else
    fprintf('✗ (will be normalized)\n\n');
end

% Uncomment to run this scenario
% run_custom_spacing_simulation(desired_spacings);

fprintf('\n========================================\n');
fprintf('  EXAMPLE COMPLETE\n');
fprintf('========================================\n');

%% Helper function
function run_custom_spacing_simulation(desired_spacings)
    % Parameters
    N = length(desired_spacings);
    r_nominal = 2.5;
    target_pos = [0, 0];
    t_final = 35;
    
    % CBF parameters
    enable_cbf = true;
    d_safe = 0.4;
    d_target_min = 0.3;
    cbf_alpha = 1.0;
    
    % Check and adjust spacings
    [spacings_adj, N_adj, ~, adj_made] = check_spacing_feasibility(...
        desired_spacings, N);
    
    if adj_made
        fprintf('Spacings were adjusted\n');
    end
    
    N = N_adj;
    desired_spacings = spacings_adj;
    
    % Generate strategic radii
    [radii, var_pct] = calculate_radii_from_spacings(...
        desired_spacings, r_nominal, N);
    
    % Calculate control parameters
    epsilon = 0.1;
    c_init = 1 - epsilon;
    [c, kv, kw, opt_success] = optimize_control_parameters(...
        desired_spacings, radii, c_init);
    
    if ~opt_success
        fprintf('Note: Using approximate parameters\n');
    end
    
    % Agent-specific parameters
    delta = 1 ./ radii;
    a = zeros(1, N);
    for i = 1:N
        i_next = mod(i, N) + 1;
        a(i) = 1 - c * (radii(i) / radii(i_next));
    end
    
    % Initial conditions (random)
    x0 = zeros(3*N, 1);
    radius_init = 1.5 * max(radii);
    for i = 1:N
        angle_init = 2*pi*rand();
        x0(3*i-2) = target_pos(1) + radius_init * cos(angle_init);
        x0(3*i-1) = target_pos(2) + radius_init * sin(angle_init);
        x0(3*i) = 2*pi*rand();
    end
    
    % Package parameters
    params.N = N;
    params.radii = radii;
    params.a = a;
    params.delta = delta;
    params.kv = kv;
    params.kw = kw;
    params.target_pos = target_pos;
    params.desired_spacings = desired_spacings;
    params.enable_cbf = enable_cbf;
    params.d_safe = d_safe;
    params.d_target_min = d_target_min;
    params.cbf_alpha = cbf_alpha;
    
    % Simulate
    fprintf('\nRunning simulation...\n');
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    [t, x] = ode45(@(t,x) unicycle_dynamics_cbf(t, x, params), ...
        [0 t_final], x0, options);
    
    % Analyze results
    fprintf('\nFinal Results:\n');
    
    % Check spacings
    angles = zeros(N, 1);
    for i = 1:N
        xi = x(end, 3*i-2) - target_pos(1);
        yi = x(end, 3*i-1) - target_pos(2);
        angles(i) = atan2(yi, xi);
    end
    angles = mod(angles, 2*pi);
    [angles_sorted, idx] = sort(angles);
    actual_spacings = diff([angles_sorted; angles_sorted(1) + 2*pi]);
    
    fprintf('  Angular spacing comparison:\n');
    for i = 1:N
        agent_num = idx(i);
        fprintf('    Agent %d→%d: Desired %.1f°, Actual %.1f° (error %.1f°)\n', ...
            agent_num, mod(agent_num,N)+1, ...
            desired_spacings(agent_num)*180/pi, ...
            actual_spacings(i)*180/pi, ...
            abs(desired_spacings(agent_num)*180/pi - actual_spacings(i)*180/pi));
    end
    
    % Animate
    fprintf('\nCreating animation...\n');
    animate_circumnavigation(t, x, params, 1.5);
end
