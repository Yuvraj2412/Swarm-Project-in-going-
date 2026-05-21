%% Example: Agents on Different Orbits (Replicating Paper's Table 1)
% This example replicates the simulation from Section 4 of the paper
% with 6 agents on two different orbit radii.

clear; close all; clc;

%% Configuration - Matching Table 1 from the paper
N = 6;
radii = [2, 2, 2, 3, 3, 3];  % 3 agents on r=2, 3 agents on r=3
d = 2;                        % This gives psi_bar = 2*2*pi/6 = 2π/3

target_pos = [0, 0];
t_final = 35;

% Control gains from Table 1
kv = 2.750;
kw = 3.897;

% Initial conditions
initial_type = 'random';

% Animation
animate_realtime = true;
animation_speed = 1.0;

%% ========================================================================
%  DO NOT MODIFY BELOW
%  ========================================================================

fprintf('=== Replicating Paper Example (Table 1) ===\n');
fprintf('Configuration:\n');
fprintf('  N = %d agents\n', N);
fprintf('  Radii: r = [%s]\n', num2str(radii));
fprintf('  Formation parameter: d = %d\n', d);
fprintf('  Control gains: kv = %.3f, kw = %.3f\n', kv, kw);

% Validate
if length(radii) ~= N
    error('Length of radii must equal N');
end
if d < 1 || d > N-1
    error('d must be in the range {1, 2, ..., N-1}');
end

% Calculate parameters
psi_bar = 2*d*pi/N;
fprintf('  Desired angular spacing: psi_bar = %.4f rad (%.2f deg)\n', ...
    psi_bar, psi_bar*180/pi);

epsilon = 0.1;                    %actually its better to have smaller value of 'c'
if d == 1 || d == N-1
    c = 1 - epsilon;
elseif mod(N, 2) == 1 && (d == floor(N/2) || d == ceil(N/2))
    c = -1;
elseif mod(N, 2) == 0 && (d == N/2-1 || d == N/2+1)
    c = -1 + epsilon;
else
    c = 1 - epsilon;
    warning('Formation may not be asymptotically stable for this d value');
end

fprintf('  Control parameter: c = %.4f\n', c);

% Calculate agent-specific parameters
delta = 1 ./ radii;                             %calculates attraction to target for each agent
a = zeros(1, N);
for i = 1:N
    i_next = mod(i, N) + 1;
    a(i) = 1 - c * (radii(i) / radii(i_next));  %calculates attraction to neighbour for each agent
end

fprintf('\nAgent-specific parameters:\n');
fprintf('  i    ri    delta_i    a_i\n');
fprintf('  ---  ----  ---------  ------\n');
for i = 1:N
    fprintf('  %d    %.1f   %.4f     %.3f\n', i, radii(i), delta(i), a(i));
end

% Verify kv/kw ratio
kv_kw_theoretical = (1 - c*cos(psi_bar)) / abs(c*sin(psi_bar));
kv_kw_actual = kv / kw;
fprintf('\nControl gain ratio check:\n');
fprintf('  Theoretical kv/kw = %.4f\n', kv_kw_theoretical);
fprintf('  Actual kv/kw = %.4f\n', kv_kw_actual);

if abs(kv_kw_theoretical - kv_kw_actual) > 0.01
    fprintf('  WARNING: Ratio mismatch! Adjusting kw...\n');
    kw = kv / kv_kw_theoretical;                             %if the actual value is not ame as ratio of defined K_v/K_w then this formula will correct it
    fprintf('  New kw = %.4f\n', kw);     %kv controls the agent's linear velocity gain and kw control agent's angular velocity gain
end                                       %and we are interseted in correcting their values cause they are going to be used ahead in control law equation

% Initial conditions
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

% Simulate
fprintf('\nStarting simulation...\n');
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
tic;
[t, x] = ode45(@(t,x) unicycle_dynamics(t, x, params), [0 t_final], x0, options);
elapsed = toc;
fprintf('Simulation completed in %.2f seconds\n', elapsed);

% Calculate final metrics
fprintf('\nFinal Formation Metrics:\n');
final_distances = zeros(N, 1);
for i = 1:N
    xi = x(end, 3*i-2);
    yi = x(end, 3*i-1);
    final_distances(i) = sqrt((xi - target_pos(1))^2 + (yi - target_pos(2))^2);
    error_percent = abs(final_distances(i) - radii(i)) / radii(i) * 100;
    fprintf('  Agent %d: distance = %.4f m (desired: %.1f m, error: %.2f%%)\n', ...
        i, final_distances(i), radii(i), error_percent);
end

% Calculate angular spacing
angles = zeros(N, 1);
for i = 1:N
    xi = x(end, 3*i-2) - target_pos(1);
    yi = x(end, 3*i-1) - target_pos(2);
    angles(i) = atan2(yi, xi);
end
angles = mod(angles, 2*pi);
[angles_sorted, idx] = sort(angles);
angle_diffs = diff([angles_sorted; angles_sorted(1) + 2*pi]);
fprintf('\nAngular spacing (should be %.2f deg):\n', psi_bar*180/pi);
for i = 1:N
    fprintf('  Between agent %d and %d: %.2f deg\n', ...
        idx(i), idx(mod(i,N)+1), angle_diffs(i)*180/pi);
end

% Animate
if animate_realtime
    fprintf('\nCreating animation...\n');
    animate_circumnavigation(t, x, params, animation_speed);
end

fprintf('\nExample complete!\n');
