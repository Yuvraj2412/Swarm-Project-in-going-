%% Example: All Agents on Same Orbit
% This example demonstrates uniform circumnavigation with all agents
% circling on the same orbit radius.

clear; close all; clc;

%% Configuration
N = 8;                       % 8 agents
radii = 2.5 * ones(1, N);   % All on radius 2.5
d = 1;                       % Clockwise formation
target_pos = [0, 0];         % Target at origin
t_final = 25;                % 25 seconds simulation

% Control gains
kv = 2.5;
kw = 3.5;

% Initial conditions
initial_type = 'random';

% Animation
animate_realtime = true;
animation_speed = 1.5;  % 1.5x speed

%% ========================================================================
%  DO NOT MODIFY BELOW (same as main script)
%  ========================================================================

% Validate
if length(radii) ~= N
    error('Length of radii must equal N');
end

% Calculate parameters
psi_bar = 2*d*pi/N;
epsilon = 0.1;

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

delta = 1 ./ radii;
a = zeros(1, N);
for i = 1:N
    i_next = mod(i, N) + 1;
    a(i) = 1 - c * (radii(i) / radii(i_next));
end

% Verify kv/kw ratio
kv_kw_theoretical = (1 - c*cos(psi_bar)) / abs(c*sin(psi_bar));
kv_kw_actual = kv / kw;
if abs(kv_kw_theoretical - kv_kw_actual) > 0.01
    fprintf('Adjusting kw to match theoretical ratio...\n');
    kw = kv / kv_kw_theoretical;
end

% Initial conditions
x0 = zeros(3*N, 1);
if strcmp(initial_type, 'random')
    radius_init = 1.5 * max(radii);
    for i = 1:N
        angle_init = 2*pi*rand();
        x0(3*i-2) = target_pos(1) + radius_init * cos(angle_init);
        x0(3*i-1) = target_pos(2) + radius_init * sin(angle_init);
        x0(3*i) = 2*pi*rand();
    end
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
fprintf('Starting simulation: All %d agents on same orbit (r=%.1f)...\n', N, radii(1));
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
[t, x] = ode45(@(t,x) unicycle_dynamics(t, x, params), [0 t_final], x0, options);

% Animate
if animate_realtime
    animate_circumnavigation(t, x, params, animation_speed);
end

fprintf('Done!\n');
