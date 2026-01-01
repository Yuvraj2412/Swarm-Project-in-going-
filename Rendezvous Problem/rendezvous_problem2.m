% - Global PID boosts repulsion (active from t=0)
% - Per-drone effective K_rep is recorded and plotted (no on-drone text labels)
% - Randomized "near" initial placement algorithm (polar sampling)

clc; clear; close all;

%% ===================== Parameters =====================
N_DRONES = 4;
TARGET_POS = [100; 100];

T_SIMULATION = 30;    % seconds
DT = 0.05;
num_steps = ceil(T_SIMULATION / DT);

V_MAX = 20.0;
D_SAFE = 30.0;        % safe distance used for repulsion
R_ARRIVAL = 20.0;     % periphery radius (repulsion disabled inside)
R_GOAL = 0.5;         % final tight arrival radius

MASS = 1.0;
K_ATT = 1.2;          % velocity-tracking
K_REP_BASE = 5.0;     % baseline repulsion
K_REP_MAX = 800.0;    % hard cap

K_POS = 6.0;          % inside-periphery P gain
K_DAMP = 2.0;         % inside-periphery damping

%% PID (global)
TRIGGER_DIST = 60.0;   % how far outside R_ARRIVAL PID starts acting
ACTIVATION_DIST = R_ARRIVAL + TRIGGER_DIST;
pid.Kp = 80.0;
pid.Ki = 6.0;
pid.Kd = 12.0;
pid.integral = 0.0;
pid.prev_error = 0.0;
pid.output = 0.0;

%% ================ Random "near" placement algorithm ================
% Place drones randomly near each other but outside the periphery.
% Algorithm: choose a cluster center offset from target, then sample
% random angle and radius within [r_min, r_max] and map to Cartesian.
% If sample ends up inside R_ARRIVAL, push it out radially to (R_ARRIVAL + margin).

rng('shuffle');     % different each run; use rng(2) for reproducible
cluster_center = TARGET_POS + [-55; -30]; % cluster center (tune as needed)

% ring parameters (min and max distance from cluster_center)
r_min = 2.0;      % min radial distance from center (close cluster)
r_max = 10.0;     % max radial distance from center (spread)
margin_outside_periphery = 3.0; % if sample falls inside periphery, push outward by this

pos_start = zeros(2, N_DRONES);
for i = 1:N_DRONES
    theta = 2*pi*rand();                % random angle
    r = r_min + (r_max - r_min) * rand(); % random radius between r_min and r_max
    sample = cluster_center + [r*cos(theta); r*sin(theta)];
    % ensure sample is outside periphery (if not, push radially away from target)
    d_to_target = norm(sample - TARGET_POS);
    if d_to_target <= R_ARRIVAL + 0.5
        dir = (sample - TARGET_POS);
        if norm(dir) < 1e-6
            dir = [1;0];
        end
        dir = dir / norm(dir);
        sample = TARGET_POS + dir * (R_ARRIVAL + margin_outside_periphery);
    end
    pos_start(:,i) = sample;
end
% (Optional) jitter tiny random offsets to avoid perfect symmetry:
pos_start = pos_start + 0.1 * randn(size(pos_start));

%% ================ Initialize states and logging =================
pos_current = pos_start;
vel_current = zeros(2, N_DRONES);

pos_history = zeros(2, N_DRONES, num_steps);
pos_history(:,:,1) = pos_start;
arrival_times = inf(1, N_DRONES);
has_arrived = false(1, N_DRONES);
is_repelling = false(1, N_DRONES);

% Record per-drone effective repulsion gain over time:
Krep_history = zeros(N_DRONES, num_steps);   % K_rep effective for each drone each step
Krep_global_history = zeros(1, num_steps);   % global additive part for debugging
time_log = (0:(num_steps-1)) * DT;

% Rendezvous planning -> ideal velocities
dist_to_target = vecnorm(TARGET_POS - pos_start);
T_min_i = dist_to_target / V_MAX;
T_rendezvous = max(T_min_i);
vel_ideal = (TARGET_POS - pos_start) / T_rendezvous;

%% ================ Simulation loop ===========================
figure('Name','UAV Rendezvous (no on-drone labels; K_{rep} plots below)','NumberTitle','off');
hFig = gcf;
step_end = 1;

for step = 2:num_steps
    current_time = (step-1) * DT;

    % --- PID global input ---
    dists = vecnorm(TARGET_POS - pos_current);
    min_dist_to_target = min(dists);
    pid_error = max(0, ACTIVATION_DIST - min_dist_to_target);
    pid.integral = pid.integral + pid_error * DT;
    deriv = (pid_error - pid.prev_error) / DT;
    pid.output = pid.Kp * pid_error + pid.Ki * pid.integral + pid.Kd * deriv;
    pid.prev_error = pid_error;
    K_rep_global_add = pid.output;   % additive global contribution

    % Precompute pair distances and vectors
    pair_vec = zeros(2, N_DRONES, N_DRONES);
    pair_dist = inf(N_DRONES, N_DRONES);
    for i = 1:N_DRONES
        for j = i+1:N_DRONES
            v = pos_current(:,i) - pos_current(:,j);
            dij = norm(v);
            pair_vec(:,i,j) = v;
            pair_vec(:,j,i) = -v;
            pair_dist(i,j) = dij;
            pair_dist(j,i) = dij;
        end
    end

    % reset per-step Krep_effective accumulator
    Krep_effective = zeros(1, N_DRONES);

    % compute forces
    a_total = zeros(2, N_DRONES);
    for i = 1:N_DRONES
        if has_arrived(i), continue; end
        is_repelling(i) = false;
        dist_to_target_i = norm(TARGET_POS - pos_current(:,i));

        if dist_to_target_i > R_ARRIVAL
            % velocity tracking force
            force_goal = K_ATT * (vel_ideal(:,i) - vel_current(:,i));
            force_repel = zeros(2,1);

            % sum pairwise repulsions, compute per-pair K_rep_ij and accumulate Krep_effective
            contributing_pairs = 0;
            Ksum = 0;
            for j = 1:N_DRONES
                if i==j || has_arrived(j), continue; end
                dij = pair_dist(i,j);
                if dij < 1e-6
                    continue;
                end
                % weight: scales global add according to closeness to pair
                w = max(0, 1 - (dij / D_SAFE));  % 1 when overlapping, 0 at D_SAFE and beyond
                K_rep_ij = K_REP_BASE + K_rep_global_add * w;
                K_rep_ij = min(K_rep_ij, K_REP_MAX);

                if dij < D_SAFE
                    F_mag = K_rep_ij * (1/dij - 1/D_SAFE) * (1/dij^2);
                    force_repel = force_repel + F_mag * (pair_vec(:,i,j) / dij);
                    is_repelling(i) = true;
                end
                % accumulate for effective per-drone K
                Ksum = Ksum + K_rep_ij;
                contributing_pairs = contributing_pairs + 1;
            end
            if contributing_pairs > 0
                Krep_effective(i) = Ksum / contributing_pairs; % average pair K for drone i
            else
                Krep_effective(i) = K_REP_BASE; % fallback
            end

            a_total(:,i) = (force_goal + force_repel) / MASS;
        else
            % inside periphery: disable repulsion; use PD to move to target
            pos_err = TARGET_POS - pos_current(:,i);
            force_pos = K_POS * pos_err + K_DAMP * (-vel_current(:,i));
            a_total(:,i) = force_pos / MASS;
            Krep_effective(i) = 0; % repulsion disabled inside
        end
    end

    % store per-drone Krep and global additive part
    Krep_history(:,step) = Krep_effective';
    Krep_global_history(step) = K_rep_global_add;

    % integrate dynamics
    for i = 1:N_DRONES
        if has_arrived(i)
            vel_current(:,i) = [0;0];
            pos_history(:,i,step) = pos_current(:,i);
            continue;
        end
        vel_current(:,i) = vel_current(:,i) + a_total(:,i) * DT;
        vn = norm(vel_current(:,i));
        if vn > V_MAX
            vel_current(:,i) = vel_current(:,i) * (V_MAX / vn);
        end
        pos_current(:,i) = pos_current(:,i) + vel_current(:,i) * DT;
        pos_history(:,i,step) = pos_current(:,i);

        % arrival check with tight R_GOAL -> snap to target
        if norm(TARGET_POS - pos_current(:,i)) <= R_GOAL
            if ~has_arrived(i)
                arrival_times(i) = current_time;
                has_arrived(i) = true;
                pos_current(:,i) = TARGET_POS;
                vel_current(:,i) = [0;0];
                pos_history(:,i,step) = pos_current(:,i);
            end
        end
    end

    % --- Visualization (no per-drone text; just markers & shapes) ---
    if mod(step,4) == 0 || all(has_arrived)
        if ~isvalid(hFig), break; end
        clf; hold on; grid on; box on;

        % plot drones (normal/repelling)
        normal_mask = ~is_repelling;
        if any(normal_mask)
            plot(pos_current(1,normal_mask), pos_current(2,normal_mask), 'bo', 'MarkerFaceColor','b','MarkerSize',8);
        end
        if any(is_repelling)
            plot(pos_current(1,is_repelling), pos_current(2,is_repelling), 'ro', 'MarkerFaceColor','r','MarkerSize',8);
        end

        % plot start positions
        plot(pos_start(1,:), pos_start(2,:), 'kx', 'MarkerSize',8);

        % plot target and periphery
        plot(TARGET_POS(1), TARGET_POS(2), 'rX', 'MarkerSize',12, 'LineWidth',2);
        rectangle('Position',[TARGET_POS(1)-R_ARRIVAL, TARGET_POS(2)-R_ARRIVAL, R_ARRIVAL*2, R_ARRIVAL*2], ...
                  'Curvature',[1 1], 'EdgeColor',[1 0 0 0.25], 'LineStyle','--');

        % safety circles (visual)
        for ii = 1:N_DRONES
            rectangle('Position',[pos_current(1,ii)-D_SAFE/2, pos_current(2,ii)-D_SAFE/2, D_SAFE, D_SAFE], ...
                      'Curvature',[1 1], 'EdgeColor',[0.6 0.6 0.6 0.12]);
        end

        title(sprintf('t=%.2f s | GlobalAdd=%.1f | K_base=%.1f', current_time, K_rep_global_add, K_REP_BASE));
        xlabel('X (m)'); ylabel('Y (m)');
        axis equal;
        mins = min([pos_start, pos_current, TARGET_POS], [], 2);
        maxs = max([pos_start, pos_current, TARGET_POS], [], 2);
        pad = 20;
        axis([mins(1)-pad, maxs(1)+pad, mins(2)-pad, maxs(2)+pad]);
        legend({'UAV (normal)','UAV (repelling)','Start'}, 'Location','bestoutside');
        drawnow;
    end

    step_end = step;
    if all(has_arrived)
        fprintf('All arrived at t=%.2f s\n', current_time);
        break;
    end
end

%% ================ Post simulation plotting: paths + Krep history ============
% Trim logged time to actual steps used
t_used = time_log(1:step_end);
Krep_plot = Krep_history(:, 1:step_end);
Krep_global_plot = Krep_global_history(1:step_end);

% Create a single figure with two subplots: top -> full paths, bottom -> Krep time-series
figure('Name','Paths and K_{rep} time-series','NumberTitle','off','Position',[200 200 900 600]);
subplot(2,1,1);
hold on; grid on; box on;
colors = lines(N_DRONES);
for i = 1:N_DRONES
    path_i = squeeze(pos_history(:,i,1:step_end));
    plot(path_i(1,:), path_i(2,:), 'Color', colors(i,:), 'LineWidth', 2);
    plot(pos_start(1,i), pos_start(2,i), 'o', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:));
    text(pos_start(1,i)+1, pos_start(2,i)+1, sprintf('UAV %d', i), 'FontSize',8);
end
plot(TARGET_POS(1), TARGET_POS(2), 'rX', 'MarkerSize',12, 'LineWidth',2);
rectangle('Position',[TARGET_POS(1)-R_ARRIVAL, TARGET_POS(2)-R_ARRIVAL, R_ARRIVAL*2, R_ARRIVAL*2], ...
          'Curvature',[1 1], 'EdgeColor',[1 0 0 0.25], 'LineStyle','--');
axis equal; title('Full Drone Paths'); xlabel('X (m)'); ylabel('Y (m)');

% Bottom: K_rep for each drone over time
subplot(2,1,2);
hold on; grid on; box on;
for i = 1:N_DRONES
    plot(t_used, Krep_plot(i,:), 'Color', colors(i,:), 'LineWidth', 1.6);
end
plot(t_used, Krep_global_plot, 'k--', 'LineWidth', 1); % global additive shown as dashed black
legend_labels = arrayfun(@(i) sprintf('UAV %d K_{rep}', i), 1:N_DRONES, 'UniformOutput', false);
legend_labels{end+1} = 'Global add (PID)';
legend(legend_labels, 'Location','northeast');
xlabel('Time (s)'); ylabel('K_{rep} (effective)'); title('Per-drone effective repulsion gains over time');

%% Print final pairwise K (if needed)
disp('Final per-drone effective K_rep values:');
disp(Krep_plot(:,end)');