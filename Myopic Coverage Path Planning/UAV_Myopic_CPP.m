% =========================================================
%  UAV_Myopic_CPP.m
%  3-D Myopic Coverage Path Planning for UAV
%
%  Extension of:
%    "Risk-Aware Coverage Path Planning for Lunar Micro-Rovers
%     Leveraging Global and Local Environmental Data"
%    Santra et al., arXiv:2404.18721, 2024
%
%  Modifications (as per UAV Extension Document):
%    - State space  : 2D grid  -> 3D voxel grid
%    - Neighbours   : 8 cells  -> 26 voxels
%    - Terrain risk : DEM slope -> altitude-change + obstacle-clearance
%    - Energy model : wheel    -> thrust-based (momentum theory)
%    - Obs. avoid.  : Bug algo -> Artificial Potential Field (APF)
%    - SLAM         : HDL-Graph -> LIO-SAM  (not simulated; noted)
%
%  Run in MATLAB R2021a or later.
%  Call: >> UAV_Myopic_CPP
% =========================================================

function UAV_Myopic_CPP()

clc; clear; close all;
rng(42);   % fixed seed for reproducibility

fprintf('============================================\n');
fprintf('  UAV 3-D Myopic Coverage Path Planning\n');
fprintf('============================================\n\n');

% -------------------------------------------------------
%  1. PARAMETERS
% -------------------------------------------------------

%-- Environment
Gx = 30;  Gy = 30;  Gz = 30;   % voxel grid dimensions
OBS_RATIO  = 0.1;              % 5 % of voxels are obstacles
CELL_SIZE  = 1.0;               % metres per voxel edge

%-- UAV physical constants
m_uav  = 1.5;    % total mass          [kg]
g_acc  = 9.81;   % gravitational accel [m/s^2]
rho    = 1.225;  % air density         [kg/m^3]
A_rot  = 0.10;   % rotor disk area     [m^2]  (single equivalent rotor)
A_drag = 0.05;   % drag reference area [m^2]
Cd     = 0.50;   % drag coefficient
v_nom  = 1.0;    % nominal flight speed [m/s]
z_ref  = 3;      % reference/cruise altitude [voxels]

%-- Cost-function weights  (alpha + beta + gamma = 1)
alpha = 0.40;    % basic motion + visited penalty
beta  = 0.45;    % altitude-risk term
gam   = 0.15;    % energy-consumption term

%-- Altitude-risk sub-weights (Eq. mc_alt in proposal)
w1 = 1.0;   % penalise large altitude change
w2 = 2.0;   % penalise proximity to obstacles (1/d_obs)
w3 = 0.5;   % penalise deviation from cruise altitude

%-- Visited-cell penalty  (>= 1.1 per original paper)
MC_VIS = 1.5;

%-- APF parameters
D_SAFE    = 2;   % repulsion activation radius [voxels]
APF_SCALE = 0;  % weight blending APF into total cost

%-- Simulation control
COV_TARGET = 0.95;          % stop at 90 % coverage
MAX_ITER   = Gx*Gy*Gz * 5;  % safety cap

% -------------------------------------------------------
%  2. DERIVED QUANTITIES
% -------------------------------------------------------

% Hover power  –  Momentum-theory formula (CORRECT per proposal):
%   P_hover = (m*g)^(3/2) / sqrt(2 * rho * A)
P_hover = (m_uav * g_acc)^1.5 / sqrt(2 * rho * A_rot);

fprintf('UAV Parameters:\n');
fprintf('  Mass            : %.2f kg\n',  m_uav);
fprintf('  Hover Power     : %.2f W\n',  P_hover);
fprintf('  Reference alt.  : %d voxels\n', z_ref);
fprintf('  Grid            : %d x %d x %d\n\n', Gx, Gy, Gz);

% -------------------------------------------------------
%  3. ENVIRONMENT INITIALISATION
% -------------------------------------------------------

% Cell states: 0=Unknown  1=Free  2=Obstacle  3=Visited
map       = zeros(Gx, Gy, Gz, 'uint8');
visit_cnt = zeros(Gx, Gy, Gz);          % how many times each voxel visited

% Scatter random obstacles (keep the start region [1..3,1..3,:] clear)
n_obs  = floor(OBS_RATIO * Gx * Gy * Gz);
placed = 0;
while placed < n_obs
    ox = randi(Gx);  oy = randi(Gy);  oz = randi(Gz);
    if ox <= 3 && oy <= 3,  continue;  end   % protect start area
    if map(ox, oy, oz) == 0
        map(ox, oy, oz) = 2;
        placed = placed + 1;
    end
end

free_total = sum(map(:) ~= 2);   % denominator for coverage metric

% -------------------------------------------------------
%  4. INITIALISE UAV STATE
% -------------------------------------------------------

pos = [2, 2, z_ref];   % starting voxel  [x y z]
map(pos(1), pos(2), pos(3)) = 3;
visit_cnt(pos(1), pos(2), pos(3)) = 1;

path_log  = pos;        % Nx3 trajectory record
E_total   = 0;          % accumulated energy [J]
cov_hist  = [];         % coverage at each step
iter      = 0;
coverage  = 0;

fprintf('Starting myopic coverage loop...\n');

% -------------------------------------------------------
%  5. MAIN COVERAGE LOOP
% -------------------------------------------------------
while coverage < COV_TARGET && iter < MAX_ITER

    iter = iter + 1;

    % --- 5a. Sense: reveal all 26 neighbours (myopic LiDAR range = 1 voxel) ---
    nbrs = get_26_neighbors(pos, Gx, Gy, Gz);
    for k = 1:size(nbrs, 1)
        nx = nbrs(k,1);  ny = nbrs(k,2);  nz = nbrs(k,3);
        if map(nx, ny, nz) == 0
            map(nx, ny, nz) = 1;   % revealed as Free
        end
    end

    % --- 5b. Evaluate cost for each candidate voxel ---
    min_cost = inf;
    best     = [];

    for k = 1:size(nbrs, 1)
        nx = nbrs(k,1);  ny = nbrs(k,2);  nz = nbrs(k,3);

        if map(nx, ny, nz) == 2,  continue;  end   % skip obstacles

        % -- mc_static: base cost (from original paper, direction-based;
        %               simplified to 1 here as in the 3-D extension)
        mc_static = 1.0;

        % -- visited-cell penalty  (Vi = visit count, Eq.1 in paper)
        Vi = visit_cnt(nx, ny, nz);

        % -- nearest obstacle distance (evaluated at candidate voxel)
        d_obs = nearest_obs_dist([nx, ny, nz], map, Gx, Gy, Gz);

        % -- Altitude risk cost  (Eq. mc_alt in proposal)
        %    mc_alt = w1|Δz| + w2(1/d_obs) + w3|z_next - z_ref|
        dz      = nz - pos(3);
        mc_alt  = w1 * abs(dz) ...
                + w2 * (1 / max(d_obs, 0.5)) ...
                + w3 * abs(nz - z_ref);

        % -- Energy cost  (Eq. E_move in proposal, normalised to ~[0,1])
        %    E_move = P_hover*t + m*g*(z_next-z_curr)_+ + F_drag*d
        dist_m  = sqrt((nx-pos(1))^2 + (ny-pos(2))^2 + dz^2) * CELL_SIZE;
        t_step  = dist_m / v_nom;
        F_drag  = 0.5 * Cd * rho * A_drag * v_nom^2;
        E_step  = P_hover * t_step ...
                + m_uav * g_acc * max(dz, 0) * CELL_SIZE ...
                + F_drag * dist_m;
        mc_energy = E_step / 200;   % normalise (200 J ~ upper bound per hop)

        % -- APF obstacle-avoidance correction
        apf_score = apf_repulsive_score(pos, [nx,ny,nz], map, ...
                                        Gx, Gy, Gz, D_SAFE);

        % -- TOTAL UAV cost function  (Eq. mc in proposal)
        %    mc = alpha*(mc_static + MC_VIS*Vi) + beta*mc_alt + gam*mc_energy
        total_cost = alpha  * (mc_static + MC_VIS * Vi) ...
                   + beta   *  mc_alt  ...
                   + gam    *  mc_energy ...
                   + APF_SCALE * apf_score;

        if total_cost < min_cost
            min_cost = total_cost;
            best     = [nx, ny, nz];
        end
    end

    if isempty(best),  break;  end   % trapped – no reachable neighbour

    % --- 5c. Move UAV to best voxel ---
    dz_move    = best(3) - pos(3);
    dist_move  = sqrt(sum((best - pos).^2)) * CELL_SIZE;
    t_move     = dist_move / v_nom;
    F_drag_m   = 0.5 * Cd * rho * A_drag * v_nom^2;
    E_move     = P_hover * t_move ...
               + m_uav * g_acc * max(dz_move, 0) * CELL_SIZE ...
               + F_drag_m * dist_move;
    E_total    = E_total + E_move;

    pos = best;
    visit_cnt(pos(1), pos(2), pos(3)) = visit_cnt(pos(1), pos(2), pos(3)) + 1;
    if map(pos(1), pos(2), pos(3)) ~= 2
        map(pos(1), pos(2), pos(3)) = 3;
    end
    path_log(end+1, :) = pos;  %#ok<AGROW>

    % --- 5d. Update coverage metric ---
    visited  = sum(map(:) == 3);
    coverage = visited / free_total;
    cov_hist(end+1) = coverage;  %#ok<AGROW>

    if mod(iter, 200) == 0
        fprintf('  Iter %4d | Coverage %.1f%%\n', iter, coverage*100);
    end
end

% -------------------------------------------------------
%  6. PRINT RESULTS
% -------------------------------------------------------
fprintf('\n--- Simulation Results ---\n');
fprintf('  Total iterations     : %d\n',   iter);
fprintf('  Final coverage       : %.1f%%\n', coverage * 100);
fprintf('  Total energy used    : %.2f J\n', E_total);
fprintf('  Path length ratio    : %.3f\n',   iter / free_total);
fprintf('  (Optimal ratio = 1.0 per paper)\n\n');

% -------------------------------------------------------
%  7. VISUALISATION
% -------------------------------------------------------
fig = figure('Name', 'UAV Myopic CPP Simulation', ...
             'Color', 'white', ...
             'Position', [60, 60, 1400, 520]);

%-- Panel 1: 3-D trajectory
ax1 = subplot(1, 3, 1);
hold on; grid on; box on;

% Obstacles (black semi-transparent dots)
[ox, oy, oz] = ind2sub([Gx, Gy, Gz], find(map == 2));
scatter3(ox, oy, oz, 25, [0.15 0.15 0.15], 'filled', ...
         'MarkerFaceAlpha', 0.35);

% Visited voxels (light blue)
[vx, vy, vz] = ind2sub([Gx, Gy, Gz], find(map == 3));
scatter3(vx, vy, vz, 12, [0.5 0.8 1.0], 'filled', ...
         'MarkerFaceAlpha', 0.18);

% Path line
plot3(path_log(:,1), path_log(:,2), path_log(:,3), ...
      'b-', 'LineWidth', 1.4);

% Start / end markers
plot3(path_log(1,1),   path_log(1,2),   path_log(1,3), ...
      'go', 'MarkerSize', 10, 'MarkerFaceColor', [0.1 0.8 0.1], ...
      'LineWidth', 1.5);
plot3(path_log(end,1), path_log(end,2), path_log(end,3), ...
      'rs', 'MarkerSize', 10, 'MarkerFaceColor', [0.9 0.1 0.1], ...
      'LineWidth', 1.5);

% Reference altitude plane (transparent)
[px, py] = meshgrid(1:Gx, 1:Gy);
pz = z_ref * ones(size(px));
surf(px, py, pz, 'FaceColor', [0.2 0.9 0.2], ...
     'FaceAlpha', 0.07, 'EdgeColor', 'none');

xlabel('X [voxel]'); ylabel('Y [voxel]'); zlabel('Z [voxel]');
title(sprintf('3D UAV Path\nCoverage: %.1f%%  |  Steps: %d', ...
              coverage*100, iter), 'FontSize', 10);
legend({'Obstacles','Visited','Path','Start','End','z_{ref} plane'}, ...
       'Location','best','FontSize',7);
xlim([1 Gx]); ylim([1 Gy]); zlim([1 Gz]);
view(42, 28);

%-- Panel 2: Coverage progress
subplot(1, 3, 2);
plot(1:length(cov_hist), cov_hist*100, 'b-', 'LineWidth', 1.8);
yline(COV_TARGET*100, 'r--', 'LineWidth', 1.5, ...
      'Label', sprintf('Target %.0f%%', COV_TARGET*100));
xlabel('Iteration', 'FontSize', 11);
ylabel('Coverage [%]', 'FontSize', 11);
title('Coverage Progress', 'FontSize', 11);
grid on; box on;
ylim([0, 105]);

%-- Panel 3: Altitude profile
subplot(1, 3, 3);
steps = 1:size(path_log, 1);
plot(steps, path_log(:,3), 'm-', 'LineWidth', 1.6);
yline(z_ref, 'g--', 'LineWidth', 1.5, 'Label', 'z_{ref}');
xlabel('Step', 'FontSize', 11);
ylabel('Altitude [voxel]', 'FontSize', 11);
title('Altitude Profile (UAV)', 'FontSize', 11);
grid on; box on;
ylim([0, Gz+1]);

sgtitle('UAV Myopic Coverage Path Planning  —  3D Simulation', ...
        'FontSize', 13, 'FontWeight', 'bold');

fprintf('Figures rendered. Simulation complete.\n');

end   % UAV_Myopic_CPP()


% ==========================================================
%  HELPER FUNCTIONS
% ==========================================================

function nbrs = get_26_neighbors(pos, Gx, Gy, Gz)
% Returns all valid 26-connected voxel neighbours of pos.
    nbrs  = zeros(26, 3);
    count = 0;
    for dx = -1:1
        for dy = -1:1
            for dz = -1:1
                if dx==0 && dy==0 && dz==0,  continue;  end
                nx = pos(1)+dx;  ny = pos(2)+dy;  nz = pos(3)+dz;
                if nx>=1 && nx<=Gx && ny>=1 && ny<=Gy && nz>=1 && nz<=Gz
                    count = count + 1;
                    nbrs(count, :) = [nx, ny, nz];
                end
            end
        end
    end
    nbrs = nbrs(1:count, :);
end


function d = nearest_obs_dist(pos, map, Gx, Gy, Gz)
% Euclidean distance to nearest obstacle within radius 4 voxels.
    d = inf;
    R = 4;
    for dx = -R:R
        for dy = -R:R
            for dz = -R:R
                nx = pos(1)+dx;  ny = pos(2)+dy;  nz = pos(3)+dz;
                if nx<1||nx>Gx||ny<1||ny>Gy||nz<1||nz>Gz,  continue;  end
                if map(nx, ny, nz) == 2
                    dd = sqrt(dx^2 + dy^2 + dz^2);
                    if dd < d,  d = dd;  end
                end
            end
        end
    end
end


function score = apf_repulsive_score(pos, target, map, Gx, Gy, Gz, D_safe)
% APF repulsive potential score for moving pos -> target.
% Higher score = target voxel is in a more obstacle-congested direction.
    score  = 0;
    dir    = target - pos;
    d_norm = norm(dir);
    if d_norm < 1e-9,  return;  end
    dir_u  = dir / d_norm;

    for dx = -2:2
        for dy = -2:2
            for dz = -2:2
                nx = pos(1)+dx;  ny = pos(2)+dy;  nz = pos(3)+dz;
                if nx<1||nx>Gx||ny<1||ny>Gy||nz<1||nz>Gz,  continue;  end
                if map(nx, ny, nz) == 2
                    d = sqrt(dx^2 + dy^2 + dz^2);
                    if d < D_safe && d > 0
                        % Repulsive magnitude from APF theory
                        mag = (1/d - 1/D_safe)^2;
                        % Direction away from obstacle projected on movement
                        obs_dir = -[dx, dy, dz] / d;
                        proj    = dot(dir_u, -obs_dir);   % +ve if heading toward obs
                        score   = score + mag * max(proj, 0);
                    end
                end
            end
        end
    end
end
