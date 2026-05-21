% =========================================================
%  UAV_Myopic_CPP.m  –  v3
%  3-D Myopic Coverage Path Planning for UAV
%
%  Based on:
%    "Risk-Aware Coverage Path Planning for Lunar Micro-Rovers"
%    Santra et al., arXiv:2404.18721, 2024
%
%  Full cost function (mirrors paper structure exactly):
%    mc = alpha*(mc_static + MC_VIS*Vi)   [LOCAL:  basic + revisit penalty]
%       + beta *  mc_alt                  [LOCAL:  altitude risk, real-time]
%       + gamma * mc_energy               [LOCAL:  thrust energy, real-time]
%       + delta * mc_global               [GLOBAL: pre-computed risk map]
%
%  Paper equivalent:
%    mc = alpha*(mc_static + mc_visited*Vi) + beta*mc_DEM
%    where mc_DEM is the pre-computed global terrain cost.
%    Here mc_global plays the same role for UAV.
%
%  Figures produced:
%    Figure 1  –  LIVE ANIMATION (dark theme, 2-panel: 3D + top-down)
%    Figure 2  –  3D Final Path  (time-coded rainbow trail)
%    Figure 3  –  Coverage Progress (with milestone markers)
%    Figure 4  –  Altitude Profile (colour-coded by height)
%    Figure 5  –  Global Risk Map  (pre-computed, shown before run)
%
%  Run: >> UAV_Myopic_CPP
%  MATLAB R2021a+
% =========================================================

function UAV_Myopic_CPP()

clc; clear; close all;
rng(42);

fprintf('============================================\n');
fprintf('  UAV 3-D Myopic Coverage Path Planning v3 \n');
fprintf('============================================\n\n');

% ==========================================================
%  SECTION 1 — PARAMETERS
%  All tunable knobs in one place.
% ==========================================================

% --- Environment ---
Gx = 20;  Gy = 20;  Gz = 10;   % voxel grid (x, y, z)
OBS_RATIO = 0.05;               % fraction of voxels that are obstacles
CELL_SIZE = 1.0;                % metres per voxel edge

% --- UAV Physics ---
% These are REAL physical constants for a ~DJI-Phantom-class drone
m_uav  = 1.5;    % total mass [kg]
g_acc  = 9.81;   % gravity [m/s^2]
rho    = 1.225;  % air density at sea level [kg/m^3]
A_rot  = 0.10;   % equivalent rotor disk area [m^2]
A_drag = 0.05;   % frontal drag area [m^2]
Cd     = 0.50;   % drag coefficient (bluff body approx)
v_nom  = 1.0;    % nominal cruise speed [m/s]
z_ref  = 5;      % cruise/reference altitude [voxels]

% --- Cost-Function Weights ---
% These MUST sum to 1.0 (like alpha+beta=1 in original paper,
% but we now have 4 terms).
%
%  alpha  = weight on local myopic cost  (revisit avoidance)
%  beta   = weight on altitude risk      (real-time, local)
%  gamma  = weight on energy cost        (real-time, local)
%  delta  = weight on GLOBAL risk map    (pre-computed, like DEM in paper)
%
alpha = 0.35;   % basic motion + visited penalty
beta  = 0.25;   % altitude risk
gamma = 0.20;   % energy consumption
delta = 0.20;   % global pre-computed risk  <-- NEW, mirrors mc_DEM in paper
% NOTE: alpha + beta + gamma + delta = 1.0  ✓

% --- Altitude-Risk Sub-Weights (inside beta term) ---
w1 = 1.0;   % penalise large altitude changes
w2 = 2.0;   % penalise proximity to obstacles (1/d_obs)
w3 = 0.5;   % penalise deviation from cruise altitude z_ref

% --- Visited-Cell Penalty ---
% Must be >= 1.1 (per original paper) to avoid infinite loops.
% Higher = stronger push toward unvisited areas.
MC_VIS = 1.5;

% --- APF Parameters ---
D_SAFE    = 2.5;   % APF repulsion activation radius [voxels]
APF_SCALE = 0.04;  % APF blend into total cost

% --- Global Risk Map Parameters ---
N_HAZARDS = 6;     % number of pre-known hazard zones (Gaussian bumps)
HAZARD_STR = 3.0;  % peak risk value at hazard centre

% --- Simulation Control ---
COV_TARGET = 0.90;          % stop when this fraction of free voxels visited
MAX_ITER   = Gx*Gy*Gz * 5;  % hard cap (handles the "gets stuck" case)

% --- Animation Control ---
ANIM_SKIP = 4;    % animate every Nth step. Increase to speed up.
ROTOR_SPD = 30;   % rotor spin speed [degrees/frame]

% ==========================================================
%  SECTION 2 — DERIVED PHYSICAL QUANTITIES
%
%  P_hover: Momentum theory formula (from UAV extension doc).
%  This is the MINIMUM power to stay airborne — constant baseline
%  even when the drone isn't moving horizontally.
%
%  A stationary hover for t seconds costs: P_hover * t  Joules.
%  Moving forward ADDS drag energy ON TOP of this hover cost.
%  Climbing ADDS gravitational PE on top of both.
%  → These three terms are physically independent.
% ==========================================================

P_hover = (m_uav * g_acc)^1.5 / sqrt(2 * rho * A_rot);
F_drag  = 0.5 * Cd * rho * A_drag * v_nom^2;  % constant at v_nom

fprintf('UAV Physical Parameters:\n');
fprintf('  Mass              : %.2f kg\n',  m_uav);
fprintf('  Hover Power       : %.2f W   (energy/time just staying airborne)\n', P_hover);
fprintf('  Drag Force @ v_nom: %.4f N  (extra resistance when moving)\n', F_drag);
fprintf('  Reference alt.    : %d voxels\n', z_ref);
fprintf('  Grid              : %d x %d x %d  (%d total voxels)\n\n', ...
        Gx, Gy, Gz, Gx*Gy*Gz);

% ==========================================================
%  SECTION 3 — ENVIRONMENT SETUP
%
%  Cell states:
%    0 = Unknown  (UAV has not sensed this yet — starts here)
%    1 = Free     (sensed by LiDAR, no obstacle, not yet visited)
%    2 = Obstacle (sensed, blocked — UAV cannot enter)
%    3 = Visited  (UAV has physically occupied this voxel)
%
%  This mirrors Table/Section III of the paper exactly.
% ==========================================================

map       = zeros(Gx, Gy, Gz, 'uint8');
visit_cnt = zeros(Gx, Gy, Gz);

% Place random obstacles (protect start corner [1..3, 1..3])
n_obs  = floor(OBS_RATIO * Gx * Gy * Gz);
placed = 0;
while placed < n_obs
    ox = randi(Gx);  oy = randi(Gy);  oz = randi(Gz);
    if ox <= 3 && oy <= 3,  continue;  end
    if map(ox, oy, oz) == 0
        map(ox, oy, oz) = 2;
        placed = placed + 1;
    end
end

free_total = sum(map(:) ~= 2);
fprintf('Environment: %d free voxels, %d obstacles (%.1f%%)\n\n', ...
        free_total, n_obs, OBS_RATIO*100);

% ==========================================================
%  SECTION 4 — GLOBAL RISK MAP  (pre-computed, like DEM in paper)
%
%  In the original paper, mc_DEM is computed BEFORE the mission
%  starts using NASA satellite elevation data. The rover then
%  uses this pre-computed slope cost during navigation.
%
%  For UAV, we simulate the equivalent: a pre-scanned global
%  risk map with known hazard zones (e.g. from prior aerial
%  survey, restricted airspace, or known obstacle clusters).
%
%  Formula: mc_global(x,y,z) = sum of Gaussian bumps centred
%  at hazard locations. Pre-computed once, used every step.
%
%  This is the GLOBAL term that was MISSING from v1 and v2.
% ==========================================================

global_risk = zeros(Gx, Gy, Gz);

% Random hazard centres (simulate pre-known risky zones)
rng(7);   % different seed so hazards don't overlap with obstacles
hazard_centres = [randi([4,Gx],N_HAZARDS,1), ...
                  randi([4,Gy],N_HAZARDS,1), ...
                  randi([1,Gz],N_HAZARDS,1)];
rng(42);  % restore main seed

% Build global risk map as sum of Gaussians
for i = 1:Gx
    for j = 1:Gy
        for k = 1:Gz
            r = 0;
            for h = 1:N_HAZARDS
                dx = i - hazard_centres(h,1);
                dy = j - hazard_centres(h,2);
                dz = k - hazard_centres(h,3);
                dist2 = dx^2 + dy^2 + dz^2;
                r = r + HAZARD_STR * exp(-dist2 / 8.0);
            end
            global_risk(i,j,k) = r;
        end
    end
end

% Normalise to [0, 1] so it blends cleanly with other cost terms
global_risk = global_risk / max(global_risk(:));

fprintf('Global risk map computed (%d hazard zones).\n\n', N_HAZARDS);

% ==========================================================
%  SECTION 5 — FIGURE 5: GLOBAL RISK MAP  (shown before run)
%
%  Shows what pre-known information the UAV has before it
%  starts flying — equivalent to the DEM shown in Fig.4(a)
%  of the original paper.
% ==========================================================

figure('Name','Figure 5 - Global Risk Map (Pre-Computed)', ...
       'Color','white','Position',[400 300 700 500]);

% Show middle slice (z = z_ref layer)
mid_slice = squeeze(global_risk(:,:,z_ref))';
imagesc(mid_slice);
colormap(hot); colorbar;
title(sprintf('Global Risk Map  (z = %d slice, the cruise altitude layer)', z_ref), ...
      'FontSize', 12, 'FontWeight', 'bold');
xlabel('X [voxel]','FontSize',11);
ylabel('Y [voxel]','FontSize',11);
set(gca,'YDir','normal');
hold on;

% Mark hazard centres that fall on this slice (within ±1)
for h = 1:N_HAZARDS
    if abs(hazard_centres(h,3) - z_ref) <= 1
        plot(hazard_centres(h,1), hazard_centres(h,2), 'c*', ...
             'MarkerSize',14,'LineWidth',2);
    end
end
legend({'Hazard centres (on this layer)'},'Location','best','FontSize',9);

% Also show obstacles as black X
[ox_s, oy_s] = ind2sub([Gx,Gy], find(map(:,:,z_ref) == 2));
if ~isempty(ox_s)
    plot(ox_s, oy_s, 'bx', 'MarkerSize', 8, 'LineWidth', 1.5);
end
drawnow;

% ==========================================================
%  SECTION 6 — UAV INITIALISATION
% ==========================================================

pos = [2, 2, z_ref];
map(pos(1), pos(2), pos(3)) = 3;
visit_cnt(pos(1), pos(2), pos(3)) = 1;

path_log = pos;
E_total  = 0;
cov_hist = [];
iter     = 0;
coverage = 0;

% Pre-allocate cost component log for analysis
cost_log_alt    = [];
cost_log_energy = [];
cost_log_global = [];

% ==========================================================
%  SECTION 7 — FIGURE 1: LIVE ANIMATION SETUP
%
%  Two-panel dark-theme window:
%    Left  (large) : 3D animated voxel world
%    Right (small) : 2D top-down coverage heatmap
%
%  Visual legend for 3D panel:
%    Red   cubes    = Obstacles
%    Yellow dots    = Free (sensed but NOT yet visited)
%    Green  dots    = Visited (size ∝ visit count)
%    Cyan   ring    = Current sensing sphere (26 neighbours)
%    Rainbow line   = Flight path (blue=early, red=late)
%    White/gold UAV = Current drone position
%    Magenta plane  = Cruise altitude z_ref
% ==========================================================

fig_anim = figure('Name','Figure 1 - UAV Live Animation', ...
                  'Color',[0.04 0.04 0.10], ...
                  'Position',[40, 40, 1200, 680]);

% --- LEFT: 3D animated axes ---
ax3d = subplot('Position',[0.02 0.06 0.62 0.88]);
set(ax3d,'Color',[0.04 0.04 0.10], ...
         'XColor',[0.6 0.6 0.6],'YColor',[0.6 0.6 0.6],'ZColor',[0.6 0.6 0.6], ...
         'GridColor',[0.2 0.2 0.2],'GridAlpha',0.5,'FontSize',8);
hold(ax3d,'on');  grid(ax3d,'on');  box(ax3d,'on');

% Draw obstacles once (static red squares)
[ox_v,oy_v,oz_v] = ind2sub([Gx,Gy,Gz], find(map==2));
h_obstacles = scatter3(ax3d, ox_v, oy_v, oz_v, 55, ...
                       [0.95 0.25 0.15], 's', 'filled', ...
                       'MarkerFaceAlpha', 0.85);

% Draw global hazard zone centres (magenta diamonds)
for h = 1:N_HAZARDS
    plot3(ax3d, hazard_centres(h,1), hazard_centres(h,2), hazard_centres(h,3), ...
          'd','Color',[1 0.2 1],'MarkerSize',10, ...
          'MarkerFaceColor',[1 0.2 1],'LineWidth',1.5);
end

% Cruise altitude reference plane (faint green)
[pp_x,pp_y] = meshgrid(1:Gx, 1:Gy);
surf(ax3d, pp_x, pp_y, z_ref*ones(size(pp_x)), ...
     'FaceColor',[0.2 0.9 0.4],'FaceAlpha',0.05,'EdgeColor','none');

% Free (sensed-not-visited) voxels — yellow dots, starts empty
h_free = scatter3(ax3d, nan, nan, nan, 12, [1.0 0.9 0.2], '.', ...
                  'MarkerFaceAlpha', 0.55);

% Visited voxels — green dots, starts empty
h_visited = scatter3(ax3d, nan, nan, nan, 18, [0.2 0.9 0.4], 'o', 'filled', ...
                     'MarkerFaceAlpha', 0.35);

% Sensing sphere highlight — cyan ring at current position
h_sense = scatter3(ax3d, nan, nan, nan, 45, [0.0 0.9 0.9], 'o', ...
                   'MarkerFaceAlpha', 0.10, 'LineWidth', 0.8);

% Flight trail — starts as single point, grows with time
% We'll update it segment-by-segment with colour = time
h_trail = plot3(ax3d, pos(1), pos(2), pos(3), ...
                '-', 'Color',[0.4 0.5 1.0 0.6], 'LineWidth', 1.3);

% Start marker (permanent green star)
plot3(ax3d, pos(1), pos(2), pos(3), 'p', ...
      'Color',[0.2 1.0 0.4],'MarkerSize',14, ...
      'MarkerFaceColor',[0.2 1.0 0.4],'LineWidth',1.5);

% UAV body handles
[uav_h, rotor_h] = draw_uav_3d(ax3d, pos);

% HUD text
h_hud = text(ax3d, 1.2, 1.2, Gz+0.6, 'Initialising...', ...
             'Color',[1.0 1.0 0.4],'FontSize',10, ...
             'FontWeight','bold','FontName','Courier');

% Axes formatting
xlabel(ax3d,'X','Color',[0.7 0.7 0.7],'FontSize',9);
ylabel(ax3d,'Y','Color',[0.7 0.7 0.7],'FontSize',9);
zlabel(ax3d,'Z','Color',[0.7 0.7 0.7],'FontSize',9);
title(ax3d,'UAV Myopic Coverage  –  Live 3D', ...
      'Color',[1 1 1],'FontSize',11,'FontWeight','bold');
xlim(ax3d,[1 Gx]);  ylim(ax3d,[1 Gy]);  zlim(ax3d,[1 Gz]);
view(ax3d, 38, 24);

% Legend (manual text annotations, lighter weight)
legend_items = {'Obstacles','Hazard zones (global risk)', ...
                'z_{ref} plane','Free (sensed)','Visited','Sense sphere', ...
                'Path trail','Start'};
legend(ax3d, legend_items,'FontSize',7,'Color',[0.1 0.1 0.1], ...
       'TextColor',[0.9 0.9 0.9],'Location','northeast');

% --- RIGHT: 2D Top-Down Coverage Heatmap ---
ax2d = subplot('Position',[0.68 0.54 0.30 0.40]);
set(ax2d,'Color',[0.04 0.04 0.10], ...
         'XColor',[0.7 0.7 0.7],'YColor',[0.7 0.7 0.7]);
visit_slice = squeeze(sum(visit_cnt, 3));  % sum visits across all altitudes
h_heatmap = imagesc(ax2d, visit_slice');
colormap(ax2d, [linspace(0.04,0,256)', linspace(0.04,0.8,256)', linspace(0.10,0.3,256)']);
caxis(ax2d,[0 5]);
title(ax2d,'Top-Down Visit Heatmap','Color',[1 1 1],'FontSize',9);
xlabel(ax2d,'X','Color',[0.7 0.7 0.7],'FontSize',8);
ylabel(ax2d,'Y','Color',[0.7 0.7 0.7],'FontSize',8);
set(ax2d,'YDir','normal','FontSize',7);
hold(ax2d,'on');
% Mark obstacles on heatmap
obs_xy = squeeze(any(map==2, 3));
[ox2, oy2] = find(obs_xy');
if ~isempty(ox2)
    plot(ax2d, oy2, ox2, 'rs','MarkerSize',4,'LineWidth',1);
end
h_uav2d = plot(ax2d, pos(2), pos(1), 'w^','MarkerSize',8, ...
               'MarkerFaceColor',[1 1 0.2],'LineWidth',1);

% --- RIGHT BOTTOM: Live cost bar chart ---
ax_bar = subplot('Position',[0.68 0.06 0.30 0.38]);
set(ax_bar,'Color',[0.06 0.06 0.12], ...
           'XColor',[0.7 0.7 0.7],'YColor',[0.7 0.7 0.7]);
bar_labels = {'Local\n(static+vis)', 'Alt Risk', 'Energy', 'Global Risk'};
h_bars = bar(ax_bar, [0 0 0 0], 'FaceColor','flat');
h_bars.CData = [0.4 0.6 1.0; 1.0 0.7 0.1; 0.3 1.0 0.4; 1.0 0.3 0.8];
title(ax_bar,'Live Cost Breakdown','Color',[1 1 1],'FontSize',9);
ylabel(ax_bar,'Cost contribution','Color',[0.7 0.7 0.7],'FontSize',8);
set(ax_bar,'XTickLabel',{'Local','AltRisk','Energy','Global'}, ...
           'FontSize',8,'XColor',[0.7 0.7 0.7],'YColor',[0.7 0.7 0.7]);
ylim(ax_bar,[0 3]);

rotor_angle = 0;

% ==========================================================
%  SECTION 8 — MAIN COVERAGE LOOP
%
%  Each iteration = one step of the UAV.
%  The algorithm is purely GREEDY — no look-ahead, no global
%  planning, no backtracking.
%
%  HOW COVERAGE IS ACHIEVED WITHOUT BACKPROPAGATION:
%  The mc_visited * Vi term is the key mechanism. Every visited
%  cell accumulates a penalty. This makes the algorithm naturally
%  "push away" from explored regions toward unexplored ones —
%  like leaving expensive footprints that force you to walk
%  somewhere new. It's emergent, not planned.
%
%  CAN IT FAIL? YES. If the UAV gets surrounded by visited cells
%  and obstacles with no unvisited neighbours reachable, it gets
%  trapped. MAX_ITER handles this gracefully.
% ==========================================================

fprintf('Starting coverage loop...\n');
fprintf('(Close Figure 1 to stop early)\n\n');

while coverage < COV_TARGET && iter < MAX_ITER

    if ~ishandle(fig_anim),  break;  end

    iter = iter + 1;

    % ----------------------------------------------------------
    %  STEP A: SENSE — Reveal 26 voxels around current position
    %  This is the MYOPIC part. The UAV sees only immediate
    %  neighbours. Unknown (0) voxels become Free (1).
    % ----------------------------------------------------------
    nbrs = get_26_neighbors(pos, Gx, Gy, Gz);
    for k = 1:size(nbrs,1)
        nx=nbrs(k,1); ny=nbrs(k,2); nz=nbrs(k,3);
        if map(nx,ny,nz)==0,  map(nx,ny,nz)=1;  end
    end

    % ----------------------------------------------------------
    %  STEP B: EVALUATE COST for each of the 26 neighbours
    %  Full cost function with all 4 terms.
    % ----------------------------------------------------------
    min_cost = inf;
    best     = [];
    best_cost_breakdown = [0 0 0 0];

    for k = 1:size(nbrs,1)
        nx=nbrs(k,1); ny=nbrs(k,2); nz=nbrs(k,3);
        if map(nx,ny,nz)==2,  continue;  end   % skip obstacles

        % --- LOCAL TERM 1: Basic motion + visited penalty ---
        % mc_static = 1 (base movement cost per voxel, direction-simplified)
        % mc_visited = MC_VIS * Vi (grows with each revisit)
        % This term directly mirrors Eq.(1) from the paper:
        %   mc = mc_static + mc_visited*Vi
        mc_static = 1.0;
        Vi        = visit_cnt(nx, ny, nz);
        cost_local = alpha * (mc_static + MC_VIS * Vi);

        % --- LOCAL TERM 2: Altitude risk ---
        % Three components (all real-time, sensor-dependent):
        %   w1*|dz|         = cost of changing altitude
        %   w2*(1/d_obs)    = cost of flying near obstacles
        %   w3*|z - z_ref|  = cost of deviating from cruise altitude
        dz    = nz - pos(3);
        d_obs = nearest_obs_dist([nx,ny,nz], map, Gx, Gy, Gz);
        mc_alt = w1*abs(dz) + w2*(1/max(d_obs,0.5)) + w3*abs(nz-z_ref);
        cost_alt = beta * mc_alt;

        % --- LOCAL TERM 3: Energy cost ---
        % E_move = P_hover*t  +  m*g*dz_climb  +  F_drag*dist
        %          (hover)        (climb PE)       (air drag)
        % These are physically distinct:
        %   Hover: energy to stay airborne regardless of motion
        %   Climb: energy against gravity when ascending only
        %   Drag:  energy wasted against air resistance while moving
        % Normalised by 200J (approx max energy for one voxel hop)
        dist_m  = sqrt((nx-pos(1))^2+(ny-pos(2))^2+dz^2) * CELL_SIZE;
        t_step  = dist_m / v_nom;
        E_step  = P_hover*t_step + m_uav*g_acc*max(dz,0)*CELL_SIZE + F_drag*dist_m;
        mc_energy = E_step / 200;
        cost_energy = gamma * mc_energy;

        % --- GLOBAL TERM: Pre-computed risk map ---
        % This is the equivalent of mc_DEM in the original paper.
        % Computed ONCE before the mission (see Section 4).
        % Represents pre-known hazard zones, altitude restrictions,
        % obstacle-dense regions from prior survey.
        % The UAV avoids these zones even before seeing them with sensors.
        mc_glob = global_risk(nx, ny, nz);
        cost_glob = delta * mc_glob;

        % --- APF repulsion (safety layer, not in paper's cost function) ---
        apf = apf_repulsive_score(pos,[nx,ny,nz],map,Gx,Gy,Gz,D_SAFE);

        % --- TOTAL COST (complete UAV cost function from proposal) ---
        total_cost = cost_local + cost_alt + cost_energy + cost_glob ...
                   + APF_SCALE * apf;

        if total_cost < min_cost
            min_cost = total_cost;
            best     = [nx, ny, nz];
            best_cost_breakdown = [cost_local, cost_alt, cost_energy, cost_glob];
        end
    end

    % ----------------------------------------------------------
    %  STEP C: MOVE to best voxel
    %  If no valid neighbour found, the UAV is trapped.
    %  This CAN happen if surrounded by obstacles + visited cells.
    %  MAX_ITER cap handles this gracefully.
    % ----------------------------------------------------------
    if isempty(best),  break;  end

    dz_move   = best(3) - pos(3);
    dist_move = sqrt(sum((best-pos).^2)) * CELL_SIZE;
    t_move    = dist_move / v_nom;
    E_move    = P_hover*t_move + m_uav*g_acc*max(dz_move,0)*CELL_SIZE + F_drag*dist_move;
    E_total   = E_total + E_move;

    pos = best;
    visit_cnt(pos(1),pos(2),pos(3)) = visit_cnt(pos(1),pos(2),pos(3)) + 1;
    if map(pos(1),pos(2),pos(3)) ~= 2
        map(pos(1),pos(2),pos(3)) = 3;
    end
    path_log(end+1,:) = pos; %#ok<AGROW>

    % Log cost components for Figure 6 later
    cost_log_alt(end+1)    = best_cost_breakdown(2); %#ok<AGROW>
    cost_log_energy(end+1) = best_cost_breakdown(3); %#ok<AGROW>
    cost_log_global(end+1) = best_cost_breakdown(4); %#ok<AGROW>

    % ----------------------------------------------------------
    %  STEP D: UPDATE COVERAGE METRIC
    %  Simple fraction: visited voxels / total free voxels
    %  This is the Path Length Ratio denominator from Eq.(4)
    % ----------------------------------------------------------
    visited  = sum(map(:)==3);
    coverage = visited / free_total;
    cov_hist(end+1) = coverage; %#ok<AGROW>

    % ----------------------------------------------------------
    %  STEP E: ANIMATE (every ANIM_SKIP steps)
    %  Uses set() to update existing graphics objects — much
    %  faster than delete+redraw.
    % ----------------------------------------------------------
    if mod(iter, ANIM_SKIP)==0 && ishandle(fig_anim)

        % Update free voxel cloud (yellow — sensed but unvisited)
        [fx,fy,fz] = ind2sub([Gx,Gy,Gz], find(map==1));
        set(h_free, 'XData',fx, 'YData',fy, 'ZData',fz);

        % Update visited voxel cloud (green)
        [vx,vy,vz_v] = ind2sub([Gx,Gy,Gz], find(map==3));
        set(h_visited, 'XData',vx, 'YData',vy, 'ZData',vz_v);

        % Highlight current sensing sphere (show 26 neighbours as cyan ring)
        nbrs_now = get_26_neighbors(pos, Gx, Gy, Gz);
        set(h_sense, 'XData',nbrs_now(:,1), ...
                     'YData',nbrs_now(:,2), ...
                     'ZData',nbrs_now(:,3));

        % Update trail — rainbow colour by time (blue→red)
        n_path = size(path_log,1);
        if n_path > 2
            cmap_trail = jet(n_path);
            delete(h_trail);
            h_trail = gobjects(n_path-1,1);
            for s = max(1,n_path-40):n_path-1   % only last 40 segments for speed
                ci = max(1, min(n_path, s));
                h_trail(s) = plot3(ax3d, ...
                    path_log(s:s+1,1), path_log(s:s+1,2), path_log(s:s+1,3), ...
                    '-','Color',[cmap_trail(ci,:) 0.75],'LineWidth',1.6);
            end
        end

        % Move UAV
        move_uav_3d(uav_h, rotor_h, pos, rotor_angle);
        rotor_angle = mod(rotor_angle + ROTOR_SPD, 360);

        % HUD
        set(h_hud,'String', sprintf( ...
            'Cov: %5.1f%%  |  Step: %d  |  Energy: %.0f J', ...
            coverage*100, iter, E_total));

        % 2D heatmap update
        visit_slice = squeeze(sum(visit_cnt, 3));
        set(h_heatmap, 'CData', visit_slice');
        set(h_uav2d, 'XData', pos(2), 'YData', pos(1));

        % Live cost bar
        set(h_bars, 'YData', best_cost_breakdown);
        ylim(ax_bar,[0 max(0.5, max(best_cost_breakdown)*1.3)]);

        drawnow limitrate;
    end

    if mod(iter,200)==0
        fprintf('  Iter %4d | Coverage %.1f%% | Energy %.0f J\n', ...
                iter, coverage*100, E_total);
    end
end

% Final frame
if ishandle(fig_anim)
    [fx,fy,fz] = ind2sub([Gx,Gy,Gz], find(map==1));
    set(h_free,'XData',fx,'YData',fy,'ZData',fz);
    [vx,vy,vz_v] = ind2sub([Gx,Gy,Gz], find(map==3));
    set(h_visited,'XData',vx,'YData',vy,'ZData',vz_v);
    move_uav_3d(uav_h, rotor_h, pos, rotor_angle);
    set(h_hud,'String', sprintf('DONE  Cov: %.1f%%  Steps: %d  Energy: %.0fJ', ...
              coverage*100, iter, E_total));
    % Final position marker (red square)
    plot3(ax3d, pos(1),pos(2),pos(3),'s','Color',[1 0.2 0.2], ...
          'MarkerSize',13,'MarkerFaceColor',[1 0.2 0.2],'LineWidth',1.5);
    visit_slice = squeeze(sum(visit_cnt,3));
    set(h_heatmap,'CData',visit_slice');
    drawnow;
end

% ==========================================================
%  SECTION 9 — RESULTS
% ==========================================================
fprintf('\n========== Simulation Results ==========\n');
fprintf('  Total iterations   : %d\n',   iter);
fprintf('  Final coverage     : %.1f%%\n', coverage*100);
fprintf('  Total energy used  : %.2f J\n', E_total);
fprintf('  Avg energy/step    : %.2f J\n', E_total/max(iter,1));
fprintf('  Path length ratio  : %.3f\n',   iter/free_total);
fprintf('  (Optimal ratio = 1.0: each cell visited exactly once)\n');
fprintf('=========================================\n\n');

fprintf('Rendering figures 2, 3, 4, 6...\n');

% ==========================================================
%  SECTION 10 — FIGURE 2: 3D FINAL PATH (rainbow trail)
%
%  Same data as Figure 1 but:
%  - White background for readability
%  - Full rainbow path (blue=early steps, red=late steps)
%  - Voxel states shown with distinct colours and sizes
% ==========================================================

figure('Name','Figure 2 - 3D Final Path (Rainbow Trail)', ...
       'Color','white','Position',[980, 40, 780, 660]);
hold on; grid on; box on;

% Obstacles — dark red cubes
[ox3,oy3,oz3] = ind2sub([Gx,Gy,Gz], find(map==2));
scatter3(ox3,oy3,oz3, 50, [0.6 0.05 0.05], 's', 'filled', 'MarkerFaceAlpha',0.6);

% Free (sensed but unvisited) — small yellow
[fx3,fy3,fz3] = ind2sub([Gx,Gy,Gz], find(map==1));
if ~isempty(fx3)
    scatter3(fx3,fy3,fz3, 10, [1.0 0.85 0.1], '.', 'MarkerFaceAlpha',0.3);
end

% Visited — green, size proportional to visit count
[vx3,vy3,vz3] = ind2sub([Gx,Gy,Gz], find(map==3));
vc = zeros(length(vx3),1);
for i=1:length(vx3)
    vc(i) = visit_cnt(vx3(i),vy3(i),vz3(i));
end
scatter3(vx3,vy3,vz3, 8+vc*5, [0.2 0.8 0.35], 'o', 'filled', 'MarkerFaceAlpha',0.28);

% Rainbow flight path (full, blue=early → red=late)
n_path = size(path_log,1);
cmap_r = jet(n_path);
for s = 1:n_path-1
    plot3(path_log(s:s+1,1), path_log(s:s+1,2), path_log(s:s+1,3), ...
          '-','Color', cmap_r(s,:),'LineWidth',1.7);
end

% Start / end
plot3(path_log(1,1),   path_log(1,2),   path_log(1,3), 'p', ...
      'MarkerSize',14,'MarkerFaceColor',[0.1 0.9 0.2],'Color',[0.1 0.9 0.2],'LineWidth',1.5);
plot3(path_log(end,1), path_log(end,2), path_log(end,3), 's', ...
      'MarkerSize',12,'MarkerFaceColor',[1 0.1 0.1],'Color',[1 0.1 0.1],'LineWidth',1.5);

% Hazard centres
for h = 1:N_HAZARDS
    plot3(hazard_centres(h,1),hazard_centres(h,2),hazard_centres(h,3), ...
          'd','Color',[0.8 0.0 0.9],'MarkerSize',10, ...
          'MarkerFaceColor',[0.8 0 0.9],'LineWidth',1.3);
end

% Cruise plane
[pp_x2,pp_y2] = meshgrid(1:Gx,1:Gy);
surf(pp_x2, pp_y2, z_ref*ones(size(pp_x2)), ...
     'FaceColor',[0.2 0.9 0.4],'FaceAlpha',0.07,'EdgeColor','none');

colormap(jet); cb2=colorbar; caxis([1 n_path]);
cb2.Label.String = 'Step number (blue=early, red=late)';
cb2.Label.FontSize = 9;

xlabel('X [voxel]','FontSize',11); ylabel('Y [voxel]','FontSize',11);
zlabel('Z [voxel]','FontSize',11);
title(sprintf('3D UAV Path  |  Coverage: %.1f%%  |  Steps: %d  |  Energy: %.0fJ', ...
              coverage*100, iter, E_total), 'FontSize',11,'FontWeight','bold');
legend({'Obstacles (red=blocked)','Free (sensed, unvisited)', ...
        'Visited (size ∝ revisits)','Flight path (time-coded)', ...
        'Start','End','Global hazard zones','z_{ref} plane'}, ...
       'Location','best','FontSize',8);
xlim([1 Gx]); ylim([1 Gy]); zlim([1 Gz]);
view(42,28);

% ==========================================================
%  SECTION 11 — FIGURE 3: COVERAGE PROGRESS
% ==========================================================

figure('Name','Figure 3 - Coverage Progress','Color','white', ...
       'Position',[40, 430, 700, 420]);
hold on; grid on; box on;

fill([0 length(cov_hist) length(cov_hist) 0], ...
     [0 0 COV_TARGET*100 COV_TARGET*100], ...
     [0.90 0.95 1.00],'EdgeColor','none','FaceAlpha',0.7);
fill([0 length(cov_hist) length(cov_hist) 0], ...
     [COV_TARGET*100 COV_TARGET*100 105 105], ...
     [0.90 1.00 0.90],'EdgeColor','none','FaceAlpha',0.7);

plot(1:length(cov_hist), cov_hist*100, 'b-', 'LineWidth', 2.2);

yline(COV_TARGET*100,'r--','LineWidth',1.8,...
      'Label',sprintf('Target %.0f%%',COV_TARGET*100),...
      'LabelHorizontalAlignment','left','FontSize',10);

for tgt = [25, 50, 75]
    idx = find(cov_hist*100 >= tgt, 1);
    if ~isempty(idx)
        plot(idx, tgt, 'ko','MarkerSize',7,'MarkerFaceColor',[1 0.6 0.1]);
        text(idx+8, tgt+2, sprintf('%d%% at step %d',tgt,idx),...
             'FontSize',9,'Color',[0.4 0.2 0]);
    end
end

annotation('textbox',[0.55 0.20 0.35 0.15],...
    'String',{'{\bf Why the tail slows down:}', ...
              'Unvisited voxels get rarer.', ...
              'mc_{vis} penalty pushes UAV', ...
              'outward but search radius', ...
              'grows → more revisits needed.'},...
    'FontSize',8,'BackgroundColor',[0.95 0.95 0.80], ...
    'EdgeColor',[0.6 0.6 0.0]);

xlabel('Iteration','FontSize',12,'FontWeight','bold');
ylabel('Coverage [%]','FontSize',12,'FontWeight','bold');
title(sprintf('Coverage Progress  |  Final: %.1f%%  in  %d steps', ...
              coverage*100, iter),'FontSize',12,'FontWeight','bold');
ylim([0 105]); xlim([1 length(cov_hist)]);

% ==========================================================
%  SECTION 12 — FIGURE 4: ALTITUDE PROFILE
% ==========================================================

figure('Name','Figure 4 - Altitude Profile','Color','white', ...
       'Position',[760, 430, 700, 420]);
hold on; grid on; box on;

n_steps  = size(path_log,1);
alt_vals = path_log(:,3);
cmap_alt = jet(Gz);

fill([1 n_steps n_steps 1],[z_ref-1 z_ref-1 z_ref+1 z_ref+1], ...
     [0.85 1.0 0.85],'EdgeColor','none','FaceAlpha',0.7);

for s = 1:n_steps-1
    ci = max(1,min(Gz, round(alt_vals(s))));
    plot(s:s+1, alt_vals(s:s+1),'-','Color',cmap_alt(ci,:),'LineWidth',1.8);
end

yline(z_ref,'g--','LineWidth',1.8,'Label','z_{ref} (cruise)',...
      'LabelHorizontalAlignment','left','FontSize',10);

colormap(jet(Gz));
cb4=colorbar; clim([1 Gz]);
cb4.Label.String='Altitude [voxel]'; cb4.Label.FontSize=10;

xlabel('Step','FontSize',12,'FontWeight','bold');
ylabel('Altitude [voxel]','FontSize',12,'FontWeight','bold');
title('UAV Altitude Profile  –  Colour = Height',...
      'FontSize',12,'FontWeight','bold');
ylim([0 Gz+1]); xlim([1 n_steps]);

% ==========================================================
%  SECTION 13 — FIGURE 6: COST COMPONENT BREAKDOWN
%
%  Shows how each cost term contributed over time.
%  This directly illustrates the 4-term cost function and
%  lets you see which term dominated the UAV's decisions.
% ==========================================================

figure('Name','Figure 6 - Cost Component Breakdown','Color','white', ...
       'Position',[400, 100, 800, 420]);
hold on; grid on; box on;

steps_c = 1:length(cost_log_alt);

area(steps_c, [cost_log_alt(:), cost_log_energy(:), cost_log_global(:)], ...
     'FaceAlpha', 0.5);

legend({sprintf('Alt Risk (\\beta=%.2f)',beta), ...
        sprintf('Energy    (\\gamma=%.2f)',gamma), ...
        sprintf('Global    (\\delta=%.2f)',delta)}, ...
       'Location','best','FontSize',10);

xlabel('Iteration','FontSize',12,'FontWeight','bold');
ylabel('Cost component value','FontSize',12,'FontWeight','bold');
title('Cost Function Breakdown Per Step  –  Shows Which Term Drove Each Decision',...
      'FontSize',11,'FontWeight','bold');
xlim([1 length(cost_log_alt)]);

fprintf('All 6 figures rendered.\n');
fprintf('Fig1=Animation | Fig2=3DPath | Fig3=Coverage\n');
fprintf('Fig4=Altitude  | Fig5=GlobalRisk | Fig6=CostBreakdown\n');

end  % main function


% ==========================================================
%  UAV GRAPHICS HELPERS
% ==========================================================

function [body_h, rotor_h] = draw_uav_3d(ax, pos)
    ARM=0.6; R_r=0.28; BODY=0.20;
    cx=pos(1); cy=pos(2); cz=pos(3);
    th=linspace(0,2*pi,20);
    body_h(1) = fill3(ax, cx+BODY*cos(th), cy+BODY*sin(th), cz*ones(1,20), ...
                      [0.2 0.2 0.2],'EdgeColor',[0.7 0.7 0.7],'FaceAlpha',0.95);
    arm_dirs=[1 1;1 -1;-1 1;-1 -1]*ARM/sqrt(2);
    for i=1:4
        body_h(i+1)=plot3(ax,[cx,cx+arm_dirs(i,1)],[cy,cy+arm_dirs(i,2)],[cz,cz],...
                          '-','Color',[0.6 0.6 0.6],'LineWidth',2.5);
    end
    rotor_h=gobjects(4,1);
    for i=1:4
        rx=cx+arm_dirs(i,1); ry=cy+arm_dirs(i,2);
        th_r=linspace(0,2*pi,16);
        rotor_h(i)=fill3(ax,rx+R_r*cos(th_r),ry+R_r*sin(th_r),cz*ones(1,16),...
                         [0.5 0.9 1.0],'EdgeColor',[0.2 0.6 0.9],...
                         'FaceAlpha',0.6,'LineWidth',1.0);
    end
end

function move_uav_3d(body_h, rotor_h, pos, ang_deg)
    ARM=0.6; R_r=0.28; BODY=0.20;
    cx=pos(1); cy=pos(2); cz=pos(3);
    th=linspace(0,2*pi,20);
    set(body_h(1),'XData',cx+BODY*cos(th),'YData',cy+BODY*sin(th),'ZData',cz*ones(1,20));
    arm_dirs=[1 1;1 -1;-1 1;-1 -1]*ARM/sqrt(2);
    for i=1:4
        set(body_h(i+1),'XData',[cx,cx+arm_dirs(i,1)],...
                        'YData',[cy,cy+arm_dirs(i,2)],'ZData',[cz,cz]);
    end
    ang_rad=deg2rad(ang_deg);
    th_r=linspace(0,2*pi,16)+ang_rad;
    for i=1:4
        rx=cx+arm_dirs(i,1); ry=cy+arm_dirs(i,2);
        set(rotor_h(i),'XData',rx+R_r*cos(th_r),'YData',ry+R_r*sin(th_r),...
                       'ZData',cz*ones(1,16));
    end
end


% ==========================================================
%  CORE ALGORITHM HELPERS
% ==========================================================

function nbrs = get_26_neighbors(pos, Gx, Gy, Gz)
    nbrs=zeros(26,3); count=0;
    for dx=-1:1
        for dy=-1:1
            for dz=-1:1
                if dx==0&&dy==0&&dz==0, continue; end
                nx=pos(1)+dx; ny=pos(2)+dy; nz=pos(3)+dz;
                if nx>=1&&nx<=Gx&&ny>=1&&ny<=Gy&&nz>=1&&nz<=Gz
                    count=count+1; nbrs(count,:)=[nx,ny,nz];
                end
            end
        end
    end
    nbrs=nbrs(1:count,:);
end

function d = nearest_obs_dist(pos, map, Gx, Gy, Gz)
    d=inf; R=4;
    for dx=-R:R
        for dy=-R:R
            for dz=-R:R
                nx=pos(1)+dx; ny=pos(2)+dy; nz=pos(3)+dz;
                if nx<1||nx>Gx||ny<1||ny>Gy||nz<1||nz>Gz, continue; end
                if map(nx,ny,nz)==2
                    dd=sqrt(dx^2+dy^2+dz^2);
                    if dd<d, d=dd; end
                end
            end
        end
    end
end

function score = apf_repulsive_score(pos, target, map, Gx, Gy, Gz, D_safe)
    score=0; dir=target-pos; d_norm=norm(dir);
    if d_norm<1e-9, return; end
    dir_u=dir/d_norm;
    for dx=-2:2
        for dy=-2:2
            for dz=-2:2
                nx=pos(1)+dx; ny=pos(2)+dy; nz=pos(3)+dz;
                if nx<1||nx>Gx||ny<1||ny>Gy||nz<1||nz>Gz, continue; end
                if map(nx,ny,nz)==2
                    d=sqrt(dx^2+dy^2+dz^2);
                    if d<D_safe&&d>0
                        mag=(1/d-1/D_safe)^2;
                        proj=dot(dir_u,-[dx,dy,dz]/d);
                        score=score+mag*max(proj,0);
                    end
                end
            end
        end
    end
end
