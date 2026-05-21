% =========================================================
%  UAV_Myopic_CPP.m  –  v5
%  3-D Myopic Coverage Path Planning for UAV
%  Original MPP algorithm + Dynamic Obstacle Extension
%
%  >>>  TO SWITCH WORLDS CHANGE ONLY THIS LINE:  <<<
%       WORLD_FILE = 'world_01.mat';
%
%  Run UAV_WorldBuilder.m first to create the .mat files.
%
%  =========================================================
%  PIPELINE ARCHITECTURE — what is original vs what is added
%  =========================================================
%
%  ORIGINAL MPP PIPELINE (Santra et al. 2024) — UNCHANGED:
%  ─────────────────────────────────────────────────────────
%  Sense 26 neighbours  →  compute mc_static + MC_VIS×Vi (alpha)
%                       →  compute mc_alt               (beta)
%                       →  compute mc_energy             (gamma)
%                       →  compute mc_global (DEM equiv) (delta)
%                       →  APF static repulsion
%                       →  select min-cost voxel  →  move UAV
%
%  DYNAMIC OBSTACLE EXTENSION — PARALLEL LAYER, NOT REPLACEMENT:
%  ─────────────────────────────────────────────────────────────
%  [NEW] Spawn/move/expire transient obstacles (bird model)
%  [NEW] Update dyn_map, dyn_risk
%  [NEW] compute mc_dynamic                              (epsilon)
%  [NEW] APF dynamic repulsion  (added on top of static APF)
%  [NEW] 5th term added to cost sum — nothing replaced
%
%  DYNAMIC OBSTACLE MODEL (Bird-Like Transient Behaviour):
%  ─────────────────────────────────────────────────────────
%  Each bird/transient obstacle:
%    1. SPAWNS at a random world boundary face (edge of grid)
%    2. Has a random but persistent DIRECTION of flight
%    3. MOVES in that direction each movement tick (slight drift)
%    4. EXPIRES when:
%         a) It crosses out of the grid on the other side, OR
%         b) Its lifetime counter hits zero
%    5. A new one may SPAWN each step with probability spawn_prob
%       (up to max_alive at once)
%  → The world is never "full" of dynamic obstacles.
%    They appear, cross, and vanish. You can't predict them
%    from the global risk map — that is the point.
%
%  FULL 5-TERM COST FUNCTION:
%    mc = alpha   × (mc_static + MC_VIS×Vi)  local: revisit push
%       + beta    × mc_alt                   local: altitude risk
%       + gamma   × mc_energy                local: thrust energy
%       + delta   × mc_global                global: pre-known risk
%       + epsilon × mc_dynamic               dynamic: transient risk
%    weights: alpha+beta+gamma+delta+epsilon = 1.0
%
%  FIGURES:
%    Fig 1  –  Live Animation (3D + top-down heatmap + cost bars)
%    Fig 2  –  3D Final Path (rainbow time-coded trail)
%    Fig 3  –  Coverage Progress
%    Fig 4  –  Altitude Profile
%    Fig 5  –  Global Risk Map (pre-computed, shown before run)
%    Fig 6  –  Cost Breakdown (5 terms stacked area chart)
%
%  Run: >> UAV_Myopic_CPP   (MATLAB R2021a+)
% =========================================================

function UAV_Myopic_CPP_dynamic()

clc; clear; close all;

% >>>  CHANGE THIS LINE TO SWITCH WORLDS  <<<
WORLD_FILE = 'world_01.mat';

rng(42);

fprintf('============================================\n');
fprintf('  UAV Myopic CPP  v5  –  Bird Dynamics\n');
fprintf('============================================\n');
fprintf('  World file : %s\n\n', WORLD_FILE);

% ==========================================================
%  SECTION 1 — UAV PARAMETERS
%  (World geometry/obstacles come from the .mat file.)
% ==========================================================

% UAV Physics
m_uav  = 1.5;    g_acc  = 9.81;   rho    = 1.225;
A_rot  = 0.10;   A_drag = 0.05;   Cd     = 0.50;
v_nom  = 1.0;

% Cost weights — must sum to 1.0
% Original 4 terms preserved exactly as before.
% epsilon is the NEW 5th term for dynamic obstacles.
alpha   = 0.28;   % local: basic motion + revisit penalty
beta    = 0.22;   % local: altitude risk
gamma   = 0.18;   % local: energy cost
delta   = 0.15;   % global: pre-computed risk map (= mc_DEM analogue)
epsilon = 0.17;   % dynamic: transient bird/obstacle risk  ← NEW
% 0.28+0.22+0.18+0.15+0.17 = 1.00  ✓

% Altitude-risk sub-weights
w1=1.0;  w2=2.0;  w3=0.5;

% Visited-cell penalty (must be ≥ 1.1 per original paper)
MC_VIS = 1.5;

% APF
D_SAFE      = 2.5;   % static obstacle repulsion radius
D_SAFE_DYN  = 3.5;   % dynamic obstacle repulsion radius (wider — they can move toward you)
APF_SCALE   = 0.04;

% Dynamic obstacle cost values
MC_DYN_DIRECT = 10.0;  % cost if candidate voxel IS currently occupied by a bird
MC_DYN_MAX    =  4.0;  % cap on fading trail cost

% Risk trail decay per step (< 1). At 0.85, trail is ~1% after 25 steps.
DYN_FADE_RATE = 0.85;

% Simulation
COV_TARGET = 0.90;
MAX_ITER   = 0;    % set after world load (5 × free voxels)

% Animation
ANIM_SKIP = 8;
ROTOR_SPD = 30;

% ==========================================================
%  SECTION 2 — LOAD WORLD
% ==========================================================

if ~isfile(WORLD_FILE)
    error('World file "%s" not found.\nRun UAV_WorldBuilder.m first.', WORLD_FILE);
end
tmp = load(WORLD_FILE);
W   = tmp.W;

Gx = W.Gx;  Gy = W.Gy;  Gz = W.Gz;
z_ref = W.z_ref;

map       = W.static_map;   % working map: 0=unknown 1=free 2=obs 3=visited
visit_cnt = zeros(Gx, Gy, Gz);
free_total = sum(map(:) ~= 2);
MAX_ITER   = free_total * 5;

% ==========================================================
%  SECTION 3 — DERIVED QUANTITIES
% ==========================================================

P_hover = (m_uav * g_acc)^1.5 / sqrt(2 * rho * A_rot);
F_drag  = 0.5 * Cd * rho * A_drag * v_nom^2;

fprintf('World    : %s\n', W.name);
fprintf('Grid     : %d×%d×%d  |  Free voxels: %d  |  z_ref: %d\n', ...
        Gx, Gy, Gz, free_total, z_ref);
fprintf('Hover P  : %.2f W  |  Drag: %.4f N\n', P_hover, F_drag);
fprintf('Bird params: spawn_prob=%.2f  max_alive=%d  life=[%d,%d] steps\n\n', ...
        W.spawn_prob, W.max_alive, W.lifetime_min, W.lifetime_max);

% ==========================================================
%  SECTION 4 — DYNAMIC OBSTACLE STATE  (Bird Flock Manager)
%
%  dyn_map:  binary Gx×Gy×Gz — 1 where a bird currently is
%  dyn_risk: float Gx×Gy×Gz  — fading danger trail [0,1]
%
%  flock struct array (one entry per alive bird):
%    .pos      [1×3] current voxel position
%    .dir      [1×3] unit direction vector (persistent drift)
%    .lifetime remaining steps before forced expiry
%
%  Starts empty — no birds at t=0. They spawn during the run.
% ==========================================================

dyn_map   = zeros(Gx, Gy, Gz, 'uint8');
dyn_risk  = zeros(Gx, Gy, Gz);
flock     = struct('pos',{}, 'dir',{}, 'lifetime',{});  % empty flock

% ==========================================================
%  SECTION 5 — FIGURE 5: GLOBAL RISK MAP  (before run)
% ==========================================================

figure('Name', sprintf('Figure 5 - Global Risk Map [%s]', W.name), ...
       'Color','white','Position',[420, 280, 680, 490]);
mid_slice = squeeze(W.global_risk(:,:,z_ref))';
imagesc(mid_slice); colormap(hot); colorbar; hold on;
for h=1:size(W.hazard_centres,1)
    if abs(W.hazard_centres(h,3)-z_ref)<=2
        plot(W.hazard_centres(h,1),W.hazard_centres(h,2), ...
             'c*','MarkerSize',14,'LineWidth',2);
    end
end
obs_here=W.static_map(:,:,z_ref)==2;
[oh,ok]=find(obs_here');
if ~isempty(oh), plot(ok,oh,'bx','MarkerSize',7,'LineWidth',1.5); end
legend({'Hazard centres','Static obstacles'},'Location','best','FontSize',9);
title(sprintf('Global Risk Map  (z=%d)  –  [%s]', z_ref, W.name), ...
      'FontSize',11,'FontWeight','bold');
xlabel('X [voxel]','FontSize',10); ylabel('Y [voxel]','FontSize',10);
set(gca,'YDir','normal');
drawnow;

% ==========================================================
%  SECTION 6 — UAV INITIALISATION
% ==========================================================

pos       = W.start_pos;
map(pos(1),pos(2),pos(3)) = 3;
visit_cnt(pos(1),pos(2),pos(3)) = 1;

path_log  = pos;
E_total   = 0;
cov_hist  = [];
iter      = 0;
coverage  = 0;

% Cost component logs for Figure 6
log_alt=[];  log_energy=[];  log_global=[];  log_dynobs=[];

% ==========================================================
%  SECTION 7 — FIGURE 1: LIVE ANIMATION SETUP
% ==========================================================

fig_anim = figure('Name', sprintf('Figure 1 - Live Animation [%s]', W.name), ...
                  'Color',[0.04 0.04 0.10], ...
                  'Position',[40, 40, 1200, 680]);

% LEFT: 3D axes
ax3d = subplot('Position',[0.02 0.06 0.62 0.88]);
set(ax3d,'Color',[0.04 0.04 0.10], ...
         'XColor',[0.6 0.6 0.6],'YColor',[0.6 0.6 0.6],'ZColor',[0.6 0.6 0.6], ...
         'GridColor',[0.18 0.18 0.18],'GridAlpha',0.5,'FontSize',8);
hold(ax3d,'on'); grid(ax3d,'on'); box(ax3d,'on');

% Static obstacles
[sox,soy,soz]=ind2sub([Gx,Gy,Gz],find(map==2));
h_obstacles = scatter3(ax3d, sox, soy, soz, 50, [0.85 0.15 0.08], 's', 'filled', ...
                       'MarkerFaceAlpha', 0.80);

% Hazard centres
for h=1:size(W.hazard_centres,1)
    plot3(ax3d,W.hazard_centres(h,1),W.hazard_centres(h,2),W.hazard_centres(h,3), ...
          'd','Color',[1 0.1 1],'MarkerSize',9,'MarkerFaceColor',[1 0.1 1]);
end

% Cruise altitude reference plane
[pp_x,pp_y]=meshgrid(1:Gx,1:Gy);
surf(ax3d,pp_x,pp_y,z_ref*ones(size(pp_x)), ...
     'FaceColor',[0.2 0.9 0.4],'FaceAlpha',0.05,'EdgeColor','none');

% Free voxels (yellow, grows)
h_free = scatter3(ax3d,nan,nan,nan,10,[1.0 0.9 0.2],'.','MarkerFaceAlpha',0.45);

% Visited voxels (green, grows)
h_vis = scatter3(ax3d,nan,nan,nan,16,[0.2 0.85 0.35],'o','filled','MarkerFaceAlpha',0.28);

% Sensing sphere
h_sense = scatter3(ax3d,nan,nan,nan,38,[0.0 0.9 0.9],'o','MarkerFaceAlpha',0.10,'LineWidth',0.7);

% Bird/transient obstacles (bright orange — starts empty)
h_birds = scatter3(ax3d,nan,nan,nan,110,[1.0 0.55 0.0],'o','filled', ...
                   'MarkerFaceAlpha',0.95,'MarkerEdgeColor',[1.0 0.8 0.0],'LineWidth',1.8);

% Bird direction arrows (quiver, starts empty)
h_arrows = quiver3(ax3d,nan,nan,nan,nan,nan,nan, ...
                   0.6,'Color',[1 0.8 0.2],'LineWidth',1.4,'MaxHeadSize',0.8);

% Dynamic risk trail (faint orange cloud)
h_dyn_trail = scatter3(ax3d,nan,nan,nan,18,[1.0 0.65 0.2],'o','filled','MarkerFaceAlpha',0.18);

% Flight trail
h_trail = plot3(ax3d, pos(1), pos(2), pos(3), ...
                '-', 'Color',[0.45 0.60 1.0 0.65], 'LineWidth', 1.4);

% Start marker
plot3(ax3d,pos(1),pos(2),pos(3),'p','Color',[0.2 1.0 0.4], ...
      'MarkerSize',13,'MarkerFaceColor',[0.2 1.0 0.4]);

% UAV body
[uav_h,rotor_h] = draw_uav_3d(ax3d,pos);

% HUD
h_hud = text(ax3d,1.2,1.2,Gz+0.6,'Starting...', ...
             'Color',[1.0 1.0 0.3],'FontSize',10,'FontWeight','bold','FontName','Courier');

xlabel(ax3d,'X','Color',[0.7 0.7 0.7],'FontSize',9);
ylabel(ax3d,'Y','Color',[0.7 0.7 0.7],'FontSize',9);
zlabel(ax3d,'Z','Color',[0.7 0.7 0.7],'FontSize',9);
title(ax3d,sprintf('UAV Myopic CPP  –  %s',W.name),'Color',[1 1 1],'FontSize',10,'FontWeight','bold');
xlim(ax3d,[1 Gx]); ylim(ax3d,[1 Gy]); zlim(ax3d,[1 Gz]);
view(ax3d,38,24);
legend(ax3d, ...
    [h_obstacles(1), h_free, h_vis, h_sense, h_birds, h_arrows, ...
     h_dyn_trail, h_trail, uav_h(1)], ...
    {'Static obs','Free','Visited','Sense sphere','Birds (live)', ...
     'Bird directions','Bird trail','Path','UAV'}, ...
    'FontSize',7,'TextColor',[0.9 0.9 0.9], ...
    'Color',[0.08 0.08 0.08],'Location','northeast', ...
    'AutoUpdate','off');   % ← this is the key line;

% TOP-RIGHT: visit heatmap
ax2d = subplot('Position',[0.68 0.54 0.30 0.41]);
set(ax2d,'Color',[0.04 0.04 0.10],'XColor',[0.7 0.7 0.7],'YColor',[0.7 0.7 0.7]);
h_heatmap = imagesc(ax2d,squeeze(sum(visit_cnt,3))');
colormap(ax2d,[linspace(0.04,0,256)' linspace(0.04,0.85,256)' linspace(0.10,0.30,256)']);
caxis(ax2d,[0 5]); hold(ax2d,'on');
obs_xy=squeeze(any(W.static_map==2,3));
[ohx,ohy]=find(obs_xy'); if ~isempty(ohx), plot(ax2d,ohy,ohx,'rs','MarkerSize',3); end
h_birds2d = plot(ax2d,nan,nan,'o','Color',[1 0.55 0],'MarkerSize',7,'MarkerFaceColor',[1 0.55 0]);
h_uav2d   = plot(ax2d,pos(2),pos(1),'^','Color',[1 1 0.3],'MarkerSize',8,'MarkerFaceColor',[1 1 0.3]);
title(ax2d,'Top-Down (visit density)','Color',[1 1 1],'FontSize',9);
xlabel(ax2d,'X','Color',[0.7 0.7 0.7],'FontSize',8);
ylabel(ax2d,'Y','Color',[0.7 0.7 0.7],'FontSize',8);
set(ax2d,'YDir','normal','FontSize',7);

% BOTTOM-RIGHT: live cost bars (5 terms)
ax_bar = subplot('Position',[0.68 0.06 0.30 0.38]);
set(ax_bar,'Color',[0.06 0.06 0.12],'XColor',[0.7 0.7 0.7],'YColor',[0.7 0.7 0.7]);
h_bars = bar(ax_bar,[0 0 0 0 0],'FaceColor','flat');
h_bars.CData=[0.40 0.60 1.00;
              1.00 0.70 0.10;
              0.30 1.00 0.40;
              1.00 0.20 1.00;
              1.00 0.55 0.00];
title(ax_bar,'Live Cost Breakdown','Color',[1 1 1],'FontSize',9);
ylabel(ax_bar,'Cost','Color',[0.7 0.7 0.7],'FontSize',8);
set(ax_bar,'XTickLabel',{'Local','AltRisk','Energy','Global','Birds'}, ...
           'FontSize',8,'XColor',[0.8 0.8 0.8],'YColor',[0.7 0.7 0.7]);
ylim(ax_bar,[0 3]);

rotor_angle = 0;

% ==========================================================
%  SECTION 8 — MAIN COVERAGE LOOP
% ==========================================================

fprintf('Starting coverage loop...\n\n');

while coverage < COV_TARGET && iter < MAX_ITER

    if ~ishandle(fig_anim), break; end
    iter = iter + 1;

    % --------------------------------------------------------
    %  STEP A — BIRD MANAGER
    %  Runs every W.move_interval steps.
    %  Three operations in order:
    %    1. Spawn: maybe add a new bird at a boundary face
    %    2. Move:  advance each live bird along its direction
    %    3. Expire: remove birds that exited grid or timed out
    %  dyn_map and dyn_risk are updated here.
    % --------------------------------------------------------
    if mod(iter, W.move_interval) == 0
        [flock, dyn_map, dyn_risk] = run_bird_manager( ...
            flock, dyn_map, dyn_risk, W, map, Gx, Gy, Gz);
    end

    % Fade risk trail every algorithm step regardless of movement
    dyn_risk = dyn_risk * DYN_FADE_RATE;

    % --------------------------------------------------------
    %  STEP B — SENSE 26 NEIGHBOURS (myopic LiDAR)
    % --------------------------------------------------------
    nbrs = get_26_neighbors(pos, Gx, Gy, Gz);
    for k = 1:size(nbrs,1)
        nx=nbrs(k,1); ny=nbrs(k,2); nz=nbrs(k,3);
        if map(nx,ny,nz)==0, map(nx,ny,nz)=1; end
    end

    % --------------------------------------------------------
    %  STEP C — EVALUATE 5-TERM COST
    % --------------------------------------------------------
    min_cost = inf;
    best     = [];
    best_cb  = [0 0 0 0 0];

    for k = 1:size(nbrs,1)
        nx=nbrs(k,1); ny=nbrs(k,2); nz=nbrs(k,3);
        if map(nx,ny,nz)==2, continue; end   % static obstacle

        % TERM 1 — LOCAL  (alpha) — ORIGINAL, UNCHANGED
        Vi       = visit_cnt(nx,ny,nz);
        c_local  = alpha * (1.0 + MC_VIS * Vi);

        % TERM 2 — ALTITUDE RISK  (beta) — ORIGINAL, UNCHANGED
        dz       = nz - pos(3);
        d_obs    = nearest_obs_dist([nx,ny,nz],map,Gx,Gy,Gz);
        mc_alt   = w1*abs(dz) + w2*(1/max(d_obs,0.5)) + w3*abs(nz-z_ref);
        c_alt    = beta * mc_alt;

        % TERM 3 — ENERGY  (gamma) — ORIGINAL, UNCHANGED
        dist_m   = sqrt((nx-pos(1))^2+(ny-pos(2))^2+dz^2);
        t_step   = dist_m / v_nom;
        E_step   = P_hover*t_step + m_uav*g_acc*max(dz,0) + F_drag*dist_m;
        c_energy = gamma * (E_step / 200);

        % TERM 4 — GLOBAL RISK  (delta) — ORIGINAL, UNCHANGED
        c_global = delta * W.global_risk(nx,ny,nz);

        % TERM 5 — DYNAMIC / BIRD RISK  (epsilon) — NEW
        % If a bird is currently sitting in this voxel: huge cost
        % (near-impassable but not absolutely blocked — UAV can
        %  still pass if there's genuinely no other option)
        % If a bird was recently here: fading trail cost
        if dyn_map(nx,ny,nz) == 1
            mc_dyn = MC_DYN_DIRECT;
        else
            mc_dyn = min(dyn_risk(nx,ny,nz), MC_DYN_MAX);
        end
        c_dynamic = epsilon * mc_dyn;

        % APF: static obstacles (original) + birds (new, wider radius)
        apf_st  = apf_score(pos,[nx,ny,nz],map,    Gx,Gy,Gz,D_SAFE,    false);
        apf_dyn = apf_score(pos,[nx,ny,nz],dyn_map,Gx,Gy,Gz,D_SAFE_DYN,true);

        total = c_local + c_alt + c_energy + c_global + c_dynamic ...
              + APF_SCALE*(apf_st + 1.5*apf_dyn);

        if total < min_cost
            min_cost = total;
            best     = [nx,ny,nz];
            best_cb  = [c_local, c_alt, c_energy, c_global, c_dynamic];
        end
    end

    if isempty(best), break; end   % trapped

    % --------------------------------------------------------
    %  STEP D — MOVE UAV
    % --------------------------------------------------------
    dz_move   = best(3)-pos(3);
    dist_move = sqrt(sum((best-pos).^2));
    t_move    = dist_move / v_nom;
    E_move    = P_hover*t_move + m_uav*g_acc*max(dz_move,0) + F_drag*dist_move;
    E_total   = E_total + E_move;

    pos = best;
    visit_cnt(pos(1),pos(2),pos(3)) = visit_cnt(pos(1),pos(2),pos(3))+1;
    if map(pos(1),pos(2),pos(3)) ~= 2
        map(pos(1),pos(2),pos(3)) = 3;
    end
    path_log(end+1,:) = pos; %#ok<AGROW>

    log_alt(end+1)    = best_cb(2); %#ok<AGROW>
    log_energy(end+1) = best_cb(3); %#ok<AGROW>
    log_global(end+1) = best_cb(4); %#ok<AGROW>
    log_dynobs(end+1) = best_cb(5); %#ok<AGROW>

    visited  = sum(map(:)==3);
    coverage = visited / free_total;
    cov_hist(end+1) = coverage; %#ok<AGROW>

    % --------------------------------------------------------
    %  STEP E — ANIMATE
    % --------------------------------------------------------
    if mod(iter, ANIM_SKIP)==0 && ishandle(fig_anim)

    if mod(iter, ANIM_SKIP*2)==0
        [fx,fy,fz]=ind2sub([Gx,Gy,Gz],find(map==1));
        set(h_free,'XData',fx,'YData',fy,'ZData',fz);
        [vx,vy,vz_v]=ind2sub([Gx,Gy,Gz],find(map==3));
        set(h_vis,'XData',vx,'YData',vy,'ZData',vz_v);
    end

        % Sensing sphere
        %nbrs_now=get_26_neighbors(pos,Gx,Gy,Gz);
        %set(h_sense,'XData',nbrs_now(:,1),'YData',nbrs_now(:,2),'ZData',nbrs_now(:,3));

        % Birds (orange dots + direction arrows)
        n_birds = length(flock);
        if n_birds > 0
            bpos = vertcat(flock.pos);
            set(h_birds,'XData',bpos(:,1),'YData',bpos(:,2),'ZData',bpos(:,3));
            bdir = vertcat(flock.dir);
            set(h_arrows,'XData',bpos(:,1),'YData',bpos(:,2),'ZData',bpos(:,3), ...
                         'UData',bdir(:,1),'VData',bdir(:,2),'WData',bdir(:,3));
            set(h_birds2d,'XData',bpos(:,2),'YData',bpos(:,1));
        else
            set(h_birds,'XData',nan,'YData',nan,'ZData',nan);
            set(h_arrows,'XData',nan,'YData',nan,'ZData',nan, ...
                         'UData',nan,'VData',nan,'WData',nan);
            set(h_birds2d,'XData',nan,'YData',nan);
        end

        % Risk trail
        [trx,try_,trz]=ind2sub([Gx,Gy,Gz],find(dyn_risk>0.05));
        if ~isempty(trx)
            set(h_dyn_trail,'XData',trx,'YData',try_,'ZData',trz);
        else
            set(h_dyn_trail,'XData',nan,'YData',nan,'ZData',nan);
        end

        % Trail (last 50 segments, rainbow)
        n_path=size(path_log,1);
        % if n_path>2
        %     cmap_t=jet(n_path);
        %     try, delete(h_trail); catch, end
        %     for s=max(1,n_path-50):n_path-1
        %         h_trail=plot3(ax3d,path_log(s:s+1,1),path_log(s:s+1,2), ...
        %                       path_log(s:s+1,3),'-','Color',[cmap_t(s,:) 0.7],'LineWidth',1.5); %#ok<AGROW>
        %     end
        % end
        set(h_trail, 'XData',path_log(:,1), ...
             'YData',path_log(:,2), ...
             'ZData',path_log(:,3));

        % UAV body
        move_uav_3d(uav_h,rotor_h,pos,rotor_angle);
        rotor_angle=mod(rotor_angle+ROTOR_SPD,360);

        % HUD
        set(h_hud,'String',sprintf('Cov:%5.1f%%  Step:%d  E:%.0fJ  Birds:%d', ...
            coverage*100, iter, E_total, n_birds));

        % 2D heatmap + UAV
        set(h_heatmap,'CData',squeeze(sum(visit_cnt,3))');
        set(h_uav2d,'XData',pos(2),'YData',pos(1));

        % Cost bars
        set(h_bars,'YData',best_cb);
        ylim(ax_bar,[0 max(0.5, max(best_cb)*1.4)]);

        drawnow limitrate;
    end

    if mod(iter,200)==0
        fprintf('  Iter %4d | Cov %.1f%% | Energy %.0fJ | Birds alive: %d\n', ...
                iter, coverage*100, E_total, length(flock));
    end
end

% Final frame
if ishandle(fig_anim)
    [vx,vy,vz_v]=ind2sub([Gx,Gy,Gz],find(map==3));
    set(h_vis,'XData',vx,'YData',vy,'ZData',vz_v);
    move_uav_3d(uav_h,rotor_h,pos,rotor_angle);
    set(h_hud,'String',sprintf('DONE  Cov:%.1f%%  Steps:%d  Energy:%.0fJ', ...
              coverage*100,iter,E_total));
    plot3(ax3d,pos(1),pos(2),pos(3),'s','Color',[1 0.2 0.2],'MarkerSize',13,...
          'MarkerFaceColor',[1 0.2 0.2],'LineWidth',1.5);
    set(h_heatmap,'CData',squeeze(sum(visit_cnt,3))');
    drawnow;
end

% ==========================================================
%  SECTION 9 — RESULTS
% ==========================================================
fprintf('\n========== Simulation Results ==========\n');
fprintf('  World             : %s\n', W.name);
fprintf('  Total iterations  : %d\n', iter);
fprintf('  Final coverage    : %.1f%%\n', coverage*100);
fprintf('  Total energy      : %.2f J\n', E_total);
fprintf('  Avg energy/step   : %.2f J\n', E_total/max(iter,1));
fprintf('  Path length ratio : %.3f\n', iter/free_total);
fprintf('=========================================\n\n');
fprintf('Rendering figures 2, 3, 4, 6...\n');

% ==========================================================
%  SECTION 10 — FIGURE 2: 3D FINAL PATH
% ==========================================================

figure('Name',sprintf('Figure 2 - 3D Final Path [%s]',W.name), ...
       'Color','white','Position',[980,40,780,660]);
hold on; grid on; box on;

[sox2,soy2,soz2]=ind2sub([Gx,Gy,Gz],find(W.static_map==2));
scatter3(sox2,soy2,soz2,45,[0.55 0.05 0.05],'s','filled','MarkerFaceAlpha',0.55);

[fx2,fy2,fz2]=ind2sub([Gx,Gy,Gz],find(map==1));
if ~isempty(fx2)
    scatter3(fx2,fy2,fz2,8,[1.0 0.85 0.1],'.','MarkerFaceAlpha',0.25);
end

[vx2,vy2,vz2]=ind2sub([Gx,Gy,Gz],find(map==3));
vc=arrayfun(@(a,b,c) visit_cnt(a,b,c), vx2, vy2, vz2);
scatter3(vx2,vy2,vz2,8+vc*5,[0.15 0.75 0.3],'o','filled','MarkerFaceAlpha',0.25);

n_path=size(path_log,1);
cmap_r=jet(n_path);
for s=1:n_path-1
    plot3(path_log(s:s+1,1),path_log(s:s+1,2),path_log(s:s+1,3), ...
          '-','Color',cmap_r(s,:),'LineWidth',1.7);
end

plot3(path_log(1,1),path_log(1,2),path_log(1,3),'p','MarkerSize',14,...
      'MarkerFaceColor',[0.1 0.9 0.2],'Color',[0.1 0.9 0.2]);
plot3(path_log(end,1),path_log(end,2),path_log(end,3),'s','MarkerSize',12,...
      'MarkerFaceColor',[1 0.1 0.1],'Color',[1 0.1 0.1]);

surf(pp_x,pp_y,z_ref*ones(size(pp_x)), ...
     'FaceColor',[0.2 0.9 0.4],'FaceAlpha',0.07,'EdgeColor','none');

colormap(jet); cb2=colorbar; caxis([1 n_path]);
cb2.Label.String='Step (blue=early → red=late)'; cb2.Label.FontSize=9;
xlabel('X [voxel]','FontSize',11); ylabel('Y [voxel]','FontSize',11);
zlabel('Z [voxel]','FontSize',11);
title(sprintf('3D Path  |  Cov:%.1f%%  Steps:%d  |  [%s]', ...
              coverage*100,iter,W.name),'FontSize',10,'FontWeight','bold');
legend({'Static obs','Free (unvisited)','Visited (size∝revisits)', ...
        'Flight path','Start','End','z_{ref} plane'}, ...
       'Location','best','FontSize',8);
xlim([1 Gx]); ylim([1 Gy]); zlim([1 Gz]); view(42,28);

% ==========================================================
%  SECTION 11 — FIGURE 3: COVERAGE PROGRESS
% ==========================================================

figure('Name',sprintf('Figure 3 - Coverage [%s]',W.name), ...
       'Color','white','Position',[40,430,700,420]);
hold on; grid on; box on;
n_ch=length(cov_hist);
fill([0 n_ch n_ch 0],[0 0 COV_TARGET*100 COV_TARGET*100], ...
     [0.90 0.94 1.00],'EdgeColor','none','FaceAlpha',0.7);
fill([0 n_ch n_ch 0],[COV_TARGET*100 COV_TARGET*100 105 105], ...
     [0.90 1.00 0.90],'EdgeColor','none','FaceAlpha',0.7);
plot(1:n_ch,cov_hist*100,'b-','LineWidth',2.2);
yline(COV_TARGET*100,'r--','LineWidth',1.8,...
      'Label',sprintf('Target %.0f%%',COV_TARGET*100),...
      'LabelHorizontalAlignment','left','FontSize',10);
for tgt=[25,50,75]
    idx=find(cov_hist*100>=tgt,1);
    if ~isempty(idx)
        plot(idx,tgt,'ko','MarkerSize',7,'MarkerFaceColor',[1 0.6 0.1]);
        text(idx+8,tgt+2,sprintf('%d%% @ step %d',tgt,idx),'FontSize',9);
    end
end
xlabel('Iteration','FontSize',12,'FontWeight','bold');
ylabel('Coverage [%]','FontSize',12,'FontWeight','bold');
title(sprintf('Coverage  |  %.1f%%  in  %d steps  |  [%s]', ...
              coverage*100,iter,W.name),'FontSize',11,'FontWeight','bold');
ylim([0 105]); xlim([1 n_ch]);

% ==========================================================
%  SECTION 12 — FIGURE 4: ALTITUDE PROFILE
% ==========================================================

figure('Name',sprintf('Figure 4 - Altitude [%s]',W.name), ...
       'Color','white','Position',[760,430,700,420]);
hold on; grid on; box on;
n_steps=size(path_log,1); alt_vals=path_log(:,3);
cmap_alt=jet(Gz);
fill([1 n_steps n_steps 1],[z_ref-1 z_ref-1 z_ref+1 z_ref+1], ...
     [0.85 1.0 0.85],'EdgeColor','none','FaceAlpha',0.7);
for s=1:n_steps-1
    ci=max(1,min(Gz,round(alt_vals(s))));
    plot(s:s+1,alt_vals(s:s+1),'-','Color',cmap_alt(ci,:),'LineWidth',1.8);
end
yline(z_ref,'g--','LineWidth',1.8,'Label','z_{ref}',...
      'LabelHorizontalAlignment','left','FontSize',10);
colormap(jet(Gz)); cb4=colorbar; clim([1 Gz]);
cb4.Label.String='Altitude [voxel]'; cb4.Label.FontSize=10;
xlabel('Step','FontSize',12,'FontWeight','bold');
ylabel('Altitude [voxel]','FontSize',12,'FontWeight','bold');
title(sprintf('Altitude Profile  |  [%s]',W.name),'FontSize',11,'FontWeight','bold');
ylim([0 Gz+1]); xlim([1 n_steps]);

% ==========================================================
%  SECTION 13 — FIGURE 6: 5-TERM COST BREAKDOWN
% ==========================================================

figure('Name',sprintf('Figure 6 - Cost Breakdown [%s]',W.name), ...
       'Color','white','Position',[400,100,850,420]);
hold on; grid on; box on;
steps_c=1:length(log_alt);
area(steps_c,[log_alt(:), log_energy(:), log_global(:), log_dynobs(:)], ...
     'FaceAlpha',0.55);
legend({sprintf('Alt Risk   (\\beta=%.2f)',beta), ...
        sprintf('Energy     (\\gamma=%.2f)',gamma), ...
        sprintf('Global     (\\delta=%.2f)',delta), ...
        sprintf('Birds      (\\epsilon=%.2f)',epsilon)}, ...
       'Location','best','FontSize',10);
xlabel('Iteration','FontSize',12,'FontWeight','bold');
ylabel('Cost component','FontSize',12,'FontWeight','bold');
title(sprintf('5-Term Cost Breakdown  |  [%s]',W.name),'FontSize',11,'FontWeight','bold');
xlim([1 max(length(log_alt),2)]);

fprintf('All figures rendered.\n');

end  % main


% ==========================================================
%  BIRD MANAGER
%
%  Called every W.move_interval algorithm steps.
%  Three operations:
%
%  1. EXPIRE  – remove birds that left the grid OR ran out of lifetime
%  2. MOVE    – advance each survivor along its flight direction
%               with small random drift (±15° in yaw)
%  3. SPAWN   – with probability W.spawn_prob, and if under
%               W.max_alive, inject a new bird from a random
%               boundary face with a random inward direction
%
%  Each bird struct:
%    .pos      [1×3] current voxel (integers)
%    .dir      [1×3] unit direction vector (floats, persistent)
%    .lifetime remaining steps
% ==========================================================

function [flock, dyn_map, dyn_risk] = run_bird_manager( ...
    flock, dyn_map, dyn_risk, W, map, Gx, Gy, Gz)

    % --- EXPIRE birds first ---
    new_flock = struct('pos',{}, 'dir',{}, 'lifetime',{});
    for b = 1:length(flock)
        bird = flock(b);
        p    = bird.pos;

        % Clear from dyn_map at current position
        if p(1)>=1&&p(1)<=Gx&&p(2)>=1&&p(2)<=Gy&&p(3)>=1&&p(3)<=Gz
            dyn_map(p(1),p(2),p(3)) = 0;
        end

        % Check exit conditions
        out_of_grid  = p(1)<1||p(1)>Gx||p(2)<1||p(2)>Gy||p(3)<1||p(3)>Gz;
        out_of_life  = bird.lifetime <= 0;

        if out_of_grid || out_of_life
            % Bird is gone — dyn_risk persists and fades naturally
            % Don't add to new_flock (expired)
        else
            new_flock(end+1) = bird; %#ok<AGROW>
        end
    end
    flock = new_flock;

    % --- MOVE surviving birds ---
    for b = 1:length(flock)
        old_p = flock(b).pos;

        % Clear old position
        if old_p(1)>=1&&old_p(1)<=Gx&&old_p(2)>=1&&old_p(2)<=Gy&&old_p(3)>=1&&old_p(3)<=Gz
            dyn_map(old_p(1),old_p(2),old_p(3)) = 0;
            % Leave risk trail at old position
            dyn_risk(old_p(1),old_p(2),old_p(3)) = 1.0;
        end

        % Slight random drift in direction (birds don't fly perfectly straight)
        % Rotate the XY component by a small random angle (±15 degrees)
        drift_angle = (rand-0.5) * 2 * deg2rad(15);
        dx = flock(b).dir(1);  dy = flock(b).dir(2);
        new_dx = dx*cos(drift_angle) - dy*sin(drift_angle);
        new_dy = dx*sin(drift_angle) + dy*cos(drift_angle);
        new_dir = [new_dx, new_dy, flock(b).dir(3)];
        d_norm  = norm(new_dir);
        if d_norm > 0
            new_dir = new_dir / d_norm;
        end
        flock(b).dir = new_dir;

        % New position = old + direction (rounded to voxel)
        new_p = old_p + W.speed * round(new_dir);

        % If new position is a static obstacle, try to go around
        % (bird deflects slightly rather than teleporting through walls)
        if new_p(1)>=1&&new_p(1)<=Gx&&new_p(2)>=1&&new_p(2)<=Gy && ...
           new_p(3)>=1&&new_p(3)<=Gz && map(new_p(1),new_p(2),new_p(3))==2
            % Try vertical escape first (birds go up/down to avoid)
            alt_up   = old_p + [0 0  1];
            alt_down = old_p + [0 0 -1];
            if in_grid(alt_up,Gx,Gy,Gz)&&map(alt_up(1),alt_up(2),alt_up(3))~=2
                new_p = alt_up;
            elseif in_grid(alt_down,Gx,Gy,Gz)&&map(alt_down(1),alt_down(2),alt_down(3))~=2
                new_p = alt_down;
            else
                new_p = old_p;  % completely blocked — hover briefly
            end
        end

        flock(b).pos      = new_p;
        flock(b).lifetime = flock(b).lifetime - 1;

        % Mark new position in dyn_map (even if out of grid — will expire next call)
        if in_grid(new_p,Gx,Gy,Gz)
            dyn_map(new_p(1),new_p(2),new_p(3)) = 1;
            dyn_risk(new_p(1),new_p(2),new_p(3)) = 1.0;
        end
    end

    % --- SPAWN new birds ---
    if length(flock) < W.max_alive && rand < W.spawn_prob

        % Pick a random boundary face (6 faces of the cuboid)
        face = randi(6);
        switch face
            case 1,  sp = [1,          randi(Gy), randi([1 Gz])];  base_dir = [ 1  0  0];
            case 2,  sp = [Gx,         randi(Gy), randi([1 Gz])];  base_dir = [-1  0  0];
            case 3,  sp = [randi(Gx),  1,         randi([1 Gz])];  base_dir = [ 0  1  0];
            case 4,  sp = [randi(Gx),  Gy,        randi([1 Gz])];  base_dir = [ 0 -1  0];
            case 5,  sp = [randi(Gx),  randi(Gy), 1          ];    base_dir = [ 0  0  1];
            case 6,  sp = [randi(Gx),  randi(Gy), Gz         ];    base_dir = [ 0  0 -1];
        end

        % Don't spawn on a static obstacle
        if map(sp(1),sp(2),sp(3)) == 2
            return;  % skip this spawn attempt
        end

        % Add random spread to inward direction (bird enters at angle)
        spread = 0.35;   % randomness magnitude
        raw_dir = base_dir + spread*(rand(1,3)-0.5);
        d_norm  = norm(raw_dir);
        if d_norm > 0
            raw_dir = raw_dir / d_norm;
        end

        bird.pos      = sp;
        bird.dir      = raw_dir;
        bird.lifetime = W.lifetime_min + randi(W.lifetime_max - W.lifetime_min + 1) - 1;

        flock(end+1) = bird;

        % Mark spawn position
        dyn_map(sp(1),sp(2),sp(3))  = 1;
        dyn_risk(sp(1),sp(2),sp(3)) = 1.0;
    end
end


% ==========================================================
%  HELPER: check if position is inside grid
% ==========================================================

function ok = in_grid(p, Gx, Gy, Gz)
    ok = p(1)>=1&&p(1)<=Gx && p(2)>=1&&p(2)<=Gy && p(3)>=1&&p(3)<=Gz;
end


% ==========================================================
%  APF REPULSIVE SCORE (unified for static + dynamic)
% ==========================================================

function score = apf_score(pos, target, obs_map, Gx, Gy, Gz, D_safe, is_dynamic)
    score=0; dir=target-pos; d_norm=norm(dir);
    if d_norm<1e-9, return; end
    dir_u=dir/d_norm; R=ceil(D_safe)+1;
    for dx=-R:R, for dy=-R:R, for dz=-R:R
        nx=pos(1)+dx; ny=pos(2)+dy; nz=pos(3)+dz;
        if nx<1||nx>Gx||ny<1||ny>Gy||nz<1||nz>Gz, continue; end
        cell_val=obs_map(nx,ny,nz);
        is_obs=(is_dynamic&&cell_val==1)||(~is_dynamic&&cell_val==2);
        if ~is_obs, continue; end
        d=sqrt(dx^2+dy^2+dz^2);
        if d<D_safe&&d>0
            mag=(1/d-1/D_safe)^2;
            proj=dot(dir_u,-[dx,dy,dz]/d);
            score=score+mag*max(proj,0);
        end
    end; end; end
end


% ==========================================================
%  STANDARD HELPERS
% ==========================================================

function nbrs=get_26_neighbors(pos,Gx,Gy,Gz)
    nbrs=zeros(26,3); count=0;
    for dx=-1:1, for dy=-1:1, for dz=-1:1
        if dx==0&&dy==0&&dz==0, continue; end
        nx=pos(1)+dx; ny=pos(2)+dy; nz=pos(3)+dz;
        if nx>=1&&nx<=Gx&&ny>=1&&ny<=Gy&&nz>=1&&nz<=Gz
            count=count+1; nbrs(count,:)=[nx,ny,nz];
        end
    end; end; end
    nbrs=nbrs(1:count,:);
end

function d=nearest_obs_dist(pos,map,Gx,Gy,Gz)
    d=inf; R=4;
    for dx=-R:R, for dy=-R:R, for dz=-R:R
        nx=pos(1)+dx; ny=pos(2)+dy; nz=pos(3)+dz;
        if nx<1||nx>Gx||ny<1||ny>Gy||nz<1||nz>Gz, continue; end
        if map(nx,ny,nz)==2
            dd=sqrt(dx^2+dy^2+dz^2); if dd<d, d=dd; end
        end
    end; end; end
end

function [body_h,rotor_h]=draw_uav_3d(ax,pos)
    ARM=0.6; R_r=0.28; BODY=0.20;
    cx=pos(1); cy=pos(2); cz=pos(3);
    th=linspace(0,2*pi,20);
    body_h(1)=fill3(ax,cx+BODY*cos(th),cy+BODY*sin(th),cz*ones(1,20), ...
                    [0.2 0.2 0.2],'EdgeColor',[0.7 0.7 0.7],'FaceAlpha',0.95);
    arm_dirs=[1 1;1 -1;-1 1;-1 -1]*ARM/sqrt(2);
    for i=1:4
        body_h(i+1)=plot3(ax,[cx cx+arm_dirs(i,1)],[cy cy+arm_dirs(i,2)],[cz cz], ...
                          '-','Color',[0.6 0.6 0.6],'LineWidth',2.5);
    end
    rotor_h=gobjects(4,1);
    for i=1:4
        rx=cx+arm_dirs(i,1); ry=cy+arm_dirs(i,2); th_r=linspace(0,2*pi,16);
        rotor_h(i)=fill3(ax,rx+R_r*cos(th_r),ry+R_r*sin(th_r),cz*ones(1,16), ...
                         [0.5 0.9 1.0],'EdgeColor',[0.2 0.6 0.9],'FaceAlpha',0.6);
    end
end

function move_uav_3d(body_h,rotor_h,pos,ang_deg)
    ARM=0.6; R_r=0.28; BODY=0.20;
    cx=pos(1); cy=pos(2); cz=pos(3);
    th=linspace(0,2*pi,20);
    set(body_h(1),'XData',cx+BODY*cos(th),'YData',cy+BODY*sin(th),'ZData',cz*ones(1,20));
    arm_dirs=[1 1;1 -1;-1 1;-1 -1]*ARM/sqrt(2);
    for i=1:4
        set(body_h(i+1),'XData',[cx cx+arm_dirs(i,1)], ...
                        'YData',[cy cy+arm_dirs(i,2)],'ZData',[cz cz]);
    end
    th_r=linspace(0,2*pi,16)+deg2rad(ang_deg);
    for i=1:4
        rx=cx+arm_dirs(i,1); ry=cy+arm_dirs(i,2);
        set(rotor_h(i),'XData',rx+R_r*cos(th_r),'YData',ry+R_r*sin(th_r),'ZData',cz*ones(1,16));
    end
end
