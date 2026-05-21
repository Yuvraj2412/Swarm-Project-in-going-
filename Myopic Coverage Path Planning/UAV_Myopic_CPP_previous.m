% =========================================================
%  UAV_Myopic_CPP.m  –  v2  (Animation + Separate Figures)
%  3-D Myopic Coverage Path Planning for UAV
%
%  Extension of:
%    "Risk-Aware Coverage Path Planning for Lunar Micro-Rovers
%     Leveraging Global and Local Environmental Data"
%    Santra et al., arXiv:2404.18721, 2024
%
%  What opens when you run this:
%    Figure 1  –  LIVE ANIMATION  (UAV body + growing trail + voxel map)
%    Figure 2  –  3-D Final Path  (post-run static view)
%    Figure 3  –  Coverage Progress curve
%    Figure 4  –  Altitude Profile
%
%  Run: >> UAV_Myopic_CPP
%  MATLAB R2021a or later recommended.
% =========================================================

function UAV_Myopic_CPP()

clc; clear; close all;
rng(42);

fprintf('============================================\n');
fprintf('  UAV 3-D Myopic Coverage Path Planning\n');
fprintf('============================================\n\n');

% -------------------------------------------------------
%  1.  PARAMETERS
% -------------------------------------------------------

% Environment
Gx = 20;  Gy = 20;  Gz = 10;
OBS_RATIO = 0.05;
CELL_SIZE = 1.0;

% UAV physics
m_uav  = 1.5;
g_acc  = 9.81;
rho    = 1.225;
A_rot  = 0.10;
A_drag = 0.05;
Cd     = 0.50;
v_nom  = 1.0;
z_ref  = 5;

% Cost weights  (alpha + beta + gamma = 1)
alpha = 0.40;
beta  = 0.35;
gam   = 0.25;

% Altitude-risk sub-weights
w1 = 1.0;
w2 = 2.0;
w3 = 0.5;

% Visited-cell penalty
MC_VIS = 1.5;

% APF
D_SAFE    = 2.5;
APF_SCALE = 0.05;

% Run control
COV_TARGET = 0.90;
MAX_ITER   = Gx*Gy*Gz * 5;

% Animation control
ANIM_SKIP  = 1;   % draw every Nth step (1=every step, slowest)
ROTOR_SPD  = 5;  % degrees per frame for spinning rotors

% -------------------------------------------------------
%  2.  DERIVED QUANTITIES
% -------------------------------------------------------

P_hover = (m_uav * g_acc)^1.5 / sqrt(2 * rho * A_rot);

fprintf('UAV Parameters:\n');
fprintf('  Mass            : %.2f kg\n',  m_uav);
fprintf('  Hover Power     : %.2f W\n',  P_hover);
fprintf('  Reference alt.  : %d voxels\n', z_ref);
fprintf('  Grid            : %d x %d x %d\n', Gx, Gy, Gz);
fprintf('  Animation skip  : every %d steps\n\n', ANIM_SKIP);

% -------------------------------------------------------
%  3.  ENVIRONMENT
% -------------------------------------------------------

map       = zeros(Gx, Gy, Gz, 'uint8');
visit_cnt = zeros(Gx, Gy, Gz);

n_obs  = floor(OBS_RATIO * Gx * Gy * Gz);
placed = 0;                                            %Problem in this step
while placed < n_obs
    ox = randi(Gx);  oy = randi(Gy);  oz = randi(Gz);
    if ox <= 3 && oy <= 3,  continue;  end
    if map(ox, oy, oz) == 0
        map(ox, oy, oz) = 2;
        placed = placed + 1;
    end
end

free_total = sum(map(:) ~= 2);

% -------------------------------------------------------
%  4.  INITIAL UAV STATE
% -------------------------------------------------------

pos = [2, 2, z_ref];
map(pos(1), pos(2), pos(3)) = 3;
visit_cnt(pos(1), pos(2), pos(3)) = 1;

path_log = pos;      %what he hell these path logs do?
E_total  = 0;
cov_hist = [];
iter     = 0;
coverage = 0;

% -------------------------------------------------------
%  5.  FIGURE 1  LIVE ANIMATION SETUP
% -------------------------------------------------------

fig_anim = figure('Name','Figure 1 - UAV Live Animation', ...
                  'Color',[0.05 0.05 0.12], ...
                  'Position',[40, 40, 900, 700]);

ax_a = axes('Parent', fig_anim, ...
            'Color',  [0.05 0.05 0.12], ...
            'XColor', [0.7 0.7 0.7], ...
            'YColor', [0.7 0.7 0.7], ...
            'ZColor', [0.7 0.7 0.7], ...
            'GridColor',[0.3 0.3 0.3], ...
            'GridAlpha', 0.4);
hold(ax_a,'on');  grid(ax_a,'on');  box(ax_a,'on');

% Obstacle voxels (drawn once)
[ox_v, oy_v, oz_v] = ind2sub([Gx,Gy,Gz], find(map==2));
scatter3(ax_a, ox_v, oy_v, oz_v, 40, [0.9 0.3 0.2], 'filled', ...
         'MarkerFaceAlpha', 0.55);

% Reference altitude plane
[pp_x, pp_y] = meshgrid(1:Gx, 1:Gy);
surf(ax_a, pp_x, pp_y, z_ref*ones(size(pp_x)), ...
     'FaceColor',[0.2 0.8 0.4], 'FaceAlpha',0.06, 'EdgeColor','none');

% Visited-voxel cloud (grows during animation)
h_vis = scatter3(ax_a, nan, nan, nan, 20, [0.3 0.7 1.0], 'filled', ...
                 'MarkerFaceAlpha', 0.30);

% Growing flight trail
h_trail = plot3(ax_a, pos(1), pos(2), pos(3), ...
                '-', 'Color',[0.4 0.6 1.0 0.7], 'LineWidth', 1.2);

% Start marker
plot3(ax_a, pos(1), pos(2), pos(3), 'o', ...
      'Color',[0.1 1.0 0.3], 'MarkerSize',10, ...
      'MarkerFaceColor',[0.1 1.0 0.3], 'LineWidth',1.5);

% UAV body handles
[uav_h, rotor_h] = draw_uav_3d(ax_a, pos);

% HUD
h_hud = text(ax_a, 1.5, 1.5, Gz+0.5, 'Coverage: 0.0%   Step: 0', ...
             'Color',[1 1 0.4], 'FontSize',11, ...
             'FontWeight','bold', 'FontName','Courier');

xlabel(ax_a,'X [voxel]','Color',[0.8 0.8 0.8],'FontSize',10);
ylabel(ax_a,'Y [voxel]','Color',[0.8 0.8 0.8],'FontSize',10);
zlabel(ax_a,'Z [voxel]','Color',[0.8 0.8 0.8],'FontSize',10);
title(ax_a,'UAV Myopic Coverage  -  Live 3D Animation', ...
      'Color',[1 1 1],'FontSize',12,'FontWeight','bold');
xlim(ax_a,[1 Gx]);  ylim(ax_a,[1 Gy]);  zlim(ax_a,[1 Gz]);
view(ax_a, 42, 28);

rotor_angle = 0;

% -------------------------------------------------------
%  6.  MAIN COVERAGE LOOP  +  LIVE ANIMATION
% -------------------------------------------------------

fprintf('Running coverage loop with live animation...\n');
fprintf('(Close Figure 1 to stop early)\n\n');

while coverage < COV_TARGET && iter < MAX_ITER

    if ~ishandle(fig_anim),  break;  end

    iter = iter + 1;

    % 6a. Sense 26 neighbours
    nbrs = get_26_neighbors(pos, Gx, Gy, Gz);
    for k = 1:size(nbrs,1)
        nx=nbrs(k,1); ny=nbrs(k,2); nz=nbrs(k,3);
        if map(nx,ny,nz)==0,  map(nx,ny,nz)=1;  end
    end

    % 6b. Cost evaluation
    min_cost = inf;
    best     = [];

    for k = 1:size(nbrs,1)
        nx=nbrs(k,1); ny=nbrs(k,2); nz=nbrs(k,3);
        if map(nx,ny,nz)==2,  continue;  end

        mc_static = 1.0;
        Vi        = visit_cnt(nx,ny,nz);
        d_obs     = nearest_obs_dist([nx,ny,nz], map, Gx, Gy, Gz);
        dz        = nz - pos(3);

        mc_alt = w1*abs(dz) + w2*(1/max(d_obs,0.5)) + w3*abs(nz-z_ref);

        dist_m    = sqrt((nx-pos(1))^2+(ny-pos(2))^2+dz^2)*CELL_SIZE;
        t_step    = dist_m / v_nom;          %What's the diffrence between drag and hover?
        F_drag    = 0.5*Cd*rho*A_drag*v_nom^2;
        E_step    = P_hover*t_step + m_uav*g_acc*max(dz,0)*CELL_SIZE + F_drag*dist_m;
        mc_energy = E_step / 200;       %why?

        apf_score = apf_repulsive_score(pos,[nx,ny,nz],map,Gx,Gy,Gz,D_SAFE);

        total_cost = alpha*(mc_static + MC_VIS*Vi) ...
                   + beta*mc_alt ...
                   + gam*mc_energy ...
                   + APF_SCALE*apf_score;

        if total_cost < min_cost
            min_cost = total_cost;
            best     = [nx,ny,nz];
        end
    end

    if isempty(best),  break;  end

    % 6c. Move
    dz_move   = best(3)-pos(3);
    dist_move = sqrt(sum((best-pos).^2))*CELL_SIZE;
    t_move    = dist_move / v_nom;
    F_drag_m  = 0.5*Cd*rho*A_drag*v_nom^2;
    E_move    = P_hover*t_move + m_uav*g_acc*max(dz_move,0)*CELL_SIZE + F_drag_m*dist_move;
    E_total   = E_total + E_move;

    pos = best;
    visit_cnt(pos(1),pos(2),pos(3)) = visit_cnt(pos(1),pos(2),pos(3))+1;
    if map(pos(1),pos(2),pos(3)) ~= 2
        map(pos(1),pos(2),pos(3)) = 3;
    end
    path_log(end+1,:) = pos; %#ok<AGROW>

    visited  = sum(map(:)==3);
    coverage = visited / free_total;
    cov_hist(end+1) = coverage; %#ok<AGROW>

    % 6d. Animate
    if mod(iter, ANIM_SKIP)==0 && ishandle(fig_anim)
        [vx,vy,vz] = ind2sub([Gx,Gy,Gz], find(map==3));
        set(h_vis,   'XData',vx, 'YData',vy, 'ZData',vz);
        set(h_trail, 'XData',path_log(:,1), ...
                     'YData',path_log(:,2), ...
                     'ZData',path_log(:,3));
        move_uav_3d(uav_h, rotor_h, pos, rotor_angle);
        rotor_angle = mod(rotor_angle + ROTOR_SPD, 360);
        set(h_hud,'String', ...
            sprintf('Coverage: %5.1f%%   Step: %d   Energy: %.0f J', ...
                    coverage*100, iter, E_total));
        drawnow limitrate;
    end

    if mod(iter,200)==0
        fprintf('  Iter %4d | Coverage %.1f%%\n', iter, coverage*100);
    end
end

% Final animation frame
if ishandle(fig_anim)
    [vx,vy,vz] = ind2sub([Gx,Gy,Gz], find(map==3));
    set(h_vis,   'XData',vx, 'YData',vy, 'ZData',vz);
    set(h_trail, 'XData',path_log(:,1),'YData',path_log(:,2),'ZData',path_log(:,3));
    move_uav_3d(uav_h, rotor_h, pos, rotor_angle);
    set(h_hud,'String', sprintf('DONE  Coverage: %.1f%%   Steps: %d   Energy: %.0f J', ...
              coverage*100, iter, E_total));
    plot3(ax_a, pos(1),pos(2),pos(3),'s','Color',[1 0.2 0.2],'MarkerSize',12,...
          'MarkerFaceColor',[1 0.2 0.2],'LineWidth',1.5);
    drawnow;
end

% -------------------------------------------------------
%  7.  RESULTS
% -------------------------------------------------------
fprintf('\n--- Simulation Results ---\n');
fprintf('  Total iterations     : %d\n',   iter);
fprintf('  Final coverage       : %.1f%%\n', coverage*100);
fprintf('  Total energy used    : %.2f J\n', E_total);
fprintf('  Path length ratio    : %.3f\n',   iter/free_total);
fprintf('  (Optimal = 1.0 per paper)\n\n');
fprintf('Rendering static figures 2, 3, 4...\n');

% -------------------------------------------------------
%  8.  FIGURE 2  3-D FINAL PATH
% -------------------------------------------------------

figure('Name','Figure 2 - 3D Final Path', ...
       'Color','white', 'Position',[980, 40, 750, 650]);

hold on; grid on; box on;

[ox2,oy2,oz2] = ind2sub([Gx,Gy,Gz], find(map==2));
scatter3(ox2,oy2,oz2, 35, [0.15 0.15 0.15], 'filled', 'MarkerFaceAlpha',0.4);

[vx2,vy2,vz2] = ind2sub([Gx,Gy,Gz], find(map==3));
scatter3(vx2,vy2,vz2, 15, [0.3 0.7 1.0], 'filled', 'MarkerFaceAlpha',0.22);

plot3(path_log(:,1), path_log(:,2), path_log(:,3), 'b-', 'LineWidth',1.5);

plot3(path_log(1,1),   path_log(1,2),   path_log(1,3), 'go', ...
      'MarkerSize',11,'MarkerFaceColor',[0.1 0.8 0.1],'LineWidth',1.5);
plot3(path_log(end,1), path_log(end,2), path_log(end,3), 'rs', ...
      'MarkerSize',11,'MarkerFaceColor',[0.9 0.1 0.1],'LineWidth',1.5);

[pp_x2,pp_y2] = meshgrid(1:Gx,1:Gy);
surf(pp_x2, pp_y2, z_ref*ones(size(pp_x2)), ...
     'FaceColor',[0.2 0.9 0.2],'FaceAlpha',0.08,'EdgeColor','none');

xlabel('X [voxel]','FontSize',11);
ylabel('Y [voxel]','FontSize',11);
zlabel('Z [voxel]','FontSize',11);
title(sprintf('3D UAV Flight Path  |  Coverage: %.1f%%  |  Steps: %d', ...
              coverage*100, iter), 'FontSize',12,'FontWeight','bold');
legend({'Obstacles','Visited voxels','Flight path','Start','End','z_{ref} plane'}, ...
       'Location','best','FontSize',9);
xlim([1 Gx]); ylim([1 Gy]); zlim([1 Gz]);
view(42,28);

% -------------------------------------------------------
%  9.  FIGURE 3  COVERAGE PROGRESS
% -------------------------------------------------------

figure('Name','Figure 3 - Coverage Progress', ...
       'Color','white', 'Position',[40, 420, 700, 420]);

hold on; grid on; box on;

fill([0 length(cov_hist) length(cov_hist) 0], ...
     [0 0 COV_TARGET*100 COV_TARGET*100], ...
     [0.9 0.95 1.0],'EdgeColor','none','FaceAlpha',0.6);
fill([0 length(cov_hist) length(cov_hist) 0], ...
     [COV_TARGET*100 COV_TARGET*100 105 105], ...
     [0.9 1.0 0.9],'EdgeColor','none','FaceAlpha',0.6);

plot(1:length(cov_hist), cov_hist*100, 'b-', 'LineWidth', 2.2);

yline(COV_TARGET*100,'r--','LineWidth',1.8, ...
      'Label',sprintf('Target %.0f%%',COV_TARGET*100), ...
      'LabelHorizontalAlignment','left','FontSize',10);

for tgt = [25, 50, 75]
    idx = find(cov_hist*100 >= tgt, 1);
    if ~isempty(idx)
        plot(idx, tgt, 'ko','MarkerSize',6,'MarkerFaceColor',[1 0.6 0.1]);
        text(idx+5, tgt+2, sprintf('%d%% @ step %d',tgt,idx), ...
             'FontSize',8,'Color',[0.4 0.2 0]);
    end
end

xlabel('Iteration','FontSize',12,'FontWeight','bold');
ylabel('Coverage [%]','FontSize',12,'FontWeight','bold');
title(sprintf('Coverage Progress  |  Final: %.1f%%  in  %d steps', ...
              coverage*100, iter),'FontSize',12,'FontWeight','bold');
ylim([0 105]);
xlim([1 length(cov_hist)]);

% -------------------------------------------------------
%  10.  FIGURE 4  ALTITUDE PROFILE
% -------------------------------------------------------

figure('Name','Figure 4 - Altitude Profile', ...
       'Color','white', 'Position',[760, 420, 700, 420]);

hold on; grid on; box on;

n_steps  = size(path_log,1);
steps    = 1:n_steps;
alt_vals = path_log(:,3);
cmap     = jet(Gz);

% Cruise band shading
fill([1 n_steps n_steps 1], [z_ref-1 z_ref-1 z_ref+1 z_ref+1], ...
     [0.85 1.0 0.85],'EdgeColor','none','FaceAlpha',0.7);

% Colour-coded altitude line
for s = 1:n_steps-1
    ci = max(1, min(Gz, round(alt_vals(s))));
    plot(steps(s:s+1), alt_vals(s:s+1), '-', ...
         'Color', cmap(ci,:), 'LineWidth', 1.8);
end

yline(z_ref,'g--','LineWidth',1.8,...
      'Label','z_{ref} (cruise)',...
      'LabelHorizontalAlignment','left','FontSize',10);

colormap(jet(Gz));
cb = colorbar;
clim([1 Gz]);
cb.Label.String  = 'Altitude [voxel]';
cb.Label.FontSize = 10;

xlabel('Step','FontSize',12,'FontWeight','bold');
ylabel('Altitude [voxel]','FontSize',12,'FontWeight','bold');
title('UAV Altitude Profile  -  Colour = Height', ...
      'FontSize',12,'FontWeight','bold');
ylim([0 Gz+1]);
xlim([1 n_steps]);

fprintf('\nAll 4 figures rendered.\n');
fprintf('Fig 1 = Live Animation  |  Fig 2 = 3D Path\n');
fprintf('Fig 3 = Coverage        |  Fig 4 = Altitude\n');

end  % main function


% ==========================================================
%  UAV DRAWING HELPERS
% ==========================================================

function [body_h, rotor_h] = draw_uav_3d(ax, pos)
% Draws a quadrotor: central disc + 4 arms + 4 spinning rotors.

    ARM  = 0.6;
    R_r  = 0.28;
    BODY = 0.20;

    cx = pos(1);  cy = pos(2);  cz = pos(3);

    th  = linspace(0, 2*pi, 20);
    bx  = cx + BODY*cos(th);
    by  = cy + BODY*sin(th);
    bz  = cz * ones(size(th));
    body_h(1) = fill3(ax, bx, by, bz, [0.15 0.15 0.15], ...
                      'EdgeColor','none','FaceAlpha',0.95);

    arm_dirs = [1 1; 1 -1; -1 1; -1 -1] * ARM / sqrt(2);
    for i = 1:4
        body_h(i+1) = plot3(ax, ...
            [cx, cx+arm_dirs(i,1)], ...
            [cy, cy+arm_dirs(i,2)], ...
            [cz, cz], ...
            '-','Color',[0.3 0.3 0.3],'LineWidth',2.5);
    end

    rotor_h = gobjects(4,1);
    for i = 1:4
        rx   = cx + arm_dirs(i,1);
        ry   = cy + arm_dirs(i,2);
        th_r = linspace(0, 2*pi, 16);
        rotor_h(i) = fill3(ax, rx+R_r*cos(th_r), ry+R_r*sin(th_r), ...
                           cz*ones(1,16), [0.6 0.9 1.0], ...
                           'EdgeColor',[0.2 0.5 0.8], ...
                           'FaceAlpha',0.55,'LineWidth',1.0);
    end
end


function move_uav_3d(body_h, rotor_h, pos, rotor_angle_deg)
% Re-positions the UAV graphics to current pos with rotor spin.

    ARM  = 0.6;
    R_r  = 0.28;
    BODY = 0.20;

    cx = pos(1);  cy = pos(2);  cz = pos(3);

    th = linspace(0, 2*pi, 20);
    set(body_h(1), 'XData', cx+BODY*cos(th), ...
                   'YData', cy+BODY*sin(th), ...
                   'ZData', cz*ones(1,20));

    arm_dirs = [1 1; 1 -1; -1 1; -1 -1] * ARM / sqrt(2);
    for i = 1:4
        set(body_h(i+1), ...
            'XData',[cx, cx+arm_dirs(i,1)], ...
            'YData',[cy, cy+arm_dirs(i,2)], ...
            'ZData',[cz, cz]);
    end

    ang_rad = deg2rad(rotor_angle_deg);
    th_r    = linspace(0, 2*pi, 16) + ang_rad;
    for i = 1:4
        rx = cx + arm_dirs(i,1);
        ry = cy + arm_dirs(i,2);
        set(rotor_h(i), ...
            'XData', rx + R_r*cos(th_r), ...
            'YData', ry + R_r*sin(th_r), ...
            'ZData', cz*ones(1,16));
    end
end


% ==========================================================
%  CORE ALGORITHM HELPERS
% ==========================================================

function nbrs = get_26_neighbors(pos, Gx, Gy, Gz)
    nbrs  = zeros(26,3);
    count = 0;
    for dx = -1:1
        for dy = -1:1
            for dz = -1:1
                if dx==0 && dy==0 && dz==0,  continue;  end
                nx=pos(1)+dx; ny=pos(2)+dy; nz=pos(3)+dz;
                if nx>=1&&nx<=Gx&&ny>=1&&ny<=Gy&&nz>=1&&nz<=Gz
                    count=count+1;
                    nbrs(count,:)=[nx,ny,nz];
                end
            end
        end
    end
    nbrs = nbrs(1:count,:);
end


function d = nearest_obs_dist(pos, map, Gx, Gy, Gz)
    d = inf;  R = 4;
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
    score = 0;
    dir   = target - pos;
    d_norm = norm(dir);
    if d_norm<1e-9, return; end
    dir_u = dir / d_norm;
    for dx=-2:2
        for dy=-2:2
            for dz=-2:2
                nx=pos(1)+dx; ny=pos(2)+dy; nz=pos(3)+dz;
                if nx<1||nx>Gx||ny<1||ny>Gy||nz<1||nz>Gz, continue; end
                if map(nx,ny,nz)==2
                    d=sqrt(dx^2+dy^2+dz^2);
                    if d<D_safe && d>0
                        mag  = (1/d - 1/D_safe)^2;
                        proj = dot(dir_u, -[dx,dy,dz]/d);
                        score = score + mag*max(proj,0);
                    end
                end
            end
        end
    end
end
