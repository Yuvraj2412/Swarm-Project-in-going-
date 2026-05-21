% =========================================================
%  UAV_WorldBuilder.m  –  v2
%
%  Generates world .mat files used by UAV_Myopic_CPP.m.
%  Run ONCE to create all world files.
%
%  DYNAMIC OBSTACLE MODEL (Bird-Like Transient Behaviour):
%  --------------------------------------------------------
%  Dynamic obstacles in this version are NOT permanent
%  residents of the world. They behave like birds or drones
%  passing through:
%
%    1. They SPAWN at a random world boundary (edge face)
%    2. They FLY in a roughly straight line with slight drift
%    3. They DISAPPEAR when:
%         a) They exit the grid on the other side, OR
%         b) Their lifetime (steps) expires
%    4. New ones SPAWN periodically with a spawn probability
%
%  The world file does NOT store fixed dynamic obstacle
%  positions. Instead it stores the SPAWN PARAMETERS:
%    - spawn_prob:   probability per step of a new one appearing
%    - max_alive:    max simultaneous transient obstacles
%    - lifetime_min/max: steps before forced expiry
%    - speed:        voxels per movement event
%
%  The main algorithm manages the live flock at runtime.
%
%  WORLDS:
%    world_01.mat  –  Open Field     (sparse, 3 hazards)
%    world_02.mat  –  Urban Block    (buildings, 5 hazards)
%    world_03.mat  –  Corridor Maze  (walls+gaps, 2 hazards)
%
%  HOW TO ADD A WORLD:
%    Copy build_world_03(), rename to build_world_04(),
%    edit geometry, add build_world_04(); below, re-run.
%
%  Run: >> UAV_WorldBuilder
% =========================================================

function UAV_WorldBuilder()

    fprintf('==============================================\n');
    fprintf('  UAV World Builder  v2\n');
    fprintf('==============================================\n\n');

    build_world_01();
    build_world_02();
    build_world_03();

    fprintf('\n==============================================\n');
    fprintf('  Done. Files created:\n');
    fprintf('    world_01.mat  –  Open Field\n');
    fprintf('    world_02.mat  –  Urban Block\n');
    fprintf('    world_03.mat  –  Corridor Maze\n');
    fprintf('\n  In UAV_Myopic_CPP.m change one line:\n');
    fprintf('    WORLD_FILE = ''world_01.mat'';\n');
    fprintf('==============================================\n');

end


% ==========================================================
%  WORLD STRUCT FIELDS
%
%  W.name            string description
%  W.Gx, Gy, Gz      grid dimensions
%  W.z_ref           cruise altitude [voxels]
%  W.start_pos       [1×3] UAV start voxel
%  W.static_map      uint8 [Gx×Gy×Gz]: 0=unknown, 2=static obs
%  W.OBS_RATIO       obstacle density (info only)
%  W.global_risk     float [Gx×Gy×Gz] in [0,1]
%  W.hazard_centres  [M×3] hazard zone centres
%
%  --- Bird/transient dynamic obstacle spawn parameters ---
%  W.spawn_prob      probability [0,1] of spawning one per step
%  W.max_alive       max simultaneous transient obstacles
%  W.lifetime_min    minimum steps before forced expiry
%  W.lifetime_max    maximum steps before forced expiry
%  W.speed           voxels moved per movement event
%  W.move_interval   move transients every N algorithm steps
% ==========================================================


% ==========================================================
%  WORLD 01  –  OPEN FIELD
% ==========================================================

function build_world_01()

    rng(42);

    W.name      = 'Open Field';
    W.Gx        = 20;   W.Gy = 20;   W.Gz = 10;
    W.z_ref     = 5;
    W.start_pos = [2, 2, 5];
    W.OBS_RATIO = 0.05;

    % Static obstacles: random scatter
    W.static_map = zeros(W.Gx, W.Gy, W.Gz, 'uint8');
    n_obs  = floor(W.OBS_RATIO * W.Gx * W.Gy * W.Gz);
    placed = 0;
    while placed < n_obs
        ox=randi(W.Gx); oy=randi(W.Gy); oz=randi(W.Gz);
        if ox<=3 && oy<=3, continue; end
        if W.static_map(ox,oy,oz)==0
            W.static_map(ox,oy,oz)=2;
            placed=placed+1;
        end
    end

    % Bird/transient spawn parameters
    % Open field: moderate bird traffic, mid-altitude birds
    W.spawn_prob    = 0.08;    % 8% chance per step a new bird appears
    W.max_alive     = 4;       % at most 4 birds in world at once
    W.lifetime_min  = 15;      % bird lives at least 15 steps
    W.lifetime_max  = 35;      % bird lives at most 35 steps
    W.speed         = 1;       % moves 1 voxel per movement event
    W.move_interval = 4;       % birds move every 4 algorithm steps

    % Global risk: 3 Gaussian hazard bumps
    W.hazard_centres = [10, 10, 5;
                         4, 16, 4;
                        16,  4, 6];
    W.global_risk = make_global_risk(W.Gx,W.Gy,W.Gz,W.hazard_centres,3.0);

    save('world_01.mat','W');
    fprintf('  world_01.mat saved  [%s]\n', W.name);
end


% ==========================================================
%  WORLD 02  –  URBAN BLOCK
% ==========================================================

function build_world_02()

    rng(7);

    W.name      = 'Urban Block';
    W.Gx        = 20;   W.Gy = 20;   W.Gz = 10;
    W.z_ref     = 6;
    W.start_pos = [2, 2, 6];
    W.OBS_RATIO = 0.12;

    W.static_map = zeros(W.Gx, W.Gy, W.Gz, 'uint8');

    % Four building blocks
    buildings = [4,  4, 4, 4;
                 4, 13, 4, 4;
                13,  4, 4, 4;
                13, 13, 4, 4];
    for b = 1:size(buildings,1)
        bx=buildings(b,1); by=buildings(b,2);
        bw=buildings(b,3); bh=buildings(b,4);
        for ix=bx:bx+bw-1
            for iy=by:by+bh-1
                for iz=1:W.Gz-2
                    if ix>=1&&ix<=W.Gx&&iy>=1&&iy<=W.Gy
                        if ~(ix<=3&&iy<=3)
                            W.static_map(ix,iy,iz)=2;
                        end
                    end
                end
            end
        end
    end

    % Scatter debris
    n_sc=floor(0.03*W.Gx*W.Gy*W.Gz); placed=0;
    while placed<n_sc
        ox=randi(W.Gx); oy=randi(W.Gy); oz=randi(W.Gz);
        if ox<=3&&oy<=3, continue; end
        if W.static_map(ox,oy,oz)==0
            W.static_map(ox,oy,oz)=2; placed=placed+1;
        end
    end

    % Urban: more birds (pigeons, delivery drones), shorter lived
    W.spawn_prob    = 0.12;
    W.max_alive     = 6;
    W.lifetime_min  = 10;
    W.lifetime_max  = 25;
    W.speed         = 1;
    W.move_interval = 3;

    W.hazard_centres = [4,  4, W.z_ref;
                        4, 17, W.z_ref;
                       17,  4, W.z_ref;
                       17, 17, W.z_ref;
                       10, 10, W.z_ref];
    W.global_risk = make_global_risk(W.Gx,W.Gy,W.Gz,W.hazard_centres,2.5);

    save('world_02.mat','W');
    fprintf('  world_02.mat saved  [%s]\n', W.name);
end


% ==========================================================
%  WORLD 03  –  CORRIDOR MAZE
% ==========================================================

function build_world_03()

    rng(13);

    W.name      = 'Corridor Maze';
    W.Gx        = 20;   W.Gy = 20;   W.Gz = 10;
    W.z_ref     = 4;
    W.start_pos = [2, 2, 4];
    W.OBS_RATIO = 0.15;

    W.static_map = zeros(W.Gx, W.Gy, W.Gz, 'uint8');

    % Wall barriers with gaps
    for iz=1:W.Gz
        for ix=1:14           % Wall A  y=7
            if ~(ix==9||ix==10), W.static_map(ix,7,iz)=2; end
        end
        for ix=7:W.Gx         % Wall B  y=14
            if ~(ix==13||ix==14), W.static_map(ix,14,iz)=2; end
        end
        for iy=1:7            % Wall C  x=10
            if ~(iy==4||iy==5), W.static_map(10,iy,iz)=2; end
        end
    end
    W.static_map(1:3,1:3,:)=0;   % protect start

    n_sc=floor(0.03*W.Gx*W.Gy*W.Gz); placed=0;
    while placed<n_sc
        ox=randi(W.Gx); oy=randi(W.Gy); oz=randi(W.Gz);
        if ox<=3&&oy<=3, continue; end
        if W.static_map(ox,oy,oz)==0
            W.static_map(ox,oy,oz)=2; placed=placed+1;
        end
    end

    % Maze corridors: fewer birds, they come through the gaps
    W.spawn_prob    = 0.06;
    W.max_alive     = 3;
    W.lifetime_min  = 20;
    W.lifetime_max  = 40;
    W.speed         = 1;
    W.move_interval = 5;

    W.hazard_centres = [9,  7, W.z_ref;
                       13, 14, W.z_ref];
    W.global_risk = make_global_risk(W.Gx,W.Gy,W.Gz,W.hazard_centres,2.0);

    save('world_03.mat','W');
    fprintf('  world_03.mat saved  [%s]\n', W.name);
end


% ==========================================================
%  SHARED: GLOBAL RISK MAP BUILDER
% ==========================================================

function risk = make_global_risk(Gx, Gy, Gz, centres, strength)
    risk   = zeros(Gx, Gy, Gz);
    spread = 8.0;
    for i=1:Gx, for j=1:Gy, for k=1:Gz
        r=0;
        for h=1:size(centres,1)
            dx=i-centres(h,1); dy=j-centres(h,2); dz=k-centres(h,3);
            r=r+strength*exp(-(dx^2+dy^2+dz^2)/spread);
        end
        risk(i,j,k)=r;
    end; end; end
    mx=max(risk(:));
    if mx>0, risk=risk/mx; end
end
