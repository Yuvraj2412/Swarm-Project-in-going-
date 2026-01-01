% =========================================================================
% MAIN SIMULATION SCRIPT
% =========================================================================
% This script sets up the warehouse, plans paths for 6 drones,
% and then ANIMATES their movement in 3D.
% =========================================================================

clc;
clearvars; % FIX: Changed 'clear all' to 'clearvars' for performance.
close all;

disp('Simulation Started.');

% -------------------------------------------------------------------------
% 1. DEFINE SIMULATION PARAMETERS
% -------------------------------------------------------------------------
WAREHOUSE_X_m = 40; % meters
WAREHOUSE_Y_m = 25; % meters
WAREHOUSE_Z_m = 8;  % meters (must be >= pendant height)
RESOLUTION_m = 0.2; % 20cm voxels. (0.1m is finer but MUCH slower)

% Define drone start positions (in voxels)
% (Helper function to convert meters to voxels)
m2v = @(meters) max(1, round(meters / RESOLUTION_m));

drone_starts = [
    m2v(38), m2v(5), m2v(2);
    m2v(38), m2v(8), m2v(2);
    m2v(38), m2v(11), m2v(2);
    m2v(38), m2v(14), m2v(2);
    m2v(38), m2v(17), m2v(2);
    m2v(38), m2v(20), m2v(2)
];

% -------------------------------------------------------------------------
% 2. CREATE MAP
% -------------------------------------------------------------------------
% This function generates the 3D voxel grid with all obstacles
[Map, goal_spots] = createWarehouseMap(WAREHOUSE_X_m, WAREHOUSE_Y_m, WAREHOUSE_Z_m, RESOLUTION_m);

% Combine all 6 goal spots into one list for assignment
all_goals = [
    goal_spots.table1;
    goal_spots.table2;
    goal_spots.table3(1,:);
    goal_spots.table3(2,:);
    goal_spots.table4(1,:);
    goal_spots.table4(2,:)
];

% -------------------------------------------------------------------------
% 3. TASK ASSIGNMENT (ACO / PSO)
% -------------------------------------------------------------------------
disp('Starting Task Assignment (Placeholder)...');

% %% --- START ACO/PSO LOGIC HERE --- %%
%
% 1. PRE-COMPUTATION:
%    - You need a Cost Matrix [6x6] where Cost(i, j) is the
%      cost for drone 'i' to go to goal 'j'.
%    - Cost(i, j) = A_star_3D(Map, drone_starts(i,:), all_goals(j,:))
%    - This can be slow! You can use a faster, simpler heuristic
%      (like pure Euclidean distance) for the ACO/PSO, and only
%      use the full A* later.
%
% 2. ACO/PSO OPTIMIZATION:
%    - Use your ACO or PSO algorithm to find the optimal permutation
%      (assignment) that minimizes the total cost.
%    - This is a version of the "Assignment Problem".
%
% 3. ASSIGNMENT OUTPUT:
%    - The output should be a 1x6 vector, 'assignments'
%      e.g., assignments = [3, 1, 4, 6, 2, 5]
%      This means:
%        - Drone 1 -> Goal 3
%        - Drone 2 -> Goal 1
%        - etc.
%
% %% --- END ACO/PSO LOGIC --- %%

% For this demo, we will use a simple 1-to-1 assignment
% (Drone 1 -> Goal 1, Drone 2 -> Goal 2, etc.)
assignments = 1:6;
drone_goals = all_goals(assignments, :);

% -------------------------------------------------------------------------
% 4. PATHFINDING (NEW: SWARM-THEN-SPLIT LOGIC)
% -------------------------------------------------------------------------
disp('Starting Swarm-then-Split Pathfinding...');
tic; % Start timer

% Define a "staging point" where the swarm will break formation
staging_point_m = [12, 15, 6]; % 12m(X), 15m(Y), 6m(Z)
staging_point = [m2v(staging_point_m(1)), m2v(staging_point_m(2)), m2v(staging_point_m(3))];

% Check if staging point is valid
if Map(staging_point(1), staging_point(2), staging_point(3)) > 0
    error('Staging point is inside an obstacle! Move it.');
end

% Use one drone (e.g., the 3rd one) as the "leader" for the swarm
leader_start = drone_starts(3, :);

% STAGE 1: Plan the single swarm path from start to staging point
fprintf('Planning Stage 1 (Swarm Path) to [%d,%d,%d]...\n', staging_point);
path_stage1 = A_star_3D(Map, leader_start, staging_point);

if isempty(path_stage1)
    error('No path could be found for the main swarm! Try a different staging point.');
end

% STAGE 2: Plan individual paths from staging point to goals
paths = cell(6, 1);
fprintf('Planning Stage 2 (Individual Paths) from staging point...\n');

for i = 1:6
    goal_point = drone_goals(i, :);
    
    fprintf('Planning for Drone %d: [%d,%d,%d] -> [%d,%d,%d]\n', ...
        i, staging_point, goal_point);
    
    % Plan from staging point to the final goal
    path_stage2 = A_star_3D(Map, staging_point, goal_point);
    
    if isempty(path_stage2)
        warning('No path could be found for Drone %d in Stage 2.', i);
        paths{i} = path_stage1; % Drone just goes to staging point
    else
        % Combine the swarm path and the individual path
        paths{i} = [path_stage1; path_stage2];
    end
end

toc; % End timer
disp('Swarm path planning complete.');

% -------------------------------------------------------------------------
% 4.5 NEW: PAD PATHS FOR SYNCHRONIZED ANIMATION
% -------------------------------------------------------------------------
% To animate all drones together, all path vectors must be the same length.
% We "pad" shorter paths by repeating the final goal position.

disp('Padding paths for animation...');
max_len = 0;
for i = 1:6
    if ~isempty(paths{i})
        max_len = max(max_len, size(paths{i}, 1));
    end
end

% Create a 3D matrix: [step, [x,y,z], drone_id]
all_paths_padded = zeros(max_len, 3, 6);

for i = 1:6
    if ~isempty(paths{i})
        path_i = paths{i};
        len_i = size(path_i, 1);
        
        % Copy the path
        all_paths_padded(1:len_i, :, i) = path_i;
        
        % Pad the end by repeating the last point
        last_point = path_i(end, :);
        all_paths_padded(len_i+1:max_len, :, i) = repmat(last_point, max_len - len_i, 1);
    else
        % If drone has no path, make it stay at its start point
        start_point = drone_starts(i,:);
        all_paths_padded(1:max_len, :, i) = repmat(start_point, max_len, 1);
    end
end


% -------------------------------------------------------------------------
% 5. SAVE FINAL DATA FILE
% -------------------------------------------------------------------------
% This is the "final data file" you requested.
% We now save all 6 paths.
disp('Saving path data to drone_paths.mat...');
save('drone_paths.mat', 'paths', 'all_paths_padded'); 

% -------------------------------------------------------------------------
% 6. VISUALIZATION (3D PLOT)
% -------------------------------------------------------------------------
% This 3D plot is the "graph" of your simulation
disp('Generating 3D plot (Figure 1)...');
figure('Position', [100 100 1200 800]); % Make a larger figure window
ax3D = gca; % Get handle for the 3D axis
hold(ax3D, 'on');

% Plot Obstacles by Type
plot_skip = 10; % Plot 1 in every N voxels to speed up boundaries
plot_skip_car = 2; % Plot 1 in every 2 voxels for car

% Colors
boundaryCol = [0.5 0.5 0.5]; % Grey
tableCol = [0.1 0.8 0.1];    % Green
carCol = [0.8 0.1 0.1];      % Red
pendantCol = [0.8 0.5 0.1];  % Orange
path_colors = lines(6);     % 6 distinct colors for drone paths

% 1. Boundaries (semi-transparent)
[bx, by, bz] = ind2sub(size(Map), find(Map == 1));
scatter3(ax3D, bx(1:plot_skip:end), by(1:plot_skip:end), bz(1:plot_skip:end), ...
    20, 'filled', 'MarkerFaceColor', boundaryCol, 'MarkerFaceAlpha', 0.05, 'DisplayName', 'Boundaries');

% 2. Tables
[tx, ty, tz] = ind2sub(size(Map), find(Map == 2));
scatter3(ax3D, tx, ty, tz, 30, 'filled', 'MarkerFaceColor', tableCol, 'MarkerFaceAlpha', 0.7, 'DisplayName', 'Tables');

% 3. Car
[cx, cy, cz] = ind2sub(size(Map), find(Map == 3));
scatter3(ax3D, cx(1:plot_skip_car:end), cy(1:plot_skip_car:end), cz(1:plot_skip_car:end), ...
    30, 'filled', 'MarkerFaceColor', carCol, 'MarkerFaceAlpha', 0.7, 'DisplayName', 'Car');

% 4. Pendants
[px, py, pz] = ind2sub(size(Map), find(Map == 4));
scatter3(ax3D, px, py, pz, 30, 'filled', 'MarkerFaceColor', pendantCol, 'MarkerFaceAlpha', 0.7, 'DisplayName', 'Pendants');

% Plot FULL Paths (faintly)
for i = 1:6
    if ~isempty(paths{i})
        path_i = paths{i};
        plot3(ax3D, path_i(:,1), path_i(:,2), path_i(:,3), ...
            'Color', [path_colors(i,:), 0.3], 'LineWidth', 1, 'LineStyle', ':', ...
            'DisplayName', sprintf('Drone %d Route', i));
    end
end

% Plot Start and Goal
plot3(ax3D, drone_starts(:,1), drone_starts(:,2), drone_starts(:,3), ...
    'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10, 'DisplayName', 'Start Points');
plot3(ax3D, all_goals(:,1), all_goals(:,2), all_goals(:,3), ...
    'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'All Goal Spots');

% Plot the new Staging Point
plot3(ax3D, staging_point(1), staging_point(2), staging_point(3), ...
    'm*', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Staging Point');

% --- NEW: Initialize Drone Markers for 3D Animation ---
h_drones_3D = gobjects(6, 1);
for i = 1:6
    start_pos = all_paths_padded(1, :, i);
    h_drones_3D(i) = plot3(ax3D, start_pos(1), start_pos(2), start_pos(3), ...
        'o', 'Color', path_colors(i,:), 'MarkerFaceColor', path_colors(i,:), ...
        'MarkerSize', 10, 'DisplayName', sprintf('Drone %d', i));
end
% ---

% Format Plot
title(ax3D, '3D Warehouse Path Plan (Animating...)');
xlabel(ax3D, ['X (voxels, res = ' num2str(RESOLUTION_m) 'm)']);
ylabel(ax3D, ['Y (voxels, res = ' num2str(RESOLUTION_m) 'm)']);
zlabel(ax3D, ['Z (voxels, res = ' num2str(RESOLUTION_m) 'm)']);
legend(ax3D, 'show', 'Location', 'northeastoutside'); % 'show' uses DisplayName properties
axis(ax3D, 'equal'); % Crucial for correct 3D aspect ratio
grid(ax3D, 'on');
view(ax3D, 30, 20); % Set 3D view angle
rotate3d(ax3D, 'on'); % Allow interactive rotation
hold(ax3D, 'off');

% -------------------------------------------------------------------------
% 7. TOP-DOWN 2D PLOT (REMOVED)
% -------------------------------------------------------------------------
% This section has been removed as requested.


% -------------------------------------------------------------------------
% 8. NEW: ANIMATION LOOP
% -------------------------------------------------------------------------
disp('Starting animation...');
% FIX: Slower animation speed
animation_speed_skip = 2; % Animate 1 frame for every 2 calculated steps
animation_pause = 0.02;   % Pause for 0.02s per frame

for t = 1:animation_speed_skip:max_len
    % Update 3D Plot
    for i = 1:6
        pos = all_paths_padded(t, :, i);
        set(h_drones_3D(i), 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
    end
    
    % Update titles with current time step
    title(ax3D, sprintf('3D Warehouse - Animation (Time: %d / %d)', t, max_len));
    
    % Refresh the plots
    drawnow;
    pause(animation_pause); % Pause for a short duration to control speed
end

disp('Animation Finished.');
disp('Simulation Finished.');