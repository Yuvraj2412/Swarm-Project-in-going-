%% Swarm Path & Formation Transition Logic - MATLAB Script
%
% This script simulates a centralized swarm commander.
%
% UPDATE V3.1:
% - Includes simulated sensor readings for all 6 UAVs.
% - Adds a new "FOV Overlap" sensor.
% - Generates a separate sensor plot figure for EACH of the 6 UAVs.
% - Changed Overlap sensor plot color to Magenta for visibility.

clc;
clear;
close all;

%% --- Physical & Simulation Parameters ---
num_uavs = 6;
angular_speed = 0.5;    % Speed of rotation (radians per second)
sim_time = 30.0;          % Total simulation time (seconds)
dt = 0.05;              % Simulation time step (0.05s = 20 Hz)
z_fixed = 10.0;           % Fixed altitude
fov_angle_deg = 120.0;    % Camera Field of View (degrees)

%% --- Sensor Noise Parameters ---
gps_noise_std = 0.1;       % (meters)
altimeter_noise_std = 0.05;  % (meters)
imu_noise_std = 0.02;        % (m/s^2)

%% ---DERIVED SAFETY CONSTRAINT---
fov_radius = z_fixed * tan(deg2rad(fov_angle_deg / 2));
min_separation_dist = fov_radius; % This is our "safe radius"
fprintf('--- Swarm Safety Parameters ---\n');
fprintf('Altitude (z): %.2f m\n', z_fixed);
fprintf('Camera FOV: %.1f deg\n', fov_angle_deg);
fprintf('==> FOV Footprint Radius: %.2f m\n', fov_radius);
fprintf('==> Minimum Separation Distance: %.2f m\n', min_separation_dist);
fprintf('----------------------------------\n');

%% --- Formation Parameters ---
circle_radius = min_separation_dist; 
line_spacing = min_separation_dist;  

%% --- Path Parameters ---
start_pos = [0, 0];       % Path start (X, Y)
end_pos = [100, 100];     % Path end (X, Y)
path_duration = 24.0;     % Time to complete the path
wiggle_amplitude = 2.5;   % Max deviation for the "wiggle" (meters)
wiggle_frequency = 3.0;   % Number of full wiggles over the path

%% --- Formation Transition Parameters ---
t_start_transition_to_line = 3.0;  
t_end_transition_to_line = 9.0;    
t_start_transition_to_circle = 18.0; 
t_end_transition_to_circle = 24.0; 

%% --- Initialization ---
setpoints = repmat(struct('position', [0, 0, z_fixed]), num_uavs, 1);
t_vec = 0:dt:sim_time;
num_steps = length(t_vec);

% --- UAV State Initialization ---
uav_states = repmat(struct(...
    'position', [0, 0, z_fixed], ...
    'velocity', [0, 0, 0], ...
    'acceleration', [0, 0, 0]), num_uavs, 1);

% Initialize to starting circle formation (with fix)
initial_angles = (0:num_uavs-1)' * (2*pi / num_uavs);
x_positions = start_pos(1) + circle_radius * cos(initial_angles);
y_positions = start_pos(2) + circle_radius * sin(initial_angles);
for i = 1:num_uavs
    uav_states(i).position = [x_positions(i), y_positions(i), z_fixed];
end

% --- Sensor Log Initialization ---
sensor_log.time = t_vec;
sensor_log.gps_x = zeros(num_steps, num_uavs);
sensor_log.gps_y = zeros(num_steps, num_uavs);
sensor_log.altimeter_z = zeros(num_steps, num_uavs);
sensor_log.imu_ax = zeros(num_steps, num_uavs);
sensor_log.imu_ay = zeros(num_steps, num_uavs);
sensor_log.imu_az = zeros(num_steps, num_uavs);
sensor_log.proximity = zeros(num_steps, num_uavs); % Keep logging this
sensor_log.overlap = zeros(num_steps, num_uavs); % *** NEW SENSOR ***

% --- Plot Setup ---
figure(1); % Main simulation figure
hold on;
grid on;
axis equal;
xlabel('X Position (m)');
ylabel('Y Position (m)');
% Plot the intended (wiggly) path
path_t = linspace(0, path_duration, 100);
path_prog = path_t / path_duration;
path_x = start_pos(1) + (end_pos(1) - start_pos(1)) * path_prog;
path_y_base = start_pos(2) + (end_pos(2) - start_pos(2)) * path_prog;
path_y = path_y_base + wiggle_amplitude * sin(wiggle_frequency * 2 * pi * path_prog);
plot(path_x, path_y, 'k:', 'LineWidth', 2, 'DisplayName', 'Intended Path');
% Set axis limits
axis_buffer = circle_radius + wiggle_amplitude + 5;
x_min = min(start_pos(1), end_pos(1)) - axis_buffer;
x_max = max(start_pos(1), end_pos(1)) + axis_buffer;
y_min = min(start_pos(2), end_pos(2)) - axis_buffer;
y_max = max(start_pos(2), end_pos(2)) + axis_buffer;
axis([x_min, x_max, y_min, y_max]);
% --- Create plot handles for drones and their FOV footprints ---
colors = lines(num_uavs); 
h_drones = gobjects(num_uavs, 1);
h_fov_circles = gobjects(num_uavs, 1);
fov_circle_points = 32; 
fov_circle_template = [cos(linspace(0, 2*pi, fov_circle_points)); ...
                       sin(linspace(0, 2*pi, fov_circle_points))]' * min_separation_dist;
for i = 1:num_uavs
    h_fov_circles(i) = plot(0, 0, '-', 'Color', [colors(i,:), 0.3], 'LineWidth', 1.5);
    h_drones(i) = plot(0, 0, 'o', 'MarkerSize', 8, ...
        'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k', 'DisplayName', ['UAV ' num2str(i)]);
end
legend('Location', 'northwest');
main_title = title('Starting Simulation...');

%% --- Simulation Loop ---
for k = 1:num_steps
    t = t_vec(k);
    
    % --- 1. Swarm Brain (Part A: Path Logic) ---
    local_center_x = 0.0;
    local_center_y = 0.0;
    if t <= path_duration
        % --- PHASE 1: TRAVELING ---
        t_path = min(t, path_duration); 
        path_progress = t_path / path_duration; 
        base_x = start_pos(1) + (end_pos(1) - start_pos(1)) * path_progress;
        base_y = start_pos(2) + (end_pos(2) - start_pos(2)) * path_progress;
        
        path_angle = atan2(end_pos(2) - start_pos(2), end_pos(1) - start_pos(1));
        wiggle_angle = path_angle + pi/2; % Perpendicular
        wiggle = wiggle_amplitude * sin(wiggle_frequency * 2 * pi * path_progress);
        
        local_center_x = base_x + wiggle * cos(wiggle_angle);
        local_center_y = base_y + wiggle * sin(wiggle_angle);
    else
        % --- PHASE 2: HOLDING AT DESTINATION ---
        local_center_x = end_pos(1);
        local_center_y = end_pos(2);
    end
    
    % --- 2. Swarm Brain (Part B: Formation Logic) ---
    ff = 0.0; % Default to line
    if t >= 0 && t < t_start_transition_to_line
        ff = 1.0; 
    elseif t >= t_start_transition_to_line && t < t_end_transition_to_line
        progress = (t - t_start_transition_to_line) / (t_end_transition_to_line - t_start_transition_to_line);
        ff = 1.0 - progress;
    elseif t >= t_end_transition_to_line && t < t_start_transition_to_circle
        ff = 0.0; 
    elseif t >= t_start_transition_to_circle && t < t_end_transition_to_circle
        progress = (t - t_start_transition_to_circle) / (t_end_transition_to_circle - t_start_transition_to_circle);
        ff = progress;
    elseif t >= t_end_transition_to_circle
        ff = 1.0;
    end
    
    base_angle = angular_speed * t;
    angle_spacing = (2 * pi) / num_uavs;
    
    path_vec = end_pos - start_pos; 
    norm_path_vec = path_vec / norm(path_vec); 
    
    for i = 1:num_uavs
        % A. Calculate CIRCLE position (Polar)
        a_circle = base_angle + (i - 1) * angle_spacing;
        r_circle = circle_radius;
        
        % B. Calculate LINE position (Cartesian and Polar)
        line_pos = (i - (num_uavs + 1) / 2) * line_spacing;
        x_rel_line = line_pos * norm_path_vec(1);
        y_rel_line = line_pos * norm_path_vec(2);
        r_line = sqrt(x_rel_line^2 + y_rel_line^2);
        a_line = atan2(y_rel_line, x_rel_line);
        
        % C. Interpolate Radius
        r_final = (r_circle * ff) + (r_line * (1.0 - ff));
        
        % D. Interpolate Angle (Safely)
        diff = a_circle - a_line;
        a_diff_wrapped = mod(diff + pi, 2*pi) - pi;
        a_final = a_line + (a_diff_wrapped * ff);
        
        % E. Convert final polar coordinates back to Cartesian
        x_rel_final = r_final * cos(a_final);
        y_rel_final = r_final * sin(a_final);
        
        % F. Add the relative position to the (moving) swarm center
        target_x = local_center_x + x_rel_final;
        target_y = local_center_y + y_rel_final;
        
        % G. This is the final command (the "intended" setpoint)
        setpoints(i).position = [target_x, target_y, z_fixed];
    end
    
    % --- 3. ACTIVE COLLISION AVOIDANCE & STATE UPDATE ---
    current_positions = zeros(num_uavs, 3);
    for i = 1:num_uavs
        current_positions(i, :) = setpoints(i).position;
    end
    
    adjustment_vectors = zeros(num_uavs, 2);
    repulsion_strength = 1.1; 
    collision_detected = false;
    collision_text = "";
    
    % Array to store minimum distance for proximity sensor
    min_distances_to_neighbor = ones(num_uavs, 1) * Inf;
    
    % Check all unique pairs of drones
    for i = 1:num_uavs
        for j = (i + 1):num_uavs
            vec_ij = current_positions(j, 1:2) - current_positions(i, 1:2);
            dist = norm(vec_ij);
            
            % Update proximity sensor data for both drones
            min_distances_to_neighbor(i) = min(min_distances_to_neighbor(i), dist);
            min_distances_to_neighbor(j) = min(min_distances_to_neighbor(j), dist);
            
            if dist < (min_separation_dist - 0.01)
                collision_detected = true;
                collision_text = sprintf('AVOIDANCE! UAV %d & %d: %.2f m', i, j, dist);
                
                overlap = min_separation_dist - dist;
                push_direction_i = -vec_ij / dist; 
                adjustment_i = push_direction_i * (overlap / 2) * repulsion_strength;
                adjustment_j = -adjustment_i; 
                
                adjustment_vectors(i, :) = adjustment_vectors(i, :) + adjustment_j;
                adjustment_vectors(j, :) = adjustment_vectors(j, :) + adjustment_i;
            end
        end
    end
    
    % --- 4. SENSOR SIMULATION & STATE UPDATE ---
    for i = 1:num_uavs
        % A. Get Final "Ground Truth" Position
        final_pos_xy = current_positions(i, 1:2) + adjustment_vectors(i, :);
        ground_truth_pos = [final_pos_xy, z_fixed];
        
        % B. Calculate "Ground Truth" Dynamics
        last_pos = uav_states(i).position;
        last_vel = uav_states(i).velocity;
        
        current_vel = (ground_truth_pos - last_pos) / dt;
        current_accel = (current_vel - last_vel) / dt;
        
        % C. Simulate Sensor Readings by Adding Noise
        % GPS:
        gps_reading_xy = ground_truth_pos(1:2) + (randn(1, 2) * gps_noise_std);
        % Altimeter:
        altimeter_reading_z = ground_truth_pos(3) + (randn() * altimeter_noise_std);
        % IMU (Accelerometer):
        imu_reading_accel = current_accel + (randn(1, 3) * imu_noise_std);
        % Proximity Sensor (raw distance):
        proximity_reading = min_distances_to_neighbor(i);
        % *** NEW Overlap Sensor ***
        % (safe_radius - actual_distance)
        % Positive = DANGER, Negative = SAFE
        overlap_reading = min_separation_dist - proximity_reading;
        
        
        % D. Log all sensor data
        sensor_log.gps_x(k, i) = gps_reading_xy(1);
        sensor_log.gps_y(k, i) = gps_reading_xy(2);
        sensor_log.altimeter_z(k, i) = altimeter_reading_z;
        sensor_log.imu_ax(k, i) = imu_reading_accel(1);
        sensor_log.imu_ay(k, i) = imu_reading_accel(2);
        sensor_log.imu_az(k, i) = imu_reading_accel(3);
        sensor_log.proximity(k, i) = proximity_reading;
        sensor_log.overlap(k, i) = overlap_reading; % Log new sensor
        
        % E. Update the "state" for the next loop iteration
        uav_states(i).position = ground_truth_pos;
        uav_states(i).velocity = current_vel;
        uav_states(i).acceleration = current_accel;
    end
    
    
    % --- 5. The "Visualization" ---
    for i = 1:num_uavs
        % Update drone position
        set(h_drones(i), 'XData', uav_states(i).position(1), ...
                          'YData', uav_states(i).position(2));
        
        % Update FOV circle position
        set(h_fov_circles(i), 'XData', fov_circle_template(:,1) + uav_states(i).position(1), ...
                              'YData', fov_circle_template(:,2) + uav_states(i).position(2));
    end
    
    % Update title with status and sensor data for UAV 1
    uav1_gps_x = sensor_log.gps_x(k, 1);
    uav1_alt_z = sensor_log.altimeter_z(k, 1);
    uav1_imu_ax = sensor_log.imu_ax(k, 1);
    uav1_overlap = sensor_log.overlap(k, 1); % *** Use new sensor
    
    title_line1 = sprintf('UAV Swarm (t=%.1fs)', t);
    % *** Updated title line ***
    title_line2 = sprintf('UAV 1 Sensors | GPS-X: %.1f | Alt: %.1f | IMU-aX: %.2f | Overlap: %.1f', ...
                          uav1_gps_x, uav1_alt_z, uav1_imu_ax, uav1_overlap);
    
    if collision_detected
        % Change color to red if overlap is positive (dangerous)
        if uav1_overlap > 0
            title_color = 'r';
        else
            title_color = 'b';
        end
        set(main_title, 'String', {title_line1, title_line2, collision_text}, 'Color', title_color);
    else
        set(main_title, 'String', {title_line1, title_line2}, 'Color', 'k');
    end
    
    drawnow;
    pause(dt); % Pause to make simulation viewable
    
    if collision_detected
        fprintf('Time %.2f: %s\n', t, collision_text);
    end
end

%% --- 6. Final Report & Sensor Plots ---
fprintf('Mission Successful. Sensor data logged.\n');
set(main_title, 'String', 'Mission Successful (Hold at End)', 'Color', 'g');

% --- *** NEW: Plot sensor data for ALL UAVs *** ---
% This loop creates a new figure for each UAV
for uav_to_plot = 1:num_uavs
    
    figure(1 + uav_to_plot); % Create Figure 2, Figure 3, etc.
    clf; % Clear this figure
    sgtitle(['Simulated Sensor Readings for UAV ' num2str(uav_to_plot)], 'FontSize', 14, 'FontWeight', 'bold');

    % Plot 1: GPS Position (X vs Y)
    subplot(2, 2, 1);
    plot(sensor_log.gps_x(:, uav_to_plot), sensor_log.gps_y(:, uav_to_plot), '.', 'Color', colors(uav_to_plot,:));
    hold on;
    plot(path_x, path_y, 'k:', 'LineWidth', 1.5); % Plot intended path
    title('GPS Position (Noisy)');
    xlabel('X (m)');
    ylabel('Y (m)');
    axis equal;
    grid on;

    % Plot 2: Altimeter
    subplot(2, 2, 2);
    plot(sensor_log.time, sensor_log.altimeter_z(:, uav_to_plot), 'b-');
    title('Altimeter (Noisy)');
    xlabel('Time (s)');
    ylabel('Altitude (m)');
    ylim([z_fixed - 0.5, z_fixed + 0.5]); % Zoom in around the altitude
    grid on;

    % Plot 3: IMU Accelerometer (X and Y)
    subplot(2, 2, 3);
    plot(sensor_log.time, sensor_log.imu_ax(:, uav_to_plot), 'r-', 'DisplayName', 'aX');
    hold on;
    plot(sensor_log.time, sensor_log.imu_ay(:, uav_to_plot), 'g-', 'DisplayName', 'aY');
    title('IMU - Accelerometer (Noisy)');
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    legend;
    grid on;

    % Plot 4: *** NEW Overlap Sensor Plot ***
    subplot(2, 2, 4);
    % --- *** COLOR CHANGED TO MAGENTA ('m-') FOR VISIBILITY *** ---
    plot(sensor_log.time, sensor_log.overlap(:, uav_to_plot), 'm-', 'LineWidth', 1.5);
    hold on;
    % Plot the "danger zone" line at 0
    plot([0, sim_time], [0, 0], 'r--', 'LineWidth', 2, 'DisplayName', 'Danger Limit');
    title('FOV Overlap Sensor (Nearest Neighbor)');
    xlabel('Time (s)');
    ylabel('Overlap (m)');
    legend({'Overlap', 'Danger Limit'},'Location', 'best');
    grid on;
    
end % End of plotting loop