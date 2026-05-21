% Visualize 3D Map in MATLAB
% Loads sample_3d_map.mat and renders the terrain with color overlay.

clear; clc; close all;

% Load data
filename = '../sample_3d_map.mat';
if exist(filename, 'file')
    load(filename);
else
    error('sample_3d_map.mat not found. Run python scripts/convert_sample_to_mat.py first.');
end

fprintf('Loaded map generated from: %s\n', generated_file);

% Create mesh grid
[N, M] = size(height);
[X, Y] = meshgrid(1:M, 1:N);

% Create Figure
figure('Name', '3D Map Visualization', 'Color', 'w', 'Position', [100, 100, 1000, 800]);

% Plot Surface
% CData must be correctly scaled. If color_map is [0,1], it works.
s = surf(X, Y, height, color_map);

% Beautify
s.EdgeColor = 'none'; % Remove mesh lines for smoother look
s.FaceColor = 'flat'; % Use CData

% Lighting
light('Position', [-1, -1, 1], 'Style', 'infinite');
lighting gouraud;
material dull;

% Axis settings
axis equal;
axis vis3d;
grid on;
title(['3D Terrain Visualization - ' generated_file], 'FontSize', 14);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Height (m)');

% View angle
view(45, 45);

% Add rotation slider
c = uicontrol('Style', 'slider', 'Min', 0, 'Max', 360, 'Value', 45, ...
    'Position', [400, 20, 200, 20], ...
    'Callback', @(src, event) view(src.Value, 45));
uicontrol('Style', 'text', 'Position', [400, 45, 200, 20], 'String', 'Rotate View');

fprintf('Visualization ready. Use the slider to rotate.\n');
