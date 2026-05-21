% Visualize Dynamic Fixed Terrain Results
% Shows: Sol. (Solved), Succ. (Success Rate), Ttotal (Total Time), smax (Max Slip)

clear; clc; close all;

% Load results
results_path = '../results_dynamic_fixed.mat';
if exist(results_path, 'file')
    load(results_path);
else
    error('results_dynamic_fixed.mat not found. Run run_dynamic_fixed_pipeline.py first.');
end

dataset_name = 'DynamicFixed';

if ~exist(dataset_name, 'var')
    error(['Dataset ' dataset_name ' not found in results file']);
end

% Extract data
data = eval(dataset_name);
if iscell(data)
    data = [data{:}];
end

% Display Table: Sol. | Succ. | Ttotal | smax
fprintf('========================================================================\n');
fprintf('%-15s | %-10s | %-8s | %-8s | %-10s | %-10s\n', ...
    'Dataset', 'Model', 'Sol.', 'Succ.', 'Ttotal(s)', 'smax');
fprintf('========================================================================\n');

for i = 1:length(data)
    entry = data(i);
    type_model = string(entry.type_model);
    type_embed = string(entry.type_embed);
    
    % Calculate metrics
    sol = entry.is_solved;  % Solved count/rate
    succ = entry.is_feasible;  % Success/feasible rate
    ttotal = entry.obs_time_mean;  % Total time
    smax = entry.max_slip_mean;  % Max slip
    
    fprintf('%-15s | %-10s | %-8.2f | %-8.2f | %-10.2f | %-10.2f\n', ...
        dataset_name, type_model + "-" + type_embed, sol, succ, ttotal, smax);
end
fprintf('========================================================================\n');

% Plotting
figure('Name', 'Dynamic Fixed Results', 'Color', 'w', 'Position', [100, 100, 1400, 500]);
sgtitle('Rover Performance: Sol. / Succ. / Ttotal / smax', 'FontSize', 14, 'FontWeight', 'bold');

metrics = {'is_solved', 'is_feasible', 'obs_time_mean', 'max_slip_mean'};
metric_labels = {'Sol. (Solved)', 'Succ. (Feasible)', 'Ttotal (s)', 'smax'};

for m = 1:length(metrics)
    metric = metrics{m};
    
    subplot(1, 4, m);
    hold on;
    title(metric_labels{m}, 'FontSize', 12);
    
    x_labels = {};
    y_values = [];
    
    for i = 1:length(data)
        entry = data(i);
        lbl = string(entry.type_model) + "-" + string(entry.type_embed);
        x_labels{i} = lbl;
        
        if isfield(entry, metric)
            y_values(i) = double(entry.(metric));
        else
            y_values(i) = 0;
        end
    end
    
    b = bar(y_values);
    b.FaceColor = [0.2, 0.6, 0.8];
    
    set(gca, 'XTickLabel', x_labels);
    xtickangle(15);
    grid on;
    
    % Add value labels
    for i = 1:length(y_values)
        text(i, y_values(i), sprintf('%.2f', y_values(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 9);
    end
    
    ylabel('Value');
end

fprintf('\nVisualization complete.\n');
