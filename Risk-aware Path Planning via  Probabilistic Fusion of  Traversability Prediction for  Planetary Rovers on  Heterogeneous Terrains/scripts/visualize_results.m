% Visualize Rover Navigation Results
% This script loads results.mat and visualizes performance metrics.

clear; clc; close all;

% Load results
if exist('../results.mat', 'file')
    load('../results.mat');
else
    error('results.mat not found. Please run the python pipeline first.');
end

datasets = {'Std', 'AA', 'ES'};
metrics_to_plot = {'is_solved', 'is_feasible', 'dist_mean', 'est_cost_mean', 'age_mean', 'obs_time_mean', 'max_slip_mean'};
metric_labels = {'Solved Rate (%)', 'Feasible Rate (%)', 'Distance (m)', 'Est. Cost', 'Age', 'Obs. Time (s)', 'Max Slip'};

% Check if variables exist
for i = 1:length(datasets)
    ds = datasets{i};
    if ~exist(ds, 'var')
        warning(['Dataset ' ds ' not found in results.mat']);
        continue;
    end
end

% Display Results in Command Window
fprintf('----------------------------------------------------------------\n');
fprintf('%-10s | %-10s | %-10s | %-10s | %-10s\n', 'Dataset', 'Model', 'Embed', 'Solved', 'Feasible');
fprintf('----------------------------------------------------------------\n');

for i = 1:length(datasets)
    ds_name = datasets{i};
    if exist(ds_name, 'var')
        data = eval(ds_name);
        % Check if loaded as cell (Python lists often load as cells in Matlab)
        if iscell(data)
            try
                % Convert cell array of scalar structs to struct array
                data = [data{:}]; 
            catch
                % Fallback: keep as key access if needed, but [data{:}] usually works for uniform structs
            end
        end
        for j = 1:length(data)
            entry = data(j);
            % Handle potential cell vs char issues from Python -> Matlab conversion
            type_model = string(entry.type_model);
            type_embed = string(entry.type_embed);
            
            fprintf('%-10s | %-10s | %-10s | %-10.2f | %-10.2f\n', ...
                ds_name, type_model, type_embed, entry.is_solved, entry.is_feasible);
        end
    end
end
fprintf('----------------------------------------------------------------\n');

% Plotting
% We will plot comparison for the first model/embed combo found or specific ones
% For simplicity, let's plot "gsm mean" (if available) across datasets
% Or plot all results for each metric.

% Simplified plotting: Group by Dataset
figure('Name', 'Rover Navigation Results', 'Color', 'w');
n_metrics = length(metrics_to_plot);

for m = 1:n_metrics
    metric = metrics_to_plot{m};
    
    subplot(2, 4, m);
    hold on;
    title(metric_labels{m});
    
    x_labels = {};
    y_values = [];
    
    idx = 1;
    for i = 1:length(datasets)
        ds_name = datasets{i};
        if exist(ds_name, 'var')
            data = eval(ds_name);
            if iscell(data)
                data = [data{:}];
            end
            for j = 1:length(data)
                % Create a label for the bar
                entry = data(j);
                lbl = sprintf('%s\n%s-%s', ds_name, string(entry.type_model), string(entry.type_embed));
                x_labels{idx} = lbl;
                
                % Get value
                if isfield(entry, metric)
                    val = entry.(metric);
                    y_values(idx) = double(val);
                else
                    y_values(idx) = 0;
                end
                idx = idx + 1;
            end
        end
    end
    
    b = bar(y_values);
    set(gca, 'XTick', 1:length(y_values), 'XTickLabel', x_labels);
    xtickangle(45);
    grid on;
end

sgtitle('Performance Comparison across Datasets');
