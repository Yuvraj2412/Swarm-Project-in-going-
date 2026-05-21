function [radii, variation_percentage] = calculate_radii_from_spacings(...
    desired_spacings, r_nominal, N)
%CALCULATE_RADII_FROM_SPACINGS Generate strategic radii for custom spacing
%   Solution 1: Strategic Radii Assignment
%   
%   Creates small variations in radii to encode spacing information,
%   enabling non-uniform formations while maintaining visual similarity.
%
%   Inputs:
%       desired_spacings - Vector of angular spacings [ψ₁, ψ₂, ..., ψₙ] (radians)
%       r_nominal - Nominal/base radius for all agents (meters)
%       N - Number of agents
%
%   Outputs:
%       radii - Vector of strategic radii [r₁, r₂, ..., rₙ] (meters)
%       variation_percentage - Maximum variation as fraction of nominal

if nargin < 3
    N = length(desired_spacings);
end

% Validate inputs
if length(desired_spacings) ~= N
    error('Length of desired_spacings must equal N');
end

if abs(sum(desired_spacings) - 2*pi) > 1e-4
    warning('Spacings do not sum to 2π - may cause issues');
end

%% Strategic Radii Assignment Algorithm

% Method: Map spacings to radii using a smooth function
% Goal: Larger spacing → slightly larger radius
%       This creates the coupling needed for non-uniform equilibrium

% Variation magnitude (percentage of nominal radius)
% Recommended range: 1% - 5%
% - Too small: insufficient coupling, converges to uniform
% - Too large: visually obvious, may affect stability
variation_magnitude = 0.03;  % 3% variation (good balance)

% Normalize spacings to [0, 1] range
spacing_min = min(desired_spacings);
spacing_max = max(desired_spacings);
spacing_range = spacing_max - spacing_min;

if spacing_range < 1e-6
    % All spacings equal - uniform case
    radii = r_nominal * ones(1, N);
    variation_percentage = 0;
    return;
end

spacings_normalized = (desired_spacings - spacing_min) / spacing_range;

% Mapping function options:

% Option 1: Linear mapping (simple, predictable)
% radii = r_nominal * (1 + variation_magnitude * (spacings_normalized - 0.5) * 2);

% Option 2: Quadratic mapping (smoother transitions)
% radii = r_nominal * (1 + variation_magnitude * ((spacings_normalized - 0.5) * 2).^2 .* sign(spacings_normalized - 0.5));

% Option 3: Sigmoidal mapping (bounded, smooth) ⭐ RECOMMENDED
% Maps normalized spacings through sigmoid for smooth variation
k_sigmoid = 4;  % Steepness parameter
sigmoid = @(x) 1./(1 + exp(-k_sigmoid*(x - 0.5)));
variation_factor = sigmoid(spacings_normalized);
variation_factor = (variation_factor - min(variation_factor)) / ...
                   (max(variation_factor) - min(variation_factor));  % Renormalize to [0,1]
radii = r_nominal * (1 + variation_magnitude * (variation_factor - 0.5) * 2);

% Option 4: Direct proportional (maintains spacing ratios)
% radii = r_nominal * (1 + variation_magnitude * (desired_spacings / mean(desired_spacings) - 1));

%% Alternative: Cumulative angle mapping
% This approach uses the cumulative angular position
% cumulative_angles = [0, cumsum(desired_spacings(1:end-1))];
% cumulative_normalized = cumulative_angles / (2*pi);
% radii = r_nominal * (1 + variation_magnitude * sin(2*pi*cumulative_normalized));

%% Calculate actual variation
variation_percentage = max(abs(radii - r_nominal)) / r_nominal;

%% Quality checks
fprintf('Radii generation summary:\n');
fprintf('  Base radius: %.4f m\n', r_nominal);
fprintf('  Variation magnitude: %.2f%%\n', variation_magnitude * 100);
fprintf('  Actual variation: %.2f%% (±%.4f m)\n', ...
    variation_percentage * 100, variation_percentage * r_nominal);
fprintf('  Min radius: %.4f m\n', min(radii));
fprintf('  Max radius: %.4f m\n', max(radii));

% Check if variation is reasonable
if variation_percentage > 0.10
    warning('Radii variation > 10%% - may be visually noticeable');
end

if variation_percentage < 0.001
    warning('Radii variation < 0.1%% - may be insufficient for non-uniform spacing');
end

end


function radii = alternative_mapping_function(desired_spacings, r_nominal, method)
%ALTERNATIVE_MAPPING_FUNCTION Try different mapping strategies
%   This function is not called by default but provides alternatives

N = length(desired_spacings);
variation_magnitude = 0.03;

switch method
    case 'linear'
        % Simple linear relationship
        spacing_avg = mean(desired_spacings);
        radii = r_nominal * (1 + variation_magnitude * ...
                (desired_spacings / spacing_avg - 1));
        
    case 'exponential'
        % Exponential mapping for stronger differentiation
        spacing_normalized = desired_spacings / max(desired_spacings);
        radii = r_nominal * (1 + variation_magnitude * ...
                (exp(spacing_normalized) - exp(0.5)));
        
    case 'piecewise'
        % Piecewise: group similar spacings
        spacing_median = median(desired_spacings);
        radii = r_nominal * ones(1, N);
        radii(desired_spacings < spacing_median) = ...
            r_nominal * (1 - variation_magnitude);
        radii(desired_spacings > spacing_median) = ...
            r_nominal * (1 + variation_magnitude);
        
    case 'harmonic'
        % Use harmonic function
        cumulative = [0, cumsum(desired_spacings(1:end-1))];
        radii = r_nominal * (1 + variation_magnitude * ...
                sin(2*pi*cumulative/(2*pi)));
        
    otherwise
        error('Unknown method: %s', method);
end

end
