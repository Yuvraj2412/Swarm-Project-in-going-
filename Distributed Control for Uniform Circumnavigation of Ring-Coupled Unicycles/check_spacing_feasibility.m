function [spacings_adjusted, N_adjusted, is_feasible, adjustment_made] = ...
    check_spacing_feasibility(desired_spacings, N)
%CHECK_SPACING_FEASIBILITY Validate and adjust custom spacings
%   Ensures that sum of spacings equals 2π (360 degrees)
%   Offers intelligent adjustment options if not feasible
%
%   Inputs:
%       desired_spacings - Vector of desired angular spacings (radians)
%       N - Number of agents
%
%   Outputs:
%       spacings_adjusted - Adjusted spacings (sum = 2π)
%       N_adjusted - Possibly adjusted number of agents
%       is_feasible - True if original input was feasible
%       adjustment_made - True if adjustments were made

adjustment_made = false;
spacings_adjusted = desired_spacings;
N_adjusted = N;

% Check if length matches N
if length(desired_spacings) ~= N
    fprintf('\n⚠ WARNING: Spacing vector length (%d) ≠ N (%d)\n', ...
        length(desired_spacings), N);
    N_adjusted = length(desired_spacings);
    fprintf('   Adjusting N to %d\n', N_adjusted);
    adjustment_made = true;
end

% Check if all spacings are positive
if any(desired_spacings <= 0)
    error('All spacings must be positive!');
end

% Check sum
spacing_sum = sum(desired_spacings);
tolerance = 0.01;  % 1% tolerance (about 3.6 degrees)

if abs(spacing_sum - 2*pi) < tolerance
    % Feasible!
    is_feasible = true;
    fprintf('✓ Spacing is feasible (sum = %.2f°, error = %.2f°)\n', ...
        spacing_sum*180/pi, (spacing_sum - 2*pi)*180/pi);
    return;
end

% Not feasible - need adjustment
is_feasible = false;
fprintf('\n⚠ SPACING NOT FEASIBLE\n');
fprintf('  Current sum: %.2f° (required: 360°)\n', spacing_sum*180/pi);
fprintf('  Difference: %.2f°\n\n', (spacing_sum - 2*pi)*180/pi);

% Offer adjustment options
fprintf('ADJUSTMENT OPTIONS:\n');
fprintf('  [1] Normalize all spacings proportionally\n');
fprintf('  [2] Add missing angle to largest spacing\n');
fprintf('  [3] Distribute missing angle equally\n');
fprintf('  [4] Suggest adding/removing agents\n');
fprintf('  [5] Use as-is (may not converge properly)\n\n');

% Auto-select best option based on magnitude of error
spacing_error = abs(spacing_sum - 2*pi);
spacing_error_deg = spacing_error * 180/pi;

if spacing_error_deg < 20
    % Small error - normalize
    choice = 1;
    fprintf('Auto-selecting option 1 (normalize) - error is small\n');
elseif spacing_sum < 2*pi
    % Missing angle
    choice = 3;
    fprintf('Auto-selecting option 3 (distribute) - missing angle\n');
else
    % Excess angle
    choice = 1;
    fprintf('Auto-selecting option 1 (normalize) - excess angle\n');
end

% Apply adjustment
switch choice
    case 1
        % Normalize proportionally
        spacings_adjusted = desired_spacings * (2*pi / spacing_sum);
        adjustment_made = true;
        fprintf('\n✓ Normalized spacings:\n');
        fprintf('  Original: [%s] deg\n', num2str(desired_spacings*180/pi, '%.1f '));
        fprintf('  Adjusted: [%s] deg\n', num2str(spacings_adjusted*180/pi, '%.1f '));
        
    case 2
        % Add to largest spacing
        [~, max_idx] = max(desired_spacings);
        spacings_adjusted = desired_spacings;
        spacings_adjusted(max_idx) = spacings_adjusted(max_idx) + (2*pi - spacing_sum);
        adjustment_made = true;
        fprintf('\n✓ Added %.2f° to spacing %d\n', ...
            (2*pi - spacing_sum)*180/pi, max_idx);
        
    case 3
        % Distribute equally
        adjustment_per_spacing = (2*pi - spacing_sum) / length(desired_spacings);
        spacings_adjusted = desired_spacings + adjustment_per_spacing;
        adjustment_made = true;
        fprintf('\n✓ Added %.2f° to each spacing\n', ...
            adjustment_per_spacing*180/pi);
        
    case 4
        % Suggest N adjustment
        [spacings_adjusted, N_adjusted] = suggest_agent_count(desired_spacings);
        adjustment_made = true;
        
    case 5
        % Use as-is
        spacings_adjusted = desired_spacings;
        fprintf('\n⚠ Using original spacings - may not converge!\n');
end

% Verify final sum
final_sum = sum(spacings_adjusted);
fprintf('\nFinal verification: sum = %.4f rad (%.2f°)\n', ...
    final_sum, final_sum*180/pi);

if abs(final_sum - 2*pi) > 1e-6
    warning('Adjustment did not achieve exact 360° - normalizing once more');
    spacings_adjusted = spacings_adjusted * (2*pi / final_sum);
end

end


function [spacings_new, N_new] = suggest_agent_count(desired_spacings)
%SUGGEST_AGENT_COUNT Suggest optimal number of agents for given spacings

current_sum = sum(desired_spacings);
current_sum_deg = current_sum * 180/pi;
missing_deg = 360 - current_sum_deg;

fprintf('\nSUGGESTED AGENT COUNT ADJUSTMENTS:\n');
fprintf('  Current sum: %.1f°, Missing: %.1f°\n\n', ...
    current_sum_deg, missing_deg);

% Option A: Add agents with average spacing
avg_spacing = mean(desired_spacings);
avg_spacing_deg = avg_spacing * 180/pi;
n_add = round(missing_deg / avg_spacing_deg);

if n_add > 0 && missing_deg > 0
    fprintf('  Option A: Add %d agent(s) with ~%.1f° spacing each\n', ...
        n_add, missing_deg/n_add);
end

% Option B: Remove an agent if excess
if missing_deg < 0
    n_remove = 1;
    fprintf('  Option B: Remove 1 agent and redistribute\n');
end

% Option C: Find closest feasible uniform spacing
N_current = length(desired_spacings);
for N_try = [N_current-1, N_current, N_current+1, N_current+2]
    if N_try < 3
        continue;
    end
    uniform_spacing = 360 / N_try;
    fprintf('  Option: N=%d agents → uniform %.1f° spacing\n', ...
        N_try, uniform_spacing);
end

% Auto-select: normalize current spacings
fprintf('\n→ Auto-selecting: Normalize current spacings\n');
spacings_new = desired_spacings * (2*pi / current_sum);
N_new = length(desired_spacings);

end
