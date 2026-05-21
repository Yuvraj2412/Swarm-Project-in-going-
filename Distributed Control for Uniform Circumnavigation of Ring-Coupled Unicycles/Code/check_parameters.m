%% Parameter Checker and Stability Analyzer
% This script helps you verify if your chosen parameters will result
% in a stable circumnavigation formation.

function check_parameters(N, d, radii, kv, kw)
%CHECK_PARAMETERS Verify parameter validity and stability
%
%   check_parameters(N, d, radii, kv, kw)
%
%   Example:
%       check_parameters(6, 1, [2,2,2,3,3,3], 2.75, 3.897)

if nargin < 5
    error('Usage: check_parameters(N, d, radii, kv, kw)');
end

fprintf('\n========================================\n');
fprintf('   PARAMETER VALIDATION & STABILITY    \n');
fprintf('========================================\n\n');

%% 1. Basic Validation
fprintf('1. BASIC VALIDATION:\n');
fprintf('   ----------------\n');

valid = true;

% Check N
if N < 3
    fprintf('   ❌ N = %d is too small (must be ≥ 3)\n', N);
    valid = false;
else
    fprintf('   ✓ N = %d agents\n', N);
end

% Check radii
if length(radii) ~= N
    fprintf('   ❌ radii length (%d) does not match N (%d)\n', length(radii), N);
    valid = false;
elseif any(radii <= 0)
    fprintf('   ❌ All radii must be positive\n');
    valid = false;
else
    fprintf('   ✓ radii = [%s]\n', num2str(radii));
    fprintf('     (min: %.2f, max: %.2f)\n', min(radii), max(radii));
end

% Check d
if d < 1 || d > N-1
    fprintf('   ❌ d = %d is invalid (must be 1 to %d)\n', d, N-1);
    valid = false;
else
    fprintf('   ✓ d = %d\n', d);
    psi_bar = 2*d*pi/N;
    fprintf('     → Angular spacing: %.2f° (%.4f rad)\n', psi_bar*180/pi, psi_bar);
end

% Check control gains
if kv <= 0 || kw <= 0
    fprintf('   ❌ Control gains must be positive\n');
    valid = false;
else
    fprintf('   ✓ kv = %.3f, kw = %.3f\n', kv, kw);
end

if ~valid
    fprintf('\n❌ VALIDATION FAILED - Fix errors above!\n\n');
    return;
end

fprintf('\n✓ All basic parameters are valid!\n\n');

%% 2. Stability Analysis
fprintf('2. STABILITY ANALYSIS:\n');
fprintf('   -------------------\n');

psi_bar = 2*d*pi/N;
epsilon = 0.1;

% Determine c and stability
if d == 1 || d == N-1
    c = 1 - epsilon;
    stability = 'ASYMPTOTICALLY STABLE ✓';
    case_type = 'Case I (Theorem 5)';
elseif mod(N, 2) == 1 && (d == floor(N/2) || d == ceil(N/2))
    c = -1;
    stability = 'ASYMPTOTICALLY STABLE ✓';
    case_type = 'Case II (Theorem 5)';
elseif mod(N, 2) == 0 && (d == N/2-1 || d == N/2+1)
    c = -1 + epsilon;
    stability = 'ASYMPTOTICALLY STABLE ✓';
    case_type = 'Case III (Theorem 5)';
else
    c = 1 - epsilon;
    stability = 'MAY NOT BE STABLE ⚠';
    case_type = 'Non-standard case';
end

fprintf('   Formation type: %s\n', case_type);
fprintf('   Control parameter c = %.4f\n', c);
fprintf('   Status: %s\n', stability);

if contains(stability, 'MAY NOT')
    fprintf('\n   ⚠ WARNING: This (N,d) combination is not proven stable!\n');
    fprintf('   Recommended stable values for N=%d:\n', N);
    fprintf('     • d = 1 (clockwise, %.2f° spacing)\n', 2*pi/N*180/pi);
    fprintf('     • d = %d (counterclockwise, %.2f° spacing)\n', N-1, 2*(N-1)*pi/N*180/pi);
end

%% 3. Control Gain Ratio Check
fprintf('\n3. CONTROL GAIN RATIO:\n');
fprintf('   -------------------\n');

kv_kw_theoretical = (1 - c*cos(psi_bar)) / abs(c*sin(psi_bar));
kv_kw_actual = kv / kw;

fprintf('   Required ratio (kv/kw): %.4f\n', kv_kw_theoretical);
fprintf('   Actual ratio (kv/kw):   %.4f\n', kv_kw_actual);

ratio_error = abs(kv_kw_theoretical - kv_kw_actual) / kv_kw_theoretical * 100;

if ratio_error < 1
    fprintf('   ✓ Ratio matches (error: %.2f%%)\n', ratio_error);
elseif ratio_error < 5
    fprintf('   ⚠ Small mismatch (error: %.2f%%)\n', ratio_error);
    fprintf('   Suggested kw adjustment: %.4f\n', kv / kv_kw_theoretical);
else
    fprintf('   ❌ Large mismatch (error: %.2f%%)!\n', ratio_error);
    fprintf('   Recommended kw: %.4f\n', kv / kv_kw_theoretical);
end

%% 4. Agent-Specific Parameters
fprintf('\n4. AGENT PARAMETERS:\n');
fprintf('   ------------------\n');

delta = 1 ./ radii;
a = zeros(1, N);

fprintf('   Agent    ri    δi      ai\n');
fprintf('   -----   ----  ------  ------\n');

for i = 1:N
    i_next = mod(i, N) + 1;
    a(i) = 1 - c * (radii(i) / radii(i_next));
    fprintf('     %d     %.2f  %.4f  %6.3f', i, radii(i), delta(i), a(i));
    
    % Interpretation
    if a(i) < 0
        fprintf('  (repel target, attract neighbor)\n');
    elseif a(i) == 0
        fprintf('  (ignore target)\n');
    elseif a(i) < 1
        fprintf('  (attract both)\n');
    elseif a(i) == 1
        fprintf('  (only attract target)\n');
    else
        fprintf('  (attract target, repel neighbor)\n');
    end
end

%% 5. Boundedness Check (Theorem 2)
fprintf('\n5. TRAJECTORY BOUNDEDNESS:\n');
fprintf('   -----------------------\n');

% Calculate required c range for boundedness
c_min = sec(2*floor(N/2)*pi/N)^2;
c_max = 1;

fprintf('   For bounded trajectories, c must be in:\n');
fprintf('   [%.4f, %.4f]\n', c_min, c_max);
fprintf('   Current c = %.4f\n', c);

if c >= c_min && c <= c_max
    fprintf('   ✓ Trajectories will be bounded\n');
else
    fprintf('   ❌ WARNING: Trajectories may be unbounded!\n');
end

%% Summary
fprintf('\n========================================\n');
fprintf('   SUMMARY\n');
fprintf('========================================\n\n');

if contains(stability, 'STABLE ✓') && ratio_error < 5
    fprintf('✓ Configuration is GOOD TO GO!\n');
    fprintf('  All parameters are valid and the formation\n');
    fprintf('  should converge to the desired pattern.\n\n');
elseif contains(stability, 'STABLE ✓')
    fprintf('⚠ Configuration is MOSTLY GOOD\n');
    fprintf('  Formation should be stable but consider\n');
    fprintf('  adjusting kw for better convergence.\n\n');
else
    fprintf('⚠ Configuration MAY HAVE ISSUES\n');
    fprintf('  Formation stability is not guaranteed.\n');
    fprintf('  Consider using d=1 or d=%d instead.\n\n', N-1);
end

end


%% Quick Test Function
function quick_test()
%QUICK_TEST Run some predefined test cases

fprintf('Running test cases...\n\n');

% Test 1: Paper example
fprintf('TEST 1: Paper Example (Table 1)\n');
check_parameters(6, 2, [2,2,2,3,3,3], 2.75, 3.897);
fprintf('\nPress any key to continue...\n');
pause;

% Test 2: All same radius
fprintf('\nTEST 2: All agents same radius\n');
check_parameters(8, 1, 3*ones(1,8), 2.5, 3.5);
fprintf('\nPress any key to continue...\n');
pause;

% Test 3: Unstable configuration
fprintf('\nTEST 3: Potentially unstable configuration\n');
check_parameters(6, 3, [2,2,2,3,3,3], 2.75, 3.0);

end
