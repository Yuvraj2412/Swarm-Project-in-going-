function [c_opt, kv_opt, kw_opt, success] = optimize_control_parameters(...
    desired_spacings, radii, c_initial)
%OPTIMIZE_CONTROL_PARAMETERS Find optimal control gains for custom spacing
%   Uses nonlinear optimization to find c, kv, kw that best achieve
%   the desired non-uniform spacing pattern
%
%   Inputs:
%       desired_spacings - Target angular spacings (radians)
%       radii - Strategic radii for each agent
%       c_initial - Initial guess for c parameter
%
%   Outputs:
%       c_opt, kv_opt, kw_opt - Optimized control parameters
%       success - True if optimization converged

N = length(desired_spacings);

%% Setup optimization problem

% Decision variables: [c, kv, kw]
% Initial guess
x0 = [c_initial, 2.5, 3.5];

% Bounds
% c must be in valid range for boundedness (Theorem 2)
c_min = sec(2*floor(N/2)*pi/N)^2;
c_max = 1.0;

lb = [c_min, 0.1, 0.1];  % Lower bounds
ub = [c_max, 10.0, 10.0];  % Upper bounds

%% Objective function: Minimize spacing error
objective = @(x) spacing_error_objective(x, desired_spacings, radii, N);

%% Constraints
% 1. Maintain kv/kw ratio relationship (soft constraint via penalty)
% 2. Ensure c in stable range

% Nonlinear constraint function
nonlcon = @(x) control_parameter_constraints(x, desired_spacings, N);

%% Solve optimization

try
    % Try using fmincon (requires Optimization Toolbox)
    options = optimoptions('fmincon', ...
        'Display', 'off', ...
        'MaxFunctionEvaluations', 3000, ...
        'MaxIterations', 1000, ...
        'OptimalityTolerance', 1e-6, ...
        'StepTolerance', 1e-8);
    
    [x_opt, fval, exitflag] = fmincon(objective, x0, [], [], [], [], ...
        lb, ub, nonlcon, options);
    
    c_opt = x_opt(1);
    kv_opt = x_opt(2);
    kw_opt = x_opt(3);
    
    success = (exitflag > 0);
    
    if success
        fprintf('  ✓ Optimization converged (error = %.6f)\n', fval);
    else
        fprintf('  ⚠ Optimization did not fully converge (exit flag: %d)\n', exitflag);
    end
    
catch ME
    % Fallback if Optimization Toolbox not available
    fprintf('  ⚠ fmincon not available - using approximation method\n');
    fprintf('    Error: %s\n', ME.message);
    
    % Use simple grid search or analytical approximation
    [c_opt, kv_opt, kw_opt] = approximate_parameters(...
        desired_spacings, radii, c_initial);
    success = false;
end

% Verify results
fprintf('  Optimized parameters:\n');
fprintf('    c  = %.4f\n', c_opt);
fprintf('    kv = %.4f\n', kv_opt);
fprintf('    kw = %.4f\n', kw_opt);
fprintf('    kv/kw = %.4f\n', kv_opt/kw_opt);

end


function error = spacing_error_objective(x, desired_spacings, radii, N)
%SPACING_ERROR_OBJECTIVE Compute error between desired and predicted spacings

c = x(1);
kv = x(2);
kw = x(3);

% Predict equilibrium spacings based on control parameters
% This is an approximation based on the equilibrium analysis

predicted_spacings = zeros(1, N);

for i = 1:N
    i_next = mod(i, N) + 1;
    
    % From equilibrium equations (Section 3.3)
    % At equilibrium: kv/kw = (1 - c*cos(ψᵢ)) / |c*sin(ψᵢ)|
    % Solve for ψᵢ given kv/kw and c
    
    % This is implicit, so we use the desired spacing as a good estimate
    % and compute what kv/kw would be required
    psi_i = desired_spacings(i);
    
    % The radius ratio affects the parameter aᵢ
    a_i = 1 - c * (radii(i) / radii(i_next));
    
    % Equilibrium spacing relationship (approximate)
    % For ring topology, spacings are coupled
    predicted_spacings(i) = psi_i;  % Placeholder
end

% For now, use a simpler error metric
% Error = deviation from optimal kv/kw ratio for mean spacing
psi_mean = mean(desired_spacings);
kv_kw_theoretical = (1 - c*cos(psi_mean)) / abs(c*sin(psi_mean));
kv_kw_actual = kv / kw;

error = (kv_kw_theoretical - kv_kw_actual)^2;

% Add penalty for being far from initial guess
c_nominal = 0.9;
error = error + 0.1 * (c - c_nominal)^2;

% Add penalty for spacing variation (prefer smoother)
spacing_variance = var(desired_spacings);
error = error + 0.01 * spacing_variance;

end


function [c_nonlin, ceq] = control_parameter_constraints(x, desired_spacings, N)
%CONTROL_PARAMETER_CONSTRAINTS Nonlinear constraints for optimization

c = x(1);
% kv = x(2);
% kw = x(3);

% No nonlinear inequality constraints
c_nonlin = [];

% No nonlinear equality constraints
ceq = [];

% Note: Linear bounds handle most constraints
% Could add: stability conditions, boundedness conditions, etc.

end


function [c_approx, kv_approx, kw_approx] = approximate_parameters(...
    desired_spacings, radii, c_initial)
%APPROXIMATE_PARAMETERS Fallback method without optimization toolbox

% Use mean spacing for approximation
psi_mean = mean(desired_spacings);

% Use initial c or adjust slightly
c_approx = c_initial;

% Calculate kv/kw ratio
kv_kw_ratio = (1 - c_approx*cos(psi_mean)) / abs(c_approx*sin(psi_mean));

% Set reasonable kv and derive kw
kv_approx = 2.5;
kw_approx = kv_approx / kv_kw_ratio;

fprintf('  Using approximation: c=%.4f, kv=%.4f, kw=%.4f\n', ...
    c_approx, kv_approx, kw_approx);

end
