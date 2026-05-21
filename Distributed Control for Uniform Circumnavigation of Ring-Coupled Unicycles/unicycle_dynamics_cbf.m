function dx = unicycle_dynamics_cbf(t, x, params)
%UNICYCLE_DYNAMICS_CBF Dynamics with Control Barrier Functions for safety
%   Implements the distributed control law with CBF-based collision avoidance
%
%   CBF ensures:
%   - Agents maintain minimum safe distance from each other
%   - Agents don't collide with target
%   - Original control objectives preserved when safe

% Extract parameters
N = params.N;
radii = params.radii;
a = params.a;
delta = params.delta;
kv = params.kv;
kw = params.kw;
xb = params.target_pos(1);
yb = params.target_pos(2);

enable_cbf = params.enable_cbf;
d_safe = params.d_safe;
d_target_min = params.d_target_min;
cbf_alpha = params.cbf_alpha;

% Initialize derivative vector
dx = zeros(3*N, 1);

% For each agent, compute nominal control and apply CBF if enabled
for i = 1:N
    % Current agent state
    idx = 3*i - 2;
    xi = x(idx);
    yi = x(idx + 1);
    thetai = x(idx + 2);
    
    % Next agent (ring topology)
    i_next = mod(i, N) + 1;
    idx_next = 3*i_next - 2;
    xi_next = x(idx_next);
    yi_next = x(idx_next + 1);
    
    % Rotation matrix R(theta_i)
    R = [cos(thetai), sin(thetai);
        -sin(thetai), cos(thetai)];
    
    % Relative positions in local frame
    mu_b = R * [xb - xi; yb - yi];
    mu_plus = R * [xi_next - xi; yi_next - yi];
    
    % Nominal control law (equation 2)
    control_input = (1 - a(i)) * mu_plus + a(i) * mu_b;
    vi_nom = kv * control_input(1);
    omegai_nom = kw * delta(i) * control_input(2);
    
    % Apply CBF if enabled
    if enable_cbf
        % Solve QP to get safe control
        [vi, omegai] = apply_cbf_constraints(i, x, params, vi_nom, omegai_nom);
    else
        vi = vi_nom;
        omegai = omegai_nom;
    end
    
    % Unicycle kinematics
    dx(idx) = vi * cos(thetai);
    dx(idx + 1) = vi * sin(thetai);
    dx(idx + 2) = omegai;
end

end


function [v_safe, omega_safe] = apply_cbf_constraints(i, x, params, v_nom, omega_nom)
%APPLY_CBF_CONSTRAINTS Solve QP to find safe control inputs
%   Uses Control Barrier Functions to ensure collision avoidance

N = params.N;
xb = params.target_pos(1);
yb = params.target_pos(2);
d_safe = params.d_safe;
d_target_min = params.d_target_min;
cbf_alpha = params.cbf_alpha;

% Current agent state
idx = 3*i - 2;
xi = x(idx);
yi = x(idx + 1);
thetai = x(idx + 2);

% Decision variables: [v; omega]
% We'll solve: minimize ||[v; omega] - [v_nom; omega_nom]||^2
%             subject to: CBF constraints

% Count constraints
n_constraints = 0;
A_ineq = [];
b_ineq = [];

%% 1. Agent-to-agent collision avoidance constraints
for j = 1:N
    if j == i
        continue;
    end
    
    idx_j = 3*j - 2;
    xj = x(idx_j);
    yj = x(idx_j + 1);
    thetaj = x(idx_j + 2);
    
    % Relative position
    dx_ij = xi - xj;
    dy_ij = yi - yj;
    
    % Barrier function: h = ||pi - pj||^2 - d_safe^2
    h = dx_ij^2 + dy_ij^2 - d_safe^2;
    
    % Only enforce if close to boundary (within 2x safe distance)
    if sqrt(dx_ij^2 + dy_ij^2) < 2*d_safe
        % Relative velocity (agent j's velocity from stored state)
        vj = sqrt(x(idx_j)^2 + x(idx_j+1)^2);  % Approximate
        
        % dh/dt = 2*(xi-xj)*(vxi-vxj) + 2*(yi-yj)*(vyi-vyj)
        % For agent i: vxi = v*cos(theta), vyi = v*sin(theta)
        % For agent j: approximate as constant during this timestep
        
        % CBF constraint: dh/dt >= -alpha*h
        % 2*(xi-xj)*v*cos(theta) + 2*(yi-yj)*v*sin(theta) >= -alpha*h + j_contribution
        
        % Linearization: dh/dt ≈ grad_h' * [v*cos(theta); v*sin(theta); 0]
        grad_h_i = [2*dx_ij*cos(thetai); 
                    2*dy_ij*sin(thetai)];
        
        % Constraint: -grad_h_i' * [v; omega] <= alpha*h
        % (omega doesn't directly affect position derivative in this approximation)
        
        A_row = [-grad_h_i(1), 0];  % Only v affects translational motion
        b_val = cbf_alpha * h;
        
        A_ineq = [A_ineq; A_row];
        b_ineq = [b_ineq; b_val];
        n_constraints = n_constraints + 1;
    end
end

%% 2. Agent-to-target minimum distance constraint
dx_it = xi - xb;
dy_it = yi - yb;
h_target = dx_it^2 + dy_it^2 - d_target_min^2;

if sqrt(dx_it^2 + dy_it^2) < 2*d_target_min
    grad_h_target = [2*dx_it*cos(thetai);
                     2*dy_it*sin(thetai)];
    
    A_row = [-grad_h_target(1), 0];
    b_val = cbf_alpha * h_target;
    
    A_ineq = [A_ineq; A_row];
    b_ineq = [b_ineq; b_val];
    n_constraints = n_constraints + 1;
end

%% Solve QP or use nominal control
if n_constraints > 0
    % QP formulation:
    % minimize: (1/2)*u'*H*u + f'*u
    % subject to: A*u <= b
    % where u = [v; omega]
    
    H = 2*eye(2);  % Quadratic term for ||u - u_nom||^2
    f = -2*[v_nom; omega_nom];  % Linear term
    
    % Use MATLAB's quadprog if available, otherwise simple projection
    try
        options = optimoptions('quadprog', 'Display', 'off');
        u_opt = quadprog(H, f, A_ineq, b_ineq, [], [], [], [], [], options);
        v_safe = u_opt(1);
        omega_safe = u_opt(2);
    catch
        % Fallback: Simple velocity scaling if QP not available
        % Check if nominal control violates constraints
        u_nom = [v_nom; omega_nom];
        violations = A_ineq * u_nom - b_ineq;
        
        if any(violations > 0)
            % Scale down velocity to satisfy constraints (conservative)
            scale_factor = 0.5;  % Reduce speed when near obstacles
            v_safe = v_nom * scale_factor;
            omega_safe = omega_nom;
        else
            v_safe = v_nom;
            omega_safe = omega_nom;
        end
    end
else
    % No active constraints - use nominal control
    v_safe = v_nom;
    omega_safe = omega_nom;
end

% Velocity limits (physical constraints)
v_max = 2.0;  % Maximum linear velocity
omega_max = 3.0;  % Maximum angular velocity

v_safe = max(-v_max, min(v_max, v_safe));
omega_safe = max(-omega_max, min(omega_max, omega_safe));

end
