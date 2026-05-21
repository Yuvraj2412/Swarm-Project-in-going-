function dx = unicycle_dynamics(t, x, params)
%UNICYCLE_DYNAMICS Dynamics of ring-coupled unicycles with distributed control
%   Implements control law (2) from Zheng et al., Automatica 53 (2015) 23-29
%
%   Inputs:
%       t      - Current time
%       x      - State vector [x1,y1,theta1, x2,y2,theta2, ..., xN,yN,thetaN]
%       params - Structure containing:
%                .N: number of agents
%                .radii: desired radii for each agent
%                .a: control parameters ai
%                .delta: control parameters δi
%                .kv: linear velocity gain
%                .kw: angular velocity gain
%                .target_pos: target position [xb, yb]
%
%   Output:
%       dx - Time derivative of state vector

% Extract parameters
N = params.N;
radii = params.radii;
a = params.a;
delta = params.delta;
kv = params.kv;
kw = params.kw;
xb = params.target_pos(1);         %Just to clear these are relative coordinates to target not global
yb = params.target_pos(2);

% Initialize derivative vector
dx = zeros(3*N, 1);               %Initialize the velocity vector with zeroes as entries

% For each agent
for i = 1:N
    % Current agent index
    idx = 3*i - 2;  % Starting index for agent i
    
    % Extract current state
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
    
    % Relative position of target in local frame: μ_b^(i)
    mu_b = R * [xb - xi; yb - yi];
    
    % Relative position of next agent in local frame: μ_+^(i)
    mu_plus = R * [xi_next - xi; yi_next - yi];
    
    % Control law (2): [vi; ωi] = [kv, 0; 0, kw*δi] * [(1-ai)*μ_+ + ai*μ_b]
    control_input = (1 - a(i)) * mu_plus + a(i) * mu_b;
    
    vi = kv * control_input(1);
    omegai = kw * delta(i) * control_input(2);
    
    % Unicycle kinematics (1)
    dx(idx) = vi * cos(thetai);       % ẋi
    dx(idx + 1) = vi * sin(thetai);   % ẏi
    dx(idx + 2) = omegai;              % θ̇i
end

end
