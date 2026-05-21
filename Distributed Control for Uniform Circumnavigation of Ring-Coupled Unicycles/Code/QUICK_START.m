%% QUICK START GUIDE
% Distributed Circumnavigation Algorithm Implementation
% ====================================================

%% METHOD 1: Run the default example
% Simply execute the main script:

main_circumnavigation

% This will simulate 6 agents (3 on radius 2, 3 on radius 3)
% with real-time animation.

%% METHOD 2: Run predefined examples

% Example A: All agents on same orbit
example_same_radius

% Example B: Agents on different orbits (replicates paper)
example_different_radii

%% METHOD 3: Custom configuration

% Open main_circumnavigation.m and modify these parameters:

N = 8;                          % Number of agents
radii = [2, 2, 2, 2, 3, 3, 3, 3];  % Radius for each agent
d = 1;                          % Formation parameter (1 to N-1)
target_pos = [0, 0];            % Target location
t_final = 30;                   % Simulation time (seconds)

% Then run:
main_circumnavigation

%% METHOD 4: Check if your parameters are valid

% Before running a simulation, verify your parameters:

check_parameters(6, 1, [2,2,2,3,3,3], 2.75, 3.897)

% This will tell you if your configuration is stable!

%% COMMON CONFIGURATIONS

% 1. Simple case: 6 agents, all on radius 2, clockwise
N = 6; radii = 2*ones(1,6); d = 1;

% 2. Two orbits: 4 agents on r=2, 4 agents on r=3
N = 8; radii = [2,2,2,2,3,3,3,3]; d = 1;

% 3. Three orbits
N = 9; radii = [1.5,1.5,1.5,2,2,2,3,3,3]; d = 1;

%% PARAMETER MEANINGS

% N - Number of agents (must be ≥ 3)
% radii - Vector of desired radii (length must equal N)
% d - Formation spacing parameter:
%     d = 1: minimum spacing (clockwise, MOST STABLE)
%     d = N-1: maximum spacing (counterclockwise, MOST STABLE)
%     Other values: intermediate spacings (may be less stable)
% target_pos - [x, y] coordinates of target
% t_final - How long to simulate

%% TIPS

% • For stable results: use d = 1 or d = N-1
% • Simulation takes longer with more agents
% • Set animate_realtime = false for faster simulation
% • Increase t_final if formation hasn't converged yet
% • All radii must be positive
% • Control gains are auto-calculated (don't worry about them)

%% TROUBLESHOOTING

% Problem: "Length of radii must equal N"
% Solution: Make sure radii vector has exactly N elements
%   Example: N=5 → radii = [2, 2, 3, 3, 3]  ✓
%            N=5 → radii = [2, 3]           ✗

% Problem: Agents don't reach desired orbits
% Solution: Increase t_final or check parameters with:
%   check_parameters(N, d, radii, kv, kw)

% Problem: Animation is too slow/fast
% Solution: Change animation_speed in main script
%   animation_speed = 0.5  → slow motion
%   animation_speed = 2.0  → 2x speed

%% FILE DESCRIPTIONS

% main_circumnavigation.m  → Main simulation (START HERE!)
% unicycle_dynamics.m      → System dynamics (don't modify)
% animate_circumnavigation.m → Animation (don't modify)
% check_parameters.m       → Validate your settings
% example_same_radius.m    → Example: all agents same orbit
% example_different_radii.m → Example: different orbits
% README.md               → Full documentation

%% MINIMAL WORKING EXAMPLE

% Copy and paste this into MATLAB command window:
% -------------------------------------------------
% N = 6;
% radii = [2, 2, 2, 3, 3, 3];
% d = 1;
% target_pos = [0, 0];
% t_final = 25;
% kv = 2.75;
% 
% % Auto-calculate other parameters
% psi_bar = 2*d*pi/N;
% c = 1 - 0.1;
% kw = kv / ((1 - c*cos(psi_bar)) / abs(c*sin(psi_bar)));
% delta = 1 ./radii;
% a = zeros(1, N);
% for i = 1:N
%     i_next = mod(i, N) + 1;
%     a(i) = 1 - c * (radii(i) / radii(i_next));
% end
% 
% % Initial conditions
% x0 = zeros(3*N, 1);
% for i = 1:N
%     angle = 2*pi*rand();
%     x0(3*i-2:3*i) = [4*cos(angle); 4*sin(angle); 2*pi*rand()];
% end
% 
% % Simulate
% params.N = N; params.radii = radii; params.a = a;
% params.delta = delta; params.kv = kv; params.kw = kw;
% params.target_pos = target_pos;
% [t, x] = ode45(@(t,x) unicycle_dynamics(t,x,params), [0 t_final], x0);
% animate_circumnavigation(t, x, params, 1.0);
% -------------------------------------------------

%% NEXT STEPS

% 1. Run: main_circumnavigation
% 2. Try: example_same_radius
% 3. Modify parameters in main_circumnavigation.m
% 4. Check validity: check_parameters(...)
% 5. Read README.md for full details

% Happy simulating! 🚁

%Modifications-:
%1. Use collisson avoidance algorithms like CBS
%2. We can modify the code such that we can set the angle between the
%agents and accordingly the Kv or Kw will change (nothing else will be
%affected