%% Test Prerequisites - Verify MATLAB Setup
% This script checks if your MATLAB installation is ready to run
% the circumnavigation algorithms

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║     MATLAB CIRCUMNAVIGATION - SETUP VERIFICATION       ║\n');
fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('\n');

%% 1. MATLAB Version Check
fprintf('1. MATLAB VERSION CHECK\n');
fprintf('   ───────────────────────────────────────────────────\n');

v = version;
fprintf('   Detected version: %s\n', v);

% Extract version number
version_parts = strsplit(v, ' ');
version_release = version_parts{1};

% Check if R2020b or later
version_year = str2double(regexp(version_release, '\d{4}', 'match', 'once'));
version_letter = regexp(version_release, '[ab]$', 'match', 'once');

if ~isempty(version_year) && version_year >= 2020
    if version_year > 2020 || (version_year == 2020 && strcmp(version_letter, 'b'))
        fprintf('   ✓ Version is compatible (R2020b or later)\n');
        matlab_ok = true;
    else
        fprintf('   ✗ Version too old - need R2020b or later\n');
        matlab_ok = false;
    end
else
    fprintf('   ⚠ Could not determine version - proceeding anyway\n');
    matlab_ok = true;
end
fprintf('\n');

%% 2. Toolbox Check
fprintf('2. TOOLBOX AVAILABILITY\n');
fprintf('   ───────────────────────────────────────────────────\n');

% Check Optimization Toolbox
has_optim = license('test', 'optimization_toolbox');
fprintf('   Optimization Toolbox: ');
if has_optim
    fprintf('✓ INSTALLED\n');
    try
        ver_info = ver('optim');
        fprintf('     Version: %s\n', ver_info.Version);
    catch
    end
    fprintf('     → Optimal CBF available\n');
    fprintf('     → Parameter optimization available\n');
else
    fprintf('⚠ NOT INSTALLED\n');
    fprintf('     → Fallback methods will be used\n');
    fprintf('     → Algorithm still works, but less optimal\n');
end
fprintf('\n');

%% 3. Core Functions Test
fprintf('3. CORE FUNCTIONALITY TEST\n');
fprintf('   ───────────────────────────────────────────────────\n');

all_tests_passed = true;

% Test 3a: ODE Solver
fprintf('   [Test 3a] ODE solver (ode45)...\n');
try
    [t, x] = ode45(@(t,x) -x, [0 1], 1);
    if length(t) > 1
        fprintf('             ✓ ODE solver works\n');
    else
        fprintf('             ✗ ODE solver returned empty result\n');
        all_tests_passed = false;
    end
catch ME
    fprintf('             ✗ Error: %s\n', ME.message);
    all_tests_passed = false;
end

% Test 3b: Plotting
fprintf('   [Test 3b] Plotting functions...\n');
try
    fig = figure('Visible', 'off');
    plot(1:10, (1:10).^2);
    xlabel('Test');
    close(fig);
    fprintf('             ✓ Plotting works\n');
catch ME
    fprintf('             ✗ Error: %s\n', ME.message);
    all_tests_passed = false;
end

% Test 3c: Matrix operations
fprintf('   [Test 3c] Matrix operations...\n');
try
    A = rand(3,3);
    B = inv(A);
    C = A * B;
    if norm(C - eye(3)) < 1e-10
        fprintf('             ✓ Linear algebra works\n');
    else
        fprintf('             ⚠ Numerical accuracy issue detected\n');
    end
catch ME
    fprintf('             ✗ Error: %s\n', ME.message);
    all_tests_passed = false;
end

% Test 3d: Quadratic programming (if Optimization Toolbox available)
if has_optim
    fprintf('   [Test 3d] QP solver (quadprog)...\n');
    try
        H = eye(2);
        f = -ones(2,1);
        options = optimoptions('quadprog', 'Display', 'off');
        x_sol = quadprog(H, f, [], [], [], [], zeros(2,1), ones(2,1), [], options);
        if ~isempty(x_sol)
            fprintf('             ✓ QP solver works\n');
        else
            fprintf('             ✗ QP solver returned empty\n');
            all_tests_passed = false;
        end
    catch ME
        fprintf('             ✗ Error: %s\n', ME.message);
        all_tests_passed = false;
    end
end

fprintf('\n');

%% 4. File Availability Check
fprintf('4. REQUIRED FILES CHECK\n');
fprintf('   ───────────────────────────────────────────────────\n');

required_files = {
    'main_circumnavigation.m'
    'unicycle_dynamics.m'
    'animate_circumnavigation.m'
    'main_circumnavigation_advanced.m'
    'unicycle_dynamics_cbf.m'
    'check_spacing_feasibility.m'
    'calculate_radii_from_spacings.m'
    'optimize_control_parameters.m'
};

files_present = true;
for i = 1:length(required_files)
    if exist(required_files{i}, 'file')
        fprintf('   ✓ %s\n', required_files{i});
    else
        fprintf('   ✗ %s (MISSING!)\n', required_files{i});
        files_present = false;
    end
end

if ~files_present
    fprintf('\n   ⚠ Some files are missing!\n');
    fprintf('     Make sure all .m files are in the current directory\n');
    fprintf('     Current directory: %s\n', pwd);
end
fprintf('\n');

%% 5. Memory Check
fprintf('5. SYSTEM RESOURCES\n');
fprintf('   ───────────────────────────────────────────────────\n');

if ispc
    [~, sys_view] = memory;
    total_mem = sys_view.PhysicalMemory.Total / 1e9;
    avail_mem = sys_view.PhysicalMemory.Available / 1e9;
    fprintf('   Total RAM: %.1f GB\n', total_mem);
    fprintf('   Available RAM: %.1f GB\n', avail_mem);
    if avail_mem > 2
        fprintf('   ✓ Sufficient memory\n');
    else
        fprintf('   ⚠ Low memory - simulations may be slow\n');
    end
else
    fprintf('   Memory check not available on this platform\n');
end
fprintf('\n');

%% 6. Summary and Recommendations
fprintf('╔════════════════════════════════════════════════════════╗\n');
fprintf('║                   SUMMARY & RECOMMENDATIONS            ║\n');
fprintf('╠════════════════════════════════════════════════════════╣\n');

overall_status = matlab_ok && all_tests_passed && files_present;

if overall_status && has_optim
    fprintf('║                                                        ║\n');
    fprintf('║  ✓✓✓ PERFECT SETUP ✓✓✓                                ║\n');
    fprintf('║                                                        ║\n');
    fprintf('║  Your system is fully ready!                          ║\n');
    fprintf('║                                                        ║\n');
    fprintf('║  You can use:                                         ║\n');
    fprintf('║  • Original algorithm                                 ║\n');
    fprintf('║  • Advanced algorithm with CBF                        ║\n');
    fprintf('║  • Custom spacing                                     ║\n');
    fprintf('║  • All optimization features                          ║\n');
    fprintf('║                                                        ║\n');
    fprintf('║  READY TO RUN:                                        ║\n');
    fprintf('║  >> main_circumnavigation                             ║\n');
    fprintf('║  >> main_circumnavigation_advanced                    ║\n');
    fprintf('║                                                        ║\n');
elseif overall_status && ~has_optim
    fprintf('║                                                        ║\n');
    fprintf('║  ✓ GOOD SETUP (with limitations)                      ║\n');
    fprintf('║                                                        ║\n');
    fprintf('║  Your system is ready, but:                           ║\n');
    fprintf('║                                                        ║\n');
    fprintf('║  ⚠ Optimization Toolbox not installed                 ║\n');
    fprintf('║                                                        ║\n');
    fprintf('║  What works:                                          ║\n');
    fprintf('║  ✓ Original algorithm (perfect)                       ║\n');
    fprintf('║  ✓ Advanced algorithm (fallback mode)                 ║\n');
    fprintf('║  ✓ Custom spacing (approximate)                       ║\n');
    fprintf('║                                                        ║\n');
    fprintf('║  What''s affected:                                     ║\n');
    fprintf('║  ⚠ CBF less optimal                                   ║\n');
    fprintf('║  ⚠ Parameter optimization approximate                 ║\n');
    fprintf('║                                                        ║\n');
    fprintf('║  RECOMMENDATION:                                      ║\n');
    fprintf('║  Install Optimization Toolbox for best results        ║\n');
    fprintf('║                                                        ║\n');
    fprintf('║  READY TO RUN:                                        ║\n');
    fprintf('║  >> main_circumnavigation                             ║\n');
    fprintf('║                                                        ║\n');
else
    fprintf('║                                                        ║\n');
    fprintf('║  ✗ ISSUES DETECTED                                    ║\n');
    fprintf('║                                                        ║\n');
    if ~matlab_ok
        fprintf('║  ✗ MATLAB version too old                             ║\n');
        fprintf('║    Need R2020b or later                               ║\n');
    end
    if ~all_tests_passed
        fprintf('║  ✗ Some core functions failed                         ║\n');
        fprintf('║    Check error messages above                         ║\n');
    end
    if ~files_present
        fprintf('║  ✗ Required files missing                             ║\n');
        fprintf('║    Download all .m files to same folder               ║\n');
    end
    fprintf('║                                                        ║\n');
    fprintf('║  Please fix the issues above before running           ║\n');
    fprintf('║                                                        ║\n');
end

fprintf('╚════════════════════════════════════════════════════════╝\n');
fprintf('\n');

%% 7. Quick Start Guide
fprintf('QUICK START:\n');
fprintf('───────────────────────────────────────────────────────\n');
fprintf('1. For basic uniform formation:\n');
fprintf('   >> main_circumnavigation\n\n');
fprintf('2. For custom spacing with CBF:\n');
fprintf('   >> main_circumnavigation_advanced\n\n');
fprintf('3. For examples:\n');
fprintf('   >> example_same_radius\n');
fprintf('   >> example_different_radii\n');
fprintf('   >> example_custom_spacing\n\n');
fprintf('4. For help:\n');
fprintf('   >> open README.md\n');
fprintf('   >> open README_ADVANCED.md\n');
fprintf('   >> open PREREQUISITES.md\n\n');
fprintf('───────────────────────────────────────────────────────\n\n');

%% Return status
if overall_status
    fprintf('✓ Setup verification complete - System ready!\n\n');
else
    fprintf('⚠ Setup verification complete - Issues found (see above)\n\n');
end
