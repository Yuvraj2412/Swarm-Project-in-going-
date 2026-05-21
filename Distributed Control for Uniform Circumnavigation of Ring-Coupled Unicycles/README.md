# Distributed Control for Uniform Circumnavigation of Ring-Coupled Unicycles

MATLAB implementation of the algorithm presented in:

**Zheng, R., Lin, Z., Fu, M., & Sun, D. (2015). Distributed control for uniform circumnavigation of ring-coupled unicycles. Automatica, 53, 23-29.**

## Overview

This code implements a distributed control strategy for a team of unicycle-type agents to achieve coordinated circumnavigation around a stationary target. The agents form uniform formations while circling on concentric orbits with predefined radii.

## File Structure

```
circumnavigation_simulation/
│
├── main_circumnavigation.m       # Main simulation script (START HERE)
├── unicycle_dynamics.m           # System dynamics with control law
├── animate_circumnavigation.m    # Real-time animation function
├── example_different_radii.m     # Example: agents on different orbits
├── example_same_radius.m         # Example: all agents on same orbit
└── README.md                     # This file
```

## Quick Start

1. **Basic Usage:**
   ```matlab
   % Simply run the main script
   main_circumnavigation
   ```

2. **Custom Configuration:**
   Edit the parameters in `main_circumnavigation.m`:
   ```matlab
   N = 6;                          % Number of agents
   radii = [2, 2, 2, 3, 3, 3];    % Desired radius for each agent
   d = 1;                          % Formation parameter
   target_pos = [0, 0];            % Target location
   t_final = 30;                   % Simulation time
   ```

## Configurable Parameters

### Essential Parameters (in `main_circumnavigation.m`)

| Parameter | Description | Valid Range | Example |
|-----------|-------------|-------------|---------|
| `N` | Number of agents | Integer ≥ 3 | `6` |
| `radii` | Desired radius for each agent (vector of length N) | Positive values | `[2, 2, 3, 3]` |
| `d` | Formation parameter (angular spacing) | 1 to N-1 | `1` |
| `target_pos` | Target position [x, y] | Any real values | `[0, 0]` |
| `t_final` | Simulation duration (seconds) | Positive | `30` |
| `kv` | Linear velocity gain | Positive | `2.75` |
| `kw` | Angular velocity gain | Positive | Auto-calculated |

### Formation Parameter `d`

The parameter `d` controls the angular spacing between agents:
- Angular spacing: `ψ̄ = 2πd/N`
- `d = 1`: Minimum spacing (most stable, clockwise)
- `d = N-1`: Maximum spacing (most stable, counterclockwise)
- Other values of `d` create different formations (may be less stable)

**Examples for N = 6:**
- `d = 1`: ψ̄ = 60° (clockwise motion)
- `d = 2`: ψ̄ = 120°
- `d = 3`: ψ̄ = 180°
- `d = 5`: ψ̄ = 300° (counterclockwise motion)

### Initial Conditions

```matlab
initial_type = 'random';   % Random positions (default)
initial_type = 'circle';   % Start on a circle
```

### Animation Settings

```matlab
animate_realtime = true;     % Enable real-time animation
animate_realtime = false;    % Skip animation, show only final result
animation_speed = 1.0;       % Real-time speed
animation_speed = 2.0;       % 2x speed
animation_speed = 0.5;       % Half speed
```

## Algorithm Description

The algorithm implements a distributed control law where each agent:
1. **Senses** the relative position of:
   - The target (in its local frame)
   - Its next neighbor agent (ring topology)
2. **Computes** control inputs (linear and angular velocity) using local information only
3. **Achieves** coordinated circular motion around the target

### Control Law (Equation 2 from paper)

For agent *i*:
```
[vᵢ]   [kᵥ    0  ] [(1-aᵢ)μ₊⁽ⁱ⁾ + aᵢμᵦ⁽ⁱ⁾]
[ωᵢ] = [0   kω δᵢ]
```

Where:
- `vᵢ, ωᵢ`: linear and angular velocities
- `μ₊⁽ⁱ⁾`: relative position of next agent in local frame
- `μᵦ⁽ⁱ⁾`: relative position of target in local frame
- `δᵢ = 1/rᵢ`: radius-dependent parameter
- `aᵢ = 1 - c(rᵢ/rᵢ₊₁)`: agent-specific parameter

## Examples

### Example 1: All Agents on Same Orbit

```matlab
N = 8;
radii = 3 * ones(1, N);  % All agents on radius 3
d = 1;                    % Clockwise formation
target_pos = [0, 0];
t_final = 25;
```

### Example 2: Agents on Two Different Orbits

```matlab
N = 6;
radii = [2, 2, 2, 4, 4, 4];  % 3 agents on r=2, 3 on r=4
d = 1;
target_pos = [5, 5];
t_final = 30;
```

### Example 3: Custom Configuration

```matlab
N = 4;
radii = [1.5, 2, 2.5, 3];  % Each agent on different orbit
d = 1;                      % Clockwise
target_pos = [-2, 3];       % Target not at origin
t_final = 40;
initial_type = 'circle';    % Start on a circle
```

## Theory Behind the Parameters

### Stability (Theorem 5 from paper)

The code automatically selects parameter `c` based on `d` and `N`:

1. **Most stable formations:** `d = 1` or `d = N-1`
   - Uses `c = 1 - ε` (ε = 0.1)
   - Guarantees asymptotic stability

2. **Special cases:**
   - Odd N, `d = ⌊N/2⌋` or `⌈N/2⌉`: uses `c = -1`
   - Even N, `d = N/2 ± 1`: uses `c = -1 + ε`

3. **Other values of d:** May not be asymptotically stable (warning issued)

### Control Gain Ratio (Theorem 3)

The ratio `kᵥ/kω` is automatically calculated to ensure convergence:

```
kᵥ/kω = (1 - c·cos(ψ̄)) / |c·sin(ψ̄)|
```

If you manually set both `kv` and `kw`, the code will adjust `kw` to match this ratio.

## Output and Visualization

The simulation produces:

1. **Real-time animation** (if enabled):
   - Agents shown as colored circles with direction indicators
   - Trajectories drawn in real-time
   - Time display
   - Target and desired orbit circles

2. **Summary plots:**
   - Distance from target vs time (convergence to desired radii)
   - Angular velocities vs time (convergence to uniform rotation)
   - Final formation diagram

## Troubleshooting

### Common Issues

1. **"Length of radii must equal N"**
   - Solution: Ensure `radii` vector has exactly `N` elements

2. **"d must be in range {1, 2, ..., N-1}"**
   - Solution: Choose valid `d` value (not 0, not N or larger)

3. **Warning: "Formation may not be asymptotically stable"**
   - Not an error, but simulation may not converge perfectly
   - Consider using `d = 1` or `d = N-1` for guaranteed stability

4. **Agents don't converge to desired radii**
   - Check the `kv/kw` ratio (code auto-adjusts, but verify)
   - Increase simulation time `t_final`
   - Verify `c` parameter is in valid range

### Performance Tips

- For faster simulation: set `animate_realtime = false`
- For smoother animation: decrease `animation_speed`
- For quicker convergence: increase `kv` (maintain ratio)

## Mathematical Background

### System Model (Equation 1)

Unicycle kinematics for agent *i*:
```
ẋᵢ = vᵢ cos(θᵢ)
ẏᵢ = vᵢ sin(θᵢ)
θ̇ᵢ = ωᵢ
```

### Convergence Conditions

The uniform circumnavigation problem requires:
1. `lim(t→∞) ‖(xᵢ,yᵢ) - (xᵦ,yᵦ)‖ = rᵢ` (reach desired orbit)
2. `lim(t→∞) ω₁ = ... = ωₙ = ω̄` (equal angular velocities)
3. `lim(t→∞) v₁/r₁ = ... = vₙ/rₙ = v̄` (proportional linear velocities)
4. `lim(t→∞) ψ₁ = ... = ψₙ = ψ̄` (uniform angular spacing)

## Citation

If you use this code in your research, please cite the original paper:

```bibtex
@article{zheng2015distributed,
  title={Distributed control for uniform circumnavigation of ring-coupled unicycles},
  author={Zheng, Ronghao and Lin, Zhiyun and Fu, Minyue and Sun, Dong},
  journal={Automatica},
  volume={53},
  pages={23--29},
  year={2015},
  publisher={Elsevier}
}
```

## License

This implementation is for educational and research purposes. Please refer to the original paper for theoretical details and proofs.

## Contact & Support

For questions or issues with this implementation:
1. Check the troubleshooting section above
2. Verify parameter values are within valid ranges
3. Review the original paper for theoretical background

---

**Last Updated:** February 2026
**MATLAB Version:** Tested on R2020b and later
**Dependencies:** None (uses base MATLAB only)
