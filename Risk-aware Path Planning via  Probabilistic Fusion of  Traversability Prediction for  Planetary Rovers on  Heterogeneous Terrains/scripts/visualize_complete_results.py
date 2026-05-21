"""
Visualize Complete Evaluation Results
Generates comparison charts for all datasets (Std, ES, AA, Dynamic)
"""
import scipy.io
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

def visualize_results(results_path='results.mat'):
    """Visualize results from .mat file"""
    if not os.path.exists(results_path):
        print(f"File not found: {results_path}")
        return

    print(f"Loading results from {results_path}...")
    data = scipy.io.loadmat(results_path)
    
    datasets = ['Std', 'ES', 'AA', 'Dynamic']
    metrics = ['Success Rate', 'Path Length', 'Observation Time', 'Max Slip', 'Est Cost']
    
    # Extract data
    results = {}
    for ds in datasets:
        if ds in data:
            ds_data = data[ds]
            # Check structure: it seems to be an array of structs
            # 0: Mean (Risk-Neutral), 1: CVaR (Risk-Aware)
            
            # Extract risk-aware (CVaR) results which are usually the second entry
            val = ds_data[0] # Get array
            if val.size > 1:
                # Get the risk-aware model (usually index 1)
                idx = 1
            else:
                idx = 0
                
            try:
                # Access struct fields - scipy.io loads structs as numpy structured arrays
                item = val[idx]
                
                # Helper to extract scalar from nested array
                def get_scalar(arr):
                    try:
                        if isinstance(arr, np.ndarray):
                            val = arr.item() if arr.size == 1 else arr[0][0]
                            return float(val)
                        return float(arr)
                    except:
                        return 0.0

                is_solved = get_scalar(item['is_solved'])
                is_feasible = get_scalar(item['is_feasible'])
                dist_mean = get_scalar(item['dist_mean'])
                obs_time_mean = get_scalar(item['obs_time_mean'])
                max_slip_mean = get_scalar(item['max_slip_mean'])
                est_cost_mean = get_scalar(item['est_cost_mean'])
                
                success_rate = (is_feasible / is_solved * 100) if is_solved > 0 else 0
                
                results[ds] = {
                    'Success Rate': success_rate,
                    'Path Length': dist_mean,
                    'Observation Time': obs_time_mean,
                    'Max Slip': max_slip_mean,
                    'Est Cost': est_cost_mean
                }
            except Exception as e:
                print(f"Error parsing {ds}: {e}")
                # Print debug info
                try:
                    print(f"  Debug {ds}: item shape={item.shape}, dtype={item.dtype}")
                    print(f"  is_solved raw: {item['is_solved']}")
                except:
                    pass
    
    # Plotting
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    
    # 1. Success Rate
    ax1 = axes[0, 0]
    names = list(results.keys())
    values = [results[ds]['Success Rate'] for ds in names]
    colors = ['#3498db', '#e74c3c', '#9b59b6', '#2ecc71'] # Blue, Red, Purple, Green
    
    bars = ax1.bar(names, values, color=colors)
    ax1.set_title('Success Rate (Risk-Aware)', fontsize=14, fontweight='bold')
    ax1.set_ylabel('Success Rate (%)')
    ax1.set_ylim(0, 100)
    
    # Add labels
    for bar in bars:
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height + 1,
                f'{height:.1f}%', ha='center', va='bottom', fontweight='bold')
    
    # 2. Observation Time
    ax2 = axes[0, 1]
    values = [results[ds]['Observation Time'] for ds in names]
    bars = ax2.bar(names, values, color=colors)
    ax2.set_title('Avg. Observation Time', fontsize=14, fontweight='bold')
    ax2.set_ylabel('Time (s)')
    
    for bar in bars:
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.1f}s', ha='center', va='bottom')

    # 3. Path Length
    ax3 = axes[1, 0]
    values = [results[ds]['Path Length'] for ds in names]
    bars = ax3.bar(names, values, color=colors)
    ax3.set_title('Avg. Path Length', fontsize=14, fontweight='bold')
    ax3.set_ylabel('Distance (m)')
    
    for bar in bars:
        height = bar.get_height()
        ax3.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.1f}m', ha='center', va='bottom')

    # 4. Max Slip
    ax4 = axes[1, 1]
    values = [results[ds]['Max Slip'] for ds in names]
    bars = ax4.bar(names, values, color=colors)
    ax4.set_title('Avg. Maximum Slip', fontsize=14, fontweight='bold')
    ax4.set_ylabel('Slip Ratio')
    
    for bar in bars:
        height = bar.get_height()
        ax4.text(bar.get_x() + bar.get_width()/2., height,
                f'{height:.2f}', ha='center', va='bottom')

    plt.suptitle('Optimized Rover Navigation Pipeline Results\nComparison across Standard, Extreme, Alien, and Dynamic Datasets', 
                 fontsize=16, fontweight='bold', y=0.98)
    plt.tight_layout()
    
    output_path = 'final_results_visualization.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n✓ Visualization saved to: {output_path}")
    
    # Print summary table
    print("\n" + "="*60)
    print(f"{'Dataset':<10} {'Success':<10} {'Time(s)':<10} {'Dist(m)':<10} {'Slip':<10}")
    print("-" * 60)
    for ds in names:
        r = results[ds]
        print(f"{ds:<10} {r['Success Rate']:<10.1f} {r['Observation Time']:<10.1f} {r['Path Length']:<10.1f} {r['Max Slip']:<10.2f}")
    print("=" * 60)

if __name__ == "__main__":
    visualize_results()
