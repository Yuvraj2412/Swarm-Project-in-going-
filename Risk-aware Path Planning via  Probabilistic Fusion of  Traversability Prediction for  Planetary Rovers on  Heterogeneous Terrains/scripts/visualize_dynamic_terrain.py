"""
Visualize Dynamic Terrain Dataset
Creates sample visualizations of the dynamic terrain with obstacles
"""
import numpy as np
import matplotlib.pyplot as plt
import sys, os

BASE_PATH = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.join(BASE_PATH, '..')
sys.path.append(PROJECT_ROOT)

from planning_project.env.env import GridMap
from planning_project.env.slip_models import SlipModelsGenerator
import random

def generate_sample_dynamic_terrain(seed=42):
    """Generate a sample dynamic terrain for visualization"""
    np.random.seed(seed)
    random.seed(seed)
    
    # Create base terrain
    n = 96
    grid_map = GridMap(n, 1, seed=seed)
    grid_map.set_terrain_env(
        is_crater=True, 
        is_fractal=True, 
        num_crater=7,  # More craters for dynamic environment
        max_a=25, 
        max_r=30
    )
    
    # Set terrain distribution
    occ = [0.15, 0.1, 0.2, 0.05, 0.15, 0.1, 0.1, 0.05, 0.05, 0.05]
    grid_map.set_terrain_distribution(occ=occ, type_dist="noise")
    
    # Add dynamic obstacles
    obstacle_map = np.zeros((n, n), dtype=np.int32)
    n_dynamic_objects = 5
    object_sizes = [(3, 3), (5, 5), (7, 7)]
    object_types = ['rock', 'crater', 'debris']
    
    for i in range(n_dynamic_objects):
        margin = 10
        x = np.random.randint(margin, n - margin)
        y = np.random.randint(margin, n - margin)
        
        size_w, size_h = random.choice(object_sizes)
        obj_type = random.choice(object_types)
        
        x_start = max(0, x - size_w // 2)
        x_end = min(n, x + size_w // 2)
        y_start = max(0, y - size_h // 2)
        y_end = min(n, y + size_h // 2)
        
        if obj_type == 'rock':
            obstacle_map[x_start:x_end, y_start:y_end] = 1
        elif obj_type == 'crater':
            obstacle_map[x_start:x_end, y_start:y_end] = 2
        else:  # debris
            obstacle_map[x_start:x_end, y_start:y_end] = 3
    
    # Get terrain data
    height_map = np.reshape(grid_map.data.height, (n, n))
    color_map = grid_map.data.color
    
    # Modify height based on obstacles
    height_map_with_obstacles = height_map + obstacle_map * 0.2
    
    return {
        'height_map': height_map,
        'height_with_obstacles': height_map_with_obstacles,
        'color_map': color_map,
        'obstacle_map': obstacle_map,
        'grid_size': n
    }

def visualize_dynamic_terrain(terrain_data, save_path='dynamic_terrain_sample.png'):
    """Create visualization of dynamic terrain"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    
    # 1. Height Map (Original)
    ax1 = axes[0, 0]
    im1 = ax1.imshow(terrain_data['height_map'], cmap='terrain', origin='lower')
    ax1.set_title('Base Height Map', fontsize=14, fontweight='bold')
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    plt.colorbar(im1, ax=ax1, label='Height')
    
    # 2. Height Map with Obstacles
    ax2 = axes[0, 1]
    im2 = ax2.imshow(terrain_data['height_with_obstacles'], cmap='terrain', origin='lower')
    ax2.set_title('Height Map with Dynamic Obstacles', fontsize=14, fontweight='bold')
    ax2.set_xlabel('X Position')
    ax2.set_ylabel('Y Position')
    plt.colorbar(im2, ax=ax2, label='Height')
    
    # 3. Terrain Color/Type Distribution
    ax3 = axes[1, 0]
    im3 = ax3.imshow(terrain_data['color_map'], cmap='tab10', origin='lower')
    ax3.set_title('Terrain Type Distribution', fontsize=14, fontweight='bold')
    ax3.set_xlabel('X Position')
    ax3.set_ylabel('Y Position')
    plt.colorbar(im3, ax=ax3, label='Terrain Type')
    
    # 4. Obstacle Map
    ax4 = axes[1, 1]
    obstacle_colors = np.ma.masked_where(terrain_data['obstacle_map'] == 0, terrain_data['obstacle_map'])
    im4 = ax4.imshow(terrain_data['height_map'], cmap='gray', origin='lower', alpha=0.3)
    im4_obstacles = ax4.imshow(obstacle_colors, cmap='Reds', origin='lower', alpha=0.8, vmin=0, vmax=3)
    ax4.set_title('Dynamic Obstacles Overlay', fontsize=14, fontweight='bold')
    ax4.set_xlabel('X Position')
    ax4.set_ylabel('Y Position')
    
    # Add legend for obstacles
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor='white', edgecolor='black', label='Clear'),
        Patch(facecolor='#fee5d9', label='Rock (High Slip)'),
        Patch(facecolor='#fc9272', label='Crater (Very High Slip)'),
        Patch(facecolor='#de2d26', label='Debris (Moderate Slip)')
    ]
    ax4.legend(handles=legend_elements, loc='upper right', fontsize=9)
    
    plt.suptitle('Dynamic Terrain Dataset Sample\n(With Moving Obstacles and Varying Difficulty)', 
                 fontsize=16, fontweight='bold', y=0.995)
    plt.tight_layout()
    
    # Save figure
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"✓ Visualization saved to: {save_path}")
    
    return fig

def create_multiple_samples(num_samples=3):
    """Create multiple sample visualizations"""
    print("="*70)
    print("DYNAMIC TERRAIN VISUALIZATION")
    print("="*70)
    
    for i in range(num_samples):
        print(f"\nGenerating sample {i+1}/{num_samples}...")
        seed = 42 + i * 100
        
        terrain_data = generate_sample_dynamic_terrain(seed=seed)
        save_path = f'dynamic_terrain_sample_{i+1}.png'
        
        visualize_dynamic_terrain(terrain_data, save_path)
        
        # Print statistics
        n_obstacles = np.sum(terrain_data['obstacle_map'] > 0)
        obstacle_coverage = (n_obstacles / (terrain_data['grid_size'] ** 2)) * 100
        
        print(f"  Grid size: {terrain_data['grid_size']}x{terrain_data['grid_size']}")
        print(f"  Obstacles: {n_obstacles} cells ({obstacle_coverage:.1f}% coverage)")
        print(f"  Height range: {terrain_data['height_map'].min():.2f} to {terrain_data['height_map'].max():.2f}")
    
    print("\n" + "="*70)
    print("✓ VISUALIZATION COMPLETE!")
    print("="*70)
    print(f"\nGenerated {num_samples} sample visualizations:")
    for i in range(num_samples):
        print(f"  - dynamic_terrain_sample_{i+1}.png")
    print("\nThese show what the dynamic terrain dataset will look like:")
    print("  • Base terrain with craters and fractal patterns")
    print("  • Dynamic obstacles (rocks, craters, debris)")
    print("  • Varying terrain types and difficulty zones")
    print("  • Height variations for challenging navigation")

if __name__ == "__main__":
    create_multiple_samples(num_samples=3)
