
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os
import random
from matplotlib.animation import FuncAnimation

# Setup paths
BASE_PATH = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.join(BASE_PATH, '..')
sys.path.append(PROJECT_ROOT)

from planning_project.env.env import GridMap

class Dynamic3DVisualizer:
    def __init__(self, n=96):
        self.n = n
        self.object_sizes = [(3, 3), (5, 5), (7, 7)]
        self.object_types = ['rock', 'crater', 'debris']
        self.n_dynamic_objects = 5
        
    def generate_base_terrain(self, seed=42):
        np.random.seed(seed)
        random.seed(seed)
        grid_map = GridMap(self.n, 1, seed=seed)
        # Use parameters similar to the actual generator
        grid_map.set_terrain_env(
            is_crater=True, 
            is_fractal=True, 
            num_crater=7, 
            max_a=25, 
            max_r=30
        )
        return grid_map

    def add_objects(self, grid_map, seed=None):
        if seed is not None:
            np.random.seed(seed)
            random.seed(seed)
            
        obstacle_map = np.zeros((self.n, self.n), dtype=np.int32)
        
        for _ in range(self.n_dynamic_objects):
            margin = 10
            x = np.random.randint(margin, self.n - margin)
            y = np.random.randint(margin, self.n - margin)
            size_w, size_h = random.choice(self.object_sizes)
            obj_type = random.choice(self.object_types)
            
            x_start = max(0, x - size_w // 2)
            x_end = min(self.n, x + size_w // 2)
            y_start = max(0, y - size_h // 2)
            y_end = min(self.n, y + size_h // 2)
            
            if obj_type == 'rock': val = 1
            elif obj_type == 'crater': val = 2
            else: val = 3
            
            obstacle_map[x_start:x_end, y_start:y_end] = val
            
        return obstacle_map

    def visualize(self, save_path="dynamic_3d.png", make_gif=False):
        print("Generating 3D Visualization...")
        grid_map = self.generate_base_terrain()
        base_height = np.reshape(grid_map.data.height, (self.n, self.n))
        
        X = np.arange(0, self.n, 1)
        Y = np.arange(0, self.n, 1)
        X, Y = np.meshgrid(X, Y)
        
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        def update(frame):
            ax.clear()
            seed = 100 + frame
            # Generate new random obstacles for this frame (simulating the dataset's behavior)
            obs_map = self.add_objects(grid_map, seed=seed)
            
            # Combine height: Base + Obstacles
            # Exaggerate obstacle height clearly for 3D viz
            Z = base_height + obs_map * 1.5 
            
            # Create a localized color array
            # Normalize Z for base colormap
            Z_norm = (Z - Z.min()) / (Z.max() - Z.min())
            
            # Base color map (Terra/Earth tones)
            colors = plt.cm.gist_earth(Z_norm)
            
            # Overlay specific colors for obstacles to make them pop
            # Rock: Dark Grey
            colors[obs_map == 1] = [0.3, 0.3, 0.3, 1.0] 
            # Crater: Dark Red/Maroon
            colors[obs_map == 2] = [0.6, 0.1, 0.1, 1.0]
            # Debris: Gold/Yellow
            colors[obs_map == 3] = [0.8, 0.7, 0.1, 1.0]
            
            surf = ax.plot_surface(X, Y, Z, facecolors=colors, linewidth=0, antialiased=False, shade=True)
            
            title = "Dynamic Terrain Dataset 3D Visualization\n"
            title += "Note: Obstacles randomly respawn (current dataset behavior)" if make_gif else "Single Snapshot"
            ax.set_title(title)
            ax.set_zlim(0, 30)
            
            # Rotate view
            ax.view_init(elev=60, azim=frame if make_gif else 45)
            return surf,

        if make_gif:
            print(f"Creating animation (this may take a moment)...")
            # 30 Frames, rotating 
            frames = 30
            anim = FuncAnimation(fig, update, frames=frames, interval=200)
            gif_path = save_path.replace('.png', '.gif')
            anim.save(gif_path, writer='pillow')
            print(f"✓ Saved 3D GIF to {gif_path}")
        else:
            update(0)
            plt.savefig(save_path, dpi=150)
            print(f"✓ Saved 3D Image to {save_path}")

if __name__ == "__main__":
    viz = Dynamic3DVisualizer()
    # Generate a GIF to show the "dynamic" (random) nature
    viz.visualize(save_path="dynamic_terrain_3d_viz.png", make_gif=True)
