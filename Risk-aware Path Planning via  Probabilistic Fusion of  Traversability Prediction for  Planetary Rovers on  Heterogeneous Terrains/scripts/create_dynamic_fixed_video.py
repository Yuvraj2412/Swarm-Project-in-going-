
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import os
import sys

def create_video_from_dataset():
    # Path to generated files
    data_dir = os.path.join(os.path.dirname(__file__), '..', 'datasets', 'dataset_DynamicFixed', 'train')
    
    if not os.path.exists(data_dir):
        print(f"Dataset directory not found: {data_dir}")
        return

    # Load first 30 instances from Environment 00
    # They share the same terrain, but obstacles move.
    frames_data = []
    print("Loading data frames...")
    
    for i in range(30):
        filename = f"env_00_{i:04d}.npy"
        filepath = os.path.join(data_dir, filename)
        
        if os.path.exists(filepath):
            data = np.load(filepath, allow_pickle=True).item()
            frames_data.append(data)
        else:
            print(f"Missing file: {filename}")
            break
            
    if not frames_data:
        print("No data found!")
        return

    print(f"Loaded {len(frames_data)} frames.")
    
    n = frames_data[0]['height'].shape[0]
    X, Y = np.meshgrid(np.arange(n), np.arange(n))
    
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    def update(frame_idx):
        ax.clear()
        
        data = frames_data[frame_idx]
        height = data['height']
        color_map = data['input'] # [3,H,W] or [H,W,3]
        
        # Normalize color for display
        if color_map.shape[0] == 3:
            # [3, H, W] -> [H, W, 3]
            color_disp = np.transpose(color_map, (1, 2, 0))
        else:
            color_disp = color_map
            
        # Ensure valid range [0, 1]
        color_disp = np.clip(color_disp, 0, 1)
        
        # Plot surface
        # We perform a slight smoothing or stride to make it faster/cleaner if needed, 
        # but 96x96 is fine.
        surf = ax.plot_surface(X, Y, height, facecolors=color_disp, 
                               linewidth=0, antialiased=False, shade=True)
        
        ax.set_title(f"Dynamic Fixed Dataset: Frame {frame_idx}\n(Fixed Terrain, Moving Obstacles)")
        ax.set_zlim(0, 30)
        
        # Camera angle (rotate slightly)
        ax.view_init(elev=60, azim=45 + frame_idx * 0.5)
        
        return surf,

    print("Creating animation...")
    anim = animation.FuncAnimation(fig, update, frames=len(frames_data), interval=200)
    
    output_path = os.path.join(os.path.dirname(__file__), '..', 'dynamic_dataset_demo.gif')
    anim.save(output_path, writer='pillow')
    print(f"✓ Saved video to {output_path}")

if __name__ == "__main__":
    create_video_from_dataset()
