import numpy as np
import os

def calculate_obstacle_density():
    """Calculate actual obstacle density from generated dataset"""
    
    data_dir = "datasets/dataset_DynamicFixed/train"
    
    if not os.path.exists(data_dir):
        print(f"Dataset not found at {data_dir}")
        return
    
    # Sample files from different environments and instances
    sample_files = [
        "env_00_0000.npy",  # Env 0, start
        "env_00_0050.npy",  # Env 0, middle
        "env_00_0099.npy",  # Env 0, end
        "env_05_0000.npy",  # Env 5, start
        "env_09_0099.npy",  # Env 9, end
    ]
    
    densities = []
    
    for filename in sample_files:
        filepath = os.path.join(data_dir, filename)
        if os.path.exists(filepath):
            data = np.load(filepath, allow_pickle=True).item()
            
            # Get color map
            color_map = data['input']
            
            # Check if obstacles are present (looking for gray, red, yellow colors)
            # Obstacles have distinct colors: rock=(0.3,0.3,0.3), crater=(0.6,0.1,0.1), debris=(0.8,0.7,0.1)
            
            if color_map.shape[0] == 3:
                # [3, H, W]
                r = color_map[0]
                g = color_map[1]
                b = color_map[2]
            else:
                # [H, W, 3]
                r = color_map[:, :, 0]
                g = color_map[:, :, 1]
                b = color_map[:, :, 2]
            
            # Detect obstacles by color
            # Rock: gray (r≈g≈b≈0.3)
            rock_mask = (np.abs(r - 0.3) < 0.05) & (np.abs(g - 0.3) < 0.05) & (np.abs(b - 0.3) < 0.05)
            
            # Crater: red (r≈0.6, g≈0.1, b≈0.1)
            crater_mask = (np.abs(r - 0.6) < 0.05) & (np.abs(g - 0.1) < 0.05) & (np.abs(b - 0.1) < 0.05)
            
            # Debris: yellow (r≈0.8, g≈0.7, b≈0.1)
            debris_mask = (np.abs(r - 0.8) < 0.05) & (np.abs(g - 0.7) < 0.05) & (np.abs(b - 0.1) < 0.05)
            
            # Combined obstacle mask
            obstacle_mask = rock_mask | crater_mask | debris_mask
            
            obstacle_pixels = np.sum(obstacle_mask)
            total_pixels = obstacle_mask.size
            density = (obstacle_pixels / total_pixels) * 100
            
            densities.append(density)
            
            print(f"{filename}: {obstacle_pixels}/{total_pixels} pixels = {density:.2f}%")
            print(f"  Rocks: {np.sum(rock_mask)} | Craters: {np.sum(crater_mask)} | Debris: {np.sum(debris_mask)}")
        else:
            print(f"File not found: {filename}")
    
    if densities:
        print(f"\n{'='*50}")
        print(f"Average Obstacle Density: {np.mean(densities):.2f}%")
        print(f"Min: {np.min(densities):.2f}% | Max: {np.max(densities):.2f}%")
        print(f"{'='*50}")

if __name__ == "__main__":
    calculate_obstacle_density()
