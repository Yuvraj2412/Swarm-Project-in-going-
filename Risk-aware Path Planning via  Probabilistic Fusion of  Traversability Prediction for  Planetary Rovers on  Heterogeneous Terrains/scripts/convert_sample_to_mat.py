
import numpy as np
import scipy.io
import os
import sys

def convert_sample():
    # Path to a sample file
    # We try to get one from the dynamic dataset
    base_path = os.path.join(os.path.dirname(__file__), '..', 'datasets', 'dataset_DynamicFixed', 'train')
    
    # Try different instances to find one with obstacles clearly visible
    # Instance 10 usually has obstacles moved from start
    filename = "env_00_0010.npy" 
    npy_path = os.path.join(base_path, filename)
    
    if not os.path.exists(npy_path):
        # Fallback to 0000
        filename = "env_00_0000.npy"
        npy_path = os.path.join(base_path, filename)
        
    if not os.path.exists(npy_path):
        print(f"Error: Could not find sample file at {npy_path}")
        return

    print(f"Loading {filename}...")
    data = np.load(npy_path, allow_pickle=True).item()
    
    height = data['height']
    color_map = data['input']
    
    # Check format
    if color_map.shape[0] == 3:
        # CHW -> HWC for MATLAB
        color_map = np.transpose(color_map, (1, 2, 0))
        
    # MATLAB expects double/single normally for surf CData
    mat_data = {
        'height': height,
        'color_map': color_map,
        'generated_file': filename
    }
    
    out_path = os.path.join(os.path.dirname(__file__), '..', 'sample_3d_map.mat')
    scipy.io.savemat(out_path, mat_data)
    print(f"✓ Saved converted data to {out_path}")

if __name__ == "__main__":
    convert_sample()
