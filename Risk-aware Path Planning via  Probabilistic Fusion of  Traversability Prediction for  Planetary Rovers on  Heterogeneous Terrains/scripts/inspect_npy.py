
import numpy as np
import os
import sys

def inspect_file(path, name):
    print(f"\n--- {name} ({path}) ---")
    try:
        if not os.path.exists(path):
            print("File not found.")
            return

        data = np.load(path, allow_pickle=True)
        print(f"Type: {type(data)}")
        print(f"Shape: {data.shape}")
        
        try:
            item = data.item()
            print(f"Item type: {type(item)}")
            if isinstance(item, dict):
                print(f"Keys: {list(item.keys())}")
                for key in item:
                    val = item[key]
                    if hasattr(val, 'shape'):
                        print(f"  {key}: shape={val.shape}, dtype={val.dtype}")
                    else:
                        print(f"  {key}: type={type(val)}")
        except Exception as e:
            print(f"item() extraction failed: {e}")
            
    except Exception as e:
        print(f"Load failed: {e}")

# Inspect official data
inspect_file('datasets/dataset_Std/train/env_00_0000.npy', 'Official Std')

# Inspect dynamic data
inspect_file('datasets/dataset_Dynamic/train/env_00_0000.npy', 'Dynamic Gen')
