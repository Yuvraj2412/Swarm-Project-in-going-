
import numpy as np
import matplotlib.pyplot as plt
import os

def verify_dynamic_dataset():
    base_path = "datasets/dataset_DynamicFixed/train/"
    
    # Load two instances from same environment
    file1 = os.path.join(base_path, "env_00_0000.npy")
    file2 = os.path.join(base_path, "env_00_0001.npy")
    
    if not os.path.exists(file1) or not os.path.exists(file2):
        print(f"Files not found. Waiting for generation... Checked: {file1}")
        return
    
    data1 = np.load(file1, allow_pickle=True).item()
    data2 = np.load(file2, allow_pickle=True).item()
    
    h1 = data1['height']
    h2 = data2['height']
    c1 = data1['input'] # [3, N, N] or [N, N, 3]
    c2 = data2['input']
    
    # Check dimensions
    print(f"Height shape: {h1.shape}")
    print(f"Color shape: {c1.shape}")
    
    # Difference
    diff_h = np.abs(h1 - h2)
    
    if c1.shape[0] == 3:
        # Channels first
        diff_c = np.abs(c1 - c2).sum(axis=0)
    else:
        diff_c = np.abs(c1 - c2).sum(axis=2)
        
    print(f"Max Height Diff: {diff_h.max()}")
    print(f"Max Color Diff: {diff_c.max()}")
    
    if diff_h.max() > 0:
        print("✓ SUCCESS: Height map changes between instances (Objects moving).")
    else:
        print("✗ FAILURE: Height map identical.")
        
    if diff_c.max() > 0:
        print("✓ SUCCESS: Color map changes between instances (Objects visible).")
    else:
        print("✗ FAILURE: Color map identical (Objects invisible in RGB).")

    # Visualize
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    
    # Instance 1
    axes[0, 0].imshow(h1, cmap='terrain')
    axes[0, 0].set_title('Inst 0 Height')
    
    if c1.shape[0] == 3:
        img1 = np.transpose(c1, (1, 2, 0))
        img2 = np.transpose(c2, (1, 2, 0))
    else:
        img1 = c1
        img2 = c2
        
    axes[0, 1].imshow(img1)
    axes[0, 1].set_title('Inst 0 Color')
    
    # Instance 2
    axes[1, 0].imshow(h2, cmap='terrain')
    axes[1, 0].set_title('Inst 1 Height')
    axes[1, 1].imshow(img2)
    axes[1, 1].set_title('Inst 1 Color')
    
    # Diff
    axes[0, 2].imshow(diff_h, cmap='hot')
    axes[0, 2].set_title('Height Diff (Moving Objects)')
    axes[1, 2].imshow(diff_c, cmap='hot')
    axes[1, 2].set_title('Color Diff')
    
    plt.savefig('verification_dynamic_fixed.png')
    print("✓ Saved verification_dynamic_fixed.png")

if __name__ == "__main__":
    verify_dynamic_dataset()
