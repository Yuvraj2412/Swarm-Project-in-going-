import scipy.io
import os

try:
    mat = scipy.io.loadmat('results.mat')
    print("Keys found in results.mat:")
    for key in mat:
        if not key.startswith('__'):
            print(f"- {key}: {type(mat[key])}")
            # Try to print first element if it's iterable
            val = mat[key]
            if hasattr(val, 'shape'):
                 print(f"  Shape: {val.shape}")
                 if val.size > 0:
                     print(f"  First item type: {type(val[0])}")
                     print(f"  First item content (brief): {val[0]}")
except Exception as e:
    print(f"Error loading results.mat: {e}")
