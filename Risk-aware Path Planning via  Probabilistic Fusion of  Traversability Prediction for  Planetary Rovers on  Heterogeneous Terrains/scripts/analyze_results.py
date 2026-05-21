import scipy.io
import numpy as np

# Load results
data = scipy.io.loadmat('results.mat')

print("=" * 60)
print("DETAILED RESULTS ANALYSIS")
print("=" * 60)

datasets = ['Std', 'ES', 'AA']

for ds_name in datasets:
    if ds_name not in data:
        print(f"\n{ds_name}: NOT FOUND")
        continue
    
    print(f"\n{'='*60}")
    print(f"{ds_name} Dataset Results")
    print(f"{'='*60}")
    
    ds_data = data[ds_name]
    print(f"Type: {type(ds_data)}")
    print(f"Shape: {ds_data.shape}")
    
    # Try to extract structured data
    if hasattr(ds_data, 'dtype') and ds_data.dtype.names:
        print(f"Fields: {ds_data.dtype.names}")
        for field in ds_data.dtype.names:
            try:
                field_data = ds_data[field][0, 0]
                print(f"\n  {field}:")
                print(f"    Type: {type(field_data)}")
                if isinstance(field_data, np.ndarray):
                    print(f"    Shape: {field_data.shape}")
                    if field_data.size < 20:
                        print(f"    Value: {field_data}")
                    else:
                        print(f"    Value (first 5): {field_data.flatten()[:5]}")
                else:
                    print(f"    Value: {field_data}")
            except Exception as e:
                print(f"    Error accessing: {e}")
    else:
        # Try to access as nested structure
        print(f"\nRaw data preview:")
        try:
            if ds_data.size > 0:
                first_elem = ds_data[0, 0]
                print(f"  First element type: {type(first_elem)}")
                if hasattr(first_elem, 'dtype') and first_elem.dtype.names:
                    print(f"  Fields: {first_elem.dtype.names}")
                    for field in first_elem.dtype.names:
                        try:
                            val = first_elem[field]
                            print(f"    {field}: {val}")
                        except:
                            pass
        except Exception as e:
            print(f"  Error: {e}")

print("\n" + "=" * 60)
print("SUMMARY")
print("=" * 60)
print("\nTo match paper results, we need:")
print("- Success rate (Succ.)")
print("- Total time (T_total)")
print("- Max slip (s_max)")
print("\nPaper shows 'Ours (MGP+CVaR)' method with:")
print("  Std: 96% success, 23.2±5.2 time, 45.9±24.2 slip")
print("  ES:  77% success, 27.2±37.4 time, 60.5±37.1 slip")
print("  AA:  95% success, 26.4±6.3 time, 63.7±21.6 slip")
