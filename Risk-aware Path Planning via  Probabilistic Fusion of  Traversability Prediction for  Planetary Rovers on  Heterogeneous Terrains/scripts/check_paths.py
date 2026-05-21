import os

base_path = os.path.dirname(os.path.abspath(__file__))
print(f"Base path: {base_path}")

datasets = ['Std', 'ES', 'AA']

for dataset in datasets:
    model_path = os.path.join(base_path, f"../trained_models/models/dataset_{dataset}/best_model.pth")
    abs_path = os.path.abspath(model_path)
    exists = os.path.exists(model_path)
    print(f"Dataset: {dataset}")
    print(f"  Constructed Path: {model_path}")
    print(f"  Absolute Path:    {abs_path}")
    print(f"  Exists:           {exists}")
