"""
Optimized Pipeline Runner with GPU Acceleration and Parallel Processing
- Downloads official datasets
- Generates dynamic terrain dataset
- Trains models with GPU
- Evaluates with GPU + parallel processing on all 100 test instances
- SUPPORTS: Parallel evaluation of multiple datasets
"""
import os
import sys
import scipy.io
import numpy as np
import torch
import multiprocessing
from multiprocessing import cpu_count

# Add the project root to sys.path
BASE_PATH = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.join(BASE_PATH, '..')
sys.path.append(PROJECT_ROOT)

from scripts.create_data import create_datasets
from scripts.train import fix_randomness, Runner as TrainRunner, TrainParams
from planning_project.models.unet import Unet
from scripts.eval import Runner as EvalRunner, HyperParams, PlanMetrics

# Import GPU Evaluator (needs project root in path)
try:
    from scripts.eval_gpu import GPUAcceleratedEvaluator
    GPU_EVAL_AVAILABLE = True
except ImportError:
    GPU_EVAL_AVAILABLE = False
    print("⚠ GPU Evaluator not found, using CPU fallback")

def evaluate_single_dataset_task(ds_name, device, num_workers, queue):
    """
    Worker function to evaluate a single dataset in a separate process
    """
    try:
        print(f"[{ds_name}] Starting evaluation on {device} with {num_workers} workers...")
        
        # Setup paths
        data_dir = os.path.join(PROJECT_ROOT, f'datasets/dataset_{ds_name}/')
        nn_model_dir = os.path.join(PROJECT_ROOT, f'trained_models/models/dataset_{ds_name}/')
        best_model_path = os.path.join(nn_model_dir, 'best_model.pth')
        results_dir = os.path.join(PROJECT_ROOT, f'results/dataset_{ds_name}/')
        
        if ds_name == 'AA':
            n_terrains = 8
        else:
            n_terrains = 10
        
        # Evaluation parameters
        res = 1
        margin = 8
        start_pos = (margin, margin)
        goal_pos = (96 - margin, 96 - margin)
        idx_instances = list(range(100))  # Evaluate on ALL 100 test instances
        
        plan_metrics = [
            PlanMetrics(is_plan=True, type_model="gsm", type_embed="mean", alpha=None),
            PlanMetrics(is_plan=True, type_model="gsm", type_embed="var", alpha=0.99),
        ]
        
        hyper_params = HyperParams(
            nn_model_dir=best_model_path,
            data_dir=data_dir,
            results_dir=results_dir,
            n_terrains=n_terrains,
            res=res,
            start_pos=start_pos,
            goal_pos=goal_pos,
            plan_metrics=plan_metrics,
            n_ds=ds_name,
            idx_instances=idx_instances
        )
        
        # Run evaluation
        if GPU_EVAL_AVAILABLE and device != 'cpu':
            try:
                evaluator = GPUAcceleratedEvaluator(
                    hyper_params,
                    device=device,
                    num_workers=num_workers
                )
                abs_evals = evaluator.evaluate_dataset_parallel(plan_metrics)
            except Exception as e:
                print(f"[{ds_name}] ⚠ GPU evaluation failed: {e}")
                print(f"[{ds_name}] Fallback to CPU...")
                eval_runner = EvalRunner(hyper_params, is_run=True)
                abs_evals = eval_runner.absolute_evaluations()
        else:
            print(f"[{ds_name}] Using standard CPU evaluation...")
            eval_runner = EvalRunner(hyper_params, is_run=True)
            abs_evals = eval_runner.absolute_evaluations()
            
        # Convert results to dictionary for serialization
        ds_results = []
        for abs_eval in abs_evals:
            eval_data = {
                'type_model': abs_eval.plan_metrics.type_model,
                'type_embed': abs_eval.plan_metrics.type_embed,
                'is_solved': abs_eval.is_solved,
                'is_feasible': abs_eval.is_feasible,
                'dist_mean': abs_eval.dist[0],
                'dist_var': abs_eval.dist[1],
                'est_cost_mean': abs_eval.est_cost[0],
                'est_cost_var': abs_eval.est_cost[1],
                'obs_time_mean': abs_eval.obs_time[0],
                'obs_time_var': abs_eval.obs_time[1],
                'max_slip_mean': abs_eval.max_slip[0],
                'max_slip_var': abs_eval.max_slip[1]
            }
            ds_results.append(eval_data)
        
        print(f"[{ds_name}] ✓ Evaluation complete")
        queue.put((ds_name, ds_results))
        
    except Exception as e:
        print(f"[{ds_name}] ✗ CRITICAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        queue.put((ds_name, []))

def run_optimized_pipeline(use_gpu: bool = True, num_workers: int = None, include_dynamic: bool = True):
    """
    Run optimized pipeline with GPU acceleration and parallel processing
    """
    if num_workers is None:
        num_workers = max(1, cpu_count() - 2)
    
    device = 'cuda' if (use_gpu and torch.cuda.is_available()) else 'cpu'
    
    print("="*70)
    print("OPTIMIZED ROVER NAVIGATION PIPELINE")
    print("="*70)
    print(f"Device: {device.upper()}")
    print(f"Total Parallel Workers: {num_workers}")
    print(f"Include Dynamic Terrain: {include_dynamic}")
    print("="*70)
    
    datasets = ['Std', 'ES', 'AA']
    if include_dynamic:
        datasets.append('Dynamic')
    
    # Phase 1: Data Verification
    print("\n" + "="*70)
    print("PHASE 1: DATA VERIFICATION")
    print("="*70)
    
    for ds_name in datasets:
        data_dir = os.path.join(PROJECT_ROOT, f'datasets/dataset_{ds_name}/')
        test_dir = os.path.join(data_dir, 'test')
        
        if os.path.exists(test_dir):
            test_files = len([f for f in os.listdir(test_dir) if f.endswith('.npy') and f.startswith('env')])
            print(f"✓ {ds_name}: {test_files} test files found")
        else:
            print(f"✗ {ds_name}: Test data not found")
    
    # Phase 1.5: Generate Dynamic Dataset
    if include_dynamic:
        dynamic_data_dir = os.path.join(PROJECT_ROOT, 'datasets/dataset_Dynamic/')
        if not os.path.exists(os.path.join(dynamic_data_dir, 'test')):
            print("\n" + "="*70)
            print("PHASE 1.5: DYNAMIC DATASET GENERATION")
            print("="*70)
            try:
                from scripts.generate_dynamic import create_dynamic_datasets
                print("Generating dynamic terrain dataset...")
                create_dynamic_datasets()
                print("✓ Dynamic dataset generation complete")
            except Exception as e:
                print(f"✗ Dynamic dataset generation failed: {e}")
                datasets.remove('Dynamic')
        else:
            print(f"\n✓ Dynamic dataset already exists")
    
    # Phase 2: Training (Sequential to avoid OOM)
    print("\n" + "="*70)
    print("PHASE 2: MODEL TRAINING")
    print("="*70)
    
    for ds_name in datasets:
        nn_model_dir = os.path.join(PROJECT_ROOT, f'trained_models/models/dataset_{ds_name}/')
        best_model_path = os.path.join(nn_model_dir, 'best_model.pth')
        
        if os.path.exists(best_model_path):
            print(f"✓ {ds_name}: Model exists")
        else:
            print(f"⚙ {ds_name}: Training model...")
            # ... (Training code remains same) ...
            log_dir = os.path.join(PROJECT_ROOT, f'trained_models/logs/dataset_{ds_name}/{ds_name}/')
            data_dir = os.path.join(PROJECT_ROOT, f'datasets/dataset_{ds_name}/')
            seed = 0
            n_terrains = 8 if ds_name == 'AA' else 10
            
            os.makedirs(nn_model_dir, exist_ok=True)
            os.makedirs(log_dir, exist_ok=True)
            
            model = Unet(n_terrains=n_terrains, in_channels=3).set_model()
            if device == 'cuda': model = model.cuda()
            
            train_params = TrainParams(data_dir, nn_model_dir, log_dir)
            train_runner = TrainRunner(model, train_params, seed=seed)
            train_runner.run_trainval()
            print(f"✓ {ds_name}: Training complete")
    
    # Phase 3: Parallel Evaluation
    print("\n" + "="*70)
    print("PHASE 3: PARALLEL EVALUATION (ALL DATASETS)")
    print("="*70)
    
    # Determine workers per dataset
    workers_per_ds = max(1, num_workers // len(datasets))
    print(f"🚀 Launching {len(datasets)} parallel processes")
    print(f"   Workers per dataset: {workers_per_ds}")
    print(f"   Device: {device}")
    
    queue = multiprocessing.Queue()
    processes = []
    
    for ds_name in datasets:
        p = multiprocessing.Process(
            target=evaluate_single_dataset_task,
            args=(ds_name, device, workers_per_ds, queue)
        )
        p.start()
        processes.append(p)
        print(f"   - Started process for {ds_name} (PID: {p.pid})")
    
    # Wait for completion
    for p in processes:
        p.join()
    
    # Collect results
    results_for_matlab = {}
    while not queue.empty():
        ds_name, res = queue.get()
        results_for_matlab[ds_name] = res
        
        # Print summary for this dataset
        print(f"\n{ds_name} Results Summary:")
        for eval_data in res:
            success_rate = (eval_data['is_feasible'] / eval_data['is_solved'] * 100) if eval_data['is_solved'] > 0 else 0
            print(f"  {eval_data['type_model']}+{eval_data['type_embed']}: {success_rate:.1f}% success")
            
    # Save results
    print("\n" + "="*70)
    mat_file_path = os.path.join(PROJECT_ROOT, 'results.mat')
    scipy.io.savemat(mat_file_path, results_for_matlab)
    print(f"✓ Results saved to: {mat_file_path}")

if __name__ == '__main__':
    # Initialize multiprocessing support for Windows
    multiprocessing.freeze_support()
    run_optimized_pipeline(use_gpu=True, num_workers=8, include_dynamic=True)
