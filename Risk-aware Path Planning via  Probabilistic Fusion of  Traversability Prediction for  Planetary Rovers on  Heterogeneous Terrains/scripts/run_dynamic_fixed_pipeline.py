
import os
import sys
import scipy.io
import numpy as np
import torch

# Add project root
BASE_PATH = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.join(BASE_PATH, '..')
sys.path.append(PROJECT_ROOT)

from scripts.generate_dynamic_fixed import create_dynamic_fixed_datasets
from scripts.train import fix_randomness, Runner as TrainRunner, TrainParams
from planning_project.models.unet import Unet
from planning_project.utils.structs import PlanMetrics, HyperParams

def run_pipeline():
    ds_name = "DynamicFixed"
    n_terrains = 10
    
    print(f"=== Starting Pipeline for {ds_name} ===")
    
    # --- Phase 1: Data Generation ---
    data_dir = os.path.join(PROJECT_ROOT, f'datasets/dataset_{ds_name}/')
    if (os.path.exists(data_dir) and 
        os.path.exists(os.path.join(data_dir, 'train')) and 
        len(os.listdir(os.path.join(data_dir, 'train'))) > 0):
        print("✓ Data already exists. Skipping generation.")
    else:
        print("Generating data...")
        create_dynamic_fixed_datasets()

    # --- Phase 2: Training ---
    nn_model_dir = os.path.join(PROJECT_ROOT, f'trained_models/models/dataset_{ds_name}/')
    log_dir = os.path.join(PROJECT_ROOT, f'trained_models/logs/dataset_{ds_name}/{ds_name}/')
    best_model_path = os.path.join(nn_model_dir, 'best_model.pth')
    
    if os.path.exists(best_model_path):
        print("✓ Model already trained. Skipping training.")
    else:
        print("Training model...")
        seed = 0
        fix_randomness(seed)
        
        os.makedirs(nn_model_dir, exist_ok=True)
        os.makedirs(log_dir, exist_ok=True)
        
        model = Unet(n_terrains=n_terrains, in_channels=3).set_model()
        train_params = TrainParams(data_dir, nn_model_dir, log_dir)
        
        # Override batch size if needed for GPU memory management
        # train_params.batch_size_train = 16 
        
        train_runner = TrainRunner(model, train_params, seed=seed)
        train_runner.run_trainval()
        print("✓ Training complete.")

    # --- Phase 3: Evaluation ---
    print("Evaluating model...")
    
    # We evaluate on the first 100 instances of test set
    idx_instances = list(range(100))
    
    plan_metrics = [
         PlanMetrics(is_plan=True, type_model="gsm", type_embed="mean", alpha=None),
         PlanMetrics(is_plan=True, type_model="gsm", type_embed="var", alpha=0.99),
    ]
    
    results_dir = os.path.join(PROJECT_ROOT, f'results/dataset_{ds_name}/')
    
    hyper_params = HyperParams(
        nn_model_dir=best_model_path,
        data_dir=data_dir,
        results_dir=results_dir,
        n_terrains=n_terrains,
        res=1,
        start_pos=(8, 8),
        goal_pos=(88, 88),
        plan_metrics=plan_metrics,
        n_ds=ds_name,
        idx_instances=idx_instances
    )
    
    # Use standard evaluation runner
    from scripts.eval import Runner as EvalRunner
    eval_runner = EvalRunner(hyper_params, is_run=True)
    abs_evals = eval_runner.absolute_evaluations()

    
    # --- Phase 4: Save Results for MATLAB ---
    print("Saving results for MATLAB...")
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
        
    results_struct = {ds_name: ds_results}
    
    mat_file_path = os.path.join(PROJECT_ROOT, 'results_dynamic_fixed.mat')
    scipy.io.savemat(mat_file_path, results_struct)
    
    print(f"✓ Results saved to {mat_file_path}")
    print("=== Pipeline Complete ===")

if __name__ == "__main__":
    run_pipeline()
