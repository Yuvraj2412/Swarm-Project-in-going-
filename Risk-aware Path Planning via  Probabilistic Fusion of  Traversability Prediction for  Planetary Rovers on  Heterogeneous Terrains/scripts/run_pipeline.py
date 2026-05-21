import os
import sys
import scipy.io
import numpy as np
import torch

# Add the project root to sys.path
BASE_PATH = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.join(BASE_PATH, '..')
sys.path.append(PROJECT_ROOT)

from scripts.create_data import create_datasets
from scripts.train import fix_randomness, Runner as TrainRunner, TrainParams
from planning_project.models.unet import Unet
from scripts.eval import Runner as EvalRunner, HyperParams, PlanMetrics

def run_pipeline():
    datasets = ['Std', 'ES', 'AA']
    
    # Phase 1: Data Generation
    print("--- Phase 1: Data Generation ---")
    for ds_name in datasets:
        data_dir = os.path.join(PROJECT_ROOT, f'datasets/dataset_{ds_name}/')
        if (os.path.exists(data_dir) and 
            os.listdir(os.path.join(data_dir, 'train')) and 
            os.listdir(os.path.join(data_dir, 'valid')) and 
            os.listdir(os.path.join(data_dir, 'test'))):
            print(f"Data for {ds_name} already exists. Skipping generation.")
        else:
            print(f"Creating data for {ds_name}...")
            create_datasets(n_ds=ds_name)

    # Phase 2: Training
    print("\n--- Phase 2: Training ---")
    for ds_name in datasets:
        # Correct path from previous fix
        nn_model_dir = os.path.join(PROJECT_ROOT, f'trained_models/models/dataset_{ds_name}/')
        log_dir = os.path.join(PROJECT_ROOT, f'trained_models/logs/dataset_{ds_name}/{ds_name}/')
        best_model_path = os.path.join(nn_model_dir, 'best_model.pth')
        
        if os.path.exists(best_model_path):
             print(f"Model for {ds_name} already exists. Skipping training.")
        else:
            print(f"Training model for {ds_name}...")
            seed = 0
            # fix_randomness(seed=seed) # This might reset global seed? Keep it if it was there.
            
            if ds_name == 'AA':
                n_terrains = 8
            else:
                n_terrains = 10
            
            # Ensure directories exist
            os.makedirs(nn_model_dir, exist_ok=True)
            os.makedirs(log_dir, exist_ok=True)

            model = Unet(n_terrains=n_terrains, in_channels=3).set_model()
            train_params = TrainParams(data_dir, nn_model_dir, log_dir)
            
            train_runner = TrainRunner(model, train_params, seed=seed)
            train_runner.run_trainval()

    # Phase 3: Evaluation
    print("\n--- Phase 3: Evaluation ---")
    results_for_matlab = {}
    for ds_name in datasets:
        print(f"Evaluating model for {ds_name}...")
        
        data_dir = os.path.join(PROJECT_ROOT, f'datasets/dataset_{ds_name}/')
        nn_model_dir = os.path.join(PROJECT_ROOT, f'trained_models/models/dataset_{ds_name}/')
        best_model_path = os.path.join(nn_model_dir, 'best_model.pth')
        results_dir = os.path.join(PROJECT_ROOT, f'results/dataset_{ds_name}/')
        
        if ds_name == 'AA':
            n_terrains = 8
        else:
            n_terrains = 10

        # Eval parameters
        res = 1
        margin = 8
        start_pos = (margin, margin)
        goal_pos = (96 - margin, 96 - margin)
        idx_instances = list(range(10)) # First 10 instances for testing
        
        plan_metrics = [
            PlanMetrics(is_plan=True, type_model="gsm", type_embed="mean", alpha=None),
            PlanMetrics(is_plan=True, type_model="gsm", type_embed="var", alpha=0.99),
        ]
        
        hyper_params = HyperParams(nn_model_dir=best_model_path,
                                    data_dir=data_dir,
                                    results_dir=results_dir,
                                    n_terrains=n_terrains,
                                    res=res,
                                    start_pos=start_pos,
                                    goal_pos=goal_pos,
                                    plan_metrics=plan_metrics,
                                    n_ds=ds_name,
                                    idx_instances=idx_instances)
                                    
        eval_runner = EvalRunner(hyper_params, is_run=True) # is_run=True forces re-run, might want logic to load if exists?
        # eval.py saves results to pickle. Helper absolute_evaluations() loads them if they exist in memory or computes?
        # Absolute evaluations use self.metrics_ds which is populated by run/load_experiments.
        # So it's fine.
        
        abs_evals = eval_runner.absolute_evaluations()
        
        # Store results
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
        
        results_for_matlab[ds_name] = ds_results

    # Save to .mat
    mat_file_path = os.path.join(PROJECT_ROOT, 'results.mat')
    print(f"Saving results to {mat_file_path}...")
    
    # Scipy.io.savemat handles dictionaries well. 
    # We might need to restructure if we want a specific struct array format in Matlab,
    # but a simple struct of structs/cells is usually a good start.
    scipy.io.savemat(mat_file_path, results_for_matlab)
    print("Done!")

if __name__ == '__main__':
    run_pipeline()
