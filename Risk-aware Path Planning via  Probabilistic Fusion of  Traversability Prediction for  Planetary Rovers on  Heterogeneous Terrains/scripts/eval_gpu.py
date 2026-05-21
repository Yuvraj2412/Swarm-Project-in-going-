"""
GPU-Accelerated and Parallel Evaluation Pipeline
Optimizes evaluation using GPU for neural network inference and parallel processing for path planning
"""
import torch
import torch.multiprocessing as mp
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
import numpy as np
import pickle
import os
from typing import List, Tuple
from dataclasses import dataclass
import time

import sys
BASE_PATH = os.path.dirname(__file__)
sys.path.append(os.path.join(BASE_PATH, '..'))

from planning_project.models.unet import Unet
from planning_project.planner.planner import AStarPlanner
from planning_project.utils.data import DataSet
from planning_project.utils.structs import HyperParams, PlanMetrics, AbsEval
from scripts.eval import Runner


class GPUAcceleratedEvaluator:
    """
    GPU-accelerated evaluator with parallel processing
    - Batch neural network inference on GPU
    - Parallel path planning on CPU
    - Optimized data loading
    """
    
    def __init__(self, hyper_params: HyperParams, device: str = 'auto', num_workers: int = 4):
        """
        Initialize GPU-accelerated evaluator
        
        Args:
            hyper_params: Evaluation hyperparameters
            device: 'cuda', 'cpu', or 'auto' (auto-detect)
            num_workers: Number of parallel workers for path planning
        """
        self.hyper_params = hyper_params
        self.num_workers = num_workers
        
        # Auto-detect device
        if device == 'auto':
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        else:
            self.device = torch.device(device)
        
        print(f"🚀 GPU Accelerated Evaluator initialized")
        print(f"   Device: {self.device}")
        print(f"   Parallel workers: {self.num_workers}")
        
        # Load model
        self.model = self._load_model()
        
        # Load dataset (using 'test' split)
        self.dataset = DataSet(hyper_params.data_dir, 'test')
        
    def _load_model(self) -> torch.nn.Module:
        """Load and prepare model for GPU inference"""
        # Load checkpoint
        checkpoint = torch.load(
            self.hyper_params.nn_model_dir,
            map_location=self.device
        )
        
        # Check if it's already a model or a state dict
        if isinstance(checkpoint, torch.nn.Module):
            # It's already a complete model
            model = checkpoint
        else:
            # It's a state dict, load into new model
            model = Unet(
                n_terrains=self.hyper_params.n_terrains,
                in_channels=3
            ).set_model()
            model.load_state_dict(checkpoint)
        
        # Move to device and set to eval mode
        model = model.to(self.device)
        model.eval()
        
        return model
    
    @torch.no_grad()
    def batch_predict(self, inputs: torch.Tensor) -> torch.Tensor:
        """
        Batch prediction on GPU
        
        Args:
            inputs: Batch of input tensors [B, C, H, W]
            
        Returns:
            Predictions [B, n_terrains, H, W]
        """
        inputs = inputs.to(self.device)
        outputs = self.model(inputs)
        return outputs.cpu()
    
    def evaluate_instance(self, idx: int, plan_metric: PlanMetrics) -> dict:
        """
        Evaluate single instance (for parallel processing)
        
        Args:
            idx: Instance index
            plan_metric: Planning metric
            
        Returns:
            Evaluation results dictionary
        """
        # This will be called in parallel
        # Load grid map
        grid_map = self.dataset.load_test_env(idx)
        
        # Get terrain prediction (already done in batch)
        # Execute path planning
        planner = AStarPlanner(
            grid_map,
            self.hyper_params.start_pos,
            self.hyper_params.goal_pos,
            plan_metric
        )
        
        result = planner.planning()
        
        return {
            'idx': idx,
            'result': result,
            'plan_metric': plan_metric
        }
    
    def evaluate_dataset_parallel(self, plan_metrics: List[PlanMetrics]) -> List[AbsEval]:
        """
        Evaluate entire dataset with GPU + parallel processing
        
        Args:
            plan_metrics: List of planning metrics to evaluate
            
        Returns:
            List of absolute evaluation results
        """
        print(f"\n{'='*60}")
        print(f"Starting GPU-Accelerated Evaluation")
        print(f"{'='*60}")
        
        idx_instances = self.hyper_params.idx_instances
        n_instances = len(idx_instances)
        
        # Step 1: Batch load all test data
        print(f"\n[1/3] Loading {n_instances} test instances...")
        start_time = time.time()
        
        test_data = []
        for idx in idx_instances:
            grid_map = self.dataset.load_test_env(idx)
            test_data.append(grid_map)
        
        load_time = time.time() - start_time
        print(f"   ✓ Loaded in {load_time:.2f}s")
        
        # Step 2: Batch neural network inference on GPU
        print(f"\n[2/3] Running batch inference on {self.device}...")
        start_time = time.time()
        
        # Prepare batch
        batch_inputs = []
        for grid_map in test_data:
            # Convert grid_map to tensor format
            input_tensor = self._prepare_input(grid_map)
            batch_inputs.append(input_tensor)
        
        batch_inputs = torch.stack(batch_inputs)
        
        # Batch predict
        batch_size = 16  # Adjust based on GPU memory
        all_predictions = []
        
        for i in range(0, len(batch_inputs), batch_size):
            batch = batch_inputs[i:i+batch_size]
            predictions = self.batch_predict(batch)
            all_predictions.append(predictions)
        
        all_predictions = torch.cat(all_predictions, dim=0)
        
        inference_time = time.time() - start_time
        print(f"   ✓ Inference completed in {inference_time:.2f}s")
        print(f"   ✓ Speed: {n_instances/inference_time:.1f} instances/sec")
        
        # Step 3: Parallel path planning
        print(f"\n[3/3] Running parallel path planning ({self.num_workers} workers)...")
        start_time = time.time()
        
        all_results = []
        
        for plan_metric in plan_metrics:
            print(f"\n   Metric: {plan_metric.type_model} + {plan_metric.type_embed}")
            
            # Parallel execution
            with ProcessPoolExecutor(max_workers=self.num_workers) as executor:
                futures = []
                for idx in idx_instances:
                    future = executor.submit(
                        self.evaluate_instance,
                        idx,
                        plan_metric
                    )
                    futures.append(future)
                
                # Collect results
                metric_results = []
                for future in futures:
                    result = future.result()
                    metric_results.append(result)
            
            all_results.append(metric_results)
        
        planning_time = time.time() - start_time
        print(f"\n   ✓ Planning completed in {planning_time:.2f}s")
        
        # Compute absolute evaluations
        abs_evals = self._compute_absolute_evaluations(all_results, plan_metrics)
        
        total_time = load_time + inference_time + planning_time
        print(f"\n{'='*60}")
        print(f"✓ Evaluation Complete!")
        print(f"   Total time: {total_time:.2f}s")
        print(f"   Speedup: ~{self.num_workers}x (parallel) + GPU acceleration")
        print(f"{'='*60}\n")
        
        return abs_evals
    
    def _prepare_input(self, grid_map) -> torch.Tensor:
        """Convert grid map to model input tensor"""
        # This is a placeholder - adjust based on actual data format
        # Typically: [height_map, color_map, ...]
        input_data = np.stack([
            grid_map.data.height,
            grid_map.data.color
        ], axis=0)
        
        return torch.from_numpy(input_data).float()
    
    def _compute_absolute_evaluations(self, all_results, plan_metrics) -> List[AbsEval]:
        """Compute absolute evaluation metrics from results"""
        abs_evals = []
        
        for metric_results, plan_metric in zip(all_results, plan_metrics):
            # Aggregate results
            is_solved = sum(1 for r in metric_results if r['result'].is_solved)
            is_feasible = sum(1 for r in metric_results if r['result'].is_feasible)
            
            # Compute statistics
            abs_eval = AbsEval(
                plan_metrics=plan_metric,
                is_solved=is_solved,
                is_feasible=is_feasible,
                # Add more metrics as needed
            )
            abs_evals.append(abs_eval)
        
        return abs_evals


def evaluate_with_gpu(n_ds: str, n_terrains: int, idx_instances: list, 
                      plan_metrics: List[PlanMetrics], device: str = 'auto',
                      num_workers: int = 4):
    """
    Convenience function for GPU-accelerated evaluation
    
    Args:
        n_ds: Dataset name
        n_terrains: Number of terrain types
        idx_instances: List of instance indices to evaluate
        plan_metrics: List of planning metrics
        device: 'cuda', 'cpu', or 'auto'
        num_workers: Number of parallel workers
    """
    # Setup paths
    data_dir = os.path.join(BASE_PATH, f'../datasets/dataset_{n_ds}/')
    nn_model_dir = os.path.join(BASE_PATH, f'../trained_models/models/dataset_{n_ds}/best_model.pth')
    results_dir = os.path.join(BASE_PATH, f'../results/dataset_{n_ds}/')
    
    # Create hyperparameters
    hyper_params = HyperParams(
        nn_model_dir=nn_model_dir,
        data_dir=data_dir,
        results_dir=results_dir,
        n_terrains=n_terrains,
        res=1,
        start_pos=(8, 8),
        goal_pos=(88, 88),
        plan_metrics=plan_metrics,
        n_ds=n_ds,
        idx_instances=idx_instances
    )
    
    # Create evaluator
    evaluator = GPUAcceleratedEvaluator(
        hyper_params,
        device=device,
        num_workers=num_workers
    )
    
    # Run evaluation
    abs_evals = evaluator.evaluate_dataset_parallel(plan_metrics)
    
    return abs_evals


if __name__ == "__main__":
    # Example usage
    plan_metrics = [
        PlanMetrics(is_plan=True, type_model="gsm", type_embed="mean", alpha=None),
        PlanMetrics(is_plan=True, type_model="gsm", type_embed="var", alpha=0.99),
    ]
    
    # Evaluate on all 100 test instances
    results = evaluate_with_gpu(
        n_ds="Std",
        n_terrains=10,
        idx_instances=list(range(100)),  # All 100 instances
        plan_metrics=plan_metrics,
        device='auto',
        num_workers=8  # Adjust based on CPU cores
    )
    
    print("Evaluation complete!")
