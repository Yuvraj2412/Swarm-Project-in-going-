"""
Dynamic Terrain + Object Dataset Generator
Generates terrain with dynamic obstacles and moving objects for rover navigation
"""
import numpy as np
import sys, os
import random
from typing import List, Tuple

BASE_PATH = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.join(BASE_PATH, '..')
sys.path.append(PROJECT_ROOT)

from planning_project.env.env import GridMap
from planning_project.env.slip_models import SlipModelsGenerator

class DynamicTerrainGenerator:
    """
    Generates terrain maps with dynamic elements:
    - Moving obstacles (rocks, debris)
    - Varying terrain difficulty zones
    - Temporal changes in traversability
    """
    
    def __init__(self, n_ds: str = "Dynamic", split: str = "train", 
                 n: int = 96, n_envs: int = 10, n_terrains: int = 10, 
                 n_colors: int = 5, n_instances: int = 100,
                 n_dynamic_objects: int = 5, is_show: bool = False):
        """
        Initialize Dynamic Terrain Generator
        
        Args:
            n_ds: Dataset name
            split: Data split (train/valid/test)
            n: Grid size (n x n)
            n_envs: Number of environment types
            n_terrains: Number of terrain features
            n_colors: Number of terrain colors
            n_instances: Number of instances per environment
            n_dynamic_objects: Number of dynamic obstacles per map
            is_show: Visualize generated maps
        """
        self.n_ds = n_ds
        self.split = split
        self.n = n
        self.n_terrains = n_terrains
        self.n_colors = n_colors
        self.n_envs = n_envs
        self.n_instances = n_instances
        self.n_dynamic_objects = n_dynamic_objects
        self.is_show = is_show
        
        # Create output directory
        self.path2ds = os.path.join(BASE_PATH, f'../datasets/dataset_{self.n_ds}/{self.split}/')
        os.makedirs(self.path2ds, exist_ok=True)
        
        # Initialize slip model generator
        self.smg = SlipModelsGenerator(
            os.path.join(BASE_PATH, f'../datasets/dataset_{self.n_ds}/'),
            self.n_terrains, 
            is_update=True, 
            type_noise='diverse'
        )
        
        # Dynamic object parameters
        self.object_sizes = [(3, 3), (5, 5), (7, 7)]  # Small, medium, large obstacles
        self.object_types = ['rock', 'crater', 'debris']
        
    def generate_dynamic_objects(self, grid_map: GridMap, seed: int = None) -> np.ndarray:
        """
        Add dynamic obstacles to the terrain
        
        Args:
            grid_map: Base grid map
            seed: Random seed for reproducibility
            
        Returns:
            Modified terrain with dynamic objects
        """
        if seed is not None:
            np.random.seed(seed)
            random.seed(seed)
        
        obstacle_map = np.zeros((self.n, self.n), dtype=np.int32)
        
        for i in range(self.n_dynamic_objects):
            # Random position (avoid edges)
            margin = 10
            x = np.random.randint(margin, self.n - margin)
            y = np.random.randint(margin, self.n - margin)
            
            # Random size
            size_w, size_h = random.choice(self.object_sizes)
            
            # Random object type
            obj_type = random.choice(self.object_types)
            
            # Place object
            x_start = max(0, x - size_w // 2)
            x_end = min(self.n, x + size_w // 2)
            y_start = max(0, y - size_h // 2)
            y_end = min(self.n, y + size_h // 2)
            
            if obj_type == 'rock':
                # High slip, difficult terrain
                obstacle_map[x_start:x_end, y_start:y_end] = 1
            elif obj_type == 'crater':
                # Very high slip, avoid
                obstacle_map[x_start:x_end, y_start:y_end] = 2
            else:  # debris
                # Moderate slip
                obstacle_map[x_start:x_end, y_start:y_end] = 3
        
        return obstacle_map
    
    def create_one_hot_label(self, t_class: np.ndarray) -> np.ndarray:
        """Convert terrain class to one-hot encoding"""
        label = np.zeros((t_class.size, self.n_terrains))
        for i in range(t_class.size):
            label[i, int(t_class.flatten()[i])] = 1
        return label
    
    def set_terrain_occ(self, seed: int = None, n_terrains: int = 10) -> List[float]:
        """Generate terrain occupancy vector"""
        if seed is not None:
            np.random.seed(seed)
        
        # Generate random occupancy with some terrains more prevalent
        occ = np.random.dirichlet(np.ones(n_terrains) * 2)
        return occ.tolist()
    
    def create_data(self):
        """Generate dynamic terrain dataset"""
        print(f"\n{'='*60}")
        print(f"Generating Dynamic Terrain Dataset: {self.split}")
        print(f"{'='*60}")
        
        seed_envs = 1000  # Start from different seed than static datasets
        seed_instances = 2000
        
        for i in range(self.n_envs):
            seed_envs += 1
            occ = self.set_terrain_occ(seed=seed_envs, n_terrains=self.n_terrains)
            
            print(f"\nEnvironment {i+1}/{self.n_envs}")
            print(f"  Occupancy vector: {[f'{x:.3f}' for x in occ]}")
            
            for j in range(self.n_instances):
                seed_instances += 1
                
                # Create base terrain
                grid_map = GridMap(self.n, 1, seed=seed_instances)
                grid_map.set_terrain_env(
                    is_crater=True, 
                    is_fractal=True, 
                    num_crater=7,  # More craters for dynamic environment
                    max_a=25, 
                    max_r=30
                )
                
                # Set terrain distribution
                grid_map.set_terrain_distribution(occ=occ, type_dist="noise")
                
                # Add dynamic objects
                obstacle_map = self.generate_dynamic_objects(grid_map, seed=seed_instances)
                
                # Modify terrain based on obstacles
                height_map = np.reshape(grid_map.data.height, (grid_map.n, grid_map.n))
                height_map = height_map + obstacle_map * 0.2  # Increase height for obstacles
                
                # Create labels
                color_map = grid_map.data.color
                label_one_hot = np.reshape(
                    self.create_one_hot_label(grid_map.data.t_class),
                    (grid_map.n, grid_map.n, self.n_terrains)
                )
                
                # Stack and save
                # env_data = np.dstack([height_map, color_map, label_one_hot])
                
                # Save as dictionary to match official format
                env_data = {
                    'input': color_map.astype(np.float32),
                    'label': label_one_hot.astype(np.float64),
                    'height': height_map.astype(np.float64)
                }
                
                filename = f"env_{i:02d}_{j:04d}.npy"
                filepath = os.path.join(self.path2ds, filename)
                np.save(filepath, env_data)
                
                if (j + 1) % 10 == 0:
                    print(f"  Generated {j+1}/{self.n_instances} instances")
        
        # Save seed info
        seed_info = np.array([seed_envs, seed_instances])
        np.save(os.path.join(self.path2ds, 'seed_info.npy'), seed_info)
        
        print(f"\n✓ Dynamic terrain dataset generation complete!")
        print(f"  Total files: {self.n_envs * self.n_instances}")
        print(f"  Location: {self.path2ds}")


def create_dynamic_datasets():
    """Create all splits for dynamic terrain dataset"""
    print("\n" + "="*60)
    print("DYNAMIC TERRAIN + OBJECT DATASET GENERATION")
    print("="*60)
    
    # Training set: 10 envs × 100 instances = 1000 files
    print("\n[1/3] Generating training data...")
    train_gen = DynamicTerrainGenerator(
        n_ds="Dynamic",
        split="train",
        n=96,
        n_envs=10,
        n_terrains=10,
        n_colors=5,
        n_instances=100,
        n_dynamic_objects=5
    )
    train_gen.create_data()
    
    # Validation set: 10 envs × 50 instances = 500 files
    print("\n[2/3] Generating validation data...")
    valid_gen = DynamicTerrainGenerator(
        n_ds="Dynamic",
        split="valid",
        n=96,
        n_envs=10,
        n_terrains=10,
        n_colors=5,
        n_instances=50,
        n_dynamic_objects=5
    )
    valid_gen.create_data()
    
    # Test set: 10 envs × 100 instances = 1000 files
    print("\n[3/3] Generating test data...")
    test_gen = DynamicTerrainGenerator(
        n_ds="Dynamic",
        split="test",
        n=96,
        n_envs=10,
        n_terrains=10,
        n_colors=5,
        n_instances=100,
        n_dynamic_objects=5
    )
    test_gen.create_data()
    
    print("\n" + "="*60)
    print("✓ ALL DYNAMIC DATASETS GENERATED SUCCESSFULLY!")
    print("="*60)


if __name__ == "__main__":
    create_dynamic_datasets()
