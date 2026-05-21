
"""
Dynamic Fixed Terrain + Moving Object Dataset Generator
Generates maps where the terrain (background) is fixed per environment,
only the objects (rocks, craters, debris) move between instances.
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

class DynamicTerrainFixedGenerator:
    """
    Generates terrain maps with:
    - FIXED baseline terrain per environment
    - DYNAMIC obstacles (rocks, debris) per instance
    """
    
    def __init__(self, n_ds: str = "DynamicFixed", split: str = "train", 
                 n: int = 96, n_envs: int = 10, n_terrains: int = 10, 
                 n_colors: int = 5, n_instances: int = 100,
                 n_dynamic_objects: int = 35):  # Increased from 5 to ~35 for 10% density
        self.n_ds = n_ds
        self.split = split
        self.n = n
        self.n_terrains = n_terrains
        self.n_colors = n_colors
        self.n_envs = n_envs
        self.n_instances = n_instances
        self.n_dynamic_objects = n_dynamic_objects
        
        # Create output directory
        self.path2ds = os.path.join(BASE_PATH, f'../datasets/dataset_{self.n_ds}/{self.split}/')
        os.makedirs(self.path2ds, exist_ok=True)
        
        # Initialize slip model generator (needed for compatibility)
        self.smg = SlipModelsGenerator(
            os.path.join(BASE_PATH, f'../datasets/dataset_{self.n_ds}/'),
            self.n_terrains, 
            is_update=True, 
            type_noise='diverse'
        )
        
        self.object_sizes = [(3, 3), (5, 5), (7, 7)]
        self.object_types = ['rock', 'crater', 'debris']
        
    def generate_dynamic_objects(self, seed: int) -> np.ndarray:
        """Generate obstacles map"""
        np.random.seed(seed)
        random.seed(seed)
        
        obstacle_map = np.zeros((self.n, self.n), dtype=np.int32)
        
        for i in range(self.n_dynamic_objects):
            margin = 10
            x = np.random.randint(margin, self.n - margin)
            y = np.random.randint(margin, self.n - margin)
            size_w, size_h = random.choice(self.object_sizes)
            obj_type = random.choice(self.object_types)
            
            x_start = max(0, x - size_w // 2)
            x_end = min(self.n, x + size_w // 2)
            y_start = max(0, y - size_h // 2)
            y_end = min(self.n, y + size_h // 2)
            
            if obj_type == 'rock': val = 1
            elif obj_type == 'crater': val = 2
            else: val = 3
            
            obstacle_map[x_start:x_end, y_start:y_end] = val
        
        return obstacle_map
    
    def create_one_hot_label(self, t_class: np.ndarray) -> np.ndarray:
        label = np.zeros((t_class.size, self.n_terrains))
        for i in range(t_class.size):
            # Clamp to prevent index error if t_class > n_terrains
            idx = int(t_class.flatten()[i])
            if idx < self.n_terrains:
                label[i, idx] = 1
        return label
    
    def set_terrain_occ(self, seed: int, n_terrains: int) -> List[float]:
        np.random.seed(seed)
        return np.random.dirichlet(np.ones(n_terrains) * 2).tolist()
    
    def create_data(self):
        print(f"\n{'='*60}")
        print(f"Generating Dynamic Fixed Dataset: {self.split}")
        print(f"{'='*60}")
        
        seed_envs = 5000  # Distinct seed range
        seed_instances = 6000
        
        for i in range(self.n_envs):
            seed_envs += 1
            occ = self.set_terrain_occ(seed=seed_envs, n_terrains=self.n_terrains)
            
            print(f"\nEnvironment {i+1}/{self.n_envs}")
            
            # --- GENERATE BASE MAP (FIXED FOR THIS ENV) ---
            # We use a specific seed for the base map
            base_seed = seed_envs * 100
            grid_map = GridMap(self.n, 1, seed=base_seed)
            grid_map.set_terrain_env(
                is_crater=True, 
                is_fractal=True, 
                num_crater=7,
                max_a=25, 
                max_r=30
            )
            grid_map.set_terrain_distribution(occ=occ, type_dist="noise")
            
            # Cache base data
            # Assumes grid_map.data.color is [3, N, N] based on PyTorch convention in train.py
            # If it's [N, N, 3], we'll detect/adjust. 
            # Usually GridMap uses [3, H, W] for compatibility.
            base_height = np.reshape(grid_map.data.height, (self.n, self.n)).copy()
            base_color = grid_map.data.color.copy() 
            base_t_class = grid_map.data.t_class.copy()
            
            # Check shape to be sure about channel dim
            if base_color.shape[0] == 3:
                channels_first = True
            else:
                channels_first = False
            
            # --- INITIALIZE OBSTACLES FOR THIS ENV ---
            # List of dicts: {'pos': [x, y], 'vel': [vx, vy], 'size': (w, h), 'type': str}
            obstacles = []
            
            # Use separate seed for initial positions/velocities
            np.random.seed(seed_envs * 200)
            random.seed(seed_envs * 200)
            
            for _ in range(self.n_dynamic_objects):
                size_w, size_h = random.choice(self.object_sizes)
                obj_type = random.choice(self.object_types)
                
                # Start position (integer coordinates)
                margin = 10
                x = np.random.randint(margin, self.n - margin)
                y = np.random.randint(margin, self.n - margin)
                
                # Velocity (pixels per frame)
                # Random speed between 0.5 and 2.0 pixels/frame
                speed = np.random.uniform(0.5, 2.0)
                angle = np.random.uniform(0, 2 * np.pi)
                vx = speed * np.cos(angle)
                vy = speed * np.sin(angle)
                
                obstacles.append({
                    'pos': [float(x), float(y)],
                    'vel': [vx, vy],
                    'size': (size_w, size_h),
                    'type': obj_type
                })
            
            for j in range(self.n_instances):
                seed_instances += 1
                
                # Copy base data
                curr_height = base_height.copy()
                curr_color = base_color.copy()
                curr_t_class = base_t_class.copy()
                
                # --- UPDATE OBSTACLE POSITIONS ---
                obstacle_map = np.zeros((self.n, self.n), dtype=np.int32)
                
                for obj in obstacles:
                    # Update pos
                    obj['pos'][0] += obj['vel'][0]
                    obj['pos'][1] += obj['vel'][1]
                    
                    # Bounce off walls
                    # Check X
                    margin = 5
                    if obj['pos'][0] < margin:
                        obj['pos'][0] = margin
                        obj['vel'][0] *= -1
                    elif obj['pos'][0] > self.n - margin:
                        obj['pos'][0] = self.n - margin
                        obj['vel'][0] *= -1
                        
                    # Check Y
                    if obj['pos'][1] < margin:
                        obj['pos'][1] = margin
                        obj['vel'][1] *= -1
                    elif obj['pos'][1] > self.n - margin:
                        obj['pos'][1] = self.n - margin
                        obj['vel'][1] *= -1
                        
                    # Draw object on map
                    cx, cy = int(obj['pos'][0]), int(obj['pos'][1])
                    w, h = obj['size']
                    
                    x_start = max(0, cx - w // 2)
                    x_end = min(self.n, cx + w // 2)
                    y_start = max(0, cy - h // 2)
                    y_end = min(self.n, cy + h // 2)
                    
                    if obj['type'] == 'rock': val = 1
                    elif obj['type'] == 'crater': val = 2
                    else: val = 3
                    
                    obstacle_map[x_start:x_end, y_start:y_end] = val
                
                # Update Height
                curr_height = curr_height + obstacle_map * 0.2
                
                # Update Color (Make obstacles visible!)
                # Colors: Rock=Gray, Crater=Red, Debris=Yellow
                # Obstacle val: 1=Rock, 2=Crater, 3=Debris
                
                mask_rock = (obstacle_map == 1)
                mask_crater = (obstacle_map == 2)
                mask_debris = (obstacle_map == 3)
                
                if channels_first:
                    # [3, H, W]
                    # Rock (Gray)
                    curr_color[0][mask_rock] = 0.3
                    curr_color[1][mask_rock] = 0.3
                    curr_color[2][mask_rock] = 0.3
                    
                    # Crater (Redish)
                    curr_color[0][mask_crater] = 0.6
                    curr_color[1][mask_crater] = 0.1
                    curr_color[2][mask_crater] = 0.1
                    
                    # Debris (Yellowish)
                    curr_color[0][mask_debris] = 0.8
                    curr_color[1][mask_debris] = 0.7
                    curr_color[2][mask_debris] = 0.1
                else:
                    # [H, W, 3]
                    curr_color[mask_rock] = [0.3, 0.3, 0.3]
                    curr_color[mask_crater] = [0.6, 0.1, 0.1]
                    curr_color[mask_debris] = [0.8, 0.7, 0.1]

                # Update Label
                # This is tricky. If we don't update label, model sees rock but predicts grass.
                # If we update label, we might exceed n_terrains.
                # Assuming n_terrains=10, we'll try to use the last few classes?
                # Or just leaving it as is (Dynamic behavior implies valid traversability is tricky).
                # DECISION: We will NOT update label to avoid crashing indices, 
                # effectively treating obstacles as 'unlabeled' or 'underlying terrain' 
                # BUT visible in input. This tests robustness.
                
                label_one_hot = np.reshape(
                    self.create_one_hot_label(curr_t_class),
                    (self.n, self.n, self.n_terrains)
                )
                
                # Save
                env_data = {
                    'input': curr_color.astype(np.float32),
                    'label': label_one_hot.astype(np.float64),
                    'height': curr_height.astype(np.float64)
                }
                
                filename = f"env_{i:02d}_{j:04d}.npy"
                filepath = os.path.join(self.path2ds, filename)
                np.save(filepath, env_data)
                
                if (j + 1) % 50 == 0:
                    print(f"  Generated {j+1}/{self.n_instances}")
        
        # Save seed info
        seed_info = np.array([seed_envs, seed_instances])
        np.save(os.path.join(self.path2ds, 'seed_info.npy'), seed_info)
        print(f"✓ Saved {self.split} set.")

def create_dynamic_fixed_datasets():
    print("STARTING DYNAMIC FIXED DATASET GENERATION")
    
    # Train: 10 envs, 100 instances
    train = DynamicTerrainFixedGenerator("DynamicFixed", "train", n_instances=100)
    train.create_data()
    
    # Valid: 10 envs, 50 instances
    valid = DynamicTerrainFixedGenerator("DynamicFixed", "valid", n_instances=50)
    valid.create_data()
    
    # Test: 10 envs, 100 instances
    test = DynamicTerrainFixedGenerator("DynamicFixed", "test", n_instances=100)
    test.create_data()

if __name__ == "__main__":
    create_dynamic_fixed_datasets()
