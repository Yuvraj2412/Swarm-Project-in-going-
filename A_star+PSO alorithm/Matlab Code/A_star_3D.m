function path = A_star_3D(Map, start_node, goal_node)
    % A_star_3D - Finds a path in a 3D logical map using A*
    %
    % INPUTS:
    %   Map - 3D logical matrix (1=Obstacle, 0=Free).
    %   start_node - [x, y, z] start coordinates (voxels).
    %   goal_node - [x, y, z] goal coordinates (voxels).
    %
    % OUTPUT:
    %   path - Nx3 matrix of [x, y, z] coordinates from start to goal.
    %          Returns empty [] if no path is found.

    disp('Starting 3D A* pathfinding...');

    % 1. Initialization
    [X_max, Y_max, Z_max] = size(Map);

    % Check if start or goal is in an obstacle
    if Map(start_node(1), start_node(2), start_node(3))
        error('Start node is inside an obstacle!');
    end
    if Map(goal_node(1), goal_node(2), goal_node(3))
        error('Goal node is inside an obstacle!');
    end

    % Define 26 possible neighbors in 3D (connectivity)
    [dx, dy, dz] = meshgrid(-1:1, -1:1, -1:1);
    neighbors = [dx(:), dy(:), dz(:)];
    neighbors(14, :) = []; % Remove the (0,0,0) neighbor
    
    % G-cost: Cost from start to current node
    G_cost = inf(X_max, Y_max, Z_max);
    G_cost(start_node(1), start_node(2), start_node(3)) = 0;

    % F-cost (G + H): Estimated cost from start to goal via this node
    F_cost = inf(X_max, Y_max, Z_max);
    % Heuristic (H-cost): Euclidean distance to goal
    H_cost = @(n) sqrt((n(1) - goal_node(1))^2 + ...
                     (n(2) - goal_node(2))^2 + ...
                     (n(3) - goal_node(3))^2);
    F_cost(start_node(1), start_node(2), start_node(3)) = H_cost(start_node);

    % Parent: Stores the node from which we reached the current node
    Parent = zeros(X_max, Y_max, Z_max, 3);

    % Open List (Priority Queue)
    % Pre-allocate for speed. 1M nodes is a safe upper bound for this map.
    OpenList = zeros(X_max * Y_max, 5); % [F-cost, G-cost, X, Y, Z]
    OpenList(1, :) = [F_cost(start_node(1), start_node(2), start_node(3)), 0, start_node];
    openListCount = 1; % Counter for how many nodes are in the open list
    
    % Closed List (visited)
    ClosedList = false(X_max, Y_max, Z_max);

    path_found = false;

    % 2. A* Search Loop
    while openListCount > 0 % FIX: Loop while OpenList is not empty
        
        % Find node with lowest F-cost in Open List
        [~, list_idx] = min(OpenList(1:openListCount, 1));
        current_node = OpenList(list_idx, 3:5);
        current_g = OpenList(list_idx, 2);
        
        % Remove current node from Open List (fast swap-and-pop)
        OpenList(list_idx, :) = OpenList(openListCount, :); % Move last element to "popped" spot
        openListCount = openListCount - 1; % Decrease count
        
        % Add to Closed List
        if ClosedList(current_node(1), current_node(2), current_node(3))
            continue; % Already processed
        end
        ClosedList(current_node(1), current_node(2), current_node(3)) = true;
        
        % --- Goal Check ---
        if all(current_node == goal_node)
            path_found = true;
            break;
        end
        
        % 3. Check Neighbors
        for i = 1:size(neighbors, 1)
            neighbor_node = current_node + neighbors(i, :);
            nx = neighbor_node(1);
            ny = neighbor_node(2);
            nz = neighbor_node(3);
            
            % Check if neighbor is valid
            % (a) Inside map bounds
            if nx < 1 || nx > X_max || ny < 1 || ny > Y_max || nz < 1 || nz > Z_max
                continue;
            end
            % (b) Not an obstacle
            if Map(nx, ny, nz)
                continue;
            end
            % (c) Not on Closed List
            if ClosedList(nx, ny, nz)
                continue;
            end
            
            % Calculate cost to move to this neighbor
            move_cost = sqrt(sum(neighbors(i, :).^2)); % Euclidean distance
            tentative_g = current_g + move_cost;
            
            % Update costs if this is a better path
            if tentative_g < G_cost(nx, ny, nz)
                G_cost(nx, ny, nz) = tentative_g;
                F_cost(nx, ny, nz) = tentative_g + H_cost(neighbor_node);
                Parent(nx, ny, nz, :) = current_node;
                
                % Add to Open List (or update if already there)
                % For this simple pre-allocated list, we just add
                openListCount = openListCount + 1;
                OpenList(openListCount, :) = [F_cost(nx, ny, nz), tentative_g, neighbor_node];
            end
        end
    end % End while loop

    % 4. Reconstruct Path
    if ~path_found
        disp('No path found!');
        path = [];
        return;
    end
    
    disp('Path found! Reconstructing path...');
    
    % FIX: Pre-allocate path and build it backwards, then flip.
    % This avoids resizing the 'path' array on every iteration.
    % Max path length is unknown, but 50,000 steps is a very safe upper bound.
    path_temp = zeros(50000, 3); 
    path_temp(1, :) = goal_node;
    current = goal_node;
    pathCount = 1;
    
    while ~all(current == start_node)
        pathCount = pathCount + 1;
        parent_node = squeeze(Parent(current(1), current(2), current(3), :))';
        
        if isempty(parent_node) || all(parent_node == [0,0,0])
            error('Path reconstruction failed. Parent node not found.');
        end
        
        path_temp(pathCount, :) = parent_node;
        current = parent_node;
    end
    
    % Trim and flip the path to be start-to-goal
    path = flipud(path_temp(1:pathCount, :));
    
    disp('A* complete.');

end