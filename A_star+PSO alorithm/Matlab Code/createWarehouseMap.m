function [Map, goal_spots] = createWarehouseMap(X_dim, Y_dim, Z_dim, res)
    % createWarehouseMap - Generates a 3D voxel map of the warehouse
    %
    % INPUTS:
    %   X_dim, Y_dim, Z_dim - Dimensions of the warehouse in meters
    %   res - Resolution in meters (e.g., 0.1 for 10cm voxels)
    %
    % OUTPUTS:
    %   Map - A 3D logical matrix (X,Y,Z). 1=Obstacle, 0=Free
    %   goal_spots - A struct containing landing spots coordinates

    disp('Creating 3D warehouse map...');

    % 1. Initialize Map
    % Calculate map dimensions in voxels
    X_vox = round(X_dim / res);
    Y_vox = round(Y_dim / res);
    Z_vox = round(Z_dim / res);

    % Create an empty map (all free space)
    Map = uint8(zeros(X_vox, Y_vox, Z_vox)); % FIX: Use uint8 for multiple types

    % Helper function to convert meters to voxel indices
    m2v = @(meters) max(1, round(meters / res));

    % 2. Add Boundaries (Walls, Floor, Ceiling)
    Map(1, :, :) = 1; % Wall
    Map(X_vox, :, :) = 1; % Wall
    Map(:, 1, :) = 1; % Wall
    Map(:, Y_vox, :) = 1; % Wall
    Map(:, :, 1) = 1; % Floor
    Map(:, :, Z_vox) = 1; % Ceiling

    % 3. Add Tables (Goals)
    % Table height
    table_h_m = 1.0; % 1 meter high
    table_h_vox = m2v(table_h_m);

    % Landing spots will be 1 voxel above the table
    landing_z_vox = table_h_vox + 1;

    % --- Table 1 (1 Drone Max) ---
    % Position: X=[5, 7]m, Y=[10, 12]m
    x_range = m2v(5):m2v(7);
    y_range = m2v(10):m2v(12);
    z_range = 1:table_h_vox;
    Map(x_range, y_range, z_range) = 2; % FIX: Type 2 for Table
    % Define its single landing spot (center of table)
    goal_spots.table1 = [m2v(6), m2v(11), landing_z_vox];

    % --- Table 2 (1 Drone Max) ---
    % Position: X=[5, 7]m, Y=[15, 17]m
    x_range = m2v(5):m2v(7);
    y_range = m2v(15):m2v(17);
    z_range = 1:table_h_vox;
    Map(x_range, y_range, z_range) = 2; % FIX: Type 2 for Table
    goal_spots.table2 = [m2v(6), m2v(16), landing_z_vox];

    % --- Table 3 (2 Drones Max) ---
    % Position: X=[10, 13]m, Y=[10, 12]m
    x_range = m2v(10):m2v(13);
    y_range = m2v(10):m2v(12);
    z_range = 1:table_h_vox;
    Map(x_range, y_range, z_range) = 2; % FIX: Type 2 for Table
    goal_spots.table3 = [m2v(11), m2v(11), landing_z_vox; ...
                         m2v(12), m2v(11), landing_z_vox];

    % --- Table 4 (2 Drones Max) ---
    % Position: X=[10, 13]m, Y=[15, 17]m
    x_range = m2v(10):m2v(13);
    y_range = m2v(15):m2v(17);
    z_range = 1:table_h_vox;
    Map(x_range, y_range, z_range) = 2; % FIX: Type 2 for Table
    goal_spots.table4 = [m2v(11), m2v(16), landing_z_vox; ...
                         m2v(12), m2v(16), landing_z_vox];

    % 4. Add Car (Double table height = 2m)
    car_h_vox = m2v(2.0);
    % Position: X=[15, 18]m, Y=[8, 14]m
    x_range = m2v(15):m2v(18);
    y_range = m2v(8):m2v(14);
    z_range = 1:car_h_vox;
    Map(x_range, y_range, z_range) = 3; % FIX: Type 3 for Car

    % 5. Add 10 Pendants (50x50cm, hanging) - COMPLEX LAYOUT
    pendant_w_vox = m2v(0.5); % 50cm width
    pendant_l_vox = m2v(0.5); % 50cm length
    
    % Pendants hang from 8m (ceiling) down to 4m
    pendant_z_start = m2v(4.0);
    pendant_z_end = Z_vox; % Hang from ceiling
    z_range = pendant_z_start:pendant_z_end;

    % Define centers of the 10 pendants
    % This layout is designed to force the A* to weave
    pendant_centers = [ ...
        % First "curtain" of pendants (before the car)
        m2v(22), m2v(5);
        m2v(22), m2v(10);
        m2v(22), m2v(15);
        m2v(22), m2v(20);
        
        % Pendants flanking the car
        m2v(16), m2v(5);  % Left of car
        m2v(16), m2v(17); % Right of car
        
        % Second "curtain" of pendants (between car and tables)
        m2v(12), m2v(7);
        m2v(12), m2v(12);
        m2v(12), m2v(17);
        m2v(12), m2v(22);
    ];

    half_w = floor(pendant_w_vox / 2);
    half_l = floor(pendant_l_vox / 2);

    for i = 1:size(pendant_centers, 1)
        x_c = pendant_centers(i, 1);
        y_c = pendant_centers(i, 2);
        
        x_range = (x_c - half_w):(x_c + half_w);
        y_range = (y_c - half_l):(y_c + half_l);
        
        % Ensure indices are valid
        x_range = max(1, min(X_vox, x_range));
        y_range = max(1, min(Y_vox, y_range));
        
        Map(x_range, y_range, z_range) = 4; % FIX: Type 4 for Pendant
    end
    
    disp('Map generation complete.');
end