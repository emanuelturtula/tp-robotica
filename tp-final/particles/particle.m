classdef particle
    %% PROPERTIES
    properties
        pose_x = 0;
        pose_y = 0;
        pose_t = 0;
        lidar_x;
        lidar_y;
        map;
        l_map;
        occupied_grid_x = ones(1000, 1);
        occupied_grid_y = ones(1000, 1);
        n_occupied_grid = 0;
        N_occupied_grid = 1000;
        weight = 0;
    end
    
    %% METHODS
    methods
        function obj = particle(map, pose, N)
            obj.map = map;
            obj.l_map = log(obj.map./(1 - obj.map));
            obj.pose_x = pose(1);
            obj.pose_y = pose(2);
            obj.pose_t = pose(3);
            obj.N_occupied_grid = N;
            obj.occupied_grid_x = ones(N, 1);
            obj.occupied_grid_y = ones(N, 1);
        end
    end
end