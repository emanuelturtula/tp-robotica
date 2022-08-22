function [path_grid, path_world] = planningForExploration(start, map)

size_x = map.GridSize(2);
size_y = map.GridSize(1);

d_threshold = 0.3;

inflate(map, 0.1);
occMatrix = map.occupancyMatrix;
occThreshold = map.OccupiedThreshold;
[gridX, gridY] = find(occMatrix > occThreshold);
n_occupied_grid = size(gridX, 1);
occupied_grid_y(1:n_occupied_grid) = gridX;
occupied_grid_x(1:n_occupied_grid) = gridY;

% Inicializar de costos y matriz de heurísticas
costs = ones(size_y, size_x)*inf;
heuristics = zeros(size_y, size_x);
closed_list = zeros(size_y, size_x);

H_map = -(occMatrix.*log2(occMatrix) + (1 - occMatrix).*log2(1 - occMatrix));
H_map(isnan(H_map)) = 0;
p_bin_threshold = 0.6;
h_bin_threshold = -(p_bin_threshold.*log2(p_bin_threshold) + (1 - p_bin_threshold).*log2(1 - p_bin_threshold));
Hb_map = floor(H_map + 1 - h_bin_threshold);
heuristic_val = 0;

% Matrices del camino optimo
previous_x = zeros(size_y, size_x) - 1;
previous_y = zeros(size_y, size_x) - 1;

% Empieza a iterar por el start
parent = start;
costs(start(1), start(2)) = 0;

%loop until the goal is found
while Hb_map(parent(1), parent(2)) == 0 || norm(parent - start)/map.Resolution < d_threshold

    %generate mask to assign infinite costs for cells already visited
    closed_mask = closed_list;
    closed_mask(closed_mask == 1) = Inf;

    %find the candidates for expansion (open list/frontier)
    open_list = costs + closed_mask + heuristics;

    %check if a non-infinite entry exists in open list (list is not empty)
    if min(open_list(:)) == Inf
        break
    end
        
    %find the cell with the minimum cost in the open list
    [y, x] = find(open_list == min(open_list(:)));
    parent = [y(1), x(1)];

    %put parent in closed list
    closed_list(parent(1), parent(2)) = 1;
    
    %get neighbors of parent
    n = neighbors(parent, [size_y, size_x]);
    for i = 1:size(n, 1)
        child = n(i, :);

        %calculate the cost of reaching the cell
        norms2 = (occupied_grid_x(1:n_occupied_grid) - child(2)).^2 + (occupied_grid_y(1:n_occupied_grid) - child(1)).^2;
        child_wall_distance = sqrt(min(norms2))/map.Resolution;
        
        cost_val = costs(parent(1), parent(2)) + edgeCostForExploration(parent, child, child_wall_distance, ...
            [previous_y(parent(1), parent(2)), previous_x(parent(1), parent(2))]);

        %update cost of cell
        if(cost_val < costs(child(1), child(2)))
          costs(child(1), child(2)) = cost_val;
          heuristics(child(1), child(2)) = heuristic_val;

          %safe child's parent
          previous_x(child(1), child(2)) = parent(2);
          previous_y(child(1), child(2)) = parent(1);
        end
    end
    
end

N = 200;
path_grid = zeros(N, 2);
path_world = zeros(N, 2);
n = 0;

while(previous_x(parent(1), parent(2)) >= 0 && n <= N)
    n = n + 1;
    path_grid(n, :) = parent;
    path_world(n, :) = grid2world(map, parent);

    child = [previous_y(parent(1), parent(2)), previous_x(parent(1), parent(2))];
    parent = child;
end

n = n + 1;
path_grid(n, :) = parent;
path_world(n, :) = grid2world(map, parent);

path_grid = flipud(path_grid(1:n, :));
path_world = flipud(path_world(1:n, :));

end