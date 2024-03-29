function [path_grid, path_world] = planning(start, goal, map, likelihood_map, particle, vigilancia, timestep, planning_ax, planning_fig, print_planning, show_planning, est_t)

size_x = map.GridSize(2);
size_y = map.GridSize(1);

d_threshold = 0.3;

% Inicializar de costos y matriz de heur�sticas
costs = ones(size_y, size_x)*inf;
heuristics = zeros(size_y, size_x);
closed_list = zeros(size_y, size_x);

if(~vigilancia)
    particle.map = 1 - 1./(1 + exp(particle.l_map));
    H_map = -(particle.map.*log2(particle.map) + (1 - particle.map).*log2(1 - particle.map));
    H_map(isnan(H_map)) = 0;
    p_bin_threshold = 0.7;
    h_bin_threshold = -(p_bin_threshold.*log2(p_bin_threshold) + (1 - p_bin_threshold).*log2(1 - p_bin_threshold));
    Hb_map = floor(H_map + 1 - h_bin_threshold);
    heuristic_val = 0;
end

% Matrices del camino �ptimo
previous_x = zeros(size_y, size_x) - 1;
previous_y = zeros(size_y, size_x) - 1;

%%%%% plots
plot_planning = false;

if(show_planning || print_planning)
    plot_planning = true;
end

if(~isvalid(planning_fig))
    plot_planning = false;
end

if(plot_planning)
    cla(planning_ax)
    hold(planning_ax, 'on')
    if(vigilancia)
        show(map, 'Parent', planning_ax);
        aux = grid2world(map, goal);
        plot(planning_ax, aux(1), aux(2), '.g', 'MarkerSize', 10);
    else
        aux_map = robotics.OccupancyGrid(particle.map, map.Resolution);
        show(aux_map, 'Parent', planning_ax);
    end
    aux = grid2world(map, start);
    plot(planning_ax, aux(1), aux(2), '.r', 'MarkerSize', 10);
%     if(show_planning)
%         pause(1);
%     end
end

% Empieza a iterar por el start
parent = start;
costs(start(1), start(2)) = 0;

%loop until the goal is found
while((vigilancia && (parent(1) ~= goal(1) || parent(2) ~= goal(2))) || (~vigilancia && ((Hb_map(parent(1), parent(2)) == 0) || norm(parent - start)/map.Resolution < d_threshold)))

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
        if(vigilancia)
            child_wall_distance = likelihood_map(child(1), child(2));
        else
            norms2 = (particle.occupied_grid_x(1:particle.n_occupied_grid) - child(2)).^2 + (particle.occupied_grid_y(1:particle.n_occupied_grid) - child(1)).^2;
            child_wall_distance = sqrt(min(norms2))/map.Resolution;
        end
        cost_val = costs(parent(1), parent(2)) + edge_cost(parent, child, child_wall_distance, ...
            [previous_y(parent(1), parent(2)), previous_x(parent(1), parent(2))], vigilancia, est_t);

        if(vigilancia)
            heuristic_val = heuristic(child, goal, vigilancia);
        end

        %update cost of cell
        if(cost_val < costs(child(1), child(2)))
          costs(child(1), child(2)) = cost_val;
          heuristics(child(1), child(2)) = heuristic_val;

          %safe child's parent
          previous_x(child(1), child(2)) = parent(2);
          previous_y(child(1), child(2)) = parent(1);
        end
    end
    
    if(plot_planning)
        aux = grid2world(map, parent);
        plot(planning_ax, aux(1), aux(2), '.y', 'MarkerSize', 5);
        if(parent(1) == start(1) && parent(2) == start(2))
            aux = grid2world(map, start);
            plot(planning_ax, aux(1), aux(2), '.r', 'MarkerSize', 10);
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
    if(plot_planning)
        plot(planning_ax, path_world(n, 1), path_world(n, 2), '.b', 'MarkerSize', 5);
    end
    
    if(plot_planning)
        if(parent(1) == goal(1) && parent(2) == goal(2))
            aux = grid2world(map, goal);
            plot(planning_ax, aux(1), aux(2), '.g', 'MarkerSize', 5);
        end
    end
    child = [previous_y(parent(1), parent(2)), previous_x(parent(1), parent(2))];
    parent = child;
end

if(plot_planning)
    hold(planning_ax, 'off')
end

if(print_planning)
    filename = sprintf('plots/planning/planning_%03d.png', timestep);
    print(planning_fig, filename, '-dpng');
end

n = n + 1;
path_grid(n, :) = parent;
path_world(n, :) = grid2world(map, parent);

path_grid = flipud(path_grid(1:n, :));
path_world = flipud(path_world(1:n, :));

end