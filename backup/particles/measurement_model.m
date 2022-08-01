function particles = measurement_model(particles, map, red_ranges, likelihood_map, vigilancia)

    sigma = 0.2;
    p_norm = normpdf(0, 0, sigma);
    p_arbitraria = 0.01;
    delta_R = 0.02;
    Rt = 0.35/2;
    res = map.Resolution;
    epsilon = exp(-14);

    for k = 1:numel(particles)
        particles(k).weight = 1;
        x = [particles(k).pose_x, particles(k).pose_y, particles(k).pose_t];
        x_z = [particles(k).lidar_x; x(1)];
        y_z = [particles(k).lidar_y; x(2)];
        if(map.XWorldLimits(1) < x(1) && x(1) < map.XWorldLimits(2) && map.YWorldLimits(1) < x(2) && x(2) < map.YWorldLimits(2))
            if(particles(k).n_occupied_grid > 0)
                grid_xyz = world2grid_tito(map, [x_z, y_z]);
                p = ones(1, length(x_z));
                %%%%%% Vigilancia
                if(vigilancia)
                    grid_xyz_idx = sub2ind(size(likelihood_map), grid_xyz(:, 1), grid_xyz(:, 2));
                    d = likelihood_map(grid_xyz_idx);
                else
                %%%%%% Exploración
                    d = ones(1, length(x_z));
                    for j = 1:length(x_z)
                        norms2 = (particles(k).occupied_grid_x(1:particles(k).n_occupied_grid) - grid_xyz(j, 2)).^2 + (particles(k).occupied_grid_y(1:particles(k).n_occupied_grid) - grid_xyz(j, 1)).^2;
                        d(j) = sqrt(min(norms2))/res;
                    end
                end
                p(1:end - 1) = normpdf(d(1:end - 1), 0, sigma)/p_norm;
                particles(k).weight = particles(k).weight*prod(p);
                if(d(end) < Rt + delta_R)
                    particles(k).weight = particles(k).weight*(d(end)/(Rt + delta_R));
                end
            end % salteamos el cálculo cuando todavía no hay ninguna celda ocupada en el mapa
        else
            particles(k).weight = epsilon;
        end
        particles(k).weight = particles(k).weight*p_arbitraria^(length(red_ranges) - length(x_z) + 1);
    end
    w_avg = sum([particles(:).weight]);
    aux_w = num2cell([particles(:).weight]/w_avg);
    [particles(:).weight] = deal(aux_w{:});
    
end