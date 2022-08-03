function newParticles = measurement_model(actualParticles, actualLidar, actualMap, actualDistanceMap, occCells, ranges)
    sigma = 0.2;
    p_norm = normpdf(0, 0, sigma);
    p_arbitraria = 0.01;
    delta_R = 0.02;
    Rt = 0.35/2;
    res = actualMap.Resolution;
    epsilon = exp(-14);
    
    for k = 1:numel(actualParticles)
        weight = 1;     
        [proyX, proyY] = proyectRangesToParticlePose(actualParticles(k), ranges, actualLidar, actualMap);
        xZ = [proyX; actualParticles(k).x];
        yZ = [proyY; actualParticles(k).y];

        if actualParticles(k).isValid(actualMap) && occCells > 0
            rangesAndParticleLoc = world2grid_tito(actualMap, [xZ, yZ]);
            distancesIdx = sub2ind(size(actualDistanceMap), rangesAndParticleLoc(:, 1), rangesAndParticleLoc(:, 2));
            distances = actualDistanceMap(distancesIdx);
            p = ones(1, length(xZ));
            p(1:end - 1) = normpdf(distances(1:end - 1), 0, sigma)/p_norm;
            weight = weight*prod(p);
            if(distances(end) < Rt + delta_R)
                weight = weight*(distances(end)/(Rt + delta_R));
            end
        else
            weight = epsilon;
        end
        weight = weight*p_arbitraria^(length(ranges) - length(xZ) + 1);
        actualParticles(k).weight = weight;
    end
    avgWeight = sum([actualParticles(:).weight]);
    weights = num2cell([actualParticles(:).weight]/avgWeight);
    [actualParticles(:).weight] = weights{:};
    newParticles = actualParticles;
end

function [x_z, y_z] = proyectRangesToParticlePose(particle, ranges, lidar, map)
    scanAngles = lidar.scanAngles(1:end - 1);
    scanAngles = scanAngles(~isnan(ranges));
    sensorPose = [particle.x, particle.y, particle.theta];

    x_z = sensorPose(1) + ranges.*cos(sensorPose(3) + scanAngles');
    y_z = sensorPose(2) + ranges.*sin(sensorPose(3) + scanAngles');

    valid_ranges = logical((map.XWorldLimits(1) < x_z).*...
                   (map.XWorldLimits(2) > x_z).*...
                   (map.YWorldLimits(1) < y_z).*...
                   (map.YWorldLimits(2) > y_z));

    x_z = x_z(valid_ranges);
    y_z = y_z(valid_ranges);
end

function [grid_pose] = world2grid_tito(map, pose)
    r = map.Resolution;
    grid_x = ceil(pose(:, 1)*r);
    grid_y = map.GridSize(1) - floor(pose(:, 2)*r);
    grid_x(grid_x == 0) = 1;
    grid_y(grid_y == 0) = 1;
    grid_pose = [grid_y, grid_x];
end