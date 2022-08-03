function newParticles = init_random_particles(m, n, nAngles)
    poses = [
        unifrnd(m.XWorldLimits(1)+0.5, m.XWorldLimits(2)-1, n/nAngles, 1), ...
        unifrnd(m.YWorldLimits(1)+0.5, m.YWorldLimits(2)-1, n/nAngles, 1), ...
        unifrnd(-pi, pi, n/nAngles, 1)
    ];

    poses = repmat(poses, nAngles, 1);
    angleVector = linspace(-pi, pi - pi/nAngles, nAngles);
    for k = 1:nAngles
        poses(1 + (k-1)*n/nAngles:k*n/nAngles, 3) = angleVector(k);
    end

    newParticles = repmat(Particle, n, 1);
    posesX = num2cell(poses(:,1)');
    posesY = num2cell(poses(:,2)');
    posesTheta = num2cell(poses(:,3)');
    [newParticles.x] = posesX{:};
    [newParticles.y] = posesY{:};
    [newParticles.theta] = posesTheta{:};
end

