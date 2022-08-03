function newParticles = sample_motion_model(odometry, oldParticles)
        u = odometry.getOdometry();
        count = numel(oldParticles);
        particlePoses = repmat([0,0,0], count, 1);
        particlePoses(:, 1) = [oldParticles.x]';
        particlePoses(:, 2) = [oldParticles.y]';
        particlePoses(:, 3) = [oldParticles.theta]';

        odom = repmat([u(1), u(2), u(3)], count, 1);

        % Agregar ruido de medición
%         noise = [0.1 0.1 0.05 0.05];
%         M = zeros(count, 3);
%         S = repmat([
%             max(0.00001, noise(3)*u(1)  + noise(4)*(abs(u(2)) + abs(u(3)))) ...
%             max(0.00001, noise(1)*abs(u(2)) + noise(2)*u(1)) ...
%             max(0.00001, noise(1)*abs(u(3)) + noise(2)*u(1))
%         ], count, 1);
%         N = normrnd(M, S, count, 3);
%         odom = odom + N;

        % Proyectar las poses
        particlePoses = particlePoses + [
            odom(:, 1).*cos(particlePoses(:, 3) + odom(:, 2)), ...
            odom(:, 1).*sin(particlePoses(:, 3) + odom(:, 2)), ...
            odom(:, 2) + odom(:, 3)
        ]; 

        newParticles = oldParticles; 

        posesX = num2cell(particlePoses(:,1));
        posesY = num2cell(particlePoses(:,2));
        posesTheta = num2cell(particlePoses(:,3));
        [newParticles.x] = posesX{:};
        [newParticles.y] = posesY{:};
        [newParticles.theta] = posesTheta{:};
end

