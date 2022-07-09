function particle_poses = initialize_particles(count, map, n_angle)

particle_poses = [
        unifrnd(map.XWorldLimits(1) + 1, map.XWorldLimits(2) - 1, count/n_angle, 1), ...
        unifrnd(map.YWorldLimits(1) + 1, map.YWorldLimits(2) - 1, count/n_angle, 1), ...
        unifrnd(-pi, pi, count/n_angle, 1)
    ];

particle_poses = repmat(particle_poses, n_angle, 1);

angle_vector = linspace(-pi, pi - pi/n_angle, n_angle);

for k = 1:n_angle
    particle_poses(1 + (k - 1)*count/n_angle:k*count/n_angle, 3) = angle_vector(k);
end

end