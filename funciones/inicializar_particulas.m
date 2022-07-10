function particulas = inicializar_particulas(cantidad, mapa, n_angulo)
    poses = [
        unifrnd(mapa.XWorldLimits(1) + 1, mapa.XWorldLimits(2) - 1, cantidad/n_angulo, 1), ...
        unifrnd(mapa.YWorldLimits(1) + 1, mapa.YWorldLimits(2) - 1, cantidad/n_angulo, 1), ...
        unifrnd(-pi, pi, cantidad/n_angulo, 1)
    ];

    poses = repmat(poses, n_angulo, 1);

    angle_vector = linspace(-pi, pi - pi/n_angulo, n_angulo);

    for k = 1:n_angulo
        poses(1 + (k - 1)*cantidad/n_angulo:k*cantidad/n_angulo, 3) = angle_vector(k);
    end
    
    particulas = repmat(Particula, cantidad, 1);
    for i = 1:length(poses(:,1))
        particulas(i).pose = poses(i,:);
    end
    
end