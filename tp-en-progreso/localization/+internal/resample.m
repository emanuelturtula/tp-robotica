function new_particles = resample(old_particles)

    new_particles = old_particles;

    M = numel(old_particles);
    
    r = rand(1)*M^-1; % Valor inicial de U
    c = old_particles(1).weight; % Umbral
    i = 1;
        
    for m = 1:M
        U = r + (m - 1)*M^-1;
        while(U > c) % Cuando supera el umbral avanza a la siguiente partícula
            i = i + 1;
            c = c + old_particles(i).weight;
        end
        new_particles(m) = old_particles(i);
    end

end