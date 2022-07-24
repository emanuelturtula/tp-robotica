function particles = sample_motion_model(u, particles, use_roomba)
    % Samples new particle positions, based on old positions and odometry.
    %
    % u: odometry reading
    % x: set of old particles
    
    x = [particles(:).pose_x; particles(:).pose_y; particles(:).pose_t]';
    
    % Particle count
    pc = size(x, 1);
    
    odom = repmat([u(1), u(2), u(3)], pc, 1);
        
    if ~use_roomba
        % Noise parameters
        noise = [0.1 0.1 0.05 0.05];

        % Compute normal distributed noise
        M = zeros(pc, 3);
        S = repmat([
            max(0.00001, noise(3)*u(1)  + noise(4)*(abs(u(2)) + abs(u(3)))) ...
            max(0.00001, noise(1)*abs(u(2)) + noise(2)*u(1)) ...
            max(0.00001, noise(1)*abs(u(3)) + noise(2)*u(1))
        ], pc, 1);
        N = normrnd(M, S, pc, 3);

        % Add noise to the motion for every particle
        odom = odom + N;
    end
    
    % Compute new particle positions
    x = x + [
        odom(:, 1).*cos(x(:, 3) + odom(:, 2)), ...
        odom(:, 1).*sin(x(:, 3) + odom(:, 2)), ...
        odom(:, 2) + odom(:, 3)
    ];

    aux_x = num2cell(x(:, 1));
    aux_y = num2cell(x(:, 2));
    aux_t = num2cell(x(:, 3));
%     aux_t = num2cell(wrapToPi(x(:,3)));
    [particles(:).pose_x] = deal(aux_x{:});
    [particles(:).pose_y] = deal(aux_y{:});
    [particles(:).pose_t] = deal(aux_t{:});

end
