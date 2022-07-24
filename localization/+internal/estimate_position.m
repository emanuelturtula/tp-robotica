function [estPos, varPos] = estimate_position(particles)
    x = [particles(:).x]';
    y = [particles(:).y]';
    theta = [particles(:).theta]';
    weights = [particles(:).weight];

    estPos(:, 1) = weights*x;
    estPos(:, 2) = weights*y;
    estPos(:, 3) = weights*theta;
    estPos(:, 3) = wrapTo2Pi(estPos(:, 3));

    varPos = var([x, y, theta], weights);
end

