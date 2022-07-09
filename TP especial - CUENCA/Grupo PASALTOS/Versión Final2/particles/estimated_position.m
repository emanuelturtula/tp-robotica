function [est_pos, var_pos] = estimated_position(particles)

    pose_x = [particles(:).pose_x]';
    pose_y = [particles(:).pose_y]';
    pose_t = [particles(:).pose_t]';
    weights = [particles(:).weight];

    est_pos(:, 1) = weights*pose_x;
    est_pos(:, 2) = weights*pose_y;
    est_pos(:, 3) = weights*pose_t;
    est_pos(:, 3) = wrapTo2Pi(est_pos(:, 3));
    
    var_pos = var([pose_x, pose_y, pose_t], weights);
    
end