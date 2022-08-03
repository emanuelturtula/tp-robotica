function l_map = occupancy_grid_mapping(particle, map, red_ranges, scanAngles)

l_map = particle.l_map;
pose = [particle.pose_x, particle.pose_y, particle.pose_t];

p_occupied = 0.8;
p_free = 1-p_occupied;
p_robot = 0.1;
p_scan = 0.47;
Rt = 0.35/2;

p_clean_threshold = 0.65;
l_clean_threshold = log(p_clean_threshold/(1 - p_clean_threshold));
min_clean_threshold = 0.1;
clean_factor = 1.2;
logical_matrix = logical((abs(l_map) < l_clean_threshold).*(abs(l_map) > min_clean_threshold));
aux = find(logical_matrix);
l_map(aux) = l_map(aux)/clean_factor;

interPts = [particle.lidar_x, particle.lidar_y];

interPts_grid = world2grid_tito(map, interPts);
[min_dist, ~] = min(sqrt((interPts(:,1) - pose(1)).^2 + (interPts(:,2) - pose(2)).^2));
min_dist_mod = max(0, min_dist/sqrt(2));

aux_min_x = max([map.XWorldLimits(1), pose(1) - min_dist_mod]);
aux_min_y = max([map.YWorldLimits(1), pose(2) - min_dist_mod]);
aux_max_x = min([map.XWorldLimits(2), pose(1) + min_dist_mod]);
aux_max_y = min([map.YWorldLimits(2), pose(2) + min_dist_mod]);
dl_corner = world2grid_tito(map, [aux_min_x, aux_min_y]);
ur_corner = world2grid_tito(map, [aux_max_x, aux_max_y]);
l_map(ur_corner(1):dl_corner(1), dl_corner(2):ur_corner(2)) = particle.l_map(ur_corner(1):dl_corner(1), dl_corner(2):ur_corner(2)) + log(p_free/(1 - p_free));

aux_min_x = max([map.XWorldLimits(1), pose(1) - Rt]);
aux_min_y = max([map.YWorldLimits(1), pose(2) - Rt]);
aux_max_x = min([map.XWorldLimits(2), pose(1) + Rt]);
aux_max_y = min([map.YWorldLimits(2), pose(2) + Rt]);
dl_corner = world2grid_tito(map, [aux_min_x, aux_min_y]);
ur_corner = world2grid_tito(map, [aux_max_x, aux_max_y]);
l_map(ur_corner(1):dl_corner(1), dl_corner(2):ur_corner(2)) = particle.l_map(ur_corner(1):dl_corner(1), dl_corner(2):ur_corner(2)) + log(p_robot/(1 - p_robot));

aux = sub2ind(size(l_map), interPts_grid(:, 1), interPts_grid(:, 2));
l_map(aux) = particle.l_map(aux) + log(p_occupied/(1 - p_occupied));

[~, right_idx] = min(wrapToPi(scanAngles + particle.pose_t));
[~, up_idx] = min(wrapToPi(scanAngles + particle.pose_t - pi/2));
[~, left_idx] = min(wrapToPi(scanAngles + particle.pose_t - pi));
[~, down_idx] = min(wrapToPi(scanAngles + particle.pose_t - 3*pi/2));
idx_vector = [left_idx, up_idx, right_idx, down_idx];
[~, max_idx] = max(idx_vector);
idx_vector = wshift('1', idx_vector, max_idx);

min_dist_vec = zeros(4, 1);
for i = 1:3
    [min_dist_vec(i), ~] = min(red_ranges(idx_vector(i):idx_vector(i + 1)));
end
[min_dist_vec(4), ~] = min([red_ranges(idx_vector(4):end); red_ranges(1:idx_vector(1))]);
min_dist_vec = wshift('1', min_dist_vec, -max_idx);
min_dist_vec_mod = min_dist_vec/sqrt(2) - 0.08;

%left - up
aux_min_x = max([map.XWorldLimits(1), pose(1) - min_dist_vec_mod(1)]);
aux_min_y = max([map.YWorldLimits(1), pose(2)]);
aux_max_x = min([map.XWorldLimits(2), pose(1)]);
aux_max_y = min([map.YWorldLimits(2), pose(2) + min_dist_vec_mod(1)]);
dl_corner = world2grid_tito(map, [aux_min_x, aux_min_y]);
ur_corner = world2grid_tito(map, [aux_max_x, aux_max_y]);
l_map(ur_corner(1):dl_corner(1), dl_corner(2):ur_corner(2)) = particle.l_map(ur_corner(1):dl_corner(1), dl_corner(2):ur_corner(2)) + log(p_scan/(1 - p_scan));

%up - right
aux_min_x = max([map.XWorldLimits(1), pose(1)]);
aux_min_y = max([map.YWorldLimits(1), pose(2)]);
aux_max_x = min([map.XWorldLimits(2), pose(1) + min_dist_vec_mod(2)]);
aux_max_y = min([map.YWorldLimits(2), pose(2) + min_dist_vec_mod(2)]);
dl_corner = world2grid_tito(map, [aux_min_x, aux_min_y]);
ur_corner = world2grid_tito(map, [aux_max_x, aux_max_y]);
l_map(ur_corner(1):dl_corner(1), dl_corner(2):ur_corner(2)) = particle.l_map(ur_corner(1):dl_corner(1), dl_corner(2):ur_corner(2)) + log(p_scan/(1 - p_scan));

%right - down
aux_min_x = max([map.XWorldLimits(1), pose(1)]);
aux_min_y = max([map.YWorldLimits(1), pose(2) - min_dist_vec_mod(3)]);
aux_max_x = min([map.XWorldLimits(2), pose(1) + min_dist_vec_mod(3)]);
aux_max_y = min([map.YWorldLimits(2), pose(2)]);
dl_corner = world2grid_tito(map, [aux_min_x, aux_min_y]);
ur_corner = world2grid_tito(map, [aux_max_x, aux_max_y]);
l_map(ur_corner(1):dl_corner(1), dl_corner(2):ur_corner(2)) = particle.l_map(ur_corner(1):dl_corner(1), dl_corner(2):ur_corner(2)) + log(p_scan/(1 - p_scan));

%down - left
aux_min_x = max([map.XWorldLimits(1), pose(1) - min_dist_vec_mod(4)]);
aux_min_y = max([map.YWorldLimits(1), pose(2) - min_dist_vec_mod(4)]);
aux_max_x = min([map.XWorldLimits(2), pose(1)]);
aux_max_y = min([map.YWorldLimits(2), pose(2)]);
dl_corner = world2grid_tito(map, [aux_min_x, aux_min_y]);
ur_corner = world2grid_tito(map, [aux_max_x, aux_max_y]);
l_map(ur_corner(1):dl_corner(1), dl_corner(2):ur_corner(2)) = particle.l_map(ur_corner(1):dl_corner(1), dl_corner(2):ur_corner(2)) + log(p_scan/(1 - p_scan));

end