function [grid_pose] = world2grid_(map, pose)

r = map.Resolution;
grid_x = ceil(pose(:, 1)*r);
grid_y = map.GridSize(1) - floor(pose(:, 2)*r);
grid_x(grid_x == 0) = 1;
grid_y(grid_y == 0) = 1;
grid_pose = [grid_y, grid_x];

end