function collision = checkchoq(pose, map, likelihood_map)

grid_pose = world2grid_tito(map, pose);

if(likelihood_map(grid_pose(1), grid_pose(2)) < 0.35/2)
    collision = true;
else
    collision = false;
end

end