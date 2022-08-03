function distanceMap = generate_distance_map(map)
    sizeX = map.GridSize(1);
    sizeY = map.GridSize(2);
    pOccupied = map.OccupiedThreshold;
    resolution = map.Resolution;

    distanceMap = zeros(sizeX, sizeY);

    [occX, occY] = find(map.occupancyMatrix > pOccupied);

    for j = 1:sizeY
        for i = 1:sizeX
            d = (occX - i).^2 + (occY - j).^2;
            distanceMap(i, j) = sqrt(min(d))/resolution;
        end
    end
end

