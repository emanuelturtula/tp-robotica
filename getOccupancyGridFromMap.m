function [occGridX, occGridY, N] = getOccupancyGridFromMap(map)
    arguments
        map occupancyMap;
    end
    pOccupied = 0.8;
    
    [occGridX, occGridY] = find(map.occupancyMatrix > pOccupied);
    N = size(occGridX, 1);
end