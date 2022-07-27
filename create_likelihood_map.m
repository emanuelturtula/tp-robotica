% Creación de likelihood map

load mapa_2022_1c.mat;

imagen_mapa = 1 - double(imread('mapa_2022_1c.tiff'))/255;
map = robotics.OccupancyGrid(imagen_mapa, 25);

size_x = map.GridSize(2);
size_y = map.GridSize(1);
p_occupied = map.OccupiedThreshold;
res = map.Resolution;

likelihood_map = zeros(size_y, size_x);

[occupied_y, occupied_x] = find(map.occupancyMatrix > p_occupied);

for j = 1:size_y
    for i = 1:size_x
        norms2 = (occupied_x - i).^2 + (occupied_y - j).^2;
        likelihood_map(j, i) = sqrt(min(norms2))/res;
    end
end

save('likelihood_map.mat', 'likelihood_map', '-mat');