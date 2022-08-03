function n = neighbors(cell, map_dimensions)

n_t = zeros(8, 2);
n_num = 0;

pos_y = cell(1);
pos_x = cell(2);
size_y = map_dimensions(1);
size_x = map_dimensions(2);

for j = pos_y - 1:pos_y + 1  %Recorro todos los posibles vecinos + cell
    for i = pos_x - 1:pos_x + 1
        if(j == pos_y && i == pos_x) % Esta es la cell, continuo al siguiente vecino.
            continue;
        elseif(j == 0 || j == size_y + 1 || i == 0 || i == size_x + 1) %Si supere los bordes, no adjunto el vecino.
            continue;
        end
        n_num = n_num + 1;
        n_t(n_num, :) = [j, i];
    end
end

n = n_t(1:n_num, :);
  
end