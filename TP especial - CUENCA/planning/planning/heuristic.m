function heur = heuristic(cell, goal, patrullaje)

if(patrullaje)
    h_factor = 5;
else
    h_factor = 0.1;
end

heur = sqrt(norm(cell(1) - goal(1))^2 + norm(cell(2) - goal(2))^2);

heur = h_factor*heur;

end