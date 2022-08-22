function heur = heuristicForVigilance(cell, goal)

h_factor = 5;

heur = sqrt(norm(cell(1) - goal(1))^2 + norm(cell(2) - goal(2))^2);

heur = h_factor*heur;

end