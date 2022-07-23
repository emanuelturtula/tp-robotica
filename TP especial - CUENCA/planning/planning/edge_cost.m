function cost = edge_cost(parent, child, child_wall_distance, pre_parent, patrullaje, est_t)

cost = inf;
collision_distance = 0.35/2 + 0.08;
close_distance = 0.35;
wall_distance = child_wall_distance;
diff = pre_parent - parent;
turn_factor = 1 + abs(parent(1) - child(1) - diff(1)) + abs(parent(2) - child(2) - diff(2));
if(pre_parent(1) == -1 && pre_parent(2) == -1)
%     turn_factor = 1;
    collision_distance = collision_distance + 0.08;
    next_angle = cart2pol(child(2) - parent(2), -(child(1) - parent(1)));
    turn_factor = 1 + abs(floor(wrapToPi(next_angle - est_t)/(pi/4)))*10;
end
if(wall_distance > collision_distance)
    cost = turn_factor*4;
    if(child(1) - parent(1) ~= 0 && child(2) - parent(2) ~= 0)
        cost = cost*sqrt(2);
    end
    if(wall_distance < close_distance)
        if(patrullaje)
            cost = cost*1.5;
        else
            cost = cost/4;
        end
    end
elseif(wall_distance > 0)
    cost = turn_factor*20;
end


end