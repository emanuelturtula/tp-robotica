classdef heuristic
    
    properties
       cell;
       goal;
       patrullaje;
    end
    
    properties
       h_factor;
       heuristic_;
    end
    
    methods
        function obj = h_factor_set(obj, patrullaje)
            if(patrullaje)
                obj.h_factor = 5;
            else
                obj.h_factor = 0.75;
            end
        end
        function obj = heuristic_met(obj,cell,goal)
            %obj.h_factor = h_factor_set(obj, patrullaje);
            obj.heuristic_ = cell * goal;
            %obj.heuristic_ = (sqrt(norm(cell(1) - goal(1))^2 + norm(cell(2) - goal(2))^2))*obj.h_factor; 
        end
    end
end
