classdef heuristic
    
    properties
       factor;
       heuristic_;
    end
    
    methods
        function obj = heuristic(patrullaje,cell,goal)
            if(patrullaje)
                obj.factor = 5;
            else
                obj.factor = 0.75;
            end
            obj.heuristic_ = (sqrt(norm(cell(1) - goal(1))^2 + norm(cell(2) - goal(2))^2))*obj.factor;
        end
    end
end
