classdef Particle

    properties
        weight = 0;
        x = 0;
        y = 0;
        theta = 0;
        lidarX;
        lidarY;
        map;
    end
    
    methods       
        function result = isValid(this, map)                      
            if (map.XWorldLimits(1) < this.x && ...
                map.XWorldLimits(2) > this.x && ...
                map.YWorldLimits(1) < this.y && ...
                map.YWorldLimits(2) > this.y )
                
                result = true;
                return
            end
            
            result = false;
        end
        function obj = get_l_map(obj, map)
            obj = log(map./(1 - map));
        end
    end
end

