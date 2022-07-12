classdef Readings
    
    properties
        readings;
        lidarPose;
    end
    
    methods (Static)
        function r = newReadings(ranges, angles, lidarPose)
            r = Readings;
            r.readings = [ranges', angles'];
            r.lidarPose = lidarPose;
        end
    end
    
    methods        
        function [x, y] = getValidReadingsInRectangular(readings, map)
            % TODO: Las validaciones sobre mediciones hay que hacerlas acá
            valid_ranges = logical((map.XWorldLimits(1) < x_z).*(map.XWorldLimits(2) > x_z).*(map.YWorldLimits(1) < y_z).*(map.YWorldLimits(2) > y_z));
            [x, y] = readings.toRectangular();
            x = x(valid_ranges);
            y = y(valid_ranges);
        end
    end
    
    methods (Access = private)
        function [x, y] = toRectangular(readings)
            ranges = readings(:, 1);
            angles = readings(:, 2);
            x_lidar = readings.lidarPose(1);
            y_lidar = readings.lidarPose(2);
            theta_lidar = readings.lidarPose(3);
            x = x_lidar + ranges.*cos(theta_lidar + angles);
            y = y_lidar + ranges.*sin(theta_lidar + angles);
        end
    end
end

