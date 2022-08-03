classdef Odometry
    
    properties (Access = private)
        odometry (1,3) double = [0,0,0];
    end
    
    methods
        function obj = Odometry(pose, previousPose)
            x = pose(1);
            previousX = previousPose(1);
            y = pose(2);
            previousY = previousPose(2);
            theta = wrapToPi(pose(3));
            
            previousTheta = previousPose(3);

            dtr = sqrt((x - previousX)^2 + (y - previousY)^2);
            dr1 = atan2(y - previousY, x - previousX) - previousTheta;
            dr2 = wrapToPi(theta - previousTheta - dr1);
            obj.odometry = [dtr, dr1, dr2];
        end
        
        function odo = getOdometry(obj)
            odo = obj.odometry; 
        end

    end
end

