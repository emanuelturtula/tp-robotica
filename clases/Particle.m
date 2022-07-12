classdef Particle

    properties
        weight = 0;
        pose = [0,0,0];
        lidar;
    end
  
    methods (Static)
        function particle = newParticle(lidar)
            arguments
                lidar Lidar
            end
            particle.lidar = lidar;
        end
        
        function particles = initRandomParticles(amount, map, n_angle, lidar)
            poses = [
                unifrnd(map.XWorldLimits(1) + 1, map.XWorldLimits(2) - 1, amount/n_angle, 1), ...
                unifrnd(map.YWorldLimits(1) + 1, map.YWorldLimits(2) - 1, amount/n_angle, 1), ...
                unifrnd(-pi, pi, amount/n_angle, 1)
            ];

            poses = repmat(poses, n_angle, 1);

            angle_vector = linspace(-pi, pi - pi/n_angle, n_angle);

            for k = 1:n_angle
                poses(1 + (k - 1)*amount/n_angle:k*amount/n_angle, 3) = angle_vector(k);
            end

            particles = repmat(Particle.newParticle(lidar), amount, 1);
            for i = 1:length(poses(:,1))
                particles(i).pose = poses(i,:);
            end
        end
    end
    
    methods 
        function result = isValid(particle, map)
            arguments
                particle Particle
                map occupancyMap
            end
            
            x = particle.pose(1);
            y = particle.pose(2);
            
            if (map.XWorldLimits(1) < x && ...
                map.XWorldLimits(2) > x && ...
                map.YWorldLimits(1) < y && ...
                map.YWorldLimits(2) > y )
                
                result = true;
                return
            end
            
            result = false;
        end
        
        function [sensorPose, scanAngles] = getLidarPoseAndScanAngles(particle)
            % Cada particula es una 'instancia' de un robot, por lo que las
            % particulas poseen un lidar también. Al tomar mediciones, hay 
            % que transformarlas a la terna de la particula, por lo que
            % necesitamos estos datos del lidar
            x = particle.pose(1);
            y = particle.pose(2);
            theta = particle.pose(3);
            offsetVec = [cos(theta) -sin(theta);
                         sin(theta)  cos(theta)]*particle.lidar.sensorOffset';

            sensorLoc = [x, y] + offsetVec';
            sensorPose = [sensorLoc, theta + particle.lidar.sensorAngleOffset];
            scanAngles = particle.lidar.scanAngles;
        end
    end
end

