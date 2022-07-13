classdef Particle
    % Particle es una representación de una partícula. La partícula es una
    % "instancia" del robot, por lo que tiene una pose, y tiene un sensor
    % lidar attacheado. También posee mediciones en su terna.
       
    properties
        weight = 0;
        pose = [0,0,0];
        lidar = LidarSensor;
        ranges;
    end
  
    methods (Static)
        function particle = newParticle(lidar)
            arguments
                lidar LidarSensor
            end
            particle = Particle;
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

            particles(1:amount) = Particle;
            for i = 1:length(poses(:,1))
                particles(i) = Particle.newParticle(lidar);
                particles(i).pose = poses(i,:);
            end
        end
    end
    
    methods 
        function result = isValid(particle, map)
            % Devuelve true si la particula está dentro del mapa. Caso
            % contrario, devuelve false
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
        
        function particle = setRanges(particle, lidarRanges, map)
            % setRanges toma las mediciones en la terna del lidar
            % y guarda las mediciones en coordenadas cartesianas de la
            % terna de la partícula. Utiliza el mapa para solo guardar
            % mediciones válidas
           
            [sensorPose, scanAngles] = particle.getLidarPoseAndScanAngles();
            
            % Se filtran las mediciones inválidas
            lidarRanges = lidarRanges(~isnan(lidarRanges(1:end - 1)));
            scanAngles = scanAngles(~isnan(scanAngles(1:end - 1)));
            
            x_z = sensorPose(1) + lidarRanges.*cos(sensorPose(3) + scanAngles');
            y_z = sensorPose(2) + lidarRanges.*sin(sensorPose(3) + scanAngles');
            
            validRanges = logical((map.XWorldLimits(1) < x_z).*...
                                  (map.XWorldLimits(2) > x_z).*...
                                  (map.YWorldLimits(1) < y_z).*...
                                  (map.YWorldLimits(2) > y_z));
                              
            particle.ranges = [x_z(validRanges), y_z(validRanges)];             
        end
    end
    
    methods (Access = private)
        function [sensorPose, scanAngles] = getLidarPoseAndScanAngles(particle)
            % Cada particula es una 'instancia' de un robot, por lo que las
            % particulas poseen un lidar también. Al tomar mediciones, hay 
            % que transformarlas a la terna de la particula, por lo que
            % necesitamos estos datos del lidar
            sensorAngleOffset = 0;
            
            x = particle.pose(1);
            y = particle.pose(2);
            theta = particle.pose(3);
            offsetVec = [cos(theta) -sin(theta);
                         sin(theta)  cos(theta)]*particle.lidar.sensorOffset';

            sensorLoc = [x, y] + offsetVec';
            sensorPose = [sensorLoc, theta + sensorAngleOffset];
            scanAngles = particle.lidar.scanAngles;
        end
    end
end

