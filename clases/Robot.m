%% Clase Robot

classdef Robot
    properties (Access = private)
        % Describe las dimensiones del robot
        dimensions = RobotDimensions;
        % Describe la pose actual
        initialPose = [0,0,0];
        % Describe si está en modo vigilancia o exploración
        vigilanceMode = false; 
        particles;
        lidar;
    end
    
    methods (Static)
        function robot = newRobot(dimensions, initialPose, mode, lidar)  
            arguments
                dimensions RobotDimensions
                initialPose (1,:) {mustBeNumeric}
                mode {mustBeNumericOrLogical}
                lidar LidarSensor
            end
            
            robot = Robot;
            robot.dimensions = dimensions;
            robot.initialPose = initialPose;
            robot.vigilanceMode = mode;
            robot.lidar = lidar;
        end
    end  
    
    methods
        function robot = attachLidar(robot, lidar)
            arguments
                robot Robot
                lidar LidarSensor
            end
            robot.lidar = lidar;
        end
        
        function [pos, variance] = estimatePosition(robot)
            poseX = robot.particles(:).pose(1)';
            poseY = robot.particles(:).pose(2)';
            poseT = robot.particles(:).pose(3)';
            pesos = robot.particles(:).weight;

            pos(:, 1) = pesos*poseX;
            pos(:, 2) = pesos*poseY;
            pos(:, 3) = wrapTo2Pi(pesos*poseT);

            variance = variance([poseX, poseY, poseT], pesos);
        end
               
        function robot = setParticles(robot, particles)
            arguments
                robot Robot
                particles (1,:) Particle
            end
            
            robot.particles = particles;
        end
        
        function particulas = getParticles(robot)
            particulas = robot.particles;
        end
                
        function robot = updateParticlesWithRanges(robot, ranges, map, likelihood_map)
            % TODO: Agregar validacion de parametros
            % TODO: Agregar el descartado de mediciones < 20cm 
            % (por los postes del robot)
            % TODO: Ver que pasa cuando usamos la roomba
            % TODO: Las mediciones están respecto al lidar. Hay que
            % pasarlas a la terna del global.
            
            newParticles = robot.particles;
            
            % ranges: es un conjunto de mediciones tomadas por el lidar. Tienen
            % que estar en la terna global, y cada elemento debe contar con una 
            % coordenada X e Y. Las mediciones tienen que ser validadas
            % anteriormente (no pueden estar por fuera del mapa)

            % map: es el mapa provisto en el enunciado

            % likelihood_map: un mapa con la probabilidad de encontrar un 
            % obstáculo (generado a partir del mapa del enunciado)

            sigma = 0.2;
            epsilon = exp(-14);

            for n = 1:numel(newParticles)
                newParticles(n).weight = 1;
                % Si la particula es valida (está dentro de los limites del
                % mapa), continuo
                if newParticles(n).isValid(map) 
                    % Obtengo la pose de la particula, del lidar y las
                    % mediciones vistas desde la particula
                    particlePose = newParticles(n).pose;
                    [lidarPose, scanAngles] = newParticles(n).getLidarPoseAndScanAngles();
                    readings = Readings.newReadings(ranges, scanAngles, lidarPose);
                    % Obtengo las lecturas validas en coordenadas
                    % cartesianas vistas desde la particula
                    [x, y] = readings.getValidReadingsInRectangular(map);
                    x_z = [x; particlePose(1)];
                    y_z = [y; particlePose(2)];
                    probabilities = ones(1, length(ranges));
                    grid_xyz = world2grid(map, [x_z, y_z]);
                else
                    newParticles(n).weight = epsilon;
                end
                
                world2grid
            end
            
        end
    end
end