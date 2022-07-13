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

            variance = var([poseX, poseY, poseT], pesos);
        end
               
        function robot = setParticles(robot, particles)
            arguments
                robot Robot
                particles (1,:) Particle
            end
            
            robot.particles = particles';
        end
        
        function particulas = getParticles(robot)
            particulas = robot.particles;
        end
                
        function robot = updateParticlesWithOdometry(robot, u)
            oldParticles = robot.particles;
            count = numel(oldParticles);
            particlePoses = repmat([0,0,0], count, 1);
            % TODO: Ver como desaparecer este for
            for i = 1:count
                particlePoses(i, :) = oldParticles(i).pose;
            end
            
            odom = repmat([u(1), u(2), u(3)], count, 1);
            %TODO: Implementar funcionalidad para cuando se usa la roomba
            %if ~use_roomba 
            % HACER X COSA
            %end
            
            particlePoses = particlePoses + [
                odom(:, 1).*cos(particlePoses(:, 3) + odom(:, 2)), ...
                odom(:, 1).*sin(particlePoses(:, 3) + odom(:, 2)), ...
                odom(:, 2) + odom(:, 3)
            ]; 
            
            newParticles = oldParticles;
            % TODO: Ver como desaparecer este for
            for n = 1:count
                newParticles(i).pose = particlePoses(i,:);
            end

            robot.particles = newParticles;
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
            delta_R = 0.02;
            Rt = 0.35/2;
            p_norm = normpdf(0, 0, sigma);
            p_arbitraria = 0.01;
            
            for n = 1:numel(newParticles)
                % Guardo las mediciones en la particula y las adapto a su
                % terna
                newParticles(n) = newParticles(n).setRanges(ranges, map);
                
                % A priori, su peso será 1
                newParticles(n).weight = 1;
                
                % Obtengo la pose de la particula
                particlePose = newParticles(n).pose;
                
                % Si la particula es valida (está dentro de los limites del
                % mapa), continuo
                if newParticles(n).isValid(map)                  
                    % Estos dos vectores terminan siendo coordenadas del
                    % mapa
                    x_z = [newParticles(n).ranges(:,1); particlePose(1)];
                    y_z = [newParticles(n).ranges(:,2); particlePose(2)];
                    
                    % Se convierten a una grilla relacionada al mapa. 
                    % Entiendo que eso es básicamente agarrar cada
                    % coordenada del mapa y mapearlo a una grilla 
                    % (que depende de la resolucion del mapa)
                    grid_xy = world2grid_tito(map, [x_z, y_z]);
                    
                    % Esos puntos de la grilla, tienen una correspondencia
                    % en el mapa de likelihood
                    likelihoods = likelihood_map(sub2ind(size(likelihood_map), grid_xy(:, 1), grid_xy(:, 2)));
                    
                    % Ademas, tenemos un vector de probabilidades, que
                    % asigna una probabilidad a cada coordenada
                    p = ones(1, length(x_z));
                    
                    % Lo que nos interesa, es la probabilidad de las
                    % mediciones, es decir, de todos los likelihoods
                    % excepto el ultimo elemento que es el likelihood de la
                    % pose de la particula
                    p(1:end-1) = normpdf(likelihoods(1:end-1), 0, sigma)/p_norm;
                    
                    % Calculamos el nuevo peso de la particula
                    newParticles(n).weight = newParticles(n).weight*prod(p);
                    
                    % Si el likelihood de la particula es menor a un número
                    % (TODO: VER QUE ES ESTE NUMERO), actualizamos el peso
                    if(likelihoods(end) < Rt + delta_R)
                        newParticles(n).weight = newParticles(n).weight*(likelihoods(end)/(Rt + delta_R));
                    end
                else
                    % Si la particula no es válida, le damos un peso de
                    % epsilon
                    newParticles(n).weight = epsilon;
                end
                % TODO: Ver que onda esto que quedo medio falopa, pero la
                % idea es actualizar el peso en base a una probabilidad
                % arbitraria.
                newParticles(n).weight = newParticles(n).weight*p_arbitraria^(length(newParticles(n).ranges(:,1)) - length(x_z) + 1);
            end
            w_avg = sum([newParticles(:).weight]);
            aux_w = num2cell([newParticles(:).weight]/w_avg);
            [newParticles(:).weight] = deal(aux_w{:});
            
            robot.particles = newParticles;
        end
        
        function robot = resampleParticles(robot)
            oldParticles = robot.particles;
            newParticles = oldParticles;
            
            particleCount = numel(oldParticles);
            % Valor inicial de U
            r = rand(1)*particleCount^-1; 
            % Umbral
            c = oldParticles(1).weight; 
            i = 1;

            for m = 1:particleCount
                U = r + (m - 1)*particleCount^-1;
                % Cuando supera el umbral avanza a la siguiente partícula
                while(U > c) 
                    i = i + 1;
                    c = c + oldParticles(i).weight;
                end
                newParticles(m) = oldParticles(i);
            end
            
            robot.particles = newParticles;
        end
    end
end

% TODO: Habría que ver que diferencia con world2grid
function [grid_pose] = world2grid_tito(map, pose)
    r = map.Resolution;
    grid_x = ceil(pose(:, 1)*r);
    grid_y = map.GridSize(1) - floor(pose(:, 2)*r);
    grid_x(grid_x == 0) = 1;
    grid_y(grid_y == 0) = 1;
    grid_pose = [grid_y, grid_x];
end