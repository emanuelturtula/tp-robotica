classdef CustomParticleFilter
     
    properties
        varPosThreshold = 0.1;
        firstVarPosThreshold = 0.1/5;
        varAngleThreshold = 0.1*pi;
        nInitialParticles = 2500;
        nInitialAngles = 10;
        nNominalParticles = 250;
        isLocalized = false;
        firstLocalization = true;
    end
    
    properties (Access = private)
        particles (1,:) Particle;
        map occupancyMap;
        occGridX double = 0;
        occGridY double = 0;
        occCells double = 0;
        distanceMap;
        lidar (1,1) LidarSensor;
        estimatedPose (1,3) double;
        estimatedVar (1,3) double = [inf, inf, inf]; 
    end
    
    methods
        function obj = get_weigths(obj)
            obj = obj.particles(:).weight;
        end
        
        function obj = get_particles(obj,max_w)
            obj = obj.particles(max_w);
        end
        
        function obj = CustomParticleFilter(map, lidar)
            arguments 
                map occupancyMap
                lidar LidarSensor
            end
            obj.map = map;
            obj.lidar = lidar;
            obj.distanceMap = obj.generateDistanceMap();
            [obj.occGridX, obj.occGridY, obj.occCells] = getOccupancyGridFromMap(map);
            obj = obj.initRandomParticles(obj.nInitialParticles, obj.nInitialAngles);
        end
                
        function obj = localize(obj, odometry, rawRanges)

            obj = obj.resizeParticles();
            obj = obj.computePredictionStep(odometry);
            obj = obj.computeCorrectionStep(rawRanges);
            obj = obj.resample();
            obj = obj.estimatePosition();
            
        end
        
        function [estPos, varPos] = getEstimatedPoseAndVariance(obj)
            estPos = obj.estimatedPose;
            varPos = obj.estimatedVar;
        end
        
        function plotParticles(obj, particlesFig, realPose)            
            Rt = 0.35/2;
            poseX = [obj.particles(:).x]';
            poseY = [obj.particles(:).y]';
            
            particlesAx = axes('Parent', particlesFig);
            particleViz = ParticleViz;
            hold(particlesAx, 'on')
            show(obj.map, 'Parent', particlesAx);
            particleViz.handle_est = plot(particlesAx, 0, 0, 'xg');
            particleViz.handle_particles = plot(particlesAx, poseX, poseY, '.b');
            particleViz.handle_pose = plot(particlesAx, 0, 0, 'xr');
            particleViz.handle_lidar = plot(particlesAx, 0, 0, '-g');
            particleViz.handle_robot = plot(particlesAx, 0, 0, '-g');

            set(particleViz.handle_particles, 'xdata', poseX, 'ydata', poseY);
            set(particleViz.handle_est, 'xdata', obj.estimatedPose(1), 'ydata', obj.estimatedPose(2));
            [xc, yc] = circlePoints(obj.estimatedPose(1), obj.estimatedPose(2), Rt, 17);
            set(particleViz.handle_robot, 'xdata', xc, 'ydata', yc);
%             [xc, yc] = circlePoints(realPose(1), realPose(2), Rt, 17);
%             set(particleViz.handle_pose, 'xdata', xc, 'ydata', yc);
            set(particleViz.handle_pose, 'xdata', realPose(1), 'ydata', realPose(2));

            len = 2*Rt;
            xp = [obj.estimatedPose(1), obj.estimatedPose(1) + (len*cos(obj.estimatedPose(3)))];
            yp = [obj.estimatedPose(2), obj.estimatedPose(2) + (len*sin(obj.estimatedPose(3)))];
            set(particleViz.handle_lidar, 'xdata', xp, 'ydata', yp);
            hold(particlesAx, 'off')
        end
    end

    methods (Access = private)
        
        function distanceMap = generateDistanceMap(obj)
            sizeX = obj.map.GridSize(1);
            sizeY = obj.map.GridSize(2);
            pOccupied = obj.map.OccupiedThreshold;
            %pOccupied = 0.8;
            resolution = obj.map.Resolution;

            distanceMap = zeros(sizeX, sizeY);

            [occX, occY] = find(obj.map.occupancyMatrix > pOccupied);

            for j = 1:sizeY
                for i = 1:sizeX
                    d = (occX - i).^2 + (occY - j).^2;
                    distanceMap(i, j) = sqrt(min(d))/resolution;
                end
            end
        end
        
        function obj = initRandomParticles(obj, n, nAngles)
            m = obj.map;
            poses = [
                unifrnd(m.XWorldLimits(1) + 1, m.XWorldLimits(2) - 1, n/nAngles, 1), ...
                unifrnd(m.YWorldLimits(1) + 1, m.YWorldLimits(2) - 1, n/nAngles, 1), ...
                unifrnd(-pi, pi, n/nAngles, 1)
            ];

            poses = repmat(poses, nAngles, 1);
            angleVector = linspace(-pi, pi - pi/nAngles, nAngles);
            for k = 1:nAngles
                poses(1 + (k-1)*n/nAngles:k*n/nAngles, 3) = angleVector(k);
            end
            
            
            newParticles = repmat(Particle, n, 1);
            posesX = num2cell(poses(:,1)');
            posesY = num2cell(poses(:,2)');
            posesTheta = num2cell(poses(:,3)');
            [newParticles.x] = posesX{:};
            [newParticles.y] = posesY{:};
            [newParticles.theta] = posesTheta{:};
            
            weights = 1/n;
            [newParticles.weight] = deal(weights);
            obj.particles = newParticles;
        end
        
        function obj = resizeParticles(obj)
            if obj.firstLocalization == true                
                obj.firstLocalization = ~(max(obj.estimatedVar(1:2)) > obj.firstVarPosThreshold || obj.estimatedVar(3) > obj.varAngleThreshold);
                poses = [obj.particles(:).x; obj.particles(:).y; obj.particles(:).theta]';
                noise = normrnd(0, 0.1, obj.nInitialParticles, 3);
                poses = poses + noise;
                x = num2cell(poses(:, 1));
                y = num2cell(poses(:, 2));
                theta = num2cell(poses(:, 3));
                [obj.particles(:).x] = x{:};
                [obj.particles(:).y] = y{:};
                [obj.particles(:).theta] = theta{:};
            else
                obj.isLocalized = ~(max(obj.estimatedVar(1:2)) > obj.varPosThreshold || obj.estimatedVar(3) > obj.varAngleThreshold);
                obj.particles = obj.particles(1:obj.nNominalParticles);
            end
        end
        
        function obj = computePredictionStep(obj, odometry)
            arguments
                obj CustomParticleFilter
                odometry Odometry
            end
            
            u = odometry.getOdometry();
            % TODO: Falta agregar ruido al modelo de movimiento 
            oldParticles = obj.particles;
            
            count = numel(oldParticles);
            particlePoses = repmat([0,0,0], count, 1);
            particlePoses(:, 1) = [oldParticles.x]';
            particlePoses(:, 2) = [oldParticles.y]';
            particlePoses(:, 3) = [oldParticles.theta]';
            
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
            
            posesX = num2cell(particlePoses(:,1));
            posesY = num2cell(particlePoses(:,2));
            posesTheta = num2cell(particlePoses(:,3));
            [newParticles.x] = posesX{:};
            [newParticles.y] = posesY{:};
            [newParticles.theta] = posesTheta{:};
           
            obj.particles = newParticles;
        end
        
        function obj = computeCorrectionStep(obj, ranges)            
            sigma = 0.2;
            p_norm = normpdf(0, 0, sigma);
            p_arbitraria = 0.01;
            delta_R = 0.02;
            Rt = 0.35/2;
            res = obj.map.Resolution;
            epsilon = exp(-14);
            
            actualParticles = obj.particles;
            actualLidar = obj.lidar;
            actualMap = obj.map;
            actualDistanceMap = obj.distanceMap;
            for k = 1:numel(actualParticles)
                actualParticles(k).weight = 1;     
                [proyX, proyY] = proyectRangesToParticlePose(actualParticles(k), ranges, actualLidar, actualMap);
                xZ = [proyX; actualParticles(k).x];
                yZ = [proyY; actualParticles(k).y];
                
                if obj.particles(k).isValid(obj.map) && obj.occCells > 0
                    rangesAndParticleLoc = world2grid(actualMap, [xZ, yZ]);
                    distancesIdx = sub2ind(size(actualDistanceMap), rangesAndParticleLoc(:, 1), rangesAndParticleLoc(:, 2));
                    distances = actualDistanceMap(distancesIdx);
                    p = ones(1, length(xZ));
                    p(1:end - 1) = normpdf(distances(1:end - 1), 0, sigma)/p_norm;
                    actualParticles(k).weight = actualParticles(k).weight*prod(p);
                    if(distances(end) < Rt + delta_R)
                        actualParticles(k).weight = actualParticles(k).weight*(distances(end)/(Rt + delta_R));
                    end
                else
                    actualParticles(k).weight = epsilon;
                end
                actualParticles(k).weight = actualParticles(k).weight*p_arbitraria^(length(ranges) - length(xZ) + 1);
            end
            avgWeight = sum([actualParticles(:).weight]);
            weights = num2cell([actualParticles(:).weight]/avgWeight);
            [actualParticles(:).weight] = weights{:};
            obj.particles = actualParticles;
        end

        function obj = resample(obj)
            arguments
                obj CustomParticleFilter
            end
            oldParticles = obj.particles;
            newParticles = obj.particles;

            M = numel(oldParticles);

            r = rand(1)*M^-1; % Valor inicial de U
            c = oldParticles(1).weight; % Umbral
            i = 1;

            for m = 1:M
                U = r + (m - 1)*M^-1;
                % Cuando supera el umbral avanza a la siguiente partícula
                while(U > c) 
                    i = i + 1;
                    c = c + oldParticles(i).weight;
                end
                newParticles(m) = oldParticles(i);
            end
            
            obj.particles = newParticles;
        end
        
        function obj = estimatePosition(obj)
            x = [obj.particles(:).x]';
            y = [obj.particles(:).y]';
            theta = [obj.particles(:).theta]';
            weights = [obj.particles(:).weight];
            
            estPos(:, 1) = weights*x;
            estPos(:, 2) = weights*y;
            estPos(:, 3) = weights*theta;
            estPos(:, 3) = wrapTo2Pi(estPos(:, 3));

            varPos = var([x, y, theta], weights);
            
            obj.estimatedPose = estPos;
            obj.estimatedVar = varPos;
        end
    end
end

%% Funciones auxiliares

function [occGridX, occGridY, N] = getOccupancyGridFromMap(map)
    arguments
        map occupancyMap;
    end
    pOccupied = 0.8;
    
    [occGridX, occGridY] = find(map.occupancyMatrix > pOccupied);
    N = size(occGridX, 1);
end

function [x_z, y_z] = proyectRangesToParticlePose(particle, ranges, lidar, map)
    scanAngles = lidar.scanAngles(1:end - 1);
    scanAngles = scanAngles(~isnan(ranges));
    sensorPose = [particle.x, particle.y, particle.theta];

    x_z = sensorPose(1) + ranges.*cos(sensorPose(3) + scanAngles');
    y_z = sensorPose(2) + ranges.*sin(sensorPose(3) + scanAngles');

    valid_ranges = logical((map.XWorldLimits(1) < x_z).*...
                   (map.XWorldLimits(2) > x_z).*...
                   (map.YWorldLimits(1) < y_z).*...
                   (map.YWorldLimits(2) > y_z));

    x_z = x_z(valid_ranges);
    y_z = y_z(valid_ranges);
end

function debugPoses(map, poses, weight)
    if weight > 1E-5
        figure;
        surf(map);
        hold on
        plot(poses(1:end-1,2), poses(1:end-1,1), '*r');
        plot(poses(end,2), poses(end,1), '*w');
        hold off
        disp(weight);
    end
end

function [cx,cy] = circlePoints(x,y,R,N)
    theta = linspace(-pi,pi,N);
    cx = R*cos(theta) + x;
    cy = R*sin(theta) + y;
end