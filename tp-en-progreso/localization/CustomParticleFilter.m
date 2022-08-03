classdef CustomParticleFilter
     
    properties
        varPosThreshold = 0.1;
        firstVarPosThreshold = 0.1/5;
        varAngleThreshold = 0.1*pi;
        nInitialParticles = 4000;
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
            obj = obj.estimatePosition();
            obj = obj.resample();
        end
        
        function [estPos, varPos] = getEstimatedPoseAndVariance(obj)
            estPos = obj.estimatedPose;
            varPos = obj.estimatedVar;
        end
        
        function plotParticles(obj, particlesFig, realPose)          
            internal.plot_particles(obj.particles, realPose, obj.estimatedPose, obj.map, particlesFig);
        end
    end

    methods (Access = private)
        
        function distanceMap = generateDistanceMap(obj)
            distanceMap = internal.generate_distance_map(obj.map);
        end
        
        function obj = initRandomParticles(obj, n, nAngles)           
            newParticles = internal.init_random_particles(obj.map, n, nAngles);
            obj.particles = newParticles;
        end
        
        function obj = resizeParticles(obj)
            if obj.firstLocalization == true                
                obj.firstLocalization = (max(obj.estimatedVar(1:2)) > obj.firstVarPosThreshold || obj.estimatedVar(3) > obj.varAngleThreshold);
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
            newParticles = internal.sample_motion_model(odometry, obj.particles);
            obj.particles = newParticles;
        end
        
        function obj = computeCorrectionStep(obj, ranges)  
            newParticles = internal.measurement_model(obj.particles, obj.lidar, obj.map, obj.distanceMap, obj.occCells, ranges);
            obj.particles = newParticles;
        end

        function obj = resample(obj)
            newParticles = internal.resample(obj.particles);
            obj.particles = newParticles;
        end
  
        function obj = estimatePosition(obj)           
            [estPos, varPos] = internal.estimate_position(obj.particles);
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

