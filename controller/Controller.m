classdef Controller   
    properties (Access = public)
        shouldPlan = false;
        shouldWait = false;
        finishedInitialTurn = false;
        followLeftRanges = true;
    end
    
    properties (Access = private)
        sampleTime = 0.1;
        explorationMode = false;
        
        %% Goals y Waypoints
        goal;
        goalRadius = 0.05;
        controller = controllerPurePursuit;
        
        %% Movimiento inicial
        isInitialMove = true; 
        initialTurn = 180;

        %% Poses
        expectedPose = [0,0,0]; % La pose inicial esperada
        previousExpectedPose = [0,0,0]; % La pose previa esperada
        realPose = [0,0,0]; % Esta es la pose que se calcula a partir del SLAM
        
    end
            
    methods (Access = public)
        function obj = Controller(startPose, sampleTime, isInExplorationMode)
            obj.sampleTime = sampleTime;
            obj.expectedPose = startPose;
            obj.realPose = startPose;
            obj.previousExpectedPose = startPose;
            obj.explorationMode = isInExplorationMode;
        end
        
        function [obj, v_cmd, w_cmd] = getSpeedCommands(obj, realPose, scans)

            obj.previousExpectedPose = obj.expectedPose;
            obj.realPose = realPose;
            
            if ~obj.explorationMode
                % 1. Inicialmente, hacemos un giro de 360 grados para un scan
                % inicial
                if obj.isInitialMove
                    [obj, v_cmd, w_cmd] = performInitialMove(obj, obj.initialTurn);
                    return
                end
                [obj, v_cmd, w_cmd] = getSpeedsForVigilanceMode(obj);
                obstacleDetected = checkObstacles(scans, 0.3, true);
                if obstacleDetected
                    w_cmd = 0.5;
                    v_cmd = 0;
                end
            else
                % Para el caso de exploracion, quiero que el robot siga a
                % la pared del lado izquierdo basado en los scans
%                 if obj.isInitialMove
%                     [obj, v_cmd, w_cmd] = performInitialMove(obj, obj.initialTurn*2);
%                     v_cmd = 0.01;
%                 else
%                     [obj, v_cmd, w_cmd] = getSpeedsForExploration(obj, scans);
%                 end
%                [obj, v_cmd, w_cmd] = getSpeedsForExploration(obj, scans);
                v_cmd = 0.1;
                w_cmd = 0;
                obstacleDetected = checkObstacles(scans, 0.4, obj.followLeftRanges);
                if obstacleDetected
                    if obj.followLeftRanges
                        w_cmd = -0.5;
                    else
                        w_cmd = 0.5;
                    end
                    v_cmd = 0.01;
                end
            end           
        end
        
        function obj = setNavigationPath(obj, navigationPath)
            [desiredVelocity, maxAngularVelocity, lookahead] = getControllerConfiguration(navigationPath);
            obj.controller.DesiredLinearVelocity = desiredVelocity;
            obj.controller.MaxAngularVelocity = maxAngularVelocity;
            obj.controller.LookaheadDistance = lookahead;
            obj.controller.Waypoints = navigationPath;
            obj.goal = navigationPath(end,:);
        end
    end
    
    methods (Access = private)        
        function [obj, v_cmd, w_cmd] = performInitialMove(obj, rotation)
            v_cmd = 0;
            w_cmd = 0.2;
            obj.expectedPose = getExpectedPose(obj.previousExpectedPose, v_cmd, w_cmd, obj.sampleTime);
            if obj.expectedPose(3) > deg2rad(rotation)
                obj.expectedPose(3) = wrapTo2Pi(obj.expectedPose(3));
                obj.isInitialMove = false;
                obj.finishedInitialTurn = true;
                obj.shouldPlan = true;
            end
        end
        
        function [obj, v_cmd, w_cmd] = getSpeedsForVigilanceMode(obj)
            % 1. Una vez que se hizo la rotacion inicial, tendríamos
            % que generar un plan
            if obj.shouldPlan 
                % Planear. Acá solo nos encargamos de generar velocidades. 
                % Cuando tengamos que planear, no vamos a hacer nada acá, 
                % solo dejamos al robot quieto.
                v_cmd = 0;
                w_cmd = 0;
                return
            end
            
            % 2. Si estamos acá, es porque se hizo el giro inicial y ya
            % tenemos un plan.
            distanceToGoal = norm(obj.realPose(1:2) - obj.goal(:)');
            if distanceToGoal > obj.goalRadius
                % Si todavia no llegamos al objetivo final, pedimos las
                % velocidades para llegar al proximo waypoint
               
                % HACK: Si trazamos una linea desde la posicion actual y la
                % el objetivo, y vemos que el robot tiene que girar más de
                % 90 grados, dejamos el robot fijo (velocidad angular 0) y 
                % giramos a maxima velocidad.
                v_cmd = 0;
                [shouldTurn, w_cmd] = shouldOnlyTurn(obj.realPose, obj.goal(:)');
                if shouldTurn
                    return
                end
                
                % Si no es un giro puro, le pedimos las velocidades al
                % controller
                [v_cmd, w_cmd] = obj.controller(obj.realPose);
                return
            end
            
            % 3. Llegado a este punto, estamos en uno de los objetivos, por
            % lo que tenemos que volver a planear. Si estamos en el
            % ejercicio de vigilancia, activamos un flag para esperar 3
            % segundos.
            obj.shouldPlan = true;
            obj.shouldWait = true;
            v_cmd = 0;
            w_cmd = 0;
        end
        
        function [obj, v_cmd, w_cmd] = getSpeedsForExploration(obj, scans)
            lookupAngle = pi/3;
            minDistanceToWall = 0.4;
            maxDistanceToWall = 0.6;
            longestMinRange = minDistanceToWall/sin(lookupAngle);
            longestMaxRange = maxDistanceToWall/sin(lookupAngle);
            
            minMean = (longestMinRange + minDistanceToWall)/2;
            maxMean = (longestMaxRange + maxDistanceToWall)/2;
            
            ranges = scans.Ranges;
            angles = scans.Angles;
            
            if obj.followLeftRanges
                indexes = find(angles > lookupAngle);
            else
                indexes = find(angles < -lookupAngle);
            end
            
            ranges = ranges(indexes);
            actualMean = sum(ranges)/numel(ranges);
            
            v_cmd = 0.01;
            w_cmd = 0;
            if actualMean < minMean
                if obj.followLeftRanges
                    w_cmd = -0.5;
                else
                    w_cmd = 0.5;
                end
                return
            end
            
            if actualMean > maxMean 
                if obj.followLeftRanges
                    w_cmd = 0.5;
                else
                    w_cmd = -0.5;
                end
                return
            end
            
            v_cmd = 0.1;
        end
    end
end

function expectedPose = getExpectedPose(previousPose, v_cmd, w_cmd, timestep)
    theta = previousPose(3);
    linearDistance = v_cmd*timestep;
    angularDistance = w_cmd*timestep;
    expectedPose(1:2) = previousPose(1:2) + linearDistance.*[cos(theta), sin(theta)];
    expectedPose(3) = previousPose(3) + angularDistance;
end

function [desiredVelocity, maxAngularVelocity, lookahead] = getControllerConfiguration(navigationPath)
    % TODO: Configurar el controller dependiendo del path
    desiredVelocity = 0.1;
    maxAngularVelocity = 0.5;
    lookahead = 0.1;
end

function [shouldTurn, w_cmd] = shouldOnlyTurn(actualPose, goal)
    vec = goal-actualPose(1:2);
    alpha = atan2(vec(2),vec(1));
    angleResult = abs(alpha)-abs(actualPose(3));
    if abs(angleResult) > deg2rad(45)
        shouldTurn = true;
        w_cmd = sign(angleResult)*0.5;
        return
    end

    shouldTurn = false;
    w_cmd = 0;
end
