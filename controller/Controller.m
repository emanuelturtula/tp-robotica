classdef Controller   
    properties (Access = public)
        shouldPlan = false;
        shouldWait = false;
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
        initialTurn = 360;

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
        
        function [obj, v_cmd, w_cmd] = getSpeedCommands(obj, realPose)

            obj.previousExpectedPose = obj.expectedPose;
            obj.realPose = realPose;
            % 1. Inicialmente, hacemos un giro de 360 grados para un scan
            % inicial
            if obj.isInitialMove
                [obj, v_cmd, w_cmd] = performInitialMove(obj);
                return
            end
            
            % 2. Luego, una vez que se hizo la rotacion inicial, tendríamos
            % que generar un plan
            if obj.shouldPlan 
                % Planear. Acá solo nos encargamos de generar velocidades. 
                % Cuando tengamos que planear, no vamos a hacer nada acá, 
                % solo dejamos al robot quieto.
                v_cmd = 0;
                w_cmd = 0;
                return
            end
            
            % 3. Si estamos acá, es porque se hizo el giro inicial y ya
            % tenemos un plan.
            distanceToGoal = norm(realPose(1:2) - obj.goal(:)');
            if distanceToGoal > obj.goalRadius
                % Si todavia no llegamos al objetivo final, pedimos las
                % velocidades para llegar al proximo waypoint
               
                % HACK: Si trazamos una linea desde la posicion actual y la
                % el objetivo, y vemos que el robot tiene que girar más de
                % 90 grados, dejamos el robot fijo (velocidad angular 0) y 
                % giramos a maxima velocidad.
                v_cmd = 0;
                [shouldTurn, w_cmd] = shouldOnlyTurn(realPose, obj.goal(:)');
                if shouldTurn
                    return
                end
                
                % Si no es un giro puro, le pedimos las velocidades al
                % controller
                [v_cmd, w_cmd] = obj.controller(realPose);
                return
            end
            
            % 4. Llegado a este punto, estamos en uno de los objetivos, por
            % lo que tenemos que volver a planear. Si estamos en el
            % ejercicio de vigilancia, activamos un flag para esperar 3
            % segundos.
            obj.shouldPlan = true;
            if ~obj.explorationMode
                obj.shouldWait = true;
            end
            v_cmd = 0;
            w_cmd = 0;
            
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
        function [obj, v_cmd, w_cmd] = performInitialMove(obj)
            v_cmd = 0;
            w_cmd = 0.5;
            obj.expectedPose = getExpectedPose(obj.previousExpectedPose, v_cmd, w_cmd, obj.sampleTime);
            if obj.expectedPose(3) > deg2rad(obj.initialTurn)
                obj.expectedPose(3) = wrapTo2Pi(obj.expectedPose(3));
                obj.isInitialMove = false;
                obj.shouldPlan = true;
            end
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
