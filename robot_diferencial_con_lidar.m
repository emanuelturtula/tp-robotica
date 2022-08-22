%% Robot diferencial con lidar
% Robotica Movil - 2022 1c
close all
clear all
clc

addpath('simulador');
addpath('exploration');
addpath('planning');
addpath('planning/vigilance');
addpath('planning/exploration');
addpath('controller');
addpath('utils');

%% Config
simular_ruido_lidar = false; %simula datos no validos del lidar real, probar si se la banca
use_roomba=true;  % false para desarrollar usando el simulador, true para conectarse al robot real

vigilancia = true;
exploracion = ~vigilancia;

initPose = [2;1;0];               % Pose inicial (x y theta) del robot
initPoseUpdated = false;
goals = [1.5,1.3;
         4.3,2.1];
goalIdx = 0;

%% Roomba
if use_roomba   % si se usa el robot real, se inicializa la conexion    
    rosshutdown
    pause(1)
    ipaddress_core = '192.168.0.102';
    ipaddress_local = '192.168.0.101';  %mi ip en a red TurtleNet
    setenv('ROS_IP', '192.168.0.101');
    setenv('ROS_MASTER_URI', ['http://', ipaddress_core, ':11311']);
    rosinit(ipaddress_core,11311, 'NodeHost', ipaddress_local)
    pause(1)
    laserSub = rossubscriber('/scan');
    odomSub = rossubscriber('/odom');
    cmdPub = rospublisher('/auto_cmd_vel', 'geometry_msgs/Twist');
    pause(1) % Esperar a que se registren los canales
    cmdMsg = rosmessage(cmdPub);
end
    

%% Definicion del robot (disco de diametro = 0.35m)
R = 0.072/2;                % Radio de las ruedas [m]
L = 0.235;                  % Distancia entre ruedas [m]
dd = DifferentialDrive(R,L); % creacion del Simulador de robot diferencial

%% Creacion del entorno
load mapa_2022_1c.mat     %carga el mapa como occupancyMap en la variable 'map'

%% Crear sensor lidar en simulador
lidar = LidarSensor;
lidar.sensorOffset = [0,0];   % Posicion del sensor en el robot (asumiendo mundo 2D)
scaleFactor = 3;                %decimar lecturas de lidar acelera el algoritmo
num_scans = 513/scaleFactor;
hokuyo_step_a = deg2rad(-90);
hokuyo_step_c = deg2rad(90);

lidar.scanAngles = linspace(hokuyo_step_a,hokuyo_step_c,num_scans);
lidar.maxRange = 5;

%% Crear visualizaciones
viz = Visualizer2D();
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

if vigilancia
    FigureName = 'Particles';
    FigureTag = 'Particles';
    particlesFig = figure('Name', FigureName, 'Tag', FigureTag);
    particlesAx = axes('Parent', particlesFig);

    FigureName = 'Planning';
    FigureTag = 'Planning';
    planningFig = figure('Name', FigureName, 'Tag', FigureTag);
    planningAx = axes('Parent', planningFig);
else    
    FigureName = 'Exploration Map';
    FigureTag = 'ExplorationMap';
    explorationFig = figure('Name', FigureName, 'Tag', FigureTag);
    explorationAx = axes('Parent', explorationFig);
end

%% Parametros de la Simulacion
simulationDuration = 3*60;          % Duracion total [s]
sampleTime = 0.1;                   % Sample time [s]

% Inicializar vectores de tiempo, entrada y pose
tVec = 0:sampleTime:simulationDuration;         % Vector de Tiempo para duracion total

%% Matriz de poses
pose = zeros(3,numel(tVec));    % Inicializar matriz de pose
pose(:,1) = initPose;

%% Simulacion
r = rateControl(1/sampleTime);  %definicion para R2020a, y posiblemente cualquier version nueva

%% Determinar el algortimo de localizacion dependiendo del ejercicio

% Iniciamos con un mapa. Si es el modo exploracion, esto va a ser
% reemplazado en cada iteracion
current_map = map;
shouldPlanAfterObstaclePassed = false;

% Para vigilancia
mcl = monteCarloLocalization('GlobalLocalization', true, 'ParticleLimits', [300, 4000]);
mcl.UseLidarScan = true;
mcl.MotionModel = odometryMotionModel;
mcl.SensorModel = getSensorModel(current_map, [0.05, lidar.maxRange], num_scans);
mcl.UpdateThresholds = [0.001 0.001 deg2rad(0.1)];
mcl.ResamplingInterval = 3;

% Para la exploracion
maxLidarRange = lidar.maxRange;
mapResolution = 25;
slamObj = lidarSLAM(mapResolution, maxLidarRange);
slamObj.MovementThreshold = [0.2, deg2rad(20)];
slamObj.LoopClosureThreshold = 100;
%slamObj.LoopClosureSearchRadius = 1;

v_cmd = 0;
w_cmd = 0;

% Controlador. Sirve tanto para exploracion como para vigilancia
controller = Controller(initPose', sampleTime, exploracion);

%% Loop principal
for idx = 2:numel(tVec)       
    %% a partir de aca el robot real o el simulador ejecutan v_cmd y w_cmd:
    if use_roomba       % para usar con el robot real
        % Enviar comando de velocidad
        cmdMsg.Linear.X = v_cmd;
        cmdMsg.Angular.Z = w_cmd;
        send(cmdPub,cmdMsg);
        
        % Recibir datos de lidar y odometría
        scanMsg = receive(laserSub);
        odompose = odomSub.LatestMessage;
        
        % Obtener vector de distancias del lidar
        ranges_full = laserSub.LatestMessage.Ranges;
        ranges = ranges_full(1:scaleFactor:end);
        %ranges = circshift(ranges,length(ranges)/2);  % verificar
        ranges(ranges==0)=NaN; % lecturas erroneas y maxrange
        % Obtener pose del robot [x,y,yaw] de datos de odometría (integrado por encoders).
        odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
        odomRotation = quat2eul(odomQuat);
        pose(:,idx) = [odompose.Pose.Pose.Position.X + initPose(1); odompose.Pose.Pose.Position.Y+ initPose(2); odomRotation(1)];
    else        % para usar el simulador
        % Mover el robot segun los comandos generados
        [wL,wR] = inverseKinematics(dd,v_cmd,w_cmd);
        % Velocidad resultante
        [v,w] = forwardKinematics(dd,wL,wR);
        velB = [v;0;w]; % velocidades en la terna del robot [vx;vy;w]
        vel = bodyToWorld(velB,pose(:,idx-1));  % Conversion de la terna del robot a la global
        % Realizar un paso de integracion
        pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
        % Tomar nueva medicion del lidar
        ranges = lidar(pose(:,idx));
        if simular_ruido_lidar
            % Simular ruido de un lidar ruidoso (probar a ver si se la banca)
            chance_de_medicion_no_valida = 0.17;
            not_valid=rand(length(ranges),1);
            ranges(not_valid<=chance_de_medicion_no_valida)=NaN;
        end
    end
    %%
    % Aca el robot ya ejecutó las velocidades comandadas y devuelve en la
    % variable ranges la medicion del lidar para ser usada y
    % en la variable pose(:,idx) la odometría actual.
    
    %% COMPLETAR ACA:
        % hacer algo con la medicion del lidar (ranges) y con el estado
        % actual de la odometria ( pose(:,idx) )
        scans = lidarScan(ranges, lidar.scanAngles);
        % TODO: parametrizar la siguiente linea
        scans = removeInvalidData(scans,'RangeLimits',[0.1 5]);
        if vigilancia
            [isUpdated, estimatedPose, covariance] = mcl(pose(:,idx), scans);
            if isUpdated
                particles = getParticles(mcl);
                show(map, 'Parent', particlesAx);
                hold(particlesAx, 'on');
                plot(particlesAx, particles(:,1),particles(:,2), '.b', 'MarkerSize', 5);
                hold(particlesAx, 'off');
            end
            if controller.finishedInitialTurn && ~initPoseUpdated
                initPoseUpdated = true;
                initPose = estimatedPose;               
            end
                
            if controller.shouldWait
                % Cuando nos dice que esperemos, es porque llegamos al
                % objetivo. Hay que esperar 3 segundos
                pause(3);
                controller.shouldWait = false;
            end
            
        end
        
        if exploracion
            % Usamos lidarSlam para localizarnos y mapear simultaneamente
            % El mapa generado lo usamos para planificar más adelante
            [slamObj, ~, slamPoses] = performSlam(slamObj, scans);
            estimatedPose = slamPoses(end,:);
            if idx > switchFollowRangesSideThreshold && controller.followLeftRanges
                controller.followLeftRanges = false;
            end
        end
        
        % 2. Generar velocidades para este timestep 
        [controller, v_cmd, w_cmd] = controller.getSpeedCommands(estimatedPose, scans);
            
        % 3. Planificar si es necesario
        if controller.shouldPlan
            % Obtenemos un nuevo plan dependiendo del modo
            if vigilancia
                goalIdx = goalIdx+1;
                if goalIdx > size(goals(:,1))
                    % Si pasa esto es porque no hay mas goals
                    break
                end
                % TODO: Remover esa variable "vigilancia" (habría que hacer
                % un planning para vigilancia y otro para exploracion)
                [controller, pathWorld] = getNewPlanForVigilance(controller, current_map, estimatedPose(end,:), goals(goalIdx,:));
                show(current_map, 'Parent', planningAx);
                hold(planningAx, 'on')
                plot(planningAx, pathWorld(:,1), pathWorld(:,2), '.b', 'MarkerSize', 5);
            end
        end
        % Fin del COMPLETAR ACA
        
        if exploracion && rem(idx, 10) == 0
            [scans, poses]  = scansAndPoses(slamObj);
            exploredMap = buildMap(scans, poses, slamObj.MapResolution, slamObj.MaxLidarRange);
            hold(explorationAx, 'on');
            show(exploredMap, 'Parent', explorationAx);
            show(slamObj.PoseGraph, 'Parent', explorationAx, 'IDs', 'off');
            hold(explorationAx, 'off');
        end
    %%
    % actualizar visualizacion
    
    viz(pose(:,idx),ranges)
    waitfor(r);
end
