%% Robot diferencial con lidar
% Robotica Movil - 2022 1c
close all
clear all
clc

addpath('simulador');
addpath('exploration');
addpath('planning');
addpath('controller');
addpath('utils');

%% Config
simular_ruido_lidar = false; %simula datos no validos del lidar real, probar si se la banca
use_roomba=false;  % false para desarrollar usando el simulador, true para conectarse al robot real

vigilancia = true;
exploracion = ~vigilancia;

initPose = [3;1;0];               % Pose inicial (x y theta) del robot
goals = [1.5,1.3;
         4.3,2.1];
goalIdx = 0;


%% Roomba
if use_roomba   % si se usa el robot real, se inicializa la conexion    
    rosshutdown
    pause(1)
    ipaddress_core = '192.168.0.101';
    ipaddress_local = '192.168.0.100';  %mi ip en a red TurtleNet
    setenv('ROS_IP', '192.168.0.100');
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

FigureName = 'Particles';
FigureTag = 'Particles';
particlesFig = figure('Name', FigureName, 'Tag', FigureTag);
particlesAx = axes('Parent', particlesFig);

FigureName = 'Planning';
FigureTag = 'Planning';
planningFig = figure('Name', FigureName, 'Tag', FigureTag);
planningAx = axes('Parent', planningFig);
show(map, 'Parent', planningAx);
hold(planningAx, 'on')

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

% Estas variables indican si el mcl debería reducir la cantidad de
% particulas
shouldAdjustParticles = false;
adjustedParticles = false;

% Para vigilancia
mcl = monteCarloLocalization('GlobalLocalization', true, 'ParticleLimits', [300, 4000]);
mcl.UseLidarScan = true;
mcl.MotionModel = odometryMotionModel;
mcl.SensorModel = getSensorModel(current_map, [0.05, lidar.maxRange], num_scans);
mcl.UpdateThresholds = [0.001 0.001 deg2rad(0.1)];
mcl.ResamplingInterval = 3;

% Para la exploracion
maxLidarRange = lidar.maxRange;
mapResolution = map.Resolution;
slamObj = lidarSLAM(mapResolution, maxLidarRange);
slamObj.MovementThreshold = [0.05, deg2rad(1)];
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
            [slamObj, current_map, slamPoses] = performSlam(slamObj, scans, initPose);
            estimatedPose = slamPoses(end,:);
        end
        
        % 2. Generar velocidades para este timestep 
        [controller, v_cmd, w_cmd] = controller.getSpeedCommands(estimatedPose);
        % 3. TODO: Agregar proteccion contra obstáculos (frenar en caso de
        % que se esté por chocar y girar unicamente)
        shouldStop = checkObstacles(scans);
        if shouldStop
            v_cmd = 0;
            % TODO: La direccion de giro deberia depender de dónde esté el
            % obstáculo.
            w_cmd = 0.5;
        end
        
        % 4. Planificar si es necesario
        if controller.shouldPlan
            goalIdx = goalIdx+1;
            if goalIdx > size(goals(:,1))
                % Si pasa esto es porque no hay mas goals
                break
            end
            % Obtenemos un nuevo plan dependiendo del 
            if vigilancia
                % TODO: Remover esa variable "vigilancia" (habría que hacer
                % un planning para vigilancia y otro para exploracion)
                [controller, pathWorld] = getNewPlan(controller, current_map, estimatedPose(end,:), goals(goalIdx,:), vigilancia);
            else
                [controller, pathWorld] = getNewPlan(controller, current_map, estimatedPose(end,:), goals(goalIdx,:), vigilancia);
            end
            plot(planningAx, pathWorld(:,1), pathWorld(:,2), '.b', 'MarkerSize', 5);
            
        end
        % Fin del COMPLETAR ACA
        
    %%
    % actualizar visualizacion
    viz(pose(:,idx),ranges)
    waitfor(r);
end

function shouldStop = checkObstacles(scans)
    % TODO: Este algoritmo tiene que determinar hacia que lado girar, en
    % base a las mediciones. Dónde esta el obstaculo? En frente, a la
    % derecha o a la izquierda?
    % . En frente es casi imposible, tiene que darse que hay la misma
    % cantidad de mediciones del lado derecho y del izquierdo, lo que
    % implica que el obstaculo es perfecto. Si pasara esto, girar para la
    % derecha (porque si).
    
    ranges = scans.Ranges;
    angles = scans.Angles;
    indexes = find(angles<pi/3 & angles>-pi/3);
    filteredRanges = ranges(indexes);
    filteredRanges = filteredRanges(filteredRanges < 0.25);
    if numel(filteredRanges) > 0
        shouldStop = true;
        return
    end
    shouldStop = false;
end

