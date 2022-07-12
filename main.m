%% Robot diferencial con lidar
% Robotica Movil - 2022 1c
close all
clear all
clc

addpath('simulador')
addpath('clases')
addpath('funciones')

verMatlab = ver('MATLAB');   % en MATLAB2020a funciona bien, ajustado para R2016b, los demas a pelearla...

c = Config();

simular_ruido_lidar = c.simular_ruido_lidar; %simula datos no validos del lidar real, probar si se la banca
use_roomba = c.use_roomba;  % false para desarrollar usando el simulador, true para conectarse al robot real



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
    

%% Creacion del entorno
load 2021_2c_tp_map.mat     %carga el mapa como occupancyMap en la variable 'map'
load likelihood_map.mat     %carga el likelihoodMap en la variable 'likelihood_map'

if verMatlab.Release=='(R2016b)'
    %Para versiones anteriores de MATLAB, puede ser necesario ajustar mapa
    imagen_mapa = 1-double(imread('imagen_2021_2c_mapa_tp.tiff'))/255;
    map = robotics.OccupancyGrid(imagen_mapa, 25);
elseif verMatlab.Release==c.matlab_version    % Completar con la version que tengan
    %Ni idea que pasa, ver si el truco R2016b funciona
    disp('ver si la compatibilidad R2016b funciona');
else
    disp(['Utilizando MATLAB ', verMatlab.Release]);
end

%% Crear sensor lidar en simulador
lidar = LidarSensor;
lidar.sensorOffset = [0,0];   % Posicion del sensor en el robot (asumiendo mundo 2D)
scaleFactor = 3;                %decimar lecturas de lidar acelera el algoritmo
num_scans = 513/scaleFactor;
hokuyo_step_a = deg2rad(-90);
hokuyo_step_c = deg2rad(90);

lidar.scanAngles = linspace(hokuyo_step_a,hokuyo_step_c,num_scans);
lidar.maxRange = 5;

%% Crear visualizacion
viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Parametros de la Simulacion

simulationDuration = c.simulation_duration;          % Duracion total [s]
sampleTime = c.sample_time;                   % Sample time [s]
initPose = c.init_pose;         % Pose inicial (x y theta) del robot simulado (el robot pude arrancar en cualquier lugar valido del mapa)

% Inicializar vectores de tiempo, entrada y pose
tVec = 0:sampleTime:simulationDuration;         % Vector de Tiempo para duracion total

%% generar comandos a modo de ejemplo
vxRef = 0.05*ones(size(tVec));   % Velocidad lineal a ser comandada
wRef = zeros(size(tVec));       % Velocidad angular a ser comandada
wRef(tVec < 5) = -0.2;
wRef(tVec >=7.5) = 0.2;

pose = zeros(3,numel(tVec));    % Inicializar matriz de pose
pose(:,1) = initPose;

%% Robot

R = 0.072/2;                % Radio de las ruedas [m]
L = 0.235;                  % Distancia entre ruedas [m]
dd = DifferentialDrive(R,L); % creacion del Simulador de robot diferencial
% Este robot es una representacion de la roomba.
dimensions = RobotDimensions;
dimensions.TotalDiameter = 0.35;
dimensions.WheelDistance = 0.235;
dimensions.WheelDiameter = 0.072;
dimensions.LidarLocation = [0, 0, 0.15];

% Inicializamos el robot en el modo que corresponda
robot = Robot.newRobot(dimensions, initPose, c.vigilancia, lidar);
% Inicializamos las particulas y las guardamos en el robot
particles = Particle.initRandomParticles(250, map, 1, lidar);
robot = robot.setParticles(particles);

%% Simulacion

if verMatlab.Release=='(R2016b)'
    r = robotics.Rate(1/sampleTime);    %matlab viejo no tiene funcion rateControl
else
    r = rateControl(1/sampleTime);  %definicion para R2020a, y posiblemente cualquier version nueva
end

for idx = 2:numel(tVec)   

    % Generar aqui criteriosamente velocidades lineales v_cmd y angulares w_cmd
    % -0.5 <= v_cmd <= 0.5 and -4.25 <= w_cmd <= 4.25
    % (mantener las velocidades bajas (v_cmd < 0.1) (w_cmd < 0.5) minimiza vibraciones y
    % mejora las mediciones.   
    v_cmd = vxRef(idx-1);   % estas velocidades estan como ejemplo ...
    w_cmd = wRef(idx-1);    %      ... para que el robot haga algo.
    
    %% COMPLETAR ACA:
        % generar velocidades para este timestep
        % fin del COMPLETAR ACA
    
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
        
        robot = robot.updateParticlesWithRanges(ranges, map, likelihood_map);
        
%         robot = robot.ActualizarParticulas(ranges, map);
%         [posEstimada, varEstimada] = robot.EstimarPosicion();
%         pose(:,idx) = posEstimada;
        
        % Fin del COMPLETAR ACA
        
    %%
    % actualizar visualizacion
    %viz(pose(:,idx),ranges)
    waitfor(r);
end




