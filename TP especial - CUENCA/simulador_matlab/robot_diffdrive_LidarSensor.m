%% Robot diferencial con lidar
% Robotica Movil - 2020
close all
clear all

verMatlab= ver('MATLAB');   % en MATLAB2020a funciona bien, ajustado para R2016b, los demas a pelearla...

use_roomba=false;  % false para desarrollar usando el simulador, true para robot real

%% Roomba
if use_roomba   % si se usa el robot real, se inicializa el sistema
    rosshutdown
    pause(1)
    ipaddress = '10.42.0.1';
    ipaddress_local = '10.42.0.184';
    setenv('ROS_IP', '10.42.0.184');
    setenv('ROS_MASTER_URI', ['http://', ipaddress, ':11311']);
    rosinit(ipaddress,11311, 'NodeHost', ipaddress_local)
    pause(2)
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
load tp_map.mat     %carga el mapa como occupancyMap en la variable 'map'


if verMatlab.Release=='(R2016b)'
    %Para versiones anteriores de MATLAB, puede ser necesario ajustar mapa
    imagen_mapa = 1-double(imread('imagen_mapa_tp.tiff'))/255;
    map = robotics.OccupancyGrid(imagen_mapa, 25);
elseif verMatlab.Release=='(R2017a)'    % Completar con la version que tengan
    %Ni idea que pasa
    disp('ver si la compatibilidad R2016b funciona');
else
    disp(['Utilizando MATLAB ', verMatlab.Release]);
end

% Crear sensor lidar
lidar = LidarSensor;
lidar.sensorOffset = [.09,0];   % Posicion del sensor en el robot (asumiendo mundo 2D)
scaleFactor = 5;                %decimar lecturas de lidar acelera el algoritmo
num_scans = 720/scaleFactor;
lidar.scanAngles = linspace(-pi,pi,num_scans);
lidar.maxRange = 8;

% Crear visualizacion
viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Parametros de la Simulacion

simulationDuration = 10;%3*60;         % Duracion total [s]
sampleTime = 0.1;               % Sample time [s]
initPose = [2.5; 2; pi];        % Pose inicial (x y theta)

% Inicializar vectores de tiempo, entrada y pose
tVec = 0:sampleTime:simulationDuration;         % Vector de Tiempo (maximo 3 minutos)

%generar comandos a modo de ejemplo
vxRef = 0.05*ones(size(tVec));   % Velocidad angular a comandar
wRef = zeros(size(tVec));       % Velocidad angular a comandar
wRef(tVec < 5) = -0.2;
wRef(tVec >=7.5) = 0.2;

pose = zeros(3,numel(tVec));    % Inicializar matriz de pose
pose(:,1) = initPose;

%% Simulacion

if verMatlab.Release=='(R2017a)'
    r = robotics.Rate(1/sampleTime);
else
    r = rateControl(1/sampleTime);  %definicion para R2020a, y posiblemente algun otro de los nuevos
end

for idx = 2:numel(tVec)   

    % Generar aqui criteriosamente velocidades lineales y angulares
    % -0.5 <= linear.x <= 0.5 and -4.25 <= angular.z <= 4.25
    % (mantener las velocidades bajas (v < 0.1) minimiza vibraciones y
    % mejora las mediciones.   
    v_cmd = vxRef(idx-1);
    w_cmd = wRef(idx-1);
    
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
        % Obtener pose del robot [x,y,yaw] de datos de odometría (integrado por encoders).
        odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
        odomRotation = quat2eul(odomQuat);
        pose(:,idx) = [odompose.Pose.Pose.Position.X + initPose(1); odompose.Pose.Pose.Position.Y+ initPose(2); odomRotation(1)+initPose(3)];
    
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
        
    end
    % actualizar visualizacion
    viz(pose(:,idx),ranges)
    waitfor(r);
end

