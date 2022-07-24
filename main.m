%% Robot diferencial con lidar
% Robotica Movil - 2022 1c
close all
clear all
clc

addpath('simulador')
%addpath('plots')
addpath('localization')
%addpath('funciones')
addpath('planning')

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
load mapa_2022_1c.mat     %carga el mapa como occupancyMap en la variable 'map'

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
hokuyo_step_a = deg2rad(-180);
hokuyo_step_c = deg2rad(180);

lidar.scanAngles = linspace(hokuyo_step_a,hokuyo_step_c,num_scans);
lidar.maxRange = 5;

%% Crear visualizacion
viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Parametros de la Simulacion

% Inicializar likelihood field
load('likelihood_map', 'likelihood_map') % de donde sacaron esto

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

%% Filtro de particulas
localizer = CustomParticleFilter(map, lidar);


%% variables de planeamiento
goals = [1.5 1.3; 4.3 2.1];
goal_grid = world2grid_(map,goals);
goal_idx = 1;
goal_reached = false;
path_threshold = 0.5;
goal_threshold = 0.08;
path_world = 0;
sim_pos = initPose;
final_point = false;
new_plan = true;
patrullaje = true;
if(~patrullaje)
    new_plan = false;
    first_exploration_plan = true;
end
success = false;
collision = false;
n_plan = 0;
%% Simulacion

if verMatlab.Release=='(R2016b)'
    r = robotics.Rate(1/sampleTime);    %matlab viejo no tiene funcion rateControl
else
    r = rateControl(1/sampleTime);  %definicion para R2020a, y posiblemente cualquier version nueva
end

%% Gráficos
%Figura para las partículas
figureName = 'Particles';
figureTag = 'Particles';
particlesFig = figure('Name', figureName, 'Tag', figureTag);
figureName = 'planning';
figureTag = 'planning';
planningFig = figure('Name', figureName, 'Tag', figureTag);
planning_ax = axes('Parent', planningFig);
show(map, 'Parent', planning_ax);
print_planning = false;
show_planning = false;
%% Loop principal
for idx = 2:numel(tVec)   

    % Generar aqui criteriosamente velocidades lineales v_cmd y angulares w_cmd
    % -0.5 <= v_cmd <= 0.5 and -4.25 <= w_cmd <= 4.25
    % (mantener las velocidades bajas (v_cmd < 0.1) (w_cmd < 0.5) minimiza vibraciones y
    % mejora las mediciones.   
    v_cmd = vxRef(idx-1);   % estas velocidades estan como ejemplo ...
    w_cmd = wRef(idx-1);    %      ... para que el robot haga algo.
%     v_cmd = 0;
%     w_cmd = 0;
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
    viz(pose(:,idx),ranges)
    waitfor(r);
    %% COMPLETAR ACA:
        % hacer algo con la medicion del lidar (ranges) y con el estado
        % actual de la odometria ( pose(:,idx) )     
        
        % Armamos la información de odometria
        odometry = Odometry(pose(:, idx), pose(:, idx-1));
        
        % Filtramos mediciones inválidas
        ranges = ranges(1:end - 1);
        scanAngles = lidar.scanAngles(1:end - 1);
        scanAngles = scanAngles(~isnan(ranges));
        ranges = ranges(~isnan(ranges));
        
        localizer = localizer.localize(odometry, ranges);     
        localizer.plotParticles(particlesFig, pose(:, idx));
        
        [estPos, varPos] = localizer.getEstimatedPoseAndVariance();
        weights = localizer.get_weigths();
        [~, max_w] = max(weights);
        particles_max_w = localizer.get_particles(max_w);
        %[path_grid, path_world] = planning(world2grid_(map, estPos(1:2)), ...
        %    goal_grid(goal_idx, :), map, likelihood_map, particles_max_w, patrullaje, ...
        %    idx - 1, planning_ax, planningFig, print_planning, show_planning, estPos(3));
        % Fin del COMPLETAR ACA
        
    %%
    % actualizar visualizacion

end




