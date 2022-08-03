%% Robot diferencial con lidar

% Robotica Movil - 2020
close all
clear all
clc

use_roomba = false; % false para desarrollar usando el simulador, true para robot real

activate_rate = true; % true para mantener el loop principal con la misma
% frecuencia que el sampleo

patrullaje = false; % true para el ejercicio de patrullaje, false para el
% de exploración

addpath('librobotics');
addpath('planning');
addpath('particles');
addpath('simulator');

%% Roomba
if(use_roomba)   % Si se usa el robot real, se inicializa el sistema
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

%% Definicion del robot (disco de diametro = 0.35 m)

R = 0.072/2;                    % Radio de las ruedas [m]
L = 0.235;                      % Distancia entre ruedas [m]
dd = DifferentialDrive(R, L);    % Creación del Simulador de robot diferencial
Rt = 0.35/2;                    % Radio del robot [m]
v_perm = [-0.5, 0.5];            % Velocidades lineales permitidas [m/s]
v_rec = [-0.1, 0.1];             % Velocidades lineales recomendadas [m/s]
w_perm = [-4.25, 4.25];          % Velocidades angulares permitidas [rad/s]
w_rec = [-0.1, 0.1];             % Velocidades angulares recomendadas [rad/s]
lid_range = [0.1, 10];           % Rango del lidar
lid_div = 144;                  % Divisiones máximas del lidar

%% Creacion del entorno

load mapa_2022_1c.mat      % Carga el mapa como occupancyMap en la variable 'map'

% Mapa resultante:
%         OccupiedThreshold: 0.6500
%             FreeThreshold: 0.2000
%     ProbabilitySaturation: [0.0010 0.9990]
%                  GridSize: [141 131] ([y x])
%                Resolution: 25
%              XWorldLimits: [0 5.2400]
%              YWorldLimits: [0 5.6400]
%       GridLocationInWorld: [0 0]

% Crear sensor lidar
lidar = LidarSensor;
lidar.sensorOffset = [.09, 0];   % Posición del sensor en el robot (asumiendo mundo 2D) - z = 0.18;
lidar.sensorAngleOffset = pi;   % Ángulo del sensor respecto a la dirección del robot
scaleFactor = 20;               % Decimar lecturas de lidar acelera el algoritmo
num_scans = 720/scaleFactor;    % 144
lidar.scanAngles = -linspace(-pi, pi, num_scans); % El lidar barre en sentido horario
lidar.minRange = lid_range(1);
lidar.maxRange = lid_range(2);

if(~use_roomba)
    % Crear visualización
    viz = Visualizer2D;
    viz.mapName = 'map';
    attachLidarSensor(viz, lidar);
    % Se agregó el radio del robot para graficarlo con su tamaño real
    setRobotRadius(viz, Rt);
end

%% Parametros de la Simulacion
simulationDuration = 180;       % Duracion total [s]
sampleTime = 0.1;               % Sample time [s]

% Inicializar likelihood field
load('likelihood_map', 'likelihood_map')

initPose = [2, 2, 0];

clear particles;

% Exploración o patrullaje
if(patrullaje)
    exploration = false;
else
    exploration = true;
end

if(patrullaje)
    % Mapa conocido
    size_x = map.GridSize(2);
    size_y = map.GridSize(1);
    current_map = map;
    
    % Inicializar partículas
    n_particles_init = 2500;
    n_particles = 250;
    n_angle = 10;

    [aux_grid_x, aux_grid_y] = find(map.occupancyMatrix > p_occupied);
    N = size(aux_grid_x, 1);

    particles(1:n_particles_init) = particle(map.occupancyMatrix, initPose_particles, N);
    x = initialize_particles(n_particles_init, map, n_angle);
    aux_x = num2cell(x(:, 1));
    aux_y = num2cell(x(:, 2));
    aux_t = num2cell(x(:, 3));
    [particles(:).pose_x] = deal(aux_x{:});
    [particles(:).pose_y] = deal(aux_y{:});
    [particles(:).pose_t] = deal(aux_t{:});
    for k = 1:n_particles_init
        particles(k).occupied_grid_x = aux_grid_y;
        particles(k).occupied_grid_y = aux_grid_x;
        particles(k).n_occupied_grid = N;
    end
    particles = particles(1:n_particles_init);
end

%%%%%% Exploración
if(exploration)    
    % Mapa desconocido
    size_y = 160;
    size_x = 160;
    u_map_matrix = 0.5*ones(size_y, size_x);
    u_map = robotics.OccupancyGrid(u_map_matrix, map.Resolution);
    current_map = u_map;
    
    % Pose inicial fija
    initPose_particles = [size_x/(2*u_map.Resolution), size_y/(2*u_map.Resolution), 0];
    
    % Probabilidad de ocupación
    p_occupied = 0.8;
    l_occupied = log(p_occupied/(1 - p_occupied));
    
    % Inicializar partículas
    n_particles = 75;
    N = 1000;
    particles(1:n_particles) = particle(u_map_matrix, initPose_particles, N);
    particles = particles(1:n_particles);
end

particles = particles';

% Vector de tiempo
tVec = 0:sampleTime:simulationDuration;         

% Matriz de pose
pose = zeros(3, numel(tVec));
pose(:, 1) = initPose;

% Varianza muestral de las partículas y variables de localización
var_pos = [inf, inf, inf];
var_pos_threshold = 0.1;
first_var_pos_threshold = var_pos_threshold/5;
var_angle_threshold = 0.1*pi;
localized = false;
first_localized = false;

% Matrices que describen el camino planeado
previous_x = zeros(size_x, size_y) - 1;
previous_y = zeros(size_x, size_y) - 1;

% Variables de objetivos y planeamiento
goals = [1.5 1.3;
        4.3 2.1];

goal_grid = world2grid_tito(current_map, goals);
goal_idx = 1;
goal_reached = false;
path_threshold = 0.5;
goal_threshold = 0.08;
path_world = 0;
sim_pos = initPose;
final_point = false;
new_plan = true;
if(~patrullaje)
    new_plan = false;
    first_exploration_plan = true;
end
success = false;
collision = false;
n_plan = 0;

% Variables de velocidades
v_cmd = 0;
if(~patrullaje)
    v_cmd = 0.05;
end
w_cmd = 0;

% Protección
angle_div = 2*pi/(length(lidar.scanAngles) - 1);
pastel = 2/3*pi;
pastel_ancho = pastel;
pastel_angosto = pastel;
dist_min = Rt + 0.1;
dist_min_ancho = dist_min;
dist_min_angosto = dist_min;
if(~patrullaje)
    pastel_ancho = pi;
    pastel_angosto = pi/4;
    dist_min_ancho = Rt + 0.05;
    dist_min_angosto = Rt + 0.4;
end
pastel_n_ancho = ceil(pastel_ancho/angle_div);
pastel_n_angosto = ceil(pastel_angosto/angle_div);
stop_and_turn = false;

% Índices y límites de decisiones de control
lost_idx = 0;
lost_limit = 10;
if(~patrullaje)
    lost_limit = 20;
end
active_loc = false;
loc_idx = 0;
loc_limit = 10;
new_plan_idx = 0;
new_plan_limit = 15;

% Mediciones reducidas
ranges = [];
red_ranges = [];

%% PLOTS

% Activar gráficos
show_planning = true;
print_planning = false;
show_particles = true;
print_particles = false;
show_viz = true;
print_viz = false;
show_mapping = true;
print_mapping = false;
print_final_mapping = false;

%%%%%%%%%%%%% NO TOCAR %%%%%%%%%%%%%
if(use_roomba)
    show_planning = false;
    print_planning = false;
    show_particles = false;
    print_particles = false;
    show_viz = false;
    if(patrullaje)
        show_viz = true;
    end
    print_viz = false;
    if(~patrullaje)
        show_mapping = true;
    end
    print_mapping = false;
    if(~patrullaje)
        print_final_mapping = true;
    end
end

% Figura para el planeamiento
FigureName = 'Path Planning';
FigureTag = 'PathPlanning';
planning_fig = figure('Name', FigureName, 'Tag', FigureTag);
planning_ax = axes('Parent', planning_fig);
show(current_map, 'Parent', planning_ax);
if(~show_planning)
    set(planning_fig, 'Visible', 'Off');
end
if(print_planning)
    delete plots/planning/*.png
end
% ffmpeg -r 10 -i planning_%03d.png planning.mp4

% Figura para las partículas
FigureName = 'Particles';
FigureTag = 'Particles';
particles_fig = figure('Name', FigureName, 'Tag', FigureTag);
particles_ax = axes('Parent', particles_fig);
delete plots/*.png
if(~show_particles)
    set(particles_fig, 'Visible', 'Off');
end
particle_viz = particleViz;
hold(particles_ax, 'on')
show(current_map, 'Parent', particles_ax);
pose_x_fig = [particles(:).pose_x]';
pose_y_fig = [particles(:).pose_y]';
particle_viz.handle_est = plot(particles_ax, 0, 0, 'xg');
particle_viz.handle_particles = plot(particles_ax, pose_x_fig, pose_y_fig, '.b');
particle_viz.handle_pose = plot(particles_ax, 0, 0, 'xr');
particle_viz.handle_lidar = plot(particles_ax, 0, 0, '-g');
particle_viz.handle_robot = plot(particles_ax, 0, 0, '-g');
if(print_particles)
    delete plots/particles/*.png
end
hold(particles_ax, 'off')
% ffmpeg -r 10 -i particles_%03d.png particles.mp4

% Figura de la trayectoria y el lidar
if(~use_roomba && (show_viz || print_viz))
    viz(pose(:, 1), ranges)
    FigureTag = 'RobotVisualization';
    viz_fig = findobj('Type', 'Figure', 'Tag', FigureTag);
    if(~show_viz)
        set(viz_fig, 'Visible', 'Off');
    end
    if(print_viz)
        delete plots/viz/*.png
    end
    if(print_viz)
        filename = sprintf('plots/viz/viz_%03d.png', 0);
        print(viz_fig, filename, '-dpng');
    end
end
% ffmpeg -r 10 -i viz_%03d.png trayectoria.mp4

% Figura para el mapa
FigureName = 'Mapping';
FigureTag = 'Mapping';
mapping_fig = figure('Name', FigureName, 'Tag', FigureTag);
mapping_ax = axes('Parent', mapping_fig);
colormap('gray');
if(~show_mapping)
    set(mapping_fig, 'Visible', 'Off');
end
if(print_mapping)
    delete plots/mapping/*.png
end
% ffmpeg -r 10 -i mapping_%03d.png mapping.mp4

% Figura para el mapa finales
FigureName = 'Final Mapping';
FigureTag = 'FinalMapping';
final_mapping_fig = figure('Name', FigureName, 'Tag', FigureTag);
final_mapping_ax = axes('Parent', final_mapping_fig);
set(final_mapping_fig, 'Visible', 'Off');
if(print_final_mapping)
    delete plots/final_mapping/*.png
end

% pause(3)

%% Simulación

% Número de iteración del loop principal
idx = 1;

r = robotics.Rate(1/sampleTime);

% Si se cumplieron los objetivos, chocó, o se acaba el tiempo, el loop termina
while(~success && ~collision && idx <= numel(tVec))
    idx = idx + 1;
    
    % Imprime para confirmar que está iterando
    if(~use_roomba)
        fprintf('.');
        if mod(idx, 10) - 1 == 0
            fprintf('\n')
        end
    end
    
    % Primera localización
    if(first_localized)
        localized = ~(max(var_pos(1:2)) > var_pos_threshold || var_pos(3) > var_angle_threshold);
        particles = particles(1:n_particles);
    else
        first_localized = ~(max(var_pos(1:2)) > first_var_pos_threshold || var_pos(3) > var_angle_threshold);
        if(patrullaje)    
            x = [particles(:).pose_x; particles(:).pose_y; particles(:).pose_t]';
            noise = normrnd(0, 0.1, n_particles_init, 3);
            x = x + noise;
            aux_x = num2cell(x(:, 1));
            aux_y = num2cell(x(:, 2));
            aux_t = num2cell(x(:, 3));
            [particles(:).pose_x] = deal(aux_x{:});
            [particles(:).pose_y] = deal(aux_y{:});
            [particles(:).pose_t] = deal(aux_t{:});
        end
    end
    
    % Si no está localizado busca avanzar hacia un lugar libre según la
    % información del lidar
    if(~localized || length(path_world(:,1)) == 1)
        if(lost_idx > 0)
            lost_idx = lost_idx + 1;
        end
        if(lost_idx > lost_limit)
            [v_cmd, w_cmd] = avoid_walls(ranges, lidar);
            if(patrullaje)
                new_plan = true;
            else
                active_loc = true;
                path_world = 0;
                loc_idx = 0;
            end
        end
    else
        if(~patrullaje && active_loc)
            if(loc_idx < loc_limit)
                loc_idx = loc_idx + 1;
            else
                new_plan = true;
                lost_idx = 1;
                stop_and_turn = false;
            end
        end
        % Si alcanzó el objetivo pasa al siguiente
        if(patrullaje && (norm(est_pos(1:2) - goals(goal_idx,:)) < goal_threshold))
            goal_reached = true;
            goal_idx = goal_idx + 1;
            % Si alcanzó todos los objetivos, termina el programa
            if(goal_idx == 4)
                success = true;
                break;
            end
            v_cmd = 0;
            w_cmd = 0;
            final_point = false;
        else
            % Si se desvió demasiaod del camino vuelve a planear
            if(norm(sim_pos(1:2) - est_pos(1:2)) > path_threshold)
                lost_idx = lost_idx + 1;
            else
                lost_idx = 1;
            end
            % Si alcanzó el objetivo, o alcanzó el final del camino (o se
            % localizó por primera vez), planea el camino al siguiente
            % objetivo. Si está perdido, vuelve a planear
            if(goal_reached || lost_idx > lost_limit || new_plan || final_point) % no estamos considerando el desvío del ángulo
                v_cmd = 0;
                w_cmd = 0;
                lost_idx = 1;
            % En otro caso, continua avanzando hacia el camino, y mantiene
            % un camino simulado en base a los comandos
            elseif(length(path_world(:, 1)) > 1)
                [v_cmd, w_cmd, final_point, stop_and_turn] = controller(path_grid, path_world, sim_pos, world2grid_tito(current_map, est_pos(1:2)), ...
                    est_pos(3), patrullaje || stop_and_turn, norm_xy_d);
                sim_pos = sim_pos + [v_cmd*sampleTime*cos(sim_pos(3)), v_cmd*sampleTime*sin(sim_pos(3)), w_cmd*sampleTime];
            elseif(~use_roomba)
                fprintf("no se encontró un camino válido");
            end
        end
    end
    
    % Inicialización de patrullaje con muchas partículas
    if(~first_localized && patrullaje)
        v_cmd = 0;
        w_vmd = 0;
    end
    % Protección
    % Frena si está por chocar
    if(~isempty(red_ranges))
        norm_xy_d = inf;
        for i = -ceil((pastel_n_ancho - 1)/2):floor((pastel_n_ancho - 1)/2)
            j = mod(i - 1, length(red_ranges) - 1) + 1;
            x_d = norm([lidar.sensorOffset(1), lidar.sensorOffset(2)]) + red_ranges(j)*cos(scanAngles(j) - lidar.sensorAngleOffset);
            y_d = red_ranges(j)*sin(scanAngles(j) - lidar.sensorAngleOffset);
            norm_xy_d = min([norm([x_d, y_d]), norm_xy_d]);
        end
        if(v_cmd ~= 0 && norm_xy_d < dist_min_ancho)
            v_cmd = 0;
            w_cmd = 0;
            if(patrullaje)
                new_plan = true;
            else
                stop_and_turn = true;
                new_plan = true;
            end
        end
    else
        norm_xy_d = 0;
    end
    
    % Primera acción de exploración
    % Si está explorando y está avanzando hacia una pared por mucho tiempo
    % vuelve a planear
    if(exploration)
        if(~isempty(red_ranges))
            norm_xy_d_exp = inf;
            for i = -ceil((pastel_n_angosto - 1)/2):floor((pastel_n_angosto - 1)/2)
                j = mod(i - 1, length(red_ranges) - 1) + 1;
                x_d_exp = norm([lidar.sensorOffset(1), lidar.sensorOffset(2)]) + red_ranges(j)*cos(scanAngles(j) - lidar.sensorAngleOffset);
                y_d_exp = red_ranges(j)*sin(scanAngles(j) - lidar.sensorAngleOffset);
                norm_xy_d_exp = min([norm([x_d_exp, y_d_exp]), norm_xy_d_exp]);
            end
            if(norm([x_d, y_d]) < dist_min_angosto && w_cmd == 0 && ~stop_and_turn)
                if(first_exploration_plan)
                    v_cmd = 0;
                    w_cmd = 0;
                    new_plan = true;
                else
                    new_plan_idx = new_plan_idx + 1;
                    if(new_plan_idx > new_plan_limit)
                        new_plan = true;
                        new_plan_idx = 0;
                        v_cmd = 0;
                    end
                end
            else
                new_plan_idx = 0;
            end
        end
    end
    
    % Comandos, odometría, y mediciones del lidar
    % Para usar con el robot
    if(use_roomba)
        % Enviar comando de velocidad
        cmdMsg.Linear.X = v_cmd;
        cmdMsg.Angular.Z = w_cmd;
        send(cmdPub, cmdMsg);
        
        % Recibir datos de lidar y odometría
        scanMsg = receive(laserSub);
        odompose = odomSub.LatestMessage;
        
        % Obtener vector de distancias del lidar
        ranges_full = laserSub.LatestMessage.Ranges;
        ranges = ranges_full(1:scaleFactor:end);
        
        % Obtener pose del robot [x,y,yaw] de datos de odometría (integrado por encoders)
        odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
        odomRotation = quat2eul(odomQuat);
        pose(:, idx) = [odompose.Pose.Pose.Position.X + initPose(1); odompose.Pose.Pose.Position.Y + initPose(2); odomRotation(1) + initPose(3)];
    % Para usar el simulador
    else
        % Mover el robot segun los comandos generados
        [wL, wR] = inverseKinematics(dd, v_cmd, w_cmd);
        
        % Velocidad resultante, pasadas a la terna del robot, y luego a la
        % global
        [v, w] = forwardKinematics(dd, wL, wR);
        velB = [v; 0; w];
        vel = bodyToWorld(velB, pose(:, idx - 1));
        
        % Realizar un paso de integracion
        pose(:, idx) = pose(:, idx - 1) + vel*sampleTime;
        % Tomar nueva medicion del lidar
        ranges = lidar(pose(:, idx));
        
        % Chequear que el robot no haya chocado
        collision = checkchoq(pose(1:2, idx)', map, likelihood_map);%%%%%%%%%%%%%%%%%%%%%% cambiar el mapa si usamos otro mapa para descubrir
    end
    
    % Nos quedamos con las mediciones válidas
    ranges = ranges(1:end - 1);
    scanAngles = lidar.scanAngles(1:end - 1);
    scanAngles = scanAngles(~isnan(ranges));
    red_ranges = ranges(~isnan(ranges));

    % Actualización del mapa y obtención de la proyección de las mediciones
    % para cada partícula
    for k = 1:length(particles)
        theta = particles(k).pose_t;
        offsetVec = [cos(theta) -sin(theta);
                     sin(theta)  cos(theta)]*lidar.sensorOffset';

        sensorLoc = [particles(k).pose_x, particles(k).pose_y] + offsetVec';
        sensorPose = [sensorLoc, theta + lidar.sensorAngleOffset];

        x_z = sensorPose(1) + red_ranges.*cos(sensorPose(3) + scanAngles');
        y_z = sensorPose(2) + red_ranges.*sin(sensorPose(3) + scanAngles');
        
        valid_ranges = logical((current_map.XWorldLimits(1) < x_z).*(current_map.XWorldLimits(2) > x_z).*(current_map.YWorldLimits(1) < y_z).*(current_map.YWorldLimits(2) > y_z));
        particles(k).lidar_x = x_z(valid_ranges);
        particles(k).lidar_y = y_z(valid_ranges);
        
        %%%%%% Exploración
        if(exploration && ~stop_and_turn)
            particles(k).l_map = occupancy_grid_mapping(particles(k), current_map, red_ranges, scanAngles);
            [aux_x, aux_y] = find(particles(k).l_map > l_occupied);
            particles(k).n_occupied_grid = size(aux_x, 1);
            if(size(aux_x, 1) > particles(k).N_occupied_grid)
                particles(k).occupied_grid_x = ones(particles(k).N_occupied_grid + N, 1);
                particles(k).occupied_grid_y = ones(particles(k).N_occupied_grid + N, 1);
                particles(k).N_occupied_grid = particles(k).N_occupied_grid + N;
            end
            particles(k).occupied_grid_y(1:particles(k).n_occupied_grid) = aux_x;
            particles(k).occupied_grid_x(1:particles(k).n_occupied_grid) = aux_y;
        end
    end
    
    pose(3, idx) = wrapToPi(pose(3, idx));
    
    % Odometría: dtr - dr1 - dr2
    dtr = norm(pose(1:2, idx) - pose(1:2, idx - 1));
    dr1 = atan2(pose(2, idx) - pose(2, idx - 1), pose(1, idx) - pose(1, idx - 1)) - pose(3, idx - 1);
    dr2 = wrapToPi(pose(3, idx) - pose(3, idx - 1) - dr1);
    odo = [dtr, dr1, dr2];
    
    % Obtención de las nuevas partículas y sus pesos
    new_particles = sample_motion_model(odo, particles, use_roomba);
    new_particles = measurement_model(new_particles, current_map, ranges, likelihood_map, patrullaje);
    
    %%%%%%% Exploración
    % Gráfico del mapa de la partícula de mayor peso
    if(show_mapping && isvalid(mapping_fig))
        weights = [particles(:).weight];
        [~, max_w] = max(weights);
        particles(max_w).map = 1 - 1./(1 + exp(particles(max_w).l_map));
        imagesc(1 - particles(max_w).map, 'Parent', mapping_ax);
        axis(mapping_ax, 'equal');
        xlim(mapping_ax, [1, size_x]);
        if(print_mapping)
            filename = sprintf('plots/mapping/mapping_%03d.png', idx - 1);
            print(mapping_fig, filename, '-dpng');
        end
    end
    
    % Remuestreo de partículas
    particles = resample(new_particles);
    
    % Estimación de la posición con su varianza muestral
    [est_pos, var_pos] = estimated_position(new_particles);
    
    %%%%%%% Patrullaje
    % Gráfico de las partículas
    if(~use_roomba && (show_particles || print_particles))
        plot_state(new_particles, pose(1:2, idx), est_pos, idx - 1, particles_ax, particles_fig, print_particles, particle_viz);
    end
    
    % Actualizar visualización
    if(~use_roomba && (show_viz || print_viz))
        viz(pose(:, idx), ranges) % idx
        if(print_viz)
            filename = sprintf('plots/viz/viz_%03d.png', idx - 1);
            print(viz_fig, filename, '-dpng');
        end
    end

    % Nuevo plan
    if(localized && (goal_reached || lost_idx > lost_limit || new_plan || final_point))
        sim_pos = est_pos;
        final_point = false;
        lost_idx = 1;
        new_plan = false;
        v_cmd = 0;
        w_cmd = 0;
        first_exploration_plan = false;
        n_plan = n_plan + 1;
        weights = [particles(:).weight];
        [~, max_w] = max(weights);
        [path_grid, path_world] = planning(world2grid_tito(current_map, est_pos(1:2)), ...
            goal_grid(goal_idx, :), current_map, likelihood_map, particles(max_w), patrullaje, ...
            idx - 1, planning_ax, planning_fig, print_planning, show_planning, est_pos(3));
    end

    % Frena el robot por 3 segundos cuando llega a un objetivo
    if(goal_reached)
        goal_reached = false;
        pause(3);
        if(~use_roomba)
            fprintf("\n goal %i reached in t = %g\n", goal_idx - 1, tVec(idx - 1));
        end
    end
    
    if(activate_rate)
        waitfor(r);
    end
end

idx = idx - 1;

if(goal_reached)
    goal_reached = false;
    if(~use_roomba)
        fprintf("\n goal %i reached in t = %g\n", goal_idx - 1, tVec(idx));
    end
end

% Imprimir la pose final y una indicación si el robot chocó
if(~use_roomba)
    fprintf("\n pose final: [%g, %g] \n", pose(1, idx), pose(2, idx))
    fprintf("\n número de planes: %i", n_plan)
    fprintf("\n tiempo simulado final: %g \n", tVec(idx))
    if collision
        fprintf("\n SIGUEN CHOCANDO \n")
    end
    if(activate_rate)
        fprintf("\n tiempo real total = %g \n", r.TotalElapsedTime);
    end
end

weights = [particles(:).weight];
[~, max_w] = max(weights);
particles(max_w).map = 1 - 1./(1 + exp(particles(max_w).l_map));
if(print_final_mapping && isvalid(final_mapping_fig))
    aux_map = robotics.OccupancyGrid(particles(max_w).map, current_map.Resolution);
    show(aux_map, 'Parent', final_mapping_ax);
    print(final_mapping_fig, 'plots/final_mapping/final_mapping.png', '-dpng');
    save('pasaltos.mat', 'aux_map', '-mat');
end
