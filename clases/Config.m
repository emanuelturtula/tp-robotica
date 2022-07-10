%% Archivo de configuración

classdef Config
    properties
        matlab_version = '(R2020a)';

        %% Seleccion entre simulador y roomba
        % Switch para seleccionar entre roomba y simulación
        use_roomba = false;

        %% Seleccion de ejericio
        % Switch para seleccionar entre ejercicio de vigilancia y de exploracion
        vigilancia = true;

        %% Parametros del simulador
        % Simula datos no validos del lidar real, probar si se la banca
        simular_ruido_lidar = false;
        % Tiempo de simulacion en segundos
        simulation_duration = 3*60;
        % Tiempo de muestreo en segundos
        sample_time = 0.1; 
        % Pose inicial (x y theta) del robot simulado (el robot pude arrancar en cualquier lugar valido del mapa)
        init_pose = [2; 2.5; -pi/2]; 
    end
end


