%% Clase Robot

classdef Robot
    properties (Access = private)
        % Describe las dimensiones del robot
        dimensiones = DimensionesRobot;
        % Describe la pose actual
        poseInicial = [0,0,0];
        % Describe si está en modo vigilancia o exploración
        modoVigilancia = false; 
        particulas;
        lidar;
    end
    
    methods (Static)
        function robot = NuevoRobot(dimensiones, poseInicial, modo, lidar)  
            arguments
                dimensiones DimensionesRobot
                poseInicial (1,:) {mustBeNumeric}
                modo {mustBeNumericOrLogical};
                lidar Lidar
            end
            
            robot = Robot;
            robot.dimensiones = dimensiones;
            robot.poseInicial = poseInicial;
            robot.modoVigilancia = modo;
            robot.lidar = lidar;
        end
    end  
    
    methods
        function [posEstimada, varEstimada] = EstimarPosicion(robot)
            poseX = [robot.particulas(:).pose(1)]';
            poseY = [robot.particulas(:).pose(2)]';
            poseT = [robot.particulas(:).pose(3)]';
            pesos = [robot.particulas(:).weight];

            posEstimada(:, 1) = pesos*poseX;
            posEstimada(:, 2) = pesos*poseY;
            posEstimada(:, 3) = wrapTo2Pi(pesos*poseT);

            varEstimada = var([poseX, poseY, poseT], pesos);
        end
               
        function robot = GuardarParticulas(robot, particulas)
            arguments
                robot Robot
                particulas (1,:) Particula
            end
            
            robot.particulas = particulas;
        end
        
        function particulas = LeerParticulas(robot)
            particulas = robot.particulas;
        end
        
%         function [v_cmd, w_cmd] = GenerarVelocidades(robot)
%             arguments
%                 robot Robot
%             end
%             
%             
%         end
        
        function robot = ActualizarParticulasConMediciones(robot, mediciones, mapa, mapa_likelihood)
            % TODO: Agregar el descartado de mediciones < 20cm 
            % (por los postes del robot)
            % TODO: Ver que pasa cuando usamos la roomba
            % TODO: Las mediciones están respecto al lidar. Hay que
            % pasarlas a la terna del robot.
            
            robot.calcularParticulasDesdeMediciones(robot.particulas, mediciones, mapa, mapa_likelihood)
            
        end
    end
    
    methods (Access = private)
        function particulasNuevas = calcularParticulasDesdeMediciones(robot, mediciones, mapa, mapa_likelihood)
        
                % Filtramos las mediciones
                mediciones = mediciones(1:end-1);
                angulosEscaneo = robot.lidar.scanAngles(1:end-1);
                angulosEscaneo = robot.lidar.scanAngles(~isnan(mediciones));
                mediciones = mediciones(~isnan(mediciones));
            
                particulasNuevas = robot.particulas;
                sigma = 0.2;
                p_norm = normpdf(0, 0, sigma);
                p_arbitraria = 0.01;
                Rt = 0.35/2;
                res = map.Resolution;
                epsilon = exp(-14);
        
                % Para cada una de las particulas
                for k = 1:numel(particles)
                    x = particulasNuevas(k).pose(1);
                    y = particulasNuevas(k).pose(2);
                    theta = particulasNuevas(k).pose(3);
                    offsetVec = [cos(theta) -sin(theta);
                                 sin(theta)  cos(theta)]*robot.lidar.sensorOffset';
                    
                    sensorLoc = [x, y] + offsetVec';
                    sensorPose = [sensorLoc, theta + robot.lidar.sensorAngleOffset];
                    
                    x_z_lidar = sensorPose(1) + mediciones.*cos(sensorPose(3) + scanAngles');
                    y_z_lidar = sensorPose(2) + mediciones.*sin(sensorPose(3) + scanAngles');
                    
                    particulasNuevas(k).peso = 1;

                    x_z = [x_z_lidar; x];
                    y_z = [y_z_lidar; y];
                end
        end
         
    end
end

