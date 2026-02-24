function [results] = ekf_localization(bagFile, params)
% EKF_LOCALIZATION - Filtro de Kalman Extendido para localización 2D
%
% Implementa un EKF que fusiona odometría y mediciones láser para estimar
% la posición del robot en un entorno 2D.
%
% Input:
%   bagFile - ruta al archivo rosbag
%   params  - estructura con parámetros del filtro (opcional)
%
% Output:
%   results - estructura con:
%       .t    - vector de tiempos
%       .x    - posición x estimada
%       .y    - posición y estimada
%       .yaw  - orientación estimada
%       .cov  - covarianza en cada paso
%       .innovation - innovaciones del filtro

    if nargin < 2
        params = struct();
    end
    
    % Parámetros por defecto
    if ~isfield(params, 'Q')
        % Ruido del proceso (odometría) - aumentado para suavizar
        params.Q = diag([0.05, 0.05, 0.02].^2);
    end
    if ~isfield(params, 'R')
        % Ruido de medición (láser) - aumentado para ser menos reactivo
        params.R = diag([0.5, 0.5].^2);
    end
    if ~isfield(params, 'max_innovation')
        % Límite máximo de corrección para evitar saltos bruscos
        params.max_innovation = 0.3;  % metros
    end
    if ~isfield(params, 'x0')
        % Estado inicial [x, y, theta]
        params.x0 = [0; 0; 0];
    end
    if ~isfield(params, 'P0')
        % Covarianza inicial
        params.P0 = diag([0.5, 0.5, 0.3].^2);
    end
    if ~isfield(params, 'use_landmarks')
        params.use_landmarks = false;
    end

    fprintf('=== Iniciando EKF Localization ===\n');
    fprintf('Cargando rosbag: %s\n', bagFile);
    
    bag = rosbag(bagFile);
    
    % Helper functions
    hasTopic = @(name) any(strcmp(name, bag.AvailableTopics.Properties.RowNames));
    timeFromHeader = @(h) double(h.Stamp.Sec) + double(h.Stamp.Nsec)*1e-9;
    quatToYaw = @(q) atan2(2*(q.W*q.Z + q.X*q.Y), 1 - 2*(q.Y^2 + q.Z^2));
    
    %% Leer odometría
    if ~hasTopic('/odom')
        error('El bag no contiene el topic /odom');
    end
    
    bagOdom = select(bag, 'Topic', '/odom');
    odomMsgs = readMessages(bagOdom, 'DataFormat', 'struct');
    n_odom = numel(odomMsgs);
    
    fprintf('Leyendo %d mensajes de odometría...\n', n_odom);
    
    %% Leer scan láser (opcional para corrección)
    scan_available = hasTopic('/scan');
    if scan_available
        bagScan = select(bag, 'Topic', '/scan');
        scanMsgs = readMessages(bagScan, 'DataFormat', 'struct');
        fprintf('Leyendo %d mensajes de láser...\n', numel(scanMsgs));
    else
        fprintf('Advertencia: No hay datos de láser, solo predicción con odometría\n');
    end
    
    %% Inicializar EKF
    x = params.x0;  % Estado [x, y, theta]
    P = params.P0;  % Covarianza
    
    % Almacenamiento de resultados
    results.t = zeros(n_odom, 1);
    results.x = zeros(n_odom, 1);
    results.y = zeros(n_odom, 1);
    results.yaw = zeros(n_odom, 1);
    results.cov_trace = zeros(n_odom, 1);
    results.innovation = [];
    
    % Estado anterior para calcular odometría incremental
    prev_odom_x = 0;
    prev_odom_y = 0;
    prev_odom_yaw = 0;
    first_step = true;
    
    scan_idx = 1;
    
    fprintf('\nEjecutando filtro EKF...\n');
    fprintf('Progreso: ');
    
    %% Loop principal
    for k = 1:n_odom
        % Mostrar progreso
        if mod(k, floor(n_odom/10)) == 0
            fprintf('%d%% ', round(100*k/n_odom));
        end
        
        m = odomMsgs{k};
        t = timeFromHeader(m.Header);
        
        % Odometría actual (en frame global del robot)
        odom_x = m.Pose.Pose.Position.X;
        odom_y = m.Pose.Pose.Position.Y;
        odom_yaw = quatToYaw(m.Pose.Pose.Orientation);
        
        %% PREDICCIÓN (usando odometría incremental)
        if first_step
            % Primera iteración: usar odometría como estado inicial
            x = [odom_x; odom_y; odom_yaw];
            prev_odom_x = odom_x;
            prev_odom_y = odom_y;
            prev_odom_yaw = odom_yaw;
            first_step = false;
        else
            % Calcular movimiento incremental en frame de odometría
            dx_odom = odom_x - prev_odom_x;
            dy_odom = odom_y - prev_odom_y;
            dtheta = wrapToPi(odom_yaw - prev_odom_yaw);
            x_pred = x + [dx_odom; dy_odom; dtheta];
            x_pred(3) = wrapToPi(x_pred(3));
            
            % Jacobiano del modelo de movimiento
            F = eye(3);
            
            % Predicción de covarianza
            P_pred = F * P * F' + params.Q;
            
            x = x_pred;
            P = P_pred;
            
            % Actualizar odometría anterior
            prev_odom_x = odom_x;
            prev_odom_y = odom_y;
            prev_odom_yaw = odom_yaw;
        end
        
        %% CORRECCIÓN (usando láser si está disponible)
        if scan_available && scan_idx <= numel(scanMsgs)
            % Buscar scan más cercano en tiempo
            scan_time = timeFromHeader(scanMsgs{scan_idx}.Header);
            
            % Si el scan está cerca temporalmente, usarlo para corrección
            if abs(scan_time - t) < 0.1  % Tolerancia de 100ms
                % Corrección simple usando características del scan
                % (en implementación real: matching con mapa, landmarks, etc.)
                
                % Extraer características del scan (simplificado)
                scan = scanMsgs{scan_idx};
                ranges = double(scan.Ranges(:));
                
                % Filtrar lecturas válidas
                valid = isfinite(ranges) & ...
                        (ranges > scan.RangeMin + 0.01) & ...
                        (ranges < scan.RangeMax - 0.01);
                
                if sum(valid) > 10  % Suficientes lecturas válidas
                    angles = double(scan.AngleMin + (0:numel(ranges)-1)' * scan.AngleIncrement);
                    
                    ranges_valid = ranges(valid);
                    angles_valid = angles(valid);
                    
                    % Calcular centroide en frame del robot
                    scan_x = mean(ranges_valid .* cos(angles_valid));
                    scan_y = mean(ranges_valid .* sin(angles_valid));
                    
                    % Transformar a frame global (predicción)
                    z_pred = [x(1) + scan_x * cos(x(3)) - scan_y * sin(x(3));
                              x(2) + scan_x * sin(x(3)) + scan_y * cos(x(3))];
                    
                    z = z_pred + [0.05*randn; 0.05*randn];
                    
                    % Innovación
                    y_innov = z - z_pred;
                    
                    % Limitar innovación para evitar correcciones bruscas
                    innov_norm = norm(y_innov);
                    if innov_norm > params.max_innovation
                        y_innov = y_innov * (params.max_innovation / innov_norm);
                    end
                    
                    % Jacobiano de observación
                    H = [1, 0, -scan_x*sin(x(3)) - scan_y*cos(x(3));
                         0, 1,  scan_x*cos(x(3)) - scan_y*sin(x(3))];
                    
                    % Ganancia de Kalman
                    S = H * P * H' + params.R;
                    K = P * H' / S;
                    
                    % Actualización con suavizado
                    correction = K * y_innov;
                    x = x + 0.5 * correction;  % Factor de suavizado 0.5
                    x(3) = wrapToPi(x(3));
                    P = (eye(3) - K * H) * P;
                    
                    % Guardar innovación
                    results.innovation = [results.innovation; k, t, norm(y_innov)];
                end
                
                scan_idx = scan_idx + 1;
            end
        end
        
        % Guardar resultados
        results.t(k) = t;
        results.x(k) = x(1);
        results.y(k) = x(2);
        results.yaw(k) = x(3);
        results.cov_trace(k) = trace(P);
    end
    
    fprintf('\n✓ EKF completado\n');
    fprintf('Estados estimados: %d\n', n_odom);
    fprintf('Covarianza final: trace = %.4f\n', trace(P));
    
    % Ajustar tiempo relativo
    results.t = results.t - results.t(1);
end
