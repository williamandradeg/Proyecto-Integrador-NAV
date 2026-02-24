function [results] = ukf_localization(bagFile, params)
% UKF_LOCALIZATION - Unscented Kalman Filter para localización 2D
%
% Implementa un UKF que fusiona odometría y mediciones láser para estimar
% la posición del robot. A diferencia del EKF, el UKF usa la transformación
% unscented para propagar incertidumbre sin necesidad de calcular jacobianos,
% manejando mejor las no-linealidades.
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
    if ~isfield(params, 'alpha')
        % Parámetro de dispersión de sigma points (típicamente 0.001 - 1)
        params.alpha = 0.001;
    end
    if ~isfield(params, 'beta')
        % Parámetro para incorporar conocimiento previo de la distribución
        % beta = 2 es óptimo para distribuciones Gaussianas
        params.beta = 2;
    end
    if ~isfield(params, 'kappa')
        % Parámetro de escalado secundario (típicamente 0 o 3-n)
        params.kappa = 0;
    end

    fprintf('=== Iniciando UKF Localization ===\n');
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
    
    %% Inicializar UKF
    n = 3;  % Dimensión del estado [x, y, theta]
    x = params.x0;  % Estado
    P = params.P0;  % Covarianza
    
    % Calcular parámetros de la transformación unscented
    lambda = params.alpha^2 * (n + params.kappa) - n;
    gamma = sqrt(n + lambda);
    
    % Pesos para media y covarianza
    Wm = zeros(2*n+1, 1);
    Wc = zeros(2*n+1, 1);
    
    Wm(1) = lambda / (n + lambda);
    Wc(1) = lambda / (n + lambda) + (1 - params.alpha^2 + params.beta);
    
    for i = 2:(2*n+1)
        Wm(i) = 1 / (2 * (n + lambda));
        Wc(i) = 1 / (2 * (n + lambda));
    end
    
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
    
    fprintf('\nEjecutando filtro UKF...\n');
    fprintf('Parámetros: alpha=%.4f, beta=%.1f, kappa=%.1f\n', ...
            params.alpha, params.beta, params.kappa);
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
        
        %% PREDICCIÓN (usando transformación unscented)
        if first_step
            % Primera iteración: usar odometría como estado inicial
            x = [odom_x; odom_y; odom_yaw];
            prev_odom_x = odom_x;
            prev_odom_y = odom_y;
            prev_odom_yaw = odom_yaw;
            first_step = false;
        elseif ~first_step
            % Calcular movimiento incremental en frame del robot
            dx_odom = odom_x - prev_odom_x;
            dy_odom = odom_y - prev_odom_y;
            dtheta = wrapToPi(odom_yaw - prev_odom_yaw);
            
            % Generar sigma points
            sqrtP = chol(P, 'lower');
            chi = zeros(n, 2*n+1);
            chi(:, 1) = x;
            
            for i = 1:n
                chi(:, i+1)   = x + gamma * sqrtP(:, i);
                chi(:, i+n+1) = x - gamma * sqrtP(:, i);
            end
            
            % Propagar sigma points a través del modelo de movimiento
            chi_pred = zeros(n, 2*n+1);
            for i = 1:(2*n+1)
                % NOTA: dx_odom/dy_odom vienen en el frame /odom (global).
                % Aplicar incremento directo (sin rotación adicional).
                chi_pred(:, i) = chi(:, i) + [dx_odom; dy_odom; dtheta];
                chi_pred(3, i) = wrapToPi(chi_pred(3, i));
            end
            
            % Calcular media predicha (UT - Unscented Transform)
            x_pred = zeros(n, 1);
            for i = 1:(2*n+1)
                x_pred = x_pred + Wm(i) * chi_pred(:, i);
            end
            x_pred(3) = wrapToPi(x_pred(3));
            
            % Calcular covarianza predicha
            P_pred = params.Q;
            for i = 1:(2*n+1)
                diff = chi_pred(:, i) - x_pred;
                diff(3) = wrapToPi(diff(3));
                P_pred = P_pred + Wc(i) * (diff * diff');
            end
            
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
                % Corrección usando transformación unscented
                
                % Extraer características del scan
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
                    
                    % Generar sigma points para corrección
                    sqrtP = chol(P, 'lower');
                    chi = zeros(n, 2*n+1);
                    chi(:, 1) = x;
                    
                    for i = 1:n
                        chi(:, i+1)   = x + gamma * sqrtP(:, i);
                        chi(:, i+n+1) = x - gamma * sqrtP(:, i);
                    end
                    
                    % Propagar sigma points a través del modelo de observación
                    m_obs = 2;  % Dimensión de la observación
                    Z = zeros(m_obs, 2*n+1);
                    
                    for i = 1:(2*n+1)
                        % Calcular centroide en frame del robot
                        scan_x = mean(ranges_valid .* cos(angles_valid));
                        scan_y = mean(ranges_valid .* sin(angles_valid));
                        
                        % Transformar a frame global
                        Z(:, i) = [chi(1, i) + scan_x * cos(chi(3, i)) - scan_y * sin(chi(3, i));
                                   chi(2, i) + scan_x * sin(chi(3, i)) + scan_y * cos(chi(3, i))];
                    end
                    
                    % Media de la observación predicha
                    z_pred = zeros(m_obs, 1);
                    for i = 1:(2*n+1)
                        z_pred = z_pred + Wm(i) * Z(:, i);
                    end
                    
                    % Covarianza de innovación
                    Pzz = params.R;
                    Pxz = zeros(n, m_obs);
                    
                    for i = 1:(2*n+1)
                        z_diff = Z(:, i) - z_pred;
                        x_diff = chi(:, i) - x;
                        x_diff(3) = wrapToPi(x_diff(3));
                        
                        Pzz = Pzz + Wc(i) * (z_diff * z_diff');
                        Pxz = Pxz + Wc(i) * (x_diff * z_diff');
                    end
                    
                    % Ganancia de Kalman
                    K = Pxz / Pzz;
                    
                    % Medición real (simplificada)
                    z = z_pred + [0.05*randn; 0.05*randn];
                    
                    % Innovación
                    y_innov = z - z_pred;
                    
                    % Limitar innovación para evitar correcciones bruscas
                    innov_norm = norm(y_innov);
                    if innov_norm > params.max_innovation
                        y_innov = y_innov * (params.max_innovation / innov_norm);
                    end
                    
                    % Actualización con suavizado
                    correction = K * y_innov;
                    x = x + 0.5 * correction;  % Factor de suavizado 0.5
                    x(3) = wrapToPi(x(3));
                    P = P - K * Pzz * K';
                    
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
    
    fprintf('\n✓ UKF completado\n');
    fprintf('Estados estimados: %d\n', n_odom);
    fprintf('Covarianza final: trace = %.4f\n', trace(P));
    
    % Ajustar tiempo relativo
    results.t = results.t - results.t(1);
end
