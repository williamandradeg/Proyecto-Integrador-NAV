function [results] = mcl_localization(bagFile, params)
% MCL_LOCALIZATION - Monte Carlo Localization (Filtro de Partículas)
%
% Implementa MCL para localización del robot usando odometría y mediciones
% láser. Este método es robusto ante incertidumbre multimodal.
%
% Input:
%   bagFile - ruta al archivo rosbag
%   params  - estructura con parámetros (opcional):
%       .n_particles - número de partículas (default: 1000)
%       .map_size    - tamaño del mapa [xmin xmax ymin ymax]
%       .alpha       - ruido de odometría [a1 a2 a3 a4]
%       .sensor_std  - desviación estándar del sensor láser
%
% Output:
%   results - estructura con resultados de localización

    if nargin < 2
        params = struct();
    end
    
    % Parámetros por defecto
    if ~isfield(params, 'n_particles')
        params.n_particles = 1000;
    end
    if ~isfield(params, 'map_size')
        params.map_size = [-10 10 -10 10];  % [xmin xmax ymin ymax]
    end
    if ~isfield(params, 'alpha')
        % Ruido de odometría [traslación, rotación, traslación, rotación]
        % Reducido para evitar dispersión excesiva
        params.alpha = [0.05, 0.05, 0.02, 0.02];
    end
    if ~isfield(params, 'sensor_std')
        params.sensor_std = 0.3;  % metros - reducido para ser más preciso
    end
    if ~isfield(params, 'resample_threshold')
        params.resample_threshold = 0.5;  % N_eff threshold
    end
    
    fprintf('=== Iniciando MCL (Monte Carlo Localization) ===\n');
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
    
    %% Leer scan láser
    scan_available = hasTopic('/scan');
    if scan_available
        bagScan = select(bag, 'Topic', '/scan');
        scanMsgs = readMessages(bagScan, 'DataFormat', 'struct');
        fprintf('Leyendo %d mensajes de láser...\n', numel(scanMsgs));
    else
        warning('No hay datos de láser, MCL necesita sensores para corrección');
    end
    
    %% Inicializar partículas
    N = params.n_particles;
    
    % Inicialización uniforme en el espacio de estados
    % En práctica real: inicializar cerca de posición conocida o AMCL global localization
    particles = zeros(3, N);  % [x; y; theta]
    particles(1,:) = params.map_size(1) + (params.map_size(2)-params.map_size(1)) * rand(1, N);
    particles(2,:) = params.map_size(3) + (params.map_size(4)-params.map_size(3)) * rand(1, N);
    particles(3,:) = 2*pi*rand(1, N) - pi;
    
    weights = ones(1, N) / N;  % Pesos uniformes inicialmente
    
    % Almacenamiento de resultados
    results.t = zeros(n_odom, 1);
    results.x = zeros(n_odom, 1);
    results.y = zeros(n_odom, 1);
    results.yaw = zeros(n_odom, 1);
    results.std_x = zeros(n_odom, 1);
    results.std_y = zeros(n_odom, 1);
    results.n_eff = zeros(n_odom, 1);  % Número efectivo de partículas
    
    % Estado anterior para calcular odometría incremental
    prev_odom_x = 0;
    prev_odom_y = 0;
    prev_odom_yaw = 0;
    first_step = true;
    
    scan_idx = 1;
    
    fprintf('\nEjecutando filtro MCL...\n');
    fprintf('Partículas: %d\n', N);
    fprintf('Progreso: ');
    
    %% Loop principal
    for k = 1:n_odom
        % Mostrar progreso
        if mod(k, floor(n_odom/10)) == 0
            fprintf('%d%% ', round(100*k/n_odom));
        end
        
        m = odomMsgs{k};
        t = timeFromHeader(m.Header);
        
        % Odometría actual
        odom_x = m.Pose.Pose.Position.X;
        odom_y = m.Pose.Pose.Position.Y;
        odom_yaw = quatToYaw(m.Pose.Pose.Orientation);
        
        %% PREDICCIÓN (Motion Model)
        if first_step
            % Primera iteración: inicializar partículas alrededor de odometría inicial
            for i = 1:N
                particles(1, i) = odom_x + 0.1*randn;
                particles(2, i) = odom_y + 0.1*randn;
                particles(3, i) = odom_yaw + 0.05*randn;
            end
            prev_odom_x = odom_x;
            prev_odom_y = odom_y;
            prev_odom_yaw = odom_yaw;
            first_step = false;
        else
            % Calcular movimiento incremental en coordenadas de odometría
            dx_odom = odom_x - prev_odom_x;
            dy_odom = odom_y - prev_odom_y;
            dtheta = wrapToPi(odom_yaw - prev_odom_yaw);
            
            % Mover cada partícula
            for i = 1:N
                % Añadir ruido al movimiento
                noisy_dx = dx_odom + params.alpha(1)*abs(dx_odom)*randn + params.alpha(2)*abs(dtheta)*randn;
                noisy_dy = dy_odom + params.alpha(1)*abs(dy_odom)*randn + params.alpha(2)*abs(dtheta)*randn;
                noisy_dtheta = dtheta + params.alpha(3)*abs(dtheta)*randn + params.alpha(4)*sqrt(dx_odom^2 + dy_odom^2)*randn;

                % NOTA: dx_odom/dy_odom vienen en /odom (global). Aplicar directamente.
                particles(1, i) = particles(1, i) + noisy_dx;
                particles(2, i) = particles(2, i) + noisy_dy;
                particles(3, i) = wrapToPi(particles(3, i) + noisy_dtheta);
            end
            
            % Actualizar odometría anterior
            prev_odom_x = odom_x;
            prev_odom_y = odom_y;
            prev_odom_yaw = odom_yaw;
        end
        
        %% CORRECCIÓN (Sensor Model / Importance Weighting)
        if scan_available && scan_idx <= numel(scanMsgs)
            scan_time = timeFromHeader(scanMsgs{scan_idx}.Header);
            
            if abs(scan_time - t) < 0.1
                scan = scanMsgs{scan_idx};
                ranges = double(scan.Ranges(:));
                angles = double(scan.AngleMin + (0:numel(ranges)-1)' * scan.AngleIncrement);
                
                % Filtrar lecturas válidas
                valid = isfinite(ranges) & ...
                        (ranges > scan.RangeMin + 0.01) & ...
                        (ranges < scan.RangeMax - 0.01);
                
                if sum(valid) > 10
                    % Muestreo de rayos para evaluación (para eficiencia)
                    ray_step = max(1, floor(sum(valid) / 20));  % Usar ~20 rayos
                    valid_indices = find(valid);
                    sample_indices = valid_indices(1:ray_step:end);
                    
                    ranges_sample = ranges(sample_indices);
                    angles_sample = angles(sample_indices);
                    
                    % Calcular likelihood para cada partícula
                    for i = 1:N
                        % Modelo de sensor simplificado: comparar con mediciones
                        % En práctica real: ray casting contra mapa ocupado
                        
                        % Estimación de likelihood basada en coherencia espacial
                        % (simulación simplificada sin mapa explícito)
                        
                        % Calcular centroide de mediciones en frame global
                        x_sensor = particles(1, i) + ranges_sample .* cos(angles_sample + particles(3, i));
                        y_sensor = particles(2, i) + ranges_sample .* sin(angles_sample + particles(3, i));
                        
                        % Likelihood: probabilidad de que estas mediciones sean consistentes
                        % Versión simplificada: penalizar dispersión excesiva
                        dispersion = std(x_sensor) + std(y_sensor);
                        
                        % Modelo de likelihood Gaussiano
                        weights(i) = weights(i) * exp(-dispersion^2 / (2*params.sensor_std^2));
                    end
                end
                
                scan_idx = scan_idx + 1;
            end
        end
        
        %% Normalizar pesos
        weights = weights / sum(weights);
        
        % Calcular número efectivo de partículas
        n_eff = 1 / sum(weights.^2);
        results.n_eff(k) = n_eff;
        
        %% RESAMPLING (si es necesario)
        if n_eff < params.resample_threshold * N
            % Low variance resampling
            particles = low_variance_resample(particles, weights);
            weights = ones(1, N) / N;
        end
        
        %% Estimar pose (media ponderada)
        results.x(k) = sum(weights .* particles(1,:));
        results.y(k) = sum(weights .* particles(2,:));
        
        % Para orientación: usar media circular
        results.yaw(k) = atan2(sum(weights .* sin(particles(3,:))), ...
                               sum(weights .* cos(particles(3,:))));
        
        % Calcular desviación estándar
        results.std_x(k) = sqrt(sum(weights .* (particles(1,:) - results.x(k)).^2));
        results.std_y(k) = sqrt(sum(weights .* (particles(2,:) - results.y(k)).^2));
        
        results.t(k) = t;
    end
    
    fprintf('\n✓ MCL completado\n');
    fprintf('Estados estimados: %d\n', n_odom);
    fprintf('N_eff promedio: %.1f / %d\n', mean(results.n_eff), N);
    
    % Ajustar tiempo relativo
    results.t = results.t - results.t(1);
    
    % Guardar partículas finales para visualización
    results.final_particles = particles;
    results.final_weights = weights;
end

%% Funciones auxiliares

function sample = sample_normal_distribution(b)
    % Muestrear de distribución normal con varianza b^2
    % Usando aproximación de suma de uniformes (más eficiente que randn)
    if b == 0
        sample = 0;
    else
        sample = b * randn;
    end
end

function new_particles = low_variance_resample(particles, weights)
    % Low variance resampling algorithm
    N = size(particles, 2);
    new_particles = zeros(size(particles));
    
    r = rand / N;
    c = weights(1);
    i = 1;
    
    for m = 1:N
        U = r + (m-1) / N;
        while U > c
            i = i + 1;
            c = c + weights(i);
        end
        new_particles(:, m) = particles(:, i);
    end
end
