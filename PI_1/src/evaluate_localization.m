function [metrics] = evaluate_localization(bagFile, method, params)
% EVALUATE_LOCALIZATION - Evalúa algoritmos de localización y calcula métricas
%
% Ejecuta un método de localización y lo compara con ground truth (si disponible)
% Calcula métricas de precisión, error y consistencia.
%
% Input:
%   bagFile - ruta al archivo rosbag
%   method  - 'ekf', 'mcl', 'amcl', o 'odom'
%   params  - parámetros del algoritmo (opcional)
%
% Output:
%   metrics - estructura con métricas de evaluación

    if nargin < 2
        method = 'ekf';
    end
    if nargin < 3
        params = struct();
    end

    % Opción de evaluación: alinear series al frame del ground truth
    % (útil si /odom y /base_pose_ground_truth están en frames distintos).
    if ~isfield(params, 'align_to_gt')
        params.align_to_gt = true;
    end

    fprintf('=== EVALUACIÓN DE LOCALIZACIÓN ===\n');
    fprintf('Método: %s\n', upper(method));
    fprintf('Archivo: %s\n\n', bagFile);
    
    %% Cargar ground truth y odometría
    bag = rosbag(bagFile);
    hasTopic = @(name) any(strcmp(name, bag.AvailableTopics.Properties.RowNames));
    timeFromHeader = @(h) double(h.Stamp.Sec) + double(h.Stamp.Nsec)*1e-9;
    quatToYaw = @(q) atan2(2*(q.W*q.Z + q.X*q.Y), 1 - 2*(q.Y^2 + q.Z^2));
    
    % Ground truth
    gt_available = hasTopic('/base_pose_ground_truth');
    if gt_available
        fprintf('✓ Ground truth disponible\n');
        bagGT = select(bag, 'Topic', '/base_pose_ground_truth');
        gtMsgs = readMessages(bagGT, 'DataFormat', 'struct');
        
        n_gt = numel(gtMsgs);
        gt.t = zeros(n_gt, 1);
        gt.x = zeros(n_gt, 1);
        gt.y = zeros(n_gt, 1);
        gt.yaw = zeros(n_gt, 1);
        
        for k = 1:n_gt
            m = gtMsgs{k};
            gt.t(k) = timeFromHeader(m.Header);
            gt.x(k) = m.Pose.Pose.Position.X;
            gt.y(k) = m.Pose.Pose.Position.Y;
            gt.yaw(k) = quatToYaw(m.Pose.Pose.Orientation);
        end
        gt.t = gt.t - gt.t(1);
    else
        fprintf('⚠ Ground truth NO disponible (métricas limitadas)\n');
        gt = [];
    end
    
    % Odometría (para comparación)
    if hasTopic('/odom')
        bagOdom = select(bag, 'Topic', '/odom');
        odomMsgs = readMessages(bagOdom, 'DataFormat', 'struct');
        
        n_odom = numel(odomMsgs);
        odom.t = zeros(n_odom, 1);
        odom.x = zeros(n_odom, 1);
        odom.y = zeros(n_odom, 1);
        odom.yaw = zeros(n_odom, 1);
        
        for k = 1:n_odom
            m = odomMsgs{k};
            odom.t(k) = timeFromHeader(m.Header);
            odom.x(k) = m.Pose.Pose.Position.X;
            odom.y(k) = m.Pose.Pose.Position.Y;
            odom.yaw(k) = quatToYaw(m.Pose.Pose.Orientation);
        end
        odom.t = odom.t - odom.t(1);
    end
    
    %% Ejecutar método de localización
    fprintf('\nEjecutando método de localización...\n');
    
    switch lower(method)
        case 'ekf'
            results = ekf_localization(bagFile, params);
        case 'ukf'
            results = ukf_localization(bagFile, params);
        case 'mcl'
            results = mcl_localization(bagFile, params);
        case 'amcl'
            % Leer resultados de AMCL del bag
            if hasTopic('/amcl_pose')
                bagAmcl = select(bag, 'Topic', '/amcl_pose');
                amclMsgs = readMessages(bagAmcl, 'DataFormat', 'struct');
                
                n = numel(amclMsgs);
                results.t = zeros(n, 1);
                results.x = zeros(n, 1);
                results.y = zeros(n, 1);
                results.yaw = zeros(n, 1);
                
                for k = 1:n
                    m = amclMsgs{k};
                    results.t(k) = timeFromHeader(m.Header);
                    results.x(k) = m.Pose.Pose.Position.X;
                    results.y(k) = m.Pose.Pose.Position.Y;
                    results.yaw(k) = quatToYaw(m.Pose.Pose.Orientation);
                end
                results.t = results.t - results.t(1);
            else
                error('No hay datos de AMCL en el bag');
            end
        case 'odom'
            results = odom;
        otherwise
            error('Método no reconocido: %s', method);
    end
    
    fprintf('✓ Localización completada\n\n');
    
    %% Calcular métricas
    metrics = struct();
    metrics.method = method;
    metrics.n_samples = length(results.t);
    metrics.duration = results.t(end);

    % Alinear resultados (y odometría) al ground truth si está disponible
    if gt_available && params.align_to_gt
        % Usar odometría como base del frame cuando exista, para que la
        % comparación vs odom (baseline) use la misma transformación.
        if exist('odom', 'var') && ~isempty(odom)
            [odom_aligned, T_align] = align_trajectory_2d(odom, gt, 0);
            odom = odom_aligned;

            % Aplicar la misma transformación a los resultados del método
            results_tmp = results;
            P = T_align.R * [results_tmp.x(:)'; results_tmp.y(:)'];
            results_tmp.x = (P(1, :)' + T_align.t(1));
            results_tmp.y = (P(2, :)' + T_align.t(2));
            if isfield(results_tmp, 'yaw') && ~isempty(results_tmp.yaw)
                results_tmp.yaw = wrapToPi(results_tmp.yaw(:) + T_align.dyaw);
            end
            results = results_tmp;
        else
            [results, T_align] = align_trajectory_2d(results, gt, 0);
        end

        metrics.alignment = T_align;
        metrics.alignment.applied = true;
    else
        metrics.alignment = struct('applied', false);
    end
    
    if gt_available
        fprintf('--- MÉTRICAS vs GROUND TRUTH ---\n');
        
        % Interpolar resultados al tiempo de ground truth
        est_x = interp1(results.t, results.x, gt.t, 'linear', 'extrap');
        est_y = interp1(results.t, results.y, gt.t, 'linear', 'extrap');
        est_yaw = interp1(results.t, results.yaw, gt.t, 'linear', 'extrap');
        
        % Error de posición
        error_x = est_x - gt.x;
        error_y = est_y - gt.y;
        error_pos = sqrt(error_x.^2 + error_y.^2);
        
        % Error de orientación (angular)
        error_yaw = wrapToPi(est_yaw - gt.yaw);
        
        % Métricas estadísticas
        metrics.rmse_x = sqrt(mean(error_x.^2));
        metrics.rmse_y = sqrt(mean(error_y.^2));
        metrics.rmse_pos = sqrt(mean(error_pos.^2));
        metrics.rmse_yaw = sqrt(mean(error_yaw.^2));
        
        metrics.mae_x = mean(abs(error_x));
        metrics.mae_y = mean(abs(error_y));
        metrics.mae_pos = mean(error_pos);
        metrics.mae_yaw = mean(abs(error_yaw));
        
        metrics.max_error_pos = max(error_pos);
        metrics.max_error_yaw = max(abs(error_yaw));
        
        metrics.std_error_x = std(error_x);
        metrics.std_error_y = std(error_y);
        metrics.std_error_pos = std(error_pos);
        metrics.std_error_yaw = std(error_yaw);
        
        % Guardar errores temporales para gráficas
        metrics.error_x = error_x;
        metrics.error_y = error_y;
        metrics.error_pos = error_pos;
        metrics.error_yaw = error_yaw;
        metrics.error_t = gt.t;
        
        % Imprimir resumen
        fprintf('RMSE posición:    %.4f m\n', metrics.rmse_pos);
        fprintf('RMSE orientación: %.4f rad (%.2f°)\n', metrics.rmse_yaw, rad2deg(metrics.rmse_yaw));
        fprintf('MAE posición:     %.4f m\n', metrics.mae_pos);
        fprintf('MAE orientación:  %.4f rad (%.2f°)\n', metrics.mae_yaw, rad2deg(metrics.mae_yaw));
        fprintf('Error máx posición: %.4f m\n', metrics.max_error_pos);
        fprintf('Desv. std posición: %.4f m\n', metrics.std_error_pos);
        
    else
        fprintf('⚠ No se pueden calcular métricas sin ground truth\n');
    end
    
    %% Métricas de consistencia (si están disponibles)
    if isfield(results, 'cov_trace')
        metrics.mean_uncertainty = mean(results.cov_trace);
        metrics.final_uncertainty = results.cov_trace(end);
        fprintf('\nIncertidumbre media: %.4f\n', metrics.mean_uncertainty);
    end
    
    if isfield(results, 'n_eff')
        metrics.mean_n_eff = mean(results.n_eff);
        fprintf('N_eff promedio: %.1f\n', metrics.mean_n_eff);
    end
    
    %% Calcular deriva de odometría (si disponible)
    if gt_available && exist('odom', 'var')
        odom_x_interp = interp1(odom.t, odom.x, gt.t, 'linear', 'extrap');
        odom_y_interp = interp1(odom.t, odom.y, gt.t, 'linear', 'extrap');
        
        error_odom_pos = sqrt((odom_x_interp - gt.x).^2 + (odom_y_interp - gt.y).^2);
        metrics.odom_rmse_pos = sqrt(mean(error_odom_pos.^2));
        
        % Mejora relativa respecto a odometría
        if metrics.odom_rmse_pos > 0
            improvement = (metrics.odom_rmse_pos - metrics.rmse_pos) / metrics.odom_rmse_pos * 100;
            metrics.improvement_vs_odom = improvement;
            fprintf('\n--- COMPARACIÓN CON ODOMETRÍA ---\n');
            fprintf('RMSE odometría: %.4f m\n', metrics.odom_rmse_pos);
            fprintf('Mejora:         %.1f%%\n', improvement);
        end
    end
    
    %% Guardar datos para visualización
    metrics.results = results;
    if gt_available
        metrics.ground_truth = gt;
    end
    if exist('odom', 'var')
        metrics.odom = odom;
    end
    
    fprintf('\n=== EVALUACIÓN COMPLETADA ===\n\n');
end
