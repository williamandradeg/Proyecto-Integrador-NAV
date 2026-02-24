function compare_methods(bagFile, methods, params)
% COMPARE_METHODS - Compara múltiples métodos de localización
%
% Ejecuta varios algoritmos de localización y genera comparativas
%
% Input:
%   bagFile - ruta al archivo rosbag
%   methods - cell array con nombres de métodos: {'ekf', 'ukf', 'mcl', 'amcl', 'odom'}
%   params  - estructura con parámetros para cada método (opcional)
%
% Ejemplo:
%   compare_methods('data.bag', {'ekf', 'ukf', 'mcl', 'odom'})

    if nargin < 2
        methods = {'ekf', 'ukf', 'mcl', 'odom'};
    end
    if nargin < 3
        params = struct();
    end
    
    fprintf('=== COMPARACIÓN DE MÉTODOS DE LOCALIZACIÓN ===\n\n');
    
    n_methods = length(methods);
    all_metrics = cell(n_methods, 1);
    
    %% Ejecutar cada método
    for i = 1:n_methods
        fprintf('\n>>> Evaluando método %d/%d: %s\n', i, n_methods, upper(methods{i}));
        
        % Parámetros específicos para este método
        if isfield(params, methods{i})
            method_params = params.(methods{i});
        else
            method_params = struct();
        end
        
        try
            all_metrics{i} = evaluate_localization(bagFile, methods{i}, method_params);
        catch ME
            warning('Error en método %s: %s', methods{i}, ME.message);
            all_metrics{i} = [];
        end
    end
    
    %% Crear figura comparativa de trayectorias
    fig1 = figure('Name', 'Comparación de Trayectorias', 'NumberTitle', 'off', ...
                  'Position', [100, 100, 1000, 700]);
    hold on; grid on; axis equal;
    
    % Ground truth
    if ~isempty(all_metrics{1}) && isfield(all_metrics{1}, 'ground_truth')
        gt = all_metrics{1}.ground_truth;
        plot(gt.x, gt.y, 'k-', 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
    end
    
    % Colores y estilos
    colors = {'r', 'b', 'g', 'm', 'c'};
    styles = {'-', '--', '-.', ':', '-'};
    
    % Cada método
    for i = 1:n_methods
        if ~isempty(all_metrics{i})
            res = all_metrics{i}.results;
            plot(res.x, res.y, [colors{mod(i-1,5)+1} styles{mod(i-1,5)+1}], ...
                 'LineWidth', 2, 'DisplayName', upper(methods{i}));
        end
    end
    
    xlabel('Posición X [m]', 'FontSize', 12);
    ylabel('Posición Y [m]', 'FontSize', 12);
    title('Comparación de Trayectorias', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 11);
    hold off;
    
    % Guardar figura
    if ~exist('../results', 'dir')
        mkdir('../results');
    end
    saveas(fig1, '../results/comparacion_trayectorias.png');
    fprintf('\n✓ Guardado: comparacion_trayectorias.png\n');
    
    %% Crear tabla comparativa de métricas
    if ~isempty(all_metrics{1}) && isfield(all_metrics{1}, 'rmse_pos')
        fprintf('\n=== TABLA COMPARATIVA DE MÉTRICAS ===\n\n');
        
        % Encabezados
        fprintf('%-15s | %10s | %10s | %10s | %10s | %10s\n', ...
                'Método', 'RMSE [m]', 'MAE [m]', 'Max [m]', 'RMSE θ[°]', 'MAE θ[°]');
        fprintf('%s\n', repmat('-', 1, 85));
        
        % Filas
        for i = 1:n_methods
            if ~isempty(all_metrics{i}) && isfield(all_metrics{i}, 'rmse_pos')
                m = all_metrics{i};
                fprintf('%-15s | %10.4f | %10.4f | %10.4f | %10.2f | %10.2f\n', ...
                        upper(methods{i}), m.rmse_pos, m.mae_pos, m.max_error_pos, ...
                        rad2deg(m.rmse_yaw), rad2deg(m.mae_yaw));
            else
                fprintf('%-15s | %s\n', upper(methods{i}), 'N/A (sin ground truth)');
            end
        end
        fprintf('\n');
    end
    
    %% Gráfica comparativa de errores
    if ~isempty(all_metrics{1}) && isfield(all_metrics{1}, 'error_pos')
        fig2 = figure('Name', 'Comparación de Errores', 'NumberTitle', 'off', ...
                      'Position', [150, 150, 1000, 600]);
        
        subplot(2,1,1);
        hold on; grid on;
        for i = 1:n_methods
            if ~isempty(all_metrics{i}) && isfield(all_metrics{i}, 'error_pos')
                m = all_metrics{i};
                plot(m.error_t, m.error_pos, [colors{mod(i-1,5)+1} styles{mod(i-1,5)+1}], ...
                     'LineWidth', 1.5, 'DisplayName', upper(methods{i}));
            end
        end
        xlabel('Tiempo [s]', 'FontSize', 11);
        ylabel('Error de Posición [m]', 'FontSize', 11);
        title('Comparación de Errores de Posición', 'FontSize', 13);
        legend('Location', 'best');
        hold off;
        
        subplot(2,1,2);
        hold on; grid on;
        for i = 1:n_methods
            if ~isempty(all_metrics{i}) && isfield(all_metrics{i}, 'error_yaw')
                m = all_metrics{i};
                plot(m.error_t, rad2deg(m.error_yaw), [colors{mod(i-1,5)+1} styles{mod(i-1,5)+1}], ...
                     'LineWidth', 1.5, 'DisplayName', upper(methods{i}));
            end
        end
        xlabel('Tiempo [s]', 'FontSize', 11);
        ylabel('Error Angular [°]', 'FontSize', 11);
        title('Comparación de Errores Angulares', 'FontSize', 13);
        legend('Location', 'best');
        hold off;
        
        saveas(fig2, '../results/comparacion_errores.png');
        fprintf('✓ Guardado: comparacion_errores.png\n');
    end
    
    %% Gráfica de barras con métricas
    if ~isempty(all_metrics{1}) && isfield(all_metrics{1}, 'rmse_pos')
        fig3 = figure('Name', 'Métricas Comparativas', 'NumberTitle', 'off', ...
                      'Position', [200, 200, 900, 500]);
        
        % Extraer métricas
        rmse_values = zeros(n_methods, 1);
        mae_values = zeros(n_methods, 1);
        method_labels = cell(n_methods, 1);
        
        for i = 1:n_methods
            if ~isempty(all_metrics{i}) && isfield(all_metrics{i}, 'rmse_pos')
                rmse_values(i) = all_metrics{i}.rmse_pos;
                mae_values(i) = all_metrics{i}.mae_pos;
                method_labels{i} = upper(methods{i});
            else
                rmse_values(i) = NaN;
                mae_values(i) = NaN;
                method_labels{i} = upper(methods{i});
            end
        end
        
        % Gráfico de barras
        X = categorical(method_labels);
        X = reordercats(X, method_labels);
        
        bar(X, [rmse_values, mae_values]);
        grid on;
        ylabel('Error [m]', 'FontSize', 12);
        title('Comparación de Métricas de Error', 'FontSize', 14, 'FontWeight', 'bold');
        legend({'RMSE', 'MAE'}, 'Location', 'best', 'FontSize', 11);
        
        saveas(fig3, '../results/comparacion_metricas.png');
        fprintf('✓ Guardado: comparacion_metricas.png\n');
    end
    
    %% Exportar tabla a archivo CSV
    if ~isempty(all_metrics{1}) && isfield(all_metrics{1}, 'rmse_pos')
        csv_file = '../results/metricas_comparativas.csv';
        fid = fopen(csv_file, 'w');
        
        fprintf(fid, 'Metodo,RMSE_pos[m],MAE_pos[m],Max_error[m],RMSE_theta[deg],MAE_theta[deg],Std_pos[m]\n');
        
        for i = 1:n_methods
            if ~isempty(all_metrics{i}) && isfield(all_metrics{i}, 'rmse_pos')
                m = all_metrics{i};
                fprintf(fid, '%s,%.6f,%.6f,%.6f,%.4f,%.4f,%.6f\n', ...
                        methods{i}, m.rmse_pos, m.mae_pos, m.max_error_pos, ...
                        rad2deg(m.rmse_yaw), rad2deg(m.mae_yaw), m.std_error_pos);
            end
        end
        
        fclose(fid);
        fprintf('✓ Guardado: metricas_comparativas.csv\n');
    end
    
    fprintf('\n=== COMPARACIÓN COMPLETADA ===\n\n');
    
    % Retornar métricas si se solicita
    if nargout > 0
        varargout{1} = all_metrics;
    end
end
