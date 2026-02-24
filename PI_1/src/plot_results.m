function plot_results(metrics, options)
% PLOT_RESULTS - Genera visualizaciones completas de resultados de localización
%
% Crea gráficas para el análisis y presentación de resultados, incluyendo:
% - Trayectorias 2D comparadas con ground truth
% - Evolución temporal de errores
% - Análisis de incertidumbre
% - Distribución de errores
%
% Input:
%   metrics - estructura de métricas (de evaluate_localization)
%   options - estructura con opciones de graficado (opcional):
%       .save_figures - guardar figuras (default: true)
%       .output_dir   - directorio de salida (default: '../results')
%       .format       - formato de exportación (default: 'png')
%       .show_particles - mostrar partículas finales en MCL (default: true)

    if nargin < 2
        options = struct();
    end
    
    % Opciones por defecto
    if ~isfield(options, 'save_figures')
        options.save_figures = true;
    end
    if ~isfield(options, 'output_dir')
        options.output_dir = '../results';
    end
    if ~isfield(options, 'format')
        options.format = 'png';
    end
    if ~isfield(options, 'show_particles')
        options.show_particles = true;
    end
    
    % Crear directorio de resultados si no existe
    if options.save_figures && ~exist(options.output_dir, 'dir')
        mkdir(options.output_dir);
    end
    
    method_name = upper(metrics.method);
    
    %% Figura 1: Trayectorias 2D
    fig1 = figure('Name', 'Trayectorias 2D', 'NumberTitle', 'off', 'Position', [100, 100, 800, 600]);
    hold on; grid on; axis equal;
    
    % Ground truth
    if isfield(metrics, 'ground_truth')
        gt = metrics.ground_truth;
        plot(gt.x, gt.y, 'k-', 'LineWidth', 2, 'DisplayName', 'Ground Truth');
    end
    
    % Odometría
    if isfield(metrics, 'odom')
        odom = metrics.odom;
        plot(odom.x, odom.y, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Odometría');
    end
    
    % Estimación
    results = metrics.results;
    plot(results.x, results.y, 'r-', 'LineWidth', 2, 'DisplayName', method_name);
    
    % Mostrar partículas finales (solo para MCL)
    if strcmp(lower(metrics.method), 'mcl') && isfield(results, 'final_particles') && options.show_particles
        particles = results.final_particles;
        weights = results.final_weights;
        
        % Mostrar solo partículas con peso significativo
        [~, idx_sorted] = sort(weights, 'descend');
        n_show = min(200, length(weights));
        idx_show = idx_sorted(1:n_show);
        
        scatter(particles(1, idx_show), particles(2, idx_show), 10, ...
                weights(idx_show), 'filled', 'DisplayName', 'Partículas');
        colormap('hot');
        cb = colorbar;
        cb.Label.String = 'Peso';
    end
    
    % Marcar inicio y fin
    plot(results.x(1), results.y(1), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Inicio');
    plot(results.x(end), results.y(end), 'rs', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Fin');
    
    xlabel('Posición X [m]', 'FontSize', 12);
    ylabel('Posición Y [m]', 'FontSize', 12);
    title(sprintf('Trayectoria Estimada - %s', method_name), 'FontSize', 14);
    legend('Location', 'best', 'FontSize', 10);
    hold off;
    
    if options.save_figures
        saveas(fig1, fullfile(options.output_dir, sprintf('trayectoria_%s.%s', lower(metrics.method), options.format)));
        fprintf('✓ Guardado: trayectoria_%s.%s\n', lower(metrics.method), options.format);
    end
    
    %% Figura 2: Error de posición temporal
    if isfield(metrics, 'error_pos')
        fig2 = figure('Name', 'Error de Posición', 'NumberTitle', 'off', 'Position', [150, 150, 900, 600]);
        
        subplot(3,1,1);
        plot(metrics.error_t, metrics.error_x, 'b-', 'LineWidth', 1.5);
        grid on;
        ylabel('Error X [m]', 'FontSize', 11);
        title(sprintf('Errores de Localización - %s', method_name), 'FontSize', 13);
        yline(0, 'k--', 'LineWidth', 0.5);
        
        subplot(3,1,2);
        plot(metrics.error_t, metrics.error_y, 'r-', 'LineWidth', 1.5);
        grid on;
        ylabel('Error Y [m]', 'FontSize', 11);
        yline(0, 'k--', 'LineWidth', 0.5);
        
        subplot(3,1,3);
        plot(metrics.error_t, metrics.error_pos, 'k-', 'LineWidth', 1.5);
        hold on;
        yline(metrics.rmse_pos, 'r--', sprintf('RMSE = %.3fm', metrics.rmse_pos), 'LineWidth', 1.5);
        yline(metrics.mae_pos, 'g--', sprintf('MAE = %.3fm', metrics.mae_pos), 'LineWidth', 1.5);
        grid on;
        xlabel('Tiempo [s]', 'FontSize', 11);
        ylabel('Error Posición [m]', 'FontSize', 11);
        legend('Error', 'RMSE', 'MAE', 'Location', 'best');
        hold off;
        
        if options.save_figures
            saveas(fig2, fullfile(options.output_dir, sprintf('error_posicion_%s.%s', lower(metrics.method), options.format)));
            fprintf('✓ Guardado: error_posicion_%s.%s\n', lower(metrics.method), options.format);
        end
    end
    
    %% Figura 3: Error de orientación temporal
    if isfield(metrics, 'error_yaw')
        fig3 = figure('Name', 'Error de Orientación', 'NumberTitle', 'off', 'Position', [200, 200, 900, 400]);
        
        plot(metrics.error_t, rad2deg(metrics.error_yaw), 'b-', 'LineWidth', 1.5);
        hold on;
        yline(rad2deg(metrics.rmse_yaw), 'r--', sprintf('RMSE = %.2f°', rad2deg(metrics.rmse_yaw)), 'LineWidth', 1.5);
        yline(rad2deg(metrics.mae_yaw), 'g--', sprintf('MAE = %.2f°', rad2deg(metrics.mae_yaw)), 'LineWidth', 1.5);
        yline(0, 'k--', 'LineWidth', 0.5);
        grid on;
        xlabel('Tiempo [s]', 'FontSize', 12);
        ylabel('Error de Orientación [°]', 'FontSize', 12);
        title(sprintf('Error Angular - %s', method_name), 'FontSize', 14);
        legend('Error', 'RMSE', 'MAE', 'Location', 'best');
        hold off;
        
        if options.save_figures
            saveas(fig3, fullfile(options.output_dir, sprintf('error_orientacion_%s.%s', lower(metrics.method), options.format)));
            fprintf('✓ Guardado: error_orientacion_%s.%s\n', lower(metrics.method), options.format);
        end
    end
    
    %% Figura 4: Distribución de errores (histogramas)
    if isfield(metrics, 'error_pos')
        fig4 = figure('Name', 'Distribución de Errores', 'NumberTitle', 'off', 'Position', [250, 250, 900, 600]);
        
        subplot(2,2,1);
        histogram(metrics.error_x, 30, 'Normalization', 'probability', 'FaceColor', 'b');
        xlabel('Error X [m]', 'FontSize', 10);
        ylabel('Probabilidad', 'FontSize', 10);
        title('Distribución Error X', 'FontSize', 11);
        grid on;
        xline(0, 'r--', 'LineWidth', 1.5);
        
        subplot(2,2,2);
        histogram(metrics.error_y, 30, 'Normalization', 'probability', 'FaceColor', 'r');
        xlabel('Error Y [m]', 'FontSize', 10);
        ylabel('Probabilidad', 'FontSize', 10);
        title('Distribución Error Y', 'FontSize', 11);
        grid on;
        xline(0, 'r--', 'LineWidth', 1.5);
        
        subplot(2,2,3);
        histogram(metrics.error_pos, 30, 'Normalization', 'probability', 'FaceColor', 'k');
        xlabel('Error de Posición [m]', 'FontSize', 10);
        ylabel('Probabilidad', 'FontSize', 10);
        title('Distribución Error Posición', 'FontSize', 11);
        grid on;
        xline(metrics.mae_pos, 'g--', 'MAE', 'LineWidth', 1.5);
        
        subplot(2,2,4);
        histogram(rad2deg(metrics.error_yaw), 30, 'Normalization', 'probability', 'FaceColor', 'm');
        xlabel('Error Angular [°]', 'FontSize', 10);
        ylabel('Probabilidad', 'FontSize', 10);
        title('Distribución Error Angular', 'FontSize', 11);
        grid on;
        xline(0, 'r--', 'LineWidth', 1.5);
        
        sgtitle(sprintf('Distribuciones de Error - %s', method_name), 'FontSize', 13);
        
        if options.save_figures
            saveas(fig4, fullfile(options.output_dir, sprintf('distribucion_errores_%s.%s', lower(metrics.method), options.format)));
            fprintf('✓ Guardado: distribucion_errores_%s.%s\n', lower(metrics.method), options.format);
        end
    end
    
    %% Figura 5: Incertidumbre (si está disponible)
    if isfield(metrics.results, 'cov_trace')
        fig5 = figure('Name', 'Incertidumbre', 'NumberTitle', 'off', 'Position', [300, 300, 900, 400]);
        
        plot(results.t, metrics.results.cov_trace, 'b-', 'LineWidth', 1.5);
        hold on;
        yline(metrics.mean_uncertainty, 'r--', sprintf('Media = %.4f', metrics.mean_uncertainty), 'LineWidth', 1.5);
        grid on;
        xlabel('Tiempo [s]', 'FontSize', 12);
        ylabel('Traza de Covarianza', 'FontSize', 12);
        title(sprintf('Evolución de Incertidumbre - %s', method_name), 'FontSize', 14);
        legend('Incertidumbre', 'Media', 'Location', 'best');
        hold off;
        
        if options.save_figures
            saveas(fig5, fullfile(options.output_dir, sprintf('incertidumbre_%s.%s', lower(metrics.method), options.format)));
            fprintf('✓ Guardado: incertidumbre_%s.%s\n', lower(metrics.method), options.format);
        end
    end
    
    %% Figura 6: N_eff para MCL
    if isfield(metrics.results, 'n_eff')
        fig6 = figure('Name', 'Número Efectivo de Partículas', 'NumberTitle', 'off', 'Position', [350, 350, 900, 400]);
        
        plot(results.t, metrics.results.n_eff, 'g-', 'LineWidth', 1.5);
        hold on;
        yline(metrics.mean_n_eff, 'r--', sprintf('Media = %.1f', metrics.mean_n_eff), 'LineWidth', 1.5);
        grid on;
        xlabel('Tiempo [s]', 'FontSize', 12);
        ylabel('N_{eff}', 'FontSize', 12);
        title('Número Efectivo de Partículas (MCL)', 'FontSize', 14);
        legend('N_{eff}', 'Media', 'Location', 'best');
        hold off;
        
        if options.save_figures
            saveas(fig6, fullfile(options.output_dir, sprintf('neff_%s.%s', lower(metrics.method), options.format)));
            fprintf('✓ Guardado: neff_%s.%s\n', lower(metrics.method), options.format);
        end
    end
    
    fprintf('\n✓ Todas las visualizaciones generadas\n');
end
