
clear; close all; clc;

% Configuración
addpath('src');

bagFile = 'mir_basics_20251210_114529.bag';
output_dir = 'resultados';

% Crear directorio si no existe
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

% Cargar métricas (deben haber sido generadas previamente con main_project.m)
if ~exist('resultados/all_metrics.mat', 'file')
    error('Ejecuta primero main_project.m para generar las métricas');
end

load('resultados/all_metrics.mat');
fprintf('✓ Métricas cargadas\n\n');

%% Configuración de estilo de figuras

% Colores consistentes
color_gt = [0 0 0];           % Negro
color_odom = [0 0.4470 0.7410];  % Azul
color_ekf = [0.8500 0.3250 0.0980];  % Naranja
color_ukf = [0.9290 0.6940 0.1250];  % Amarillo/Oro
color_mcl = [0.4660 0.6740 0.1880];  % Verde

% Tamaño de figuras para el informe (ajustar según necesidad)
fig_width = 8;   % pulgadas
fig_height = 6;  % pulgadas

% Función helper para guardar figuras
save_figure = @(fig, name) saveas(fig, fullfile(output_dir, [name '.png']));

%% FIGURA 1: Diagrama de arquitectura (placeholder - crear manualmente)
fprintf('📊 Figura 1: Diagrama de arquitectura\n');
fprintf('   → Crear manualmente en draw.io o similar\n');
fprintf('   → Guardar como: arquitectura_sistema.png\n\n');

%% FIGURA 2: Trayectorias comparadas con ground truth

fprintf('📊 Figura 2: Trayectorias comparadas\n');

fig2 = figure('Position', [100, 100, 800, 600]);
hold on; grid on; axis equal;

if isfield(metrics_ekf, 'ground_truth')
    gt = metrics_ekf.ground_truth;
    plot(gt.x, gt.y, 'Color', color_gt, 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');
end

plot(metrics_odom.results.x, metrics_odom.results.y, '--', 'Color', color_odom, ...
     'LineWidth', 1.5, 'DisplayName', 'Odometría');
plot(metrics_ekf.results.x, metrics_ekf.results.y, '-', 'Color', color_ekf, ...
     'LineWidth', 2, 'DisplayName', 'EKF');
plot(metrics_mcl.results.x, metrics_mcl.results.y, '-', 'Color', color_mcl, ...
     'LineWidth', 2, 'DisplayName', 'MCL');

% Marcar inicio y fin
plot(gt.x(1), gt.y(1), 'go', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Inicio');
plot(gt.x(end), gt.y(end), 'rs', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Fin');

xlabel('Posición X [m]', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Posición Y [m]', 'FontSize', 13, 'FontWeight', 'bold');
title('Comparación de Trayectorias Estimadas', 'FontSize', 15, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 11);
set(gca, 'FontSize', 11);
hold off;

save_figure(fig2, 'fig2_trayectorias_comparadas');
fprintf('   ✓ Guardado: fig2_trayectorias_comparadas.png\n\n');

%% FIGURA 3: Error temporal de posición (EKF, UKF, MCL)

fprintf('📊 Figura 3: Evolución temporal de errores de posición\n');

if isfield(metrics_ekf, 'error_pos') && exist('metrics_ukf', 'var')
    fig3 = figure('Position', [100, 100, 900, 700]);
    
    subplot(3,1,1);
    plot(metrics_ekf.error_t, metrics_ekf.error_pos, 'Color', color_ekf, 'LineWidth', 2);
    hold on;
    yline(metrics_ekf.rmse_pos, '--', sprintf('RMSE = %.3f m', metrics_ekf.rmse_pos), ...
          'Color', color_ekf, 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
    grid on;
    ylabel('Error de Posición [m]', 'FontSize', 12, 'FontWeight', 'bold');
    title('EKF - Error de Posición', 'FontSize', 13, 'FontWeight', 'bold');
    set(gca, 'FontSize', 11);
    hold off;
    
    subplot(3,1,2);
    plot(metrics_ukf.error_t, metrics_ukf.error_pos, 'Color', color_ukf, 'LineWidth', 2);
    hold on;
    yline(metrics_ukf.rmse_pos, '--', sprintf('RMSE = %.3f m', metrics_ukf.rmse_pos), ...
          'Color', color_ukf, 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
    grid on;
    ylabel('Error de Posición [m]', 'FontSize', 12, 'FontWeight', 'bold');
    title('UKF - Error de Posición', 'FontSize', 13, 'FontWeight', 'bold');
    set(gca, 'FontSize', 11);
    hold off;
    
    subplot(3,1,3);
    plot(metrics_mcl.error_t, metrics_mcl.error_pos, 'Color', color_mcl, 'LineWidth', 2);
    hold on;
    yline(metrics_mcl.rmse_pos, '--', sprintf('RMSE = %.3f m', metrics_mcl.rmse_pos), ...
          'Color', color_mcl, 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
    grid on;
    xlabel('Tiempo [s]', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Error de Posición [m]', 'FontSize', 12, 'FontWeight', 'bold');
    title('MCL - Error de Posición', 'FontSize', 13, 'FontWeight', 'bold');
    set(gca, 'FontSize', 11);
    hold off;
    
    save_figure(fig3, 'fig3_error_temporal_posicion');
    fprintf('   ✓ Guardado: fig3_error_temporal_posicion.png\n\n');
end

%% FIGURA 4: Error temporal de orientación

fprintf('📊 Figura 4: Evolución temporal de error angular\n');

if isfield(metrics_ekf, 'error_yaw') && exist('metrics_ukf', 'var')
    fig4 = figure('Position', [100, 100, 900, 500]);
    
    plot(metrics_ekf.error_t, rad2deg(metrics_ekf.error_yaw), 'Color', color_ekf, ...
         'LineWidth', 2, 'DisplayName', 'EKF');
    hold on;
    plot(metrics_ukf.error_t, rad2deg(metrics_ukf.error_yaw), 'Color', color_ukf, ...
         'LineWidth', 2, 'DisplayName', 'UKF');
    plot(metrics_mcl.error_t, rad2deg(metrics_mcl.error_yaw), 'Color', color_mcl, ...
         'LineWidth', 2, 'DisplayName', 'MCL');
    plot(metrics_odom.error_t, rad2deg(metrics_odom.error_yaw), '--', 'Color', color_odom, ...
         'LineWidth', 1.5, 'DisplayName', 'Odometría');
    yline(0, 'k--', 'LineWidth', 0.5);
    grid on;
    
    xlabel('Tiempo [s]', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Error Angular [°]', 'FontSize', 12, 'FontWeight', 'bold');
    title('Comparación de Errores de Orientación', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 11);
    set(gca, 'FontSize', 11);
    hold off;
    
    save_figure(fig4, 'fig4_error_temporal_orientacion');
    fprintf('   ✓ Guardado: fig4_error_temporal_orientacion.png\n\n');
end

%% FIGURA 5: Distribuciones de error (histogramas EKF, UKF, MCL)

fprintf('📊 Figura 5: Distribuciones de error\n');

if isfield(metrics_ekf, 'error_pos') && exist('metrics_ukf', 'var')
    fig5 = figure('Position', [100, 100, 1200, 800]);
    
    subplot(3,2,1);
    histogram(metrics_ekf.error_x, 30, 'Normalization', 'probability', ...
              'FaceColor', color_ekf, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    xlabel('Error X [m]', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Probabilidad', 'FontSize', 11, 'FontWeight', 'bold');
    title('EKF - Distribución Error X', 'FontSize', 12);
    grid on;
    xline(0, 'r--', 'LineWidth', 1.5);
    
    subplot(3,2,2);
    histogram(metrics_ekf.error_y, 30, 'Normalization', 'probability', ...
              'FaceColor', color_ekf, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    xlabel('Error Y [m]', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Probabilidad', 'FontSize', 11, 'FontWeight', 'bold');
    title('EKF - Distribución Error Y', 'FontSize', 12);
    grid on;
    xline(0, 'r--', 'LineWidth', 1.5);
    
    subplot(3,2,3);
    histogram(metrics_ukf.error_x, 30, 'Normalization', 'probability', ...
              'FaceColor', color_ukf, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    xlabel('Error X [m]', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Probabilidad', 'FontSize', 11, 'FontWeight', 'bold');
    title('UKF - Distribución Error X', 'FontSize', 12);
    grid on;
    xline(0, 'r--', 'LineWidth', 1.5);
    
    subplot(3,2,4);
    histogram(metrics_ukf.error_y, 30, 'Normalization', 'probability', ...
              'FaceColor', color_ukf, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    xlabel('Error Y [m]', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Probabilidad', 'FontSize', 11, 'FontWeight', 'bold');
    title('UKF - Distribución Error Y', 'FontSize', 12);
    grid on;
    xline(0, 'r--', 'LineWidth', 1.5);
    
    subplot(3,2,5);
    histogram(metrics_mcl.error_x, 30, 'Normalization', 'probability', ...
              'FaceColor', color_mcl, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    xlabel('Error X [m]', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Probabilidad', 'FontSize', 11, 'FontWeight', 'bold');
    title('MCL - Distribución Error X', 'FontSize', 12);
    grid on;
    xline(0, 'r--', 'LineWidth', 1.5);
    
    subplot(3,2,6);
    histogram(metrics_mcl.error_y, 30, 'Normalization', 'probability', ...
              'FaceColor', color_mcl, 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    xlabel('Error Y [m]', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Probabilidad', 'FontSize', 11, 'FontWeight', 'bold');
    title('MCL - Distribución Error Y', 'FontSize', 12);
    grid on;
    xline(0, 'r--', 'LineWidth', 1.5);
    
    sgtitle('Distribuciones de Error', 'FontSize', 14, 'FontWeight', 'bold');
    
    save_figure(fig5, 'fig5_distribuciones_error');
    fprintf('   ✓ Guardado: fig5_distribuciones_error.png\n\n');
end

%% FIGURA 6: Gráfico de barras comparativo

fprintf('📊 Figura 6: Gráfico de barras comparativo de métricas\n');

if isfield(metrics_ekf, 'rmse_pos') && exist('metrics_ukf', 'var')
    fig6 = figure('Position', [100, 100, 900, 500]);
    
    methods = {'Odometría', 'EKF', 'UKF', 'MCL'};
    rmse_values = [metrics_odom.rmse_pos, metrics_ekf.rmse_pos, metrics_ukf.rmse_pos, metrics_mcl.rmse_pos];
    mae_values = [metrics_odom.mae_pos, metrics_ekf.mae_pos, metrics_ukf.mae_pos, metrics_mcl.mae_pos];
    
    X = categorical(methods);
    X = reordercats(X, methods);
    
    b = bar(X, [rmse_values; mae_values]', 'grouped');
    b(1).FaceColor = [0.2 0.2 0.8];
    b(2).FaceColor = [0.8 0.2 0.2];
    
    grid on;
    ylabel('Error [m]', 'FontSize', 12, 'FontWeight', 'bold');
    title('Comparación de Métricas de Error', 'FontSize', 14, 'FontWeight', 'bold');
    legend({'RMSE', 'MAE'}, 'Location', 'best', 'FontSize', 11);
    set(gca, 'FontSize', 11);
    
    % Añadir valores sobre las barras
    xtips1 = b(1).XEndPoints;
    ytips1 = b(1).YEndPoints;
    labels1 = string(round(b(1).YData, 3));
    text(xtips1, ytips1, labels1, 'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'bottom', 'FontSize', 9);
    
    xtips2 = b(2).XEndPoints;
    ytips2 = b(2).YEndPoints;
    labels2 = string(round(b(2).YData, 3));
    text(xtips2, ytips2, labels2, 'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'bottom', 'FontSize', 9);
    
    save_figure(fig6, 'fig6_comparacion_metricas');
    fprintf('   ✓ Guardado: fig6_comparacion_metricas.png\n\n');
end

%% FIGURA 7: Incertidumbre temporal (EKF)

fprintf('📊 Figura 7: Evolución de incertidumbre (EKF)\n');

if isfield(metrics_ekf.results, 'cov_trace')
    fig7 = figure('Position', [100, 100, 900, 400]);
    
    plot(metrics_ekf.results.t, metrics_ekf.results.cov_trace, ...
         'Color', color_ekf, 'LineWidth', 2);
    hold on;
    yline(mean(metrics_ekf.results.cov_trace), '--', ...
          sprintf('Media = %.4f', mean(metrics_ekf.results.cov_trace)), ...
          'Color', [0.8 0 0], 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
    grid on;
    
    xlabel('Tiempo [s]', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Traza de Covarianza', 'FontSize', 12, 'FontWeight', 'bold');
    title('Evolución de Incertidumbre - EKF', 'FontSize', 14, 'FontWeight', 'bold');
    set(gca, 'FontSize', 11);
    hold off;
    
    save_figure(fig7, 'fig7_incertidumbre_ekf');
    fprintf('   ✓ Guardado: fig7_incertidumbre_ekf.png\n\n');
end

%% FIGURA 8: N_eff temporal (MCL)

fprintf('📊 Figura 8: Número efectivo de partículas (MCL)\n');

if isfield(metrics_mcl.results, 'n_eff')
    fig8 = figure('Position', [100, 100, 900, 400]);
    
    plot(metrics_mcl.results.t, metrics_mcl.results.n_eff, ...
         'Color', color_mcl, 'LineWidth', 2);
    hold on;
    yline(mean(metrics_mcl.results.n_eff), '--', ...
          sprintf('Media = %.1f', mean(metrics_mcl.results.n_eff)), ...
          'Color', [0.8 0 0], 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left');
    grid on;
    
    xlabel('Tiempo [s]', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('N_{eff}', 'FontSize', 12, 'FontWeight', 'bold');
    title('Número Efectivo de Partículas - MCL', 'FontSize', 14, 'FontWeight', 'bold');
    set(gca, 'FontSize', 11);
    hold off;
    
    save_figure(fig8, 'fig8_neff_mcl');
    fprintf('   ✓ Guardado: fig8_neff_mcl.png\n\n');
end

%% FIGURA 9: Scan láser representativo

fprintf('📊 Figura 9: Ejemplo de scan láser\n');

bag = rosbag(bagFile);
if any(strcmp('/scan', bag.AvailableTopics.Properties.RowNames))
    bagScan = select(bag, 'Topic', '/scan');
    scanMsgs = readMessages(bagScan, 'DataFormat', 'struct');
    
    % Escoger scan del medio
    idx = round(numel(scanMsgs)/2);
    s = scanMsgs{idx};
    
    ranges = double(s.Ranges(:));
    N = numel(ranges);
    angles = double(s.AngleMin + (0:N-1)' * s.AngleIncrement);
    
    % Filtrar puntos válidos
    mask = isfinite(ranges) & (ranges > s.RangeMin + 0.01) & (ranges < s.RangeMax - 0.01);
    ranges_obs = ranges(mask);
    angles_obs = angles(mask);
    
    x = ranges_obs .* cos(angles_obs);
    y = ranges_obs .* sin(angles_obs);
    
    fig9 = figure('Position', [100, 100, 700, 700]);
    scatter(x, y, 15, 'filled', 'MarkerFaceColor', [0.8 0.2 0.2]);
    axis equal; grid on;
    xlabel('X [m]', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Y [m]', 'FontSize', 12, 'FontWeight', 'bold');
    title(sprintf('Scan Láser 2D - %d puntos', numel(x)), 'FontSize', 14, 'FontWeight', 'bold');
    set(gca, 'FontSize', 11);
    
    save_figure(fig9, 'fig9_scan_laser');
    fprintf('   ✓ Guardado: fig9_scan_laser.png\n\n');
end

%% Resumen final

fprintf('================================================\n');
fprintf('   ✓ TODAS LAS FIGURAS GENERADAS CON ÉXITO     \n');
fprintf('================================================\n\n');

fprintf('Figuras guardadas en: %s\n\n', output_dir);
fprintf('Lista de figuras generadas:\n');
fprintf('  1. fig2_trayectorias_comparadas.png\n');
fprintf('  2. fig3_error_temporal_posicion.png\n');
fprintf('  3. fig4_error_temporal_orientacion.png\n');
fprintf('  4. fig5_distribuciones_error.png\n');
fprintf('  5. fig6_comparacion_metricas.png\n');
fprintf('  6. fig7_incertidumbre_ekf.png\n');
fprintf('  7. fig8_neff_mcl.png\n');
fprintf('  8. fig9_scan_laser.png\n\n');

fprintf('Figuras pendientes de crear manualmente:\n');
fprintf('  - Diagrama de arquitectura del sistema\n\n');

fprintf('Estas figuras están listas para insertar en el informe.\n');
fprintf('Usa sintaxis Markdown para incluirlas:\n');
fprintf('  ![Descripción](../results/figures_report/nombre_figura.png)\n\n');
