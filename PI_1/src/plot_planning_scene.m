function plot_planning_scene(occ, occInflated, start, goal, path, dyn, ttl)
% PLOT_PLANNING_SCENE - Visualiza mapa, ruta y obstáculo dinámico (si existe)

    if nargin < 7
        ttl = '';
    end

    clf;
    tiledlayout(1,2, 'Padding','compact', 'TileSpacing','compact');

    % Panel 1: Mapa original
    nexttile;
    imagesc(~occ); % libre=1, ocupado=0
    axis equal tight;
    colormap(gray);
    hold on;
    plot(goal(2), goal(1), 'gx', 'MarkerSize', 12, 'LineWidth', 2);
    plot(start(2), start(1), 'go', 'MarkerSize', 8, 'LineWidth', 2);
    if ~isempty(path)
        plot(path(:,2), path(:,1), 'r-', 'LineWidth', 2);
    end
    if nargin >= 6 && ~isempty(dyn)
        plot(dyn.center(2), dyn.center(1), 'mo', 'MarkerSize', 10, 'LineWidth', 2);
    end
    set(gca, 'YDir', 'normal');
    title('Mapa (original)');
    xlabel('col (x)'); ylabel('row (y)');
    grid on;

    % Panel 2: Mapa inflado (seguridad)
    nexttile;
    imagesc(~occInflated);
    axis equal tight;
    colormap(gray);
    hold on;
    plot(goal(2), goal(1), 'gx', 'MarkerSize', 12, 'LineWidth', 2);
    plot(start(2), start(1), 'go', 'MarkerSize', 8, 'LineWidth', 2);
    if ~isempty(path)
        plot(path(:,2), path(:,1), 'c-', 'LineWidth', 2);
    end
    if nargin >= 6 && ~isempty(dyn)
        plot(dyn.center(2), dyn.center(1), 'mo', 'MarkerSize', 10, 'LineWidth', 2);
    end
    set(gca, 'YDir', 'normal');
    title('Mapa (inflado)');
    xlabel('col (x)'); ylabel('row (y)');
    grid on;

    if ~isempty(ttl)
        sgtitle(ttl);
    end
end
