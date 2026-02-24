function occInflated = inflate_obstacles_grid(occ, radiusCells)
% INFLATE_OBSTACLES_GRID - Infla obstáculos en un grid por un radio (células)
%
% Implementación sin toolboxes: dilatación aproximada con convolución.
%
% Inputs:
%   occ         - matriz lógica (1 ocupado)
%   radiusCells - entero >= 0

    if radiusCells <= 0
        occInflated = logical(occ);
        return;
    end

    k = 2*radiusCells + 1;
    kernel = ones(k, k);
    occInflated = conv2(double(occ), kernel, 'same') > 0;
    occInflated = logical(occInflated);
end
