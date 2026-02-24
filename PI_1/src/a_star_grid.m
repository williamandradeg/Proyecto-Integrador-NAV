function path = a_star_grid(occ, start, goal, connectivity)
% A_STAR_GRID - Planificación A* sobre un mapa de ocupación (grid)
%
% Inputs:
%   occ          - matriz lógica (1=ocupado)
%   start, goal  - [row col]
%   connectivity - 4 u 8 (default 8)
%
% Output:
%   path - Nx2 [row col] desde start a goal. [] si no hay ruta.

    if nargin < 4
        connectivity = 8;
    end

    occ = logical(occ);
    nRows = size(occ, 1);
    nCols = size(occ, 2);

    if any(start < 1) || start(1) > nRows || start(2) > nCols
        error('Start fuera del mapa');
    end
    if any(goal < 1) || goal(1) > nRows || goal(2) > nCols
        error('Goal fuera del mapa');
    end
    if occ(start(1), start(2)) || occ(goal(1), goal(2))
        path = [];
        return;
    end

    if connectivity == 4
        moves = [ -1  0;
                   1  0;
                   0 -1;
                   0  1];
        moveCost = [1; 1; 1; 1];
    else
        moves = [ -1  0;
                   1  0;
                   0 -1;
                   0  1;
                  -1 -1;
                  -1  1;
                   1 -1;
                   1  1];
        moveCost = [1; 1; 1; 1; sqrt(2); sqrt(2); sqrt(2); sqrt(2)];
    end

    startIdx = sub2ind([nRows, nCols], start(1), start(2));
    goalIdx  = sub2ind([nRows, nCols], goal(1), goal(2));

    gScore = inf(nRows, nCols);
    fScore = inf(nRows, nCols);
    cameFrom = zeros(nRows, nCols, 'uint32');

    gScore(start(1), start(2)) = 0;
    fScore(start(1), start(2)) = heuristic_(start, goal);

    open = false(nRows, nCols);
    closed = false(nRows, nCols);
    open(start(1), start(2)) = true;

    while any(open(:))
        % Nodo con menor fScore en open-set
        fTmp = fScore;
        fTmp(~open) = inf;
        [~, currentIdx] = min(fTmp(:));
        if currentIdx == goalIdx
            path = reconstruct_path_(cameFrom, currentIdx, startIdx, [nRows, nCols]);
            return;
        end

        [cr, cc] = ind2sub([nRows, nCols], currentIdx);
        open(cr, cc) = false;
        closed(cr, cc) = true;

        for i = 1:size(moves,1)
            nr = cr + moves(i,1);
            nc = cc + moves(i,2);

            if nr < 1 || nr > nRows || nc < 1 || nc > nCols
                continue;
            end
            if occ(nr, nc) || closed(nr, nc)
                continue;
            end

            tentativeG = gScore(cr, cc) + moveCost(i);
            if ~open(nr, nc)
                open(nr, nc) = true;
            elseif tentativeG >= gScore(nr, nc)
                continue;
            end

            cameFrom(nr, nc) = uint32(currentIdx);
            gScore(nr, nc) = tentativeG;
            fScore(nr, nc) = tentativeG + heuristic_([nr nc], goal);
        end
    end

    path = [];
end

function h = heuristic_(a, b)
    % Euclídea en grid
    dr = double(a(1) - b(1));
    dc = double(a(2) - b(2));
    h = sqrt(dr*dr + dc*dc);
end

function path = reconstruct_path_(cameFrom, currentIdx, startIdx, sz)
    % Devuelve Nx2 [row col]
    maxLen = numel(cameFrom);
    idxs = zeros(maxLen, 1, 'uint32');
    k = 1;
    idxs(k) = uint32(currentIdx);

    while idxs(k) ~= uint32(startIdx)
        [r, c] = ind2sub(sz, double(idxs(k)));
        prev = cameFrom(r, c);
        if prev == 0
            path = [];
            return;
        end
        k = k + 1;
        idxs(k) = prev;
        if k >= maxLen
            break;
        end
    end

    idxs = idxs(1:k);
    idxs = flipud(idxs);
    [rr, cc] = ind2sub(sz, double(idxs));
    path = [rr(:), cc(:)];
end
