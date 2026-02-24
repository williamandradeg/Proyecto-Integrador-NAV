function ok = is_path_valid_grid(occ, path)
% IS_PATH_VALID_GRID - Comprueba si una ruta sigue siendo válida
%
% Una ruta se considera inválida si algún punto cae en celda ocupada.

    if isempty(path)
        ok = false;
        return;
    end

    occ = logical(occ);
    rows = path(:,1);
    cols = path(:,2);

    inBounds = rows >= 1 & rows <= size(occ,1) & cols >= 1 & cols <= size(occ,2);
    if ~all(inBounds)
        ok = false;
        return;
    end

    idx = sub2ind(size(occ), rows, cols);
    ok = ~any(occ(idx));
end
