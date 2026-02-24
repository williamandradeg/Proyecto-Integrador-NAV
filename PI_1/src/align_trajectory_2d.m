function [traj_aligned, T] = align_trajectory_2d(traj, ref, t0)
% ALIGN_TRAJECTORY_2D - Alinea una trayectoria 2D a una referencia
%
% Calcula una transformación rígida 2D (rotación + traslación) para llevar
% la trayectoria `traj` al frame/origen de `ref` usando la pose en el
% instante `t0`.
%
% Esto es útil cuando `/odom`, `/amcl_pose` y `/base_pose_ground_truth`
% están expresados en frames distintos (offset/rotación constante).
%
% Inputs:
%   traj - struct con campos .t, .x, .y y opcionalmente .yaw
%   ref  - struct con campos .t, .x, .y y opcionalmente .yaw
%   t0   - instante (en segundos, en la escala de `traj.t` y `ref.t`)
%
% Outputs:
%   traj_aligned - misma estructura que `traj`, con x/y (y yaw si existe)
%                  transformados al frame de `ref`
%   T            - struct con campos:
%                  .R (2x2), .t (2x1), .dyaw (rad), .t0

    if nargin < 3
        t0 = 0;
    end

    % ROS bags a veces contienen timestamps repetidos o no estrictamente
    % crecientes. interp1 requiere puntos de muestreo únicos.
    traj = sanitizeTrajectory_(traj);
    ref  = sanitizeTrajectory_(ref);

    % La salida debe mantener coherencia de longitudes (t/x/y/yaw)
    traj_aligned = traj;

    if numel(traj.t) < 2 || numel(ref.t) < 2
        error('align_trajectory_2d:NotEnoughSamples', ...
              'Se necesitan al menos 2 muestras válidas en traj y ref.');
    end

    % Interpolar pose inicial (en t0)
    src_x0 = interp1(traj.t, traj.x, t0, 'linear', 'extrap');
    src_y0 = interp1(traj.t, traj.y, t0, 'linear', 'extrap');
    ref_x0 = interp1(ref.t, ref.x, t0, 'linear', 'extrap');
    ref_y0 = interp1(ref.t, ref.y, t0, 'linear', 'extrap');

    hasYaw = isfield(traj, 'yaw') && ~isempty(traj.yaw) && ...
             isfield(ref, 'yaw') && ~isempty(ref.yaw);

    if hasYaw
        src_yaw0 = interp1(traj.t, traj.yaw, t0, 'linear', 'extrap');
        ref_yaw0 = interp1(ref.t, ref.yaw, t0, 'linear', 'extrap');
        dyaw = wrapToPi(ref_yaw0 - src_yaw0);
    else
        dyaw = 0;
    end

    R = [cos(dyaw), -sin(dyaw); sin(dyaw), cos(dyaw)];
    t = [ref_x0; ref_y0] - R * [src_x0; src_y0];

    % Aplicar a toda la trayectoria
    P = R * [traj.x(:)'; traj.y(:)'];
    traj_aligned.x = (P(1, :)' + t(1));
    traj_aligned.y = (P(2, :)' + t(2));

    if isfield(traj, 'yaw') && ~isempty(traj.yaw)
        traj_aligned.yaw = wrapToPi(traj.yaw(:) + dyaw);
    end

    T = struct();
    T.R = R;
    T.t = t;
    T.dyaw = dyaw;
    T.t0 = t0;
end

function s = sanitizeTrajectory_(s)
    % Quitar NaNs/Infs y garantizar tiempos únicos (stable)
    t = s.t(:);
    x = s.x(:);
    y = s.y(:);

    valid = isfinite(t) & isfinite(x) & isfinite(y);
    if isfield(s, 'yaw') && ~isempty(s.yaw)
        yaw = s.yaw(:);
        valid = valid & isfinite(yaw);
    else
        yaw = [];
    end

    t = t(valid);
    x = x(valid);
    y = y(valid);
    if ~isempty(yaw)
        yaw = yaw(valid);
    end

    % Si hay duplicados de tiempo, quedarse con la primera ocurrencia
    [t_unique, ia] = unique(t, 'stable');
    x = x(ia);
    y = y(ia);
    if ~isempty(yaw)
        yaw = yaw(ia);
    end

    s.t = t_unique;
    s.x = x;
    s.y = y;
    if ~isempty(yaw)
        s.yaw = yaw;
    end
end
