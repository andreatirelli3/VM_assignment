%TODO: Struttura della funzione
%   - range v: v_interval
%   - Step di 5 km/h
%   - Controllo di Lyapunov:
%       * Se passa, mantieni K
%       * Se non passa riformula K
%           + usa flag per chiamare design_k o lq_regolator
%   - Metti nella struttura K

function [K, delta, S, w_n] = power_train(v_interval, flag)
% flag = true => Pole Placement, call design_k
%
% flag = false => LQ Regolator, call lq_regolator

d_vx = 10;

% Vehicle geomtry
[mass, i_z, l_f, l_r, C_f, C_r] = vehicle_geometry();

v_min = v_interval(1) * 1000 / 3600;
v_max = v_interval(2) * 1000 / 3600;

% Min v_x in the velocity interval
% Road Allignment system matrices A, B, B_d
[A_min, B_min, B_dmin] = road_aligned_matrices(mass, i_z, l_f, l_r, C_f, C_r, v_min);

% Max v_x in the velocity interval
% Road Allignment system matrices A, B, B_d
[A_max, B_max, B_dmax] = road_aligned_matrices(mass, i_z, l_f, l_r, C_f, C_r, v_max);

% Steering constraints
[Ts, overshoot_max] = steering_constraints();

if flag 
    [K_min, delta_min, S_min, w_nmin] = design_k(Ts, overshoot_max, A_min, B_min);
    [K_max, delta_max, S_max, w_nmax] = design_k(Ts, overshoot_max, A_max, B_max);
else
    [Q_min, K_min, S_min, CLP_min] = lq_regolator(A_min, B_min);
    [Q_max, K_max, S_max, CLP_max] = lq_regolator(A_max, B_max);
end

% Initialize cell arrays
K = {};
delta = {};
S = {};
w_n = {};

K{end + 1} = K_min;
delta{end + 1} = delta_min;
S{end + 1} = S_min;
w_n{end + 1} = w_nmin;

for v_x = v_interval(1) + 10:d_vx:v_interval(2) - 1
    % Normalize in m/s
    v_x = v_x * 1000 / 3600;

    % Road Allignment system matrices A, B, B_d
    [A, B, B_d] = road_aligned_matrices(mass, i_z, l_f, l_r, C_f, C_r, v_x);
    if flag
        [K_t, delta_t, S_t, w_nt] = design_k(Ts, overshoot_max, A, B);
    else
        [Q_t, K_t, S_t, CLP_t] = lq_regolator(A, B);
    end

    % Add the new controller K_t to K
    K{end+1} = K_t;
    delta{end+1} = delta_t;
    S{end+1} = S_t;
    w_n{end+1} = w_nt;

    v_x = v_x *3.6;
end


K{end + 1} = K_max;
delta{end + 1} = delta_max;
S{end + 1} = S_max;
w_n{end + 1} = w_nmax;

end