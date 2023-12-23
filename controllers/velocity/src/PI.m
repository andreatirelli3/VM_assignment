function PI = PI(t, v_x, v_des, k_p, k_i)
    % Symbolic variables
    syms x_des(t)
    x_des(t) = int(v_x - v_des, 0, t);

    PI = -k_p*(v_x - v_des) - k_i*x_des(t);
end