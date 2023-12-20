function [psi_des_dot, psi_des, x_des, y_des] = road_aligned_integrator(v_x, R, t_i)
    % Integral function
    psi_des_dot = v_x / R;

    % Symbolic variables
    syms psi_des_sym(t) x_des_sym(t) y_des_sym(t)

    % Integrate psi_des_dot to get psi_des
    psi_des_sym(t) = int(psi_des_dot, 0, t);

    % Integrate v*cos(psi_des) to get x_des
    x_des_sym(t) = int(v_x * cos(psi_des_sym(t)), 0, t);

    % Integrate v*sin(psi_des) to get y_des
    y_des_sym(t) = int(v_x * sin(psi_des_sym(t)), 0, t);

    % Evaluate the symbolic functions at the given time points
    psi_des = double(psi_des_sym(t_i));
    x_des = double(x_des_sym(t_i));
    y_des = double(y_des_sym(t_i));
end
