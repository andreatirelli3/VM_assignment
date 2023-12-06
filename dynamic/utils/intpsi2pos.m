% Integrate in 0 to i the psi_des_dot, psi_des, x_des and y_des to
% obtain the global frame position of the vehicle
function [psi_des_dot, psi_des, x_des, y_des] = intpsi2pos(i, v_x, R)
    syms i real;  % Define i as a real symbolic variable
    syms psi_des_dot(i) psi_des(i) x_des(i) y_des(i)

    % Calculate psi_des_dot
    psi_des_dot(i) = v_x / R;

    % Integrate psi_des_dot to get psi_des
    psi_des(i) = int(psi_des_dot(i), i, 0, i);

    % Integrate v*cos(psi_des) to get x_des
    x_des(i) = int(v_x * cos(psi_des(i)), i, 0, i);

    % Integrate v*sin(psi_des) to get y_des
    y_des(i) = int(v_x * sin(psi_des(i)), i, 0, i);

    % Convert the symbolic expressions to double precision
    psi_des_dot = double(psi_des_dot);
    psi_des = double(psi_des);
    x_des = double(x_des);
    y_des = double(y_des);
end

