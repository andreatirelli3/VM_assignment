% Bicycle Dynamical model ODE function
function xdot = xdot(x, u, A, B)
    xdot = A*x + B*u;
end

% Bicycle Dynamical model ODE function /w road allignment
function xdot_ra = xdot_ra(x, u, A, B, d, psi_des_dot)
    xdot_ra = A*x+B*u+d*psi_des_dot;
end