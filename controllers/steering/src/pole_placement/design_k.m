function [K, delta, S, w_n] = design_k(Ts, overshoot_max, A, B)
delta = .4;
S = 100 * exp ((-pi*delta)/sqrt(1-delta^2));

% Check the respect of the overshoot constraint
if S >= overshoot_max
    error('Error! The controller design does not satisfy overshoot constraint.');
end

w_n = 3/(delta*Ts);                 % natural frequency

% Pole computation
p1 = -delta*w_n + 1i*w_n*sqrt(1 - delta^2); % Dominant eigenvalue +
p2 = -delta*w_n - 1i*w_n*sqrt(1 - delta^2); % Dominant eigenvalue -
p = [p1; p2; -50; -100];                    % eigenvalues vector

% Design K placing the eigenvalues in the space
K = place(A,B,p);
end