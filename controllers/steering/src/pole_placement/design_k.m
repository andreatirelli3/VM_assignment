function [K, delta, S, w_n] = design_k(Ts, overshoot_max, A, B)
delta = .4;
S = 100 * exp ((-pi*delta)/sqrt(1-delta^2));

% Check the respect of the overshoot constraint
if S >= overshoot_max
    error('Error! The controller design does not satisfy overshoot constraint.');
end

w_n = 3/(delta*Ts);                 % natural frequency
dom_eig = delta*1i*w_n;             % dominant eigenvalue
p = [dom_eig; -dom_eig; -40; -100]; % eigenvalues vector

% Design K placing the eigenvalues in the space
K = place(A,B,p);
end