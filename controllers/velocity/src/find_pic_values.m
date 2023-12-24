function [sys, k_p, k_i] = find_pic_values(tau, ratio)
    k_p = 0:.01:.75;

    sys = tf([1 ratio], [tau 1 0 0]);

    k_p = .7;
    k_i = k_p * ratio;
end